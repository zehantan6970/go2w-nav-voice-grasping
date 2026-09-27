#!/usr/bin/env python3
import cv2
import numpy as np
import pyrealsense2 as rs
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
import threading
import subprocess
import json
import sys

# 修正 TensorRT 與新版 Numpy 的相容性問題
np.bool = bool 

# ============================================================
# 📐 1. 機械臂運動學參數與校準配置
# ============================================================
DIRECTION_MAP = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])

JOINT_AXES_RAW = [
    np.array([0, 0, -1]), np.array([0, 1, 0]), np.array([0, 1, 0]),
    np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([1, 0, 0]),
]
JOINT_ORIGINS = [
    np.array([0.0, 0.0, 0.0738]), np.array([0.0, -0.0276, 0.0578]),
    np.array([0.0, -0.0004, 0.27]), np.array([0.05, 0.0275, 0.041325]),
    np.array([0.15468, -0.0258, 0.0001]), np.array([0.0777, 0.025822, -0.0010718]),
]
GRIPPER_OFFSET = np.array([0.0718, 0.0, 0.0031])

# ============================================================
# 🎥 2. 眼在手上（實體量測校準）：相機相對於末端法蘭面
# ============================================================
# 【已修正】依據對應關係建立旋轉矩陣：Arm_X=Cam_Z, Arm_Y=Cam_X, Arm_Z=Cam_Y
R_end_to_cam = np.array([
    [0.0, 0.0, 1.0],  # 機械臂 X 軸對應相機 Z 軸
    [1.0, 0.0, 0.0],  # 機械臂 Y 軸對應相機 X 軸
    [0.0, 1.0, 0.0]   # 機械臂 Z 軸對應相機 Y 軸
]) 

# 【已修正】平移向量也必須依據新的機械臂軸向順序 (X, Y, Z) 重新排列
# 相機在末端：Arm_X 軸方向偏 +5cm, Arm_Y 軸方向偏 -6cm, Arm_Z 軸方向偏 0cm
T_end_to_cam = np.array([0.05, -0.06, 0.0]) 

# ============================================================
# 📡 3. 背景執行緒：實時讀取並解析 Unitree 關節角度
# ============================================================
current_joint_angles = [0.0] * 6  # 全域變數（角度制）

def unitree_angle_reader():
    global current_joint_angles
    cmd = "/home/unitree/D1_SDK/build/get_arm_joint_angle"
    try:
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        print(f"[INFO] 成功啟動 Unitree 角度監聽執行檔...")
        
        for line in iter(process.stdout.readline, ''):
            if "armFeedback_data:" in line and '"funcode":1' in line:
                try:
                    json_str = line.split("armFeedback_data:")[1].strip()
                    payload = json.loads(json_str)
                    angles = payload.get("data", {})
                    if "angle0" in angles:
                        current_joint_angles = [
                            angles["angle0"], angles["angle1"], angles["angle2"],
                            angles["angle3"], angles["angle4"], angles["angle5"]
                        ]
                except Exception:
                    pass
    except Exception as e:
        print(f"[ERROR] 無法執行讀取角度程式: {e}")

reader_thread = threading.Thread(target=unitree_angle_reader, daemon=True)
reader_thread.start()

# ============================================================
# 🧮 4. 運動學核心運算
# ============================================================
def joint_rot(i, theta):
    a = theta * float(DIRECTION_MAP[i])
    ax = JOINT_AXES_RAW[i]
    c, s = np.cos(a), np.sin(a)
    if abs(ax[1]) > 0.9: return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    if abs(ax[0]) > 0.9: return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if ax[2] > 0: return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])

def fk(joints_rad):
    R, p = np.eye(3), np.zeros(3)
    for i in range(6):
        p = p + R @ JOINT_ORIGINS[i]
        R = R @ joint_rot(i, joints_rad[i])
    p = p + R @ GRIPPER_OFFSET
    return p, R

def fk_flange_only(joints_rad):
    R, p = np.eye(3), np.zeros(3)
    for i in range(6):
        p = p + R @ JOINT_ORIGINS[i]
        R = R @ joint_rot(i, joints_rad[i])
    return p, R

def jacobian(joints_rad):
    R, p = np.eye(3), np.zeros(3)
    jpos = [np.zeros(3)]
    for i in range(6):
        p = p + R @ JOINT_ORIGINS[i]
        jpos.append(p.copy())
        R = R @ joint_rot(i, joints_rad[i])
    p_ee = p + R @ GRIPPER_OFFSET
    J = np.zeros((6, 6))
    R_j = np.eye(3)
    for i in range(6):
        ax = JOINT_AXES_RAW[i] * DIRECTION_MAP[i]
        z_i = R_j @ ax
        J[:3, i] = np.cross(z_i, p_ee - jpos[i])
        J[3:, i] = z_i
        R_j = R_j @ joint_rot(i, joints_rad[i])
    return J

def solve_ik(target_pos, max_iter=200, tol=1e-4):
    global current_joint_angles
    q = np.deg2rad(current_joint_angles)
    
    for i in range(max_iter):
        p_cur, _ = fk(q)
        err = target_pos - p_cur
        if np.linalg.norm(err) < tol: return q, True
        J = jacobian(q)
        damping = 0.05
        dq = J[:3].T @ np.linalg.pinv(J[:3] @ J[:3].T + damping**2 * np.eye(3)) @ err
        q += np.clip(dq, -0.1, 0.1)
    return q, False

def transform_eye_in_hand(x_c, y_c, z_c):
    global current_joint_angles
    joints_rad = np.deg2rad(current_joint_angles)
    
    # 1. 計算當前手臂法蘭面的空間位置
    p_flange, R_flange = fk_flange_only(joints_rad)
    
    # 2. 將相機座標點轉到法蘭面系下（套用修正後的旋轉與平移）
    p_cam = np.array([x_c, y_c, z_c])
    p_end = R_end_to_cam @ p_cam + T_end_to_cam
    
    # 3. 將法蘭面坐標轉到手臂 Base 基座
    p_base = R_flange @ p_end + p_flange
    return p_base

def send_to_arm(q_rad):
    final_deg = np.rad2deg(q_rad)
    cmd = ["/home/unitree/D1_SDK/build/python_bridge"] + [f"{a:.2f}" for a in np.concatenate([final_deg, [0.0]])]
    print(f"\n[🚀 SEND COMMAND] 執行控制指令: {' '.join(cmd)}")
    subprocess.run(cmd)

# ============================================================
# 🧠 5. TensorRT & RealSense 初始化環境
# ============================================================
ENGINE_PATH = "bottle.engine"
CONF_THRESHOLD = 0.5
IOU_THRESHOLD = 0.4
INPUT_WIDTH, INPUT_HEIGHT = 640, 640

logger = trt.Logger(trt.Logger.WARNING)
with open(ENGINE_PATH, "rb") as f, trt.Runtime(logger) as runtime:
    engine = runtime.deserialize_cuda_engine(f.read())
context = engine.create_execution_context()

inputs, outputs, bindings, stream = [], [], [], cuda.Stream()
for binding in engine:
    size = trt.volume(engine.get_binding_shape(binding))
    dtype = trt.nptype(engine.get_binding_dtype(binding))
    host_mem = cuda.pagelocked_empty(size, dtype)
    device_mem = cuda.mem_alloc(host_mem.nbytes)
    bindings.append(int(device_mem))
    if engine.binding_is_input(binding): inputs.append({'host': host_mem, 'device': device_mem})
    else: outputs.append({'host': host_mem, 'device': device_mem})

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
profile = pipeline.start(config)
align = rs.align(rs.stream.color)
intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

def preprocess_image(img):
    image_resized = cv2.resize(img, (INPUT_WIDTH, INPUT_HEIGHT))
    img_norm = cv2.cvtColor(image_resized, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    return np.expand_dims(np.transpose(img_norm, (2, 0, 1)), axis=0)

# ============================================================
# 🎮 6. 主事件監聽控制迴圈
# ============================================================
print("\n" + "="*50)
print("🤖 Unitree 眼在手上(Eye-in-Hand) 視覺整合控制系統啟動成功！")
print("👉 畫面上會實時顯示瓶子在機械臂基座下(Base系)的真實空間座標(cm)")
print("👉 操作指引：")
print("   - 按【 g 】鍵：捕捉當前偵測物座標，送入 IK 求解並控制手臂移去抓取。")
print("   - 按【 q 】鍵：安全退出程式。")
print("="*50 + "\n")

try:
    while True:
        frames = align.process(pipeline.wait_for_frames())
        color_frame, depth_frame = frames.get_color_frame(), frames.get_depth_frame()
        if not color_frame or not depth_frame: continue
            
        color_image = np.asanyarray(color_frame.get_data())
        
        # 進行 TensorRT 加速推論
        inp = preprocess_image(color_image)
        np.copyto(inputs[0]['host'], inp.ravel())
        cuda.memcpy_htod_async(inputs[0]['device'], inputs[0]['host'], stream)
        context.execute_async_v2(bindings=bindings, stream_handle=stream.handle)
        cuda.memcpy_dtoh_async(outputs[0]['host'], outputs[0]['device'], stream)
        stream.synchronize()
        
        # 解析檢測結果
        out = outputs[0]['host']
        preds = np.reshape(out, (len(out) // 8400, 8400)).T
        boxes, confs, target_base_3d = [], [], None
        
        x_f, y_f = color_image.shape[1]/INPUT_WIDTH, color_image.shape[0]/INPUT_HEIGHT
        for row in preds:
            scores = row[4:]
            class_id = np.argmax(scores)
            if scores[class_id] > CONF_THRESHOLD:
                cx, cy, w, h = row[0], row[1], row[2], row[3]
                boxes.append([int((cx-w/2)*x_f), int((cy-h/2)*y_f), int(w*x_f), int(h*y_f)])
                confs.append(float(scores[class_id]))
                
        indices = cv2.dnn.NMSBoxes(boxes, confs, CONF_THRESHOLD, IOU_THRESHOLD)
        if len(indices) > 0:
            for i in np.array(indices).flatten():
                x, y, w, h = boxes[i]
                cv2.rectangle(color_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                
                cx_pix, cy_pix = max(0, min(int(x+w/2), 639)), max(0, min(int(y+h/2), 479))
                dist = depth_frame.get_distance(cx_pix, cy_pix)
                
                if dist > 0:
                    pt_cam = rs.rs2_deproject_pixel_to_point(intrinsics, [cx_pix, cy_pix], dist)
                    # 內部已套用修正後的對調矩陣
                    target_base_3d = transform_eye_in_hand(pt_cam[0], pt_cam[1], pt_cam[2])
                    
                    xb, yb, zb = target_base_3d
                    cv2.putText(color_image, f"BASE: X={xb*100:.1f}, Y={yb*100:.1f}, Z={zb*100:.1f} cm", 
                                (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 0), 2)

        cv2.imshow("Unitree Eye-in-Hand Smart Control System", color_image)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'): 
            break
        elif key == ord('g'):
            if target_base_3d is not None:
                print(f"\n[🎯 TRIGGER] 鎖定目標: Base 坐標 -> X={target_base_3d[0]:.3f}m, Y={target_base_3d[1]:.3f}m, Z={target_base_3d[2]:.3f}m")
                print("⏳ 正在執行阻尼最小二乘法 IK 逆向解算...")
                
                q_res, success = solve_ik(target_base_3d)
                if success:
                    print(f"✅ IK 解算成功！即將發送目標關節角度。")
                    send_to_arm(q_res)
                else:
                    print("❌ [WARNING] IK 逆解失敗！目標位置可能超出工作半徑(不可達)。")
            else:
                print("❌ [WARNING] 畫面上未偵測到瓶子，無法觸發抓取！")

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    print("[INFO] 系統已安全關閉。")
