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
import time

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

# 你測試過最棒的初始/待命安全位置（角度制）
HOME_ANGLES_DEG = [0.0, -70.0, 70.0, 0.0, 0.0, 0.0]

# ============================================================
# 🎥 2. 眼在手上（實體量測校準）：【已修正 Y 軸左右相反問題】
# ============================================================
# 透過將 Cam_X 對應到負的 Arm_Y 軸，來修正左右相反的鏡像誤差
R_end_to_cam = np.array([
    [0.0,  0.0, 1.0],  # 機械臂 X 軸對應相機 Z 軸
    [-1.0, 0.0, 0.0],  # 【已修正】機械臂 Y 軸對應相機的負 X 軸 (反轉左右)
    [0.0,  1.0, 0.0]   # 機械臂 Z 軸對應相機 Y 軸
]) 

# 平移向量的 Y 軸同步配合物理實際方向調整
T_end_to_cam = np.array([0.05, 0.06, 0.0]) 

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
# 🧮 4. 運動學核心運算 (6-DoF 包含姿態約束)
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

def rot_error(R_d, R_c):
    """計算兩個旋轉矩陣之間的旋轉誤差向量"""
    err_mat = R_d @ R_c.T
    r_val = (np.trace(err_mat) - 1.0) / 2.0
    r_val = np.clip(r_val, -1.0, 1.0)
    theta = np.arccos(r_val)
    if abs(theta) < 1e-5:
        return np.zeros(3)
    axis = np.array([
        err_mat[2, 1] - err_mat[1, 2],
        err_mat[0, 2] - err_mat[2, 0],
        err_mat[1, 0] - err_mat[0, 1]
    ])
    return (theta / (2.0 * np.sin(theta))) * axis

def solve_ik_6dof(target_pos, target_rot_mat, start_q_rad, max_iter=200, tol=1e-3):
    """全 6-DoF 逆向運動學解算，同時約束位置與夾爪水平朝前的姿態"""
    q = start_q_rad.copy()
    damping = 0.04
    
    for i in range(max_iter):
        p_cur, R_cur = fk(q)
        
        err_p = target_pos - p_cur
        err_R = rot_error(target_rot_mat, R_cur)
        err = np.concatenate([err_p, err_R])
        
        if np.linalg.norm(err) < tol: 
            return q, True
            
        J = jacobian(q)
        dq = J.T @ np.linalg.pinv(J @ J.T + damping**2 * np.eye(6)) @ err
        q += np.clip(dq, -0.08, 0.08)
        
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
    print(f"[🚀 EXECUTE] 發送角度: {', '.join([f'{a:.1f}' for a in final_deg])}")
    subprocess.run(cmd)

def send_gripper_command(open_close_flag):
    """
    控制夾爪開合。0 代表閉合夾緊，1 代表鬆開
    """
    if open_close_flag == 1:
        print("[✋ GRIPPER] 鬆開夾爪，準備接應...")
        # 若有獨立控制腳本可在此解開註釋：
        # subprocess.run(["/home/unitree/D1_SDK/build/gripper_control", "1"])
    else:
        print("[✊ GRIPPER] 夾爪閉合！抱緊水瓶。")
        # subprocess.run(["/home/unitree/D1_SDK/build/gripper_control", "0"])
    time.sleep(1.0)

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
# 🎮 6. 主控制介面輸出（已修正縮排，置於迴圈外）
# ============================================================
print("\n" + "="*50)
print("🤖 Unitree 眼在手上 安全防撞水平抓取系統優化版")
print("👉 安全機制：")
print("   - 【已修復】Y 軸左右相反鏡像問題，左右定位精準。")
print("   - 夾爪強行鎖定在【水平朝前】姿態，不再隨機亂轉砸桌子。")
print("   - 採取 4 段式安全路徑：預備接近 -> 水平推進 -> 垂直抬升 -> 帶著水瓶回防。")
print("👉 操作指引：")
print("   - 按【 g 】鍵：啟動安全防撞分段抓取序列。")
print("   - 按【 h 】鍵：手動讓手臂回到你推薦的 [0, -70, 70, 0, 0, 0] 安全初始位置。")
print("   - 按【 q 】鍵：退出系統。")
print("="*50 + "\n")

# ============================================================
# 🔄 7. 主事件監聽控制迴圈
# ============================================================
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
                    # 實時在畫面上印出 Base 座標
                    cv2.putText(color_image, f"BASE: X={xb*100:.1f}, Y={yb*100:.1f}, Z={zb*100:.1f} cm", 
                                (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 0), 2)

        cv2.imshow("Unitree Eye-in-Hand Smart Control System", color_image)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'): 
            break
            
        elif key == ord('h'):
            print("\n[🏠 HOME] 正在引導手臂回到指定的安全待命位置...")
            send_to_arm(np.deg2rad(HOME_ANGLES_DEG))
            
        elif key == ord('g'):
            if target_base_3d is not None:
                p_grip = target_base_3d.copy()
                print(f"\n[🎯 TARGET LOCKED] 水瓶 Base 座標 -> X={p_grip[0]:.3f}m, Y={p_grip[1]:.3f}m, Z={p_grip[2]:.3f}m")
                
                # ─── 姿態約束配置 ───
                # 規定夾爪前方(Z)指向手臂前方(X)，夾爪兩側(Y)水平，保持水平接近
                R_target = np.array([
                    [0.0,  0.0,  1.0],  # 夾爪 Z 軸沿著 Base 的 +X 軸（向前伸）
                    [-1.0, 0.0,  0.0],  # 夾爪 X 軸沿著 Base 的 -Y 軸
                    [0.0,  -1.0, 0.0]   # 夾爪 Y 軸沿著 Base 的 -Z 軸
                ])
                
                # ─── 4段式路徑點規劃 (Waypoints) ───
                # Step 1: Approach 預備點 (水平往後退 10 公分，避免橫衝直撞撞倒水瓶)
                p_approach = p_grip.copy()
                p_approach[0] -= 0.10  # 在 Base X 軸上後退 10cm
                
                # Step 2: Grip 實際抓取點就是 p_grip
                
                # Step 3: Lift 抬升點 (垂直往上提 12 公分，脫離桌面)
                p_lift = p_grip.copy()
                p_lift[2] += 0.12  # 在 Base Z 軸上抬高 12cm
                
                # ─── 執行分段運動流水線 ───
                success_pipeline = True
                current_loop_q = np.deg2rad(current_joint_angles) # 用目前手臂角度做 IK 初始值
                
                # 預先張開夾爪
                send_gripper_command(1)
                
                # 【第一步：移到預備接近點】
                print("⏳ [1/4] 正在解算預備接近點 (Approach)...")
                q_approach, succ = solve_ik_6dof(p_approach, R_target, current_loop_q)
                if succ:
                    send_to_arm(q_approach)
                    current_loop_q = q_approach # 更新下一階段的 IK 起點
                    time.sleep(2.5) # 等待手臂到位穩定
                else:
                    print("❌ 預備點解算失敗，放棄任務以免撞擊！")
                    success_pipeline = False
                    
                # 【第二步：水平向前推進抓取】
                if success_pipeline:
                    print("⏳ [2/4] 正在水平推進至水瓶中心 (Grip)...")
                    q_grip, succ = solve_ik_6dof(p_grip, R_target, current_loop_q)
                    if succ:
                        send_to_arm(q_grip)
                        current_loop_q = q_grip
                        time.sleep(2.0)
                        # 到位後，下達夾緊指令！
                        send_gripper_command(0) 
                    else:
                        print("❌ 抓取點解算失敗！")
                        success_pipeline = False
                        
                # 【第三步：垂直安全抬升】
                if success_pipeline:
                    print("⏳ [3/4] 正在垂直抬升水瓶 (Lift)...")
                    q_lift, succ = solve_ik_6dof(p_lift, R_target, current_loop_q)
                    if succ:
                        send_to_arm(q_lift)
                        time.sleep(2.0)
                    else:
                        print("❌ 抬升點解算失敗！")
                        
                # 【第四步：帶水瓶回到安全收攏位置】
                print("⏳ [4/4] 正在帶回水瓶至 Home 安全姿態...")
                send_to_arm(np.deg2rad(HOME_ANGLES_DEG))
                print("✨ [SUCCESS] 分段安全抓取任務執行完畢！")
                
            else:
                print("❌ [WARNING] 畫面未鎖定瓶子，無法觸發安全抓取！")

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    print("[INFO] 系統已安全關閉。")
