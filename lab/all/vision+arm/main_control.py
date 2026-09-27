import numpy as np
import pinocchio as pin
import os
from pynput import keyboard as pynput_keyboard

# --- 設定 ---
URDF_PATH = "/home/unitree/vision+arm/d1_550_description/urdf/d1_550_description.urdf"
SDK_BUILD_DIR = "/home/unitree/D1_SDK/build/"
SDK_EXEC = "./python_bridge"

model = pin.buildModelFromUrdf(URDF_PATH)
data = model.createData()
frame_id = model.getFrameId("Joint6")
target_pos = np.array([0.4, 0.0, 0.3])

# 初始化姿勢：確保維度與 model.nq 一致
current_q = pin.neutral(model)

# --- 運動學：適配 model.nv 與 model.nq ---
def solve_ik(target_pos, q_start):
    q = q_start.copy()
    oMdes = pin.SE3(pin.Quaternion(np.array([0., 0., 0., 1.])), target_pos)
    
    for i in range(50):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        err = pin.log(data.oMf[frame_id].actInv(oMdes)).vector
        if np.linalg.norm(err) < 1e-4: break
        
        # 使用 pin.computeFrameJacobian 並限制在 6 維誤差
        J = pin.computeFrameJacobian(model, data, q, frame_id, pin.LOCAL)[:6, :]
        v_full = np.zeros(model.nv)
        v_6 = - J.T.dot(np.linalg.solve(J.dot(J.T) + 0.5 * np.eye(6), err))
        
        # 將計算出的速度填充到 model.nv 的維度中
        v_full[:len(v_6)] = np.clip(v_6, -np.radians(0.5), np.radians(0.5))
        q = pin.integrate(model, q, v_full)
        
    return q

def send_to_robot(q_new):
    global current_q
    # 確保傳輸給 python_bridge 的數據永遠是 7 個關節角度
    current_q = q_new.copy()
    angles = np.degrees(current_q[:7])
    
    # 執行 C++ 程式
    cmd = f'{SDK_EXEC} {" ".join([f"{a:.2f}" for a in angles])}'
    os.system(f'cd {SDK_BUILD_DIR} && {cmd}')

if __name__ == "__main__":
    print(f"模型已載入: nq={model.nq}, nv={model.nv}")
    print("啟動成功。按 [w,s] 前後, [a,d] 左右, [q,e] 上下。")
    
    def on_press(key):
        global target_pos, current_q
        try:
            k = key.char if hasattr(key, 'char') else key.name
            step = 0.005
            if k == 'w': target_pos[0] += step
            elif k == 's': target_pos[0] -= step
            elif k == 'a': target_pos[1] += step
            elif k == 'd': target_pos[1] -= step
            elif k == 'e': target_pos[2] += step
            elif k == 'q': target_pos[2] -= step
            else: return
            
            current_q = solve_ik(target_pos, current_q)
            send_to_robot(current_q)
        except Exception as e:
            print(f"計算錯誤: {e}")

    with pynput_keyboard.Listener(on_press=on_press) as listener:
        listener.join()
