import os
from pynput import keyboard as pynput_keyboard
import numpy as np

# --- 設定 ---
# 確保這個路徑與你在終端執行 ./python_bridge 時的路徑一致
SDK_BUILD_DIR = "/home/unitree/D1_SDK/build/"
SDK_EXEC = "./python_bridge"

# 當前角度緩存
current_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

def send_to_robot():
    # 強制限制夾爪邊界 [-15, 50]
    current_angles[6] = np.clip(current_angles[6], -15.0, 50.0)
    
    # 格式化指令
    angles_str = " ".join([f"{a:.2f}" for a in current_angles])
    cmd = f'cd {SDK_BUILD_DIR} && {SDK_EXEC} {angles_str}'
    
    # 執行發送
    os.system(cmd)
    print(f"已發送: {angles_str}")

def on_press(key):
    global current_angles
    try:
        k = key.char if hasattr(key, 'char') else key.name
        step = 2.0
        
        if k == 'w': current_angles[0] += step
        elif k == 's': current_angles[0] -= step
        elif k == 'a': current_angles[1] += step
        elif k == 'd': current_angles[1] -= step
        elif k == 'q': current_angles[2] += step
        elif k == 'e': current_angles[2] -= step
        elif k == 'z': current_angles[6] = -15.0 # 開
        elif k == 'x': current_angles[6] = 50.0  # 關
        elif k == 'esc': return False
        else: return
        
        send_to_robot()
    except Exception as e:
        print(f"按鍵錯誤: {e}")

print("--- 穩定控制模式啟動 ---")
print("w/s:J0, a/d:J1, q/e:J2, z:開, x:關, ESC:退出")

with pynput_keyboard.Listener(on_press=on_press) as listener:
    listener.join()
