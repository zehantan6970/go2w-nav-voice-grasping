#!/usr/bin/env python3
"""
機械臂歸位工具（附位置驗證+重試）
由 SLAM demo 的巡航任務調用，將機械臂定位到 HOME 位置。
對應 3dcatchv5.py 的 step_joint() 邏輯。
"""
import subprocess, json, time, os, sys

J1_OFFSET = 90.0
HOME_DEG = [0.0, -75.0, 75.0, 0.0, 0.0, 0.0]
GRIP_OPEN = 55.0
SETTLE = 6.0
MAX_RETRY = 3
ARM_BRIDGE = "/home/unitree/D1sdk/build/python_bridge"
ANGLE_READER = "/home/unitree/D1sdk/build/get_arm_joint_angle"
DDS_LD = "LD_LIBRARY_PATH=/home/unitree/cyclonedds/install/lib:/usr/local/lib"

def send(d, g=0):
    """發送角度+夾爪，自動加 J1_OFFSET"""
    arr = list(d) + [g]
    arr[0] += J1_OFFSET
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = "/home/unitree/cyclonedds/install/lib:/usr/local/lib"
    cmd = [ARM_BRIDGE] + [f"{v:.2f}" for v in arr]
    subprocess.run(cmd, env=env, capture_output=True)

def read_angles():
    """讀取機械臂當前角度（從 armFeedback topic）"""
    try:
        r = subprocess.run(["timeout", "2", ANGLE_READER],
                          capture_output=True, text=True, timeout=3)
        for line in r.stdout.split('\n'):
            if 'armFeedback_data:' in line and '"funcode":1' in line:
                d = json.loads(line.split('armFeedback_data:')[1].strip()).get('data',{})
                if 'angle0' in d:
                    return [d[f'angle{i}'] for i in range(7)]
    except:
        pass
    return [0.0] * 7

def main():
    target = HOME_DEG[:6]  # 6 joint angles (gripper handled separately)
    target_deg = target  # reference for comparison
    
    for attempt in range(MAX_RETRY + 1):
        send(HOME_DEG, GRIP_OPEN)
        time.sleep(SETTLE)
        
        act = read_angles()
        # read_angles 返回物理值，減去 J1_OFFSET 再比較
        act_deg = list(act[:6])
        act_deg[0] -= J1_OFFSET
        
        err = max(abs(act_deg[i] - target_deg[i]) for i in range(6))
        
        if err < 5.0:
            print(f"[Arm] HOME 到位 err={err:.1f}°")
            return 0
        elif attempt < MAX_RETRY:
            print(f"[Arm] ⚠️ err={err:.1f}°，第{attempt+1}次重試...")
            # 清理殘留進程
            subprocess.run(["pkill", "-f", "get_arm_joint_angle"], capture_output=True)
        else:
            print(f"[Arm] ❌ 重試{MAX_RETRY}次皆失敗 err={err:.1f}°")
            return 1
    
    return 1

if __name__ == "__main__":
    sys.exit(main())
