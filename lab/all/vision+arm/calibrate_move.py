import numpy as np
import subprocess
import time

BRIDGE = "/home/unitree/D1_SDK/build/python_bridge"

def send(joints_deg, gripper=0.0):
    a = " ".join(f"{a:.2f}" for a in np.concatenate([joints_deg, [gripper]]))
    print(f"发送角度: {joints_deg}")
    subprocess.run(f"{BRIDGE} {a}", shell=True)

# 交互式校准测试
def manual_test():
    print("--- 物理校准模式：输入要测试的关节编号 (1-6) 和角度 ---")
    while True:
        cmd = input("输入格式: 关节编号 角度 (例如: 2 30)，输入 q 退出: ")
        if cmd == 'q': break
        try:
            j_idx, angle = map(float, cmd.split())
            q = np.zeros(6)
            q[int(j_idx)-1] = angle
            send(q)
        except:
            print("输入格式错误")

if __name__ == "__main__":
    manual_test()
