#!/usr/bin/env python3
import numpy as np
import subprocess

# 将此处的参数保持和你目前代码中的一致
JOINT_ORIGINS = [
    np.array([0.0, 0.0, 0.0738]), np.array([0.0, -0.0276, 0.0578]),
    np.array([0.0, -0.0004, 0.27]), np.array([0.05, 0.0275, 0.041325]),
    np.array([0.15468, -0.0258, 0.0001]), np.array([0.0777, 0.025822, -0.0010718]),
]
GRIPPER_OFFSET = np.array([0.0718, 0.0, 0.0031])

def get_manual_test_fk(angles_deg):
    """只计算 FK，不涉及 IK 逻辑"""
    q_rad = np.deg2rad(angles_deg)
    R = np.eye(3)
    p = np.zeros(3)
    for i in range(6):
        p = p + R @ JOINT_ORIGINS[i]
        # 简化版旋转，暂时不带方向修正，只看物理连杆
        R = R @ np.eye(3) 
    p = p + R @ GRIPPER_OFFSET
    return p

# 诊断程序
print("=== 物理对齐诊断 ===")
# 1. 发送零位
print("正在发送零位指令...")
subprocess.run(["/home/unitree/D1_SDK/build/python_bridge", "0", "0", "0", "0", "0", "0", "0"])

# 2. 测量反馈
pos = get_manual_test_fk([0,0,0,0,0,0])
print(f"理论零位位置 (代码计算): X={pos[0]:.4f}, Y={pos[1]:.4f}, Z={pos[2]:.4f}")
print("请务必使用钢尺测量：末端尖端到基座中心的实际距离 X, Y, Z 是多少？")
