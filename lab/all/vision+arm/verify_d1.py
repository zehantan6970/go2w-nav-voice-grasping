#!/usr/bin/env python3
import numpy as np
import subprocess

# ============================================================
# 参数配置区 (此处数据需与你的 D1-550 说明书保持一致)
# ============================================================
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

# 若某关节方向与真机相反，在此处将方向因子设为 -1
# 索引从 0 开始 (对应 J1-J6)
DIRECTION_MAP = {0: 1.0, 1: -1.0, 2: 1.0, 3: -1.0, 4: -1.0, 5: -1.0}

# ============================================================
# 核心运动学函数
# ============================================================
def rot_x(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

def rot_y(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

def rot_z(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

def joint_rot(i, theta):
    # 应用方向修正因子
    a = DIRECTION_MAP.get(i, 1.0) * theta
    ax = JOINT_AXES_RAW[i]
    if abs(ax[1]) > 0.9: return rot_y(a)
    if abs(ax[0]) > 0.9: return rot_x(a)
    return rot_z(a) if ax[2] > 0 else rot_z(-a)

def fk(joints_rad):
    R, p = np.eye(3), np.zeros(3)
    for i in range(6):
        p = p + R @ JOINT_ORIGINS[i]
        R = R @ joint_rot(i, joints_rad[i])
    p = p + R @ GRIPPER_OFFSET
    return p, R

# ============================================================
# 交互式工具
# ============================================================
if __name__ == "__main__":
    print("--- D1-550 运动学校准工具 ---")
    mode = input("选择模式: [1] FK验证 [2] 物理测试: ")
    
    if mode == '1':
        angles = [float(x) for x in input("输入6个关节角度(空格分隔): ").split()]
        pos, _ = fk(np.deg2rad(angles))
        print(f"计算结果: X={pos[0]:.4f}, Y={pos[1]:.4f}, Z={pos[2]:.4f}")
        print("对比技巧: 移动后测量末端高度差，若计算增高而实物降低，修改 DIRECTION_MAP 对应值")
        
    elif mode == '2':
        bridge = "/home/unitree/D1_SDK/build/python_bridge"
        print("输入 'j 角度' 测试(j:1-6)，输入 q 退出")
        while True:
            cmd = input("> ")
            if cmd == 'q': break
            try:
                parts = cmd.split()
                j_idx, val = int(parts[0])-1, float(parts[1])
                q = np.zeros(6)
                q[j_idx] = val
                a_str = " ".join(f"{x:.2f}" for x in np.concatenate([q, [0]]))
                subprocess.run(f"{bridge} {a_str}", shell=True)
            except: print("格式错误")
