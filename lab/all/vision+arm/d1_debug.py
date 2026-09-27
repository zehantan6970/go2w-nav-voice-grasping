#!/usr/bin/env python3
import numpy as np
import sys

# ============================================================
# 1. 强力定义：直接写在函数外部，确保全局唯一
# ============================================================
# 修改这里的数值 (1.0 或 -1.0) 即可直接改变计算结果
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

def joint_rot(i, theta):
    # 强制应用全局的 DIRECTION_MAP
    a = theta * float(DIRECTION_MAP[i])
    ax = JOINT_AXES_RAW[i]
    c, s = np.cos(a), np.sin(a)
    if abs(ax[1]) > 0.9: return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    if abs(ax[0]) > 0.9: return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) if ax[2] > 0 else np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])

def fk(joints_rad):
    R, p = np.eye(3), np.zeros(3)
    # 打印正在使用的修正系数
    print(f"DEBUG: 正在使用修正系数: {DIRECTION_MAP}")
    for i in range(6):
        p = p + R @ JOINT_ORIGINS[i]
        R = R @ joint_rot(i, joints_rad[i])
    p = p + R @ GRIPPER_OFFSET
    return p, R

if __name__ == "__main__":
    if len(sys.argv) < 7:
        print("用法: python3 d1_debug.py <J1> <J2> <J3> <J4> <J5> <J6> (度)")
    else:
        angles = [float(x) for x in sys.argv[1:7]]
        pos, _ = fk(np.deg2rad(angles))
        print(f"计算位置: X={pos[0]:.4f}, Y={pos[1]:.4f}, Z={pos[2]:.4f}")
