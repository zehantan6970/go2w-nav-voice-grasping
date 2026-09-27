#!/usr/bin/env python3
import numpy as np
import subprocess
import sys

# ============================================================
# 运动学参数与校准配置
# ============================================================
# 修正表：请确保此处与你的 d1_debug.py 验证结果完全一致
# Y轴关节方向: J2取反, J3正向(用户实测), J5取反
DIRECTION_MAP = np.array([1.0, -1.0, 1.0, 1.0, -1.0, 1.0])

JOINT_AXES_RAW = [
    np.array([0, 0, -1]), np.array([0, 1, 0]), np.array([0, 1, 0]),
    np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([1, 0, 0]),
]
JOINT_ORIGINS = [
    np.array([0.0, 0.0, 0.0738]), np.array([0.0, -0.0276, 0.0578]),
    np.array([0.0, 0.0, 0.27]), np.array([0.05, 0.0275, 0.041325]),
    np.array([0.15468, -0.0258, 0.0]), np.array([0.0777, 0.025822, 0.00]),
]
GRIPPER_OFFSET = np.array([0.1218, 0.0, 0.0031])  # 原7.18cm + 末端机械爪5cm

# ============================================================
# 运动学核心函数
# ============================================================
def joint_rot(i, theta):
    a = theta * float(DIRECTION_MAP[i])
    ax = JOINT_AXES_RAW[i]
    c, s = np.cos(a), np.sin(a)
    if abs(ax[1]) > 0.9: return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    if abs(ax[0]) > 0.9: return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) if ax[2] > 0 else np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])

def fk(joints_rad):
    R, p = np.eye(3), np.zeros(3)
    for i in range(6):
        p = p + R @ JOINT_ORIGINS[i]
        R = R @ joint_rot(i, joints_rad[i])
    p = p + R @ GRIPPER_OFFSET
    return p, R

def jacobian(joints_rad):
    R, p = np.eye(3), np.zeros(3)
    jpos = [np.zeros(3)]  # jpos[0] = base
    for i in range(6):
        p = p + R @ JOINT_ORIGINS[i]
        jpos.append(p.copy())  # jpos[i+1] = joint i origin (before rotation)
        R = R @ joint_rot(i, joints_rad[i])
    p_ee = p + R @ GRIPPER_OFFSET
    J = np.zeros((6, 6))
    R_j = np.eye(3)
    for i in range(6):
        ax = JOINT_AXES_RAW[i] * DIRECTION_MAP[i]
        z_i = R_j @ ax
        # 修正: 旋转中心是 jpos[i+1] (当前关节原点), 而不是 jpos[i]
        J[:3, i] = np.cross(z_i, p_ee - jpos[i+1])
        J[3:, i] = z_i
        R_j = R_j @ joint_rot(i, joints_rad[i])
    return J

# ============================================================
# IK 求解器 (阻尼最小二乘法)
# ============================================================
def solve_ik(target_pos, max_iter=200, tol=1e-4):
    q = np.zeros(6)
    for i in range(max_iter):
        p_cur, _ = fk(q)
        err = target_pos - p_cur
        if np.linalg.norm(err) < tol: return q, True
        J = jacobian(q)
        # 阻尼系数，防止奇异点震荡
        damping = 0.05
        # 伪逆求解
        dq = J[:3].T @ np.linalg.pinv(J[:3] @ J[:3].T + damping**2 * np.eye(3)) @ err
        q += np.clip(dq, -0.1, 0.1) # 限制单步最大变化量
    return q, False

# ============================================================
# 指令发送
# ============================================================
def send(q_rad):
    # 如果你有零位偏差校准，可以在此减去 ZERO_OFFSETS
    final_deg = np.rad2deg(q_rad)
    cmd = ["/home/unitree/D1_SDK/build/python_bridge"] + [f"{a:.2f}" for a in np.concatenate([final_deg, [0.0]])]
    print(f"发送指令: {' '.join(cmd)}")
    subprocess.run(cmd)

if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("用法: python3 ik_solver.py <x> <y> <z>")
    else:
        target = np.array([float(x) for x in sys.argv[1:4]])
        q_res, success = solve_ik(target)
        if success:
            print(f"收敛成功，目标: {target}")
            send(q_res)
        else:
            print("未能收敛，目标位置可能不可达。")
