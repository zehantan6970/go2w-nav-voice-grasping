#!/usr/bin/env python3
import numpy as np
import subprocess
import sys

# ============================================================
# 参数配置 (已确认修正方向)
# ============================================================
JOINT_AXES_RAW = [
    np.array([0, 0, -1]), np.array([0, 1, 0]), np.array([0, 1, 0]),
    np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([1, 0, 0]),
]
# 修正表：J2, J3, J5 为反向
DIRECTION_MAP = np.array([1.0, -1.0, -1.0, 1.0, -1.0, 1.0])

JOINT_ORIGINS = [
    np.array([0.0, 0.0, 0.0738]), np.array([0.0, -0.0276, 0.0578]),
    np.array([0.0, -0.0004, 0.27]), np.array([0.05, 0.0275, 0.041325]),
    np.array([0.15468, -0.0258, 0.0001]), np.array([0.0777, 0.025822, -0.0010718]),
]
GRIPPER_OFFSET = np.array([0.0718, 0.0, 0.0031])

# ============================================================
# 运动学核心
# ============================================================
def joint_rot(i, theta):
    a = theta * DIRECTION_MAP[i]
    ax = JOINT_AXES_RAW[i]
    if abs(ax[1]) > 0.9: return np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])
    if abs(ax[0]) > 0.9: return np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])
    return np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]]) if ax[2] > 0 else np.array([[np.cos(a), np.sin(a), 0], [-np.sin(a), np.cos(a), 0], [0, 0, 1]])

def fk(joints_rad):
    R, p = np.eye(3), np.zeros(3)
    for i in range(6):
        p = p + R @ JOINT_ORIGINS[i]
        R = R @ joint_rot(i, joints_rad[i])
    p = p + R @ GRIPPER_OFFSET
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

def solve_ik(target_pos):
    q = np.zeros(6)
    for i in range(100):
        p_cur, _ = fk(q)
        err = target_pos - p_cur
        if np.linalg.norm(err) < 1e-4: return q, True
        J = jacobian(q)
        # 使用伪逆法求解，增加阻尼防止震荡
        dq = J[:3].T @ np.linalg.pinv(J[:3] @ J[:3].T + 0.1**2 * np.eye(3)) @ err
        q += np.clip(dq, -0.1, 0.1) # 限制单次步长，避免突变
    return q, False

# ============================================================
# 执行部分
# ============================================================
def send(joints_deg):
    cmd = "/home/unitree/D1_SDK/build/python_bridge"
    args = [f"{a:.2f}" for a in np.concatenate([joints_deg, [0.0]])]
    try:
        print(f"正在调用指令: {cmd} {' '.join(args)}")
        res = subprocess.run([cmd] + args, capture_output=True, text=True, check=True)
        print("输出:", res.stdout)
    except Exception as e:
        print(f"指令发送失败: {e}", file=sys.stderr)

if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("用法: python3 script.py <x> <y> <z>")
    else:
        target = np.array([float(x) for x in sys.argv[1:4]])
        q_rad, success = solve_ik(target)
        if success:
            print("计算收敛，发送指令...")
            send(np.rad2deg(q_rad))
        else:
            print("计算未收敛，停止发送")
