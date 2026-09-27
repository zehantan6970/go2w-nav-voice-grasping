#!/usr/bin/env python3
"""
3dcatchv2.py — 机械臂视觉抓取 v2
"""

import numpy as np
if not hasattr(np, 'bool'):
    np.bool = bool

import cv2, sys, os, time, subprocess, threading, json, signal

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ik_solver2 as _ik
_ik.DIRECTION_MAP = np.array([1.0, -1.0, 1.0, 1.0, -1.0, 1.0])
from ik_solver2 import fk, solve_ik, JOINT_ORIGINS, joint_rot, jacobian

ARM_BRIDGE = "/home/unitree/D1_SDK/build/python_bridge"

# ============================================================
# 📐 可调参数（按需修改）
# ============================================================

# ── 归零姿态（关节角度） ──
HOME_JOINT_DEG = [0.0, -75.0, 75.0, 0.0, 0.0, 0.0]

# ── 夹爪角度 ──
GRIP_OPEN = 55.0       # 张开角度
GRIP_CLOSE = -5.0       # 闭合角度（负值=过紧）

# ── 轨迹高度（各阶段高于目标物的距离，单位米） ──
TRANSIT_Z_OFFSET = 0.15 # 过渡位高于目标(m)
PREP_Z_OFFSET = 0.10    # 预备位高于目标(m)
LIFT_Z_OFFSET = 0.04    # 抓取后抬升高度(m)

# ── 动作间隔 ──
SETTLE_TIME = 4.0       # 每一步等待时间(秒)

# ── 相机坐标偏移校准（针对检测偏差） ──
CAM_OFFSET_X = 0.0    # X偏移，正=向前
CAM_OFFSET_Y = 0.03    # Y偏移，正=向左
CAM_OFFSET_Z = 0.08     # Z偏移，正=向上

# ── 工作空间安全边界 ──
ARM_REACH_MIN = 0.10    # X最小(m)
ARM_REACH_MAX = 0.62    # X最大(m)
ARM_Z_MIN = 0.05        # Z最小(m)
ARM_Z_MAX = 0.60        # Z最大(m)

# ── 固定相机外参（相机在机体上的安装位置） ──
CAM_BASE_POS = np.array([0.06, -0.01, 0.35])  # [前移, 左右, 高度](m)
CAM_BASE_ROT = np.array([[0,0,1],[-1,0,0],[0,-1,0]], dtype=float)  # 轴映射

# ============================================================
# 🚀 核心函数（以下无需修改）
# ============================================================

def send_cmd(deg_list, gripper=0.0):
    vals = [str(f"{v:.2f}") for v in list(deg_list) + [gripper]]
    subprocess.run([ARM_BRIDGE] + vals, capture_output=True, timeout=5)

def send_with_verify(deg_list, gripper=0.0, label="", max_retries=3):
    cmd_deg = np.array(deg_list, dtype=float)
    for attempt in range(max_retries + 1):
        send_cmd(deg_list, gripper)
        time.sleep(2)
        actual_q = get_q()
        actual_deg = np.rad2deg(actual_q) if np.any(actual_q) else np.zeros(6)
        err = np.abs(cmd_deg[:6] - actual_deg[:6])
        max_err = np.max(err)
        if max_err < 5.0:
            if attempt > 0:
                print(f"    ✓ 第{attempt}次重试后到位 (max_err={max_err:.1f}°)")
            return True
        elif attempt < max_retries:
            print(f"    ⚠️ {label} 未执行(max_err={max_err:.1f}°)，第{attempt+1}次重试...")
        else:
            print(f"    ❌ {label} 重试{max_retries}次仍未执行!")
            return False
    return False

# ── 实时关节角度读取 ──
for _ln in os.popen('pgrep -f get_arm_joint_angle').read().strip().split('\n'):
    if _ln.strip(): os.system(f'kill {_ln.strip()} 2>/dev/null')
_current_deg = [0.0] * 6
_lock = threading.Lock()
_reader_proc = [None]

def _reader():
    try:
        proc = subprocess.Popen("/home/unitree/D1_SDK/build/get_arm_joint_angle",
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        _reader_proc[0] = proc
        for line in iter(proc.stdout.readline, ''):
            if 'armFeedback_data:' in line and '"funcode":1' in line:
                try:
                    d = json.loads(line.split('armFeedback_data:')[1].strip()).get('data', {})
                    if 'angle0' in d:
                        with _lock:
                            _current_deg = [d['angle0'], d['angle1'], d['angle2'],
                                            d['angle3'], d['angle4'], d['angle5']]
                except: pass
    except: pass
    finally:
        _reader_proc[0] = None

_reader_th = threading.Thread(target=_reader, daemon=True)
_reader_th.start()

def get_q():
    with _lock:
        return np.deg2rad(_current_deg)

def stop_reader():
    if _reader_proc[0]:
        _reader_proc[0].terminate()
        _reader_proc[0] = None

# ── 自适应阻尼 IK ──
def solve_ik_robust(target_pos, max_iter=500, tol=5e-4):
    q = np.zeros(6)
    err_prev = float('inf')
    for i in range(max_iter):
        p_cur, _ = fk(q)
        err = target_pos - p_cur
        n = np.linalg.norm(err)
        if n < tol:
            return q, True, i
        if n > err_prev:
            damping = min(damping * 1.5, 3.0)
        else:
            damping = max(0.3 * n + 0.05, 0.08)
        err_prev = n
        J = jacobian(q)[:3]
        dq = J.T @ np.linalg.lstsq(J @ J.T + damping**2 * np.eye(3), err, rcond=None)[0]
        step = min(0.15, 0.08 + n * 0.5)
        q += np.clip(dq, -step, step)
    p_cur, _ = fk(q)
    n = np.linalg.norm(target_pos - p_cur)
    return q, n < 5*tol, max_iter

# ── 发送（带验证） ──
def move_joint(target_deg, gripper=0.0, label=""):
    print(f"  {label}: [{target_deg[0]:.0f},{target_deg[1]:.0f},{target_deg[2]:.0f},...] 爪={gripper}°")
    send_with_verify(target_deg, gripper, label)

def move_cart(target_xyz, gripper=0.0, label=""):
    q, ok, _ = solve_ik_robust(np.array(target_xyz, dtype=float))
    p, _ = fk(q)
    e = np.linalg.norm(np.array(target_xyz) - p)
    j_deg = np.rad2deg(q)
    tag = '✅' if e < 0.02 else '⚠️'
    print(f"  {tag} {label}: 目标=({target_xyz[0]:.3f},{target_xyz[1]:.3f},{target_xyz[2]:.3f})")
    print(f"        FK=({p[0]:.3f},{p[1]:.3f},{p[2]:.3f}) err={e*1000:.1f}mm")
    print(f"        关节=[{j_deg[0]:.1f},{j_deg[1]:.1f},{j_deg[2]:.1f},{j_deg[3]:.1f},{j_deg[4]:.1f},{j_deg[5]:.1f}]")
    if ok or e < 0.02:
        send_with_verify(j_deg, gripper, label)
    else:
        print(f"  ❌ 目标不可达！跳过发送")
    return q

def cam_to_arm(x, y, z):
    p_cam = np.array([x, y, z])
    return CAM_BASE_ROT @ p_cam + CAM_BASE_POS

# ── 抓取轨迹 ──
def pick(target_xyz):
    tx, ty, tz = target_xyz
    ph, _ = fk(np.deg2rad(HOME_JOINT_DEG))
    home_x, home_z = ph[0], ph[2]

    print(f"\n{'='*50}")
    print(f"🎯 目标: ({tx:.3f},{ty:.3f},{tz:.3f})")
    if tx < ARM_REACH_MIN or tx > ARM_REACH_MAX:
        print(f"  ❌ X={tx:.2f}m 超出 [{ARM_REACH_MIN},{ARM_REACH_MAX}]")
        return False
    if tz < ARM_Z_MIN or tz > ARM_Z_MAX:
        print(f"  ❌ Z={tz:.2f}m 超出 [{ARM_Z_MIN},{ARM_Z_MAX}]")
        return False
    print(f"{'='*50}")

    trans = [tx, ty, tz + TRANSIT_Z_OFFSET]   # 过渡位
    pre = [tx, ty, tz + PREP_Z_OFFSET]          # 预备位
    lift = [tx, ty, tz + LIFT_Z_OFFSET]         # 抓取后抬升

    # [0] HOME
    print(f"\n[0/5] HOME + 张开")
    move_joint(HOME_JOINT_DEG, GRIP_OPEN, "HOME")
    time.sleep(SETTLE_TIME)

    # [1a] 过渡（HOME→预备的中间点）
    mid = [(tx + home_x) / 2, ty / 2, (trans[2] + home_z) / 2]
    print(f"\n[1a] 过渡: ({mid[0]:.3f},{mid[1]:.3f},{mid[2]:.3f})")
    move_cart(mid, GRIP_OPEN, "过渡")
    time.sleep(SETTLE_TIME)

    # [1] 预备
    print(f"\n[1/5] 预备")
    move_cart(pre, GRIP_OPEN, "预备")
    time.sleep(SETTLE_TIME)

    # [2] 前进
    print(f"\n[2/5] 前进 → 抓取位")
    move_cart([tx, ty, tz], GRIP_OPEN, "前进")
    time.sleep(SETTLE_TIME)

    # [3] 闭合
    print(f"\n[3/5] 闭合夹爪")
    move_cart([tx, ty, tz], GRIP_CLOSE, "闭合")
    time.sleep(2.0)
    print("       ✅ 已夹紧")

    # [4] 抬升
    print(f"\n[4/5] 抬升 ↑{LIFT_Z_OFFSET*100:.0f}cm")
    move_cart(lift, GRIP_CLOSE, "抬升")
    time.sleep(SETTLE_TIME)

    # [5] HOME
    print(f"\n[5/5] HOME")
    move_joint(HOME_JOINT_DEG, GRIP_CLOSE, "归位")
    time.sleep(SETTLE_TIME)

    print(f"\n{'='*50}\n✅ 完成!\n")
    return True

# ── 视觉主循环 ──
def vision_loop(engine_path="bottle.engine"):
    import pyrealsense2 as rs
    import tensorrt as trt
    import pycuda.driver as cuda, pycuda.autoinit

    logger = trt.Logger(trt.Logger.WARNING)
    with open(engine_path, "rb") as f, trt.Runtime(logger) as rt:
        engine = rt.deserialize_cuda_engine(f.read())
    ctx = engine.create_execution_context()
    inp, out, bind, stm = [], [], [], cuda.Stream()
    for b in engine:
        sz = trt.volume(engine.get_binding_shape(b))
        try: dt = trt.nptype(engine.get_binding_dtype(b))
        except: dt = np.float32
        h = cuda.pagelocked_empty(sz, dt)
        d = cuda.mem_alloc(h.nbytes)
        bind.append(int(d))
        (inp if engine.binding_is_input(b) else out).append({'host': h, 'device': d})

    pipe = rs.pipeline(); cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    prof = pipe.start(cfg)
    align = rs.align(rs.stream.color)
    intr = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

    def preproc(img):
        r = cv2.resize(img, (640, 640))
        n = cv2.cvtColor(r, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        return np.expand_dims(np.transpose(n, (2, 0, 1)), 0)

    print("\n"+"="*60)
    print("🤖 3DCatch v2   [g]抓取  [q]退出")
    print("="*60)

    try:
        while True:
            frames = align.process(pipe.wait_for_frames())
            cf, df = frames.get_color_frame(), frames.get_depth_frame()
            if not cf or not df: continue
            img = np.asanyarray(cf.get_data())
            data = preproc(img)
            np.copyto(inp[0]['host'], data.ravel())
            cuda.memcpy_htod_async(inp[0]['device'], inp[0]['host'], stm)
            ctx.execute_async_v2(bindings=bind, stream_handle=stm.handle)
            cuda.memcpy_dtoh_async(out[0]['host'], out[0]['device'], stm)
            stm.synchronize()

            raw = out[0]['host']
            nc = len(raw) // 8400
            preds = np.reshape(raw, (nc, 8400)).T
            boxes, confs, target = [], [], None
            for row in preds:
                ss = row[4:]; ci = np.argmax(ss)
                if ss[ci] > 0.5:
                    cx, cy, w, h = row[0], row[1], row[2], row[3]
                    xf, yf = img.shape[1]/640, img.shape[0]/640
                    boxes.append([int((cx-w/2)*xf), int((cy-h/2)*yf), int(w*xf), int(h*yf)])
                    confs.append(float(ss[ci]))
            idx = cv2.dnn.NMSBoxes(boxes, confs, 0.5, 0.4)
            if len(idx) > 0:
                bi = idx[0] if isinstance(idx, (list,np.ndarray)) else idx.flatten()[0]
                x, y, w, h = boxes[bi]
                cv2.rectangle(img, (x, y), (x+w, y+h), (0,255,0), 2)
                cxp = max(0, min(int(x+w/2), 639))
                cyp = max(0, min(int(y+h/2), 479))
                d = df.get_distance(cxp, cyp)
                if d > 0:
                    pc = rs.rs2_deproject_pixel_to_point(intr, [cxp, cyp], d)
                    target = cam_to_arm(pc[0], pc[1], pc[2])
                    in_range = (ARM_REACH_MIN < target[0] < ARM_REACH_MAX and ARM_Z_MIN < target[2] < ARM_Z_MAX)
                    color = (0,255,0) if in_range else (0,0,255)
                    cv2.putText(img, f"X={target[0]*100:.0f} Y={target[1]*100:.0f} Z={target[2]*100:.0f}cm",
                                (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            cv2.imshow("3DCatch v2", img)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'): break
            if key == ord('g') and target is not None:
                adj_target = target.copy()
                adj_target[0] += CAM_OFFSET_X
                adj_target[1] += CAM_OFFSET_Y
                adj_target[2] += CAM_OFFSET_Z
                print(f"\n🔴 抓取: X={adj_target[0]:.3f} Y={adj_target[1]:.3f} Z={adj_target[2]:.3f}")
                print(f"         (原始: {target[0]:.3f},{target[1]:.3f},{target[2]:.3f})")
                if adj_target[0] < ARM_REACH_MIN or adj_target[0] > ARM_REACH_MAX:
                    print(f"  ⚠️ X超范围")
                elif adj_target[2] < ARM_Z_MIN or adj_target[2] > ARM_Z_MAX:
                    print(f"  ⚠️ Z超范围")
                else:
                    q_try, ok_try, _ = solve_ik_robust(adj_target, max_iter=300, tol=5e-4)
                    p_try, _ = fk(q_try)
                    ik_err = np.linalg.norm(adj_target - p_try)
                    if ik_err < 0.02:
                        print(f"  ✅ IK可达")
                        pick(adj_target)
                    else:
                        print(f"  ⚠️ IK不可达 (err={ik_err*1000:.0f}mm)")
            elif key == ord('g') and target is None:
                print("  ⚠️ 无目标")
    finally:
        stop_reader()
        pipe.stop(); cv2.destroyAllWindows()
        print("[关闭]")

if __name__ == "__main__":
    if len(sys.argv) >= 4:
        pick([float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])])
    elif len(sys.argv) == 2:
        vision_loop(sys.argv[1])
    else:
        ep = os.path.join(os.path.dirname(os.path.abspath(__file__)), "bottle.engine")
        vision_loop(ep) if os.path.exists(ep) else print("用法: python3 3dcatchv2.py [x y z]")
