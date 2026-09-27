#!/usr/bin/env python3
"""
3dcatchv5.py — 多饮品视觉抓取（V5: J1固定-90°）

轨迹: HOME → 预备 → 目标 → 抬升 → HOME
"""

import numpy as np, time, subprocess, sys, os, json, cv2, argparse
if not hasattr(np, 'bool'): np.bool = bool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ik_solver2 as _ik
_ik.DIRECTION_MAP = np.array([1.0, -1.0, 1.0, 1.0, -1.0, 1.0])
from ik_solver2 import fk, jacobian

# ═══════════════════════════════════════════
# 参数（与V3完全一致）
# ═══════════════════════════════════════════
J1_OFFSET = 90.0   # 固定J1偏移（所有角度输出J1-90°）
HOME_DEG = [0.0, -75.0, 75.0, 0.0, 0.0, 0.0]  # send()会自动加J1_OFFSET
GRIP_OPEN = 55.0; GRIP_CLOSE = -7; SETTLE = 2.3
PREP_FACTOR = 0.7
DRINK_NAMES = ["水", "可乐", "茶", "王老吉"]
CONF_THRESH = 0.85

# 相机偏移
CAM_OFFSET_X = -0.06 #-0.08
CAM_OFFSET_Y = 0.018 #0.018
CAM_OFFSET_Z = 0.16 #0.12
CAM_SCALE_Y_BASE = 0.45   # Y轴基础比例（在0.3m距离处）
CAM_SCALE_Y_PER_M = 0   # Y轴比例随深度变化率（每米）
CAM_BASE_POS = np.array([0.06, 0.0, 0.35])
CAM_BASE_ROT = np.array([[0,0,1],[-1,0,0],[0,-1,0]], dtype=float)
ARM_REACH_MIN = 0.10; ARM_REACH_MAX = 2.0; ARM_Z_MIN = 0.05; ARM_Z_MAX = 2.0
ARM_BRIDGE = "/home/unitree/D1sdk/build/python_bridge"

_selected_class = -1

# ═══════════════════════════════════════════
# 核心函数（与V3完全一致）
# ═══════════════════════════════════════════

def read_angles():
    try:
        r = subprocess.run(["timeout","2","/home/unitree/D1sdk/build/get_arm_joint_angle"],
                          capture_output=True, text=True, timeout=3)
        for line in r.stdout.split('\n'):
            if 'armFeedback_data:' in line and '"funcode":1' in line:
                d = json.loads(line.split('armFeedback_data:')[1].strip()).get('data',{})
                if 'angle0' in d:
                    return [d[f'angle{i}'] for i in range(7)]
    except: pass
    return [0.0]*7

def send(d, g=0):
    arr = list(d)
    arr[0] += J1_OFFSET  # J1偏移
    os.system(f"{ARM_BRIDGE} {' '.join(f'{v:.2f}' for v in arr+[g])}")

def solve_ik_robust(pos, max_iter=500, tol=5e-4):
    q = np.zeros(6); ep = float('inf'); dmp = 0.3
    for i in range(max_iter):
        pc,_=fk(q); e=np.linalg.norm(pos-pc)
        if e<tol: return q,True
        if e>ep: dmp=min(dmp*1.5,3)
        else: dmp=max(0.3*e+0.05,0.08)
        ep=e
        J3=jacobian(q)[:3]
        dq=J3.T@np.linalg.lstsq(J3@J3.T+dmp**2*np.eye(3),pos-pc,rcond=None)[0]
        q+=np.clip(dq,-min(0.15,0.08+e*0.5),min(0.15,0.08+e*0.5))
    pc,_=fk(q)
    return q,np.linalg.norm(pos-pc)<5*tol

def step_joint(jnt_deg, grip=0.0, label="", max_retry=2):
    cmd = np.array(jnt_deg[:6], dtype=float)
    for attempt in range(max_retry + 1):
        send(jnt_deg, grip)
        time.sleep(SETTLE)
        act = read_angles()
        # read_angles返回物理值，减去J1偏移再比较
        act_adj = np.array(act[:6], dtype=float)
        act_adj[0] -= J1_OFFSET
        err = max(abs(cmd - act_adj))
        if err < 7.0:
            if attempt > 0:
                print(f"    ✅ {label}: 第{attempt}次重试后到位 err={err:.1f}°")
            else:
                print(f"    ✅ {label}: err={err:.1f}°")
            return act
        elif attempt < max_retry:
            print(f"    ⚠️ {label}: err={err:.1f}°，第{attempt+1}次重试...")
            for _ln in os.popen('pgrep -f get_arm_joint_angle').read().strip().split(chr(10)):
                if _ln.strip():
                    try: os.kill(int(_ln.strip()), 9)
                    except: pass
        else:
            print(f"    ❌ {label}: 重试{max_retry}次失败 err={err:.1f}°")
    return act

def cam_to_arm(x, y, z):
    p = CAM_BASE_ROT @ np.array([x, y, z]) + CAM_BASE_POS
    # 距离自适应Y轴修正（近处畸变大，远处畸变小）
    depth_scale = CAM_SCALE_Y_BASE + CAM_SCALE_Y_PER_M * (z - 0.3)
    p[1] *= max(0.3, min(depth_scale, 2.0))  # 限制范围0.3~2.0
    return np.array([p[2], p[1], p[0]])  # X与Z交换

def pick_xyz(x, y, z, drink_name=""):
    target = np.array([x, y, z])
    q_target, ok = solve_ik_robust(target)
    target_deg = np.rad2deg(q_target)
    if not ok: print("   IK不可达"); return False
    home_np = np.array(HOME_DEG)
    prep_deg = home_np + (target_deg - home_np) * PREP_FACTOR
    print(f"   {drink_name} @ ({x:.3f},{y:.3f},{z:.3f})")

    step_joint(HOME_DEG, GRIP_OPEN, "HOME")
    step_joint(list(prep_deg), GRIP_OPEN, "预备")
    step_joint(list(target_deg), GRIP_OPEN, "★目标")

    send(list(target_deg), GRIP_OPEN); time.sleep(0.4)
    for _try in range(3):
        send(list(target_deg), GRIP_CLOSE); time.sleep(0.5)
        act = read_angles()
        grip_actual = act[6] if len(act) > 6 else 0
        if abs(GRIP_CLOSE - grip_actual) < 15.0:
            print(f"   ✅ 已夹紧 ({grip_actual:.1f}°)")
            break
        print(f"   ⚠️ 夹爪尝试{_try+1}: 实际{grip_actual:.1f}°")

    step_joint(list(prep_deg), GRIP_CLOSE, "抬升")
    step_joint(HOME_DEG, GRIP_CLOSE, "HOME")
    return True

# ═══════════════════════════════════════════
# 视觉主循环
# ═══════════════════════════════════════════

def vision_loop(engine_path="bottle.engine"):
    global _selected_class
    # 停掉cam_transfer，释放相机
    os.system("pkill -f cam_transfer 2>/dev/null; sleep 0.5")
    import pyrealsense2 as rs, tensorrt as trt, pycuda.driver as cuda, pycuda.autoinit

    logger = trt.Logger(trt.Logger.WARNING)
    with open(engine_path, "rb") as f, trt.Runtime(logger) as rt:
        engine = rt.deserialize_cuda_engine(f.read())
    ctx = engine.create_execution_context()
    inp, out, bind, stm = [], [], [], cuda.Stream()
    for b in engine:
        sz = trt.volume(engine.get_binding_shape(b))
        try: dt = trt.nptype(engine.get_binding_dtype(b))
        except: dt = np.float32
        h = cuda.pagelocked_empty(sz, dt); d = cuda.mem_alloc(h.nbytes)
        bind.append(int(d))
        (inp if engine.binding_is_input(b) else out).append({'host': h, 'device': d})

    pipe = rs.pipeline(); cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    prof = pipe.start(cfg); align = rs.align(rs.stream.color)
    intr = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

    def preproc(img):
        r = cv2.resize(img, (640, 640))
        n = cv2.cvtColor(r, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        return np.expand_dims(np.transpose(n, (2, 0, 1)), 0)

    print("\n"+"="*60)
    print("🤖 3DCatch v5 — 多饮品抓取")
    print(f"   {', '.join(f'{i}={n}' for i,n in enumerate(DRINK_NAMES))}")
    print(f"   阈值: {CONF_THRESH*100:.0f}%")
    print("  [0-3] 选择  [g] 抓取  [q] 退出")
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

            raw = out[0]['host']; nc = len(raw) // 8400
            preds = np.reshape(raw, (nc, 8400)).T
            detections = []

            for row in preds:
                scores = row[4:8]
                ci = np.argmax(scores)
                conf = float(scores[ci])
                if conf > CONF_THRESH:
                    cx, cy, w, h = row[0], row[1], row[2], row[3]
                    xf, yf = img.shape[1]/640, img.shape[0]/640
                    x1 = int((cx-w/2)*xf); y1 = int((cy-h/2)*yf)
                    x2 = int((cx+w/2)*xf); y2 = int((cy+h/2)*yf)
                    cxp = max(0, min(int(cx*xf), 639))
                    cyp = max(0, min(int(cy*yf), 479))
                    d = df.get_distance(cxp, cyp)
                    if d > 0:
                        pc = rs.rs2_deproject_pixel_to_point(intr, [cxp, cyp], d)
                        target = cam_to_arm(pc[0], pc[1], pc[2])
                        target[0] += CAM_OFFSET_X
                        target[1] += CAM_OFFSET_Y
                        target[2] += CAM_OFFSET_Z
                        detections.append((ci, conf, x1, y1, x2, y2, target))

            for ci, conf, x1, y1, x2, y2, target in detections:
                color = (0,255,0) if ci == _selected_class else (100,100,100)
                cv2.rectangle(img, (x1,y1), (x2,y2), color, 2)
                cv2.putText(img, f"{DRINK_NAMES[ci]} {conf*100:.0f}%",
                          (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            if _selected_class >= 0:
                cv2.putText(img, f"> {DRINK_NAMES[_selected_class]}", (10,30),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
            else:
                cv2.putText(img, "按 0-3 选择饮品", (10,30),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

            # 写入JPEG供Web前端实时查看
            
            cv2.imwrite("/tmp/cam2_feed.jpg", img)  # 供前端使用
            cv2.imshow("3DCatch v5", img)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'): break
            elif ord('0') <= key <= ord('3'):
                _selected_class = key - ord('0')
                print(f"\n🎯 已选择: {DRINK_NAMES[_selected_class]}")
            elif key == ord('g'):
                if _selected_class < 0:
                    print("  ⚠️ 请先按 0-3 选择饮品"); continue
                best = None
                for ci, conf, _, _, _, _, target in detections:
                    if ci == _selected_class and conf > CONF_THRESH:
                        if best is None or conf > best[1]:
                            best = (ci, conf, target)
                if best is None:
                    print(f"  ⚠️ 未检测到{DRINK_NAMES[_selected_class]}"); continue
                _, conf, target = best
                tx, ty, tz = target
                print(f"\n🔴 {DRINK_NAMES[_selected_class]} ({conf*100:.0f}%)")
                print(f"   ({tx:.3f}, {ty:.3f}, {tz:.3f})")
                if not (ARM_REACH_MIN < tx < ARM_REACH_MAX): print("  ⚠️ X超范围"); continue
                if not (ARM_Z_MIN < tz < ARM_Z_MAX): print("  ⚠️ Z超范围"); continue
                q_t, ok_ik = solve_ik_robust(np.array([tx, ty, tz]), max_iter=300, tol=5e-4)
                if np.linalg.norm(np.array([tx, ty, tz]) - fk(q_t)[0]) < 0.02:
                    pick_xyz(tx, ty, tz, DRINK_NAMES[ci])
                else:
                    print("  ⚠️ IK不可达")
    finally:
        pipe.stop(); cv2.destroyAllWindows()
        print("[关闭]")

# ═══════════════════════════════════════════
# 自动抓取模式（导航触发，无需按键）
# ═══════════════════════════════════════════

def auto_grasp_mode(engine_path, drink_class, delay=3.0):
    """
    自动抓取模式：选择饮品类别，等待delay秒后自动检测并抓取
    由导航程序触发，无需人工按键
    """
    global _selected_class
    _selected_class = drink_class
    drink_name = DRINK_NAMES[drink_class]
    print(f"\n[auto] 目标: {drink_name} (class={drink_class})")
    print(f"[auto] 等待 {delay}s 让导航到位...")
    time.sleep(delay)

    # 停掉cam_transfer，释放相机
    os.system("pkill -f cam_transfer 2>/dev/null; sleep 0.5")

    import pyrealsense2 as rs, tensorrt as trt, pycuda.driver as cuda, pycuda.autoinit

    logger = trt.Logger(trt.Logger.WARNING)
    with open(engine_path, "rb") as f, trt.Runtime(logger) as rt:
        engine = rt.deserialize_cuda_engine(f.read())
    ctx = engine.create_execution_context()
    inp, out, bind, stm = [], [], [], cuda.Stream()
    for b in engine:
        sz = trt.volume(engine.get_binding_shape(b))
        try: dt = trt.nptype(engine.get_binding_dtype(b))
        except: dt = np.float32
        h = cuda.pagelocked_empty(sz, dt); d = cuda.mem_alloc(h.nbytes)
        bind.append(int(d))
        (inp if engine.binding_is_input(b) else out).append({'host': h, 'device': d})

    pipe = rs.pipeline(); cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    prof = pipe.start(cfg); align = rs.align(rs.stream.color)
    intr = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

    def preproc(img):
        r = cv2.resize(img, (640, 640))
        n = cv2.cvtColor(r, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        return np.expand_dims(np.transpose(n, (2, 0, 1)), 0)

    print(f"[auto] 开始检测 {drink_name}...")
    found = False
    best_target = None
    best_conf = 0.0

    try:
        # 持续扫描最多30帧（约3秒）或直到找到高置信度目标
        for frame_i in range(10):
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

            raw = out[0]['host']; nc = len(raw) // 8400
            preds = np.reshape(raw, (nc, 8400)).T

            for row in preds:
                scores = row[4:8]
                ci = np.argmax(scores)
                conf = float(scores[ci])
                if ci == drink_class and conf > CONF_THRESH:
                    cx, cy = row[0], row[1]
                    cxp = max(0, min(int(cx * img.shape[1] / 640), 639))
                    cyp = max(0, min(int(cy * img.shape[0] / 640), 479))
                    d = df.get_distance(cxp, cyp)
                    if d > 0:
                        pc = rs.rs2_deproject_pixel_to_point(intr, [cxp, cyp], d)
                        target = cam_to_arm(pc[0], pc[1], pc[2])
                        target[0] += CAM_OFFSET_X
                        target[1] += CAM_OFFSET_Y
                        target[2] += CAM_OFFSET_Z
                        if (ARM_REACH_MIN < target[0] < ARM_REACH_MAX and
                            ARM_Z_MIN < target[2] < ARM_Z_MAX):
                            if conf > best_conf:
                                best_conf = conf
                                best_target = target.copy()
                                print(f"  [auto] 帧{frame_i+1}: {drink_name} {conf*100:.0f}%"
                                      f" @ ({target[0]:.3f},{target[1]:.3f},{target[2]:.3f})")

            if best_conf > CONF_THRESH:
                # 确认检测稳定：多看到一帧
                if frame_i > 0 and best_conf >= 0.9:
                    found = True
                    break
                time.sleep(0.1)

        # 再额外扫几帧确认
        if not found and best_conf > CONF_THRESH:
            found = True

    finally:
        pipe.stop()
        cv2.destroyAllWindows()

    if not found or best_target is None:
        print(f"  ❌ [auto] 未检测到 {drink_name}")
        return False

    tx, ty, tz = best_target
    print(f"\n✅ [auto] 锁定额{drink_name} @ ({tx:.3f}, {ty:.3f}, {tz:.3f})")
    q_t, ok_ik = solve_ik_robust(np.array([tx, ty, tz]), max_iter=300, tol=5e-4)
    if np.linalg.norm(np.array([tx, ty, tz]) - fk(q_t)[0]) < 0.02:
        success = pick_xyz(tx, ty, tz, drink_name)
        print(f"\n{'✅' if success else '❌'} [auto] 抓取{'成功' if success else '失败'}")
        return success
    else:
        print(f"  ❌ [auto] IK不可达")
        return False


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="3DCatch v5 — 多饮品视觉抓取")
    parser.add_argument("--drink", type=int, default=None, choices=[0,1,2,3],
                        help=f"饮品类别: 0=水, 1=可乐, 2=茶, 3=王老吉")
    parser.add_argument("--delay", type=float, default=3.0,
                        help="导航到位后等待秒数（默认3s）")
    args = parser.parse_args()

    ep = os.path.join(os.path.dirname(os.path.abspath(__file__)), "bottle.engine")
    if not os.path.exists(ep):
        print(f"找不到引擎文件: {ep}")
        sys.exit(1)

    try:
        if args.drink is not None:
            # 自动抓取模式
            print(f"[v5] 自动模式启动: drink={args.drink}({DRINK_NAMES[args.drink]}), delay={args.delay}s")
            success = auto_grasp_mode(ep, args.drink, args.delay)
            sys.exit(0 if success else 1)
        else:
            # 交互模式
            vision_loop(ep)
    finally:
        os.system(f"nohup python3 {os.path.dirname(os.path.abspath(__file__))}/cam_transfer.py > /dev/null 2>&1 &")
        print("[v5] cam_transfer 已重新启动")
