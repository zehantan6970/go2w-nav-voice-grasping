#!/usr/bin/env python3
"""
自动抓取脚本（Headless，无需按g）
在导航航点触发，自动检测并抓取水瓶
"""

import numpy as np, time, subprocess, sys, os, json, threading, cv2
if not hasattr(np, 'bool'): np.bool = bool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ik_solver2 as _ik
_ik.DIRECTION_MAP = np.array([1.0, -1.0, 1.0, 1.0, -1.0, 1.0])
from ik_solver2 import fk, jacobian

import pyrealsense2 as rs, tensorrt as trt, pycuda.driver as cuda, pycuda.autoinit

# ── 参数（与3dcatchv3保持一致） ──
HOME_DEG = [0.0, -75.0, 75.0, 0.0, 0.0, 0.0]
GRIP_OPEN = 55.0; GRIP_CLOSE = -5.0; SETTLE = 6.0
TRANSIT_FACTOR = 0.4; PREP_FACTOR = 0.7
CAM_OFFSET_X = 0.0; CAM_OFFSET_Y = 0.02; CAM_OFFSET_Z = 0.0
CAM_BASE_POS = np.array([0.06, 0.0, 0.35])
CAM_BASE_ROT = np.array([[0,0,1],[-1,0,0],[0,-1,0]], dtype=float)
ARM_REACH_MIN = 0.10; ARM_REACH_MAX = 0.62; ARM_Z_MIN = 0.05; ARM_Z_MAX = 0.60
ARM_BRIDGE = "/home/unitree/D1sdk/build/python_bridge"

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
    os.system(f"{ARM_BRIDGE} {' '.join(f'{v:.2f}' for v in list(d)+[g])}")

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

def step_joint(jnt_deg, grip=0.0, label="", max_retry=3):
    cmd = np.array(jnt_deg[:6], dtype=float)
    for attempt in range(max_retry + 1):
        send(jnt_deg, grip)
        time.sleep(SETTLE)
        act = read_angles()
        err = max(abs(cmd - np.array(act[:6])))
        if err < 5.0:
            if attempt > 0: print(f"    ✅ {label}: 第{attempt}次重试后到位")
            else: print(f"    ✅ {label}")
            return True
        elif attempt < max_retry:
            print(f"    ⚠️ {label}: err={err:.1f}°，第{attempt+1}次重试...")
            for ln in os.popen('pgrep -f get_arm_joint_angle').read().strip().split(chr(10)):
                if ln.strip():
                    try: os.kill(int(ln.strip()), 9)
                    except: pass
        else:
            print(f"    ❌ {label}: 重试失败")
    return False

def pick_xyz(x, y, z):
    target = np.array([x, y, z])
    q_target, ok = solve_ik_robust(target)
    target_deg = np.rad2deg(q_target)
    if not ok: print("   IK不可达"); return False
    home_np = np.array(HOME_DEG)
    transit_deg = home_np + (target_deg - home_np) * TRANSIT_FACTOR
    prep_deg = home_np + (target_deg - home_np) * PREP_FACTOR
    print(f"  目标: ({x:.3f},{y:.3f},{z:.3f}) J2={target_deg[1]:.1f}°")
    step_joint(HOME_DEG, GRIP_OPEN, "HOME")
    step_joint(list(transit_deg), GRIP_OPEN, "过渡")
    step_joint(list(prep_deg), GRIP_OPEN, "预备")
    step_joint(list(target_deg), GRIP_OPEN, "★目标")
    # 张开→闭合
    send(list(target_deg), GRIP_OPEN); time.sleep(1.5)
    send(list(target_deg), GRIP_CLOSE); time.sleep(2)
    print("   ✅ 已夹紧")
    step_joint(list(prep_deg), GRIP_CLOSE, "抬升")
    step_joint(HOME_DEG, GRIP_CLOSE, "HOME")
    return True

def cam_to_arm(x, y, z):
    p = CAM_BASE_ROT @ np.array([x, y, z]) + CAM_BASE_POS
    return np.array([p[2], p[1], p[0]])

def auto_grasp():
    """自动检测并抓取"""
    print("\n[auto_grasp] 启动...")

    # 清理DDS
    for ln in os.popen('pgrep -f get_arm_joint_angle').read().strip().split('\n'):
        if ln.strip(): os.system(f'kill {ln.strip()} 2>/dev/null')
    time.sleep(1)

    # 初始化相机+模型
    ep = os.path.join(os.path.dirname(os.path.abspath(__file__)), "bottle.engine")
    logger = trt.Logger(trt.Logger.WARNING)
    with open(ep,"rb") as f, trt.Runtime(logger) as rt:
        engine = rt.deserialize_cuda_engine(f.read())
    ctx = engine.create_execution_context()
    inp,out,bind,stm = [],[],[],cuda.Stream()
    for b in engine:
        sz = trt.volume(engine.get_binding_shape(b))
        try: dt = trt.nptype(engine.get_binding_dtype(b))
        except: dt = np.float32
        h = cuda.pagelocked_empty(sz,dt); d = cuda.mem_alloc(h.nbytes)
        bind.append(int(d))
        (inp if engine.binding_is_input(b) else out).append({'host':h,'device':d})

    pipe = rs.pipeline(); cfg = rs.config()
    cfg.enable_stream(rs.stream.color,640,480,rs.format.bgr8,30)
    cfg.enable_stream(rs.stream.depth,640,480,rs.format.z16,30)
    prof = pipe.start(cfg); align = rs.align(rs.stream.color)
    intr = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

    def preproc(img):
        r = cv2.resize(img,(640,640))
        n = cv2.cvtColor(r,cv2.COLOR_BGR2RGB).astype(np.float32)/255.0
        return np.expand_dims(np.transpose(n,(2,0,1)),0)

    found = False
    try:
        # 尝试3帧检测
        for frame_i in range(5):
            print(f"  [scan] 帧 {frame_i+1}/5...")
            frames = align.process(pipe.wait_for_frames())
            cf,df = frames.get_color_frame(), frames.get_depth_frame()
            if not cf or not df: continue
            img = np.asanyarray(cf.get_data())
            data = preproc(img)
            np.copyto(inp[0]['host'],data.ravel())
            cuda.memcpy_htod_async(inp[0]['device'],inp[0]['host'],stm)
            ctx.execute_async_v2(bindings=bind,stream_handle=stm.handle)
            cuda.memcpy_dtoh_async(out[0]['host'],out[0]['device'],stm)
            stm.synchronize()

            raw = out[0]['host']; nc = len(raw)//8400
            preds = np.reshape(raw,(nc,8400)).T
            best_conf = 0; best_row = None
            for row in preds:
                ss = row[4:]; ci = np.argmax(ss)
                if ss[ci] > best_conf:
                    best_conf = ss[ci]
                    best_row = row
            if best_conf > 0.5 and best_row is not None:
                cx,cy = best_row[0],best_row[1]
                cxp = max(0,min(int(cx*img.shape[1]/640),639))
                cyp = max(0,min(int(cy*img.shape[0]/640),479))
                d = df.get_distance(cxp,cyp)
                if d > 0:
                    pc = rs.rs2_deproject_pixel_to_point(intr,[cxp,cyp],d)
                    target = cam_to_arm(pc[0],pc[1],pc[2])
                    ox,oy,oz = CAM_OFFSET_X, CAM_OFFSET_Y, CAM_OFFSET_Z
                    target[0] += ox; target[1] += oy; target[2] += oz
                    if ARM_REACH_MIN < target[0] < ARM_REACH_MAX and ARM_Z_MIN < target[2] < ARM_Z_MAX:
                        print(f"  ✅ 检测到水瓶: ({target[0]:.3f},{target[1]:.3f},{target[2]:.3f})")
                        found = True
                        break
            time.sleep(0.3)
    finally:
        pipe.stop()
        cv2.destroyAllWindows()

    if not found:
        print("  ❌ 未检测到水瓶")
        return False

    # 执行抓取
    print("  [pick] 开始抓取...")
    success = pick_xyz(target[0], target[1], target[2])
    if success:
        print("  ✅ 抓取成功!")
    else:
        print("  ⚠️ 抓取失败")
    return success

if __name__ == "__main__":
    success = auto_grasp()
    sys.exit(0 if success else 1)
