#!/usr/bin/env python3
"""
3dcatchv3.py — 视觉抓取精度验证

流程: 相机检测目标→偏移校准→IK→关节轨迹→读取实际角度→报告到位精度
"""
import time
import os, signal as _sig
for _ln in os.popen("pgrep -f \"get_arm_joint\\|python_bridge\"").read().strip().split(chr(10)):
    if _ln.strip():
        try: os.kill(int(_ln.strip()), _sig.SIGKILL)
        except: pass
time.sleep(1)
import numpy as np, time, subprocess, sys, os, json

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ik_solver2 as _ik
_ik.DIRECTION_MAP = np.array([1.0, -1.0, 1.0, 1.0, -1.0, 1.0])
from ik_solver2 import fk, jacobian

# ── 参数 ──
HOME_DEG = [0.0, -75.0, 75.0, 0.0, 0.0, 0.0]
GRIP_OPEN = 55.0; GRIP_CLOSE = -7.5; SETTLE = 6.0
TRANSIT_FACTOR = 0.4; PREP_FACTOR = 0.7

# ═══ 相机偏移校准 ═══
# 调这些值直到画面显示的坐标 = 实际水瓶位置
# CAM_OFFSET_X: 正数=向前   CAM_OFFSET_Y: 正数=向左   CAM_OFFSET_Z: 正数=向上
CAM_OFFSET_X = 0.07     # HEIGHT
CAM_OFFSET_Y = 0.02    # +=LEFT
CAM_OFFSET_Z = 0.0    # FRONT
CAM_BASE_POS = np.array([0.06, 0.0, 0.35])
CAM_BASE_ROT = np.array([[0,0,1],[-1,0,0],[0,-1,0]], dtype=float)
ARM_REACH_MIN = 0.10; ARM_REACH_MAX = 0.62; ARM_Z_MIN = 0.05; ARM_Z_MAX = 0.60
ARM_BRIDGE = "/home/unitree/D1_SDK/build/python_bridge"

def read_angles():
    """返回 [j1..j6, gripper] 共7个角度"""
    try:
        r = subprocess.run(["timeout","2","/home/unitree/D1_SDK/build/get_arm_joint_angle"],
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

def grip_with_verify(jnt_deg, target_grip, label="", max_retry=3):
    """发送夹爪指令并验证角度到位"""
    for attempt in range(max_retry + 1):
        send(jnt_deg, target_grip)
        time.sleep(1.5)
        act = read_angles()
        actual_grip = act[6] if len(act) > 6 else 0
        err = abs(target_grip - actual_grip)
        if err < 10.0:
            if attempt > 0:
                print(f"    ✅ {label}: 第{attempt}次重试后到位 ({actual_grip:.1f}°)")
            else:
                print(f"    ✅ {label}: 夹爪={actual_grip:.1f}°")
            return True
        elif attempt < max_retry:
            print(f"    ⚠️ {label}: 实际{actual_grip:.1f}°≠目标{target_grip:.0f}°，重试...")
        else:
            print(f"    ❌ {label}: 重试失败 ({actual_grip:.1f}°) ")
    return False

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
            if attempt > 0:
                print(f"    ✅ {label}: 第{attempt}次重试后到位 err={err:.1f}°")
            else:
                print(f"    ✅ {label}: err={err:.1f}°")
            return act
        elif attempt < max_retry:
            print(f"    ⚠️ {label}: err={err:.1f}°，第{attempt+1}次重试...")
            # 重试前清理残留DDS进程
            for _ln in os.popen('pgrep -f get_arm_joint_angle').read().strip().split(chr(10)):
                if _ln.strip():
                    try: os.kill(int(_ln.strip()), 9)
                    except: pass
        else:
            print(f"    ❌ {label}: 重试{max_retry}次失败 err={err:.1f}°")
    return act

def cam_to_arm(x, y, z):
    p = CAM_BASE_ROT @ np.array([x, y, z]) + CAM_BASE_POS
    return np.array([p[2], p[1], p[0]])  # X与Z交换

def run_verification():
    """视觉主循环：检测→抓取→报告精度"""
    import cv2, pyrealsense2 as rs
    import tensorrt as trt, pycuda.driver as cuda, pycuda.autoinit

    # TensorRT
    ep = os.path.join(os.path.dirname(os.path.abspath(__file__)), "bottle.engine")
    logger = trt.Logger(trt.Logger.WARNING)
    with open(ep, "rb") as f, trt.Runtime(logger) as rt:
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
    print("🤖 3DCatch v3 — 视觉抓取精度验证")
    print("  [g] 检测→抓取→报告精度")
    print("  [q] 退出")
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
            boxes, confs, target_raw = [], [], None
            for row in preds:
                ss = row[4:]; ci = np.argmax(ss)
                if ss[ci] > 0.5:
                    cx, cy, w, h = row[0], row[1], row[2], row[3]
                    xf, yf = img.shape[1]/640, img.shape[0]/640
                    boxes.append([int((cx-w/2)*xf), int((cy-h/2)*yf), int(w*xf), int(h*yf)])
                    confs.append(float(ss[ci]))
            idx = cv2.dnn.NMSBoxes(boxes, confs, 0.5, 0.4)
            if len(idx) > 0:
                bi = idx[0] if isinstance(idx,(list,np.ndarray)) else idx.flatten()[0]
                x,y,w,h = boxes[bi]
                cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
                cxp = max(0, min(int(x+w/2), 639))
                cyp = max(0, min(int(y+h/2), 479))
                d = df.get_distance(cxp, cyp)
                if d > 0:
                    pc = rs.rs2_deproject_pixel_to_point(intr, [cxp, cyp], d)
                    target_raw = cam_to_arm(pc[0], pc[1], pc[2])
                    in_range = (ARM_REACH_MIN < target_raw[0] < ARM_REACH_MAX and ARM_Z_MIN < target_raw[2] < ARM_Z_MAX)
                    color = (0,255,0) if in_range else (0,0,255)
                    cv2.putText(img, f"X={target_raw[0]*100:.0f} Y={target_raw[1]*100:.0f} Z={target_raw[2]*100:.0f}cm",
                                (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            cv2.imshow("3DCatch v3", img)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'): break
            if key == ord('s'):
                # 自动扫描: 旋转机器狗寻找水瓶
                print("\n🔍 自动扫描模式...")
                import subprocess as _sp
                _gw = _sp.Popen(["python3", os.path.expanduser("~/openclaw/go2-openclaw-skill/go2_gateway.py")],
                                stdout=_sp.DEVNULL, stderr=_sp.DEVNULL)
                time.sleep(3)
                import urllib.request as _ur
                _found = False
                for _step in range(30):
                    try:
                        _ur.urlopen("http://localhost:8520/move",
                                    data=b'{"direction":"turn_left","speed":0.2,"duration":0.5}',
                                    timeout=2)
                    except: pass
                    time.sleep(0.3)
                    # 处理一帧画面
                    frames = align.process(pipe.wait_for_frames())
                    cf, df = frames.get_color_frame(), frames.get_depth_frame()
                    if not cf or not df: continue
                    _img = np.asanyarray(cf.get_data())
                    _data = preproc(_img)
                    np.copyto(inp[0]['host'], _data.ravel())
                    cuda.memcpy_htod_async(inp[0]['device'], inp[0]['host'], stm)
                    ctx.execute_async_v2(bindings=bind, stream_handle=stm.handle)
                    cuda.memcpy_dtoh_async(out[0]['host'], out[0]['device'], stm)
                    stm.synchronize()
                    _r = out[0]['host']; _nc = len(_r)//8400
                    _preds = np.reshape(_r, (_nc, 8400)).T
                    for _row in _preds:
                        if max(_row[4:]) > 0.6:
                            _found = True
                            break
                    if _found:
                        print(f"  ✅ 检测到水瓶 (第{_step}步)")
                        break
                _gw.terminate()
                _gw.wait()
                if not _found:
                    print("  ❌ 未检测到水瓶")
                else:
                    # 停稳后重新采集一帧
                    time.sleep(1)
                    frames = align.process(pipe.wait_for_frames())
                    cf, df = frames.get_color_frame(), frames.get_depth_frame()
                    if cf and df:
                        _img2 = np.asanyarray(cf.get_data())
                        _data2 = preproc(_img2)
                        np.copyto(inp[0]['host'], _data2.ravel())
                        cuda.memcpy_htod_async(inp[0]['device'], inp[0]['host'], stm)
                        ctx.execute_async_v2(bindings=bind, stream_handle=stm.handle)
                        cuda.memcpy_dtoh_async(out[0]['host'], out[0]['device'], stm)
                        stm.synchronize()
                        _r2 = out[0]['host']; _nc2 = len(_r2)//8400
                        _preds2 = np.reshape(_r2, (_nc2, 8400)).T
                        for _row in _preds2:
                            _ss = _row[4:]; _ci = np.argmax(_ss)
                            if _ss[_ci] > 0.5:
                                _cx,_cy,_w,_h = _row[0],_row[1],_row[2],_row[3]
                                _cxp = max(0,min(int(_cx*_img2.shape[1]/640),639))
                                _cyp = max(0,min(int(_cy*_img2.shape[0]/640),479))
                                _d = df.get_distance(_cxp,_cyp)
                                if _d > 0:
                                    import pyrealsense2 as _rs
                                    _pc = _rs.rs2_deproject_pixel_to_point(intr,[_cxp,_cyp],_d)
                                    _auto_target = cam_to_arm(_pc[0],_pc[1],_pc[2])
                                    _auto_target[0] += CAM_OFFSET_X
                                    _auto_target[1] += CAM_OFFSET_Y
                                    _auto_target[2] += CAM_OFFSET_Z
                                    print(f"  🎯 目标: ({_auto_target[0]:.3f},{_auto_target[1]:.3f},{_auto_target[2]:.3f})")
                                    if ARM_REACH_MIN < _auto_target[0] < ARM_REACH_MAX and ARM_Z_MIN < _auto_target[2] < ARM_Z_MAX:
                                        _qt,_ = solve_ik_robust(_auto_target, max_iter=300, tol=5e-4)
                                        if np.linalg.norm(_auto_target - fk(_qt)[0]) < 0.02:
                                            print("  ✅ 开始抓取")
                                            # 直接跑抓取流程（不用按键）
                                            _td = np.rad2deg(_qt)
                                            _hn = np.array(HOME_DEG)
                                            _trd = _hn + (_td - _hn) * 0.4
                                            _ppd = _hn + (_td - _hn) * 0.7
                                            step_joint(HOME_DEG, GRIP_OPEN, "HOME")
                                            step_joint(list(_trd), GRIP_OPEN, "过渡")
                                            step_joint(list(_ppd), GRIP_OPEN, "预备")
                                            step_joint(list(_td), GRIP_OPEN, "目标")
                                            grip_with_verify(list(_td), GRIP_OPEN, "张开")
                                            grip_with_verify(list(_td), GRIP_CLOSE, "闭合")
                                            step_joint(list(_ppd), GRIP_CLOSE, "抬升")
                                            step_joint(HOME_DEG, GRIP_CLOSE, "HOME")
                                            print(f"\n✅ 完成!\n")
                                        else: print("  ⚠️ IK不可达")
                                    else: print("  ⚠️ 坐标超范围")
                                break
            if key == ord('g') and target_raw is not None:
                # 应用偏移
                target = target_raw.copy()
                target[0] += CAM_OFFSET_X
                target[1] += CAM_OFFSET_Y
                target[2] += CAM_OFFSET_Z

                print(f"\n{'='*55}")
                print(f"🔴 检测目标: ({target_raw[0]:.3f},{target_raw[1]:.3f},{target_raw[2]:.3f})")
                print(f"   加偏移后: ({target[0]:.3f},{target[1]:.3f},{target[2]:.3f})")

                if target[0] < ARM_REACH_MIN or target[0] > ARM_REACH_MAX:
                    print("  ⚠️ X超范围"); continue
                if target[2] < ARM_Z_MIN or target[2] > ARM_Z_MAX:
                    print("  ⚠️ Z超范围"); continue

                # IK
                q_t, ok = solve_ik_robust(target, max_iter=300, tol=5e-4)
                ik_err = np.linalg.norm(target - fk(q_t)[0])
                if ik_err > 0.02:
                    print(f"  ⚠️ IK不可达 (err={ik_err*1000:.0f}mm)"); continue
                print(f"  ✅ IK可达 (err={ik_err*1000:.0f}mm)")
                target_deg = np.rad2deg(q_t)
                home_np = np.array(HOME_DEG)
                transit_deg = home_np + (target_deg - home_np) * TRANSIT_FACTOR
                prep_deg = home_np + (target_deg - home_np) * PREP_FACTOR

                # 走到目标点
                print(f"  J2: {HOME_DEG[1]:.0f}°→{target_deg[1]:.1f}°")
                step_joint(HOME_DEG, GRIP_OPEN, "HOME")
                final_act = step_joint(list(target_deg), GRIP_OPEN, "★目标")

                # 夹爪→抬升→回家
                print(f"")
                grip_with_verify(list(target_deg), GRIP_OPEN, "张开")
                grip_with_verify(list(target_deg), GRIP_CLOSE, "闭合")
                step_joint(list(prep_deg), GRIP_CLOSE, "[4] 抬升")
                step_joint(HOME_DEG, GRIP_CLOSE, "[5] HOME")
                print(f"\n✅ 完成!\n")

            elif key == ord('g') and target_raw is None:
                print("  ⚠️ 无目标")
    finally:
        pipe.stop(); cv2.destroyAllWindows()
        print("[关闭]")

if __name__ == "__main__":
    run_verification()
