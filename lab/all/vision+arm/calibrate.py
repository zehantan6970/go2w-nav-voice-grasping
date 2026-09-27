#!/usr/bin/env python3
"""校准工具: 调CAM_OFFSET直到坐标对准真实水瓶"""
import numpy as np, sys, os, cv2
if not hasattr(np, 'bool'): np.bool = bool

# ═══ 在这里调 ═══
OX = 0.04   # X偏移(正=前)
OY = -0.01  # Y偏移(正=左)
OZ = -0.02  # Z偏移(正=上)
# ═══════════════

import pyrealsense2 as rs, tensorrt as trt, pycuda.driver as cuda, pycuda.autoinit

CAM_BASE_POS = np.array([0.06, 0.0, 0.35])
CAM_BASE_ROT = np.array([[0,0,1],[-1,0,0],[0,-1,0]], dtype=float)
def cam_to_arm(x,y,z):
    return CAM_BASE_ROT@np.array([x,y,z])+CAM_BASE_POS

ep = os.path.join(os.path.dirname(__file__),"bottle.engine")
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

print("\n=== 校准模式 ===")
print("调3dcatchv3.py顶部的 OX, OY, OZ")
print("直到画面坐标 = 水瓶实际位置")
print("画面显示: 原始坐标 → 加偏移后坐标")
print("按 q 退出\n")

try:
    while True:
        frames = align.process(pipe.wait_for_frames())
        cf,df = frames.get_color_frame(),frames.get_depth_frame()
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
        for row in preds:
            ss = row[4:]; ci = np.argmax(ss)
            if ss[ci] > 0.5:
                cx,cy,w,h = row[0],row[1],row[2],row[3]
                xf,yf = img.shape[1]/640, img.shape[0]/640
                x1,y1 = int((cx-w/2)*xf), int((cy-h/2)*yf)
                x2,y2 = int((cx+w/2)*xf), int((cy+h/2)*yf)
                cv2.rectangle(img,(x1,y1),(x2,y2),(0,255,0),2)
                cxp = max(0,min(int(cx*xf),639))
                cyp = max(0,min(int(cy*yf),479))
                d = df.get_distance(cxp,cyp)
                if d > 0:
                    pc = rs.rs2_deproject_pixel_to_point(intr,[cxp,cyp],d)
                    raw_arm = cam_to_arm(pc[0],pc[1],pc[2])
                    adj = [raw_arm[0]+OX, raw_arm[1]+OY, raw_arm[2]+OZ]
                    cv2.putText(img,f"原始: X={raw_arm[0]*100:.0f} Y={raw_arm[1]*100:.0f} Z={raw_arm[2]*100:.0f}cm",
                                (x1,y1-30),cv2.FONT_HERSHEY_SIMPLEX,0.45,(0,0,255),2)
                    cv2.putText(img,f"偏移: X={adj[0]*100:.0f} Y={adj[1]*100:.0f} Z={adj[2]*100:.0f}cm",
                                (x1,y1-10),cv2.FONT_HERSHEY_SIMPLEX,0.45,(255,100,0),2)
        cv2.imshow("校准 - 调OX OY OZ直到对准", img)
        if cv2.waitKey(1)&0xFF == ord('q'): break
finally:
    pipe.stop(); cv2.destroyAllWindows()
