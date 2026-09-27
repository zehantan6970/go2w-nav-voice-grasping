#!/usr/bin/env python3
"""
scan_cam.py — 扫描机器狗身上所有镜头

检测所有可用相机并保存一张样本画面到 /tmp/
"""

import cv2, numpy as np, sys, os

print("="*50)
print("📷 扫描机器狗身上所有镜头...")
print("="*50)

found = 0

# 1. 通过 OpenCV 扫描所有 /dev/video*
print("\n[1] OpenCV video设备扫描:")
for i in range(10):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        ret, frame = cap.read()
        if ret and frame is not None:
            found += 1
            h, w = frame.shape[:2]
            cv2.imwrite(f"/tmp/cam_opencv_{i}.jpg", frame)
            print(f"  ✅ /dev/video{i}: {w}x{h} → /tmp/cam_opencv_{i}.jpg")
        else:
            print(f"  ⚠️ /dev/video{i}: 打开但无法读取")
        cap.release()
    else:
        if i == 0: print("  (无OpenCV可访问的摄像头)")

# 2. RealSense深度相机
print("\n[2] RealSense深度相机:")
try:
    import pyrealsense2 as rs
    ctx = rs.context()
    devices = ctx.query_devices()
    if len(devices) == 0:
        print("  (无RealSense设备)")
    else:
        for i, d in enumerate(devices):
            name = d.get_info(rs.camera_info.name)
            sn = d.get_info(rs.camera_info.serial_number)
            print(f"  ✅ [{i}] {name} S/N: {sn}")
            found += 1
            # 取一帧保存
            pipe = rs.pipeline()
            cfg = rs.config()
            cfg.enable_device(sn)
            cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            try:
                pipe.start(cfg)
                frames = pipe.wait_for_frames()
                cf = frames.get_color_frame()
                if cf:
                    img = np.asanyarray(cf.get_data())
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                    cv2.imwrite(f"/tmp/cam_realsense_{i}.jpg", img)
                    print(f"     → /tmp/cam_realsense_{i}.jpg ({img.shape[1]}x{img.shape[0]})")
                pipe.stop()
            except Exception as e:
                print(f"     ❌ 取帧失败: {e}")
except ImportError:
    print("  (pyrealsense2 未安装)")
except Exception as e:
    print(f"  ❌ 错误: {e}")

# 3. Go2机器人前端相机 (via unitree SDK)
print("\n[3] Go2机器人前摄像头 (unitree SDK):")
try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.video.video_client import VideoClient
    
    ChannelFactoryInitialize(0)
    client = VideoClient()
    client.SetTimeout(3.0)
    client.Init()
    code, data = client.GetImageSample()
    if code == 0 and data:
        img = cv2.imdecode(np.frombuffer(bytes(data), dtype=np.uint8), cv2.IMREAD_COLOR)
        if img is not None:
            found += 1
            cv2.imwrite("/tmp/cam_go2_front.jpg", img)
            print(f"  ✅ 可用: {img.shape[1]}x{img.shape[0]} → /tmp/cam_go2_front.jpg")
        else:
            print("  ⚠️ 解码失败")
    else:
        print(f"  ❌ 连接失败 (code={code})")
except ImportError:
    print("  (unitree SDK 未安装)")
except Exception as e:
    print(f"  ❌ 错误: {e}")

print(f"\n{'='*50}")
print(f"📊 共找到 {found} 个镜头")
print(f"   样本保存在 /tmp/cam_*.jpg")
print(f"{'='*50}")
