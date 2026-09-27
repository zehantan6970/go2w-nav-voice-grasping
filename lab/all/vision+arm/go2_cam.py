#!/usr/bin/env python3
"""
go2_cam.py — Go2机器人前摄像头中转站

持续读取Go2前端相机画面，写入/tmp/cam1_feed.jpg供前端使用
"""
import cv2, numpy as np, sys, signal, os, time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient

def cleanup(s, f):
    print("\n[go2_cam] 关闭")
    cv2.destroyAllWindows()
    sys.exit(0)

signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)

if len(sys.argv) > 1:
    ChannelFactoryInitialize(0, sys.argv[1])
else:
    ChannelFactoryInitialize(0)
client = VideoClient()
client.SetTimeout(3.0)
client.Init()

print(f"[go2_cam] ✅ Go2前摄像头已连接 → /tmp/cam1_feed.jpg")

last_frame = 0
while True:
    try:
        now = time.time()
        if now - last_frame < 0.1:  # 10fps
            time.sleep(0.02)
            continue
        code, data = client.GetImageSample()
        if code == 0 and data:
            img = cv2.imdecode(np.frombuffer(bytes(data), dtype=np.uint8), cv2.IMREAD_COLOR)
            if img is not None:
                img = cv2.resize(img, (640, 360))
                cv2.imwrite("/tmp/cam1_feed.jpg", img)
                last_frame = time.time()
    except Exception as e:
        print(f"[go2_cam] 错误: {e}")
