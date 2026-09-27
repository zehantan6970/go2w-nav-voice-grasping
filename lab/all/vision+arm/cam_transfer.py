#!/usr/bin/env python3
"""
cam_transfer.py — RealSense相机画面中转站

持续读取D435i，写入/tmp/cam2_feed.jpg供前端使用
独立于3dcatchv5运行，关闭3dcatch画面也不会断
"""

import pyrealsense2 as rs
import cv2
import sys
import signal
import os
import numpy as np
import time

def cleanup(s, f):
    print("\n[cam_transfer] 关闭")
    cv2.destroyAllWindows()
    sys.exit(0)

signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)

pipe = rs.pipeline()
cfg = rs.config()
# 使用第二台 D435I (第一台由 handover 使用)
cfg.enable_device("242322075785")
cfg.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 30)
try:
    pipe.start(cfg)
    print("[cam_transfer] ✅ CAM2 (242322075785) 已启动 → /tmp/cam2_feed.jpg @424x240 20fps")
except Exception as e:
    print(f"[cam_transfer] ❌ 启动失败: {e}")
    sys.exit(1)

FPS_LIMIT = 20
JPEG_QUALITY = 60
last_frame_time = 0

while True:
    try:
        frames = pipe.wait_for_frames()
        cf = frames.get_color_frame()
        if not cf:
            continue

        # FPS 限流
        now = time.monotonic()
        if now - last_frame_time < 1.0 / FPS_LIMIT:
            continue
        last_frame_time = now

        img = np.asanyarray(cf.get_data())
        # 壓縮畫質 + 原子寫入
        _, enc = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        with open("/tmp/cam2_feed_tmp.jpg", "wb") as f:
            f.write(enc.tobytes())
        os.rename("/tmp/cam2_feed_tmp.jpg", "/tmp/cam2_feed.jpg")
    except Exception as e:
        print(f"[cam_transfer] 错误: {e}")
