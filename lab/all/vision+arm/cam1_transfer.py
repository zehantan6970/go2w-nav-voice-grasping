#!/usr/bin/env python3
"""
cam1_transfer.py — 第二台 D435I 画面中转站 (S/N: 244222075086)

写入 /tmp/cam1_feed.jpg 供前端 CAM1 显示
独立于 3dcatchv5 运行，640×360，10fps
"""

import pyrealsense2 as rs
import cv2
import sys
import signal
import os
import numpy as np
import time

def cleanup(s, f):
    print("\n[cam1_transfer] 关闭")
    cv2.destroyAllWindows()
    sys.exit(0)

signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)

# 用 Serial Number 指定第二台 D435I
ctx = rs.context()
devs = ctx.query_devices()
target_sn = "244222075086"
selected = None
for d in devs:
    sn = d.get_info(rs.camera_info.serial_number)
    if sn == target_sn:
        selected = d
        break

if selected is None:
    print(f"[cam1_transfer] ❌ 找不到 S/N 為 {target_sn} 的相機")
    print(f"[cam1_transfer] 可用相機:")
    for d in devs:
        print(f"  {d.get_info(rs.camera_info.name)} S/N: {d.get_info(rs.camera_info.serial_number)}")
    sys.exit(1)

print(f"[cam1_transfer] ✅ 找到相機: {selected.get_info(rs.camera_info.name)} S/N: {target_sn}")

cfg = rs.config()
cfg.enable_device(target_sn)
cfg.enable_stream(rs.stream.color, 640, 360, rs.format.bgr8, 30)

pipe = rs.pipeline()
try:
    pipe.start(cfg)
    print("[cam1_transfer] ✅ 已啟動 → /tmp/cam1_feed.jpg @640x360 20fps")
except Exception as e:
    print(f"[cam1_transfer] ❌ 啟動失敗: {e}")
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
        with open("/tmp/cam1_feed_tmp.jpg", "wb") as f:
            f.write(enc.tobytes())
        os.rename("/tmp/cam1_feed_tmp.jpg", "/tmp/cam1_feed.jpg")
    except Exception as e:
        print(f"[cam1_transfer] 錯誤: {e}")
