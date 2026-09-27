#!/usr/bin/env python3
"""
handover_main.py — 手势导引机械狗 (手指骨骼版)

流程:
  1. CAM1 實時檢測手掌中心 (指尖→骨骼→掌心中點)
  2. 3D 定位 → 控制 Go2 移向手掌
  3. 到達 0.4m 停止

使用:
  python3 handover_main.py                 # 自動模式
  python3 handover_main.py --show          # 顯示 CAM1 畫面
  python3 handover_main.py --no-move       # 僅檢測不移動 (測試)
  python3 handover_main.py --keep-cam2     # 不自動停用其他相機
"""

import cv2
import sys
import time
import argparse
import subprocess
import numpy as np
import os
from hand_tracker import HandTracker
from go2_dog import Go2Dog


MOVE_INTERVAL = 0.05
MISSED_THRESHOLD = 5

CAM_TRANSFER_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "cam_transfer.py")


def stop_cam_transfer():
    try:
        subprocess.run(["pkill", "-f", "cam_transfer.py"],
                       timeout=3, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(0.3)
        print("[HandOver] ⏸️ 已停用 cam_transfer")
    except Exception:
        pass


def start_cam_transfer():
    try:
        proc = subprocess.Popen(
            ["python3", CAM_TRANSFER_PATH],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print(f"[HandOver] ▶️ 已重启 cam_transfer (PID: {proc.pid})")
        return proc
    except Exception as e:
        print(f"[HandOver] ⚠️ 重启cam_transfer失败: {e}")
        return None


def parse_args():
    p = argparse.ArgumentParser(description="手势导引机械狗 (手指骨骼)")
    p.add_argument("--show", action="store_true", help="显示 CAM1 实时画面")
    p.add_argument("--no-move", action="store_true", help="仅检测不移动 (测试模式)")
    p.add_argument("--keep-cam2", action="store_true", help="不自动停用其他相机")
    return p.parse_args()


def main():
    args = parse_args()
    print("=" * 50)
    print("  🐕‍🦺 HandOver - 手指骨骼导引")
    print("=" * 50)
    print("  手伸到CAM1前 → 指尖骨骼 → 掌心中點")
    print("  左/右 → 机器狗转向")
    print("  远/近 → 机器狗前进/后退")
    print("  到达约 0.4m 时停止")
    print("  ESC 退出")
    print("=" * 50)

    if not args.keep_cam2:
        stop_cam_transfer()
        time.sleep(0.5)

    tracker = HandTracker()
    dog = Go2Dog()
    move_enabled = not args.no_move

    missed_count = 0
    last_cmd_time = 0
    last_action = ""

    try:
        while True:
            color, aligned_depth, raw = tracker.get_frames()
            if color is None:
                time.sleep(0.1)
                continue

            fingers, palm, nd = tracker.detect_hand(color, aligned_depth)
            now = time.time()
            pos3d = None

            if palm:
                cx, cy = palm
                pos3d = tracker.get_3d_position(cx, cy, aligned_depth)

                if pos3d:
                    x3d, y3d, z3d = pos3d
                    missed_count = 0
                    nf = len(fingers)

                    print(f"[Hand] PALM({cx:3d},{cy:3d}) "
                          f"3D=({x3d:+.2f}, {y3d:+.2f}, {z3d:.2f})m "
                          f"Fingers={nf} Defects={nd}")

                    if move_enabled and (now - last_cmd_time >= MOVE_INTERVAL):
                        action = dog.move_toward(x3d, z3d)
                        if action != last_action:
                            print(f"  → {action}")
                            last_action = action
                        last_cmd_time = now
                else:
                    if missed_count < MISSED_THRESHOLD:
                        missed_count += 1
                    else:
                        if move_enabled:
                            dog.stop()
                        print("[Hand] ⚠️ 深度无效")
            else:
                missed_count += 1
                if missed_count >= MISSED_THRESHOLD:
                    if move_enabled and last_action != "停止":
                        dog.stop()
                        print("[Hand] ❌ 手部丢失，停止")
                        last_action = "停止"

            if args.show:
                if palm:
                    display = tracker.draw_hand(color, fingers, palm)
                    if pos3d:
                        x, y, z = pos3d
                        cv2.putText(display, f"3D: ({x:.2f}, {y:.2f}, {z:.2f})m",
                                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                    info = last_action if last_action else "Wait..."
                    cv2.putText(display, info, (10, 55),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                else:
                    display = color.copy()
                    cv2.putText(display, "No hand", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.imshow("HandOver - Finger Bone", display)
                if cv2.waitKey(1) & 0xFF == 27:
                    break

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[HandOver] 用户中断")
    finally:
        if move_enabled:
            dog.stop()
        tracker.stop()
        cv2.destroyAllWindows()
        if not args.keep_cam2:
            start_cam_transfer()
        print("[HandOver] 已退出")


if __name__ == "__main__":
    main()
