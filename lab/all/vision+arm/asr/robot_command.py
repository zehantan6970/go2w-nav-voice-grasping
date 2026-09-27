#!/usr/bin/env python3
"""
機器狗直接動作執行器
直接使用 Unitree SDK SportClient 控制，不經 OpenClaw 自然語言處理。

用法：
  python3 robot_command.py forward     # 前進 1s
  python3 robot_command.py backward    # 後退 1s
  python3 robot_command.py left        # 左轉 1s
  python3 robot_command.py right       # 右轉 1s
  python3 robot_command.py stop        # 停止移動
  python3 robot_command.py sit         # 坐下
  python3 robot_command.py stand       # 站起（平衡站立）
  python3 robot_command.py hello       # 打招呼
  python3 robot_command.py heart       # 比心
  python3 robot_command.py dance       # 跳舞
  python3 robot_command.py arm_home    # 機械臂歸位 HOME
"""

import sys
import time
import os

# 設定 DDS 庫路徑
os.environ.setdefault("LD_LIBRARY_PATH", "/home/unitree/cyclonedds/install/lib")

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

MOVE_DURATION = 1.0   # 前進/後退/轉彎持續秒數
MOVE_SPEED = 0.3      # 移動速度 m/s
TURN_SPEED = 0.5      # 旋轉速度 rad/s


def get_client() -> SportClient:
    ChannelFactoryInitialize(0, "eth0")
    client = SportClient()
    client.SetTimeout(10.0)
    client.Init()
    return client


def main():
    if len(sys.argv) < 2:
        print("用法: python3 robot_command.py <action>")
        sys.exit(1)

    action = sys.argv[1]
    client = get_client()

    if action == "forward":
        print("[Move] 前進")
        client.Move(MOVE_SPEED, 0, 0)
        time.sleep(MOVE_DURATION)
        client.StopMove()

    elif action == "backward":
        print("[Move] 後退")
        client.Move(-MOVE_SPEED, 0, 0)
        time.sleep(MOVE_DURATION)
        client.StopMove()

    elif action == "left":
        print("[Move] 左轉")
        client.Move(0, 0, TURN_SPEED)
        time.sleep(MOVE_DURATION)
        client.StopMove()

    elif action == "right":
        print("[Move] 右轉")
        client.Move(0, 0, -TURN_SPEED)
        time.sleep(MOVE_DURATION)
        client.StopMove()

    elif action == "stop":
        print("[Move] 停止")
        client.StopMove()

    elif action == "sit":
        print("[Move] 坐下")
        client.Sit()

    elif action == "stand":
        print("[Move] 站起")
        client.BalanceStand()

    elif action == "hello":
        print("[Move] 打招呼")
        client.Hello()

    elif action == "heart":
        print("[Move] 比心")
        client.Heart()

    elif action == "dance":
        print("[Move] 跳舞")
        client.Dance1()
        time.sleep(3)
        client.StopMove()

    elif action == "arm_home":
        # 機械臂歸位：不需 SportClient，直接呼叫 arm_home.py
        print("[Arm] 機械臂歸位 HOME...")
        ret = os.system(
            "LD_LIBRARY_PATH=/home/unitree/cyclonedds/install/lib:/usr/local/lib "
            "python3 /home/unitree/lab/all/vision+arm/arm_home.py"
        )
        if ret == 0:
            print("[Arm] 歸位完成")
        else:
            print(f"[Arm] 歸位異常 (返回碼:{ret})")

    else:
        print(f"未知動作: {action}")
        sys.exit(1)

    print(f"[Move] {action} 完成")


if __name__ == "__main__":
    main()
