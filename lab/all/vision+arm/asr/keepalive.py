#!/usr/bin/env python3
"""
Go2-W Keepalive — 每 8 秒發送 BalanceStand 防止機器人進入鎖定姿態
由 systemd 管理（unitree-keepalive.service），開機自動啟動。
"""

import os
import time

os.environ.setdefault("LD_LIBRARY_PATH", "/home/unitree/cyclonedds/install/lib")
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient


def main():
    ChannelFactoryInitialize(0, "eth0")
    c = SportClient()
    c.SetTimeout(3.0)
    c.Init()
    print("🟢 Keepalive 啟動（每 8 秒 BalanceStand）")
    while True:
        try:
            c.BalanceStand()
            time.sleep(0.2)
            c.StopMove()
        except Exception as e:
            print(f"⚠️  Keepalive 錯誤: {e}")
        time.sleep(8)


if __name__ == "__main__":
    main()
