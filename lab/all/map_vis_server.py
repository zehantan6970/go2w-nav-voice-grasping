#!/usr/bin/env python3
"""
地圖視覺化伺服器 — 輕量版
提供 HTTP 服務，讓瀏覽器顯示地圖和機器人即時位置。

服務：
- 靜態檔案（index.html, map.png 等）
- 透過 symlink 提供 /robot_pose.json

用法（或透過 systemd 自動啟動）：
  python3 map_vis_server.py
"""

import http.server
import os
import socketserver
import sys

PORT = 8080
WWW_DIR = "/home/unitree/lab/all"


class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=WWW_DIR, **kwargs)

    def log_message(self, fmt, *args):
        pass  # 安靜


def main():
    # 確保 symlink 存在
    pose_src = "/tmp/robot_pose.json"
    pose_link = os.path.join(WWW_DIR, "robot_pose.json")
    if not os.path.exists(pose_link):
        try:
            os.symlink(pose_src, pose_link)
        except:
            pass

    httpd = socketserver.TCPServer(("0.0.0.0", PORT), Handler)
    print(f"🗺️  地圖視覺化: http://0.0.0.0:{PORT}/")
    print(f"   index.html: http://0.0.0.0:{PORT}/index.html")
    print(f"   pose data:  http://0.0.0.0:{PORT}/robot_pose.json")
    httpd.serve_forever()


if __name__ == "__main__":
    main()
