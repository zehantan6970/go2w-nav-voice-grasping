#!/usr/bin/env python3
"""
SLAM DDS → ROS2 Topic Bridge (via ros2 topic pub CLI)
不使用 rclpy，透過 subprocess 呼叫 ros2 topic pub 發布位姿。

用法：
  python3 /home/unitree/lab/all/slam_ros_bridge.py
"""

import json
import os
import subprocess
import sys
import time

os.environ.setdefault("LD_LIBRARY_PATH", "/home/unitree/cyclonedds/install/lib")
os.environ.setdefault("ROS_DOMAIN_ID", "0")
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber

# 訂閱 SLAM 位姿 Topic
pose_data = {"x": 0, "y": 0, "z": 0, "qx": 0, "qy": 0, "qz": 0, "qw": 1}

def slam_handler(msg):
    global pose_data
    try:
        data = json.loads(str(msg))
        p = data.get("data", {}).get("currentPose", {})
        if p:
            pose_data = {
                "x": p.get("x", 0), "y": p.get("y", 0), "z": p.get("z", 0),
                "qx": p.get("q_x", 0), "qy": p.get("q_y", 0),
                "qz": p.get("q_z", 0), "qw": p.get("q_w", 1),
            }
    except:
        pass

def publish_via_ros2():
    """每隔 0.5 秒透過 ros2 topic pub 發布 Odometry"""
    source_cmd = "source /opt/ros/foxy/setup.bash"
    while True:
        p = pose_data
        # 建立 YAML 格式的 Odometry 消息
        yaml_msg = (
            f"header:\\n  frame_id: map\\nchild_frame_id: base_link\\n"
            f"pose:\\n  pose:\\n    position:\\n      x: {p['x']}\\n      y: {p['y']}\\n      z: {p['z']}\\n"
            f"    orientation:\\n      x: {p['qx']}\\n      y: {p['qy']}\\n      z: {p['qz']}\\n      w: {p['qw']}"
        )
        cmd = f'bash -c \'{source_cmd} && ros2 topic pub /unitree/slam_relocation/odom nav_msgs/msg/Odometry "{yaml_msg}" --once 2>/dev/null\''
        subprocess.Popen(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(0.5)


def write_pose_to_json():
    """每隔 0.2 秒寫入 JSON 檔，給 demo2 修改或網頁讀取"""
    while True:
        p = pose_data
        with open("/tmp/robot_pose.json", "w") as f:
            json.dump({
                "x": p["x"], "y": p["y"], "z": p["z"],
                "q_x": p["qx"], "q_y": p["qy"], "q_z": p["qz"], "q_w": p["qw"]
            }, f)
        time.sleep(0.2)


def main():
    print("🚀 SLAM→ROS2 橋接啟動")
    import threading

    ChannelFactoryInitialize(0, "eth0")
    sub = ChannelSubscriber("rt/slam_info", 0, slam_handler)
    sub.Init()
    print("✅ 已訂閱 rt/slam_info")

    # 啟動發布執行緒
    threading.Thread(target=publish_via_ros2, daemon=True).start()
    threading.Thread(target=write_pose_to_json, daemon=True).start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("停止")

if __name__ == "__main__":
    main()
