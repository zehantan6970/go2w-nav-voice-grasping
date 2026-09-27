# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import time

# 注意：请确保你的环境中已经编译了 unitree_go 相关的 ROS2 消息包
# 如果报错找不到 unitree_go，请检查 ROS2 环境变量是否 source 正确
try:
    from unitree_go.msg import LowState
except ImportError:
    print("[Error] 无法导入 unitree_go 消息，请检查 ROS2 环境变量")
    LowState = None

class BridgeNode(Node):
    """基础 ROS2 节点类"""
    def __init__(self, name):
        super().__init__(name)

def ros_bridge_compressed(shared_frames, topic_index, topic_name, key):
    """
    订阅 ROS2 图像话题，并将 bytes 数据存入共享字典
    """
    # 在 rclpy.init() 之前设置 ROS_DOMAIN_ID
    import os
    os.environ['ROS_DOMAIN_ID'] = str(topic_index)
    
    rclpy.init()
    node = BridgeNode(f'node_cam_{key}')
    
    def callback(msg):
        shared_frames[key] = msg.data

    node.create_subscription(CompressedImage, topic_name, callback, 10)
    
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"[Bridge] Camera Bridge Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

def ros_bridge_telemetry(shared_telemetry):
    """
    订阅 ROS2 /lowstate 话题，并将遥测数据格式化后存入共享字典 (每 0.5s 更新一次)
    """
    rclpy.init()
    node = BridgeNode('node_telemetry_bridge')
    
    # 限流控制
    last_update_time = [0.0]
    THROTTLE_INTERVAL = 0.5

    def lowstate_callback(msg):
        now = time.time()
        # 频率限制：检查是否达到 0.5 秒
        if (now - last_update_time[0]) < THROTTLE_INTERVAL:
            return
        last_update_time[0] = now

        try:
            # 构建前端需要的数据结构
            # 必须使用 float() 或 int() 将 ROS2 类型强制转换为 Python 标准类型
            telemetry = {
                "power_v": float(getattr(msg, 'power_v', 0.0)),
                "power_a": float(getattr(msg, 'power_a', 0.0)),
                "bms_state": {
                    "soc": int(msg.bms_state.soc) if hasattr(msg, 'bms_state') else 0
                },
                "temperature_ntc1": float(getattr(msg, 'temperature_ntc1', 0.0)),
                "temperature_ntc2": float(getattr(msg, 'temperature_ntc2', 0.0)),
                "tick": int(getattr(msg, 'tick', 0)),
                "imu_state": {
                    "quaternion": [float(x) for x in msg.imu_state.quaternion] if hasattr(msg, 'imu_state') else [0.0,0.0,0.0,0.0],
                    "gyroscope": [float(x) for x in msg.imu_state.gyroscope] if hasattr(msg, 'imu_state') else [0.0,0.0,0.0],
                    "accelerometer": [float(x) for x in msg.imu_state.accelerometer] if hasattr(msg, 'imu_state') else [0.0,0.0,0.0]
                },
                "motor_state": []
            }

            # 填充电机数据
            if hasattr(msg, 'motor_state'):
                for m in msg.motor_state:
                    telemetry["motor_state"].append({
                        "lost": int(getattr(m, 'lost', 0)),
                        "q": float(getattr(m, 'q', 0.0)),
                        "dq": float(getattr(m, 'dq', 0.0)),
                        "tau_est": float(getattr(m, 'tau_est', 0.0)),
                        "temperature": int(getattr(m, 'temperature', 0))
                    })
            
            shared_telemetry.update(telemetry)

        except Exception as e:
            print(f"[Bridge] Telemetry parsing error: {e}")

    # 订阅低位状态话题
    node.create_subscription(LowState, '/lowstate', lowstate_callback, 10)
    
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"[Bridge] Telemetry Bridge Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()