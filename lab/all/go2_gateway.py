
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 先修正 CycloneDDS 庫路徑（Jetson ARM64 需要本地編譯版）
import os
os.environ.setdefault('LD_LIBRARY_PATH',
    '/home/unitree/cyclonedds/install/lib:' + os.environ.get('LD_LIBRARY_PATH', ''))
os.environ.setdefault('CYCLONEDDS_HOME', '/home/unitree/cyclonedds/install')

"""
Go2-W OpenClaw 网关 - 真正控制版本 (Go2-W 专用)
Go2-W 轮式机器狗支持的功能:
  - StandUp / StandDown / BalanceStand / Damp
  - Move (前进/后退/左平移/右平移/左转/右转)
  - StopMove / Euler / SpeedLevel / SwitchJoystick
不支持: 倒立、跳舞、空翻等腿式动作
"""

import json
import sys
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse

# 尝试导入 unitree_sdk2py
UNITREE_SDK_AVAILABLE = False
possible_paths = [
    '/home/unitree/unitree_sdk2_python',
    '/unitree/module/unitree_sdk2/lib/python3.8/site-packages',
    '/usr/local/lib/python3.8/site-packages',
    '/usr/lib/python3/dist-packages',
]

for p in possible_paths:
    if p not in sys.path and p != '':
        sys.path.insert(0, p)

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    UNITREE_SDK_AVAILABLE = True
    print("[OK] unitree_sdk2py 导入成功")
except ImportError as e:
    print(f"[!] unitree_sdk2py 不可用: {e}")
    print("[!] 网关以模拟模式运行")

MAX_SPEED = 0.5
DEFAULT_SPEED = 0.3

class Go2Controller:
    def __init__(self):
        self.initialized = False
        self.battery = 85
        self.mode = "idle"
        self._sport_client = None
        
    def init_dds(self):
        if not UNITREE_SDK_AVAILABLE:
            print("[模拟模式] SDK 不可用")
            self.initialized = True
            return True
        try:
            ChannelFactoryInitialize(0, "eth0")
            self._sport_client = SportClient()
            self._sport_client.SetTimeout(5.0)
            self._sport_client.Init()
            self.initialized = True
            print("[OK] DDS 初始化成功，SportClient 已创建")
            return True
        except Exception as e:
            print(f"[ERR] DDS 初始化失败: {e}")
            return False
    
    # ========== 基础姿态 ==========
    def stand_up(self):
        print("[动作] >>> 站立 (StandUp)")
        if self._sport_client:
            code = self._sport_client.StandUp()
            return {"success": code == 0, "action": "stand_up", "code": code}
        return {"success": True, "action": "stand_up", "mode": "simulated"}
    
    def stand_down(self):
        print("[动作] >>> 趴下 (StandDown)")
        if self._sport_client:
            code = self._sport_client.StandDown()
            return {"success": code == 0, "action": "stand_down", "code": code}
        return {"success": True, "action": "stand_down", "mode": "simulated"}
    
    def balance_stand(self):
        print("[动作] >>> 平衡站立 (BalanceStand)")
        if self._sport_client:
            code = self._sport_client.BalanceStand()
            return {"success": code == 0, "action": "balance_stand", "code": code}
        return {"success": True, "action": "balance_stand", "mode": "simulated"}
    
    def damp(self):
        print("[动作] >>> 阻尼模式 (Damp)")
        if self._sport_client:
            code = self._sport_client.Damp()
            return {"success": code == 0, "action": "damp", "code": code}
        return {"success": True, "action": "damp", "mode": "simulated"}
    
    # ========== 移动控制 ==========
    def move(self, direction, duration, speed):
        speed = min(float(speed), MAX_SPEED)
        print(f"[动作] >>> Move {direction} 持续{duration}秒, 速度={speed}")
        if self._sport_client:
            vx, vy, vyaw = 0.0, 0.0, 0.0
            if direction == "forward":
                vx = speed
            elif direction == "backward":
                vx = -speed
            elif direction == "left":
                vy = speed
            elif direction == "right":
                vy = -speed
            elif direction == "turn_left":
                vyaw = speed
            elif direction == "turn_right":
                vyaw = -speed
            else:
                return {"success": False, "error": f"未知方向: {direction}"}
            
            code = self._sport_client.Move(vx, vy, vyaw)
            time.sleep(float(duration))
            self._sport_client.Move(0, 0, 0)
            return {"success": code == 0, "direction": direction, "duration": float(duration), "speed": speed, "code": code}
        return {"success": True, "direction": direction, "duration": float(duration), "speed": speed, "mode": "simulated"}
    
    def stop(self):
        print("[动作] >>> 急停 (StopMove)")
        if self._sport_client:
            code = self._sport_client.StopMove()
            self._sport_client.Move(0, 0, 0)
            return {"success": code == 0, "action": "stop", "code": code}
        return {"success": True, "action": "stop", "mode": "simulated"}
    
    # ========== 姿态调整 ==========
    def euler(self, roll, pitch, yaw):
        print(f"[动作] >>> 调整姿态 Euler({roll}, {pitch}, {yaw})")
        if self._sport_client:
            code = self._sport_client.Euler(float(roll), float(pitch), float(yaw))
            return {"success": code == 0, "action": "euler", "roll": roll, "pitch": pitch, "yaw": yaw, "code": code}
        return {"success": True, "action": "euler", "roll": roll, "pitch": pitch, "yaw": yaw, "mode": "simulated"}
    
    # ========== 速度档位 ==========
    def speed_level(self, level):
        print(f"[动作] >>> 设置速度档位 {level}")
        if self._sport_client:
            code = self._sport_client.SpeedLevel(int(level))
            return {"success": code == 0, "action": "speed_level", "level": level, "code": code}
        return {"success": True, "action": "speed_level", "level": level, "mode": "simulated"}
    
    # ========== 障碍物规避 ==========
    def obstacles_avoid(self, on):
        print(f"[动作] >>> 障碍物规避 {'开启' if on else '关闭'}")
        if UNITREE_SDK_AVAILABLE:
            from unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client import ObstaclesAvoidClient
            client = ObstaclesAvoidClient()
            client.SetTimeout(3.0)
            client.Init()
            code = client.SwitchSet(bool(on))
            return {"success": code == 0, "action": "obstacles_avoid", "on": on, "code": code}
        return {"success": True, "action": "obstacles_avoid", "on": on, "mode": "simulated"}

    # ========== 摇杆切换 ==========
    def switch_joystick(self, on):
        print(f"[动作] >>> 摇杆控制 {'开启' if on else '关闭'}")
        if self._sport_client:
            code = self._sport_client.SwitchJoystick(bool(on))
            return {"success": code == 0, "action": "switch_joystick", "on": on, "code": code}
        return {"success": True, "action": "switch_joystick", "on": on, "mode": "simulated"}
    
    def get_status(self):
        return {
            "robot": "Unitree Go2-W EDU U3",
            "battery": self.battery,
            "mode": self.mode,
            "dds_connected": self.initialized,
            "sdk_available": UNITREE_SDK_AVAILABLE
        }

controller = Go2Controller()

class RequestHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        print(f"[HTTP] {self.client_address[0]} - {format % args}")
    
    def _send_json(self, data, status=200):
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(json.dumps(data, ensure_ascii=False).encode('utf-8'))
    
    def do_GET(self):
        path = urlparse(self.path).path
        if path == '/':
            self._send_json({
                "service": "Go2-W OpenClaw Gateway",
                "version": "1.1.0",
                "robot": "Go2-W (轮式)",
                "endpoints": ["/status", "/battery", "/action", "/move", "/stop", "/euler", "/speed", "/joystick", "/obstacles"]
            })
        elif path == '/status':
            self._send_json(controller.get_status())
        elif path == '/battery':
            self._send_json({"battery": controller.battery})
        else:
            self._send_json({"error": "未知端点"}, 404)
    
    def do_POST(self):
        path = urlparse(self.path).path
        content_length = int(self.headers.get('Content-Length', 0))
        try:
            data = json.loads(self.rfile.read(content_length).decode('utf-8')) if content_length > 0 else {}
        except json.JSONDecodeError:
            self._send_json({"error": "JSON 解析失败"}, 400)
            return
        
        if path == '/action':
            action = data.get('action', '')
            if action == 'stand_up':
                result = controller.stand_up()
            elif action == 'stand_down':
                result = controller.stand_down()
            elif action == 'balance_stand':
                result = controller.balance_stand()
            elif action == 'damp':
                result = controller.damp()
            elif action == 'stop':
                result = controller.stop()
            else:
                self._send_json({"error": f"未知动作: {action}", 
                    "supported": ["stand_up", "stand_down", "balance_stand", "damp", "stop"],
                    "note": "Go2-W 轮式版不支持倒立/跳舞/空翻等腿式动作"}, 400)
                return
            self._send_json(result)
        
        elif path == '/move':
            result = controller.move(
                data.get('direction', 'forward'),
                data.get('duration', 1.0),
                data.get('speed', DEFAULT_SPEED)
            )
            self._send_json(result)
        
        elif path == '/stop':
            self._send_json(controller.stop())
        
        elif path == '/euler':
            result = controller.euler(
                data.get('roll', 0.0),
                data.get('pitch', 0.0),
                data.get('yaw', 0.0)
            )
            self._send_json(result)
        
        elif path == '/speed':
            result = controller.speed_level(data.get('level', 1))
            self._send_json(result)
        
        elif path == '/obstacles':
            result = controller.obstacles_avoid(data.get('on', False))
            self._send_json(result)
        
        elif path == '/joystick':
            result = controller.switch_joystick(data.get('on', True))
            self._send_json(result)
        
        else:
            self._send_json({"error": "未知端点"}, 404)

def main():
    print("=" * 60)
    print(" Go2-W OpenClaw 网关")
    print(" 轮式机器狗专用 | 真实 SDK 控制")
    print("=" * 60)
    
    controller.init_dds()
    
    host, port = '0.0.0.0', 8520
    server = HTTPServer((host, port), RequestHandler)
    
    print(f"\n[OK] 网关已启动: http://{host}:{port}")
    print("\n--- Go2-W 支持的功能 ---")
    print("姿态: stand_up | stand_down | balance_stand | damp")
    print("移动: forward | backward | left | right | turn_left | turn_right")
    print("其他: stop | euler | speed | joystick")
    print("\n[按 Ctrl+C 停止]\n")
    
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[!] 停止中...")
        server.shutdown()
        print("[OK] 网关已关闭")

if __name__ == '__main__':
    main()
GATEWAY_EOF
