#!/usr/bin/env python3
"""
go2_dog.py — 机器狗运动控制 API (快速响应版)

通过 HTTP 调用本地网关 go2_gateway (port 8520)
控制 Go2-W 机器狗移动

快速响应特性:
  - 每 0.12s 发一次短脉冲 (0.2s 持续时间)
  - 死区更小 (0.05m), 轻微偏移即响应
  - 速度曲线更激进, 越远越快
  - 转向优先于前进
"""

import json
import urllib.request
import urllib.error

GATEWAY_URL = "http://localhost:8520"
DEFAULT_SPEED = 0.4
DEFAULT_DURATION = 0.2


class Go2Dog:
    def __init__(self, url=GATEWAY_URL):
        self.url = url
        self._check_gateway()

    def _check_gateway(self):
        """检查网关是否在线"""
        try:
            req = urllib.request.Request(f"{self.url}/", method="GET")
            with urllib.request.urlopen(req, timeout=1):
                pass
            print("[Go2Dog] ✅ 网关在线")
        except Exception:
            print("[Go2Dog] ⚠️ 网关未启动，运动功能不可用")
            print("[Go2Dog]    请运行: cd ~/openclaw/go2-openclaw-skill && python3 go2_gateway.py &")

    def _post(self, endpoint, data=None):
        """发送 POST 请求"""
        url = f"{self.url}{endpoint}"
        body = json.dumps(data).encode() if data else b"{}"
        req = urllib.request.Request(url, data=body, method="POST")
        req.add_header("Content-Type", "application/json")
        try:
            with urllib.request.urlopen(req, timeout=2) as resp:
                return resp.read().decode()
        except Exception as e:
            print(f"[Go2Dog] ❌ {endpoint} 失败: {e}")
            return None

    def move_forward(self, speed=DEFAULT_SPEED, duration=DEFAULT_DURATION):
        return self._post("/move", {"direction": "forward", "speed": speed, "duration": duration})

    def move_backward(self, speed=DEFAULT_SPEED, duration=DEFAULT_DURATION):
        return self._post("/move", {"direction": "backward", "speed": speed, "duration": duration})

    def turn_left(self, speed=DEFAULT_SPEED, duration=DEFAULT_DURATION):
        return self._post("/move", {"direction": "turn_left", "speed": speed, "duration": duration})

    def turn_right(self, speed=DEFAULT_SPEED, duration=DEFAULT_DURATION):
        return self._post("/move", {"direction": "turn_right", "speed": speed, "duration": duration})

    def strafe_left(self, speed=DEFAULT_SPEED, duration=DEFAULT_DURATION):
        return self._post("/move", {"direction": "left", "speed": speed, "duration": duration})

    def strafe_right(self, speed=DEFAULT_SPEED, duration=DEFAULT_DURATION):
        return self._post("/move", {"direction": "right", "speed": speed, "duration": duration})

    def stop(self):
        return self._post("/stop")

    def move_toward(self, x3d, z3d):
        """
        根据手掌 3D 位置发出运动命令 (高响应版)

        每 0.12s 发一个 0.2s 短脉冲, 实现平滑跟随
        死区小, 响应曲线陡

        Args:
            x3d: 水平偏移 (米), 正=右, 负=左
            z3d: 深度距离 (米), 相机到手的距离

        Returns:
            str: 执行的动作描述
        """
        TARGET_DIST = 0.4    # 目标距离 (米)
        DEADBAND_X = 0.05    # 水平死区 (米) - 比之前小一倍
        MAX_DIST = 2.0       # 最远跟踪距离
        MIN_DIST = 0.15      # 最近跟踪距离

        if z3d > MAX_DIST or z3d < MIN_DIST:
            self.stop()
            return "超出范围"

        # --- 水平转向 (优先, 更激进) ---
        if abs(x3d) > DEADBAND_X:
            # 减少死区: 0.05m 即响应
            # 速度曲线更陡: 小偏移就有明显反应
            turn_speed = min(0.7, 0.15 + abs(x3d) * 1.2)
            if x3d < 0:
                self.turn_left(speed=turn_speed, duration=0.2)
                return f"左转 {turn_speed:.1f}"
            else:
                self.turn_right(speed=turn_speed, duration=0.2)
                return f"右转 {turn_speed:.1f}"

        # --- 前后移动 ---
        dist_err = z3d - TARGET_DIST
        if dist_err > 0.10:  # 前进
            speed = min(0.7, 0.3 + dist_err * 0.5)
            self.move_forward(speed=speed, duration=0.2)
            return f"前进 {speed:.1f}"
        elif dist_err < -0.10:  # 后退
            speed = 0.25
            self.move_backward(speed=speed, duration=0.2)
            return f"后退 {speed:.1f}"
        else:
            self.stop()
            return "到位"
