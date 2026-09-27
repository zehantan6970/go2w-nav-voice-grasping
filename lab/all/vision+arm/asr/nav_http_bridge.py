#!/usr/bin/env python3
"""
导航 HTTP 桥接服务
- 接收 Python 语音程序的 HTTP 请求
- 通过 tmux 向 demo1 发送按键指令
"""

from flask import Flask, request, jsonify
import subprocess
import json
import os

app = Flask(__name__)

# tmux session 名称（demo1 运行的 session）
TMUX_SESSION = "slam_session"  # 根据你的实际 session 名修改

def send_tmux_keys(keys: str):
    """向 demo1 的 tmux session 发送按键"""
    try:
        cmd = f"tmux send-keys -t {TMUX_SESSION} '{keys}'"
        subprocess.run(cmd, shell=True, check=True, 
                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return True
    except Exception as e:
        print(f"[Bridge] tmux 发送失败: {e}")
        return False

@app.route("/nav/command", methods=["POST"])
def nav_command():
    """接收导航指令"""
    data = request.json or {}
    action = data.get("action", "")
    target = data.get("target", "")
    
    print(f"[Bridge] 收到指令: action={action}, target={target}")
    
    # 指令映射为 tmux 按键序列
    key_map = {
        "goto_point": f"g\n{target}\n",      # g → 输入点位 → 回车
        "start_loop": "d",                      # 循环巡航
        "start_single": "c",                    # 单次巡航
        "pause": "z",                           # 暂停
        "resume": "x",                          # 恢复
        "save_point": f"s\ny\n{target}\n\n\n",  # 保存点位
        "list_points": "l",                     # 查看列表
        "stop_nav": "o",                        # 停止（确认）
    }
    
    keys = key_map.get(action)
    if not keys:
        return jsonify({"status": "error", "msg": f"未知指令: {action}"}), 400
    
    if send_tmux_keys(keys):
        return jsonify({"status": "ok", "action": action})
    else:
        return jsonify({"status": "error", "msg": "tmux 发送失败"}), 500

@app.route("/nav/status", methods=["GET"])
def nav_status():
    """查询导航状态（预留）"""
    return jsonify({"status": "ok", "navigating": False})

if __name__ == "__main__":
    print("[Bridge] 导航 HTTP 桥接服务启动，端口 5002")
    app.run(host="0.0.0.0", port=5002, threaded=True)
