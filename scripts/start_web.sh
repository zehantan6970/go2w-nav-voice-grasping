#!/bin/bash

# --- 强制指定 PulseAudio 环境 ---
# 确保 XDG_RUNTIME_DIR 指向 unitree 用户的运行时目录
export XDG_RUNTIME_DIR=/run/user/$(id -u unitree)
# 明确告诉 pactl 去哪里连接
export PULSE_SERVER=unix:$XDG_RUNTIME_DIR/pulse/native

echo "[调试] 当前 XDG_RUNTIME_DIR 为: $XDG_RUNTIME_DIR"
echo "[调试] 当前 PULSE_SERVER 为: $PULSE_SERVER"

# 检查环境变量是否生效
if [ ! -S "$XDG_RUNTIME_DIR/pulse/native" ]; then
    echo "[错误] 找不到 PulseAudio Socket 文件，路径可能不正确！"
else
    echo "[成功] 已定位到 PulseAudio Socket"
fi
# ------------------------------

# 原有的启动脚本代码...
# 使用确认的绝对路径
/home/unitree/miniforge3/envs/mmdialog/bin/python /home/unitree/robot_ws/qwen/web_app.py
