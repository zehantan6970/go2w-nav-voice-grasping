#!/bin/bash

# --- 强制指定 PulseAudio 环境 ---
export XDG_RUNTIME_DIR=/run/user/$(id -u unitree)
export PULSE_SERVER=unix:$XDG_RUNTIME_DIR/pulse/native

# 启动 AI 语音助手独立服务器 (端口 8766)
/home/unitree/miniforge3/envs/mmdialog/bin/python /home/unitree/robot_ws/qwen/chat_server.py
