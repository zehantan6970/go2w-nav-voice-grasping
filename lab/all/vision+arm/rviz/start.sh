#!/bin/bash

WEB_DIR="/home/unitree/vision+arm/rviz/web"
MAP_DIR="/home/unitree/0514map"

# 加载 ROS2 Foxy 环境
source /opt/ros/foxy/setup.bash

echo "🔄 清理旧进程..."
pkill -f "python3 -m http.server 8889" 2>/dev/null
pkill -f "rosbridge_websocket" 2>/dev/null
sleep 1

# 创建地图软链接（HTTP 通过 /maps/xxx.png 访问地图文件）
if [ ! -L "$WEB_DIR/maps" ]; then
    ln -sf "$MAP_DIR" "$WEB_DIR/maps"
    echo "🔗 已创建软链接: $WEB_DIR/maps -> $MAP_DIR"
fi

# 检查 PNG 是否存在
if [ ! -f "$MAP_DIR/005.png" ]; then
    echo "⚠️  警告: 未找到 PNG 地图，正在自动转换..."
    cd "$MAP_DIR"
    for f in *.pgm; do
        [ -f "$f" ] && convert "$f" "${f%.pgm}.png" && echo "✅ $f -> ${f%.pgm}.png"
    done
fi

echo "📡 启动 rosbridge (ws://127.0.0.1:9090)..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml > /dev/null 2>&1 &
sleep 3

echo "🌐 启动 HTTP 服务器 (http://0.0.0.0:8889/)..."
echo "   前端地址: http://<go2-ip>:8889/rviz_live/"
echo ""
cd "$WEB_DIR"
python3 -m http.server 8889