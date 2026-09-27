#!/bin/bash
set -e
SLAM_DIR="/unitree/module/unitree_slam/bin"
DRIVER_NAME="mid360_driver"
SLAM_NAME="unitree_slam"
DRIVER_LOG="/tmp/mid360_driver.log"
SLAM_LOG="/tmp/unitree_slam.log"
BRIDGE_LOG="/tmp/rosbridge_server.log"
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; CYAN='\033[0;36m'; NC='\033[0m'
log_info()  { echo -e "${GREEN}[INFO]${NC}  $1"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }
log_step()  { echo -e "${CYAN}[STEP]${NC}  $1"; }
is_running() { pgrep -x "$1" > /dev/null 2>&1; }
cleanup() {
    log_step "检查并清理已有进程..."
    for proc in "$DRIVER_NAME" "$SLAM_NAME" "rosbridge_server" "ros2"; do
        if is_running "$proc"; then
            log_warn "发现 $proc 正在运行，正在终止..."
            pkill -x "$proc" 2>/dev/null || true; sleep 1
            is_running "$proc" && kill -9 $(pgrep -x "$proc" 2>/dev/null) 2>/dev/null || true
        fi
    done
    log_info "清理完成"
}
check_files() {
    log_step "检查文件..."
    [ ! -x "$SLAM_DIR/$DRIVER_NAME" ] && { log_error "找不到: $SLAM_DIR/$DRIVER_NAME"; exit 1; }
    [ ! -x "$SLAM_DIR/$SLAM_NAME" ] && { log_error "找不到: $SLAM_DIR/$SLAM_NAME"; exit 1; }
    log_info "文件检查通过"
}
start_driver() {
    log_step "启动 $DRIVER_NAME ..."
    cd "$SLAM_DIR"
    nohup ./$DRIVER_NAME > "$DRIVER_LOG" 2>&1 &
    sleep 3
    if ! is_running "$DRIVER_NAME"; then
        log_error "$DRIVER_NAME 启动失败！日志: tail -n 50 $DRIVER_LOG"; exit 1
    fi
    log_info "$DRIVER_NAME 启动成功"
}
start_slam() {
    log_step "启动 $SLAM_NAME ..."
    cd "$SLAM_DIR"
    nohup ./$SLAM_NAME > "$SLAM_LOG" 2>&1 &
    sleep 5
    if ! is_running "$SLAM_NAME"; then
        log_error "$SLAM_NAME 启动失败！日志: tail -n 50 $SLAM_LOG"; exit 1
    fi
    log_info "$SLAM_NAME 启动成功"
}
start_rosbridge() {
    log_step "启动 rosbridge_server ..."
    source /opt/ros/foxy/setup.bash 2>/dev/null || true
    nohup ros2 launch rosbridge_server rosbridge_websocket_launch.xml > "$BRIDGE_LOG" 2>&1 &
    sleep 3
    if ! pgrep -f "rosbridge" > /dev/null 2>&1; then
        log_warn "rosbridge_server 可能启动失败，日志: tail -n 50 $BRIDGE_LOG"
    else
        log_info "rosbridge_server 启动成功"
    fi
}
show_status() {
    echo ""; echo "============================================================"
    echo -e "              ${GREEN}✓ SLAM 系统启动完成${NC}"
    echo "============================================================"
    echo ""; echo "  运行中的进程:"
    ps -ef | grep -E "mid360_driver|unitree_slam|rosbridge" | grep -v grep | awk '{printf "  PID: %-8s %s\n", $2, $8}'
    echo ""; echo "  日志查看:"; echo "    tail -f $DRIVER_LOG"; echo "    tail -f $SLAM_LOG"; echo "    tail -f $BRIDGE_LOG"
    echo ""; echo "  WebSocket地址: ws://$(hostname -I | awk '{print $1}'):9090"
    echo ""; echo "  停止命令: ./slam_stop.sh"
    echo "============================================================"
}
echo "============================================================"
echo "           Unitree SLAM 一键启动脚本"
echo "============================================================"
cleanup; check_files; start_driver; start_slam; start_rosbridge; show_status
