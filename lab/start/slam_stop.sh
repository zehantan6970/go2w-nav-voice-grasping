#!/bin/bash
GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
log_info()  { echo -e "${GREEN}[INFO]${NC}  $1"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }
echo "============================================================"
echo "           Unitree SLAM 一键停止脚本"
echo "============================================================"
for proc in "rosbridge" "unitree_slam" "mid360_driver"; do
    if [ "$proc" = "rosbridge" ]; then
        PIDS=$(pgrep -f "$proc" 2>/dev/null)
    else
        PIDS=$(pgrep -x "$proc" 2>/dev/null)
    fi
    if [ -n "$PIDS" ]; then
        for PID in $PIDS; do
            log_info "停止 $proc (PID: $PID) ..."
            kill -9 "$PID" 2>/dev/null
        done
        sleep 1
    else
        log_warn "$proc 未运行"
    fi
done
log_info "所有进程已停止"
echo "============================================================"
