#!/bin/bash

if [ "$EUID" -ne 0 ]; then
  echo "🔑 正在请求管理员权限以重置网卡和路由..."
  exec sudo "$0" "$@"
fi

echo "======= 阶段 1: 硬件重启与驱动重载 ======="

echo "1. 正在卸载驱动 r8188eu..."
modprobe -r r8188eu

echo "2. 从 USB 总线断电 (1-2.3)..."
echo '1-2.3' > /sys/bus/usb/drivers/usb/unbind

echo "3. 重新上电枚举..."
echo '1-2.3' > /sys/bus/usb/drivers/usb/bind

echo "4. 重新加载驱动 r8188eu..."
modprobe r8188eu

echo "⏳ 等待 8 秒让网卡重新初始化并连接网络..."
sleep 8

echo "======= 阶段 2: 路由表修复与网络测试 ======="

# 检查 wlan0 是否已经分配到 IP，如果没有则先跳过路由修改
if ! ip addr show dev wlan0 | grep -q "inet "; then
  echo "⚠️ wlan0 尚未获取到 IP 地址，尝试动态申请 (dhclient)..."
  dhclient wlan0 -v
  sleep 2
fi

echo "1. 清除 wlan0 原有的默认路由..."
ip route del default via 10.160.71.254 dev wlan0 2>/dev/null

echo "2. 重新添加默认路由，将 metric 设为 100..."
if ip route add default via 10.160.71.254 dev wlan0 metric 100; then
  echo "✅ 路由规则修改成功！"
else
  echo "❌ 路由规则修改失败，请检查 10.160.71.254 是否在当前网段内。"
fi

echo "3. 当前路由表状态："
echo "----------------------------------------"
ip route show
echo "----------------------------------------"

echo "4. 开始网络连通性测试..."
echo "-> 路由追踪 (8.8.8.8):"
ip route get 8.8.8.8

echo "-> Ping 公网 DNS (8.8.8.8)..."
ping -c 3 8.8.8.8

echo "-> Ping 百度域名 (baidu.com)..."
ping -c 3 baidu.com

echo "🎉 脚本执行完毕！"
