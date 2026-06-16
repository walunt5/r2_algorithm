#!/bin/bash
# ==============================================================================
#  techx_vision_bridge 一键启动脚本
#  用法: ./start_vision.sh
#  启动后终端实时显示接收到的目标数据
# ==============================================================================

set -e

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  TECHX Vision Bridge — GMK 视觉数据接收"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# 1. 环境
echo "[1/4] 加载 ROS 2 环境..."
source /opt/ros/humble/setup.bash
source /home/cyrus/robocon2026/GMK/install/setup.bash

# 2. 编译（如有改动）
echo "[2/4] 检查编译..."
cd /home/cyrus/robocon2026/GMK
colcon build --packages-select techx_vision_bridge 2>/dev/null

# 3. 网口检查
echo "[3/4] 检查网口..."
if ! ip addr show enp2s0 | grep -q "192.168.10.100"; then
    echo "  配置 enp2s0 网口..."
    echo "cyrus" | sudo -S ip addr add 192.168.10.100/24 dev enp2s0 2>/dev/null || true
    echo "cyrus" | sudo -S ip link set enp2s0 up 2>/dev/null || true
fi
echo "  enp2s0: $(ip addr show enp2s0 | grep 'inet ' | awk '{print $2}')"

# 4. 检查 Jetson
echo "[4/4] 检查 Jetson..."
if ping -c 1 -W 1 192.168.10.101 > /dev/null 2>&1; then
    echo "  Jetson 192.168.10.101 ✅ 已连接"
else
    echo "  Jetson 192.168.10.101 ⚠️  未响应（请检查网线和 Jetson 配置）"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  启动 vision_bridge_node..."
echo "  收到数据时会实时打印目标坐标"
echo "  按 Ctrl+C 停止"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# 启动节点（前台运行，实时显示）
ros2 launch techx_vision_bridge vision_bridge.launch.py
