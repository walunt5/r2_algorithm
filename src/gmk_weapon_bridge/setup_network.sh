#!/bin/bash
# GMK 桥接端网络一键配置脚本
# IP: 192.168.10.100/24 -> 目标: 192.168.10.101

TARGET_IP="192.168.10.100/24"
PEER_IP="192.168.10.101"
CON_NAME="weapon-field"

if [ "$EUID" -ne 0 ]; then
  echo "请使用 sudo 运行此脚本: sudo bash $0"
  exit 1
fi

# 1. 寻找物理网卡
if [ -n "$1" ]; then
    IFACE=$1
else
    # 过滤掉 lo, docker, veth, wlan, wl 等，寻找真实的以太网卡 (eth, en)
    IFACE=$(ip link show | grep -E '^[0-9]+: (eth|en)' | awk -F': ' '{print $2}' | head -n 1)
fi

if [ -z "$IFACE" ]; then
    echo "错误：未找到有线网卡！请手动指定网卡名，例如: sudo bash $0 eth0"
    exit 1
fi

echo ">> 选择配置网卡: $IFACE"

# 2. 配置 NetworkManager (持久化)
if command -v nmcli &> /dev/null; then
    echo ">> 使用 NetworkManager 配置持久化静态 IP: $TARGET_IP"
    nmcli con delete "$CON_NAME" &>/dev/null
    nmcli con add type ethernet con-name "$CON_NAME" ifname "$IFACE" ipv4.method manual ipv4.addresses "$TARGET_IP" ipv6.method ignore
    nmcli con up "$CON_NAME"
    echo ">> NetworkManager 配置完成，重启不丢。"
else
    echo ">> 未检测到 NetworkManager，降级使用 ip 命令临时配置 (重启失效)。"
    ip addr flush dev "$IFACE"
    ip addr add "$TARGET_IP" dev "$IFACE"
    ip link set "$IFACE" up
    echo ">> 临时配置完成。若要开机生效，请在机器上安装 NetworkManager 或配置 netplan/interfaces。"
fi

# 3. 如果开启了 ufw 防火墙，放行 UDP 12345 端口
if command -v ufw &> /dev/null; then
    if ufw status | grep -q "Status: active"; then
        echo ">> 检测到 UFW 防火墙开启，正在放行 UDP 12345 端口..."
        ufw allow 12345/udp
    fi
fi

# 4. 验证连接
echo ">> 正在 Ping Jetson 视觉端 ($PEER_IP) 检测连通性..."
if ping -c 3 -W 1 "$PEER_IP" &> /dev/null; then
    echo "========================================="
    echo "✅ OK：对端可达！网络就绪。"
    echo "========================================="
else
    echo "========================================="
    echo "⚠️ 警告：Ping 失败。Jetson 端可能还没配好，或者网线没插好。"
    echo "请在 Jetson 端也运行对应的 setup_network.sh 脚本。"
    echo "========================================="
fi
