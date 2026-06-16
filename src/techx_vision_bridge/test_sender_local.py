#!/usr/bin/env python3
# ==============================================================================
#  test_sender.py — 在 Jetson Orin NX 上运行，向 PC 发送模拟的 29 字节 UDP 包
#
#  用法：
#    1. 修改下方 SEND_TO_IP（你的 PC 网口 IP）
#    2. python3 test_sender.py
#
#  发送的包格式（29 字节小端序）：
#    Magic(2B)  Seq(4B)  Timestamp(8B)  TrackID(1B)  X/Y/Z(12B)  CRC16(2B)
# ==============================================================================

import socket
import struct
import time
import math

# ═══════════════════════════════════════════════════════════════
#  配置区域 —— 只需要改这里！
# ═══════════════════════════════════════════════════════════════
SEND_TO_IP   = "127.0.0.1"   # PC 的网口 IP ← 改这里！
SEND_TO_PORT = 12345              # 与 vision_bridge_node 的 udp_port 一致

SEND_RATE_HZ = 30                # 发包频率（Hz），默认 30 帧/秒
TARGET_COUNT = 1                 # 模拟目标数量（发多个 track_id 的目标）
# ═══════════════════════════════════════════════════════════════

# 协议常量
MAGIC_VALUE  = 0x55AA
PACKET_SIZE  = 29

# CCITT-CRC16 查表（编译期风格，Python 运行时构建）
def _build_crc16_table():
    table = []
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
        table.append(crc)
    return table

CRC16_TABLE = _build_crc16_table()

def crc16_ccitt(data: bytes) -> int:
    """计算 CCITT-CRC16 校验值，初始值 0xFFFF，与 Jetson 端 STM32 HAL_CRC 一致"""
    crc = 0xFFFF
    for byte in data:
        idx = ((crc >> 8) ^ byte) & 0xFF
        crc = ((crc << 8) & 0xFFFF) ^ CRC16_TABLE[idx]
    return crc

def build_packet(seq: int, track_id: int) -> bytes:
    """构造一帧 29 字节 UDP 包"""
    timestamp = time.time()
    # 模拟一个正弦运动的假目标
    t = timestamp * 0.5
    x = 1.0 + 0.5 * math.sin(t * 3.14)     # 在 0.5~1.5m 之间摆动
    y = 0.3 * math.cos(t * 1.57)            # 左右 ±0.3m
    z = 2.0 + 0.1 * math.sin(t * 6.28)      # 深度小幅波动

    # 打包前 27 字节（不含 CRC）
    header = struct.pack("<HIdBfff",
        MAGIC_VALUE,    # uint16 magic
        seq,            # uint32 seq
        timestamp,      # double timestamp
        track_id,       # uint8 track_id
        x, y, z         # float32 x, y, z
    )
    # 计算 CRC16
    crc = crc16_ccitt(header)
    # 拼接 CRC
    packet = header + struct.pack("<H", crc)
    return packet

def main():
    print(f"test_sender 启动")
    print(f"  → 目标地址: {SEND_TO_IP}:{SEND_TO_PORT}")
    print(f"  → 发包频率: {SEND_RATE_HZ} Hz")
    print(f"  → 目标数量: {TARGET_COUNT}")
    print(f"  → 魔数:     0x{MAGIC_VALUE:04X}")
    print()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    seq = 0
    interval = 1.0 / SEND_RATE_HZ

    try:
        while True:
            for tid in range(TARGET_COUNT):
                packet = build_packet(seq, tid)
                sock.sendto(packet, (SEND_TO_IP, SEND_TO_PORT))
                seq += 1

            if seq % (SEND_RATE_HZ * 2) == 0:  # 每 2 秒打印一次
                print(f"[OK] 已发送 {seq} 帧, 最新序列号={seq-1}, "
                      f"频率~{SEND_RATE_HZ}Hz")

            time.sleep(interval)

    except KeyboardInterrupt:
        print("\n用户中断，发送结束。")
    finally:
        sock.close()

if __name__ == "__main__":
    main()