#!/usr/bin/env python3
"""Receive raw UDP packets without ROS2 to split network issues from ROS issues."""

from __future__ import annotations

import argparse
import socket

from gmk_weapon_bridge.protocol import unpack_weapon_packet


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=12345)
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.bind, args.port))
    print(f"Listening on UDP {args.bind}:{args.port} ... (Ctrl+C stop)")
    print("Format: [Seq, Valid, U, V, Z_m, Conf]")
    try:
        while True:
            data, addr = sock.recvfrom(1024)
            unpacked = unpack_weapon_packet(data)
            if unpacked is None:
                continue
            seq, valid, u, v, z, conf = unpacked
            print(f"[{addr[0]}] Seq:{seq:06d} | Valid:{valid} | U:{u:6.1f} V:{v:6.1f} Z:{z:.3f}m Conf:{conf:.2f}")
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
