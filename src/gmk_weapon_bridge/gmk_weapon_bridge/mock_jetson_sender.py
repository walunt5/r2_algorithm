#!/usr/bin/env python3
"""Fake Jetson sender for GMK-side end-to-end testing."""

from __future__ import annotations

import argparse
import math
import socket
import time

from gmk_weapon_bridge.protocol import pack_weapon_packet


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", default="127.0.0.1", help="GMK IP; default is local loopback")
    ap.add_argument("--port", type=int, default=12345)
    ap.add_argument("--rate", type=float, default=30.0, help="Send rate in Hz")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = (args.ip, args.port)
    period = 1.0 / max(1.0, args.rate)
    seq = 0
    t0 = time.monotonic()
    print(f"[mock] sending fake weapon targets -> {addr} @ {args.rate:.0f}Hz (Ctrl+C stop)")
    try:
        while True:
            t = time.monotonic() - t0
            seq = (seq + 1) & 0xFFFFFFFF
            lost = (t % 5.0) > 4.0
            if lost:
                pkt = pack_weapon_packet(seq, 0, 0.0, 0.0, 0.0, 0.0)
            else:
                u = 320.0 + 80.0 * math.cos(t)
                v = 240.0 + 60.0 * math.sin(t)
                z = 0.50 + 0.05 * math.sin(t / 2.0)
                pkt = pack_weapon_packet(seq, 1, u, v, z, 0.90)
            sock.sendto(pkt, addr)
            if seq % int(max(1.0, args.rate) * 2) == 0:
                print(f"[mock] seq={seq} {'LOST' if lost else 'target moving'}")
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n[mock] stopped")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
