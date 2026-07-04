"""Shared UDP packet protocol for the GMK weapon bridge."""

from __future__ import annotations

import struct

MAGIC = 0x5701
PKT_FMT = "<H I B f f f f"  # magic, seq, valid, u, v, z_m, conf
PKT_SIZE = struct.calcsize(PKT_FMT)


def pack_weapon_packet(seq: int, valid: int, u: float, v: float, z_m: float, conf: float) -> bytes:
    return struct.pack(PKT_FMT, MAGIC, int(seq) & 0xFFFFFFFF, int(valid), float(u), float(v), float(z_m), float(conf))


def unpack_weapon_packet(data: bytes):
    """Return (seq, valid, u, v, z_m, conf), or None for non-protocol data."""
    if len(data) != PKT_SIZE:
        return None
    magic, seq, valid, u, v, z, conf = struct.unpack(PKT_FMT, data)
    if magic != MAGIC:
        return None
    return int(seq), int(valid), float(u), float(v), float(z), float(conf)
