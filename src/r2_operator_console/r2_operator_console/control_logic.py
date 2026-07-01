from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

# Same motion vocabulary as teleop_twist_keyboard:
# tuple = (x, y, z, th), where x/y/z are linear axes and th is yaw.
MOVE_BINDINGS: Dict[str, Tuple[int, int, int, int]] = {
    "i": (1, 0, 0, 0),
    "o": (1, 0, 0, -1),
    "j": (0, 0, 0, 1),
    "l": (0, 0, 0, -1),
    "u": (1, 0, 0, 1),
    ",": (-1, 0, 0, 0),
    ".": (-1, 0, 0, 1),
    "m": (-1, 0, 0, -1),
    "O": (1, -1, 0, 0),
    "I": (1, 0, 0, 0),
    "J": (0, 1, 0, 0),
    "L": (0, -1, 0, 0),
    "U": (1, 1, 0, 0),
    "<": (-1, 0, 0, 0),
    ">": (-1, -1, 0, 0),
    "M": (-1, 1, 0, 0),
    "t": (0, 0, 1, 0),
    "b": (0, 0, -1, 0),
}

# Same speed scaling keys as teleop_twist_keyboard:
# tuple = (linear_scale, angular_scale).
SPEED_BINDINGS: Dict[str, Tuple[float, float]] = {
    "q": (1.1, 1.1),
    "z": (0.9, 0.9),
    "w": (1.1, 1.0),
    "x": (0.9, 1.0),
    "e": (1.0, 1.1),
    "c": (1.0, 0.9),
}


@dataclass
class SpeedSettings:
    vx: float = 0.50
    vy: float = 0.50
    wz: float = 1.00
    linear_z: float = 0.50
    max_vx: float = 2.50
    max_vy: float = 2.50
    max_wz: float = 1.20
    min_speed: float = 0.0
    step: float = 0.01

    def clamp(self) -> None:
        self.vx = clamp(self.vx, self.min_speed, self.max_vx)
        self.vy = clamp(self.vy, self.min_speed, self.max_vy)
        self.wz = clamp(self.wz, self.min_speed, self.max_wz)
        self.linear_z = clamp(self.linear_z, self.min_speed, self.max_vx)

    def scaled(self, linear_scale: float, angular_scale: float) -> "SpeedSettings":
        return SpeedSettings(
            vx=clamp(self.vx * linear_scale, self.min_speed, self.max_vx),
            vy=clamp(self.vy * linear_scale, self.min_speed, self.max_vy),
            wz=clamp(self.wz * angular_scale, self.min_speed, self.max_wz),
            linear_z=clamp(self.linear_z * linear_scale, self.min_speed, self.max_vx),
            max_vx=self.max_vx,
            max_vy=self.max_vy,
            max_wz=self.max_wz,
            min_speed=self.min_speed,
            step=self.step,
        )

    def nudged(self, axis: str, delta: float) -> "SpeedSettings":
        out = SpeedSettings(
            vx=self.vx,
            vy=self.vy,
            wz=self.wz,
            linear_z=self.linear_z,
            max_vx=self.max_vx,
            max_vy=self.max_vy,
            max_wz=self.max_wz,
            min_speed=self.min_speed,
            step=self.step,
        )
        if axis == "vx":
            out.vx += delta
        elif axis == "vy":
            out.vy += delta
        elif axis == "wz":
            out.wz += delta
        elif axis == "linear_z":
            out.linear_z += delta
        else:
            raise ValueError(f"unsupported axis: {axis}")
        out.clamp()
        return out


def clamp(value: float, low: float, high: float) -> float:
    return max(float(low), min(float(high), float(value)))


def motion_from_key(key_text: str) -> Optional[Tuple[int, int, int, int]]:
    return MOVE_BINDINGS.get(key_text)


def speed_scale_from_key(key_text: str) -> Optional[Tuple[float, float]]:
    return SPEED_BINDINGS.get(key_text)


def velocity_from_motion(
    motion: Tuple[int, int, int, int],
    speeds: SpeedSettings,
) -> Tuple[float, float, float, float]:
    x, y, z, th = motion
    return (
        float(x) * speeds.vx,
        float(y) * speeds.vy,
        float(z) * speeds.linear_z,
        float(th) * speeds.wz,
    )


def percent_text(value: float, maximum: float) -> str:
    if maximum <= 0.0:
        return "0%"
    return f"{int(round(clamp(value / maximum, 0.0, 1.0) * 100.0))}%"
