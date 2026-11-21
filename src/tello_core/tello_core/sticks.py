"""Stick commands in ROS convention and their Tello SDK `rc` encoding (spec section 4.1).

Sticks are normalised to [-1, 1] with ROS directions (REP 103): x forward, y left, z up,
yaw counter-clockwise (left). SDK `rc a b c d` takes integers in [-100, 100]:
a = right, b = forward, c = up, d = clockwise.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

AXES = ("x", "y", "z", "yaw")


@dataclass(frozen=True)
class Sticks:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0

    def as_tuple(self) -> tuple[float, float, float, float]:
        return (self.x, self.y, self.z, self.yaw)


ZERO = Sticks()


def _finite(v: float) -> float:
    return v if math.isfinite(v) else 0.0


def clamp_sticks(s: Sticks, stick_max: tuple[float, float, float, float]) -> Sticks:
    """Limit each axis to [-stick_max[i], stick_max[i]]. Non-finite values become 0."""
    if len(stick_max) != 4:
        raise ValueError("stick_max needs 4 values (x, y, z, yaw)")
    out = []
    for v, m in zip(s.as_tuple(), stick_max):
        if not 0.0 <= m <= 1.0:
            raise ValueError(f"stick_max values must be in [0, 1], got {m}")
        out.append(max(-m, min(m, _finite(v))))
    return Sticks(*out)


def _q(v: float) -> int:
    """100 * v rounded half away from zero, clamped to [-100, 100]."""
    x = 100.0 * _finite(v)
    r = int(math.floor(abs(x) + 0.5))
    return max(-100, min(100, r if x >= 0 else -r))


def to_rc(s: Sticks) -> tuple[int, int, int, int]:
    return (_q(-s.y), _q(s.x), _q(s.z), _q(-s.yaw))


def from_rc(rc) -> Sticks:
    a, b, c, d = rc
    return Sticks(x=b / 100.0, y=-a / 100.0, z=c / 100.0, yaw=-d / 100.0)


def format_rc(rc) -> str:
    return "rc %d %d %d %d" % tuple(rc)
