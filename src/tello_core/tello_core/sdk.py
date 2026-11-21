"""Tello SDK state strings <-> TelemetrySample (SI units, ROS directions).

The axis and sign mapping of vgx/vgy/vgz, agx/agy/agz, pitch and roll is PROVISIONAL: it follows the
legacy code and must be confirmed by the hand-carry test (spec section 11.2, step 2). Yaw is converted
from the SDK's clockwise degrees to counter-clockwise radians. `baro` is kept unconverted.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

G = 9.80665
NAN = math.nan


@dataclass
class TelemetrySample:
    roll: float = NAN
    pitch: float = NAN
    yaw: float = NAN
    vel_x: float = NAN
    vel_y: float = NAN
    vel_z: float = NAN
    acc_x: float = NAN
    acc_y: float = NAN
    acc_z: float = NAN
    height: float = NAN
    tof: float = NAN
    baro: float = NAN
    battery: float = NAN
    temp_low: float = NAN
    temp_high: float = NAN
    motor_time: float = NAN
    raw: str = ""


def parse_fields(s: str) -> dict[str, float]:
    """'k:v;k:v;' -> {k: float(v)}. Entries that are not numbers (e.g. 'mpry:0,0,0') are skipped."""
    out: dict[str, float] = {}
    for item in s.strip().split(";"):
        key, sep, val = item.partition(":")
        if not sep:
            continue
        try:
            out[key.strip()] = float(val)
        except ValueError:
            continue
    return out


def parse_state(s: str) -> TelemetrySample:
    d = parse_fields(s)

    def g(k: str) -> float:
        return d.get(k, NAN)

    return TelemetrySample(
        roll=math.radians(g("roll")),
        pitch=math.radians(g("pitch")),
        yaw=-math.radians(g("yaw")),
        vel_x=g("vgx") / 10.0,
        vel_y=-g("vgy") / 10.0,
        vel_z=-g("vgz") / 10.0,
        acc_x=g("agx") * G / 1000.0,
        acc_y=-g("agy") * G / 1000.0,
        acc_z=-g("agz") * G / 1000.0,
        height=g("h") / 100.0,
        tof=g("tof") / 100.0,
        baro=g("baro"),
        battery=g("bat"),
        temp_low=g("templ"),
        temp_high=g("temph"),
        motor_time=g("time"),
        raw=s.strip(),
    )


def _i(v: float) -> int:
    return int(round(v)) if math.isfinite(v) else 0


def _f2(v: float) -> str:
    return f"{v:.2f}" if math.isfinite(v) else "0.00"


def format_state(t: TelemetrySample) -> str:
    """Encode like a standard Tello (SDK 1.3): integer degrees, dm/s and cm; inverse of parse_state."""
    yaw = math.atan2(math.sin(t.yaw), math.cos(t.yaw)) if math.isfinite(t.yaw) else NAN
    items = [
        ("pitch", _i(math.degrees(t.pitch))), ("roll", _i(math.degrees(t.roll))),
        ("yaw", _i(-math.degrees(yaw))),
        ("vgx", _i(10 * t.vel_x)), ("vgy", _i(-10 * t.vel_y)), ("vgz", _i(-10 * t.vel_z)),
        ("templ", _i(t.temp_low)), ("temph", _i(t.temp_high)),
        ("tof", _i(100 * t.tof)), ("h", _i(100 * t.height)), ("bat", _i(t.battery)),
        ("baro", _f2(t.baro)), ("time", _i(t.motor_time)),
        ("agx", _f2(1000 * t.acc_x / G)), ("agy", _f2(-1000 * t.acc_y / G)), ("agz", _f2(-1000 * t.acc_z / G)),
    ]
    return ";".join(f"{k}:{v}" for k, v in items) + ";\r\n"
