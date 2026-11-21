"""Feedback helpers for the `goto` and `arena` segments (spec section 7.2).

Both act on the position predicted `cmd_delay` ahead (p + v * cmd_delay): with about 0.6-0.7 s between a
command and the motion, a controller acting on the current position oscillates (Švrčina, CTU 2024).
`stick_scale` is the ASSUMED velocity per unit stick of the drone; the default is Švrčina's table 5.1.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .safety import Box
from .sticks import Sticks


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def world_to_heading(v, yaw: float) -> tuple[float, float, float]:
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * v[0] + s * v[1], -s * v[0] + c * v[1], v[2])


@dataclass(frozen=True)
class ControlConfig:
    stick_scale: tuple[float, float, float, float] = (1.065, 1.065, 0.447, 0.955)
    cmd_delay: float = 0.7
    stick_max: tuple[float, float, float, float] = (0.4, 0.4, 0.4, 0.4)


@dataclass(frozen=True)
class GotoParams:
    target: tuple[float, float, float]
    yaw: float | None = None
    kp: float = 0.8
    v_max: float = 0.4
    tol: float = 0.1
    kp_yaw: float = 1.0
    yaw_rate_max: float = 0.5
    yaw_tol: float = 0.1
    timeout: float = 15.0


def goto_command(gp: GotoParams, cfg: ControlConfig, p, yaw: float, v, yaw_rate: float) -> Sticks:
    pp = [p[i] + v[i] * cfg.cmd_delay for i in range(3)]
    vd = [gp.kp * (gp.target[i] - pp[i]) for i in range(3)]
    n = math.sqrt(sum(x * x for x in vd))
    if n > gp.v_max:
        vd = [x * gp.v_max / n for x in vd]
    vh = world_to_heading(vd, yaw)
    if gp.yaw is None:
        wz = 0.0
    else:
        ey = wrap_angle(gp.yaw - (yaw + yaw_rate * cfg.cmd_delay))
        wz = max(-gp.yaw_rate_max, min(gp.yaw_rate_max, gp.kp_yaw * ey))
    sx, sy, sz, syaw = cfg.stick_scale
    raw = (vh[0] / sx, vh[1] / sy, vh[2] / sz, wz / syaw)
    return Sticks(*[max(-m, min(m, r)) for r, m in zip(raw, cfg.stick_max)])


def goto_reached(gp: GotoParams, p, yaw: float, v) -> bool:
    dist = math.sqrt(sum((gp.target[i] - p[i]) ** 2 for i in range(3)))
    speed = math.sqrt(sum(x * x for x in v))
    ok = dist <= gp.tol and speed <= 0.15
    if gp.yaw is not None:
        ok = ok and abs(wrap_angle(gp.yaw - yaw)) <= gp.yaw_tol
    return ok


@dataclass(frozen=True)
class ArenaParams:
    duration: float
    seed: int = 0
    v_max: float = 0.3
    u_exc: float = 0.05
    margin: float = 0.3
    yaw_rate_max: float = 0.3


class Arena:
    """Random waypoints inside `box` shrunk by `margin`, reached with predicted-position P control, plus a
    smoothed random excitation on all four axes (port of the legacy scenarios.Arena)."""

    def __init__(self, ap: ArenaParams, cfg: ControlConfig, box: Box):
        self.ap, self.cfg = ap, cfg
        self.box = box.shrink(ap.margin)
        self.rng = np.random.default_rng(ap.seed)
        self.t_next = -math.inf
        self.wp = self.box.center()
        self.yaw_cmd = 0.0
        self.exc = np.zeros(4)
        self.exc_target = np.zeros(4)
        self.t_last: float | None = None

    def _new_waypoint(self, t: float) -> None:
        b, rng = self.box, self.rng
        self.wp = (float(rng.uniform(b.xmin, b.xmax)), float(rng.uniform(b.ymin, b.ymax)),
                   float(rng.uniform(b.zmin, b.zmax)))
        self.yaw_cmd = float(rng.choice([0.0, 0.0, 1.0, -1.0]) * rng.uniform(0.1, 1.0) * self.ap.yaw_rate_max)
        self.exc_target = rng.uniform(-1.0, 1.0, 4) * self.ap.u_exc
        self.t_next = t + float(rng.uniform(3.0, 6.0))

    def step(self, t: float, p, yaw: float, v, yaw_rate: float) -> Sticks:
        if t >= self.t_next:
            self._new_waypoint(t)
        dt = 0.0 if self.t_last is None else max(0.0, t - self.t_last)
        self.t_last = t
        self.exc += (self.exc_target - self.exc) * min(1.0, dt / 1.0)
        u = goto_command(GotoParams(target=self.wp, v_max=self.ap.v_max), self.cfg, p, yaw, v, yaw_rate)
        raw = np.array(u.as_tuple()) + self.exc
        raw[3] += self.yaw_cmd / self.cfg.stick_scale[3]
        lim = np.array(self.cfg.stick_max)
        return Sticks(*[float(x) for x in np.clip(raw, -lim, lim)])
