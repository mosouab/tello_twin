"""In-process fake Tello (spec sections 5.4 and 10).

First-order velocity response per axis in the heading frame, a pure command delay, Tello-like takeoff and
landing, a linear battery drain, and telemetry rounded like the SDK (encoded with sdk.format_state and
decoded with sdk.parse_state).

Default gains K are Švrčina's identified values (CTU 2024, table 5.1): 1.065 m/s horizontal, 0.447 m/s
vertical and 0.955 rad/s yaw per unit stick. The rates k and the 0.6 s delay are assumptions.
"""
from __future__ import annotations

import collections
import math
from dataclasses import dataclass

from .sdk import G, TelemetrySample, format_state, parse_state
from .sticks import Sticks, from_rc

AIRBORNE = ("taking_off", "flying", "landing")


@dataclass(frozen=True)
class FakeParams:
    K: tuple[float, float, float, float] = (1.065, 1.065, 0.447, 0.955)
    k: tuple[float, float, float, float] = (2.5, 2.5, 3.0, 4.0)
    delay: float = 0.6
    takeoff_height: float = 0.8
    climb_rate: float = 0.4
    descent_rate: float = 0.3
    land_height: float = 0.1
    battery_start: float = 100.0
    battery_drain: float = 80.0 / 420.0      # %/s while airborne: 100 -> 20 % in 7 min


@dataclass(frozen=True)
class Truth:
    p: tuple[float, float, float]
    yaw: float
    v_world: tuple[float, float, float]
    yaw_rate: float


class FakeDynamics:
    def __init__(self, params: FakeParams = FakeParams(), position=(0.0, 0.0, 0.0), yaw: float = 0.0,
                 t0: float = 0.0):
        self.prm = params
        self.p = [float(v) for v in position]
        self.yaw = float(yaw)
        self.vb = [0.0, 0.0, 0.0]
        self.r = 0.0
        self.mode = "landed"
        self.battery = params.battery_start
        self.motor_time = 0.0
        self._queue: collections.deque = collections.deque()
        self._u = Sticks()
        self._t = float(t0)

    def set_rc(self, t: float, rc) -> None:
        self._queue.append((t + self.prm.delay, from_rc(rc)))

    def takeoff(self, t: float) -> None:
        if self.mode == "landed":
            self.mode = "taking_off"

    def land(self, t: float) -> None:
        if self.mode in ("taking_off", "flying"):
            self.mode = "landing"

    def emergency(self, t: float) -> None:
        self.vb = [0.0, 0.0, 0.0]
        self.r = 0.0
        self.mode = "falling" if self.p[2] > 0.0 else "off"

    def step(self, t: float) -> list[str]:
        dt = t - self._t
        if dt <= 0.0:
            return []
        self._t = t
        while self._queue and self._queue[0][0] <= t:
            self._u = self._queue.popleft()[1]
        prm, mode = self.prm, self.mode
        if mode == "flying":
            target = [K * u for K, u in zip(prm.K, self._u.as_tuple())]
        elif mode == "taking_off":
            target = [0.0, 0.0, prm.climb_rate, 0.0]
        elif mode == "landing":
            target = [0.0, 0.0, -prm.descent_rate, 0.0]
        else:
            target = None
        if mode == "falling":
            self.vb[2] -= 9.81 * dt
        elif target is None:
            self.vb = [0.0, 0.0, 0.0]
            self.r = 0.0
        else:
            for i in range(3):
                a = math.exp(-prm.k[i] * dt)
                self.vb[i] = target[i] + (self.vb[i] - target[i]) * a
            a = math.exp(-prm.k[3] * dt)
            self.r = target[3] + (self.r - target[3]) * a
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.p[0] += (c * self.vb[0] - s * self.vb[1]) * dt
        self.p[1] += (s * self.vb[0] + c * self.vb[1]) * dt
        self.p[2] += self.vb[2] * dt
        self.yaw += self.r * dt
        if self.p[2] < 0.0:
            self.p[2] = 0.0
        events: list[str] = []
        if mode == "taking_off" and self.p[2] >= prm.takeoff_height:
            self.mode = "flying"
            events.append("takeoff_done")
        elif mode == "landing" and self.p[2] <= prm.land_height:
            self._rest("landed")
            events.append("land_done")
        elif mode == "falling" and self.p[2] <= 0.0:
            self._rest("off")
        if mode in AIRBORNE:
            self.battery = max(0.0, self.battery - prm.battery_drain * dt)
            self.motor_time += dt
        return events

    def _rest(self, mode: str) -> None:
        self.mode = mode
        self.p[2] = 0.0
        self.vb = [0.0, 0.0, 0.0]
        self.r = 0.0

    def truth(self) -> Truth:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        vw = (c * self.vb[0] - s * self.vb[1], s * self.vb[0] + c * self.vb[1], self.vb[2])
        return Truth(tuple(self.p), self.yaw, vw, self.r)

    def telemetry(self) -> TelemetrySample:
        exact = TelemetrySample(roll=0.0, pitch=0.0, yaw=self.yaw, vel_x=self.vb[0], vel_y=self.vb[1],
                                vel_z=self.vb[2], acc_x=0.0, acc_y=0.0, acc_z=G, height=self.p[2],
                                tof=self.p[2] + 0.1, baro=0.0, battery=self.battery, temp_low=60.0,
                                temp_high=62.0, motor_time=self.motor_time)
        return parse_state(format_state(exact))
