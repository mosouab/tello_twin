"""Safety rules for a flying Tello (spec section 5.3).

Pure functions: tello_node computes the ages and calls `evaluate()` every tick while FLYING, then applies
the most severe action (NONE < HOVER < LAND). Events are reported every tick while their condition holds;
`EventLatch` turns them into rising edges for publishing.
"""
from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass, field
from enum import IntEnum


@dataclass(frozen=True)
class Box:
    xmin: float
    xmax: float
    ymin: float
    ymax: float
    zmin: float
    zmax: float

    @staticmethod
    def from_list(v) -> "Box":
        if len(v) != 6:
            raise ValueError("box needs 6 values: xmin xmax ymin ymax zmin zmax")
        b = Box(*[float(x) for x in v])
        if not (b.xmin < b.xmax and b.ymin < b.ymax and b.zmin < b.zmax):
            raise ValueError(f"empty box {list(v)}")
        return b

    def contains(self, p) -> bool:
        x, y, z = p
        return self.xmin <= x <= self.xmax and self.ymin <= y <= self.ymax and self.zmin <= z <= self.zmax

    def contains_xy(self, p) -> bool:
        return self.xmin <= p[0] <= self.xmax and self.ymin <= p[1] <= self.ymax

    def shrink(self, m: float) -> "Box":
        return Box.from_list([self.xmin + m, self.xmax - m, self.ymin + m, self.ymax - m,
                              self.zmin + m, self.zmax - m])

    def center(self) -> tuple[float, float, float]:
        return ((self.xmin + self.xmax) / 2, (self.ymin + self.ymax) / 2, (self.zmin + self.zmax) / 2)


class Action(IntEnum):
    NONE = 0
    HOVER = 1
    LAND = 2


@dataclass(frozen=True)
class SafetyConfig:
    hard_box: Box
    cmd_timeout_hover: float = 0.3
    cmd_timeout_land: float = 5.0
    mocap_timeout_hover: float = 0.5
    mocap_timeout_land: float = 1.5
    telemetry_timeout: float = 1.0
    battery_land: float = 15.0
    cmd_delay: float = 0.7
    stop_time: float = 0.5

    def __post_init__(self) -> None:
        for name in ("cmd_timeout_hover", "cmd_timeout_land", "mocap_timeout_hover", "mocap_timeout_land",
                     "telemetry_timeout"):
            v = getattr(self, name)
            if not (math.isfinite(v) and v > 0):
                raise ValueError(f"SafetyConfig.{name} must be finite and > 0, got {v!r}")
        if not self.cmd_timeout_hover < self.cmd_timeout_land:
            raise ValueError("SafetyConfig.cmd_timeout_hover must be lower than cmd_timeout_land")
        if not self.mocap_timeout_hover < self.mocap_timeout_land:
            raise ValueError("SafetyConfig.mocap_timeout_hover must be lower than mocap_timeout_land")
        if not (math.isfinite(self.battery_land) and 0 <= self.battery_land <= 100):
            raise ValueError(f"SafetyConfig.battery_land must be finite in [0, 100], got {self.battery_land!r}")
        for name in ("cmd_delay", "stop_time"):
            v = getattr(self, name)
            if not (math.isfinite(v) and v >= 0):
                raise ValueError(f"SafetyConfig.{name} must be finite and >= 0, got {v!r}")
        if not all(math.isfinite(b) for b in dataclasses.astuple(self.hard_box)):
            raise ValueError(f"SafetyConfig.hard_box bounds must be finite, got {self.hard_box!r}")

    @property
    def lookahead(self) -> float:
        return self.cmd_delay + self.stop_time


@dataclass(frozen=True)
class SafetyInputs:
    """Caller contract for one safety tick.

    - All ages are seconds on the caller's clock; `math.inf` (or NaN) means never received.
    - Ages are computed from ARRIVAL times on the laptop's monotonic clock (sim time in simulation), never
      from message header stamps: the Vicon source stamp may be offset from the laptop clock.
    - `cmd_age` is measured since the last `/tello/cmd`, or since entering FLYING if none has arrived yet.
    - `mocap_age` is measured since the last VALID pose (finite, accepted by the Vicon adapter), NOT since
      the last message. That way an invalid or NaN stream ages out and lands after `mocap_timeout_land`.
    - `position` and `velocity` are those of that last valid pose (None if there is none).
    """

    cmd_age: float
    mocap_age: float
    telemetry_age: float
    position: tuple[float, float, float] | None
    velocity: tuple[float, float, float] | None
    battery: float = math.nan


@dataclass
class SafetyDecision:
    action: Action
    events: list[str] = field(default_factory=list)


def _age(a: float) -> float:
    return math.inf if math.isnan(a) else a


def _finite3(v) -> bool:
    return v is not None and len(v) == 3 and all(math.isfinite(x) for x in v)


def evaluate(cfg: SafetyConfig, inp: SafetyInputs) -> SafetyDecision:
    """Evaluate all safety rules for one tick and return the most severe action with its active events.

    A non-finite or missing position gives HOVER as a defensive fallback; escalation to LAND relies on the
    `mocap_age` contract in `SafetyInputs`.
    """
    d = SafetyDecision(Action.NONE)

    def hit(action: Action, event: str) -> None:
        d.events.append(event)
        d.action = max(d.action, action)

    cmd_age, mocap_age, tel_age = _age(inp.cmd_age), _age(inp.mocap_age), _age(inp.telemetry_age)
    if cmd_age > cfg.cmd_timeout_land:
        hit(Action.LAND, "watchdog_land")
    elif cmd_age > cfg.cmd_timeout_hover:
        hit(Action.HOVER, "watchdog_hover")

    pos_ok = _finite3(inp.position)
    if mocap_age > cfg.mocap_timeout_land:
        hit(Action.LAND, "mocap_lost_land")
    elif mocap_age > cfg.mocap_timeout_hover or not pos_ok:
        hit(Action.HOVER, "mocap_lost_hover")
    elif not cfg.hard_box.contains(inp.position):
        hit(Action.LAND, "volume_hard_land")
    elif _finite3(inp.velocity):
        pred = tuple(p + v * cfg.lookahead for p, v in zip(inp.position, inp.velocity))
        if not cfg.hard_box.contains(pred):
            hit(Action.HOVER, "volume_predicted")

    if math.isfinite(inp.battery) and inp.battery <= cfg.battery_land:
        hit(Action.LAND, "battery_land")
    if tel_age > cfg.telemetry_timeout:
        hit(Action.LAND, "link_lost")
    return d


class EventLatch:
    """Reports an event only when it becomes active (rising edge)."""

    def __init__(self) -> None:
        self._active: set[str] = set()

    def update(self, events: list[str]) -> list[str]:
        new = [e for e in events if e not in self._active]
        self._active = set(events)
        return new
