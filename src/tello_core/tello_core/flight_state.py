"""Flight state machine and pre-flight checks (spec section 5.2)."""
from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass
from enum import IntEnum

from .safety import Box


class FlightState(IntEnum):
    """Values match the tello_interfaces/TelloStatus constants."""
    DISCONNECTED = 0
    LANDED = 1
    TAKING_OFF = 2
    FLYING = 3
    LANDING = 4
    EMERGENCY = 5


FS = FlightState


@dataclass(frozen=True)
class PreflightConfig:
    soft_box: Box
    battery_takeoff_min: float = 30.0
    mocap_max_age: float = 0.2
    mocap_min_rate_ratio: float = 0.9
    require_recording: bool = True

    def __post_init__(self) -> None:
        if not (math.isfinite(self.battery_takeoff_min) and 0 <= self.battery_takeoff_min <= 100):
            raise ValueError(f"PreflightConfig.battery_takeoff_min must be finite in [0, 100], "
                             f"got {self.battery_takeoff_min!r}")
        if not (math.isfinite(self.mocap_max_age) and self.mocap_max_age > 0):
            raise ValueError(f"PreflightConfig.mocap_max_age must be finite and > 0, got {self.mocap_max_age!r}")
        if not (0 < self.mocap_min_rate_ratio <= 1):
            raise ValueError(f"PreflightConfig.mocap_min_rate_ratio must be in (0, 1], "
                             f"got {self.mocap_min_rate_ratio!r}")
        if not all(math.isfinite(b) for b in dataclasses.astuple(self.soft_box)):
            raise ValueError(f"PreflightConfig.soft_box bounds must be finite, got {self.soft_box!r}")


@dataclass(frozen=True)
class PreflightInputs:
    connected: bool
    battery: float
    mocap_age: float
    position: tuple[float, float, float] | None
    mocap_rate: float
    mocap_expected_rate: float
    recording: bool


def preflight_failures(cfg: PreflightConfig, inp: PreflightInputs) -> list[str]:
    """Reasons why takeoff must be refused; empty list = OK. Position is checked horizontally only."""
    f: list[str] = []
    if not inp.connected:
        f.append("backend not connected")
    if not (math.isfinite(inp.battery) and inp.battery >= cfg.battery_takeoff_min):
        f.append(f"battery {inp.battery} % below {cfg.battery_takeoff_min} %")
    if not (math.isfinite(inp.mocap_age) and inp.mocap_age <= cfg.mocap_max_age):
        f.append("no recent Vicon pose")
    elif (inp.position is None or not all(math.isfinite(v) for v in inp.position)
          or not cfg.soft_box.contains_xy(inp.position)):
        f.append("drone not inside the soft volume")
    if not (math.isfinite(inp.mocap_expected_rate) and inp.mocap_expected_rate > 0):
        f.append("Vicon expected rate invalid")
    elif not inp.mocap_rate >= cfg.mocap_min_rate_ratio * inp.mocap_expected_rate:
        f.append(f"Vicon rate {inp.mocap_rate:.0f} Hz too low (expected {inp.mocap_expected_rate:.0f} Hz)")
    if cfg.require_recording and not inp.recording:
        f.append("recorder not running")
    return f


class FlightStateMachine:
    def __init__(self) -> None:
        self.state = FS.DISCONNECTED

    @property
    def accepts_commands(self) -> bool:
        return self.state == FS.FLYING

    def on_telemetry(self) -> bool:
        """First telemetry after (re)connection: DISCONNECTED -> LANDED."""
        if self.state == FS.DISCONNECTED:
            self.state = FS.LANDED
            return True
        return False

    def request_takeoff(self, failures: list[str]) -> tuple[bool, str]:
        if self.state != FS.LANDED:
            return False, f"cannot take off in state {self.state.name}"
        if failures:
            return False, "; ".join(failures)
        self.state = FS.TAKING_OFF
        return True, "taking off"

    def on_takeoff_done(self) -> bool:
        if self.state == FS.TAKING_OFF:
            self.state = FS.FLYING
            return True
        return False

    def request_land(self) -> tuple[bool, str]:
        if self.state in (FS.TAKING_OFF, FS.FLYING):
            self.state = FS.LANDING
            return True, "landing"
        if self.state == FS.LANDING:
            return True, "already landing"
        return False, f"cannot land in state {self.state.name}"

    def on_land_done(self) -> bool:
        if self.state == FS.LANDING:
            self.state = FS.LANDED
            return True
        return False

    def on_emergency(self) -> None:
        self.state = FS.EMERGENCY

    def on_link_lost(self) -> bool:
        """Go to DISCONNECTED. Returns True when a land command should be sent first (drone in the air)."""
        if self.state == FS.EMERGENCY:
            return False
        in_air = self.state in (FS.TAKING_OFF, FS.FLYING, FS.LANDING)
        self.state = FS.DISCONNECTED
        return in_air

    def reset(self) -> bool:
        """Leave EMERGENCY explicitly; the next telemetry brings the machine back to LANDED."""
        if self.state == FS.EMERGENCY:
            self.state = FS.DISCONNECTED
            return True
        return False
