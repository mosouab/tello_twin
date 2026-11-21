"""The decision core of tello_node (spec section 5).

It holds source priority, stick limits, SDK rounding, the flight state machine, pre-flight checks and the
safety rules. Pure: tello_node feeds it ARRIVAL times on the monotonic clock (sim time in simulation) and
executes what `tick()` returns. Services (takeoff, land, emergency, hold, reset) are methods; the node sends
the matching backend command when a takeoff or land request is accepted, and always for emergency.
"""
from __future__ import annotations

import collections
import math
from dataclasses import dataclass, field

from .flight_state import FlightState as FS
from .flight_state import FlightStateMachine, PreflightConfig, PreflightInputs, preflight_failures
from .safety import Action, EventLatch, SafetyConfig, SafetyInputs, evaluate
from .sdk import TelemetrySample
from .sticks import ZERO, Sticks, clamp_sticks, from_rc, to_rc


def _finite(v) -> bool:
    return all(isinstance(x, (int, float)) and math.isfinite(x) for x in v)


@dataclass(frozen=True)
class DroneConfig:
    safety: SafetyConfig
    preflight: PreflightConfig
    stick_max: tuple[float, float, float, float] = (0.4, 0.4, 0.4, 0.4)
    mocap_expected_rate: float = 100.0
    land_z: float = 0.15            # fallback LANDING -> LANDED: Vicon height below land_z ...
    land_settle: float = 1.0        # ... continuously for land_settle s (the backend's land_done is preferred)
    rate_window: float = 1.0        # s, window for the Vicon rate

    def __post_init__(self) -> None:
        if len(self.stick_max) != 4 or not all(0 < v <= 1 for v in self.stick_max):
            raise ValueError(f"DroneConfig.stick_max must be 4 values in (0, 1], got {self.stick_max!r}")
        for name in ("mocap_expected_rate", "rate_window", "land_settle"):
            v = getattr(self, name)
            if not (math.isfinite(v) and v > 0):
                raise ValueError(f"DroneConfig.{name} must be finite and > 0, got {v!r}")
        if not math.isfinite(self.land_z):
            raise ValueError(f"DroneConfig.land_z must be finite, got {self.land_z!r}")


@dataclass(frozen=True)
class Status:
    state: FS
    link_ok: bool
    hold: bool
    battery: float
    cmd_source: str                 # none | runner | hold | safety
    mocap_age: float
    mocap_rate: float
    recording: bool

    def key(self) -> tuple:
        """The fields whose change triggers an immediate status publication."""
        return (self.state, self.link_ok, self.hold, self.cmd_source, self.recording)


@dataclass
class TickOutput:
    rc: tuple[int, int, int, int]
    sent: Sticks                    # exactly what is sent: from_rc(rc)
    actions: list[str]              # backend commands to issue now ("land")
    events: list[tuple[str, str]]   # (type, detail), newly raised
    status: Status
    status_changed: bool


class DroneCore:
    def __init__(self, cfg: DroneConfig):
        self.cfg = cfg
        self.sm = FlightStateMachine()
        self.connected = False
        self.hold = False
        self.recording = False
        self.battery = math.nan
        self._latch = EventLatch()
        self._events: list[tuple[str, str]] = []
        self._t_cmd: float | None = None
        self._cmd = ZERO
        self._t_pose: float | None = None
        self._pose = None               # (p, yaw, v) of the last VALID pose
        self._pose_times: collections.deque = collections.deque()
        self._t_tel: float | None = None
        self._t_flying: float | None = None
        self._t_low: float | None = None
        self._last_key = None

    # ---------------------------------------------------------------- inputs (t = arrival time)
    def on_connected(self, t: float) -> None:
        self.connected = True

    def on_disconnected(self, t: float, detail: str = "") -> None:
        self.connected = False
        self._event("backend_disconnected", detail)

    def on_cmd(self, t: float, sticks: Sticks) -> None:
        self._t_cmd, self._cmd = t, sticks

    def on_pose(self, t: float, p, yaw: float, v) -> bool:
        """A pose from the Vicon adapter. Non-finite poses are ignored, so the Vicon age keeps growing."""
        if not (len(p) == 3 and len(v) == 3 and _finite([*p, yaw, *v])):
            return False
        self._t_pose = t
        self._pose = (tuple(float(x) for x in p), float(yaw), tuple(float(x) for x in v))
        self._pose_times.append(t)
        return True

    def on_telemetry(self, t: float, sample: TelemetrySample) -> None:
        self._t_tel = t
        if math.isfinite(sample.battery):
            self.battery = sample.battery
        self.sm.on_telemetry()

    def on_backend_event(self, t: float, kind: str, detail: str = "") -> None:
        if kind == "takeoff_done":
            if self.sm.on_takeoff_done():
                self._t_flying = t
        elif kind == "land_done":
            self.sm.on_land_done()
        elif kind != "connected":
            self._event(f"backend_{kind}", detail)

    def set_recording(self, flag: bool) -> None:
        self.recording = bool(flag)

    # ---------------------------------------------------------------- requests (services)
    def request_takeoff(self, t: float) -> tuple[bool, str]:
        """Pre-flight checks, plus no takeoff while hold is on (hold is a level, spec 15.17)."""
        failures = preflight_failures(self.cfg.preflight, PreflightInputs(
            connected=self.connected, battery=self.battery, mocap_age=self._age(t, self._t_pose),
            position=self._pose[0] if self._pose else None, mocap_rate=self.mocap_rate(t),
            mocap_expected_rate=self.cfg.mocap_expected_rate, recording=self.recording))
        if self.hold:
            failures.insert(0, "hold active")
        ok, msg = self.sm.request_takeoff(failures)
        if not ok:
            self._event("takeoff_refused", msg)
        return ok, msg

    def request_land(self, t: float) -> tuple[bool, str]:
        return self.sm.request_land()

    def emergency(self, t: float) -> None:
        self.sm.on_emergency()
        self._event("emergency", "")

    def reset(self, t: float) -> bool:
        ok = self.sm.reset()
        if ok:
            self._event("reset", "")
        return ok

    def set_hold(self, t: float, on: bool) -> None:
        if bool(on) != self.hold:
            self.hold = bool(on)
            self._event("hold_on" if self.hold else "hold_off", "")

    # ---------------------------------------------------------------- helpers
    @staticmethod
    def _age(t: float, t_last: float | None) -> float:
        return math.inf if t_last is None else t - t_last

    def mocap_rate(self, t: float) -> float:
        while self._pose_times and self._pose_times[0] <= t - self.cfg.rate_window:
            self._pose_times.popleft()
        return len(self._pose_times) / self.cfg.rate_window

    def _event(self, kind: str, detail: str) -> None:
        self._events.append((kind, detail))

    # ---------------------------------------------------------------- periodic
    def tick(self, t: float) -> TickOutput:
        actions: list[str] = []
        safety = self.cfg.safety
        tel_age = self._age(t, self._t_tel)
        mocap_age = self._age(t, self._t_pose)
        if self.sm.state not in (FS.DISCONNECTED, FS.EMERGENCY) and tel_age > safety.telemetry_timeout:
            if self.sm.on_link_lost():
                actions.append("land")
            self._event("link_lost", f"no telemetry for {tel_age:.2f} s")
        low = (self.sm.state == FS.LANDING and self._pose is not None and mocap_age <= safety.mocap_timeout_hover
               and self._pose[0][2] < self.cfg.land_z)
        if not low:
            self._t_low = None
        elif self._t_low is None:
            self._t_low = t
        elif t - self._t_low >= self.cfg.land_settle:
            self.sm.on_land_done()
            self._t_low = None

        sticks, source = ZERO, "none"
        if self.sm.state == FS.FLYING:
            refs = [x for x in (self._t_cmd, self._t_flying) if x is not None]
            cmd_age = t - max(refs) if refs else math.inf
            pose = self._pose
            d = evaluate(safety, SafetyInputs(cmd_age=cmd_age, mocap_age=mocap_age, telemetry_age=tel_age,
                                              position=pose[0] if pose else None,
                                              velocity=pose[2] if pose else None, battery=self.battery))
            for e in self._latch.update(d.events):
                self._event(e, "")
            fresh_cmd = self._t_cmd is not None and (self._t_flying is None or self._t_cmd >= self._t_flying)
            if d.action == Action.LAND:
                ok, _ = self.sm.request_land()
                if ok:
                    actions.append("land")
                source = "safety"
            elif d.action == Action.HOVER:
                source = "safety"
            elif self.hold:
                source = "hold"
            elif fresh_cmd:
                sticks, source = self._cmd, "runner"
        else:
            self._latch.update([])

        rc = to_rc(clamp_sticks(sticks, self.cfg.stick_max))
        status = Status(state=self.sm.state, link_ok=tel_age <= safety.telemetry_timeout, hold=self.hold,
                        battery=self.battery, cmd_source=source, mocap_age=mocap_age,
                        mocap_rate=self.mocap_rate(t), recording=self.recording)
        changed = status.key() != self._last_key
        self._last_key = status.key()
        events, self._events = self._events, []
        return TickOutput(rc=rc, sent=from_rc(rc), actions=actions, events=events, status=status,
                          status_changed=changed)
