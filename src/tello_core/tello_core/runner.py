"""flight_runner's decision core (spec section 7.3), plus the flight.yaml record.

RunnerCore waits until tello_node reports LANDED with a fresh Vicon pose at the required rate (and recording,
when required), so the takeoff it requests passes the pre-flight checks. Then it drives a PlanExecutor:
- the pose handed to the executor becomes None once it is older than `stale_pose` (Plan 1 staleness contract);
- a rising `hold` in /tello/status aborts the plan ("plan_aborted:hold");
- a refused takeoff aborts the plan ("plan_aborted:takeoff_refused");
- commands are published only while FLYING.
Pure: the ROS node passes arrival times on the monotonic clock.
"""
from __future__ import annotations

import collections
import math
from dataclasses import dataclass, field

from .executor import ExecutorConfig, Observation, PlanExecutor, SegmentStart
from .flight_state import FlightState as FS
from .plan import Plan
from .safety import Box
from .sticks import ZERO, Sticks

SAFETY_EVENTS = frozenset({"watchdog_hover", "watchdog_land", "mocap_lost_hover", "mocap_lost_land",
                           "volume_predicted", "volume_hard_land", "battery_land", "link_lost", "emergency"})
FAILURE_EVENTS = ("takeoff_timeout", "takeoff_failed", "land_timeout", "goto_timeout")


@dataclass(frozen=True)
class RunnerConfig:
    stale_pose: float = 0.2
    ready_timeout: float = 10.0
    require_recording: bool = True
    mocap_max_age: float = 0.2          # same thresholds as the pre-flight checks
    mocap_min_rate: float = 90.0

    def __post_init__(self) -> None:
        for name in ("stale_pose", "ready_timeout", "mocap_max_age", "mocap_min_rate"):
            v = getattr(self, name)
            if not (math.isfinite(v) and v > 0):
                raise ValueError(f"RunnerConfig.{name} must be finite and > 0, got {v!r}")


@dataclass
class RunnerOutput:
    publish_cmd: bool = False
    sticks: Sticks = ZERO
    request: str | None = None              # "takeoff" | "land"
    started: SegmentStart | None = None
    events: list[str] = field(default_factory=list)
    finished: bool = False


def outcome(events: list[str]) -> str:
    for e in events:
        if e.startswith("plan_aborted:"):
            return "aborted:" + e.split(":", 1)[1]
        if e == "plan_aborted_by_failsafe":
            return "aborted:failsafe"
    for e in events:
        if e in FAILURE_EVENTS:
            return "failed:" + e
    return "completed"


class RunnerCore:
    def __init__(self, plan: Plan, soft_box: Box, exec_cfg: ExecutorConfig = ExecutorConfig(),
                 cfg: RunnerConfig = RunnerConfig()):
        self.executor = PlanExecutor(plan, soft_box, exec_cfg)
        self.cfg = cfg
        self.phase = "waiting"                  # waiting | running | done
        self.result: str | None = None
        self.state: FS | None = None
        self.hold = False
        self.recording = False
        self.mocap_age = math.inf
        self.mocap_rate = 0.0
        self.battery = math.nan
        self.battery_start = math.nan
        self.battery_end = math.nan
        self.event_counts: collections.Counter = collections.Counter()
        self.segments_started = 0
        self._t_start: float | None = None
        self._t_pose: float | None = None
        self._pose = (None, None, None, None)
        self._abort_reason: str | None = None
        self._exec_events: list[str] = []

    # ---------------------------------------------------------------- inputs
    def on_status(self, t: float, state: FS, hold: bool, recording: bool, battery: float,
                  mocap_age: float = math.inf, mocap_rate: float = 0.0) -> None:
        if hold and not self.hold and self.phase == "running":
            self.executor.abort("hold")
        self.state, self.hold, self.recording = state, bool(hold), bool(recording)
        self.mocap_age, self.mocap_rate = mocap_age, mocap_rate
        if math.isfinite(battery):
            self.battery = battery
            if math.isnan(self.battery_start):
                self.battery_start = battery

    def on_pose(self, t: float, p, yaw: float, v, yaw_rate: float) -> None:
        self._t_pose = t
        self._pose = (tuple(p), yaw, tuple(v), yaw_rate)

    def on_event(self, kind: str) -> None:
        self.event_counts[kind] += 1

    def on_takeoff_result(self, ok: bool, msg: str = "") -> None:
        if not ok:
            self.abort("takeoff_refused")

    def abort(self, reason: str = "operator") -> None:
        self._abort_reason = self._abort_reason or reason
        self.executor.abort(reason)

    # ---------------------------------------------------------------- periodic
    def _finish(self, out: RunnerOutput, result: str) -> None:
        self.phase, self.result = "done", result
        self.battery_end = self.battery
        out.finished = True

    def tick(self, t: float) -> RunnerOutput:
        out = RunnerOutput()
        if self._t_start is None:
            self._t_start = t
        if self.phase == "done":
            out.finished = True
            return out
        if self.phase == "waiting":
            if self._abort_reason is not None:
                self._finish(out, f"aborted:{self._abort_reason}")
                return out
            ready = (self.state == FS.LANDED and (self.recording or not self.cfg.require_recording)
                     and self.mocap_age <= self.cfg.mocap_max_age and self.mocap_rate >= self.cfg.mocap_min_rate)
            if not ready:
                if t - self._t_start > self.cfg.ready_timeout:
                    out.events.append("runner_not_ready")
                    self._finish(out, "not_ready")
                return out
            self.phase = "running"
        fresh = self._t_pose is not None and t - self._t_pose <= self.cfg.stale_pose
        p, yaw, v, r = self._pose if fresh else (None, None, None, None)
        e = self.executor.step(Observation(t, self.state, p, yaw, v, r))
        out.sticks, out.request, out.started = e.sticks, e.request, e.started
        out.events = list(e.events)
        self._exec_events += e.events
        if e.started is not None:
            self.segments_started += 1
        if e.finished:
            self._finish(out, outcome(self._exec_events))
        else:
            out.publish_cmd = self.state == FS.FLYING
        return out


def flight_record(*, flight_id: str, session: str, started: str, plan_path: str, plan: Plan, backend: str,
                  drone_info: dict, lab, soft_box: Box, hard_box: Box, notes: str, software: str,
                  runner: RunnerCore | None = None, ended: str | None = None) -> dict:
    """The flight.yaml content (spec section 8). `lab` is a tello_core.lab.LabConfig."""
    counts = dict(runner.event_counts) if runner else {}
    return {
        "flight_id": flight_id,
        "session": session,
        "started": started,
        "ended": ended,
        "result": runner.result if runner and runner.result else "running",
        "plan": {"path": plan_path, "name": plan.name, "sha256": plan.source_sha256,
                 "volume_shrink": plan.volume_shrink, "stick_max": list(plan.stick_max)},
        "backend": backend,
        "drone": {"serial": drone_info.get("serial", "unknown"), "sdk": drone_info.get("sdk", "unknown")},
        "battery": {"start": None if not runner or math.isnan(runner.battery_start) else runner.battery_start,
                    "end": None if not runner or math.isnan(runner.battery_end) else runner.battery_end},
        "vicon": {"source": lab.mocap.source, "topic": lab.mocap.raw_topic, "type": lab.mocap.raw_type,
                  "rigid_body": lab.mocap.rigid_body, "marker_layout": lab.mocap.marker_layout},
        "volume": {"soft_box": [soft_box.xmin, soft_box.xmax, soft_box.ymin, soft_box.ymax, soft_box.zmin,
                                soft_box.zmax],
                   "hard_box": [hard_box.xmin, hard_box.xmax, hard_box.ymin, hard_box.ymax, hard_box.zmin,
                                hard_box.zmax]},
        "limits": {"stick_max": list(lab.control.stick_max), "cmd_delay": lab.control.cmd_delay},
        "setup": dict(lab.setup),
        "software": software,
        "notes": notes,
        "segments": runner.segments_started if runner else 0,
        "events": counts,
        "safety_event_count": sum(n for k, n in counts.items() if k in SAFETY_EVENTS),
    }
