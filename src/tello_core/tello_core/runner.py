"""flight_runner's decision core (spec sections 7.3 and 15), plus the flight.yaml record.

Phases: waiting -> running -> [settling ->] done.
- **waiting:** RunnerCore waits until tello_node reports LANDED, no hold, a fresh Vicon pose at the required
  rate, and recording (when required), so the takeoff it requests passes the pre-flight checks. After
  `ready_timeout` it gives up: "aborted:hold" if the last status showed hold, otherwise "not_ready".
- **running:** it drives a PlanExecutor:
  - the pose handed to the executor becomes None once it is older than `stale_pose` (Plan 1 staleness contract);
  - hold is a level: any status showing hold aborts the plan ("plan_aborted:hold"; the first reason wins);
  - a last status older than `status_timeout` aborts the plan ("plan_aborted:status_lost");
  - a refused takeoff aborts the plan ("plan_aborted:takeoff_refused");
  - commands are published only while FLYING.
- **settling:** when the plan finishes (completed, aborted, failed or failsafe) while the last status is
  airborne (TAKING_OFF, FLYING, LANDING), the runner sends nothing and keeps accepting status and events until
  the status shows LANDED, EMERGENCY or DISCONNECTED, or `settle_timeout` passes. If the drone is not airborne
  when the plan finishes, the run is finalised at once.
- **done (finalisation):** `result` is the outcome decided when the plan finished (it stays None until then).
  A settling cut short by the timeout adds one "settle_timeout" event, emitted and counted like any other.
  `battery_end` is taken at finalisation. `interrupt()` finalises at once in any phase.
Events: every event `tick()` emits is counted in `event_counts` at once, so the runner's own final event is in
the record even if the run ends in the same tick. The ROS node passes the echo of its own events back through
`on_event(kind, own=True)`, which does not count them again.
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
AIRBORNE = frozenset({FS.TAKING_OFF, FS.FLYING, FS.LANDING})


@dataclass(frozen=True)
class RunnerConfig:
    stale_pose: float = 0.2
    ready_timeout: float = 10.0
    require_recording: bool = True
    mocap_max_age: float = 0.2          # same thresholds as the pre-flight checks
    mocap_min_rate: float = 90.0
    status_timeout: float = 0.5         # running: abort (status_lost) when the last /tello/status is older
    settle_timeout: float = 20.0        # settling: finalise anyway after this long

    def __post_init__(self) -> None:
        for name in ("stale_pose", "ready_timeout", "mocap_max_age", "mocap_min_rate", "status_timeout",
                     "settle_timeout"):
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
        self.phase = "waiting"                  # waiting | running | settling | done
        self.result: str | None = None          # set at finalisation
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
        self._t_status: float | None = None
        self._t_settle: float | None = None
        self._t_pose: float | None = None
        self._pose = (None, None, None, None)
        self._abort_reason: str | None = None
        self._outcome: str | None = None        # decided when the plan finishes, reported at finalisation
        self._exec_events: list[str] = []

    # ---------------------------------------------------------------- inputs
    def on_status(self, t: float, state: FS, hold: bool, recording: bool, battery: float,
                  mocap_age: float = math.inf, mocap_rate: float = 0.0) -> None:
        self._t_status = t
        if hold and self.phase == "running":
            self.abort("hold")
        self.state, self.hold, self.recording = state, bool(hold), bool(recording)
        self.mocap_age, self.mocap_rate = mocap_age, mocap_rate
        if math.isfinite(battery):
            self.battery = battery
            if math.isnan(self.battery_start):
                self.battery_start = battery

    def on_pose(self, t: float, p, yaw: float, v, yaw_rate: float) -> None:
        self._t_pose = t
        self._pose = (tuple(p), yaw, tuple(v), yaw_rate)

    def on_event(self, kind: str, own: bool = False) -> None:
        """An event from /tello/events. `own=True` marks the echo of an event this runner emitted: tick() already
        counted it, so it is not counted again."""
        if not own:
            self.event_counts[kind] += 1

    def on_takeoff_result(self, ok: bool, msg: str = "") -> None:
        if not ok:
            self.abort("takeoff_refused")

    def abort(self, reason: str = "operator") -> None:
        """Abort the plan. The first reason wins; repeated aborts are harmless."""
        self._abort_reason = self._abort_reason or reason
        self.executor.abort(self._abort_reason)

    def interrupt(self, t: float, reason: str = "interrupted") -> None:
        """Finalise the run now (node shutdown), whatever the drone is doing. Before the plan has finished:
        aborted:<reason> (an earlier abort reason wins). While settling: the outcome decided when the plan
        finished. No effect once the run is done."""
        if self.phase == "done":
            return
        if self.phase != "settling":
            self.abort(reason)
            self._outcome = f"aborted:{self._abort_reason}"
        self._finish(RunnerOutput())

    # ---------------------------------------------------------------- periodic
    def _finish(self, out: RunnerOutput) -> None:
        self.phase, self.result = "done", self._outcome
        self.battery_end = self.battery
        out.finished = True

    def tick(self, t: float) -> RunnerOutput:
        out = self._step(t)
        self.event_counts.update(out.events)     # own events are counted when emitted (see on_event)
        return out

    def _step(self, t: float) -> RunnerOutput:
        out = RunnerOutput()
        if self._t_start is None:
            self._t_start = t
        if self.phase == "done":
            out.finished = True
            return out
        if self.phase == "settling":
            if self.state not in AIRBORNE:
                self._finish(out)
            elif t - self._t_settle >= self.cfg.settle_timeout:
                out.events.append("settle_timeout")
                self._finish(out)
            return out
        if self.phase == "waiting":
            if self._abort_reason is not None:
                self._outcome = f"aborted:{self._abort_reason}"
                self._finish(out)
                return out
            ready = (self.state == FS.LANDED and not self.hold
                     and (self.recording or not self.cfg.require_recording)
                     and self.mocap_age <= self.cfg.mocap_max_age and self.mocap_rate >= self.cfg.mocap_min_rate)
            if not ready:
                if t - self._t_start > self.cfg.ready_timeout:
                    if self.hold:
                        self.abort("hold")
                        out.events.append("plan_aborted:hold")
                        self._outcome = "aborted:hold"
                    else:
                        out.events.append("runner_not_ready")
                        self._outcome = "not_ready"
                    self._finish(out)
                return out
            self.phase = "running"
        if self._t_status is None or t - self._t_status > self.cfg.status_timeout:
            self.abort("status_lost")
        fresh = self._t_pose is not None and t - self._t_pose <= self.cfg.stale_pose
        p, yaw, v, r = self._pose if fresh else (None, None, None, None)
        e = self.executor.step(Observation(t, self.state, p, yaw, v, r))
        out.sticks, out.request, out.started = e.sticks, e.request, e.started
        out.events = list(e.events)
        self._exec_events += e.events
        if e.started is not None:
            self.segments_started += 1
        if e.finished:
            self._outcome = outcome(self._exec_events)
            if self.state in AIRBORNE:
                self.phase, self._t_settle = "settling", t
            else:
                self._finish(out)
        else:
            out.publish_cmd = self.state == FS.FLYING
        return out


def flight_record(*, flight_id: str, session: str, started: str, plan_path: str, plan: Plan, backend: str,
                  drone_info: dict, lab, soft_box: Box, hard_box: Box, notes: str, software: str,
                  runner: RunnerCore | None = None, ended: str | None = None,
                  start_stamp: float | None = None) -> dict:
    """The flight.yaml content (spec section 8). `lab` is a tello_core.lab.LabConfig.

    `start_stamp` is the runner node's start time on the ROS clock (float s): event counts only include events
    stamped at or after it, and the dataset builder applies the same cut to the bag (spec 15.15). None -> null.
    """
    counts = dict(runner.event_counts) if runner else {}
    return {
        "flight_id": flight_id,
        "session": session,
        "started": started,
        "ended": ended,
        "start_stamp": start_stamp,
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
