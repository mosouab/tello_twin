"""Plan execution, one segment at a time (spec section 7.3).

Pure logic: flight_runner (ROS, Plan 2) and the dry run call `PlanExecutor.step()` every tick with an
Observation and act on the returned ExecOutput: sticks to publish, a takeoff/land request (emitted once),
a segment announcement, events, and `finished`.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable

from .control import Arena, ArenaParams, ControlConfig, GotoParams, goto_command, goto_reached
from .flight_state import FlightState
from .plan import Plan, Segment
from .safety import Box
from .signals import Signal, chirp, doublet, multisine, staircase, step
from .sticks import AXES, ZERO, Sticks, clamp_sticks

FS = FlightState


def _finite_vec(v, n: int) -> bool:
    return (v is not None and len(v) == n
            and all(isinstance(x, (int, float)) and math.isfinite(x) for x in v))


def _norm(v) -> float:
    return math.sqrt(sum(x * x for x in v))


@dataclass(frozen=True)
class Observation:
    t: float
    state: FlightState
    p: tuple[float, float, float] | None = None
    yaw: float | None = None
    v: tuple[float, float, float] | None = None
    yaw_rate: float | None = None

    @property
    def has_pose(self) -> bool:
        return _finite_vec(self.p, 3) and _finite_vec(self.v, 3) and _finite_vec((self.yaw, self.yaw_rate), 2)


@dataclass(frozen=True)
class SegmentStart:
    seq: int
    segment: Segment
    name: str


@dataclass
class ExecOutput:
    sticks: Sticks = ZERO
    request: str | None = None
    started: SegmentStart | None = None
    events: list[str] = field(default_factory=list)
    finished: bool = False


@dataclass(frozen=True)
class ExecutorConfig:
    control: ControlConfig = ControlConfig()
    stop_time: float = 0.5
    settle_speed: float = 0.1
    settle_time: float = 1.0

    @property
    def lookahead(self) -> float:
        return self.control.cmd_delay + self.stop_time


def sync_profile(p: dict) -> tuple[float, Callable[[float], Sticks]]:
    pieces = [(p["hover_pre"], ZERO), (p["yaw_time"], Sticks(yaw=p["yaw_amp"])),
              (p["yaw_time"], Sticks(yaw=-p["yaw_amp"])), (p["hover_mid"], ZERO),
              (p["z_time"], Sticks(z=p["z_amp"])), (p["z_time"], Sticks(z=-p["z_amp"])),
              (p["hover_post"], ZERO)]
    total = sum(d for d, _ in pieces)

    def u(t: float) -> Sticks:
        acc = 0.0
        for d, s in pieces:
            if t < acc + d:
                return s
            acc += d
        return ZERO

    return total, u


def excite_signals(p: dict) -> dict[str, Signal]:
    axes, sig = p["axes"], p["signal"]
    out: dict[str, Signal] = {}
    for j, ax in enumerate(axes):
        if sig == "staircase":
            out[ax] = staircase(p["amplitudes"], p["hold"])
            continue
        a = p["amplitude"][j]
        if sig == "step":
            out[ax] = step(a, p["duration"])
        elif sig == "doublet":
            out[ax] = doublet(a, p["half_period"])
        elif sig == "chirp":
            out[ax] = chirp(a, p["f_min"], p["f_max"], p["duration"])
        else:
            out[ax] = multisine(a, p["f_min"], p["f_max"], p["duration"], axis_index=j, n_axes=len(axes),
                                seed=p.get("seed"))
    return out


def nominal_duration(seg: Segment) -> float:
    """Upper bound used for time budgets: timeout for takeoff/land/goto, nominal length otherwise."""
    p = seg.params
    if seg.type in ("takeoff", "land", "goto"):
        return p["timeout"]
    if seg.type in ("hover", "arena"):
        return p["duration"]
    if seg.type == "sync":
        return sync_profile(p)[0]
    return max(s.duration for s in excite_signals(p).values())


class PlanExecutor:
    def __init__(self, plan: Plan, soft_box: Box, cfg: ExecutorConfig = ExecutorConfig()):
        self.plan, self.cfg = plan, cfg
        self.box = soft_box.shrink(plan.volume_shrink) if plan.volume_shrink > 0 else soft_box
        stick_max = tuple(min(a, b) for a, b in zip(plan.stick_max, cfg.control.stick_max))
        self.ctl = ControlConfig(cfg.control.stick_scale, cfg.control.cmd_delay, stick_max)
        self.finished = False
        self._i = -1
        self._seg: Segment | None = None
        self._rt: dict = {}
        self._t0 = 0.0
        self._seq = -1
        self._insert: Segment | None = None
        self._pending_next = False
        self._requested = False
        self._settle_since: float | None = None
        self._last_target: tuple[float, float, float] | None = None
        self._abort_reason: str | None = None

    def abort(self, reason: str = "operator") -> None:
        self._abort_reason = reason

    def step(self, obs: Observation) -> ExecOutput:
        out = ExecOutput()
        if self.finished:
            out.finished = True
            return out
        if self._abort_reason is not None:
            out.events.append(f"plan_aborted:{self._abort_reason}")
            out.request = "land" if obs.state in (FS.TAKING_OFF, FS.FLYING) else None
            self._finish(out)
            return out
        if self._seg is None or self._pending_next:
            self._pending_next = False
            self._start_next(obs, out)
            if out.finished:
                return out
        done = self._run(obs, out)
        if done and not out.finished:
            if out.started is None:
                self._start_next(obs, out)
            else:
                self._pending_next = True
        out.sticks = clamp_sticks(out.sticks, self.ctl.stick_max)
        return out

    # -- internals ---------------------------------------------------------------------------------
    def _finish(self, out: ExecOutput) -> None:
        self.finished = True
        out.finished = True

    def _fail(self, obs: Observation, out: ExecOutput, event: str) -> None:
        out.events.append(event)
        out.request = "land" if obs.state in (FS.TAKING_OFF, FS.FLYING) else None
        self._finish(out)

    def _start_next(self, obs: Observation, out: ExecOutput) -> None:
        if self._insert is not None:
            seg, name = self._insert, "recenter"
            self._insert = None
        else:
            self._i += 1
            if self._i >= len(self.plan.segments):
                self._finish(out)
                return
            seg = self.plan.segments[self._i]
            name = seg.name
        self._seg, self._t0, self._requested, self._settle_since = seg, obs.t, False, None
        self._rt = self._make_runtime(seg)
        self._seq += 1
        out.started = SegmentStart(self._seq, seg, name)

    def _make_runtime(self, seg: Segment) -> dict:
        p = seg.params
        if seg.type == "goto":
            gp = GotoParams(target=tuple(float(v) for v in p["target"]), yaw=p.get("yaw"), kp=p["kp"],
                            v_max=p["v_max"], tol=p["tol"], timeout=p["timeout"])
            if seg.index >= 0:
                self._last_target = gp.target
            return {"goto": gp}
        if seg.type == "sync":
            return {"sync": sync_profile(p)}
        if seg.type == "excite":
            sigs = excite_signals(p)
            return {"signals": sigs, "duration": max(s.duration for s in sigs.values())}
        if seg.type == "arena":
            return {"arena": Arena(ArenaParams(duration=p["duration"], seed=p["seed"], v_max=p["v_max"],
                                               u_exc=p["u_exc"]), self.ctl, self.box)}
        return {}

    def _recenter_params(self) -> dict:
        tgt = self._last_target or self.box.center()
        return {"target": list(tgt), "yaw": None, "kp": 0.8, "v_max": 0.4, "tol": 0.1, "timeout": 15.0}

    def _predicted_exit(self, obs: Observation) -> bool:
        pred = tuple(obs.p[i] + obs.v[i] * self.cfg.lookahead for i in range(3))
        return not self.box.contains(pred)

    def _run(self, obs: Observation, out: ExecOutput) -> bool:
        """Run the current segment for one tick; True when it is done."""
        seg, rt, p = self._seg, self._rt, self._seg.params
        t = obs.t - self._t0
        if seg.type == "takeoff":
            if not self._requested:
                out.request, self._requested = "takeoff", True
            if obs.state in (FS.EMERGENCY, FS.DISCONNECTED, FS.LANDING):
                self._fail(obs, out, "takeoff_failed")
                return False
            if obs.state == FS.FLYING and obs.has_pose and _norm(obs.v) < self.cfg.settle_speed:
                if self._settle_since is None:
                    self._settle_since = obs.t
                if obs.t - self._settle_since >= self.cfg.settle_time:
                    return True
            else:
                self._settle_since = None
            if t > p["timeout"]:
                self._fail(obs, out, "takeoff_timeout")
            return False
        if seg.type == "land":
            if not self._requested:
                out.request, self._requested = "land", True
            if obs.state == FS.LANDED:
                return True
            if t > p["timeout"]:
                self._fail(obs, out, "land_timeout")
            return False
        if obs.state != FS.FLYING:
            out.events.append("plan_aborted_by_failsafe")
            self._finish(out)
            return False
        if seg.type == "hover":
            return t >= p["duration"]
        if seg.type == "sync":
            total, u = rt["sync"]
            out.sticks = u(t)
            return t >= total
        if seg.type == "excite":
            if obs.has_pose and self._predicted_exit(obs):
                out.events.append("segment_truncated")
                self._insert = Segment(-1, "goto", self._recenter_params())
                return True
            sigs = rt["signals"]
            out.sticks = Sticks(*[sigs[a].value(t) if a in sigs else 0.0 for a in AXES])
            return t >= rt["duration"]
        if seg.type == "goto":
            gp = rt["goto"]
            if t > gp.timeout:
                self._fail(obs, out, "goto_timeout")
                return False
            if not obs.has_pose:
                return False
            if goto_reached(gp, obs.p, obs.yaw, obs.v):
                return True
            out.sticks = goto_command(gp, self.ctl, obs.p, obs.yaw, obs.v, obs.yaw_rate)
            return False
        if seg.type == "arena":
            if obs.has_pose:
                out.sticks = rt["arena"].step(obs.t, obs.p, obs.yaw, obs.v, obs.yaw_rate)
            return t >= p["duration"]
        raise ValueError(f"unknown segment type {seg.type}")
