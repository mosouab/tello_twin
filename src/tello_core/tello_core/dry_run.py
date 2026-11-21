"""Dry run of a flight plan against the fake dynamics (spec section 7.3).

Run before every flight. It fails on any truncated segment, timeout or abort, on leaving the soft box
while flying, on leaving the hard box, or if the plan does not finish within its time budget.
"""
from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass, field

from .control import ControlConfig
from .executor import ExecutorConfig, Observation, PlanExecutor, nominal_duration
from .fake import FakeDynamics, FakeParams
from .flight_state import FlightState as FS
from .plan import Plan, PlanError, load_plan
from .safety import Box
from .sticks import AXES, to_rc

STATE_OF_MODE = {"landed": FS.LANDED, "taking_off": FS.TAKING_OFF, "flying": FS.FLYING,
                 "landing": FS.LANDING, "falling": FS.EMERGENCY, "off": FS.EMERGENCY}
FAIL_EVENTS = {"segment_truncated", "goto_timeout", "takeoff_timeout", "takeoff_failed", "land_timeout"}


@dataclass
class DryRunReport:
    ok: bool
    problems: list[str] = field(default_factory=list)
    duration: float = 0.0
    events: list[str] = field(default_factory=list)
    max_outside: float = 0.0


def _outside(box: Box, p) -> float:
    dx = max(box.xmin - p[0], 0.0, p[0] - box.xmax)
    dy = max(box.ymin - p[1], 0.0, p[1] - box.ymax)
    dz = max(box.zmin - p[2], 0.0, p[2] - box.zmax)
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def dry_run(plan: Plan, soft_box: Box, hard_box: Box, fake_params: FakeParams = FakeParams(),
            exec_cfg: ExecutorConfig = ExecutorConfig(), start=None, dt: float = 0.02,
            max_time: float | None = None) -> DryRunReport:
    c = soft_box.center()
    problems: list[str] = []
    for i, axis in enumerate(AXES):
        if plan.stick_max[i] > exec_cfg.control.stick_max[i] + 1e-12:
            problems.append(f"limits.stick_max[{axis}] = {plan.stick_max[i]} exceeds the bring-up limit "
                            f"{exec_cfg.control.stick_max[i]}")
    try:
        ex = PlanExecutor(plan, soft_box, exec_cfg)
        if plan.volume_shrink > 0:
            soft, hard = soft_box.shrink(plan.volume_shrink), hard_box.shrink(plan.volume_shrink)
        else:
            soft, hard = soft_box, hard_box
    except ValueError as e:
        return DryRunReport(ok=False, problems=[*problems, str(e)])
    fake = FakeDynamics(fake_params, position=start or (c[0], c[1], 0.0), t0=0.0)
    limit = max_time if max_time is not None else sum(nominal_duration(s) for s in plan.segments) + 30.0
    rep = DryRunReport(ok=False, problems=problems)
    t, finished, left_hard = 0.0, False, False
    while t < limit:
        tr = fake.truth()
        out = ex.step(Observation(t, STATE_OF_MODE[fake.mode], tr.p, tr.yaw, tr.v_world, tr.yaw_rate))
        rep.events += out.events
        if out.request == "takeoff":
            fake.takeoff(t)
        elif out.request == "land":
            fake.land(t)
        fake.set_rc(t, to_rc(out.sticks))
        if out.finished:
            finished = True
            break
        t = round(t + dt, 9)
        fake.step(t)
        p = fake.truth().p
        if fake.mode == "flying":
            rep.max_outside = max(rep.max_outside, _outside(soft, p))
        outside_hard = (not hard.contains(p)) if fake.mode == "flying" else (
            fake.mode in ("taking_off", "landing", "falling") and not hard.contains_xy(p))
        if outside_hard and not left_hard:
            left_hard = True
            rep.problems.append(f"left the hard box at t={t:.1f} s, position {tuple(round(v, 2) for v in p)}")
    rep.duration = t
    if not finished:
        rep.problems.append(f"plan did not finish within {limit:.0f} s")
    for e in rep.events:
        if e in FAIL_EVENTS or e.startswith("plan_aborted"):
            rep.problems.append(f"event {e}")
    if rep.max_outside > 0.05:
        rep.problems.append(f"left the soft box by {rep.max_outside:.2f} m while flying")
    rep.ok = not rep.problems
    return rep


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Dry-run a Tello flight plan on the fake dynamics.")
    ap.add_argument("plan")
    meta = ("XMIN", "XMAX", "YMIN", "YMAX", "ZMIN", "ZMAX")
    ap.add_argument("--soft-box", nargs=6, type=float, required=True, metavar=meta)
    ap.add_argument("--hard-box", nargs=6, type=float, required=True, metavar=meta)
    ap.add_argument("--stick-max", nargs=4, type=float, default=[0.4, 0.4, 0.4, 0.4],
                    metavar=("X", "Y", "Z", "YAW"), help="bring-up stick limits (lab values)")
    ap.add_argument("--cmd-delay", type=float, default=0.7, help="bring-up command delay in s (lab value)")
    a = ap.parse_args(argv)
    try:
        plan = load_plan(a.plan)
    except PlanError as e:
        print(e)
        return 2
    try:
        cfg = ExecutorConfig(control=ControlConfig(stick_max=tuple(a.stick_max), cmd_delay=a.cmd_delay))
        r = dry_run(plan, Box.from_list(a.soft_box), Box.from_list(a.hard_box), exec_cfg=cfg)
    except ValueError as e:
        print(f"FAILED: {e}")
        return 1
    print(f"{'OK' if r.ok else 'FAILED'}: {plan.name}, {r.duration:.1f} s simulated")
    for p in r.problems:
        print("  -", p)
    return 0 if r.ok else 1


if __name__ == "__main__":
    sys.exit(main())
