import math
from pathlib import Path

import pytest

from tello_core.flight_state import FlightState as FS
from tello_core.lab import load_lab
from tello_core.plan import parse_plan
from tello_core.runner import RunnerConfig, RunnerCore, flight_record, outcome
from tello_core.safety import Box

DATA = Path(__file__).parent / "data"
SOFT = Box(-2.0, 2.0, -2.0, 2.0, 0.5, 2.0)


def plan_of(*segments):
    return parse_plan({"name": "t", "limits": {"stick_max": [0.4] * 4},
                       "segments": [{"type": "takeoff"}, *segments, {"type": "land"}]})


def core(*segments, **cfg):
    return RunnerCore(plan_of(*segments), SOFT, cfg=RunnerConfig(**cfg))


def status(rc, t, state, hold=False, recording=True, battery=80.0, mocap_age=0.01, mocap_rate=100.0):
    rc.on_status(t, state, hold, recording, battery, mocap_age, mocap_rate)


def to_goto(rc):
    """Drive the takeoff segment to completion; the goto segment starts at t = 2.0."""
    status(rc, 0.0, FS.LANDED)
    out = rc.tick(0.0)
    assert out.request == "takeoff"
    status(rc, 0.1, FS.TAKING_OFF)
    rc.tick(0.1)
    status(rc, 1.0, FS.FLYING)
    rc.on_pose(1.0, (0.0, 0.0, 1.0), 0.0, (0.0, 0.0, 0.0), 0.0)
    rc.tick(1.0)
    rc.on_pose(2.0, (0.0, 0.0, 1.0), 0.0, (0.0, 0.0, 0.0), 0.0)
    out = rc.tick(2.0)
    assert out.started.segment.type == "goto"
    return out


def test_waits_for_landed_and_recording():
    rc = core({"type": "hover", "duration": 1.0})
    status(rc, 0.0, FS.DISCONNECTED)
    assert rc.tick(0.0).request is None
    status(rc, 0.5, FS.LANDED, recording=False)
    assert rc.tick(0.5).request is None and rc.phase == "waiting"
    status(rc, 1.0, FS.LANDED, recording=True)
    out = rc.tick(1.0)
    assert out.request == "takeoff" and out.started.segment.type == "takeoff" and rc.phase == "running"


def test_waits_for_a_fresh_vicon_stream():
    rc = core({"type": "hover", "duration": 1.0})
    status(rc, 0.0, FS.LANDED, mocap_rate=50.0)
    assert rc.tick(0.0).request is None
    status(rc, 0.5, FS.LANDED, mocap_age=0.5)
    assert rc.tick(0.5).request is None
    status(rc, 1.0, FS.LANDED)
    assert rc.tick(1.0).request == "takeoff"


def test_recording_can_be_optional():
    rc = core({"type": "hover", "duration": 1.0}, require_recording=False)
    status(rc, 0.0, FS.LANDED, recording=False)
    assert rc.tick(0.0).request == "takeoff"


def test_not_ready_times_out():
    rc = core({"type": "hover", "duration": 1.0}, ready_timeout=2.0)
    rc.tick(0.0)
    out = rc.tick(2.5)
    assert out.finished and "runner_not_ready" in out.events and rc.result == "not_ready"


def test_stale_pose_reaches_the_executor_as_none():
    rc = core({"type": "goto", "target": [1.0, 0.0, 1.0]})
    to_goto(rc)
    rc.on_pose(2.02, (0.0, 0.0, 1.0), 0.0, (0.0, 0.0, 0.0), 0.0)
    assert rc.tick(2.04).sticks.x > 0              # fresh pose: goto commands forward
    assert rc.tick(2.30).sticks.x == 0.0           # pose 0.28 s old: treated as missing


def test_hold_rising_edge_aborts_and_lands():
    rc = core({"type": "hover", "duration": 10.0})
    status(rc, 0.0, FS.LANDED)
    rc.tick(0.0)
    status(rc, 1.0, FS.FLYING)
    rc.on_pose(1.0, (0.0, 0.0, 1.0), 0.0, (0.0, 0.0, 0.0), 0.0)
    rc.tick(1.0)
    status(rc, 1.5, FS.FLYING, hold=True)
    out = rc.tick(1.5)
    assert out.finished and out.request == "land" and rc.result == "aborted:hold"


def test_refused_takeoff_aborts_without_landing():
    rc = core({"type": "hover", "duration": 1.0})
    status(rc, 0.0, FS.LANDED)
    rc.tick(0.0)
    rc.on_takeoff_result(False, "no recent Vicon pose")
    out = rc.tick(0.1)
    assert out.finished and out.request is None and rc.result == "aborted:takeoff_refused"


def test_operator_abort_while_waiting():
    rc = core({"type": "hover", "duration": 1.0})
    rc.abort("operator")
    assert rc.tick(0.0).finished and rc.result == "aborted:operator"


def test_commands_published_only_while_flying():
    rc = core({"type": "goto", "target": [1.0, 0.0, 1.0]})
    status(rc, 0.0, FS.LANDED)
    assert rc.tick(0.0).publish_cmd is False
    status(rc, 0.1, FS.TAKING_OFF)
    assert rc.tick(0.1).publish_cmd is False
    status(rc, 1.0, FS.FLYING)
    rc.on_pose(1.0, (0.0, 0.0, 1.0), 0.0, (0.0, 0.0, 0.0), 0.0)
    assert rc.tick(1.0).publish_cmd is True


def test_battery_start_and_end_are_recorded():
    rc = core({"type": "hover", "duration": 0.5})
    status(rc, 0.0, FS.LANDED, battery=90.0)
    rc.tick(0.0)
    status(rc, 0.1, FS.LANDED, battery=85.0)
    rc.abort("operator")
    rc.tick(0.2)
    assert rc.battery_start == 90.0 and rc.battery_end == 85.0


@pytest.mark.parametrize("events, result", [
    ([], "completed"),
    (["segment_truncated"], "completed"),
    (["plan_aborted:hold"], "aborted:hold"),
    (["plan_aborted_by_failsafe"], "aborted:failsafe"),
    (["goto_timeout"], "failed:goto_timeout"),
])
def test_outcome(events, result):
    assert outcome(events) == result


def test_flight_record_fields():
    lab = load_lab(DATA / "lab_example.yaml")
    rc = core({"type": "hover", "duration": 1.0})
    for e in ("watchdog_hover", "watchdog_hover", "segment_truncated", "battery_land"):
        rc.on_event(e)
    rec = flight_record(flight_id="s_01_t", session="s", started="2026-11-21T10:00:00", plan_path="/p.yaml",
                        plan=rc.executor.plan, backend="fake", drone_info={"serial": "FAKE0001", "sdk": "fake"},
                        lab=lab, soft_box=lab.soft_box, hard_box=lab.hard_box, notes="n", software="abc123",
                        runner=rc)
    assert rec["result"] == "running" and rec["drone"]["serial"] == "FAKE0001"
    assert rec["vicon"]["topic"] == "/vicon/tello/pose" and rec["volume"]["hard_box"][1] == 2.5
    assert rec["safety_event_count"] == 3 and rec["events"]["segment_truncated"] == 1
    assert rec["battery"] == {"start": None, "end": None} and rec["software"] == "abc123"


def test_config_validation():
    with pytest.raises(ValueError):
        RunnerConfig(stale_pose=0.0)
    with pytest.raises(ValueError):
        RunnerConfig(ready_timeout=math.nan)
    with pytest.raises(ValueError):
        RunnerConfig(mocap_min_rate=0.0)


def test_interrupt_finalises_a_running_run():
    rc = core({"type": "hover", "duration": 1.0})
    status(rc, 0.0, FS.LANDED)
    assert rc.tick(0.0).request == "takeoff" and rc.phase == "running"
    rc.interrupt(0.5)
    assert rc.phase == "done" and rc.result == "aborted:interrupted" and rc.battery_end == 80.0
    assert rc.tick(0.6).finished and rc.result == "aborted:interrupted"


def test_interrupt_after_finish_changes_nothing():
    rc = core({"type": "hover", "duration": 1.0})
    rc.abort("operator")
    assert rc.tick(0.0).finished
    rc.interrupt(1.0)
    assert rc.result == "aborted:operator"
