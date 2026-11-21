import math
from pathlib import Path

import pytest

from tello_core.flight_state import FlightState as FS
from tello_core.lab import load_lab
from tello_core.plan import parse_plan
from tello_core.runner import RunnerConfig, RunnerCore, flight_record, outcome
from tello_core.safety import Box
from tello_core.sticks import ZERO

DATA = Path(__file__).parent / "data"
SOFT = Box(-2.0, 2.0, -2.0, 2.0, 0.5, 2.0)


def plan_of(*segments):
    return parse_plan({"name": "t", "limits": {"stick_max": [0.4] * 4},
                       "segments": [{"type": "takeoff"}, *segments, {"type": "land"}]})


def core(*segments, **cfg):
    return RunnerCore(plan_of(*segments), SOFT, cfg=RunnerConfig(**cfg))


def status(rc, t, state, hold=False, recording=True, battery=80.0, mocap_age=0.01, mocap_rate=100.0):
    rc.on_status(t, state, hold, recording, battery, mocap_age, mocap_rate)


def fly(rc):
    """Drive the takeoff segment to completion; the second segment starts at t = 2.0 (last status at 2.0)."""
    status(rc, 0.0, FS.LANDED)
    out = rc.tick(0.0)
    assert out.request == "takeoff"
    status(rc, 0.1, FS.TAKING_OFF)
    rc.tick(0.1)
    status(rc, 1.0, FS.FLYING)
    rc.on_pose(1.0, (0.0, 0.0, 1.0), 0.0, (0.0, 0.0, 0.0), 0.0)
    rc.tick(1.0)
    status(rc, 2.0, FS.FLYING)
    rc.on_pose(2.0, (0.0, 0.0, 1.0), 0.0, (0.0, 0.0, 0.0), 0.0)
    out = rc.tick(2.0)
    assert out.started is not None and rc.phase == "running"
    return out


def to_goto(rc):
    """Drive the takeoff segment to completion; the goto segment starts at t = 2.0."""
    out = fly(rc)
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
    assert out.request == "land" and not out.finished and rc.phase == "settling"   # airborne: settle first
    status(rc, 3.0, FS.LANDED, hold=True)
    assert rc.tick(3.0).finished and rc.result == "aborted:hold"


def test_hold_blocks_readiness_and_ends_aborted_hold():
    rc = core({"type": "hover", "duration": 1.0}, ready_timeout=2.0)
    status(rc, 0.0, FS.LANDED, hold=True)
    assert rc.tick(0.0).request is None and rc.phase == "waiting"
    status(rc, 2.4, FS.LANDED, hold=True)
    out = rc.tick(2.5)
    assert out.finished and out.request is None and rc.result == "aborted:hold"
    assert "plan_aborted:hold" in out.events and "runner_not_ready" not in out.events


def test_hold_released_before_the_timeout_lets_the_run_proceed():
    rc = core({"type": "hover", "duration": 1.0}, ready_timeout=2.0)
    status(rc, 0.0, FS.LANDED, hold=True)
    assert rc.tick(0.0).request is None
    status(rc, 1.0, FS.LANDED, hold=False)
    out = rc.tick(1.0)
    assert out.request == "takeoff" and rc.phase == "running"


def test_hold_seen_while_running_aborts_without_a_rising_edge():
    rc = core({"type": "hover", "duration": 10.0})
    fly(rc)
    rc.hold = True                                   # hold already seen: no rising edge is left
    status(rc, 2.1, FS.FLYING, hold=True)
    status(rc, 2.15, FS.FLYING, hold=True)           # a repeated hold is harmless
    out = rc.tick(2.2)
    assert out.request == "land" and out.events == ["plan_aborted:hold"] and rc.phase == "settling"
    status(rc, 4.0, FS.LANDED, hold=True)
    assert rc.tick(4.0).finished and rc.result == "aborted:hold"
    assert rc.event_counts["plan_aborted:hold"] == 1


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


def test_flight_record_start_stamp():
    lab = load_lab(DATA / "lab_example.yaml")
    rc = core({"type": "hover", "duration": 1.0})
    kw = dict(flight_id="s_01_t", session="s", started="2026-11-21T10:00:00", plan_path="/p.yaml",
              plan=rc.executor.plan, backend="fake", drone_info={}, lab=lab, soft_box=lab.soft_box,
              hard_box=lab.hard_box, notes="", software="abc123", runner=rc)
    assert flight_record(**kw)["start_stamp"] is None
    assert flight_record(**kw, start_stamp=1795000000.123456)["start_stamp"] == 1795000000.123456


def test_config_validation():
    with pytest.raises(ValueError):
        RunnerConfig(stale_pose=0.0)
    with pytest.raises(ValueError):
        RunnerConfig(ready_timeout=math.nan)
    with pytest.raises(ValueError):
        RunnerConfig(mocap_min_rate=0.0)
    with pytest.raises(ValueError):
        RunnerConfig(settle_timeout=0.0)
    with pytest.raises(ValueError):
        RunnerConfig(settle_timeout=math.inf)
    with pytest.raises(ValueError):
        RunnerConfig(status_timeout=math.nan)
    with pytest.raises(ValueError):
        RunnerConfig(status_timeout=-0.5)


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


# ---------------------------------------------------------------- settling (spec 15.18)
def test_abort_while_airborne_settles_until_landed():
    rc = core({"type": "hover", "duration": 10.0})
    fly(rc)
    status(rc, 2.1, FS.FLYING, battery=79.0)
    rc.abort("operator")
    out = rc.tick(2.1)
    assert out.request == "land" and not out.finished and rc.phase == "settling" and rc.result is None
    status(rc, 2.2, FS.LANDING, battery=78.0)
    rc.on_event("watchdog_hover")                    # events are still accepted while settling
    out = rc.tick(2.2)
    assert not out.finished and not out.publish_cmd and out.request is None and out.sticks == ZERO
    assert out.started is None and out.events == []
    status(rc, 5.0, FS.LANDED, battery=77.0)
    out = rc.tick(5.0)
    assert out.finished and rc.phase == "done" and rc.result == "aborted:operator"
    assert rc.battery_end == 77.0 and rc.event_counts["watchdog_hover"] == 1   # taken at finalisation


@pytest.mark.parametrize("state", [FS.LANDED, FS.EMERGENCY, FS.DISCONNECTED])
def test_settling_ends_when_the_drone_is_down(state):
    rc = core({"type": "hover", "duration": 10.0})
    fly(rc)
    rc.abort("operator")
    assert not rc.tick(2.1).finished
    status(rc, 3.0, state)
    assert rc.tick(3.0).finished and rc.result == "aborted:operator" and "settle_timeout" not in rc.event_counts


def test_settle_timeout_finalises_with_the_plan_outcome():
    rc = core({"type": "hover", "duration": 10.0})
    fly(rc)
    rc.abort("operator")
    assert not rc.tick(2.1).finished                 # settling from t = 2.1
    status(rc, 22.0, FS.FLYING)
    out = rc.tick(22.0)
    assert not out.finished and "settle_timeout" not in out.events
    out = rc.tick(22.2)
    assert out.finished and out.events == ["settle_timeout"] and rc.result == "aborted:operator"
    assert rc.event_counts["settle_timeout"] == 1


def test_settle_timeout_is_configurable():
    rc = core({"type": "hover", "duration": 10.0}, settle_timeout=1.0)
    fly(rc)
    rc.abort("operator")
    rc.tick(2.1)
    assert rc.tick(3.2).finished and rc.event_counts["settle_timeout"] == 1


def test_failsafe_landing_settles_until_landed():
    rc = core({"type": "hover", "duration": 10.0})
    fly(rc)
    status(rc, 2.1, FS.LANDING)                      # tello_node's safety rule is landing the drone
    out = rc.tick(2.1)
    assert "plan_aborted_by_failsafe" in out.events and out.request is None and not out.finished
    status(rc, 4.0, FS.LANDED)
    assert rc.tick(4.0).finished and rc.result == "aborted:failsafe"


def test_failsafe_into_emergency_finalises_at_once():
    rc = core({"type": "hover", "duration": 10.0})
    fly(rc)
    status(rc, 2.1, FS.EMERGENCY)
    out = rc.tick(2.1)
    assert out.finished and rc.result == "aborted:failsafe" and "plan_aborted_by_failsafe" in out.events


def test_completed_plan_finalises_at_once_when_landed():
    rc = core({"type": "hover", "duration": 0.5})
    fly(rc)                                          # hover from t = 2.0
    status(rc, 2.6, FS.FLYING)
    assert rc.tick(2.6).started.segment.type == "land"
    assert rc.tick(2.62).request == "land"
    status(rc, 2.7, FS.LANDING)
    assert not rc.tick(2.7).finished
    status(rc, 5.0, FS.LANDED)
    out = rc.tick(5.0)
    assert out.finished and rc.result == "completed" and rc.phase == "done"


def test_interrupt_while_settling_keeps_the_plan_outcome():
    rc = core({"type": "hover", "duration": 10.0})
    fly(rc)
    rc.abort("operator")
    rc.tick(2.1)
    assert rc.phase == "settling"
    rc.interrupt(3.0)
    assert rc.phase == "done" and rc.result == "aborted:operator"


def test_own_events_are_counted_once():
    lab = load_lab(DATA / "lab_example.yaml")
    rc = core({"type": "hover", "duration": 1.0})
    status(rc, 0.0, FS.LANDED)
    rc.tick(0.0)
    rc.on_takeoff_result(False, "hold active")
    out = rc.tick(0.1)
    assert out.finished and "plan_aborted:takeoff_refused" in out.events
    assert rc.event_counts["plan_aborted:takeoff_refused"] == 1     # counted when emitted, before any echo
    rc.on_event("plan_aborted:takeoff_refused", own=True)          # its echo through /tello/events
    rc.on_event("takeoff_refused")                                  # tello_node's event
    rec = flight_record(flight_id="s_01_t", session="s", started="2026-11-21T10:00:00", plan_path="/p.yaml",
                        plan=rc.executor.plan, backend="fake", drone_info={}, lab=lab, soft_box=lab.soft_box,
                        hard_box=lab.hard_box, notes="", software="abc123", runner=rc)
    assert rec["events"] == {"plan_aborted:takeoff_refused": 1, "takeoff_refused": 1}


# ---------------------------------------------------------------- status loss (spec 15.19)
def test_status_loss_aborts_then_settles_until_the_timeout():
    rc = core({"type": "hover", "duration": 10.0})
    fly(rc)                                          # last status at t = 2.0
    assert rc.tick(2.4).events == []                 # 0.4 s old: still fresh
    out = rc.tick(2.6)                               # 0.6 s old: tello_node is gone
    assert out.events == ["plan_aborted:status_lost"] and out.request == "land" and rc.phase == "settling"
    assert not rc.tick(22.5).finished                # no status arrives: settle until the timeout
    out = rc.tick(22.7)
    assert out.finished and "settle_timeout" in out.events and rc.result == "aborted:status_lost"


def test_status_timeout_is_configurable():
    rc = core({"type": "hover", "duration": 10.0}, status_timeout=2.0)
    fly(rc)
    assert rc.tick(3.9).events == [] and rc.phase == "running"
    assert rc.tick(4.1).events == ["plan_aborted:status_lost"]
