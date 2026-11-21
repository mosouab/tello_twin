import pytest

from tello_core.executor import Observation, PlanExecutor, sync_profile
from tello_core.flight_state import FlightState as FS
from tello_core.plan import parse_plan
from tello_core.safety import Box
from tello_core.sticks import ZERO

SOFT = Box(-2, 2, -2, 2, 0.5, 2.0)


def plan_of(*segments, stick_max=(0.4, 0.4, 0.4, 0.4)):
    return parse_plan({"name": "t", "limits": {"stick_max": list(stick_max)},
                       "segments": [{"type": "takeoff"}, *segments, {"type": "land"}]})


def ob(t, state=FS.FLYING, p=(0.0, 0.0, 1.0), v=(0.0, 0.0, 0.0), yaw=0.0, r=0.0):
    return Observation(t, state, p, yaw, v, r)


def fly_through_takeoff(ex):
    """Takeoff segment: requested at t=0, flying and still from t=1.0, settled at t=2.0 (segment 1 starts)."""
    out0 = ex.step(ob(0.0, FS.LANDED, p=(0.0, 0.0, 0.0)))
    ex.step(ob(0.1, FS.TAKING_OFF, p=(0.0, 0.0, 0.3)))
    ex.step(ob(1.0))
    return out0, ex.step(ob(2.0))


def test_takeoff_requested_once_and_announced():
    ex = PlanExecutor(plan_of({"type": "hover", "duration": 1.0}), SOFT)
    out0 = ex.step(ob(0.0, FS.LANDED, p=(0.0, 0.0, 0.0)))
    assert out0.request == "takeoff" and out0.started.seq == 0 and out0.started.segment.type == "takeoff"
    assert ex.step(ob(0.1, FS.TAKING_OFF, p=(0.0, 0.0, 0.3))).request is None


def test_takeoff_waits_for_a_settled_hover():
    ex = PlanExecutor(plan_of({"type": "hover", "duration": 1.0}), SOFT)
    _, out = fly_through_takeoff(ex)
    assert out.started.seq == 1 and out.started.segment.type == "hover" and out.started.name == "1:hover"


def test_hover_then_land_then_finished():
    ex = PlanExecutor(plan_of({"type": "hover", "duration": 1.0}), SOFT)
    fly_through_takeoff(ex)
    out = ex.step(ob(2.5))
    assert out.sticks == ZERO and out.started is None
    assert ex.step(ob(3.0)).started.segment.type == "land"
    assert ex.step(ob(3.1)).request == "land"
    assert not ex.step(ob(5.0, FS.LANDING, p=(0.0, 0.0, 0.4))).finished
    assert ex.step(ob(7.0, FS.LANDED, p=(0.0, 0.0, 0.0))).finished


def test_goto_moves_towards_the_target():
    ex = PlanExecutor(plan_of({"type": "goto", "target": [0, 0, 1.2]}), SOFT)
    fly_through_takeoff(ex)
    out = ex.step(ob(2.02, p=(0.0, 0.0, 0.8)))
    assert out.sticks.z > 0 and out.sticks.x == pytest.approx(0.0, abs=1e-9)


def test_excite_near_edge_is_truncated_and_recentred():
    ex = PlanExecutor(plan_of({"type": "excite", "axes": ["x"], "signal": "step", "amplitude": 0.3,
                               "duration": 5.0}), SOFT)
    fly_through_takeoff(ex)
    out = ex.step(ob(2.02, p=(1.8, 0.0, 1.0), v=(0.3, 0.0, 0.0)))   # predicted x = 1.8 + 0.3 * 1.2 = 2.16
    assert "segment_truncated" in out.events
    assert out.started.name == "recenter" and out.started.segment.type == "goto"
    assert out.sticks == ZERO
    # recenter target = soft box centre (no goto before): reached at once, the plan continues with land
    assert ex.step(ob(2.04, p=(0.0, 0.0, 1.25))).started.segment.type == "land"


def test_excite_without_pose_sends_zero_sticks():
    ex = PlanExecutor(plan_of({"type": "excite", "axes": ["x"], "signal": "step", "amplitude": 0.3,
                               "duration": 5.0}), SOFT)
    fly_through_takeoff(ex)
    out = ex.step(Observation(2.02, FS.FLYING, None, None, None, None))
    assert out.sticks == ZERO and not out.finished and out.started is None
    assert ex.step(ob(2.04)).sticks.x == pytest.approx(0.3)


def test_failsafe_mid_segment_stops_the_plan():
    ex = PlanExecutor(plan_of({"type": "hover", "duration": 10.0}), SOFT)
    fly_through_takeoff(ex)
    out = ex.step(ob(3.0, FS.LANDING))
    assert "plan_aborted_by_failsafe" in out.events and out.finished and out.request is None
    later = ex.step(ob(3.1, FS.LANDED))
    assert later.finished and later.request is None


def test_abort_requests_landing():
    ex = PlanExecutor(plan_of({"type": "hover", "duration": 10.0}), SOFT)
    fly_through_takeoff(ex)
    ex.abort("operator")
    out = ex.step(ob(3.0))
    assert out.request == "land" and out.finished and "plan_aborted:operator" in out.events


def test_refused_takeoff_times_out_without_landing_request():
    ex = PlanExecutor(plan_of({"type": "hover", "duration": 1.0}), SOFT)
    ex.step(ob(0.0, FS.LANDED, p=(0.0, 0.0, 0.0)))
    out = ex.step(ob(20.5, FS.LANDED, p=(0.0, 0.0, 0.0)))
    assert "takeoff_timeout" in out.events and out.finished and out.request is None


def test_sticks_clamped_to_the_plan_limits():
    ex = PlanExecutor(plan_of({"type": "goto", "target": [0, 0, 1.2]}, stick_max=(0.2, 0.2, 0.2, 0.2)), SOFT)
    fly_through_takeoff(ex)
    assert ex.step(ob(2.02, p=(0.0, 0.0, 0.5))).sticks.z == pytest.approx(0.2)


def test_sync_profile_shape():
    total, u = sync_profile({"hover_pre": 2.0, "yaw_amp": 0.4, "yaw_time": 0.5, "hover_mid": 1.0,
                             "z_amp": 0.3, "z_time": 0.4, "hover_post": 2.0})
    assert total == pytest.approx(6.8)
    assert u(1.0) == ZERO and u(2.2).yaw == 0.4 and u(2.7).yaw == -0.4 and u(3.5) == ZERO
    assert u(4.2).z == 0.3 and u(4.6).z == -0.3 and u(6.0) == ZERO
