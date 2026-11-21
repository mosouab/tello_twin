import math

import pytest

from tello_core.safety import Action, Box, EventLatch, SafetyConfig, SafetyInputs, evaluate

HARD = Box(-2, 2, -2, 2, 0.2, 2.5)
CFG = SafetyConfig(hard_box=HARD)
OK = dict(cmd_age=0.05, mocap_age=0.01, telemetry_age=0.1, position=(0.0, 0.0, 1.0),
          velocity=(0.0, 0.0, 0.0), battery=80.0)


def inputs(**kw):
    d = dict(OK)
    d.update(kw)
    return SafetyInputs(**d)


@pytest.mark.parametrize("kw, action, events", [
    ({}, Action.NONE, []),
    ({"cmd_age": 0.31}, Action.HOVER, ["watchdog_hover"]),
    ({"cmd_age": 5.1}, Action.LAND, ["watchdog_land"]),
    ({"cmd_age": math.inf}, Action.LAND, ["watchdog_land"]),
    ({"mocap_age": 0.51}, Action.HOVER, ["mocap_lost_hover"]),
    ({"mocap_age": 1.6}, Action.LAND, ["mocap_lost_land"]),
    ({"position": (2.5, 0.0, 1.0)}, Action.LAND, ["volume_hard_land"]),
    # 1.5 + 0.5 * (0.7 + 0.5) = 2.1 > 2: predicted exit
    ({"position": (1.5, 0.0, 1.0), "velocity": (0.5, 0.0, 0.0)}, Action.HOVER, ["volume_predicted"]),
    # 1.5 + 0.3 * 1.2 = 1.86: still inside
    ({"position": (1.5, 0.0, 1.0), "velocity": (0.3, 0.0, 0.0)}, Action.NONE, []),
    ({"battery": 15.0}, Action.LAND, ["battery_land"]),
    ({"battery": math.nan}, Action.NONE, []),
    ({"telemetry_age": 1.01}, Action.LAND, ["link_lost"]),
    ({"cmd_age": 0.4, "battery": 10.0}, Action.LAND, ["watchdog_hover", "battery_land"]),
])
def test_rules(kw, action, events):
    d = evaluate(CFG, inputs(**kw))
    assert d.action == action
    assert d.events == events


@pytest.mark.parametrize("pos", [None, (math.nan, 0.0, 1.0), (0.0, 0.0, math.inf)])
def test_invalid_position_means_hover(pos):
    d = evaluate(CFG, inputs(position=pos))
    assert d.action == Action.HOVER and d.events == ["mocap_lost_hover"]


def test_nan_velocity_skips_only_the_prediction():
    d = evaluate(CFG, inputs(position=(1.9, 0.0, 1.0), velocity=(math.nan, 0.0, 0.0)))
    assert d.action == Action.NONE


def test_nan_ages_count_as_never():
    assert evaluate(CFG, inputs(cmd_age=math.nan)).events == ["watchdog_land"]


def test_lookahead():
    assert CFG.lookahead == pytest.approx(1.2)


def test_box_validation_and_helpers():
    with pytest.raises(ValueError):
        Box.from_list([0, 1, 0, 1, 0])
    with pytest.raises(ValueError):
        Box.from_list([1, 0, 0, 1, 0, 1])
    b = Box.from_list([-2, 2, -1, 1, 0.5, 2.5])
    assert b.shrink(0.5) == Box(-1.5, 1.5, -0.5, 0.5, 1.0, 2.0)
    assert b.center() == (0.0, 0.0, 1.5)
    assert b.contains_xy((1.9, 0.9, 0.0)) and not b.contains((1.9, 0.9, 0.0))


def test_event_latch_reports_rising_edges():
    latch = EventLatch()
    assert latch.update(["a"]) == ["a"]
    assert latch.update(["a", "b"]) == ["b"]
    assert latch.update(["b"]) == []
    assert latch.update(["a", "b"]) == ["a"]


@pytest.mark.parametrize("kw", [
    {"cmd_timeout_hover": math.nan}, {"cmd_timeout_hover": 6.0}, {"cmd_timeout_land": 0.0},
    {"mocap_timeout_hover": 2.0}, {"mocap_timeout_land": math.inf}, {"telemetry_timeout": -1.0},
    {"battery_land": 150.0}, {"battery_land": -1.0}, {"cmd_delay": -0.1}, {"stop_time": math.nan},
])
def test_safety_config_validates(kw):
    with pytest.raises(ValueError):
        SafetyConfig(hard_box=HARD, **kw)


def test_safety_config_rejects_infinite_box_bound():
    with pytest.raises(ValueError):
        SafetyConfig(hard_box=Box(-math.inf, 2, -2, 2, 0.2, 2.5))
