import math

import pytest

from tello_core.flight_state import (FlightState as FS, FlightStateMachine, PreflightConfig,
                                     PreflightInputs, preflight_failures)
from tello_core.safety import Box

SOFT = Box(-2, 2, -2, 2, 0.5, 2.0)
PCFG = PreflightConfig(soft_box=SOFT)
GOOD = dict(connected=True, battery=80.0, mocap_age=0.05, position=(0.0, 0.0, 0.02),
            mocap_rate=100.0, mocap_expected_rate=100.0, recording=True)


def pin(**kw):
    d = dict(GOOD)
    d.update(kw)
    return PreflightInputs(**d)


def test_preflight_ok_with_drone_on_the_floor():
    assert preflight_failures(PCFG, pin()) == []


@pytest.mark.parametrize("kw, fragment", [
    ({"connected": False}, "not connected"),
    ({"battery": 29.0}, "battery"),
    ({"battery": math.nan}, "battery"),
    ({"mocap_age": 0.5}, "no recent Vicon pose"),
    ({"position": (2.5, 0.0, 0.02)}, "soft volume"),
    ({"position": None}, "soft volume"),
    ({"mocap_rate": 80.0}, "Vicon rate"),
    ({"recording": False}, "recorder"),
])
def test_preflight_failures(kw, fragment):
    f = preflight_failures(PCFG, pin(**kw))
    assert len(f) == 1 and fragment in f[0]


def test_recording_can_be_optional():
    cfg = PreflightConfig(soft_box=SOFT, require_recording=False)
    assert preflight_failures(cfg, pin(recording=False)) == []


def test_nominal_cycle():
    sm = FlightStateMachine()
    assert sm.state == FS.DISCONNECTED and not sm.accepts_commands
    assert sm.on_telemetry() and sm.state == FS.LANDED
    assert sm.request_takeoff([]) == (True, "taking off") and sm.state == FS.TAKING_OFF
    assert sm.on_takeoff_done() and sm.state == FS.FLYING and sm.accepts_commands
    assert sm.request_land()[0] and sm.state == FS.LANDING
    assert sm.on_land_done() and sm.state == FS.LANDED


def test_takeoff_refused_with_reasons():
    sm = FlightStateMachine()
    sm.on_telemetry()
    ok, msg = sm.request_takeoff(["battery 20 % below 30 %", "recorder not running"])
    assert not ok and "battery" in msg and "recorder" in msg and sm.state == FS.LANDED


def test_takeoff_only_from_landed():
    ok, msg = FlightStateMachine().request_takeoff([])
    assert not ok and "DISCONNECTED" in msg


def test_land_while_taking_off():
    sm = FlightStateMachine()
    sm.on_telemetry()
    sm.request_takeoff([])
    assert sm.request_land()[0] and sm.state == FS.LANDING


def test_land_when_landed_is_refused():
    sm = FlightStateMachine()
    sm.on_telemetry()
    assert sm.request_land()[0] is False


def test_emergency_is_sticky_until_reset():
    sm = FlightStateMachine()
    sm.on_telemetry()
    sm.request_takeoff([])
    sm.on_takeoff_done()
    sm.on_emergency()
    assert sm.state == FS.EMERGENCY
    assert sm.request_takeoff([])[0] is False
    assert sm.on_telemetry() is False and sm.on_link_lost() is False and sm.state == FS.EMERGENCY
    assert sm.reset() and sm.state == FS.DISCONNECTED
    assert sm.on_telemetry() and sm.state == FS.LANDED


def test_link_lost_in_the_air_asks_for_landing():
    sm = FlightStateMachine()
    sm.on_telemetry()
    sm.request_takeoff([])
    sm.on_takeoff_done()
    assert sm.on_link_lost() is True and sm.state == FS.DISCONNECTED
    sm.on_telemetry()
    assert sm.on_link_lost() is False


def test_values_match_tello_status_constants():
    assert [(s.name, s.value) for s in FS] == [("DISCONNECTED", 0), ("LANDED", 1), ("TAKING_OFF", 2),
                                               ("FLYING", 3), ("LANDING", 4), ("EMERGENCY", 5)]


@pytest.mark.parametrize("kw", [
    {"battery_takeoff_min": math.nan}, {"battery_takeoff_min": 150.0}, {"mocap_max_age": 0.0},
    {"mocap_max_age": math.inf}, {"mocap_min_rate_ratio": 0.0}, {"mocap_min_rate_ratio": 1.5},
])
def test_preflight_config_validates(kw):
    with pytest.raises(ValueError):
        PreflightConfig(soft_box=SOFT, **kw)


def test_preflight_config_rejects_infinite_box_bound():
    with pytest.raises(ValueError):
        PreflightConfig(soft_box=Box(-2, math.inf, -2, 2, 0.5, 2.0))


@pytest.mark.parametrize("rate", [0.0, math.nan, -5.0])
def test_preflight_invalid_expected_rate(rate):
    f = preflight_failures(PCFG, pin(mocap_expected_rate=rate))
    assert f == ["Vicon expected rate invalid"]
