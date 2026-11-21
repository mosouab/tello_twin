import math

import pytest

from tello_core.fake import FakeDynamics
from tello_core.sticks import Sticks, to_rc


def run(f, t0, t1, dt=0.005):
    events, t = [], t0
    while t < t1 - 1e-9:
        t = round(t + dt, 9)
        events += [(t, e) for e in f.step(t)]
    return events, t


def test_takeoff_reaches_height_and_reports():
    f = FakeDynamics()
    f.takeoff(0.0)
    events, _ = run(f, 0.0, 5.0)
    assert [e for _, e in events] == ["takeoff_done"]
    assert 2.0 < events[0][0] < 3.0
    assert f.mode == "flying" and f.truth().p[2] >= 0.8


def test_command_applied_after_delay(flying_fake):
    f, t = flying_fake()
    _, t = run(f, t, t + 1.0)
    t0 = t
    f.set_rc(t0, to_rc(Sticks(x=0.5)))
    _, t = run(f, t0, t0 + 0.59)
    assert f.truth().v_world[0] == 0.0
    run(f, t, t0 + 0.6 + 3.0)
    assert f.truth().v_world[0] == pytest.approx(1.065 * 0.5, rel=0.01)


def test_ccw_yaw_stick_increases_yaw(flying_fake):
    f, t = flying_fake()
    f.set_rc(t, to_rc(Sticks(yaw=0.5)))
    run(f, t, t + 2.6)
    assert f.truth().yaw > 0.5


def test_forward_follows_heading(flying_fake):
    f, t = flying_fake(yaw=math.pi / 2)
    f.set_rc(t, to_rc(Sticks(x=0.5)))
    run(f, t, t + 3.0)
    p = f.truth().p
    assert p[1] > 0.5 and abs(p[0]) < 1e-6


def test_land_reports_and_rests_on_the_floor(flying_fake):
    f, t = flying_fake()
    f.land(t)
    events, _ = run(f, t, t + 6.0)
    assert [e for _, e in events] == ["land_done"]
    assert f.mode == "landed" and f.truth().p[2] == 0.0


def test_battery_drains_only_when_airborne():
    f = FakeDynamics()
    run(f, 0.0, 10.0)
    assert f.battery == 100.0
    f.takeoff(10.0)
    run(f, 10.0, 20.0)
    assert f.battery == pytest.approx(100.0 - 10.0 * 80.0 / 420.0, rel=1e-3)


def test_emergency_falls_and_stays_off(flying_fake):
    f, t = flying_fake()
    f.emergency(t)
    _, t = run(f, t, t + 2.0)
    assert f.mode == "off" and f.truth().p[2] == 0.0
    f.takeoff(t)
    run(f, t, t + 1.0)
    assert f.mode == "off"


def test_time_going_backwards_is_ignored(flying_fake):
    f, t = flying_fake()
    before = f.truth()
    assert f.step(t - 1.0) == []
    assert f.truth() == before


def test_telemetry_is_rounded_like_the_sdk(flying_fake):
    f, _ = flying_fake()
    tel = f.telemetry()
    assert tel.height * 100 == pytest.approx(round(tel.height * 100))
    assert tel.battery == round(tel.battery)
    assert tel.raw.startswith("pitch:")
