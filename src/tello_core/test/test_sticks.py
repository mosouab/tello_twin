import math

import pytest

from tello_core.sticks import ZERO, Sticks, clamp_sticks, format_rc, from_rc, to_rc


@pytest.mark.parametrize("sticks, rc", [
    (Sticks(x=0.5), (0, 50, 0, 0)),      # forward -> b
    (Sticks(y=0.5), (-50, 0, 0, 0)),     # left -> a negative (SDK a = right)
    (Sticks(z=0.5), (0, 0, 50, 0)),      # up -> c
    (Sticks(yaw=0.5), (0, 0, 0, -50)),   # counter-clockwise -> d negative (SDK d = clockwise)
    (ZERO, (0, 0, 0, 0)),
])
def test_sign_table(sticks, rc):
    assert to_rc(sticks) == rc


def test_rounding_half_away_from_zero():
    assert to_rc(Sticks(x=0.125)) == (0, 13, 0, 0)
    assert to_rc(Sticks(x=-0.125)) == (0, -13, 0, 0)
    assert to_rc(Sticks(x=0.124)) == (0, 12, 0, 0)


def test_rc_saturates_at_100():
    assert to_rc(Sticks(x=1.7, y=-3.0)) == (100, 100, 0, 0)


def test_non_finite_become_zero():
    s = clamp_sticks(Sticks(x=math.nan, y=math.inf, z=-math.inf, yaw=0.2), (0.4, 0.4, 0.4, 0.4))
    assert s == Sticks(0.0, 0.0, 0.0, 0.2)
    assert to_rc(Sticks(x=math.nan)) == (0, 0, 0, 0)


def test_clamp_per_axis():
    assert clamp_sticks(Sticks(0.9, -0.9, 0.1, 0.6), (0.3, 0.3, 0.3, 0.5)) == Sticks(0.3, -0.3, 0.1, 0.5)


def test_clamp_rejects_bad_limits():
    with pytest.raises(ValueError):
        clamp_sticks(ZERO, (0.4, 0.4, 0.4))
    with pytest.raises(ValueError):
        clamp_sticks(ZERO, (0.4, 0.4, 0.4, 1.5))


def test_roundtrip_through_rc():
    s = Sticks(0.3, -0.21, 0.05, 0.4)
    assert from_rc(to_rc(s)).as_tuple() == pytest.approx(s.as_tuple())


def test_format_rc():
    assert format_rc((-50, 30, 0, 12)) == "rc -50 30 0 12"
