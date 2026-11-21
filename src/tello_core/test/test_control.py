import math

import pytest

from tello_core.control import (Arena, ArenaParams, ControlConfig, GotoParams, goto_command,
                                goto_reached)
from tello_core.safety import Box
from tello_core.sticks import to_rc

CFG = ControlConfig()
STILL = dict(p=(0.0, 0.0, 1.0), yaw=0.0, v=(0.0, 0.0, 0.0), yaw_rate=0.0)


def cmd(gp, cfg=CFG, **kw):
    s = dict(STILL)
    s.update(kw)
    return goto_command(gp, cfg, s["p"], s["yaw"], s["v"], s["yaw_rate"])


def test_at_target_gives_zero():
    assert cmd(GotoParams(target=(0, 0, 1), yaw=0.0)).as_tuple() == pytest.approx((0, 0, 0, 0), abs=1e-12)


def test_forward_target_gives_forward_stick():
    u = cmd(GotoParams(target=(0.2, 0, 1)))
    assert u.x == pytest.approx(0.8 * 0.2 / 1.065)
    assert u.y == pytest.approx(0, abs=1e-12) and u.z == pytest.approx(0, abs=1e-12)


def test_heading_frame_conversion():
    u = cmd(GotoParams(target=(0, 0.2, 1)), yaw=math.pi / 2)
    assert u.x == pytest.approx(0.8 * 0.2 / 1.065) and u.y == pytest.approx(0, abs=1e-9)


def test_prediction_brakes_before_the_target():
    u = cmd(GotoParams(target=(0.3, 0, 1)), v=(0.5, 0.0, 0.0))   # predicted x = 0 + 0.5 * 0.7 = 0.35
    assert u.x < 0


def test_speed_is_limited():
    assert cmd(GotoParams(target=(10, 0, 1), v_max=0.4)).x == pytest.approx(0.4 / 1.065)


def test_stick_max_clamps():
    cfg = ControlConfig(stick_max=(0.1, 0.1, 0.1, 0.1))
    assert cmd(GotoParams(target=(10, 0, 1)), cfg=cfg).x == pytest.approx(0.1)


def test_yaw_error_wraps_around_pi():
    u = cmd(GotoParams(target=(0, 0, 1), yaw=3.1), yaw=-3.1)
    expected = (3.1 - (-3.1) - 2 * math.pi) / 0.955
    assert u.yaw == pytest.approx(expected) and -0.1 < u.yaw < 0


def test_goto_reached():
    gp = GotoParams(target=(0, 0, 1), yaw=0.0, tol=0.1)
    assert goto_reached(gp, (0.05, 0, 1), 0.02, (0.05, 0, 0))
    assert not goto_reached(gp, (0.2, 0, 1), 0.0, (0, 0, 0))
    assert not goto_reached(gp, (0, 0, 1), 0.0, (0.3, 0, 0))
    assert not goto_reached(gp, (0, 0, 1), 0.5, (0, 0, 0))


def test_arena_keeps_the_fake_inside_the_box(flying_fake):
    box = Box(-2, 2, -2, 2, 0.5, 2.0)
    f, t = flying_fake()
    arena = Arena(ArenaParams(duration=120.0, seed=1), CFG, box)
    while t < 120.0:
        tr = f.truth()
        f.set_rc(t, to_rc(arena.step(t, tr.p, tr.yaw, tr.v_world, tr.yaw_rate)))
        t = round(t + 0.02, 9)
        f.step(t)
        assert box.contains(f.truth().p), (t, f.truth().p)


def test_arena_is_reproducible():
    box = Box(-2, 2, -2, 2, 0.5, 2.0)
    a, b = (Arena(ArenaParams(duration=10.0, seed=7), CFG, box) for _ in range(2))
    for i in range(200):
        t = i * 0.05
        assert a.step(t, (0.1, 0, 1), 0.0, (0, 0, 0), 0.0) == b.step(t, (0.1, 0, 1), 0.0, (0, 0, 0), 0.0)
