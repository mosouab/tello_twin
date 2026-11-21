import math

import numpy as np
import pytest

from tello_core.mocap import (MocapSimParams, MocapSimulator, MocapTracker, quat_of_yaw, rotate_yaw,
                              yaw_of_quat)


def test_quaternion_helpers():
    for yaw in (0.0, 0.5, -2.0, 3.1):
        assert yaw_of_quat(quat_of_yaw(yaw)) == pytest.approx(yaw)
    assert yaw_of_quat(rotate_yaw(quat_of_yaw(0.2), 0.3)) == pytest.approx(0.5)


def feed_line(tr, n, v=0.5, dt=0.01, t0=0.0, y0=0.0):
    out = None
    for k in range(n):
        t = t0 + k * dt
        out = tr.update(t, (v * t, y0, 1.0), quat_of_yaw(0.0)) or out
    return out


def test_constant_velocity_is_estimated():
    out = feed_line(MocapTracker(), 100)
    assert out.v[0] == pytest.approx(0.5, rel=1e-3) and out.v[1] == pytest.approx(0.0, abs=1e-9)


def test_clock_offset_does_not_change_velocity():
    tr = MocapTracker()
    out = None
    for k in range(100):
        t = k * 0.01
        out = tr.update(t + 1000.35, (0.5 * t, 0.0, 1.0), quat_of_yaw(0.0)) or out
    assert out.v[0] == pytest.approx(0.5, rel=1e-3)


def test_jump_rejected_then_threshold_grows():
    tr = MocapTracker(thr_pos=0.3)
    for k in range(10):
        assert tr.update(k * 0.01, (0.0, 0.0, 1.0), quat_of_yaw(0.0)) is not None
    results = [tr.update(0.1 + k * 0.01, (0.55, 0.0, 1.0), quat_of_yaw(0.0)) for k in range(4)]
    # thresholds 0.3, 0.4, 0.5 reject the 0.55 m jump; 0.6 accepts it
    assert [r is None for r in results] == [True, True, True, False]
    assert tr.rejected == 3 and tr.accepted == 11


@pytest.mark.parametrize("p, q, t", [
    ((math.nan, 0.0, 1.0), quat_of_yaw(0.0), 0.2),
    ((0.0, 0.0, 1.0), (0.0, 0.0, math.inf, 1.0), 0.2),
    ((0.0, 0.0, -0.5), quat_of_yaw(0.0), 0.2),          # below z_floor
    ((0.0, 0.0, 1.0), quat_of_yaw(0.0), 0.05),          # not newer than the last sample
    ((0.0, 0.0, 1.0), quat_of_yaw(1.0), 0.2),           # yaw jump 1.0 rad > 0.5
])
def test_invalid_samples_are_rejected(p, q, t):
    tr = MocapTracker()
    tr.update(0.1, (0.0, 0.0, 1.0), quat_of_yaw(0.0))
    assert tr.update(t, p, q) is None and tr.rejected == 1


def test_heading_offset_is_applied():
    out = MocapTracker(heading_offset=0.2).update(0.0, (0.0, 0.0, 1.0), quat_of_yaw(0.1))
    assert out.yaw == pytest.approx(0.3) and yaw_of_quat(out.q) == pytest.approx(0.3)


def test_simulator_decimates_to_its_rate():
    sim = MocapSimulator(MocapSimParams(rate=100.0, drop_prob=0.0))
    out = [sim.process(k * 0.005, (0.0, 0.0, 1.0), 0.0) for k in range(400)]   # 200 Hz input, 2 s
    assert sum(s is not None for s in out) == 200


def test_simulator_stamp_offset_drift_and_none_mode():
    prm = MocapSimParams(drop_prob=0.0, noise_pos=0.0, noise_yaw=0.0, clock_offset=0.35, clock_drift_ppm=20.0)
    sim = MocapSimulator(prm)
    first = sim.process(10.0, (0.0, 0.0, 1.0), 0.0)
    later = sim.process(110.0, (0.0, 0.0, 1.0), 0.0)
    assert first.stamp == pytest.approx(10.35) and later.stamp == pytest.approx(110.35 + 20e-6 * 100.0)
    none = MocapSimulator(MocapSimParams(stamp_mode="none", drop_prob=0.0)).process(5.0, (0, 0, 1), 0.0)
    assert none.stamp == 0.0


def test_simulator_delay_and_noise():
    prm = MocapSimParams(drop_prob=0.0, delay=0.003, delay_jitter=0.001, noise_pos=0.0005, seed=4)
    sim = MocapSimulator(prm)
    samples = [sim.process(k * 0.01, (0.0, 0.0, 1.0), 0.0) for k in range(5000)]
    lags = np.array([s.release - k * 0.01 for k, s in enumerate(samples)])
    assert lags.min() >= 0.002 - 1e-9 and lags.max() <= 0.004 + 1e-9
    xs = np.array([s.p[0] for s in samples])
    assert xs.std() == pytest.approx(0.0005, rel=0.1)


def test_simulator_drops_in_bursts_and_is_reproducible():
    every = MocapSimulator(MocapSimParams(drop_prob=1.0, drop_burst=1))
    assert all(every.process(k * 0.01, (0, 0, 1), 0.0) is None for k in range(50)) and every.dropped == 50
    a, b = (MocapSimulator(MocapSimParams(drop_prob=0.2, seed=7)) for _ in range(2))
    ra = [a.process(k * 0.01, (0, 0, 1), 0.0) is None for k in range(500)]
    rb = [b.process(k * 0.01, (0, 0, 1), 0.0) is None for k in range(500)]
    assert ra == rb and 0 < sum(ra) < 500


def test_simulator_params_validation():
    with pytest.raises(ValueError):
        MocapSimParams(stamp_mode="host")
    with pytest.raises(ValueError):
        MocapSimParams(drop_burst=0)


def test_simulator_keeps_jittery_input_at_its_rate():
    sim = MocapSimulator(MocapSimParams(rate=100.0, drop_prob=0.0))
    rng = np.random.default_rng(1)
    ts = [k * 0.01 + float(rng.uniform(0.0, 0.004)) for k in range(500)]      # 100 Hz, up to 4 ms late
    assert sum(sim.process(t, (0.0, 0.0, 1.0), 0.0) is not None for t in ts) == 500
