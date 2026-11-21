import numpy as np
import pytest

from tello_core.signals import chirp, doublet, harmonics, multisine, staircase, step


def sample(sig, n=20000):
    ts = np.arange(n) * sig.duration / n
    return np.array([sig.value(t) for t in ts])


def test_step_and_doublet():
    s = step(0.3, 2.0)
    assert s.duration == 2.0 and s.value(0.0) == 0.3 and s.value(1.99) == 0.3
    assert s.value(2.0) == 0.0 and s.value(-0.1) == 0.0
    d = doublet(0.25, 1.0)
    assert d.duration == 2.0 and d.value(0.5) == 0.25 and d.value(1.5) == -0.25


def test_staircase_alternates_sign():
    s = staircase([0.05, 0.1], 0.5)
    assert s.duration == 2.0
    assert [s.value(t) for t in (0.1, 0.6, 1.1, 1.6)] == [0.05, -0.05, 0.1, -0.1]


def test_chirp_starts_at_zero_and_is_bounded():
    c = chirp(0.2, 0.1, 2.0, 20.0)
    assert c.value(0.0) == 0.0
    assert np.abs(sample(c)).max() <= 0.2 + 1e-12


def test_multisine_zero_mean_and_peak():
    x = sample(multisine(0.2, 0.1, 2.0, 20.0))
    assert abs(x.mean()) < 0.002
    assert np.abs(x).max() <= 0.2 * 1.02


def test_multisine_axes_are_uncorrelated():
    a = sample(multisine(0.2, 0.1, 2.0, 20.0, axis_index=0, n_axes=2))
    b = sample(multisine(0.2, 0.1, 2.0, 20.0, axis_index=1, n_axes=2))
    assert abs(np.corrcoef(a, b)[0, 1]) < 0.02


def test_harmonics_in_band_and_disjoint():
    h0 = harmonics(0.1, 2.0, 20.0, 0, 4)
    h1 = harmonics(0.1, 2.0, 20.0, 1, 4)
    assert h0 and h1 and not set(h0) & set(h1)
    assert all(0.1 <= f <= 2.0 for f in h0 + h1)


def test_seed_is_reproducible_and_changes_the_signal():
    a, b, c = (multisine(0.2, 0.1, 2.0, 20.0, seed=s) for s in (3, 3, 4))
    assert a.value(1.234) == b.value(1.234)
    assert a.value(1.234) != c.value(1.234)


def test_bad_arguments():
    with pytest.raises(ValueError):
        multisine(0.2, 1.1, 1.2, 2.0)        # harmonics of 0.5 Hz: none in [1.1, 1.2]
    with pytest.raises(ValueError):
        step(0.3, 0.0)
    with pytest.raises(ValueError):
        doublet(0.3, -1.0)
    with pytest.raises(ValueError):
        staircase([], 1.0)
    with pytest.raises(ValueError):
        chirp(0.2, 2.0, 1.0, 10.0)
