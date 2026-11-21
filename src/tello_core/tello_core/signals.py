"""Excitation signals for `excite` segments (spec section 7.2). Values are stick units; 0 outside [0, duration)."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable

import numpy as np


@dataclass(frozen=True)
class Signal:
    duration: float
    fn: Callable[[float], float]

    def value(self, t: float) -> float:
        return float(self.fn(t)) if 0.0 <= t < self.duration else 0.0


def _positive(name: str, v: float) -> None:
    if not v > 0:
        raise ValueError(f"{name} must be > 0, got {v}")


def step(amplitude: float, duration: float) -> Signal:
    _positive("duration", duration)
    return Signal(duration, lambda t: amplitude)


def doublet(amplitude: float, half_period: float) -> Signal:
    _positive("half_period", half_period)
    return Signal(2 * half_period, lambda t: amplitude if t < half_period else -amplitude)


def staircase(amplitudes: list[float], hold: float) -> Signal:
    """For each amplitude a: +a for `hold` s, then -a for `hold` s. Maps the dead zone, averages to zero."""
    _positive("hold", hold)
    if not amplitudes:
        raise ValueError("staircase needs at least one amplitude")
    seq = [s * a for a in amplitudes for s in (1.0, -1.0)]
    return Signal(len(seq) * hold, lambda t: seq[min(int(t // hold), len(seq) - 1)])


def chirp(amplitude: float, f_min: float, f_max: float, duration: float) -> Signal:
    """Linear frequency sweep from f_min to f_max."""
    _positive("duration", duration)
    if not 0 < f_min < f_max:
        raise ValueError(f"need 0 < f_min < f_max, got {f_min}, {f_max}")
    c = (f_max - f_min) / duration
    return Signal(duration, lambda t: amplitude * math.sin(2 * math.pi * (f_min * t + 0.5 * c * t * t)))


def harmonics(f_min: float, f_max: float, duration: float, axis_index: int = 0, n_axes: int = 1) -> list[float]:
    """Frequencies k/duration in [f_min, f_max] with k % n_axes == axis_index (disjoint sets per axis)."""
    f0 = 1.0 / duration
    kmax = int(math.floor(f_max * duration + 1e-9))
    return [k * f0 for k in range(1, kmax + 1) if k * f0 >= f_min - 1e-12 and k % n_axes == axis_index]


def multisine(amplitude: float, f_min: float, f_max: float, duration: float, axis_index: int = 0,
              n_axes: int = 1, seed: int | None = None) -> Signal:
    """Sum of sines on `harmonics(...)`, Schroeder phases (low crest factor), scaled so the peak is `amplitude`.

    Axes with different `axis_index` use disjoint harmonics of 1/duration, so they are uncorrelated over the
    duration. `seed` applies a random circular time shift, which keeps the crest factor."""
    _positive("duration", duration)
    freqs = np.array(harmonics(f_min, f_max, duration, axis_index, n_axes))
    if freqs.size == 0:
        raise ValueError(f"no harmonic of {1.0 / duration:.3g} Hz in [{f_min}, {f_max}] Hz for axis {axis_index}")
    n = freqs.size
    phases = np.array([-math.pi * i * (i - 1) / n for i in range(1, n + 1)])
    if seed is not None:
        tau = np.random.default_rng(seed).random() * duration
        phases = phases + 2 * math.pi * freqs * tau
    grid = np.linspace(0.0, duration, 4096, endpoint=False)
    peak = float(np.max(np.abs(np.sin(2 * math.pi * np.outer(grid, freqs) + phases).sum(axis=1))))
    scale = amplitude / peak

    def fn(t: float) -> float:
        return scale * float(np.sin(2 * math.pi * freqs * t + phases).sum())

    return Signal(duration, fn)
