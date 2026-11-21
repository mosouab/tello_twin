"""Vicon helpers (spec sections 6.1 and 6.2): quaternion/yaw, the online tracker used by mocap_adapter, and the
simulated Vicon used by mocap_sim. Pure: the ROS nodes only convert messages.

Quaternions are (x, y, z, w). Times passed to MocapTracker are SOURCE stamps (the Vicon computer's clock):
only differences are used, so a constant clock offset cancels out.
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass

import numpy as np


def yaw_of_quat(q) -> float:
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_of_yaw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quat_mul(a, b) -> tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def rotate_yaw(q, angle: float) -> tuple[float, float, float, float]:
    """Rotate an orientation by `angle` about the world z axis."""
    return quat_mul(quat_of_yaw(angle), q)


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass(frozen=True)
class TrackOutput:
    p: tuple[float, float, float]
    q: tuple[float, float, float, float]      # heading-corrected
    yaw: float                                # heading-corrected, wrapped to (-pi, pi]
    v: tuple[float, float, float]             # world frame, m/s
    yaw_rate: float                           # rad/s


class MocapTracker:
    """Online jump filter plus velocity estimate (spec 6.1).

    A sample is rejected if it is non-finite, not newer than the last accepted one, below `z_floor`, or if it
    jumps more than `thr_pos` (from the last accepted sample extrapolated with its velocity) or `thr_yaw`.
    Thresholds grow by (3 + n_hold) / 3 while samples keep being rejected (after Švrčina), so a long glitch
    cannot lock the tracker out. Velocity: finite difference on source stamps, first-order low-pass (tau)."""

    def __init__(self, thr_pos: float = 0.3, thr_yaw: float = 0.5, z_floor: float = -0.2,
                 heading_offset: float = 0.0, tau: float = 0.05):
        self.thr_pos, self.thr_yaw, self.z_floor = thr_pos, thr_yaw, z_floor
        self.heading_offset, self.tau = heading_offset, tau
        self.received = self.accepted = self.rejected = 0
        self._n_hold = 0
        self._t = None
        self._p = None
        self._yaw = None
        self._v = (0.0, 0.0, 0.0)
        self._r = 0.0

    def _reject(self) -> None:
        self.rejected += 1
        self._n_hold += 1

    def update(self, t: float, p, q) -> TrackOutput | None:
        self.received += 1
        vals = [t, *p, *q]
        if not all(math.isfinite(v) for v in vals) or p[2] < self.z_floor:
            self._reject()
            return None
        q = rotate_yaw(q, self.heading_offset)
        yaw = yaw_of_quat(q)
        p = (float(p[0]), float(p[1]), float(p[2]))
        if self._t is not None:
            dt = t - self._t
            if dt <= 0.0:
                self._reject()
                return None
            scale = (3 + self._n_hold) / 3
            pred = [self._p[i] + self._v[i] * dt for i in range(3)]
            jump = math.sqrt(sum((p[i] - pred[i]) ** 2 for i in range(3)))
            if jump > self.thr_pos * scale or abs(_wrap(yaw - self._yaw)) > self.thr_yaw * scale:
                self._reject()
                return None
            v_raw = [(p[i] - self._p[i]) / dt for i in range(3)]
            r_raw = _wrap(yaw - self._yaw) / dt
            if self._n_hold > 0:               # after a gap: average over the gap, no filtering
                self._v, self._r = tuple(v_raw), r_raw
            else:
                a = dt / (self.tau + dt)
                self._v = tuple(self._v[i] + a * (v_raw[i] - self._v[i]) for i in range(3))
                self._r = self._r + a * (r_raw - self._r)
        self._t, self._p, self._yaw, self._n_hold = t, p, yaw, 0
        self.accepted += 1
        return TrackOutput(p, q, yaw, self._v, self._r)


@dataclass(frozen=True)
class MocapSimParams:
    rate: float = 100.0
    noise_pos: float = 0.0005
    noise_yaw: float = math.radians(0.1)
    drop_prob: float = 0.01
    drop_burst: int = 5
    delay: float = 0.003
    delay_jitter: float = 0.001
    clock_offset: float = 0.35
    clock_drift_ppm: float = 20.0
    stamp_mode: str = "source"          # "source" or "none" (zero stamps, to test the builder's fallback)
    seed: int = 0

    def __post_init__(self) -> None:
        if self.stamp_mode not in ("source", "none"):
            raise ValueError(f"MocapSimParams.stamp_mode must be 'source' or 'none', got {self.stamp_mode!r}")
        if not (self.rate > 0 and self.drop_burst >= 1 and 0 <= self.drop_prob <= 1):
            raise ValueError("MocapSimParams: rate > 0, drop_burst >= 1 and 0 <= drop_prob <= 1 required")

    def as_dict(self) -> dict:
        return asdict(self)


@dataclass(frozen=True)
class SimSample:
    release: float                            # when to publish, on the caller's clock
    stamp: float                              # header stamp on the emulated Vicon-computer clock (0.0 if none)
    p: tuple[float, float, float]
    q: tuple[float, float, float, float]


class MocapSimulator:
    """Turns ground truth into what the lab's Vicon node would publish (spec 6.2): rate decimation, noise,
    bursts of dropped frames, network delay with jitter, and a clock offset plus drift on the stamp."""

    def __init__(self, prm: MocapSimParams = MocapSimParams()):
        self.prm = prm
        self.rng = np.random.default_rng(prm.seed)
        self.dropped = 0
        self._t_next = None
        self._t0 = None
        self._skip = 0

    def process(self, t: float, p, yaw: float) -> SimSample | None:
        prm = self.prm
        period = 1.0 / prm.rate
        # schedule-based decimation: jittery input at the target rate passes, faster input is thinned out
        if self._t_next is not None and t < self._t_next - 0.45 * period:
            return None
        self._t_next = (t if self._t_next is None else self._t_next) + period
        if self._t_next < t:                     # resynchronise after a gap
            self._t_next = t + period
        if self._t0 is None:
            self._t0 = t
        if self._skip > 0:
            self._skip -= 1
            self.dropped += 1
            return None
        if self.rng.random() < prm.drop_prob:
            self._skip = int(self.rng.integers(0, prm.drop_burst))
            self.dropped += 1
            return None
        stamp = 0.0 if prm.stamp_mode == "none" else (
            t + prm.clock_offset + prm.clock_drift_ppm * 1e-6 * (t - self._t0))
        n = self.rng.normal(0.0, prm.noise_pos, 3)
        p_noisy = (p[0] + n[0], p[1] + n[1], p[2] + n[2])
        yaw_noisy = yaw + float(self.rng.normal(0.0, prm.noise_yaw))
        release = t + max(0.0, prm.delay + float(self.rng.uniform(-prm.delay_jitter, prm.delay_jitter)))
        return SimSample(release, stamp, p_noisy, quat_of_yaw(yaw_noisy))
