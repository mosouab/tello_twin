"""In-process fake backend (spec section 5.4): tello_core.fake dynamics, SDK-rounded telemetry at 10 Hz with an
emulated arrival delay, and ground truth for /sim/truth/odom. Wall clock only (the node's monotonic clock)."""
from __future__ import annotations

import collections
from dataclasses import dataclass
from typing import Callable

import numpy as np

from tello_core.fake import FakeDynamics, FakeParams

from .base import Backend, BackendError


@dataclass(frozen=True)
class FakeBackendParams:
    dynamics: FakeParams = FakeParams()
    start: tuple[float, float, float] = (0.0, 0.0, 0.0)
    start_yaw: float = 0.0
    telemetry_rate: float = 10.0
    telemetry_delay: float = 0.04
    telemetry_jitter: float = 0.01
    truth_rate: float = 100.0
    seed: int = 0


class FakeBackend(Backend):
    name = "fake"
    capabilities = frozenset({"telemetry", "truth"})

    def __init__(self, clock: Callable[[], float], params: FakeBackendParams = FakeBackendParams()):
        super().__init__()
        self.clock, self.prm = clock, params
        self.dyn: FakeDynamics | None = None
        self._rng = np.random.default_rng(params.seed)
        self._tel_queue: collections.deque = collections.deque()
        self._t_tel = self._t_truth = 0.0

    def connect(self) -> None:
        t = self.clock()
        self.dyn = FakeDynamics(self.prm.dynamics, position=self.prm.start, yaw=self.prm.start_yaw, t0=t)
        self._t_tel = self._t_truth = t
        self.on_event("connected", "")

    def close(self) -> None:
        self.dyn = None

    def _require(self) -> FakeDynamics:
        if self.dyn is None:
            raise BackendError("fake backend not connected")
        return self.dyn

    def send_sticks(self, rc) -> None:
        self._require().set_rc(self.clock(), rc)

    def takeoff(self) -> None:
        self._require().takeoff(self.clock())

    def land(self) -> None:
        self._require().land(self.clock())

    def emergency(self) -> None:
        self._require().emergency(self.clock())

    def update(self) -> None:
        if self.dyn is None:
            return
        t = self.clock()
        for e in self.dyn.step(t):
            self.on_event(e, "")
        prm = self.prm
        if t - self._t_tel > 0.5:                      # after a long stall, resynchronise instead of bursting
            self._t_tel = t
        while t >= self._t_tel:
            delay = max(0.0, prm.telemetry_delay + float(self._rng.uniform(-prm.telemetry_jitter,
                                                                           prm.telemetry_jitter)))
            self._tel_queue.append((self._t_tel + delay, self.dyn.telemetry()))
            self._t_tel += 1.0 / prm.telemetry_rate
        while self._tel_queue and self._tel_queue[0][0] <= t:
            self.on_telemetry(self._tel_queue.popleft()[1])
        if t - self._t_truth > 0.5:
            self._t_truth = t
        while t >= self._t_truth:
            self.on_truth(self.dyn.truth())
            self._t_truth += 1.0 / prm.truth_rate

    def info(self) -> dict:
        return {"backend": "fake", "serial": "FAKE0001", "sdk": "fake"}

    def sim_config(self) -> dict:
        d = self.prm.dynamics
        return {"backend": "fake", "cmd_delay": d.delay, "K": list(d.K), "k": list(d.k),
                "telemetry_rate": self.prm.telemetry_rate, "telemetry_delay": self.prm.telemetry_delay,
                "telemetry_jitter": self.prm.telemetry_jitter, "truth_rate": self.prm.truth_rate,
                "battery_start": d.battery_start, "seed": self.prm.seed}
