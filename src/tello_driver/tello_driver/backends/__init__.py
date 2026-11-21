"""Drone backends behind tello_node. Only `fake` exists in Plan 2; `sdk` comes in Plan 4 and `sim` in Plan 5."""
from __future__ import annotations

from typing import Callable

from tello_core.fake import FakeParams

from .base import Backend, BackendError
from .fake import FakeBackend, FakeBackendParams

__all__ = ["Backend", "BackendError", "FakeBackend", "FakeBackendParams", "make_backend"]


def make_backend(name: str, clock: Callable[[], float], options: dict | None = None) -> Backend:
    options = options or {}
    if name == "fake":
        dyn = FakeParams(delay=float(options.get("delay", FakeParams().delay)),
                         battery_start=float(options.get("battery_start", FakeParams().battery_start)))
        start = tuple(float(v) for v in options.get("start", (0.0, 0.0, 0.0)))
        return FakeBackend(clock, FakeBackendParams(dynamics=dyn, start=start, seed=int(options.get("seed", 0))))
    if name == "sdk":
        raise BackendError("backend 'sdk' (real Tello) is not implemented yet: Plan 4")
    if name == "sim":
        raise BackendError("backend 'sim' (Gazebo Harmonic) is not implemented yet: Plan 5")
    raise BackendError(f"unknown backend '{name}' (expected fake, sdk or sim)")
