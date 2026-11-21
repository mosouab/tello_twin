"""Backend interface (spec section 4.4). Backends only do input/output; all policy lives in tello_core."""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Callable


class BackendError(RuntimeError):
    pass


class Backend(ABC):
    name = "base"
    capabilities: frozenset[str] = frozenset()      # e.g. {"telemetry", "truth"}

    def __init__(self) -> None:
        self.on_telemetry: Callable = lambda sample: None           # tello_core.sdk.TelemetrySample
        self.on_event: Callable = lambda kind, detail: None          # e.g. ("takeoff_done", "")
        self.on_truth: Callable = lambda truth: None                 # tello_core.fake.Truth (sim/fake only)

    @abstractmethod
    def connect(self) -> None: ...

    @abstractmethod
    def close(self) -> None: ...

    @abstractmethod
    def send_sticks(self, rc: tuple[int, int, int, int]) -> None: ...

    @abstractmethod
    def takeoff(self) -> None: ...

    @abstractmethod
    def land(self) -> None: ...

    @abstractmethod
    def emergency(self) -> None: ...

    def update(self) -> None:
        """Called periodically by tello_node (in-process backends step their model here)."""

    def info(self) -> dict:
        return {"backend": self.name}

    def sim_config(self) -> dict | None:
        """Injected values (delays, noise) for /sim/backend/config; None for real hardware."""
        return None
