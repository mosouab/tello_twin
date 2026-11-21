"""Test helper for launch tests: a ROS node that records messages and calls services from test code.

    probe = Probe([(TelloStatus, "/tello/status", 10)])
    assert probe.wait_for(lambda: probe.last("/tello/status") is not None, 10.0)
    res = probe.call(Trigger, "/tello/takeoff", Trigger.Request())
    probe.close()
"""
from __future__ import annotations

import os
import random
import threading
import time
from collections import defaultdict

import rclpy
from rclpy.executors import SingleThreadedExecutor
from .qos import LATCHED  # noqa: F401  (re-exported for tests)


def isolate_domain() -> int:
    """Put this test process (and the nodes it launches) on a random local-only ROS domain, so stray nodes from
    a previous run or a real session cannot interfere, and test traffic never leaves the machine. Call at
    import time of each launch test module.

    - `ROS_DOMAIN_ID` is drawn from 1-101: with the DDS port mapping (7400 + 250 x domain), domains 102-214 use
      UDP ports inside the Linux ephemeral range (32768-60999), where another program may already hold them.
    - `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` limits discovery to this machine (not the lab network).
    """
    domain = random.randint(1, 101)
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    return domain


class Probe:
    def __init__(self, topics=(), name: str = "test_probe"):
        self.ctx = rclpy.Context()
        rclpy.init(context=self.ctx)
        self.node = rclpy.create_node(name, context=self.ctx)
        self._lock = threading.Lock()
        self._msgs: dict[str, list] = defaultdict(list)
        for msg_type, topic, qos in topics:
            self.node.create_subscription(msg_type, topic, self._recorder(topic), qos)
        self._exe = SingleThreadedExecutor(context=self.ctx)
        self._exe.add_node(self.node)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _recorder(self, topic):
        def cb(msg):
            with self._lock:
                self._msgs[topic].append((time.monotonic(), msg))
        return cb

    def _spin(self) -> None:
        while self.ctx.ok():
            try:
                self._exe.spin_once(timeout_sec=0.05)
            except Exception:            # context shut down while spinning
                break

    def all(self, topic: str) -> list:
        with self._lock:
            return [m for _, m in self._msgs[topic]]

    def stamped(self, topic: str) -> list:
        with self._lock:
            return list(self._msgs[topic])

    def last(self, topic: str):
        msgs = self.all(topic)
        return msgs[-1] if msgs else None

    def count(self, topic: str) -> int:
        with self._lock:
            return len(self._msgs[topic])

    def clear(self, topic: str) -> None:
        with self._lock:
            self._msgs[topic].clear()

    def wait_for(self, predicate, timeout: float, period: float = 0.05) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if predicate():
                return True
            time.sleep(period)
        return predicate()

    def publisher(self, msg_type, topic: str, qos=10):
        return self.node.create_publisher(msg_type, topic, qos)

    def call(self, srv_type, name: str, request, timeout: float = 5.0):
        cli = self.node.create_client(srv_type, name)
        if not cli.wait_for_service(timeout_sec=timeout):
            raise AssertionError(f"service {name} not available")
        fut = cli.call_async(request)
        t0 = time.monotonic()
        while not fut.done():
            if time.monotonic() - t0 > timeout:
                raise AssertionError(f"service {name} did not answer within {timeout} s")
            time.sleep(0.01)
        return fut.result()

    def close(self) -> None:
        self.node.destroy_node()
        rclpy.shutdown(context=self.ctx)
        self._thread.join(timeout=2.0)
