"""Clock helpers shared by the nodes (spec: ages from arrival times on the monotonic clock, stamps from the ROS
clock). With use_sim_time, the monotonic clock is replaced by sim time, so ages follow the simulation."""
from __future__ import annotations

from typing import Callable

from builtin_interfaces.msg import Time
from rclpy.clock import Clock, ClockType
from rclpy.node import Node


def monotonic_clock(node: Node) -> Callable[[], float]:
    """Seconds on the steady clock, or on the node's ROS clock when use_sim_time is set."""
    if node.get_parameter("use_sim_time").value:
        return lambda: node.get_clock().now().nanoseconds * 1e-9
    steady = Clock(clock_type=ClockType.STEADY_TIME)
    return lambda: steady.now().nanoseconds * 1e-9


def stamp_to_seconds(stamp: Time) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def seconds_to_stamp(t: float) -> Time:
    sec = int(t // 1)
    return Time(sec=sec, nanosec=min(999_999_999, int(round((t - sec) * 1e9))))
