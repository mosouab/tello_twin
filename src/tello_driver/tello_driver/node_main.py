"""Common main() for the stack's nodes: spin until SIGINT or shutdown, without spurious tracebacks.

An exception raised while the context is shutting down (a callback racing the shutdown) is ignored; one raised
while the context is still running is a real error and propagates.
"""
from __future__ import annotations

from typing import Callable

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def run(make_node: Callable[[], Node], cleanup: Callable[[Node], None] | None = None) -> None:
    rclpy.init()
    node = make_node()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        if cleanup is not None:
            cleanup(node)
        node.destroy_node()
        rclpy.try_shutdown()
