"""safety_console (spec 7.4): a terminal node to hold, land or stop the drone from the keyboard.

Run it in its own terminal (session.launch.py console:=xterm does that), or with
`ros2 run tello_flight safety_console`. Without a terminal on stdin it only displays.
"""
from __future__ import annotations

import os
import select
import sys
import termios
import tty

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger
from tello_interfaces.msg import FlightSegment, TelloEvent, TelloStatus

from tello_driver.qos import LATCHED

from .console_format import HELP, accept_keys, key_action, status_line

READ_LIMIT = 65536      # bytes drained per call at most, so an endless stream on stdin cannot stall the node


def read_keys(fd: int, timeout: float) -> str:
    """The keys pending on fd, read unbuffered (no key waits in a Python buffer); '' if none or ignored.

    Everything pending is drained in one call and filtered as a whole by accept_keys, so an escape sequence or a
    paste is dropped entirely: no tail of it is left for the next call."""
    if not select.select([fd], [], [], timeout)[0]:
        return ""
    chunk = os.read(fd, 1024)
    while chunk and len(chunk) < READ_LIMIT and select.select([fd], [], [], 0.0)[0]:
        more = os.read(fd, 1024)
        if not more:                    # end of file
            break
        chunk += more
    return accept_keys(chunk)


class SafetyConsole(Node):
    def __init__(self) -> None:
        super().__init__("safety_console")
        self.status = None
        self.segment = ""
        self.events: list[str] = []
        self.create_subscription(TelloStatus, "/tello/status", self._on_status, 10)
        self.create_subscription(FlightSegment, "/tello/flight/segment", self._on_segment, LATCHED)
        self.create_subscription(TelloEvent, "/tello/events", lambda m: self.events.append(m.type), LATCHED)
        self.cli = {name: self.create_client(Trigger, f"/tello/{name}") for name in ("land", "emergency", "takeoff")}
        self.cli_hold = self.create_client(SetBool, "/tello/hold")
        self.cli_abort = self.create_client(Trigger, "/flight_runner/abort")
        self.tty = sys.stdin.isatty()
        self.create_timer(0.2, self._draw)
        if self.tty:
            self.create_timer(0.05, self._poll_keys)
        print(HELP, flush=True)

    def _on_status(self, m) -> None:
        self.status = m

    def _on_segment(self, m) -> None:
        self.segment = m.name

    def _draw(self) -> None:
        print("\r" + status_line(self.status, self.segment, self.events)[:200] + "\033[K", end="", flush=True)

    def _poll_keys(self) -> None:
        for ch in read_keys(sys.stdin.fileno(), 0.0):
            action = key_action(ch)
            if action:
                self.do(action)

    def _call(self, cli, request, label: str) -> None:
        if not cli.service_is_ready():
            self.get_logger().warn(f"{label}: service not available")
            return
        cli.call_async(request).add_done_callback(
            lambda f: self.get_logger().info(f"{label}: {f.result().message}"))

    def do(self, action: str) -> None:
        if action == "hold_abort":
            self._call(self.cli_hold, SetBool.Request(data=True), "hold")
            self._call(self.cli_abort, Trigger.Request(), "abort")
        elif action == "release":
            self._call(self.cli_hold, SetBool.Request(data=False), "release")
        else:
            self._call(self.cli[action], Trigger.Request(), action)


def main() -> None:
    rclpy.init()
    node = SafetyConsole()
    saved = termios.tcgetattr(sys.stdin) if node.tty else None
    try:
        if node.tty:
            tty.setcbreak(sys.stdin.fileno())
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if saved is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, saved)
        print()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
