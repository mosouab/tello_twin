"""flight_runner (spec 7.3): runs one flight plan, then exits.

All decisions are in tello_core.runner.RunnerCore. The node publishes /tello/cmd (50 Hz while FLYING),
announces segments on /tello/flight/segment, forwards plan events to /tello/events, calls /tello/takeoff and
/tello/land, offers /flight_runner/abort, and writes flight.yaml (and session.yaml if absent).

Parameters: plan_file, lab_file, flight_dir, flight_id, session, backend, notes, repo_dir.
"""
from __future__ import annotations

import json
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tello_interfaces.msg import FlightSegment, TelloEvent, TelloStatus

from tello_core.flight_state import FlightState
from tello_core.lab import load_lab
from tello_core.mocap import yaw_of_quat
from tello_core.plan import load_plan
from tello_core.runner import RunnerConfig, RunnerCore, flight_record
from tello_driver.qos import LATCHED
from tello_driver.ros_time import monotonic_clock

from .records import now_iso, session_record, software_version, write_yaml


class FlightRunner(Node):
    def __init__(self) -> None:
        super().__init__("flight_runner")
        p = self.declare_parameter
        self.plan_file = p("plan_file", "").value
        self.lab = load_lab(p("lab_file", "").value)
        self.flight_dir = Path(p("flight_dir", "").value)
        self.flight_id = p("flight_id", "").value
        self.session = p("session", "").value
        self.backend = p("backend", "fake").value
        self.notes = p("notes", "").value
        self.software = software_version(p("repo_dir", ".").value)
        self.plan = load_plan(self.plan_file)
        self.mono = monotonic_clock(self)
        pre = self.lab.preflight
        self.core = RunnerCore(self.plan, self.lab.soft_box, self.lab.executor,
                               RunnerConfig(require_recording=pre.require_recording, mocap_max_age=pre.mocap_max_age,
                                            mocap_min_rate=pre.mocap_min_rate_ratio * self.lab.mocap.expected_rate))
        self.soft_used = (self.lab.soft_box.shrink(self.plan.volume_shrink) if self.plan.volume_shrink > 0
                          else self.lab.soft_box)
        self.info: dict = {}
        self.started = now_iso()
        self.done = False
        self.t_node_start = self.get_clock().now().nanoseconds

        self.pub_cmd = self.create_publisher(TwistStamped, "/tello/cmd", 10)
        self.pub_seg = self.create_publisher(FlightSegment, "/tello/flight/segment", LATCHED)
        self.pub_events = self.create_publisher(TelloEvent, "/tello/events", LATCHED)
        self.create_subscription(TelloStatus, "/tello/status", self._on_status, 10)
        self.create_subscription(Odometry, "/tello/mocap/odom", self._on_odom, qos_profile_sensor_data)
        self.create_subscription(TelloEvent, "/tello/events", self._on_event, LATCHED)
        self.create_subscription(String, "/tello/info", self._on_info, LATCHED)
        self.cli_takeoff = self.create_client(Trigger, "/tello/takeoff")
        self.cli_land = self.create_client(Trigger, "/tello/land")
        self.create_service(Trigger, "~/abort", self._srv_abort)
        self.create_timer(1.0 / self.lab.send_rate, self._tick)
        self._write_flight()
        self.get_logger().info(f"flight {self.flight_id}: plan '{self.plan.name}' ({len(self.plan.segments)} segments)")

    # ---------------------------------------------------------------- inputs
    def _on_status(self, m: TelloStatus) -> None:
        self.core.on_status(self.mono(), FlightState(m.state), m.hold, m.recording, m.battery, m.mocap_age,
                            m.mocap_rate)

    def _on_odom(self, m: Odometry) -> None:
        pp, q, v = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.linear
        self.core.on_pose(self.mono(), (pp.x, pp.y, pp.z), yaw_of_quat((q.x, q.y, q.z, q.w)), (v.x, v.y, v.z),
                          m.twist.twist.angular.z)

    def _on_event(self, m: TelloEvent) -> None:
        # The latched topic replays events of earlier flights: count only events stamped after this node started.
        if Time.from_msg(m.header.stamp).nanoseconds >= self.t_node_start:
            self.core.on_event(m.type)

    def _on_info(self, m: String) -> None:
        self.info = yaml.safe_load(m.data) or {}
        session_file = self.flight_dir.parent / "session.yaml"
        if not session_file.exists():
            write_yaml(session_file, session_record(session=self.session, backend=self.backend,
                                                    drone_info=self.info, lab_file=self.lab.source_path,
                                                    lab_name=self.lab.name))

    def _srv_abort(self, req, res):
        self.core.abort("operator")
        res.success, res.message = True, "plan aborted"
        return res

    # ---------------------------------------------------------------- periodic
    def _request(self, kind: str) -> None:
        cli = self.cli_takeoff if kind == "takeoff" else self.cli_land
        if not cli.service_is_ready():
            self.get_logger().error(f"/tello/{kind} unavailable")
            if kind == "takeoff":
                self.core.on_takeoff_result(False, "takeoff service unavailable")
            return
        fut = cli.call_async(Trigger.Request())
        if kind == "takeoff":
            fut.add_done_callback(lambda f: self.core.on_takeoff_result(f.result().success, f.result().message))

    def _tick(self) -> None:
        if self.done:
            return
        out = self.core.tick(self.mono())
        if out.request:
            self._request(out.request)
        if out.started is not None:
            s = out.started
            m = FlightSegment(index=s.seq, type=s.segment.type, name=s.name, closed_loop=s.segment.closed_loop,
                              params_json=json.dumps(s.segment.params, sort_keys=True))
            m.header.stamp = self.get_clock().now().to_msg()
            self.pub_seg.publish(m)
        for e in out.events:
            ev = TelloEvent(type=e, detail=self.flight_id)
            ev.header.stamp = self.get_clock().now().to_msg()
            self.pub_events.publish(ev)
        if out.publish_cmd:
            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = "tello"
            s = out.sticks
            cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.linear.z, cmd.twist.angular.z = s.x, s.y, s.z, s.yaw
            self.pub_cmd.publish(cmd)
        if out.finished:
            self.done = True
            self._write_flight(ended=now_iso())
            self.get_logger().info(f"flight {self.flight_id} finished: {self.core.result}")

    def _write_flight(self, ended: str | None = None) -> None:
        rec = flight_record(flight_id=self.flight_id, session=self.session, started=self.started,
                            plan_path=self.plan_file, plan=self.plan, backend=self.backend, drone_info=self.info,
                            lab=self.lab, soft_box=self.soft_used, hard_box=self.lab.hard_box, notes=self.notes,
                            software=self.software, runner=self.core, ended=ended)
        write_yaml(self.flight_dir / "flight.yaml", rec)


def main() -> None:
    rclpy.init()
    node = FlightRunner()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.core.interrupt(node.mono(), "interrupted")
        node._write_flight(ended=now_iso())
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
