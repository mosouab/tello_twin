"""tello_node: the only node that talks to a drone backend (spec section 5).

Every decision is taken by tello_core.drone.DroneCore. This node feeds it ARRIVAL times on the monotonic clock
(sim time with use_sim_time) and executes what its tick returns. Message stamps come from the ROS clock.

Parameters: lab_file (path to lab.yaml), backend (fake), fake_delay, fake_battery_start, fake_start,
fake_seed, backend_update_rate (Hz), status_rate (Hz).
"""
from __future__ import annotations

import dataclasses
import math

import yaml
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tello_interfaces.msg import TelloEvent, TelloStatus, TelloTelemetry

from tello_core.drone import DroneConfig, DroneCore
from tello_core.lab import load_lab
from tello_core.mocap import quat_of_yaw, yaw_of_quat
from tello_core.sdk import TelemetrySample
from tello_core.sticks import Sticks

from .backends import BackendError, make_backend
from .node_main import run
from .qos import LATCHED
from .ros_time import monotonic_clock

RECORDER_NODE = "tello_recorder"
TELEMETRY_FIELDS = [f.name for f in dataclasses.fields(TelemetrySample) if f.name != "raw"]


class TelloNode(Node):
    def __init__(self) -> None:
        super().__init__("tello_node")
        p = self.declare_parameter
        lab_file = p("lab_file", "").value
        self.backend_name = p("backend", "fake").value
        fake_opts = {"delay": p("fake_delay", 0.6).value, "battery_start": p("fake_battery_start", 100.0).value,
                     "start": list(p("fake_start", [0.0, 0.0, 0.0]).value), "seed": p("fake_seed", 0).value}
        update_rate = p("backend_update_rate", 200.0).value
        status_rate = p("status_rate", 10.0).value
        self.lab = load_lab(lab_file)
        self.mono = monotonic_clock(self)
        self.core = DroneCore(DroneConfig(safety=self.lab.safety, preflight=self.lab.preflight,
                                          stick_max=self.lab.control.stick_max,
                                          mocap_expected_rate=self.lab.mocap.expected_rate))
        self.backend = make_backend(self.backend_name, self.mono, fake_opts)
        self.backend.on_telemetry = self._on_telemetry
        self.backend.on_event = self._on_backend_event
        self.backend.on_truth = self._on_truth

        self.pub_cmd_sent = self.create_publisher(TwistStamped, "/tello/cmd_sent", 10)
        self.pub_tel = self.create_publisher(TelloTelemetry, "/tello/telemetry", 10)
        self.pub_tel_raw = self.create_publisher(String, "/tello/telemetry_raw", 10)
        self.pub_status = self.create_publisher(TelloStatus, "/tello/status", 10)
        self.pub_events = self.create_publisher(TelloEvent, "/tello/events", LATCHED)
        self.pub_info = self.create_publisher(String, "/tello/info", LATCHED)
        self.pub_truth = (self.create_publisher(Odometry, "/sim/truth/odom", 10)
                          if "truth" in self.backend.capabilities else None)
        self.create_subscription(TwistStamped, "/tello/cmd", self._on_cmd, 10)
        self.create_subscription(Odometry, "/tello/mocap/odom", self._on_odom, qos_profile_sensor_data)
        self.create_service(Trigger, "/tello/takeoff", self._srv_takeoff)
        self.create_service(Trigger, "/tello/land", self._srv_land)
        self.create_service(Trigger, "/tello/emergency", self._srv_emergency)
        self.create_service(Trigger, "/tello/reset", self._srv_reset)
        self.create_service(SetBool, "/tello/hold", self._srv_hold)

        self.backend.connect()
        self.core.on_connected(self.mono())
        info = dict(self.backend.info(), lab=self.lab.name)
        self.pub_info.publish(String(data=yaml.safe_dump(info, sort_keys=True)))
        cfg = self.backend.sim_config()
        if cfg is not None:
            self.create_publisher(String, "/sim/backend/config", LATCHED).publish(
                String(data=yaml.safe_dump(cfg, sort_keys=True)))
        self._status = None
        self.create_timer(1.0 / self.lab.send_rate, self._tick)
        self.create_timer(1.0 / update_rate, self.backend.update)
        self.create_timer(1.0 / status_rate, self._status_tick)
        self.get_logger().info(f"tello_node ready: backend {self.backend_name}, lab '{self.lab.name}'")

    # ---------------------------------------------------------------- inputs
    def _on_cmd(self, m: TwistStamped) -> None:
        tw = m.twist
        self.core.on_cmd(self.mono(), Sticks(tw.linear.x, tw.linear.y, tw.linear.z, tw.angular.z))

    def _on_odom(self, m: Odometry) -> None:
        pp, q, v = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.linear
        self.core.on_pose(self.mono(), (pp.x, pp.y, pp.z), yaw_of_quat((q.x, q.y, q.z, q.w)), (v.x, v.y, v.z))

    def _on_telemetry(self, s: TelemetrySample) -> None:
        self.core.on_telemetry(self.mono(), s)
        m = TelloTelemetry()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "tello"
        for name in TELEMETRY_FIELDS:
            setattr(m, name, float(getattr(s, name)))
        self.pub_tel.publish(m)
        self.pub_tel_raw.publish(String(data=s.raw))

    def _on_backend_event(self, kind: str, detail: str) -> None:
        self.core.on_backend_event(self.mono(), kind, detail)

    def _on_truth(self, tr) -> None:
        m = Odometry()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id, m.child_frame_id = "arena", "tello"
        m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = tr.p
        q = quat_of_yaw(tr.yaw)
        o = m.pose.pose.orientation
        o.x, o.y, o.z, o.w = q
        m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z = tr.v_world   # arena frame
        m.twist.twist.angular.z = tr.yaw_rate
        self.pub_truth.publish(m)

    # ---------------------------------------------------------------- services
    def _backend(self, action: str) -> None:
        try:
            getattr(self.backend, action)()
        except BackendError as e:
            self._publish_event("backend_error", f"{action}: {e}")

    def _srv_takeoff(self, req, res):
        res.success, res.message = self.core.request_takeoff(self.mono())
        if res.success:
            self._backend("takeoff")
        return res

    def _srv_land(self, req, res):
        res.success, res.message = self.core.request_land(self.mono())
        if res.success:
            self._backend("land")
        return res

    def _srv_emergency(self, req, res):
        self.core.emergency(self.mono())
        self._backend("emergency")
        res.success, res.message = True, "motors stopped"
        return res

    def _srv_reset(self, req, res):
        res.success = self.core.reset(self.mono())
        res.message = "reset" if res.success else "not in EMERGENCY"
        return res

    def _srv_hold(self, req, res):
        self.core.set_hold(self.mono(), req.data)
        res.success, res.message = True, "hold on" if req.data else "hold off"
        return res

    # ---------------------------------------------------------------- periodic
    def _tick(self) -> None:
        out = self.core.tick(self.mono())
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()      # taken just before sending
        msg.header.frame_id = "tello"
        self._backend_send(out.rc)
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = out.sent.x, out.sent.y, out.sent.z
        msg.twist.angular.z = out.sent.yaw
        self.pub_cmd_sent.publish(msg)
        for action in out.actions:
            self._backend(action)
        for kind, detail in out.events:
            self._publish_event(kind, detail)
        self._status = out.status
        if out.status_changed:
            self._publish_status()

    def _backend_send(self, rc) -> None:
        try:
            self.backend.send_sticks(rc)
        except BackendError as e:
            self._publish_event("backend_error", f"send_sticks: {e}")

    def _status_tick(self) -> None:
        infos = self.get_subscriptions_info_by_topic("/tello/cmd_sent")
        self.core.set_recording(any(i.node_name == RECORDER_NODE for i in infos))
        self._publish_status()

    def _publish_event(self, kind: str, detail: str) -> None:
        m = TelloEvent()
        m.header.stamp = self.get_clock().now().to_msg()
        m.type, m.detail = kind, detail
        self.pub_events.publish(m)
        text = f"event {kind} {detail}".strip()
        if kind in ("hold_off", "reset"):           # rclpy forbids changing severity at one call site
            self.get_logger().info(text)
        else:
            self.get_logger().warn(text)

    def _publish_status(self) -> None:
        s = self._status
        if s is None:
            return
        m = TelloStatus()
        m.header.stamp = self.get_clock().now().to_msg()
        m.state, m.backend, m.link_ok, m.hold = int(s.state), self.backend_name, s.link_ok, s.hold
        m.battery = float(s.battery)
        m.cmd_source = s.cmd_source
        m.mocap_age = float(s.mocap_age) if math.isfinite(s.mocap_age) else math.inf
        m.mocap_rate = float(s.mocap_rate)
        m.recording = self.core.recording
        self.pub_status.publish(m)


def main() -> None:
    run(TelloNode, cleanup=lambda node: node.backend.close())


if __name__ == "__main__":
    main()
