"""mocap_adapter (spec 6.1): raw Vicon topic -> /tello/mocap/pose and /tello/mocap/odom, validated.

Header stamps are the SOURCE stamps and are never rewritten. Velocity: finite differences on source stamps,
low-pass filtered (tello_core.mocap.MocapTracker). Odometry twist is expressed in the arena frame.
Counts (received / accepted / rejected) are published on /diagnostics at 1 Hz.
"""
from __future__ import annotations

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from tello_core.lab import load_lab
from tello_core.mocap import MocapTracker
from tello_driver.node_main import run
from tello_driver.ros_time import stamp_to_seconds

from .messages import TYPES, read_pose


class MocapAdapter(Node):
    def __init__(self) -> None:
        super().__init__("mocap_adapter")
        self.lab = load_lab(self.declare_parameter("lab_file", "").value)
        m = self.lab.mocap
        self.tracker = MocapTracker(thr_pos=m.thr_pos, thr_yaw=m.thr_yaw, z_floor=m.z_floor,
                                    heading_offset=m.heading_offset)
        self.pub_pose = self.create_publisher(PoseStamped, "/tello/mocap/pose", 10)
        self.pub_odom = self.create_publisher(Odometry, "/tello/mocap/odom", 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_subscription(TYPES[m.raw_type], m.raw_topic, self._on_raw, qos_profile_sensor_data)
        self.create_timer(1.0, self._diag)
        self.get_logger().info(f"mocap_adapter: {m.raw_type} on {m.raw_topic} (source {m.source})")

    def _on_raw(self, msg) -> None:
        header, p, q = read_pose(msg)
        out = self.tracker.update(stamp_to_seconds(header.stamp), p, q)
        if out is None:
            return
        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = out.p
        o = pose.pose.orientation
        o.x, o.y, o.z, o.w = out.q
        self.pub_pose.publish(pose)
        odom = Odometry()
        odom.header, odom.child_frame_id = header, self.lab.mocap.rigid_body
        odom.pose.pose = pose.pose
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = out.v
        odom.twist.twist.angular.z = out.yaw_rate
        self.pub_odom.publish(odom)

    def _diag(self) -> None:
        tr = self.tracker
        st = DiagnosticStatus(name="mocap_adapter", hardware_id=self.lab.mocap.rigid_body,
                              level=DiagnosticStatus.OK, message="ok")
        st.values = [KeyValue(key=k, value=str(v)) for k, v in
                     (("received", tr.received), ("accepted", tr.accepted), ("rejected", tr.rejected))]
        arr = DiagnosticArray(status=[st])
        arr.header.stamp = self.get_clock().now().to_msg()
        self.pub_diag.publish(arr)


def main() -> None:
    run(MocapAdapter)


if __name__ == "__main__":
    main()
