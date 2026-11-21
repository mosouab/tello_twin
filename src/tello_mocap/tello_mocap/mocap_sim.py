"""mocap_sim (spec 6.2): emulates the lab's Vicon node from /sim/truth/odom.

Publishes on the lab's raw topic and type (lab.yaml), with noise, dropped frames, network delay and a clock
offset plus drift on the stamps. The values used are latched on /sim/mocap_sim/config. The service
/mocap_sim/pause (SetBool) stops the output, to test the Vicon-loss failsafe.

Parameters (override MocapSimParams): rate, clock_offset, clock_drift_ppm, drop_prob, noise_pos, stamp_mode, seed.
"""
from __future__ import annotations

import collections

import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Header, String
from std_srvs.srv import SetBool

from tello_core.lab import load_lab
from tello_core.mocap import MocapSimParams, MocapSimulator, yaw_of_quat
from tello_driver.node_main import run
from tello_driver.qos import LATCHED
from tello_driver.ros_time import seconds_to_stamp, stamp_to_seconds

from .messages import TYPES, build_pose


class MocapSim(Node):
    def __init__(self) -> None:
        super().__init__("mocap_sim")
        p = self.declare_parameter
        self.lab = load_lab(p("lab_file", "").value)
        d = MocapSimParams()
        self.prm = MocapSimParams(rate=p("rate", d.rate).value, clock_offset=p("clock_offset", d.clock_offset).value,
                                  clock_drift_ppm=p("clock_drift_ppm", d.clock_drift_ppm).value,
                                  drop_prob=p("drop_prob", d.drop_prob).value,
                                  noise_pos=p("noise_pos", d.noise_pos).value,
                                  stamp_mode=p("stamp_mode", d.stamp_mode).value, seed=p("seed", d.seed).value)
        self.sim = MocapSimulator(self.prm)
        m = self.lab.mocap
        self.pub = self.create_publisher(TYPES[m.raw_type], m.raw_topic, 10)
        cfg = dict(self.prm.as_dict(), topic=m.raw_topic, type=m.raw_type)
        self.create_publisher(String, "/sim/mocap_sim/config", LATCHED).publish(
            String(data=yaml.safe_dump(cfg, sort_keys=True)))
        self.create_subscription(Odometry, "/sim/truth/odom", self._on_truth, qos_profile_sensor_data)
        self.create_service(SetBool, "~/pause", self._srv_pause)
        self.queue: collections.deque = collections.deque()
        self.paused = False
        self.create_timer(0.002, self._release)

    def _srv_pause(self, req, res):
        self.paused = bool(req.data)
        res.success, res.message = True, "paused" if self.paused else "running"
        return res

    def _on_truth(self, m: Odometry) -> None:
        pp, q = m.pose.pose.position, m.pose.pose.orientation
        s = self.sim.process(stamp_to_seconds(m.header.stamp), (pp.x, pp.y, pp.z), yaw_of_quat((q.x, q.y, q.z, q.w)))
        if s is not None and not self.paused:
            self.queue.append(s)

    def _release(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        m = self.lab.mocap
        while self.queue and self.queue[0].release <= now:
            s = self.queue.popleft()
            header = Header(stamp=seconds_to_stamp(s.stamp), frame_id="vicon_world")
            self.pub.publish(build_pose(m.raw_type, header, s.p, s.q, m.rigid_body))


def main() -> None:
    run(MocapSim)


if __name__ == "__main__":
    main()
