"""Launch test: tello_node (fake) + mocap_sim + mocap_adapter (spec 6.1, 6.2, 5.3)."""
import time
import unittest
from pathlib import Path

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, Trigger
from tello_interfaces.msg import TelloEvent, TelloStatus

from tello_driver.ros_time import stamp_to_seconds
from tello_driver.testing import LATCHED, Probe, isolate_domain

isolate_domain()

LAB = str(Path(__file__).parent / "data" / "lab_test.yaml")


@pytest.mark.launch_test
def generate_test_description():
    params = [{"lab_file": LAB}]
    nodes = [launch_ros.actions.Node(package="tello_driver", executable="tello_node", name="tello_node",
                                     parameters=[{"lab_file": LAB, "backend": "fake"}]),
             launch_ros.actions.Node(package="tello_mocap", executable="mocap_sim", name="mocap_sim",
                                     parameters=params),
             launch_ros.actions.Node(package="tello_mocap", executable="mocap_adapter", name="mocap_adapter",
                                     parameters=params)]
    return launch.LaunchDescription([*nodes, launch_testing.actions.ReadyToTest()])


class TestMocapChain(unittest.TestCase):
    def setUp(self):
        self.p = Probe([(TelloStatus, "/tello/status", 10), (PoseStamped, "/vicon/tello/pose", 50),
                        (Odometry, "/tello/mocap/odom", 50), (TelloEvent, "/tello/events", LATCHED)])

    def tearDown(self):
        self.p.close()

    def status(self):
        return self.p.last("/tello/status")

    def test_vicon_offset_does_not_affect_safety_then_loss_lands(self):
        p = self.p
        assert p.wait_for(lambda: self.status() is not None and self.status().state == TelloStatus.LANDED
                          and self.status().mocap_rate >= 90.0, 15.0)
        # the emulated Vicon computer's clock is 0.35 s ahead: stamps are offset, safety ages are not
        recv, msg = p.stamped("/vicon/tello/pose")[-1]
        offset = stamp_to_seconds(msg.header.stamp) - time.time()
        assert 0.30 < offset < 0.40
        assert self.status().mocap_age < 0.05
        assert p.count("/tello/mocap/odom") > 50
        res = p.call(Trigger, "/tello/takeoff", Trigger.Request())
        assert res.success, res.message
        assert p.wait_for(lambda: self.status().state == TelloStatus.FLYING, 8.0)
        p.call(SetBool, "/mocap_sim/pause", SetBool.Request(data=True))
        t0 = time.monotonic()
        assert p.wait_for(lambda: any(e.type == "mocap_lost_land" for e in p.all("/tello/events")), 3.0)
        assert time.monotonic() - t0 < 2.0
        assert p.wait_for(lambda: self.status().state in (TelloStatus.LANDING, TelloStatus.LANDED), 1.0)
