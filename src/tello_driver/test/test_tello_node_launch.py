"""Launch test: tello_node with the fake backend and no Vicon (spec 5)."""
import time
import unittest
from pathlib import Path

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tello_interfaces.msg import TelloEvent, TelloStatus, TelloTelemetry

from tello_driver.testing import LATCHED, Probe, isolate_domain

isolate_domain()

LAB = str(Path(__file__).parent / "data" / "lab_test.yaml")


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(package="tello_driver", executable="tello_node", name="tello_node",
                                   parameters=[{"lab_file": LAB, "backend": "fake"}], output="screen")
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()])


class TestTelloNode(unittest.TestCase):
    def setUp(self):
        self.probe = Probe([(TelloStatus, "/tello/status", 10), (TelloTelemetry, "/tello/telemetry", 10),
                            (TwistStamped, "/tello/cmd_sent", 10), (TelloEvent, "/tello/events", LATCHED),
                            (String, "/tello/info", LATCHED)])

    def tearDown(self):
        self.probe.close()

    def test_streams_status_telemetry_and_commands(self):
        p = self.probe
        assert p.wait_for(lambda: p.last("/tello/status") is not None
                          and p.last("/tello/status").state == TelloStatus.LANDED, 10.0)
        n_tel, n_cmd = p.count("/tello/telemetry"), p.count("/tello/cmd_sent")
        time.sleep(2.0)
        assert 15 <= p.count("/tello/telemetry") - n_tel <= 25
        assert 80 <= p.count("/tello/cmd_sent") - n_cmd <= 120
        assert "FAKE0001" in p.last("/tello/info").data
        st = p.last("/tello/status")
        assert st.backend == "fake" and st.link_ok and st.cmd_source == "none" and st.mocap_age == float("inf")

    def test_takeoff_refused_without_vicon(self):
        p = self.probe
        assert p.wait_for(lambda: p.last("/tello/status") is not None
                          and p.last("/tello/status").state == TelloStatus.LANDED, 10.0)
        res = p.call(Trigger, "/tello/takeoff", Trigger.Request())
        assert not res.success and "no recent Vicon pose" in res.message
        assert p.wait_for(lambda: any(e.type == "takeoff_refused" for e in p.all("/tello/events")), 2.0)
