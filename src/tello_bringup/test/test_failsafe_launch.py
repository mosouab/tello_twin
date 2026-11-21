"""Launch test: the safety layer through session.launch.py on the fake backend (spec 5.3, 7.4, 11.1).

The test plays the runner: it streams /tello/cmd at 50 Hz and calls the console's services.
"""
import tempfile
import threading
import time
import unittest
from pathlib import Path

import launch
import launch_testing.actions
import pytest
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from launch.launch_description_sources import PythonLaunchDescriptionSource
from std_srvs.srv import SetBool, Trigger
from tello_interfaces.msg import TelloEvent, TelloStatus

from tello_driver.testing import LATCHED, Probe, isolate_domain

isolate_domain()

HERE = Path(__file__).parent
SHARE = Path(get_package_share_directory("tello_bringup"))
LAB = str(HERE / "data" / "lab_norec.yaml")


@pytest.mark.launch_test
def generate_test_description():
    session = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(SHARE / "launch" / "session.launch.py")),
        launch_arguments={"backend": "fake", "session": "failsafe", "lab_file": LAB, "console": "none",
                          "dataset_root": tempfile.mkdtemp(prefix="tello_failsafe_")}.items())
    return launch.LaunchDescription([session, launch_testing.actions.ReadyToTest()])


class TestFailsafes(unittest.TestCase):
    def setUp(self):
        self.p = Probe([(TelloStatus, "/tello/status", 10), (TelloEvent, "/tello/events", LATCHED)])
        self.cmd_pub = self.p.publisher(TwistStamped, "/tello/cmd")
        self.streaming = threading.Event()
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._stream, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.stop.set()
        self.thread.join(timeout=1.0)
        self.p.close()

    def _stream(self):
        while not self.stop.is_set():
            if self.streaming.is_set():
                self.cmd_pub.publish(TwistStamped())
            time.sleep(0.02)

    def st(self):
        return self.p.last("/tello/status")

    def event_after(self, kind, t0):
        return any(e.type == kind for t, e in self.p.stamped("/tello/events") if t >= t0)

    def fly(self):
        p = self.p
        assert p.wait_for(lambda: self.st() is not None and self.st().state == TelloStatus.LANDED
                          and self.st().mocap_rate >= 90.0, 15.0)
        self.streaming.set()
        res = p.call(Trigger, "/tello/takeoff", Trigger.Request())
        assert res.success, res.message
        assert p.wait_for(lambda: self.st().state == TelloStatus.FLYING, 8.0)

    def test_hold_watchdog_vicon_loss_and_emergency(self):
        p = self.p
        self.fly()
        # hold overrides the runner, release gives control back
        p.call(SetBool, "/tello/hold", SetBool.Request(data=True))
        assert p.wait_for(lambda: self.st().cmd_source == "hold", 1.0)
        p.call(SetBool, "/tello/hold", SetBool.Request(data=False))
        assert p.wait_for(lambda: self.st().cmd_source == "runner", 1.0)
        # the runner stops publishing: hover within the 0.3 s watchdog (plus scheduling margin)
        t0 = time.monotonic()
        self.streaming.clear()
        assert p.wait_for(lambda: self.event_after("watchdog_hover", t0), 1.0)
        self.streaming.set()
        assert p.wait_for(lambda: self.st().cmd_source == "runner", 1.0)
        # Vicon lost: land within 1.5 s (plus margin)
        t0 = time.monotonic()
        p.call(SetBool, "/mocap_sim/pause", SetBool.Request(data=True))
        assert p.wait_for(lambda: self.event_after("mocap_lost_land", t0), 2.5)
        p.call(SetBool, "/mocap_sim/pause", SetBool.Request(data=False))
        assert p.wait_for(lambda: self.st().state == TelloStatus.LANDED, 10.0)
        # fly again, then emergency stops the motors and stays until reset
        self.fly()
        p.call(Trigger, "/tello/emergency", Trigger.Request())
        assert p.wait_for(lambda: self.st().state == TelloStatus.EMERGENCY, 1.0)
        assert not p.call(Trigger, "/tello/takeoff", Trigger.Request()).success
        assert p.call(Trigger, "/tello/reset", Trigger.Request()).success
        assert p.wait_for(lambda: self.st().state == TelloStatus.LANDED, 3.0)
