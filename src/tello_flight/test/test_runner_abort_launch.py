"""Launch test: hold mid-flight aborts the plan, and flight_runner settles until the drone is down before it
finalises flight.yaml (spec 7.3, 15.17, 15.18)."""
import tempfile
import time
import unittest
from pathlib import Path

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from rcl_interfaces.msg import Log
from std_srvs.srv import SetBool
from tello_interfaces.msg import FlightSegment, TelloStatus

from tello_driver.testing import LATCHED, Probe, isolate_domain

isolate_domain()

LAB = str(Path(__file__).parent / "data" / "lab_test.yaml")
PLAN = str(Path(get_package_share_directory("tello_flight")) / "plans" / "fake_short.yaml")
FLIGHT_DIR = Path(tempfile.mkdtemp(prefix="tello_runner_abort_")) / "s1" / "s1_01_fake_short"


@pytest.mark.launch_test
def generate_test_description():
    lab = [{"lab_file": LAB}]
    nodes = [launch_ros.actions.Node(package="tello_driver", executable="tello_node", name="tello_node",
                                     parameters=[{"lab_file": LAB, "backend": "fake"}]),
             launch_ros.actions.Node(package="tello_mocap", executable="mocap_sim", name="mocap_sim", parameters=lab),
             launch_ros.actions.Node(package="tello_mocap", executable="mocap_adapter", name="mocap_adapter",
                                     parameters=lab),
             launch_ros.actions.Node(package="tello_flight", executable="flight_runner", name="flight_runner",
                                     parameters=[{"lab_file": LAB, "plan_file": PLAN, "flight_dir": str(FLIGHT_DIR),
                                                  "flight_id": "s1_01_fake_short", "session": "s1",
                                                  "backend": "fake", "notes": "abort launch test"}])]
    return launch.LaunchDescription([*nodes, launch_testing.actions.ReadyToTest()])


class TestAbort(unittest.TestCase):
    def test_hold_aborts_and_the_record_waits_for_the_landing(self):
        p = Probe([(FlightSegment, "/tello/flight/segment", LATCHED), (TelloStatus, "/tello/status", 10),
                   (TwistStamped, "/tello/cmd", 50), (Log, "/rosout", 1000)])
        try:
            flight = FLIGHT_DIR / "flight.yaml"

            def record():
                return yaml.safe_load(flight.read_text()) if flight.exists() else None

            def result():
                rec = record()
                return rec.get("result") if rec else None

            def state():
                return p.last("/tello/status").state

            assert p.wait_for(lambda: any(m.type == "sync" for m in p.all("/tello/flight/segment")), 60.0)
            assert state() == TelloStatus.FLYING
            p.call(SetBool, "/tello/hold", SetBool.Request(data=True))
            t_hold = time.monotonic()
            assert p.wait_for(lambda: result() not in (None, "running"), 30.0), result()
            # the record is finalised only once the drone is down (the fake needs about 3 s to land)
            assert p.wait_for(lambda: state() == TelloStatus.LANDED, 0.5), state()
            rec = record()
            assert rec["result"] == "aborted:hold", rec
            # the runner's own final event is counted exactly once, like tello_node's events
            assert rec["events"].get("plan_aborted:hold") == 1 and rec["events"].get("hold_on") == 1, rec["events"]
            assert isinstance(rec["start_stamp"], float) and abs(rec["start_stamp"] - time.time()) < 120.0
            assert rec["battery"]["end"] < rec["battery"]["start"]
            # no commands while settling, and the land reply is logged
            assert not [m for t, m in p.stamped("/tello/cmd") if t > t_hold + 0.2]
            assert p.wait_for(lambda: any(m.name == "flight_runner" and "/tello/land" in m.msg
                                          for m in p.all("/rosout")), 2.0)
        finally:
            p.close()
