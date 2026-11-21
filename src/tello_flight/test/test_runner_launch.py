"""Launch test: flight_runner flies plans/fake_short.yaml on the fake backend and writes flight.yaml (spec 7.3)."""
import tempfile
import unittest
from pathlib import Path

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from tello_interfaces.msg import FlightSegment

from tello_driver.testing import LATCHED, Probe, isolate_domain

isolate_domain()

LAB = str(Path(__file__).parent / "data" / "lab_test.yaml")
PLAN = str(Path(get_package_share_directory("tello_flight")) / "plans" / "fake_short.yaml")
FLIGHT_DIR = Path(tempfile.mkdtemp(prefix="tello_runner_test_")) / "s1" / "s1_01_fake_short"


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
                                                  "backend": "fake", "notes": "launch test"}])]
    return launch.LaunchDescription([*nodes, launch_testing.actions.ReadyToTest()])


class TestRunner(unittest.TestCase):
    def test_plan_completes_and_flight_yaml_is_written(self):
        p = Probe([(FlightSegment, "/tello/flight/segment", LATCHED), (TwistStamped, "/tello/cmd", 50)])
        try:
            flight = FLIGHT_DIR / "flight.yaml"

            def result():
                return yaml.safe_load(flight.read_text()).get("result") if flight.exists() else None

            assert p.wait_for(lambda: result() not in (None, "running"), 90.0), result()
            rec = yaml.safe_load(flight.read_text())
            assert rec["result"] == "completed", rec
            assert rec["segments"] == 6 and rec["drone"]["serial"] == "FAKE0001"
            assert rec["battery"]["start"] is not None and rec["battery"]["end"] < rec["battery"]["start"]
            assert (FLIGHT_DIR.parent / "session.yaml").exists()
            assert [m.type for m in p.all("/tello/flight/segment")] == ["takeoff", "goto", "sync", "excite",
                                                                        "hover", "land"]
            assert p.count("/tello/cmd") > 300
        finally:
            p.close()
