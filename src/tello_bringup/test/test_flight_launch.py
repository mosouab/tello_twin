"""Launch test: session + flight on the fake backend, with recording (spec section 8, success criterion 1 on fake).

The flight runs with recording required: the runner must wait for the recorder, and the bag must hold the
flight. Checks after shutdown read the MCAP bag with rosbag2_py.
"""
import tempfile
import unittest
from collections import Counter
from pathlib import Path

import launch
import launch_testing
import launch_testing.actions
import pytest
import rosbag2_py
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

from tello_driver.testing import Probe, isolate_domain

isolate_domain()

HERE = Path(__file__).parent
SHARE = Path(get_package_share_directory("tello_bringup"))
LAB = str(HERE / "data" / "lab_test.yaml")
PLAN = str(Path(get_package_share_directory("tello_flight")) / "plans" / "fake_short.yaml")
ROOT = Path(tempfile.mkdtemp(prefix="tello_flight_launch_"))
SESSION = "test_session"
FLIGHT = ROOT / "fake" / "raw" / SESSION / f"{SESSION}_01_fake_short"


@pytest.mark.launch_test
def generate_test_description():
    def include(name, **args):
        return launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(SHARE / "launch" / name)),
            launch_arguments={k: str(v) for k, v in args.items()}.items())
    return launch.LaunchDescription([
        include("session.launch.py", backend="fake", session=SESSION, lab_file=LAB, console="none",
                dataset_root=ROOT),
        launch.actions.TimerAction(period=2.0, actions=[
            include("flight.launch.py", plan=PLAN, backend="fake", session=SESSION, lab_file=LAB,
                    dataset_root=ROOT, shutdown_on_exit="false", notes="launch test")]),
        launch_testing.actions.ReadyToTest()])


class TestFlight(unittest.TestCase):
    def test_flight_completes(self):
        p = Probe()
        try:
            flight = FLIGHT / "flight.yaml"

            def result():
                return yaml.safe_load(flight.read_text()).get("result") if flight.exists() else None

            assert p.wait_for(lambda: result() not in (None, "running"), 120.0), result()
            assert result() == "completed"
        finally:
            p.close()


@launch_testing.post_shutdown_test()
class TestBag(unittest.TestCase):
    def test_bag_holds_the_flight(self):
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(FLIGHT / "bag"), storage_id="mcap"),
                    rosbag2_py.ConverterOptions("", ""))
        counts = Counter()
        while reader.has_next():
            counts[reader.read_next()[0]] += 1
        assert counts["/tello/cmd_sent"] > 500 and counts["/vicon/tello/pose"] > 1000
        assert counts["/tello/flight/segment"] == 6 and counts["/sim/mocap_sim/config"] >= 1
        assert counts["/sim/truth/odom"] > 1000 and counts["/tello/telemetry"] > 100
        rec = yaml.safe_load((FLIGHT / "flight.yaml").read_text())
        assert rec["events"].get("takeoff_refused", 0) == 0 and rec["plan"]["sha256"]
        assert (FLIGHT.parent / "session.yaml").exists()
