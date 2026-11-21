"""flight.launch.py must refuse a plan that fails the dry run, before starting the recorder or the runner."""
import subprocess
from pathlib import Path

HERE = Path(__file__).parent


def test_failing_plan_starts_nothing(tmp_path):
    r = subprocess.run(["ros2", "launch", "tello_bringup", "flight.launch.py", f"plan:={HERE / 'data' / 'too_fast.yaml'}",
                        f"lab_file:={HERE / 'data' / 'lab_test.yaml'}", f"dataset_root:={tmp_path}",
                        "session:=refusal"], capture_output=True, text=True, timeout=60)
    out = r.stdout + r.stderr
    assert "dry run failed" in out and "exceeds the bring-up limit" in out
    assert not (tmp_path / "fake" / "raw" / "refusal").exists()
    assert "flight_runner" not in out.split("dry run failed")[-1]
