import copy
from pathlib import Path

import pytest
import yaml

from tello_core.lab import LabError, load_lab, parse_lab
from tello_core.safety import Box

DATA = Path(__file__).parent / "data"
EXAMPLE = yaml.safe_load((DATA / "lab_example.yaml").read_text())


def variant(**changes):
    """Copy of the example with dotted-path changes, e.g. variant(**{"volume.soft_box": [...]})."""
    d = copy.deepcopy(EXAMPLE)
    for path, value in changes.items():
        *parents, leaf = path.split(".")
        node = d
        for p in parents:
            node = node.setdefault(p, {})
        if value is None:
            node.pop(leaf, None)
        else:
            node[leaf] = value
    return d


def problems_of(data) -> str:
    with pytest.raises(LabError) as e:
        parse_lab(data)
    return "\n".join(e.value.problems)


def test_example_loads():
    lab = load_lab(DATA / "lab_example.yaml")
    assert lab.soft_box == Box(-2.0, 2.0, -2.0, 2.0, 0.5, 2.0)
    assert lab.hard_box == Box(-2.5, 2.5, -2.5, 2.5, 0.2, 2.5)
    assert lab.safety.hard_box == lab.hard_box and lab.preflight.soft_box == lab.soft_box
    assert lab.control.stick_max == (0.4, 0.4, 0.4, 0.4) and lab.control.cmd_delay == 0.7
    assert lab.executor.stop_time == 0.5 and lab.safety.lookahead == pytest.approx(1.2)
    assert lab.preflight.require_recording is True and lab.send_rate == 50.0
    assert lab.mocap.raw_topic == "/vicon/tello/pose" and lab.mocap.expected_rate == 100.0
    assert lab.setup["prop_guards"] is True and lab.source_path.endswith("lab_example.yaml")


def test_soft_box_must_lie_inside_hard_box():
    assert "soft_box must lie inside hard_box" in problems_of(
        variant(**{"volume.soft_box": [-3.0, 2.0, -2.0, 2.0, 0.5, 2.0]}))


def test_soft_box_must_straddle_the_takeoff_height():
    assert "takeoff height" in problems_of(variant(**{"volume.soft_box": [-2.0, 2.0, -2.0, 2.0, 0.9, 2.0]}))
    assert "takeoff height" in problems_of(variant(**{"volume.soft_box": [-2.0, 2.0, -2.0, 2.0, 0.3, 0.7]}))


def test_volume_is_required():
    assert "volume.soft_box" in problems_of(variant(**{"volume": None}))


def test_bad_values_are_all_reported():
    msgs = problems_of(variant(**{"mocap.raw_type": "Pose", "mocap.raw_topic": "vicon", "mocap.expected_rate": 0,
                                  "limits.stick_max": [0.4, 0.4], "safety.battery_lnad": 10, "extra": 1}))
    for fragment in ("mocap.raw_type", "mocap.raw_topic", "mocap.expected_rate", "limits.stick_max",
                     "safety: unknown key 'battery_lnad'", "unknown top-level key 'extra'"):
        assert fragment in msgs


def test_config_class_errors_become_lab_errors():
    assert "cmd_timeout_hover must be lower than cmd_timeout_land" in problems_of(
        variant(**{"safety.cmd_timeout_hover": 6.0}))
    assert "stick_max must be 4 values in (0, 1]" in problems_of(variant(**{"limits.stick_max": [0.4, 0.4, 0.4, 1.5]}))


def test_require_recording_must_be_boolean():
    assert "preflight.require_recording" in problems_of(variant(**{"preflight.require_recording": "yes"}))


def test_unreadable_or_bad_yaml(tmp_path):
    with pytest.raises(LabError) as e:
        load_lab(tmp_path / "missing.yaml")
    assert "cannot read" in e.value.problems[0]
    bad = tmp_path / "bad.yaml"
    bad.write_text("volume: [unclosed\n")
    with pytest.raises(LabError) as e:
        load_lab(bad)
    assert "YAML syntax error" in e.value.problems[0]
