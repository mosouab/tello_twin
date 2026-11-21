from pathlib import Path

import pytest

from tello_bringup.layout import check_plan, next_flight_id, record_topics, session_dir
from tello_core.lab import load_lab
from tello_core.plan import load_plan

HERE = Path(__file__).parent
PKG = HERE.parent
LAB = load_lab(HERE / "data" / "lab_test.yaml")


def test_session_dir_layout(tmp_path):
    assert session_dir(str(tmp_path), "fake", "2026-11-21_a") == tmp_path.resolve() / "fake" / "raw" / "2026-11-21_a"
    assert session_dir(str(tmp_path), "sdk", "s").parts[-3] == "real"
    with pytest.raises(ValueError):
        session_dir(str(tmp_path), "gazebo", "s")
    with pytest.raises(ValueError):
        session_dir(str(tmp_path), "fake", "../escape")


def test_flight_ids_increment(tmp_path):
    assert next_flight_id(tmp_path, "s", "p") == "s_01_p"
    (tmp_path / "s_01_p").mkdir()
    (tmp_path / "s_07_other").mkdir()
    (tmp_path / "unrelated").mkdir()
    assert next_flight_id(tmp_path, "s", "p") == "s_08_p"


def test_record_topics():
    real = record_topics(str(PKG / "config" / "record_topics.yaml"), LAB, "sdk")
    fake = record_topics(str(PKG / "config" / "record_topics.yaml"), LAB, "fake")
    assert "/tello/cmd_sent" in real and "/vicon/tello/pose" in real and "/sim/truth/odom" not in real
    assert "/sim/mocap_sim/config" in fake and len(fake) == len(set(fake))


def test_check_plan_uses_lab_limits():
    assert not check_plan(load_plan(HERE / "data" / "too_fast.yaml"), LAB).ok
