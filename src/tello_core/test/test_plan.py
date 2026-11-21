from pathlib import Path

import pytest

from tello_core.plan import PlanError, load_plan, parse_plan

DATA = Path(__file__).parent / "data"


def minimal(*segments, stick_max=(0.4, 0.4, 0.4, 0.4)):
    return {"name": "t", "limits": {"stick_max": list(stick_max)},
            "segments": [{"type": "takeoff"}, *segments, {"type": "land"}]}


def problems_of(data) -> str:
    with pytest.raises(PlanError) as e:
        parse_plan(data)
    return "\n".join(e.value.problems)


def test_example_plan_parses_with_defaults():
    plan = load_plan(DATA / "sysid_basic.yaml")
    assert plan.name == "sysid_basic" and len(plan.segments) == 13
    assert plan.stick_max == (0.3, 0.3, 0.3, 0.4)
    goto = plan.segments[1]
    assert goto.type == "goto" and goto.closed_loop and goto.name == "1:goto"
    assert goto.params["kp"] == 0.8 and goto.params["timeout"] == 15.0 and goto.params["yaw"] == 0.0
    assert plan.segments[4].params["yaw"] is None
    sync = plan.segments[2]
    assert sync.params["yaw_amp"] == 0.4 and sync.params["z_amp"] == 0.3 and not sync.closed_loop
    assert plan.segments[3].params["amplitude"] == [0.3]
    assert plan.segments[7].params["amplitude"] == [0.2, 0.2, 0.15, 0.3]
    assert plan.segments[9].closed_loop and plan.segments[9].params["seed"] == 1
    assert len(plan.source_sha256) == 64


def test_all_problems_reported_at_once():
    bad = {"name": "bad", "limits": {"stick_max": [0.3, 0.3, 0.3, 0.3]}, "segments": [
        {"type": "takeoff"},
        {"type": "excite", "axes": ["yaww"], "signal": "step", "amplitude": 0.2, "duration": 2.0},
        {"type": "excite", "axes": ["x", "y"], "signal": "doublet", "amplitude": [0.2], "half_period": 1.0},
        {"type": "excite", "axes": ["x"], "signal": "step", "amplitude": "0.3", "duration": 2.0},
        {"type": "excite", "axes": ["x"], "signal": "step", "amplitude": 0.5, "duration": 2.0},
        {"type": "hover", "duration": 1.0, "amplitdue": 3},
        {"type": "teleport"},
    ]}
    msgs = problems_of(bad)
    for fragment in ("segments[1].axes", "segments[2].amplitude", "segments[3].amplitude",
                     "segments[4].amplitude: 0.5 exceeds limits.stick_max[x] = 0.3",
                     "segments[5]: unknown key 'amplitdue'", "segments[6].type: unknown 'teleport'",
                     "last segment must be 'land'"):
        assert fragment in msgs


def test_first_segment_must_be_takeoff():
    data = {"name": "t", "limits": {"stick_max": [0.4] * 4},
            "segments": [{"type": "hover", "duration": 1.0}, {"type": "land"}]}
    assert "first segment must be 'takeoff'" in problems_of(data)


def test_limits_are_required():
    data = minimal()
    del data["limits"]
    assert "limits.stick_max" in problems_of(data)


def test_goto_target_shape():
    assert "segments[1].target" in problems_of(minimal({"type": "goto", "target": [0, 0]}))


def test_bool_is_not_a_number():
    assert "segments[1].duration" in problems_of(minimal({"type": "hover", "duration": True}))


def test_sync_amplitude_checked_against_limits():
    msgs = problems_of(minimal({"type": "sync", "yaw_amp": 0.5}, stick_max=(0.4, 0.4, 0.4, 0.4)))
    assert "segments[1].yaw_amp: 0.5 exceeds limits.stick_max[yaw] = 0.4" in msgs


def test_multisine_band_too_narrow():
    seg = {"type": "excite", "axes": ["x", "y", "z", "yaw"], "signal": "multisine", "amplitude": 0.1,
           "f_min": 1.0, "f_max": 1.1, "duration": 5.0}
    assert "no multisine harmonic" in problems_of(minimal(seg))


def test_f_min_below_f_max():
    seg = {"type": "excite", "axes": ["x"], "signal": "chirp", "amplitude": 0.1,
           "f_min": 2.0, "f_max": 1.0, "duration": 10.0}
    assert "segments[1].f_min: must be lower than f_max" in problems_of(minimal(seg))


def test_bad_yaml(tmp_path):
    p = tmp_path / "broken.yaml"
    p.write_text("name: [unclosed\n")
    with pytest.raises(PlanError) as e:
        load_plan(p)
    assert "YAML syntax error" in e.value.problems[0]


def test_not_a_mapping():
    with pytest.raises(PlanError):
        parse_plan(["takeoff", "land"])


def test_keys_unused_by_the_signal_are_rejected():
    seg = {"type": "excite", "axes": ["x"], "signal": "step", "amplitude": 0.2, "duration": 2.0, "hold": "x"}
    assert "segments[1]: key 'hold' is not used by signal 'step'" in problems_of(minimal(seg))


def test_multisine_seed_must_be_an_integer_even_with_a_bad_band():
    seg = {"type": "excite", "axes": ["x"], "signal": "multisine", "amplitude": 0.1,
           "f_min": 2.0, "f_max": 1.0, "duration": 10.0, "seed": "3"}
    msgs = problems_of(minimal(seg))
    assert "segments[1].seed: integer required" in msgs and "segments[1].f_min: must be lower than f_max" in msgs


def test_huge_integer_is_a_plan_error_not_a_crash():
    assert "segments[1].duration" in problems_of(minimal({"type": "hover", "duration": 10 ** 400}))


def test_takeoff_only_first_and_land_only_last():
    data = {"name": "t", "limits": {"stick_max": [0.4] * 4}, "segments": [
        {"type": "takeoff"}, {"type": "hover", "duration": 1}, {"type": "land"},
        {"type": "takeoff"}, {"type": "hover", "duration": 1}, {"type": "land"}]}
    msg = problems_of(data)
    assert "segments[3]: 'takeoff' is only allowed as the first segment" in msg
    assert "segments[2]: 'land' is only allowed as the last segment" in msg


def test_unknown_limits_key_rejected():
    data = minimal()
    data["limits"]["speed"] = 1
    assert "limits: unknown key 'speed'" in problems_of(data)
