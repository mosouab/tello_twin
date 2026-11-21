from pathlib import Path

from tello_core.control import ControlConfig
from tello_core.dry_run import dry_run, main
from tello_core.executor import ExecutorConfig
from tello_core.plan import load_plan, parse_plan
from tello_core.safety import Box

DATA = Path(__file__).parent / "data"
SOFT = Box(-2.5, 2.5, -2.5, 2.5, 0.5, 2.5)
HARD = Box(-3.0, 3.0, -3.0, 3.0, 0.2, 2.8)
BOX_ARGS = ["--soft-box", "-2.5", "2.5", "-2.5", "2.5", "0.5", "2.5",
            "--hard-box", "-3", "3", "-3", "3", "0.2", "2.8"]


def test_example_plan_passes():
    r = dry_run(load_plan(DATA / "sysid_basic.yaml"), SOFT, HARD)
    assert r.ok, r.problems
    assert 60.0 < r.duration < 400.0 and r.max_outside == 0.0


def test_long_step_fails_with_truncation():
    plan = parse_plan({"name": "too_long", "limits": {"stick_max": [0.3, 0.3, 0.3, 0.3]}, "segments": [
        {"type": "takeoff"},
        {"type": "goto", "target": [0.0, 0.0, 1.2]},
        {"type": "excite", "axes": ["x"], "signal": "step", "amplitude": 0.3, "duration": 20.0},
        {"type": "land"},
    ]})
    r = dry_run(plan, SOFT, HARD)
    assert not r.ok
    assert any("segment_truncated" in p for p in r.problems)


def test_cli_exit_codes(tmp_path, capsys):
    assert main([str(DATA / "sysid_basic.yaml"), *BOX_ARGS]) == 0
    assert "OK: sysid_basic" in capsys.readouterr().out
    bad = tmp_path / "bad.yaml"
    bad.write_text("name: x\nsegments: []\n")
    assert main([str(bad), *BOX_ARGS]) == 2


def goto_plan(x, shrink=0.0, stick=0.3):
    return parse_plan({"name": "g", "limits": {"stick_max": [stick] * 4}, "volume_shrink": shrink, "segments": [
        {"type": "takeoff"}, {"type": "goto", "target": [x, 0.0, 1.2]}, {"type": "land"}]})


def test_dry_run_honours_volume_shrink():
    r = dry_run(goto_plan(2.2, shrink=0.6), SOFT, HARD)
    assert r.ok is False
    assert any("soft box" in p or "hard box" in p for p in r.problems), r.problems


def test_dry_run_fails_when_plan_exceeds_bring_up_limits():
    plan = load_plan(DATA / "sysid_basic.yaml")
    r = dry_run(plan, SOFT, HARD, exec_cfg=ExecutorConfig(control=ControlConfig(stick_max=(0.2,) * 4)))
    assert r.ok is False
    assert any("exceeds the bring-up limit" in p for p in r.problems), r.problems


def test_cli_stick_max_below_plan_returns_1(capsys):
    assert main([str(DATA / "sysid_basic.yaml"), *BOX_ARGS, "--stick-max", "0.2", "0.2", "0.2", "0.2"]) == 1


def test_arena_in_too_small_box_fails_cleanly(tmp_path, capsys):
    plan = parse_plan({"name": "a", "limits": {"stick_max": [0.3] * 4}, "segments": [
        {"type": "takeoff"}, {"type": "arena", "duration": 10.0}, {"type": "land"}]})
    small = Box(-2.5, 2.5, -2.5, 2.5, 0.6, 1.1)
    r = dry_run(plan, small, HARD)
    assert r.ok is False and r.problems
    p = tmp_path / "a.yaml"
    p.write_text("name: a\nlimits: {stick_max: [0.3, 0.3, 0.3, 0.3]}\nsegments:\n"
                 "  - {type: takeoff}\n  - {type: arena, duration: 10}\n  - {type: land}\n")
    args = ["--soft-box", "-2.5", "2.5", "-2.5", "2.5", "0.6", "1.1", "--hard-box", "-3", "3", "-3", "3", "0.2", "2.8"]
    assert main([str(p), *args]) == 1
    assert "FAILED" in capsys.readouterr().out


def test_goto_outside_hard_box_but_inside_wide_soft_box_fails():
    wide = Box(-5, 5, -5, 5, 0.5, 2.5)
    r = dry_run(goto_plan(3.3), wide, HARD)
    assert r.ok is False
    assert any("left the hard box" in p for p in r.problems), r.problems
