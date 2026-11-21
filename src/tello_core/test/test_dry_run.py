from pathlib import Path

from tello_core.dry_run import dry_run, main
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
