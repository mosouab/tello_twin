import math
import os
from types import SimpleNamespace

import pytest

from tello_flight.console_format import HELP, key_action, status_line


@pytest.mark.parametrize("ch, action", [("h", "hold_abort"), ("H", "hold_abort"), ("r", "release"),
                                        (" ", "land"), ("e", "emergency"), ("T", "takeoff"), ("x", None),
                                        ("", None)])
def test_key_bindings(ch, action):
    assert key_action(ch) == action


def read_twice(data: bytes) -> list[str]:
    """Write data into a pipe, then call read_keys twice: what each call delivers."""
    from tello_flight.safety_console import read_keys

    r, w = os.pipe()
    try:
        os.write(w, data)
        return [read_keys(r, 0.0), read_keys(r, 0.0)]
    finally:
        os.close(r)
        os.close(w)


def test_read_keys_returns_every_pending_key():
    assert read_twice(b"h ") == ["h ", ""]


@pytest.mark.parametrize("seq", [b"\x1bOE", b"\x1b[E", b"\x1bOR", b"\x1bt", b"\x1b[H", b"\x1b"],
                         ids=["keypad5_ss3", "keypad5_csi", "F3", "alt_t", "home", "esc"])
def test_read_keys_drops_escape_sequences(seq):
    assert read_twice(seq) == ["", ""]


def test_read_keys_drops_a_pasted_help_line():
    assert read_twice(HELP.encode()) == ["", ""]


def test_read_keys_drops_a_long_paste_whole():
    # longer than one read: the tail ("e ", which alone would be E + Space) must not leak into the next call
    assert read_twice(b"x" * 1024 + b"e ") == ["", ""]


def st(**kw):
    d = dict(state=3, battery=72.0, mocap_age=0.012, mocap_rate=99.0, hold=False, recording=True, link_ok=True,
             cmd_source="runner")
    d.update(kw)
    return SimpleNamespace(**d)


def test_status_line_contents():
    line = status_line(st(), "3:excite", ["watchdog_hover", "hold_on", "hold_off", "segment_truncated"])
    assert line.startswith("FLYING") and "72.0%" in line and "12 ms" in line and "REC" in line
    assert "seg 3:excite" in line and "watchdog_hover" not in line and "segment_truncated" in line


def test_status_line_unknowns():
    line = status_line(st(battery=math.nan, mocap_age=math.inf, hold=True, link_ok=False), "", [])
    assert "n/a" in line and "none" in line and "HOLD" in line and "NO-LINK" in line and "seg -" in line
    assert status_line(None, "", []).startswith("waiting")
