"""Pure parts of the safety console (spec 7.4): key bindings and the status line."""
from __future__ import annotations

import math

KEYS = {"h": "hold_abort", "r": "release", " ": "land", "e": "emergency", "t": "takeoff"}
STATES = ("DISCONNECTED", "LANDED", "TAKING_OFF", "FLYING", "LANDING", "EMERGENCY")
HELP = "keys: H hold+abort | R release | SPACE land | E EMERGENCY | T takeoff | Ctrl-C quit"


def key_action(ch: str) -> str | None:
    return KEYS.get(ch.lower()) if ch else None


def status_line(status, segment: str, events: list[str]) -> str:
    """One line for the terminal. `status` is a TelloStatus-like object or None."""
    if status is None:
        return "waiting for /tello/status ..."
    state = STATES[status.state] if 0 <= status.state < len(STATES) else str(status.state)
    bat = f"{status.battery:5.1f}%" if math.isfinite(status.battery) else "  n/a "
    age = f"{status.mocap_age * 1000:4.0f} ms" if math.isfinite(status.mocap_age) else "  none "
    flags = ("HOLD " if status.hold else "") + ("REC " if status.recording else "") + (
        "" if status.link_ok else "NO-LINK ")
    return (f"{state:<12} bat {bat} vicon {age} {status.mocap_rate:5.1f} Hz  src {status.cmd_source:<7}"
            f"{flags}seg {segment or '-'} | {', '.join(events[-3:]) or '-'}")
