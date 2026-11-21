"""Pure parts of the safety console (spec 7.4): key bindings and the status line."""
from __future__ import annotations

import math

KEYS = {"h": "hold_abort", "r": "release", " ": "land", "e": "emergency", "t": "takeoff"}
STATES = ("DISCONNECTED", "LANDED", "TAKING_OFF", "FLYING", "LANDING", "EMERGENCY")
HELP = "keys: H hold+abort | R release | SPACE land | E EMERGENCY | T takeoff | Ctrl-C quit"
MAX_KEYS = 4            # bytes; a longer read is pasted text, not key presses


def key_action(ch: str) -> str | None:
    return KEYS.get(ch.lower()) if ch else None


def accept_keys(chunk: bytes) -> str:
    """The keys in one read from the terminal, or '' if the whole read must be ignored (spec 15.20): any read
    containing ESC (escape sequences: keypad, function keys, Alt+key, Home, arrows) or longer than MAX_KEYS bytes
    (a paste). Their bytes would otherwise map to keys, e.g. keypad 5 (ESC O E) to emergency."""
    if b"\x1b" in chunk or len(chunk) > MAX_KEYS:
        return ""
    return chunk.decode(errors="ignore")


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
