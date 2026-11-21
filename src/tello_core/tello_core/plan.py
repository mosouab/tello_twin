"""Flight plans (spec section 7.1): YAML loading and validation.

Every problem is reported at once (PlanError.problems), with the path of the faulty field, so a typo in a
plan never surfaces as a KeyError in the middle of a flight. Validated segments carry their defaults.
"""
from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass
from pathlib import Path

import yaml

from .signals import harmonics
from .sticks import AXES

SEGMENT_TYPES = ("takeoff", "goto", "sync", "excite", "arena", "hover", "land")
CLOSED_LOOP = frozenset({"goto", "arena"})
SIGNALS = ("step", "doublet", "staircase", "chirp", "multisine")
SYNC_DEFAULTS = {"hover_pre": 2.0, "yaw_amp": 0.4, "yaw_time": 0.5, "hover_mid": 1.0,
                 "z_amp": 0.3, "z_time": 0.4, "hover_post": 2.0}
TIMEOUT_DEFAULTS = {"takeoff": 20.0, "land": 20.0, "goto": 15.0}
ALLOWED_KEYS = {
    "takeoff": {"timeout"},
    "land": {"timeout"},
    "hover": {"duration"},
    "goto": {"target", "yaw", "kp", "v_max", "tol", "timeout"},
    "sync": set(SYNC_DEFAULTS),
    "arena": {"duration", "seed", "v_max", "u_exc"},
    "excite": {"axes", "signal", "amplitude", "amplitudes", "duration", "hold", "half_period",
               "f_min", "f_max", "seed"},
}


class PlanError(ValueError):
    def __init__(self, problems: list[str]):
        self.problems = list(problems)
        super().__init__("invalid flight plan:\n  - " + "\n  - ".join(self.problems))


@dataclass(frozen=True)
class Segment:
    index: int
    type: str
    params: dict

    @property
    def closed_loop(self) -> bool:
        return self.type in CLOSED_LOOP

    @property
    def name(self) -> str:
        return f"{self.index}:{self.type}"


@dataclass(frozen=True)
class Plan:
    name: str
    stick_max: tuple[float, float, float, float]
    volume_shrink: float
    segments: tuple[Segment, ...]
    source_sha256: str = ""


def _is_num(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _is_int(v) -> bool:
    return isinstance(v, int) and not isinstance(v, bool)


class _Checker:
    """Validates one segment's parameters and fills defaults into `out`."""

    def __init__(self, params: dict, where: str, lim: list[float], problems: list[str]):
        self.p, self.where, self.lim, self.problems = params, where, lim, problems
        self.out = dict(params)

    def bad(self, msg: str) -> None:
        self.problems.append(f"{self.where}{msg}")

    def num(self, key: str, default: float | None = None, positive: bool = False,
            minimum: float | None = None) -> float | None:
        if key not in self.p:
            if default is None:
                self.bad(f".{key}: required")
                return None
            self.out[key] = float(default)
            return float(default)
        v = self.p[key]
        if not _is_num(v) or (positive and v <= 0) or (minimum is not None and v < minimum):
            kind = "positive number" if positive else (f"number >= {minimum:g}" if minimum is not None else "number")
            self.bad(f".{key}: {kind} required, got {v!r}")
            return None
        self.out[key] = float(v)
        return float(v)

    def amp_ok(self, a: float, axis: str, key: str) -> None:
        m = self.lim[AXES.index(axis)]
        if abs(a) > m + 1e-12:
            self.bad(f".{key}: {a} exceeds limits.stick_max[{axis}] = {m}")

    def seed(self, default: int | None = None) -> None:
        if "seed" in self.p:
            if not _is_int(self.p["seed"]):
                self.bad(f".seed: integer required, got {self.p['seed']!r}")
        elif default is not None:
            self.out["seed"] = default


def _check_excite(c: _Checker) -> None:
    p = c.p
    axes = p.get("axes")
    if not (isinstance(axes, list) and axes and all(a in AXES for a in axes) and len(set(axes)) == len(axes)):
        c.bad(f".axes: non-empty list of distinct values from {', '.join(AXES)} required, got {axes!r}")
        return
    sig = p.get("signal")
    if sig not in SIGNALS:
        c.bad(f".signal: one of {', '.join(SIGNALS)} required, got {sig!r}")
        return
    if sig == "staircase":
        c.num("hold", positive=True)
        amps = p.get("amplitudes")
        if not (isinstance(amps, list) and amps and all(_is_num(a) and a > 0 for a in amps)):
            c.bad(f".amplitudes: non-empty list of positive numbers required, got {amps!r}")
            return
        c.out["amplitudes"] = [float(a) for a in amps]
        for a in amps:
            for ax in axes:
                c.amp_ok(a, ax, "amplitudes")
        return
    amp = p.get("amplitude")
    if _is_num(amp):
        amps = [float(amp)] * len(axes)
    elif isinstance(amp, list) and len(amp) == len(axes) and all(_is_num(a) for a in amp):
        amps = [float(a) for a in amp]
    else:
        c.bad(f".amplitude: number or list with one number per axis required, got {amp!r}")
        amps = None
    if amps is not None:
        c.out["amplitude"] = amps
        for a, ax in zip(amps, axes):
            c.amp_ok(a, ax, "amplitude")
    if sig == "step":
        c.num("duration", positive=True)
    elif sig == "doublet":
        c.num("half_period", positive=True)
    else:
        d = c.num("duration", positive=True)
        f0 = c.num("f_min", positive=True)
        f1 = c.num("f_max", positive=True)
        if f0 is not None and f1 is not None and f0 >= f1:
            c.bad(f".f_min: must be lower than f_max ({f0} >= {f1})")
        elif sig == "multisine" and None not in (d, f0, f1):
            c.seed()
            for j, ax in enumerate(axes):
                if not harmonics(f0, f1, d, j, len(axes)):
                    c.bad(f": no multisine harmonic for axis {ax} in [{f0}, {f1}] Hz over {d} s "
                          "(widen the band or lengthen the duration)")


def _check_segment(t: str, params: dict, where: str, lim: list[float], problems: list[str]) -> dict:
    c = _Checker(params, where, lim, problems)
    for k in params:
        if k not in ALLOWED_KEYS[t]:
            c.bad(f": unknown key '{k}'")
    if t in TIMEOUT_DEFAULTS:
        c.num("timeout", TIMEOUT_DEFAULTS[t], positive=True)
    if t == "hover":
        c.num("duration", positive=True)
    elif t == "goto":
        tgt = params.get("target")
        if isinstance(tgt, list) and len(tgt) == 3 and all(_is_num(v) for v in tgt):
            c.out["target"] = [float(v) for v in tgt]
        else:
            c.bad(f".target: list of 3 numbers required, got {tgt!r}")
        yaw = params.get("yaw")
        if yaw is not None and not _is_num(yaw):
            c.bad(f".yaw: number or null required, got {yaw!r}")
        c.out["yaw"] = float(yaw) if _is_num(yaw) else None
        for k, d in (("kp", 0.8), ("v_max", 0.4), ("tol", 0.1)):
            c.num(k, d, positive=True)
    elif t == "sync":
        for k, d in SYNC_DEFAULTS.items():
            c.num(k, d, positive=True)
        for key, axis in (("yaw_amp", "yaw"), ("z_amp", "z")):
            if _is_num(c.out.get(key)):
                c.amp_ok(c.out[key], axis, key)
    elif t == "arena":
        c.num("duration", positive=True)
        c.num("v_max", 0.3, positive=True)
        c.num("u_exc", 0.05, minimum=0.0)
        c.seed(default=0)
    elif t == "excite":
        _check_excite(c)
    return c.out


def parse_plan(data, sha256: str = "") -> Plan:
    if not isinstance(data, dict):
        raise PlanError(["plan must be a YAML mapping"])
    problems: list[str] = []
    for k in data:
        if k not in ("name", "limits", "volume_shrink", "segments"):
            problems.append(f"unknown top-level key '{k}'")
    name = data.get("name")
    if not (isinstance(name, str) and name):
        problems.append("name: non-empty string required")
    limits = data.get("limits")
    lim = limits.get("stick_max") if isinstance(limits, dict) else None
    if not (isinstance(lim, list) and len(lim) == 4 and all(_is_num(v) and 0 < v <= 1 for v in lim)):
        problems.append(f"limits.stick_max: list of 4 numbers in (0, 1] required, got {lim!r}")
        lim = [1.0, 1.0, 1.0, 1.0]
    lim = [float(v) for v in lim]
    shrink = data.get("volume_shrink", 0.0)
    if not (_is_num(shrink) and shrink >= 0):
        problems.append(f"volume_shrink: number >= 0 required, got {shrink!r}")
        shrink = 0.0
    raw = data.get("segments")
    if not (isinstance(raw, list) and raw):
        problems.append("segments: non-empty list required")
        raw = []
    segments: list[Segment] = []
    for i, s in enumerate(raw):
        where = f"segments[{i}]"
        if not (isinstance(s, dict) and "type" in s):
            problems.append(f"{where}: mapping with a 'type' key required")
            continue
        t = s["type"]
        if t not in SEGMENT_TYPES:
            problems.append(f"{where}.type: unknown '{t}' (expected one of {', '.join(SEGMENT_TYPES)})")
            continue
        params = {k: v for k, v in s.items() if k != "type"}
        segments.append(Segment(i, t, _check_segment(t, params, where, lim, problems)))
    if segments and segments[0].type != "takeoff":
        problems.append("first segment must be 'takeoff'")
    if segments and segments[-1].type != "land":
        problems.append("last segment must be 'land'")
    if problems:
        raise PlanError(problems)
    return Plan(name, tuple(lim), float(shrink), tuple(segments), sha256)


def load_plan(path) -> Plan:
    raw = Path(path).read_bytes()
    try:
        data = yaml.safe_load(raw)
    except yaml.YAMLError as e:
        raise PlanError([f"YAML syntax error: {e}"]) from e
    return parse_plan(data, hashlib.sha256(raw).hexdigest())
