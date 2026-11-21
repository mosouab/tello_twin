"""Lab configuration (`lab.yaml`): flight volumes, limits, safety, pre-flight and Vicon settings.

Every node and the dry run load the same file with `load_lab(path)`, so the boxes and limits the drone flies
with are exactly the ones the dry run checked. All problems are reported at once (LabError.problems).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import yaml

from .control import ControlConfig
from .executor import ExecutorConfig
from .fake import FakeParams
from .flight_state import PreflightConfig
from .safety import Box, SafetyConfig

TAKEOFF_HEIGHT = FakeParams().takeoff_height      # 0.8 m: the Tello's automatic takeoff height
MOCAP_TYPES = ("PoseStamped", "TransformStamped", "Odometry")
MOCAP_SOURCES = ("lab_node", "vrpn")
SECTIONS = ("lab", "volume", "limits", "safety", "preflight", "mocap", "setup")
SAFETY_KEYS = ("cmd_timeout_hover", "cmd_timeout_land", "mocap_timeout_hover", "mocap_timeout_land",
               "telemetry_timeout", "battery_land")
PREFLIGHT_KEYS = ("battery_takeoff_min", "mocap_max_age", "mocap_min_rate_ratio", "require_recording")


class LabError(ValueError):
    def __init__(self, problems: list[str]):
        self.problems = list(problems)
        super().__init__("invalid lab config:\n  - " + "\n  - ".join(self.problems))


@dataclass(frozen=True)
class MocapSettings:
    source: str = "lab_node"
    raw_topic: str = "/vicon/tello/pose"
    raw_type: str = "PoseStamped"
    rigid_body: str = "tello"
    expected_rate: float = 100.0
    heading_offset: float = 0.0
    thr_pos: float = 0.3
    thr_yaw: float = 0.5
    z_floor: float = -0.2
    marker_layout: str = ""


@dataclass(frozen=True)
class LabConfig:
    name: str
    soft_box: Box
    hard_box: Box
    safety: SafetyConfig
    preflight: PreflightConfig
    executor: ExecutorConfig
    mocap: MocapSettings
    send_rate: float
    setup: dict
    source_path: str = ""

    @property
    def control(self) -> ControlConfig:
        return self.executor.control


def _num(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _section(data: dict, key: str, problems: list[str]) -> dict:
    v = data.get(key, {})
    if v is None:
        return {}
    if not isinstance(v, dict):
        problems.append(f"{key}: mapping required")
        return {}
    return v


def _box(sec: dict, key: str, problems: list[str]) -> Box | None:
    v = sec.get(key)
    if not (isinstance(v, list) and len(v) == 6 and all(_num(x) for x in v)):
        problems.append(f"volume.{key}: list of 6 finite numbers required, got {v!r}")
        return None
    try:
        return Box.from_list(v)
    except ValueError as e:
        problems.append(f"volume.{key}: {e}")
        return None


def _inside(inner: Box, outer: Box) -> bool:
    return (outer.xmin <= inner.xmin and inner.xmax <= outer.xmax and outer.ymin <= inner.ymin
            and inner.ymax <= outer.ymax and outer.zmin <= inner.zmin and inner.zmax <= outer.zmax)


def parse_lab(data, source_path: str = "") -> LabConfig:
    if not isinstance(data, dict):
        raise LabError(["lab config must be a YAML mapping"])
    problems: list[str] = []
    for k in data:
        if k not in SECTIONS:
            problems.append(f"unknown top-level key '{k}' (expected {', '.join(SECTIONS)})")
    lab = _section(data, "lab", problems)
    vol = _section(data, "volume", problems)
    lim = _section(data, "limits", problems)
    saf = _section(data, "safety", problems)
    pre = _section(data, "preflight", problems)
    moc = _section(data, "mocap", problems)
    setup = _section(data, "setup", problems)

    soft, hard = _box(vol, "soft_box", problems), _box(vol, "hard_box", problems)
    if soft and hard:
        if not _inside(soft, hard):
            problems.append("volume: soft_box must lie inside hard_box")
        if not soft.zmin < TAKEOFF_HEIGHT < soft.zmax:
            problems.append(f"volume.soft_box: zmin must be below and zmax above the {TAKEOFF_HEIGHT} m takeoff "
                            "height")

    stick_max = lim.get("stick_max", [0.4, 0.4, 0.4, 0.4])
    if not (isinstance(stick_max, list) and len(stick_max) == 4 and all(_num(v) for v in stick_max)):
        problems.append(f"limits.stick_max: list of 4 numbers required, got {stick_max!r}")
        stick_max = [0.4] * 4
    cmd_delay, stop_time = lim.get("cmd_delay", 0.7), lim.get("stop_time", 0.5)
    send_rate = lim.get("send_rate", 50.0)
    for key, v in (("cmd_delay", cmd_delay), ("stop_time", stop_time), ("send_rate", send_rate)):
        if not _num(v):
            problems.append(f"limits.{key}: number required, got {v!r}")
    if _num(send_rate) and send_rate <= 0:
        problems.append(f"limits.send_rate: must be > 0, got {send_rate!r}")

    for sec, keys, name in ((saf, SAFETY_KEYS, "safety"), (pre, PREFLIGHT_KEYS, "preflight")):
        for k, v in sec.items():
            if k not in keys:
                problems.append(f"{name}: unknown key '{k}'")
            elif k != "require_recording" and not _num(v):
                problems.append(f"{name}.{k}: number required, got {v!r}")
    if not isinstance(pre.get("require_recording", True), bool):
        problems.append("preflight.require_recording: true or false required")

    mocap = MocapSettings()
    known = MocapSettings.__dataclass_fields__
    for k in moc:
        if k not in known:
            problems.append(f"mocap: unknown key '{k}'")
    m = {k: moc.get(k, getattr(mocap, k)) for k in known}
    if m["source"] not in MOCAP_SOURCES:
        problems.append(f"mocap.source: one of {', '.join(MOCAP_SOURCES)} required, got {m['source']!r}")
    if m["raw_type"] not in MOCAP_TYPES:
        problems.append(f"mocap.raw_type: one of {', '.join(MOCAP_TYPES)} required, got {m['raw_type']!r}")
    if not (isinstance(m["raw_topic"], str) and m["raw_topic"].startswith("/")):
        problems.append(f"mocap.raw_topic: absolute topic name required, got {m['raw_topic']!r}")
    for k in ("expected_rate", "thr_pos", "thr_yaw"):
        if not (_num(m[k]) and m[k] > 0):
            problems.append(f"mocap.{k}: positive number required, got {m[k]!r}")
    for k in ("heading_offset", "z_floor"):
        if not _num(m[k]):
            problems.append(f"mocap.{k}: number required, got {m[k]!r}")

    if problems:
        raise LabError(problems)
    try:
        safety = SafetyConfig(hard_box=hard, cmd_delay=float(cmd_delay), stop_time=float(stop_time),
                              **{k: float(v) for k, v in saf.items()})
        preflight = PreflightConfig(soft_box=soft, **{k: (v if k == "require_recording" else float(v))
                                                      for k, v in pre.items()})
        control = ControlConfig(cmd_delay=float(cmd_delay), stick_max=tuple(float(v) for v in stick_max))
        executor = ExecutorConfig(control=control, stop_time=float(stop_time))
        mocap = MocapSettings(**{k: (float(v) if k in ("expected_rate", "heading_offset", "thr_pos", "thr_yaw",
                                                         "z_floor") else str(v)) for k, v in m.items()})
    except ValueError as e:
        raise LabError([str(e)]) from e
    return LabConfig(name=str(lab.get("name", "unnamed lab")), soft_box=soft, hard_box=hard, safety=safety,
                     preflight=preflight, executor=executor, mocap=mocap, send_rate=float(send_rate),
                     setup=dict(setup), source_path=source_path)


def load_lab(path) -> LabConfig:
    p = Path(path)
    try:
        data = yaml.safe_load(p.read_text())
    except OSError as e:
        raise LabError([f"cannot read {p}: {e}"]) from e
    except yaml.YAMLError as e:
        raise LabError([f"YAML syntax error: {e}"]) from e
    return parse_lab(data, str(p))
