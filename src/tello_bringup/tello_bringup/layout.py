"""Dataset layout and flight preparation (spec section 8), shared by the launch files.

    dataset/<source>/raw/<session>/<session>_<NN>_<plan name>/{bag/, flight.yaml}
"""
from __future__ import annotations

import datetime
import re
from pathlib import Path

import yaml

from tello_core.dry_run import DryRunReport, dry_run
from tello_core.lab import LabConfig
from tello_core.plan import Plan

SOURCES = {"fake": "fake", "sim": "sim", "sdk": "real"}
SIMULATED = ("fake", "sim")


def default_session() -> str:
    return datetime.date.today().isoformat() + "_a"


def session_dir(dataset_root: str, backend: str, session: str) -> Path:
    if backend not in SOURCES:
        raise ValueError(f"unknown backend '{backend}' (expected {', '.join(SOURCES)})")
    if not re.fullmatch(r"[A-Za-z0-9_.-]+", session):
        raise ValueError(f"session name '{session}' may only use letters, digits, '_', '.' and '-'")
    return Path(dataset_root).expanduser().resolve() / SOURCES[backend] / "raw" / session


def next_flight_id(sdir: Path, session: str, plan_name: str) -> str:
    """<session>_<NN>_<plan name>, NN = 1 + the highest number already used in the session."""
    used = [int(m.group(1)) for d in (sdir.iterdir() if sdir.exists() else [])
            if (m := re.fullmatch(re.escape(session) + r"_(\d{2})_.+", d.name))]
    return f"{session}_{max(used, default=0) + 1:02d}_{plan_name}"


def record_topics(topics_file: str, lab: LabConfig, backend: str) -> list[str]:
    data = yaml.safe_load(Path(topics_file).read_text())
    topics = list(data["common"]) + [lab.mocap.raw_topic]
    if backend in SIMULATED:
        topics += list(data["simulated"])
    return list(dict.fromkeys(topics))


def check_plan(plan: Plan, lab: LabConfig) -> DryRunReport:
    """The pre-flight dry run with the lab's boxes, bring-up limits and command delay."""
    return dry_run(plan, lab.soft_box, lab.hard_box, exec_cfg=lab.executor)
