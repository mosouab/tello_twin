"""flight.yaml / session.yaml writing (spec section 8). Atomic writes, so a crash never leaves half a file."""
from __future__ import annotations

import datetime
import os
import subprocess
from pathlib import Path

import yaml


def now_iso() -> str:
    return datetime.datetime.now().astimezone().isoformat(timespec="seconds")


def software_version(repo_dir: str) -> str:
    """`git describe --always --dirty` in repo_dir, or 'unversioned'."""
    try:
        r = subprocess.run(["git", "describe", "--always", "--dirty"], cwd=repo_dir, capture_output=True,
                           text=True, timeout=5)
    except (OSError, subprocess.SubprocessError):
        return "unversioned"
    return r.stdout.strip() if r.returncode == 0 and r.stdout.strip() else "unversioned"


def write_yaml(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(yaml.safe_dump(data, sort_keys=False))
    os.replace(tmp, path)


def session_record(*, session: str, backend: str, drone_info: dict, lab_file: str, lab_name: str) -> dict:
    return {"session": session, "created": now_iso(), "backend": backend,
            "drone": {"serial": drone_info.get("serial", "unknown"), "sdk": drone_info.get("sdk", "unknown")},
            "lab_file": lab_file, "lab_name": lab_name,
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
            "rmw": os.environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp (default)")}
