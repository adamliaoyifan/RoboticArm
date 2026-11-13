#!/usr/bin/env python3
"""Serialize motion-boundary records for T2 replay (no ROS imports)."""

from __future__ import division

import json
import os
import threading

_LOCK = threading.Lock()
_SEQ = 0


def robot_traj_to_dict(robot_traj, max_points=250):
    """Compact joint-trajectory dict. ``robot_traj`` may be None."""
    if robot_traj is None:
        return None
    joint_traj = getattr(robot_traj, "joint_trajectory", None)
    if joint_traj is None and hasattr(robot_traj, "points"):
        joint_traj = robot_traj
    if joint_traj is None:
        return None
    raw = list(getattr(joint_traj, "points", None) or [])
    if not raw:
        return {
            "joint_names": list(getattr(joint_traj, "joint_names", None) or []),
            "n_points": 0,
            "points": [],
        }
    if len(raw) <= max_points:
        chosen = raw
    else:
        step = max(1, len(raw) // max_points)
        chosen = raw[::step]
        if chosen[-1] is not raw[-1]:
            chosen.append(raw[-1])
    points = []
    for point in chosen:
        stamp = getattr(point, "time_from_start", None)
        sec = float(getattr(stamp, "sec", 0) or 0)
        nsec = float(getattr(stamp, "nanosec", 0) or 0)
        points.append({
            "t": round(sec + 1e-9 * nsec, 4),
            "q": [round(float(v), 5) for v in (point.positions or [])],
        })
    return {
        "joint_names": list(getattr(joint_traj, "joint_names", None) or []),
        "n_points": len(raw),
        "points": points,
    }


def write_boundary_dump(dump_root, record):
    """Atomically write ``NNNN_name.json`` plus ``latest.json``. Return path."""
    if not dump_root:
        return None
    os.makedirs(dump_root, exist_ok=True)
    global _SEQ
    with _LOCK:
        _SEQ += 1
        seq = _SEQ
    slug = str(record.get("name") or "motion").replace(os.sep, "_")
    path = os.path.join(dump_root, "%04d_%s.json" % (seq, slug))
    payload = json.dumps(record, indent=2, sort_keys=True, default=str) + "\n"
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as handle:
        handle.write(payload)
    os.replace(tmp, path)
    latest = os.path.join(dump_root, "latest.json")
    tmp_latest = latest + ".tmp"
    with open(tmp_latest, "w", encoding="utf-8") as handle:
        handle.write(payload)
    os.replace(tmp_latest, latest)
    return path


def load_latest_boundary(dump_root):
    if not dump_root:
        return None
    path = os.path.join(dump_root, "latest.json")
    if not os.path.isfile(path):
        return None
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def replay_manifest(case_id, fail_code, stage, artifacts, missing):
    missing = [str(item) for item in (missing or [])]
    return {
        "case_id": case_id,
        "fail_code": fail_code,
        "stage": stage,
        "artifacts": artifacts,
        "missing": missing,
        "capture_complete": not missing,
        "replay_possible": not missing,
    }
