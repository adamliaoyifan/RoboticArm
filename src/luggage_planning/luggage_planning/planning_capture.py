#!/usr/bin/env python3
"""Plain candidate records kept when a route attempt stops early."""

from __future__ import division


def slot_pose(slot):
    return {
        "position": [float(v) for v in slot["position"]],
        "quaternion_xyzw": [float(v) for v in slot["quaternion_xyzw"]],
    }


def target_pairs(targets):
    """World TCP targets as ``[[xyz], [xyzw]]`` pairs."""
    pairs = []
    for target in targets or []:
        if isinstance(target, (list, tuple)) and len(target) == 2:
            pairs.append([
                [float(v) for v in target[0]],
                [float(v) for v in target[1]],
            ])
            continue
        pairs.append([
            [float(v) for v in target.position],
            [float(v) for v in target.quaternion_xyzw],
        ])
    return pairs


def capture_fields(slot, targets, reachability, seed_sources):
    return {
        "slot_pose": slot_pose(slot),
        "targets": target_pairs(targets),
        "reachability": str(reachability),
        "seed_sources": [str(source) for source in (seed_sources or [])],
    }


def normalize_targets(value):
    """Pose pairs, a JSON string, or that string split into characters."""
    if isinstance(value, str):
        import json
        try:
            value = json.loads(value)
        except ValueError:
            return []
    if (isinstance(value, list) and value
            and all(isinstance(item, str) and len(item) == 1 for item in value)):
        import json
        try:
            value = json.loads("".join(value))
        except ValueError:
            return []
    pairs = []
    for item in value or []:
        if not isinstance(item, (list, tuple)) or len(item) != 2:
            continue
        position, quaternion = item
        if (not isinstance(position, (list, tuple)) or len(position) != 3
                or not isinstance(quaternion, (list, tuple))
                or len(quaternion) != 4):
            continue
        try:
            pairs.append([
                [float(v) for v in position],
                [float(v) for v in quaternion],
            ])
        except (TypeError, ValueError):
            continue
    return pairs


def _target_list(value):
    return normalize_targets(value)


def candidates_from_rows(rows):
    """Rows that carry a slot pose. Summary rows are not candidates."""
    views = []
    for row in rows or []:
        if not isinstance(row, dict) or "slot_pose" not in row:
            continue
        if row.get("reason") == "ATTEMPT_SUMMARY":
            continue
        views.append({
            "candidate_id": row.get("candidate_id"),
            "slot_index": row.get("slot_index"),
            "slot_pose": row.get("slot_pose"),
            "targets": _target_list(row.get("targets")),
            "reachability": row.get("reachability") or "",
            "seed_sources": list(row.get("seed_sources") or []),
            "reason": row.get("reason") or "",
        })
    return views


def candidate_complete(cand):
    if not isinstance(cand, dict):
        return False
    if not cand.get("candidate_id"):
        return False
    pose = cand.get("slot_pose") or {}
    position = pose.get("position")
    quaternion = pose.get("quaternion_xyzw")
    if (not isinstance(position, list) or len(position) != 3
            or not isinstance(quaternion, list) or len(quaternion) != 4):
        return False
    if not normalize_targets(cand.get("targets")):
        return False
    if not isinstance(cand.get("seed_sources"), list):
        return False
    if not cand.get("reachability"):
        return False
    return True
