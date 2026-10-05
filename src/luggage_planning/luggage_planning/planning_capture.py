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


def _target_list(value):
    if isinstance(value, str):
        import json
        parsed = json.loads(value)
        return parsed if isinstance(parsed, list) else []
    return list(value or [])


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
    targets = cand.get("targets")
    if not isinstance(targets, list) or not targets:
        return False
    if not isinstance(cand.get("seed_sources"), list):
        return False
    if not cand.get("reachability"):
        return False
    return True
