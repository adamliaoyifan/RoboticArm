#!/usr/bin/env python3
"""Atlas reachability prior shared by the selector and offline replay.

Status codes match the selector: 3 reachable, 2 marginal, 1 unreachable,
0 unknown. A candidate is unreachable only when every TCP target is
sampled-unreachable. Atlas-off does not query and does not invent that
verdict.
"""

from __future__ import division

import math


def prior_from_statuses(statuses, seeds):
    """Collapse per-target atlas statuses into one candidate prior."""
    codes = [int(status) for status in (statuses or [])]
    if not codes:
        return {"verdict": "unknown", "seeds": []}
    if all(code == 1 for code in codes):
        return {"verdict": "unreachable", "seeds": []}
    if any(code in (2, 3) for code in codes):
        return {"verdict": "reachable", "seeds": list(seeds or [])}
    return {"verdict": "unknown", "seeds": []}


def _container_pose(transform):
    if isinstance(transform, dict):
        translation = transform["translation"]
        quaternion = transform["quaternion_xyzw"]
    else:
        translation = transform[2]
        quaternion = transform[3]
    return (
        tuple(float(v) for v in translation),
        tuple(float(v) for v in quaternion),
    )


def _coerce_targets(targets):
    """Accept a list of pairs or the legacy truncated JSON string."""
    if isinstance(targets, str):
        import json
        parsed = json.loads(targets)
        return parsed if isinstance(parsed, list) else []
    return targets or []


def _target_pose(target):
    if isinstance(target, (list, tuple)) and len(target) == 2:
        return (
            tuple(float(v) for v in target[0]),
            tuple(float(v) for v in target[1]),
        )
    return (
        tuple(float(v) for v in target.position),
        tuple(float(v) for v in target.quaternion_xyzw),
    )


def _yaw(quaternion):
    x, y, z, w = quaternion
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z))


def prior_for_targets(targets, container_transform, query_fn):
    """Query *query_fn(x, y, z, yaw)* in the container frame.

    *query_fn* may return an int status or an object with ``status`` and
    optional ``contact_seeds``. This is the selector's atlas-on rule.
    """
    from st09_contracts.attachment import compose_transform, invert_transform

    translation, quaternion = _container_pose(container_transform)
    inv_xyz, inv_quat = invert_transform(translation, quaternion)
    statuses = []
    seeds = []
    for target in targets or []:
        position, quat = _target_pose(target)
        local_pos, local_quat = compose_transform(
            inv_xyz, inv_quat, position, quat)
        yaw = _yaw(local_quat)
        result = query_fn(
            float(local_pos[0]), float(local_pos[1]),
            float(local_pos[2]), yaw)
        if isinstance(result, int):
            statuses.append(int(result))
            continue
        statuses.append(int(result.status))
        if int(result.status) in (2, 3):
            for seed in (getattr(result, "contact_seeds", None) or ()):
                seeds.append(tuple(float(v) for v in seed))
    return prior_from_statuses(statuses, seeds)


def replay_captured_candidates(document, query_fn):
    """Atlas on uses *query_fn*. Atlas off does not query.

    Each row names the captured reachability beside both verdicts so a
    later failure can be placed at synthesis, atlas, or a later stage.
    """
    document = document if isinstance(document, dict) else {}
    transform = document.get("container_transform")
    rows = []
    for cand in document.get("candidates") or []:
        if not isinstance(cand, dict):
            continue
        targets = _coerce_targets(cand.get("targets") or [])
        on = prior_for_targets(targets, transform, query_fn)
        rows.append({
            "candidate_id": cand.get("candidate_id"),
            "slot_index": cand.get("slot_index"),
            "captured_reachability": cand.get("reachability"),
            "captured_reason": cand.get("reason") or "",
            "seed_sources": list(cand.get("seed_sources") or []),
            "n_targets": len(targets),
            "atlas_on": on["verdict"],
            "atlas_off": "no_atlas",
        })
    return rows
