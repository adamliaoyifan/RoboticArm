#!/usr/bin/env python3
"""Atlas reachability prior shared by the selector and offline replay.

Status codes match the selector: 3 reachable, 2 marginal, 1 unreachable,
0 unknown. A candidate is unreachable only when every TCP target is
``hard_reject_safe``. A sampled-unreachable cell that is not a
high-confidence interior reject stays unknown. Marginal does not
collapse into reachable. Atlas-off does not query and does not invent
a verdict.
"""

from __future__ import division

import math


def prior_from_statuses(statuses, seeds, hard_reject_flags=None):
    """Collapse per-target atlas statuses into one candidate prior.

    ``unreachable`` requires every target to be a high-confidence
    interior reject. Any status-3 target is reachable. Status 2 with
    no status 3 is marginal. Everything else, including a low-confidence
    unreachable sample, is unknown.
    """
    codes = [int(status) for status in (statuses or [])]
    flags = [bool(flag) for flag in (hard_reject_flags or [])]
    if not codes:
        return {"verdict": "unknown", "seeds": []}
    if (len(flags) == len(codes) and all(flags)
            and all(code == 1 for code in codes)):
        return {"verdict": "unreachable", "seeds": []}
    kept = list(seeds or [])
    if any(code == 3 for code in codes):
        return {"verdict": "reachable", "seeds": kept}
    if any(code == 2 for code in codes):
        return {"verdict": "marginal", "seeds": kept}
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
    """Accept pose pairs, a JSON string, or that string split into characters."""
    from luggage_planning.planning_capture import normalize_targets
    return normalize_targets(targets)


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

    *query_fn* may return an int status or an object with ``status``,
    optional ``hard_reject_safe``, ``indices`` and ``contact_seeds``.
    This is the selector's atlas-on rule. The queried point is the TCP
    target transformed into the container frame; this function does not
    add ``query_offset``.
    """
    from st09_contracts.attachment import compose_transform, invert_transform

    translation, quaternion = _container_pose(container_transform)
    inv_xyz, inv_quat = invert_transform(translation, quaternion)
    statuses = []
    flags = []
    seeds = []
    queries = []
    for target in targets or []:
        position, quat = _target_pose(target)
        local_pos, local_quat = compose_transform(
            inv_xyz, inv_quat, position, quat)
        yaw = _yaw(local_quat)
        point = (float(local_pos[0]), float(local_pos[1]),
                 float(local_pos[2]))
        result = query_fn(point[0], point[1], point[2], yaw)
        if isinstance(result, int):
            status = int(result)
            hard = False
            cell = None
        else:
            status = int(result.status)
            hard = bool(getattr(result, "hard_reject_safe", False))
            indices = getattr(result, "indices", None)
            cell = None if indices is None else [int(v) for v in indices]
            if status in (2, 3):
                for seed in (getattr(result, "contact_seeds", None) or ()):
                    seeds.append(tuple(float(v) for v in seed))
        statuses.append(status)
        flags.append(hard)
        queries.append({
            "query_xyz": [round(v, 6) for v in point],
            "query_yaw": round(float(yaw), 6),
            "status": status,
            "hard_reject_safe": hard,
            "cell": cell,
        })
    prior = prior_from_statuses(statuses, seeds, flags)
    prior["queries"] = queries
    return prior


def _queries_match(captured, replayed):
    """True when a stored query record agrees with a fresh lookup.

    An empty capture has nothing to disagree with. A stored point that
    replays to a different cell or status is a contract break.
    """
    if not captured:
        return True
    if len(captured) != len(replayed):
        return False
    for left, right in zip(captured, replayed):
        if list(left.get("query_xyz") or []) != list(right.get("query_xyz") or []):
            return False
        if left.get("cell") != right.get("cell"):
            return False
        if int(left.get("status")) != int(right.get("status")):
            return False
    return True


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
        captured = cand.get("atlas_queries") or []
        replayed = on.get("queries") or []
        rows.append({
            "candidate_id": cand.get("candidate_id"),
            "slot_index": cand.get("slot_index"),
            "captured_reachability": cand.get("reachability"),
            "captured_reason": cand.get("reason") or "",
            "seed_sources": list(cand.get("seed_sources") or []),
            "n_targets": len(targets),
            "atlas_on": on["verdict"],
            "atlas_off": "no_atlas",
            "queries": replayed,
            "query_match": _queries_match(captured, replayed),
        })
    return rows
