#!/usr/bin/env python3
"""Pure selection and termination helpers for interior camera probes."""

from __future__ import division

from luggage_planning.constrained_view_planner import coverage_score


def evaluate_probe_termination(
        unknown_ratio, views_used, config, last_unknown=None, stagnant_count=0):
    """Evaluate closed-loop stopping and return the updated loop state."""
    views_used = max(0, int(views_used))
    unknown_ratio = float(unknown_ratio)
    termination = config["termination"]

    if views_used >= int(termination["max_views"]):
        return {
            "done": True,
            "reason": "max_views",
            "message": "max interior probe views reached",
            "last_unknown": unknown_ratio,
            "stagnant_count": int(stagnant_count),
        }
    if (
            views_used > 0
            and unknown_ratio <= float(termination["unknown_threshold"])):
        return {
            "done": True,
            "reason": "unknown_threshold",
            "message": "unknown below probe threshold",
            "last_unknown": unknown_ratio,
            "stagnant_count": int(stagnant_count),
        }

    next_stagnant = int(stagnant_count)
    if last_unknown is not None and views_used > 0:
        improvement = float(last_unknown) - unknown_ratio
        if improvement < float(config["min_improvement"]):
            next_stagnant += 1
        else:
            next_stagnant = 0
        if next_stagnant >= int(config["stagnation_limit"]):
            return {
                "done": True,
                "reason": "low_improvement",
                "message": "probe map improvement stagnated",
                "last_unknown": unknown_ratio,
                "stagnant_count": next_stagnant,
            }

    return {
        "done": False,
        "reason": "",
        "message": "",
        "last_unknown": unknown_ratio,
        "stagnant_count": next_stagnant,
    }


def rank_probe_candidates(views, frontier_points, used_indices, coverage_radius):
    """Rank geometry-valid unused probes by frontier coverage.

    With no useful frontier signal, deterministic geometry ordering chooses
    near-center probes before moving deeper and farther laterally.
    """
    constraints = {
        "coverage_radius": float(coverage_radius),
        "alignment_min": 0.1,
    }
    used = set(used_indices or [])
    scored = []
    states = {}
    for idx, view in enumerate(views):
        if idx in used:
            states[idx] = "used"
            continue
        if not view.get("valid_geometry", True):
            states[idx] = view.get("reject_reason", "geometry_failed")
            continue
        score = coverage_score(view, frontier_points or [], constraints)
        scored.append((score, idx, view))

    scored.sort(key=lambda item: (
        -item[0],
        item[2].get("depth", 0.0),
        abs(item[2].get("lateral_offset", 0.0)),
        item[1],
    ))
    for score, idx, _view in scored:
        states[idx] = "score=%.3f" % score
    return scored, states
