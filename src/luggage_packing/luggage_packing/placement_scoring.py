#!/usr/bin/env python3
"""Global placement scoring shared by the online planner and offline replay.

``proxy_score`` (design 5.7) already weighs EMS regularity, insertion clearance
and the "do not wall off the deep interior" penalty. Two of its terms, however,
are structurally biased towards stacking: an unobserved floor column scores 0 on
both ``observation_confidence`` and ``support_quality`` while a committed box top
scores 1 on both. ``w_floor_first`` is the counterweight, so the floor fills up
before stacking starts without making it a hard constraint.

Keeping this pure (no ROS) lets the offline replay rank with exactly the policy
that runs on the robot.
"""

from __future__ import division

from luggage_packing.insertion_corridor import proxy_score

# Chosen so a floor placement outranks a low stack under the default
# proxy weights; calibrated against the offline replay.
DEFAULT_W_FLOOR_FIRST = 0.60

# Neutral prior for candidates no atlas could speak to.
DEFAULT_REACHABILITY_PRIOR = 0.5


def floor_first_term(candidate, usable_height):
    """1.0 on the floor, decaying linearly to 0.0 at the ceiling."""
    height = max(float(usable_height), 1e-6)
    peak = max(0.0, float(candidate.get("peak", 0.0)))
    return 1.0 - min(1.0, peak / height)


def score_candidate(candidate, model, ems, usable_inner, smallest_size,
                    opening_side="negative_x",
                    w_floor_first=DEFAULT_W_FLOOR_FIRST):
    """Return ``(score, breakdown)`` for one candidate."""
    base, breakdown = proxy_score(
        candidate, model, ems, usable_inner, smallest_size,
        reachability_prior=float(
            candidate.get("reachability_prior", DEFAULT_REACHABILITY_PRIOR)),
        opening_side=opening_side)
    floor_first = floor_first_term(candidate, usable_inner[2])
    breakdown["floor_first"] = round(floor_first, 4)
    return base + w_floor_first * floor_first, breakdown


def tie_break_key(candidate):
    """Deterministic ordering for equal scores.

    Without it, equal scores fall back to candidate generation order, which is
    the numpy grid scan order -- an accidental placement policy rather than a
    chosen one.
    """
    return (
        round(float(candidate.get("peak", 0.0)), 4),
        round(float(candidate["center_local"][0]), 4),
        round(float(candidate["center_local"][1]), 4),
        round(float(candidate.get("box_yaw", 0.0)), 4),
    )


def score_candidates(candidates, model, ems, usable_inner, smallest_size,
                     opening_side="negative_x",
                     w_floor_first=DEFAULT_W_FLOOR_FIRST,
                     keep_breakdown=True):
    """Score in place and return the list ranked best-first."""
    for candidate in candidates:
        score, breakdown = score_candidate(
            candidate, model, ems, usable_inner, smallest_size,
            opening_side=opening_side, w_floor_first=w_floor_first)
        candidate["score"] = round(score, 6)
        if keep_breakdown:
            candidate["score_breakdown"] = breakdown
    candidates.sort(key=lambda cand: (-cand["score"],) + tie_break_key(cand))
    return candidates
