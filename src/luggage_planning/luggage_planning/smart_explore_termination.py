#!/usr/bin/env python3
"""Pure marginal-gain early-stop helper for smart_explore phase0.

Phase0 (the opening-arc views) currently runs every configured view
unconditionally: it has no per-view information-gain scoring (unlike phase1)
and its only exit is the global ``unknown_ratio <= unknown_threshold`` check,
which an empty container never reaches. In practice the opening-arc views
share the same look-at and orientation, so after the first one or two views
the marginal unknown-ratio improvement is negligible. This module mirrors the
stagnation logic already used by ``interior_probe_planner`` so the remaining
phase0 views can be skipped once gains dry up, falling through to phase1.
"""

from __future__ import division


def phase0_gain_exhausted(
        last_unknown, unknown_ratio, phase0_used,
        stagnant_count, min_improvement, stagnation_limit):
    """Decide whether remaining phase0 views should be skipped.

    Args:
        last_unknown: unknown_ratio observed after the previous phase0 view,
            or ``None`` if no phase0 view has completed yet.
        unknown_ratio: unknown_ratio observed after the most recent phase0
            view.
        phase0_used: number of phase0 views already executed (not just
            selected/pending). Must be >= 1 before this can report exhausted
            -- the opening views also feed ``container_opening_estimator``
            and provide a safe entry pose, so at least one must always run.
        stagnant_count: consecutive prior views whose improvement was below
            ``min_improvement``.
        min_improvement: minimum unknown_ratio drop counted as real gain.
        stagnation_limit: consecutive low-gain views required to exhaust.

    Returns:
        (exhausted, next_stagnant_count, reason) tuple. ``reason`` is
        ``"low_improvement"`` when exhausted, else ``""``.
    """
    phase0_used = max(0, int(phase0_used))
    next_stagnant = int(stagnant_count)

    if phase0_used < 1 or last_unknown is None:
        return False, 0, ""

    improvement = float(last_unknown) - float(unknown_ratio)
    if improvement < float(min_improvement):
        next_stagnant += 1
    else:
        next_stagnant = 0

    if next_stagnant >= int(stagnation_limit):
        return True, next_stagnant, "low_improvement"
    return False, next_stagnant, ""


def phase0_low_fov(phase0_used, inside_container_fov_ratio, min_inside_fov):
    """Decide whether an about-to-run phase0 candidate should be skipped
    because too little of its field of view actually falls inside the
    container (e.g. the suction-down camera's residual tilt is aimed at the
    robot's own pedestal instead of the cargo interior).

    Unlike ``phase0_gain_exhausted`` (evaluated *after* a view has executed,
    from the unknown_ratio it produced), this is evaluated on the candidate
    *before* handing it to the planner, so a low-FOV candidate can be skipped
    without paying for its OMPL solve/simplify cost.

    Args:
        phase0_used: number of phase0 views already executed. Must be >= 1
            before this can skip -- the mandatory first opening view always
            runs regardless of its own FOV, since it also seeds
            ``container_opening_estimator`` and a safe entry pose.
        inside_container_fov_ratio: fraction of the candidate's rays that
            fall inside the container's inner box, or ``None`` if the metric
            is unavailable (opening geometry not yet known, or the mapper's
            evaluate service is unreachable/erroring). ``None`` always fails
            open (never skips) -- a metric outage must not silently drop
            phase0 views.
        min_inside_fov: minimum ratio required to keep running this
            candidate.

    Returns:
        True if this (and, by the caller's convention, the remaining
        not-yet-run) phase0 candidates should be skipped.
    """
    if int(phase0_used) < 1:
        return False
    if inside_container_fov_ratio is None:
        return False
    return float(inside_container_fov_ratio) < float(min_inside_fov)
