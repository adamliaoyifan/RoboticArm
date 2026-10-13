#!/usr/bin/env python3
"""Joint-space carry paths: wrap, continuity, and dense interpolation.

ROS-free. The executor turns the dense knots into a trajectory and sweeps
the FK polyline. A branch jump is a rejection, not a point MoveIt may smooth
through with RRTConnect.
"""

from __future__ import division

import math


class JointPathError(ValueError):
    """Fail-closed joint-path rejection. ``reason`` is a stable code."""

    def __init__(self, reason):
        self.reason = str(reason)
        ValueError.__init__(self, reason)


def wrap_near(joints, reference):
    """Equivalent angles of ``joints`` nearest to ``reference``."""
    out = []
    for value, origin in zip(joints, reference):
        delta = (float(value) - float(origin) + math.pi) % (2.0 * math.pi) - math.pi
        out.append(float(origin) + delta)
    return out


def max_joint_step(left, right):
    """Largest absolute joint change after wrapping ``right`` toward ``left``."""
    wrapped = wrap_near(right, left)
    return max(abs(a - b) for a, b in zip(wrapped, left))


def _shortest_arc(left, right):
    """Signed shortest step from ``left`` to ``right``, in (-pi, pi]."""
    return (float(right) - float(left) + math.pi) % (2.0 * math.pi) - math.pi


def joint_motion_report(positions, path_length_m=None):
    """Per-joint motion of one Cartesian joint trajectory.

    ``end_delta_rad`` is the wrapped start-to-end change. ``cumulative_rad``
    sums the absolute shortest step at every sample. ``max_step_rad`` is the
    largest of those steps. The excursion ratio uses the same 0.05 m floor
    as the Cartesian gate.
    """
    rows = [tuple(float(v) for v in row) for row in (positions or ())]
    if len(rows) < 2:
        return None
    width = len(rows[0])
    if any(len(row) != width for row in rows):
        return None
    end_delta = [_shortest_arc(rows[0][i], rows[-1][i]) for i in range(width)]
    cumulative = [0.0] * width
    max_step = [0.0] * width
    for left, right in zip(rows, rows[1:]):
        for index in range(width):
            step = abs(_shortest_arc(left[index], right[index]))
            cumulative[index] += step
            if step > max_step[index]:
                max_step[index] = step
    worst = max(range(width), key=lambda index: cumulative[index])
    report = {
        "worst_joint_index": worst,
        "end_delta_rad": [round(value, 6) for value in end_delta],
        "cumulative_rad": [round(value, 6) for value in cumulative],
        "max_step_rad": [round(value, 6) for value in max_step],
        "n_points": len(rows),
        "start_q": [round(value, 6) for value in rows[0]],
        "end_q": [round(value, 6) for value in rows[-1]],
    }
    if path_length_m is not None and float(path_length_m) > 1e-9:
        denominator = max(float(path_length_m), 0.05)
        report["path_length_m"] = round(float(path_length_m), 6)
        report["denominator_m"] = round(denominator, 6)
        report["ratio"] = cumulative[worst] / denominator
        report["variation_rad"] = cumulative[worst]
    return report


def densify(knots, continuity_rad, sample_rad=0.02):
    """Interpolate ``knots`` so consecutive samples stay within ``sample_rad``.

    Consecutive input knots farther than ``continuity_rad`` raise
    ``JointPathError('joint_branch_jump')``.
    """
    if not knots:
        raise JointPathError("empty_joint_path")
    continuity = float(continuity_rad)
    sample = max(float(sample_rad), 1e-4)
    dense = [[float(v) for v in knots[0]]]
    for raw in knots[1:]:
        nxt = wrap_near(raw, dense[-1])
        step = max(abs(a - b) for a, b in zip(nxt, dense[-1]))
        if step > continuity + 1e-9:
            raise JointPathError("joint_branch_jump")
        pieces = max(1, int(math.ceil(step / sample)))
        base = dense[-1]
        for index in range(1, pieces + 1):
            blend = float(index) / float(pieces)
            dense.append([
                a + blend * (b - a) for a, b in zip(base, nxt)])
    return dense


def validate_joint_path(knots, continuity_rad, sample_rad=0.02,
                        fk_fn=None, sweep_fn=None, fk_stride=1):
    """Densify, then optionally FK and sweep. Returns the dense knots.

    ``sweep_fn(xyz_points)`` returns an object with ``collides`` and
    ``reason``, or ``(ok, reason)``.
    """
    dense = densify(knots, continuity_rad, sample_rad=sample_rad)
    if fk_fn is None and sweep_fn is None:
        return dense
    stride = max(1, int(fk_stride))
    sample_idx = list(range(0, len(dense), stride))
    if sample_idx[-1] != len(dense) - 1:
        sample_idx.append(len(dense) - 1)
    polyline = []
    for index in sample_idx:
        if fk_fn is None:
            break
        point = fk_fn(dense[index])
        if point is None:
            raise JointPathError("fk_unavailable")
        polyline.append(tuple(float(v) for v in point))
    if sweep_fn is not None and polyline:
        swept = sweep_fn(polyline)
        if isinstance(swept, tuple):
            ok, reason = swept
            if not ok:
                raise JointPathError(reason or "sweep_rejected")
        elif getattr(swept, "collides", False):
            raise JointPathError(getattr(swept, "reason", "") or "sweep_rejected")
    return dense
