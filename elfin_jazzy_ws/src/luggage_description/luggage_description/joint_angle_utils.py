#!/usr/bin/env python3
"""Shared wrap-equivalent joint angle normalization.

Several wrist/base joints on the S20 (elfin_joint1/4/5/6) accept the same
physical pose at multiple 2π-equivalent angle values. When the robot is
initialized at one branch (e.g. J5=-4.6598) but MoveIt/controller targets the
equivalent branch (J5=+1.6234), ros_control follows the numerical difference
and spins the joint a full turn even though the pose is identical.

This module centralizes:
  * the set of wrap-equivalent joints,
  * the per-joint "nearest 2π-equivalent" picker, and
  * dict/list helpers reused by startup, observe-reset and motion planning.

All paths that produce joint targets for these joints should run them through
``normalize_joint_targets`` (or ``normalize_joint_map``) so the chosen branch
sits next to the controller's current angle. Doing this consistently is what
prevents J5/J6 from drifting through a 2π loop during init.
"""

from __future__ import annotations

import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

# Joints whose pose is invariant under +/-2π and that historically suffer the
# "initialized at the other branch" problem on the S20. Keep this in sync with
# the URDF's continuous/revolute joint limits.
WRAP_EQUIVALENT_JOINTS = frozenset(
    {
        "elfin_joint1",
        "elfin_joint4",
        "elfin_joint5",
        "elfin_joint6",
    }
)

# Default joint-limit window for wrap normalization. The S20 arm joints span
# roughly +/-2π, so we stay inside that envelope when picking an equivalent.
DEFAULT_LOWER = -2.0 * math.pi
DEFAULT_UPPER = 2.0 * math.pi


def closest_angle_equivalent(
    current: float,
    target: float,
    lower: float = DEFAULT_LOWER,
    upper: float = DEFAULT_UPPER,
) -> float:
    """Return the 2π-equivalent of ``target`` closest to ``current``.

    If the closest equivalent leaves the [lower, upper] window the original
    ``target`` is returned unchanged — staying in-limit is more important than
    minimizing the numerical delta.
    """
    nearest = target + (2.0 * math.pi) * round((current - target) / (2.0 * math.pi))
    if nearest < lower or nearest > upper:
        return target
    return nearest


def _wrap_set(wrap_joints: Optional[Iterable[str]]) -> frozenset:
    if wrap_joints is None:
        return WRAP_EQUIVALENT_JOINTS
    return frozenset(wrap_joints)


def normalize_joint_targets(
    joint_names: Sequence[str],
    current_values: Sequence[float],
    target_values: Sequence[float],
    wrap_joints: Optional[Iterable[str]] = None,
    lower: float = DEFAULT_LOWER,
    upper: float = DEFAULT_UPPER,
) -> Tuple[List[float], List[Tuple[str, float, float, float]]]:
    """Normalize ``target_values`` so wrap joints sit next to ``current_values``.

    Returns ``(adjusted_targets, rewrites)`` where ``rewrites`` lists
    ``(joint_name, raw_target, adjusted_target, current)`` for every joint that
    actually changed. Callers should log ``rewrites`` so debugging never
    silently sees a target value get rewritten under them.
    """
    if len(joint_names) != len(current_values) or len(joint_names) != len(target_values):
        raise ValueError(
            "joint_names/current_values/target_values length mismatch: %d/%d/%d"
            % (len(joint_names), len(current_values), len(target_values))
        )

    wraps = _wrap_set(wrap_joints)
    adjusted: List[float] = [float(v) for v in target_values]
    rewrites: List[Tuple[str, float, float, float]] = []
    for idx, name in enumerate(joint_names):
        if name not in wraps:
            continue
        raw = adjusted[idx]
        nearest = closest_angle_equivalent(
            float(current_values[idx]), raw, lower=lower, upper=upper
        )
        if abs(nearest - raw) > 1e-6:
            adjusted[idx] = nearest
            rewrites.append((name, raw, nearest, float(current_values[idx])))
    return adjusted, rewrites


def normalize_joint_map(
    current_by_joint: Dict[str, float],
    target_by_joint: Dict[str, float],
    wrap_joints: Optional[Iterable[str]] = None,
    lower: float = DEFAULT_LOWER,
    upper: float = DEFAULT_UPPER,
) -> Tuple[Dict[str, float], List[Tuple[str, float, float, float]]]:
    """Dict-based convenience wrapper for startup/config code.

    ``target_by_joint`` keys that are missing from ``current_by_joint`` are left
    untouched (we have no anchor to choose a branch against).
    """
    wraps = _wrap_set(wrap_joints)
    adjusted: Dict[str, float] = {k: float(v) for k, v in target_by_joint.items()}
    rewrites: List[Tuple[str, float, float, float]] = []
    for name, raw in list(adjusted.items()):
        if name not in wraps or name not in current_by_joint:
            continue
        current = float(current_by_joint[name])
        nearest = closest_angle_equivalent(current, raw, lower=lower, upper=upper)
        if abs(nearest - raw) > 1e-6:
            adjusted[name] = nearest
            rewrites.append((name, raw, nearest, current))
    return adjusted, rewrites


def format_rewrites(rewrites: Sequence[Tuple[str, float, float, float]]) -> str:
    """Render a rewrite list for logging."""
    return ", ".join(
        "%s raw=%.4f adjusted=%.4f current=%.4f" % (name, raw, nearest, current)
        for name, raw, nearest, current in rewrites
    )


def wrap_angular_error(current: float, target: float) -> float:
    """Minimum absolute joint error mod 2π (for drift detection)."""
    diff = abs(float(current) - float(target))
    wrap_diff = abs(diff - (2.0 * math.pi))
    return min(diff, wrap_diff)


def max_joint_error(
    joint_names: Sequence[str],
    current_values: Sequence[float],
    target_values: Sequence[float],
    wrap_joints: Optional[Iterable[str]] = None,
) -> float:
    """Max absolute joint error, using 2π-equivalence for wrap joints."""
    if len(joint_names) != len(current_values) or len(joint_names) != len(target_values):
        raise ValueError(
            "joint_names/current_values/target_values length mismatch: %d/%d/%d"
            % (len(joint_names), len(current_values), len(target_values))
        )
    wraps = _wrap_set(wrap_joints)
    errors = []
    for name, cur, tgt in zip(joint_names, current_values, target_values):
        if name in wraps:
            errors.append(wrap_angular_error(cur, tgt))
        else:
            errors.append(abs(float(cur) - float(tgt)))
    return max(errors)
