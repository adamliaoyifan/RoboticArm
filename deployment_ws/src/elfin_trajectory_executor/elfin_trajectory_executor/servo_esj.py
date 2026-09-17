"""实机代码 (site / real-cell). Not part of the Gazebo simulation stack.

Unused on this S20. PushServoEsJ 7th field is unpublished and this CPS
rejects 0 and 0.02 with 20006. Kept only for protocol notes / offline tests.
The executor is waypoint-only.
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

from .waypoint_profile import (
    REASON_NONFINITE,
    REASON_POINT_COUNT,
    REASON_TIME_NOT_INCREASING,
    validate_joints_deg,
    validate_times_s,
)

SERVO_DT_S = 0.02
SERVO_LOOKAHEAD_S = 0.2
MAX_PUSH_POINTS = 500
# HR6.5 rejects a 151-point PushServoEsJ (~12 KB) with 20006. SDK max is
# 500; keep each TCP command to one lookahead window.
PUSH_BATCH_POINTS = 10
MIN_READ_STATE_INTERVAL_S = 0.02
# Java HRIF_PushServoEsJ_base: nPointSize * 7 == field count. 6n is
# 20007. Field 7 is required and unnamed. This S20 rejects 0 and 0.02
# with 20006 (20260916_1204_h4, 20260916_1722_servo_esj_field7_002).
VALUES_PER_POINT = 7


def interpolate_joint_grid(
    joints_deg: Sequence[Sequence[float]],
    times_s: Sequence[float],
    *,
    dt_s: float = SERVO_DT_S,
    velocities_deg: Optional[Sequence[Optional[Sequence[float]]]] = None,
) -> Tuple[Optional[List[List[float]]], Optional[str]]:
    """Sample the MoveIt trajectory on a fixed grid.

    Keeps the exact final point. Terminal velocity is zero because the last
    sample is the goal pose (repeat the last knot if needed).
    """
    if not joints_deg or not times_s:
        return None, REASON_POINT_COUNT
    if len(joints_deg) != len(times_s):
        return None, REASON_POINT_COUNT
    reason = validate_times_s(times_s)
    if reason and reason != REASON_TIME_NOT_INCREASING:
        return None, reason
    if reason:
        return None, reason
    joint_reason = validate_joints_deg(joints_deg)
    if joint_reason:
        return None, joint_reason
    if dt_s <= 0.0 or not math.isfinite(dt_s):
        return None, REASON_NONFINITE

    t0 = float(times_s[0])
    t_end = float(times_s[-1])
    if t_end < t0:
        return None, REASON_TIME_NOT_INCREASING

    samples: List[float] = []
    t = t0
    # Guard against float drift: stop before t_end, then append exact t_end.
    while t < t_end - 1e-12:
        samples.append(t)
        t += dt_s
        if len(samples) > 100000:
            return None, REASON_POINT_COUNT
    samples.append(t_end)

    grid: List[List[float]] = []
    for t_sample in samples:
        q = _interp_at(t_sample, joints_deg, times_s, velocities_deg)
        if q is None:
            return None, REASON_NONFINITE
        grid.append(q)
    grid[-1] = [float(x) for x in joints_deg[-1][:6]]
    return grid, None


def _interp_at(
    t: float,
    joints_deg: Sequence[Sequence[float]],
    times_s: Sequence[float],
    velocities_deg: Optional[Sequence[Optional[Sequence[float]]]],
) -> Optional[List[float]]:
    if t <= float(times_s[0]):
        return [float(x) for x in joints_deg[0][:6]]
    if t >= float(times_s[-1]):
        return [float(x) for x in joints_deg[-1][:6]]
    hi = 1
    while hi < len(times_s) and float(times_s[hi]) < t:
        hi += 1
    lo = hi - 1
    t0 = float(times_s[lo])
    t1 = float(times_s[hi])
    span = t1 - t0
    if span <= 1e-12:
        return [float(x) for x in joints_deg[hi][:6]]
    u = (t - t0) / span
    q0 = joints_deg[lo]
    q1 = joints_deg[hi]
    v0 = _vel_at(velocities_deg, lo)
    v1 = _vel_at(velocities_deg, hi)
    out: List[float] = []
    for j in range(6):
        p0 = float(q0[j])
        p1 = float(q1[j])
        if not math.isfinite(p0) or not math.isfinite(p1):
            return None
        if v0 is not None and v1 is not None:
            # Cubic Hermite using deg/s over the segment duration.
            m0 = float(v0[j]) * span
            m1 = float(v1[j]) * span
            u2 = u * u
            u3 = u2 * u
            val = (
                (2 * u3 - 3 * u2 + 1) * p0
                + (u3 - 2 * u2 + u) * m0
                + (-2 * u3 + 3 * u2) * p1
                + (u3 - u2) * m1
            )
        else:
            val = p0 + u * (p1 - p0)
        if not math.isfinite(val):
            return None
        out.append(val)
    return out


def _vel_at(
    velocities_deg: Optional[Sequence[Optional[Sequence[float]]]],
    idx: int,
) -> Optional[Sequence[float]]:
    if velocities_deg is None or idx >= len(velocities_deg):
        return None
    row = velocities_deg[idx]
    if row is None or len(row) < 6:
        return None
    if any(not math.isfinite(float(v)) for v in row[:6]):
        return None
    return row


def batch_joint_points(
    points: Sequence[Sequence[float]],
    max_batch: int = MAX_PUSH_POINTS,
) -> List[List[List[float]]]:
    if max_batch < 1:
        raise ValueError("max_batch must be >= 1")
    batches: List[List[List[float]]] = []
    buf: List[List[float]] = []
    for q in points:
        buf.append([float(x) for x in q[:6]])
        if len(buf) >= max_batch:
            batches.append(buf)
            buf = []
    if buf:
        batches.append(buf)
    return batches


def flatten_servo_esj_points(
    batch: Sequence[Sequence[float]],
    extra: Optional[float] = None,
    extras: Optional[Sequence[float]] = None,
) -> List[float]:
    """Flatten ACS points for HRIF_PushServoEsJ (7 fields per point).

    Fields 1-6 are joint degrees. Field 7 is required (else 20007) and
    unnamed. Constant 0.0 and 0.02 are both CPS 20006 on this S20.
    """
    if extras is not None and len(extras) != len(batch):
        raise ValueError("extras length must match batch")
    out: List[float] = []
    for i, q in enumerate(batch):
        if len(q) < 6:
            raise ValueError("ServoEsJ point needs 6 joints")
        out.extend(float(q[j]) for j in range(6))
        if extras is not None:
            out.append(float(extras[i]))
        elif len(q) > 6:
            out.append(float(q[6]))
        elif extra is not None:
            out.append(float(extra))
        else:
            out.append(float(i) * SERVO_DT_S)
    return out


def servo_push_allowed(state_row: Sequence) -> Optional[bool]:
    """Python SDK: result[0] == 0 means pushing is allowed."""
    if not state_row:
        return None
    try:
        return int(float(state_row[0])) == 0
    except (TypeError, ValueError):
        return None
