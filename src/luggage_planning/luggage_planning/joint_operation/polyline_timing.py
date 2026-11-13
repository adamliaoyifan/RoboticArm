"""Time checked joint polylines without smoothing away opening waypoints.

OMPL connections are polylines with corners. Stop at their retained vertices;
quintic controller interpolation then stays on each checked straight segment.
Only floating-point collinear redundancy is removed. Local Cartesian phases
continue using their existing smooth-path timing.
"""
import math

import numpy as np

from luggage_planning.placement_prior.contracts import finite_tuple
from luggage_planning.motion.trajectory_timing import TimedPath, cartesian_joint_limits, HARDWARE_DECEL_RAD


def polyline_vertices(positions, required_positions=()):
    rows = tuple(finite_tuple(q, 6, 'polyline position') for q in positions)
    if len(rows) < 2:
        raise ValueError('at least two polyline positions required')
    required = {finite_tuple(q, 6, 'required waypoint') for q in required_positions}
    if not required.issubset(set(rows)):
        raise ValueError('required waypoint absent from checked polyline')
    result = [rows[0]]
    for index in range(1, len(rows) - 1):
        middle = rows[index]
        if middle == result[-1]:
            continue  # already retained, including required rows
        start, point, end = map(np.asarray, (result[-1], middle, rows[index + 1]))
        direction = end - start
        denominator = float(direction @ direction)
        fraction = float((point - start) @ direction) / denominator if denominator else -1.
        # Roundoff only, scaled by the original coordinates; no physical path
        # deviation tolerance or angle wrapping is introduced here.
        coordinate_scale = max(1., *(float(abs(q).max()) for q in (start, point, end)))
        roundoff = 64 * np.finfo(float).eps * coordinate_scale
        if (middle not in required and 0 <= fraction <= 1
                and float(abs(point - start - fraction * direction).max()) <= roundoff):
            continue
        result.append(middle)
    result.append(rows[-1])
    return tuple(result)


def time_parameterize_joint_polyline(positions, vel_scale, acc_scale, *, required_positions=()):
    rows = polyline_vertices(positions, required_positions)
    vmax, amax = cartesian_joint_limits(vel_scale, acc_scale)
    amax = min(amax, HARDWARE_DECEL_RAD)
    times = [0.]
    peak = 0.
    for start, end in zip(rows, rows[1:]):
        displacement = max(abs(a - b) for a, b in zip(start, end))
        # s(u)=10u^3-15u^4+6u^5 has max |s'|=15/8 and
        # max |s''|=10/sqrt(3). All joints share this monotone progress.
        duration = max(15 * displacement / (8 * vmax),
                       math.sqrt(10 * displacement / (math.sqrt(3) * amax)))
        # Numerical time headroom, as in existing spline_timing_bounds. Wire
        # times are nanosecond-rounded here before independent certification.
        duration_ns = max(1, math.ceil(duration * (1. + 1e-6) * 1e9))
        duration = duration_ns / 1e9
        times.append(round(times[-1] * 1e9 + duration_ns) / 1e9)
        peak = max(peak, 15 * displacement / (8 * duration))
    zero = [[0.] * 6 for _ in rows]
    timed = TimedPath(times, zero, [list(v) for v in zero],
                     sum(math.dist(a, b) for a, b in zip(rows, rows[1:])),
                     times[-1], peak, vmax, amax, amax)
    return rows, timed
