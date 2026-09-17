"""实机代码 (site / real-cell). Not part of the Gazebo simulation stack.

Convert a MoveIt FollowJointTrajectory into absolute joint-degree samples
on the PushServoJ time grid. MoveIt TOTG uses ~0.10 s knots; HRIF_PushServoJ
needs a fixed servo_time (default 0.02 s).
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

from .servo_esj import interpolate_joint_grid


def fjt_to_servo_j_deg(
    positions_rad: Sequence[Sequence[float]],
    times_s: Sequence[float],
    *,
    dt_s: float,
    velocities_rad: Optional[Sequence[Optional[Sequence[float]]]] = None,
) -> Tuple[Optional[List[List[float]]], Optional[str]]:
    """Resample FJT knots (rad, seconds) onto a fixed ServoJ grid (deg)."""
    if not positions_rad:
        return [], None
    if len(positions_rad) != len(times_s):
        return None, "FJT position/time count mismatch"
    joints_deg: List[List[float]] = []
    for index, q in enumerate(positions_rad):
        if len(q) != 6:
            return None, "point %d has %d joints; expected 6" % (index, len(q))
        try:
            deg = [math.degrees(float(v)) for v in q]
        except (TypeError, ValueError):
            return None, "point %d has non-numeric positions" % index
        if any(not math.isfinite(v) for v in deg):
            return None, "point %d has non-finite positions" % index
        joints_deg.append(deg)
    vel_deg = None
    if velocities_rad is not None and len(velocities_rad) == len(positions_rad):
        converted: List[Optional[Sequence[float]]] = []
        usable = True
        for item in velocities_rad:
            if item is None or len(item) != 6:
                usable = False
                break
            try:
                converted.append([math.degrees(float(v)) for v in item])
            except (TypeError, ValueError):
                usable = False
                break
            if any(not math.isfinite(v) for v in converted[-1]):
                usable = False
                break
        if usable:
            vel_deg = converted
    return interpolate_joint_grid(
        joints_deg, times_s, dt_s=dt_s, velocities_deg=vel_deg,
    )


def densify_servo_j_path(
    path_deg: Sequence[Sequence[float]],
    dt_s: float,
    max_vel_deg: float,
) -> List[List[float]]:
    """Insert linear samples so |dq|/dt stays at or under max_vel_deg."""
    if not path_deg:
        return []
    if dt_s <= 0.0 or max_vel_deg <= 0.0 or not math.isfinite(max_vel_deg):
        return [list(map(float, q[:6])) for q in path_deg]
    max_step = max_vel_deg * dt_s
    out: List[List[float]] = [list(map(float, path_deg[0][:6]))]
    for raw in path_deg[1:]:
        nxt = list(map(float, raw[:6]))
        prev = out[-1]
        delta = max(abs(nxt[i] - prev[i]) for i in range(6))
        steps = 1 if delta <= max_step else int(math.ceil(delta / max_step))
        for k in range(1, steps + 1):
            u = k / float(steps)
            out.append([prev[j] + u * (nxt[j] - prev[j]) for j in range(6)])
    return out


def servo_j_hold_count(lookahead_time: float, servo_time: float) -> int:
    """Pushes of the final joints after the path, covering lookahead."""
    if servo_time <= 0.0 or lookahead_time <= 0.0:
        return 1
    return max(1, int(math.ceil(float(lookahead_time) / float(servo_time))))
