"""Parse Huayan CPS result lists without importing the SDK.

CPS sendAndRecv typically fills ``result`` with strings. Joint angles and
velocities are degrees / deg/s. ``HRIF_ReadActPos`` layout (6-axis):

* ``[0:6]``  actual ACS joints (deg)
* ``[6:12]`` Cartesian pose used by ``HRIF_ReadActTcpPos`` (mm, deg)
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence


def as_float_list(values: Optional[Sequence], n: int = 6) -> Optional[List[float]]:
    """Return the first ``n`` values as floats, or None if the row is short/invalid."""
    if values is None or len(values) < n:
        return None
    out: List[float] = []
    try:
        for i in range(n):
            out.append(float(values[i]))
    except (TypeError, ValueError):
        return None
    return out


def tcp_from_read_act_pos(values: Optional[Sequence]) -> Optional[List[float]]:
    """Extract TCP mm + RPY deg from a full ``HRIF_ReadActPos`` result."""
    if values is None or len(values) < 12:
        return None
    return as_float_list(values[6:12], 6)


def finite_diff_deg_s(
    prev_deg: Sequence[float],
    curr_deg: Sequence[float],
    dt_s: float,
) -> Optional[List[float]]:
    """Joint velocity (deg/s) from two ACS samples. None if dt is too small."""
    if dt_s <= 1e-4 or len(prev_deg) < 6 or len(curr_deg) < 6:
        return None
    return [(float(curr_deg[i]) - float(prev_deg[i])) / dt_s for i in range(6)]


def rpy_deg_to_quat_xyzw(rx_deg: float, ry_deg: float, rz_deg: float) -> List[float]:
    """Huayan Rx,Ry,Rz (deg) as intrinsic XYZ RPY -> quaternion xyzw."""
    roll, pitch, yaw = (math.radians(rx_deg), math.radians(ry_deg), math.radians(rz_deg))
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]
