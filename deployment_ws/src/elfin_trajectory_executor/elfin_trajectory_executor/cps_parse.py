"""Parse Huayan CPS result lists without importing the SDK.

CPS sendAndRecv typically fills ``result`` with strings. Joint angles and
velocities are degrees / deg/s. ``HRIF_ReadActPos`` layout (6-axis):

* ``[0:6]``  actual ACS joints (deg)
* ``[6:12]`` Cartesian pose used by ``HRIF_ReadActTcpPos`` (mm, deg)
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence


def as_bit(values: Optional[Sequence]) -> Optional[int]:
    """CPS DI/DO ``result[0]`` as 0 or 1. None if missing or not a bit."""
    if values is None or len(values) < 1:
        return None
    raw = values[0]
    if isinstance(raw, bool):
        return int(raw)
    if isinstance(raw, (int, float)) and not isinstance(raw, bool):
        if raw in (0, 1, 0.0, 1.0):
            return int(raw)
        return None
    text = str(raw).strip().lower()
    if text in ("1", "true", "on"):
        return 1
    if text in ("0", "false", "off"):
        return 0
    return None


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


def acs_from_read_act_pos(values: Optional[Sequence]) -> Optional[List[float]]:
    """Extract actual ACS joints (deg) from a full ``HRIF_ReadActPos`` result."""
    return as_float_list(values, 6)


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


def joints_moved_deg(
    prev_deg: Sequence[float],
    curr_deg: Sequence[float],
    eps_deg: float = 1e-4,
) -> bool:
    """True if any of the first six joints moved more than ``eps_deg``."""
    if len(prev_deg) < 6 or len(curr_deg) < 6:
        return False
    return any(
        abs(float(curr_deg[i]) - float(prev_deg[i])) > eps_deg for i in range(6)
    )


def choose_joint_vel_deg(
    cps_vel: Optional[List[float]],
    fd_vel: Optional[List[float]],
    moved: bool,
    eps_deg_s: float = 1e-6,
) -> tuple[Optional[List[float]], str]:
    """Prefer ``ReadActJointVel``; finite-diff if CPS is missing or stuck at 0 while moving."""
    if cps_vel is not None and any(abs(v) > eps_deg_s for v in cps_vel):
        return cps_vel, "cps"
    if moved and fd_vel is not None:
        return fd_vel, "finite_diff"
    if cps_vel is not None:
        return cps_vel, "cps"
    if fd_vel is not None:
        return fd_vel, "finite_diff"
    return None, "none"


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
