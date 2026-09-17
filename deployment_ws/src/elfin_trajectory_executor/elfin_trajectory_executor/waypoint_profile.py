"""实机代码 (site / real-cell). Not part of the Gazebo simulation stack.

ROS-free MoveJ command profiles. Tests import this without rclpy or CPS.so.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

from .cps_parse import as_float_list

HUAYAN_MIN_VELOCITY_DEG = 1.0
DEFAULT_COMMAND_ACCEL_DEG = 60.0
VEL_ACCEL_MARGIN_DEG = 1.0
SITE_REJECT_ACCEL_DEG = 108.0
BLEND_RADIUS_MM = 5.0
FINAL_BLEND_RADIUS_MM = 0.0

REASON_NONFINITE = "nonfinite"
REASON_TIME_NOT_INCREASING = "time_not_increasing"
REASON_JOINT_LIMIT = "joint_limit"
REASON_ACCEL_BELOW_MIN_VEL_RULE = "accel_below_min_vel_rule"
REASON_CONTROLLER_LIMIT = "controller_limit"
REASON_POINT_COUNT = "point_count"

PREFLIGHT_FAILED_LOG = "Trajectory profile preflight failed before motion"

JOINT_NAMES: List[str] = [
    "elfin_joint1",
    "elfin_joint2",
    "elfin_joint3",
    "elfin_joint4",
    "elfin_joint5",
    "elfin_joint6",
]

JOINT_LIMITS_DEG: List[Tuple[float, float]] = [
    (-360.0, 360.0),
    (-360.0, 360.0),
    (-360.0, 360.0),
    (-360.0, 360.0),
    (-360.0, 360.0),
    (-360.0, 360.0),
]


@dataclass
class ControllerLimits:
    vel_deg: Optional[List[float]] = None
    acc_deg: Optional[List[float]] = None
    jerk_deg: Optional[List[float]] = None

    def min_vel(self) -> Optional[float]:
        return _min_positive(self.vel_deg)

    def min_acc(self) -> Optional[float]:
        return _min_positive(self.acc_deg)


@dataclass
class WaypointCommand:
    index: int
    joints_deg: List[float]
    vel_deg: float
    accel_deg: float
    radius_mm: float


@dataclass
class ProfileBuild:
    commands: List[WaypointCommand] = field(default_factory=list)
    reason: Optional[str] = None


def _min_positive(values: Optional[Sequence[float]]) -> Optional[float]:
    if not values or len(values) < 6:
        return None
    finite = [float(v) for v in values[:6] if math.isfinite(float(v)) and float(v) > 0.0]
    if len(finite) < 6:
        return None
    return min(finite)


def parse_joint_limit_row(values: Optional[Sequence], n: int = 6) -> Optional[List[float]]:
    """Six finite positive CPS limit values, or None."""
    parsed = as_float_list(values, n)
    if parsed is None:
        return None
    if any(not math.isfinite(x) or x <= 0.0 for x in parsed):
        return None
    return parsed


def duration_to_sec(duration) -> float:
    if duration is None:
        return 0.0
    if isinstance(duration, (int, float)):
        return float(duration)
    sec = float(getattr(duration, "sec", 0.0))
    nanosec = float(getattr(duration, "nanosec", 0.0))
    return sec + nanosec * 1e-9


def max_joint_delta_deg(a: Sequence[float], b: Sequence[float]) -> float:
    n = min(6, len(a), len(b))
    if n <= 0:
        return 0.0
    return max(abs(float(a[i]) - float(b[i])) for i in range(n))


def clamp_command_profile(
    raw_vel_deg: float,
    *,
    command_acceleration_deg: float,
    max_velocity_deg: float,
    min_controller_vel: Optional[float] = None,
    min_controller_acc: Optional[float] = None,
    min_velocity_deg: float = HUAYAN_MIN_VELOCITY_DEG,
    vel_accel_margin_deg: float = VEL_ACCEL_MARGIN_DEG,
) -> Tuple[Optional[float], Optional[float], Optional[str]]:
    """Return (vel, accel, None) or (None, None, reason).

    Accel is the configured command value, never vel*1.5 / vel*2. The
    historically rejected 108 deg/s^2 profile is always illegal.
    """
    if not math.isfinite(raw_vel_deg) or not math.isfinite(command_acceleration_deg):
        return None, None, REASON_NONFINITE
    if not math.isfinite(max_velocity_deg):
        return None, None, REASON_NONFINITE

    accel = float(command_acceleration_deg)
    if min_controller_acc is not None:
        if not math.isfinite(min_controller_acc) or min_controller_acc <= 0.0:
            return None, None, REASON_CONTROLLER_LIMIT
        accel = min(accel, float(min_controller_acc))
    if accel >= SITE_REJECT_ACCEL_DEG - 1e-9:
        return None, None, REASON_CONTROLLER_LIMIT
    if accel <= 0.0:
        return None, None, REASON_ACCEL_BELOW_MIN_VEL_RULE

    vel_hi = float(max_velocity_deg)
    if min_controller_vel is not None:
        if not math.isfinite(min_controller_vel) or min_controller_vel <= 0.0:
            return None, None, REASON_CONTROLLER_LIMIT
        vel_hi = min(vel_hi, float(min_controller_vel))

    vel = min(max(float(raw_vel_deg), 0.0), vel_hi)
    max_legal_vel = accel - float(vel_accel_margin_deg)
    if max_legal_vel + 1e-12 < float(min_velocity_deg):
        return None, None, REASON_ACCEL_BELOW_MIN_VEL_RULE
    if vel > max_legal_vel:
        vel = max_legal_vel
    if vel < float(min_velocity_deg):
        vel = float(min_velocity_deg)
        if vel > max_legal_vel + 1e-12:
            return None, None, REASON_ACCEL_BELOW_MIN_VEL_RULE
    return vel, accel, None


def estimate_raw_velocity_deg(
    *,
    point_velocities_rad: Optional[Sequence[float]],
    joints_deg: Sequence[float],
    next_joints_deg: Optional[Sequence[float]],
    dt_s: Optional[float],
    default_velocity_deg: float,
    max_velocity_deg: float,
) -> float:
    """Unsigned command speed before accel-margin clamp."""
    fallback = max(HUAYAN_MIN_VELOCITY_DEG, float(default_velocity_deg))
    hi = max(fallback, float(max_velocity_deg))

    def _clip(vel: float) -> float:
        return max(fallback, min(float(vel), hi))

    if point_velocities_rad:
        max_vel = max(abs(math.degrees(v)) for v in point_velocities_rad)
        if max_vel > 0.0 and math.isfinite(max_vel):
            return _clip(max_vel)
    if next_joints_deg is not None and dt_s is not None and dt_s > 1e-6:
        vel = max_joint_delta_deg(next_joints_deg, joints_deg) / dt_s
        if vel > 0.0 and math.isfinite(vel):
            return _clip(vel)
    return fallback


def validate_times_s(times_s: Sequence[float]) -> Optional[str]:
    if not times_s:
        return REASON_POINT_COUNT
    for t in times_s:
        if not math.isfinite(float(t)):
            return REASON_NONFINITE
    for i in range(1, len(times_s)):
        if float(times_s[i]) <= float(times_s[i - 1]):
            return REASON_TIME_NOT_INCREASING
    return None


def validate_joints_deg(
    joints_deg: Sequence[Sequence[float]],
    joint_limits_deg: Sequence[Tuple[float, float]] = JOINT_LIMITS_DEG,
) -> Optional[str]:
    if not joints_deg:
        return REASON_POINT_COUNT
    for q in joints_deg:
        if len(q) < 6:
            return REASON_POINT_COUNT
        for idx in range(6):
            val = float(q[idx])
            if not math.isfinite(val):
                return REASON_NONFINITE
            lo, hi = joint_limits_deg[idx]
            if val < lo or val > hi:
                return REASON_JOINT_LIMIT
    return None


def build_waypoint_profiles(
    joints_deg: Sequence[Sequence[float]],
    times_s: Sequence[float],
    kept: Sequence[int],
    *,
    point_velocities_rad: Optional[Sequence[Optional[Sequence[float]]]] = None,
    joint_limits_deg: Sequence[Tuple[float, float]] = JOINT_LIMITS_DEG,
    command_acceleration_deg: float = DEFAULT_COMMAND_ACCEL_DEG,
    max_velocity_deg: float = 20.0,
    default_velocity_deg: float = 20.0,
    controller_limits: Optional[ControllerLimits] = None,
    blend_radius_mm: float = BLEND_RADIUS_MM,
    final_blend_radius_mm: float = FINAL_BLEND_RADIUS_MM,
) -> ProfileBuild:
    """Build every kept MoveJ command, or a fail-closed reason with no commands."""
    if not kept:
        return ProfileBuild(reason=REASON_POINT_COUNT)
    time_reason = validate_times_s(times_s)
    if time_reason:
        return ProfileBuild(reason=time_reason)
    joint_reason = validate_joints_deg(joints_deg, joint_limits_deg)
    if joint_reason:
        return ProfileBuild(reason=joint_reason)
    if any(i < 0 or i >= len(joints_deg) or i >= len(times_s) for i in kept):
        return ProfileBuild(reason=REASON_POINT_COUNT)

    min_vel = controller_limits.min_vel() if controller_limits else None
    min_acc = controller_limits.min_acc() if controller_limits else None
    commands: List[WaypointCommand] = []
    for k, i in enumerate(kept):
        is_last = k == len(kept) - 1
        next_i = kept[k + 1] if not is_last else None
        next_deg = joints_deg[next_i] if next_i is not None else None
        dt = (times_s[next_i] - times_s[i]) if next_i is not None else None
        pt_vel = None
        if point_velocities_rad is not None and i < len(point_velocities_rad):
            pt_vel = point_velocities_rad[i]
        raw_vel = estimate_raw_velocity_deg(
            point_velocities_rad=pt_vel,
            joints_deg=joints_deg[i],
            next_joints_deg=next_deg,
            dt_s=dt,
            default_velocity_deg=default_velocity_deg,
            max_velocity_deg=max_velocity_deg,
        )
        vel, accel, reason = clamp_command_profile(
            raw_vel,
            command_acceleration_deg=command_acceleration_deg,
            max_velocity_deg=max_velocity_deg,
            min_controller_vel=min_vel,
            min_controller_acc=min_acc,
        )
        if reason or vel is None or accel is None:
            return ProfileBuild(reason=reason or REASON_CONTROLLER_LIMIT)
        radius = final_blend_radius_mm if is_last else blend_radius_mm
        commands.append(
            WaypointCommand(
                index=int(i),
                joints_deg=[float(x) for x in joints_deg[i][:6]],
                vel_deg=float(vel),
                accel_deg=float(accel),
                radius_mm=float(radius),
            )
        )
    return ProfileBuild(commands=commands)
