"""Rest-to-rest trapezoidal timing for cartesian joint paths.

No ROS imports. Geometric knots come from GetCartesianPath; this module
assigns ``time_from_start``, joint velocity, and joint acceleration so a
short dip cannot race at constant ``v_max``.
"""

from __future__ import division

import math
from dataclasses import dataclass

# Match elfin_moveit_config/config/joint_limits.yaml.
JOINT_VEL_LIMIT_RAD = 1.57
JOINT_ACC_LIMIT_RAD = 3.14
# Proven S20 WayPoint command caps (executor.yaml).
HARDWARE_VEL_RAD = math.radians(60.0)
HARDWARE_ACC_RAD = math.radians(60.0)

_EPS_S = 1e-12
_TIME_BUMP_S = 1e-6


@dataclass(frozen=True)
class TimedPath:
    times_s: list
    velocities_rad: list
    accelerations_rad: list
    path_length: float
    duration: float
    v_peak: float
    v_max: float
    a_max: float


def cartesian_joint_limits(vel_scale, acc_scale):
    """Scaled URDF limits, capped to the site 60 deg/s and 60 deg/s^2."""
    vel_scale = float(vel_scale)
    acc_scale = float(acc_scale)
    if not math.isfinite(vel_scale) or vel_scale <= 0.0:
        vel_scale = 1e-6
    if not math.isfinite(acc_scale) or acc_scale <= 0.0:
        acc_scale = 1e-6
    v_max = min(JOINT_VEL_LIMIT_RAD * vel_scale, HARDWARE_VEL_RAD)
    a_max = min(JOINT_ACC_LIMIT_RAD * acc_scale, HARDWARE_ACC_RAD)
    return max(v_max, 1e-6), max(a_max, 1e-6)


def time_parameterize_cartesian(positions_rad, vel_scale, acc_scale):
    v_max, a_max = cartesian_joint_limits(vel_scale, acc_scale)
    return time_parameterize_rest_to_rest(positions_rad, v_max, a_max)


def time_parameterize_rest_to_rest(positions_rad, v_max, a_max):
    """Trapezoid (or triangle) on joint-space path length, rest to rest."""
    v_max = float(v_max)
    a_max = float(a_max)
    if not math.isfinite(v_max) or v_max <= 0.0:
        raise ValueError("v_max must be positive")
    if not math.isfinite(a_max) or a_max <= 0.0:
        raise ValueError("a_max must be positive")
    knots = [_as_joints(q) for q in positions_rad]
    if not knots:
        return TimedPath([], [], [], 0.0, 0.0, 0.0, v_max, a_max)
    n_joints = len(knots[0])
    s_knots = _cumulative_length(knots)
    path_length = s_knots[-1]
    profile = _trapezoid_profile(path_length, v_max, a_max)
    times = []
    vels = []
    accs = []
    prev_t = -1.0
    for index, s_query in enumerate(s_knots):
        t, s_dot, s_ddot = _profile_state(profile, s_query)
        if t <= prev_t:
            t = prev_t + _TIME_BUMP_S
        prev_t = t
        tangent = _path_tangent(knots, s_knots, index, n_joints)
        times.append(t)
        vels.append([comp * s_dot for comp in tangent])
        accs.append([comp * s_ddot for comp in tangent])
    vels[0] = [0.0] * n_joints
    accs[0] = [tangent * profile.a_max for tangent in _path_tangent(
        knots, s_knots, 0, n_joints)]
    vels[-1] = [0.0] * n_joints
    accs[-1] = [-comp * profile.a_max for comp in _path_tangent(
        knots, s_knots, len(knots) - 1, n_joints)]
    if path_length <= _EPS_S:
        accs[0] = [0.0] * n_joints
        accs[-1] = [0.0] * n_joints
    return TimedPath(
        times, vels, accs, path_length, times[-1],
        profile.v_peak, v_max, a_max,
    )


def _as_joints(raw):
    joints = [float(v) for v in raw]
    if not joints:
        raise ValueError("joint vector is empty")
    if any(not math.isfinite(v) for v in joints):
        raise ValueError("joint vector is non-finite")
    return joints


def _cumulative_length(knots):
    s_knots = [0.0]
    for prev, nxt in zip(knots, knots[1:]):
        if len(nxt) != len(prev):
            raise ValueError("joint vector length mismatch")
        delta = math.sqrt(sum((b - a) ** 2 for a, b in zip(prev, nxt)))
        s_knots.append(s_knots[-1] + delta)
    return s_knots


class _Trapezoid(object):
    __slots__ = ("s_total", "v_peak", "a_max", "t_acc", "t_cruise", "t_total",
                 "s_acc")

    def __init__(self, s_total, v_peak, a_max, t_acc, t_cruise, t_total, s_acc):
        self.s_total = s_total
        self.v_peak = v_peak
        self.a_max = a_max
        self.t_acc = t_acc
        self.t_cruise = t_cruise
        self.t_total = t_total
        self.s_acc = s_acc


def _trapezoid_profile(s_total, v_max, a_max):
    if s_total <= _EPS_S:
        return _Trapezoid(0.0, 0.0, a_max, 0.0, 0.0, 0.0, 0.0)
    # Two ramps of v^2 / (2a) need v_max^2 / a_max of path.
    if s_total * a_max <= v_max * v_max:
        v_peak = math.sqrt(s_total * a_max)
        t_acc = v_peak / a_max
        s_acc = 0.5 * s_total
        return _Trapezoid(
            s_total, v_peak, a_max, t_acc, 0.0, 2.0 * t_acc, s_acc)
    t_acc = v_max / a_max
    s_acc = 0.5 * a_max * t_acc * t_acc
    s_cruise = s_total - 2.0 * s_acc
    t_cruise = s_cruise / v_max
    return _Trapezoid(
        s_total, v_max, a_max, t_acc, t_cruise,
        2.0 * t_acc + t_cruise, s_acc,
    )


def _profile_state(profile, s_query):
    s_query = min(max(float(s_query), 0.0), profile.s_total)
    if profile.t_total <= _EPS_S:
        return 0.0, 0.0, 0.0
    s_acc = profile.s_acc
    s_cruise_end = s_acc + profile.v_peak * profile.t_cruise
    if s_query <= s_acc:
        t = math.sqrt(2.0 * s_query / profile.a_max) if profile.a_max > 0.0 else 0.0
        return t, profile.a_max * t, profile.a_max
    if s_query <= s_cruise_end + _EPS_S and profile.t_cruise > _EPS_S:
        t = profile.t_acc + (s_query - s_acc) / profile.v_peak
        return t, profile.v_peak, 0.0
    remaining = max(profile.s_total - s_query, 0.0)
    tau = math.sqrt(2.0 * remaining / profile.a_max) if profile.a_max > 0.0 else 0.0
    t = profile.t_total - tau
    return t, profile.a_max * tau, -profile.a_max


def _path_tangent(knots, s_knots, index, n_joints):
    tangent = [0.0] * n_joints
    weight = 0.0
    if index > 0:
        ds = s_knots[index] - s_knots[index - 1]
        if ds > _EPS_S:
            for j in range(n_joints):
                tangent[j] += (knots[index][j] - knots[index - 1][j]) / ds
            weight += 1.0
    if index + 1 < len(knots):
        ds = s_knots[index + 1] - s_knots[index]
        if ds > _EPS_S:
            for j in range(n_joints):
                tangent[j] += (knots[index + 1][j] - knots[index][j]) / ds
            weight += 1.0
    if weight > 0.0:
        return [comp / weight for comp in tangent]
    return tangent
