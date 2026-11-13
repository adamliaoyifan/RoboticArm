"""Generate carriage withdrawal references from a measured supported start.

These are two-axis joint setpoints for an independent fixture, not arm plans.
Collision certification and native detach acknowledgement remain mandatory
consumer preconditions; this module never grants execution admission.
"""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class CarriageMotion:
    name: str
    time_from_start_s: tuple
    joint_displacements_xz_m: tuple
    joint_velocities_xz_mps: tuple
    container_T_tool_start: tuple
    container_T_tool_end: tuple
    velocity_bound_mps: float
    acceleration_bound_mps2: float
    execution_admitted: bool = False


def _motion(name, start_q, end_q, start_tool, config, policy, fixture_policy):
    delta = np.asarray(end_q) - start_q
    distance = float(np.linalg.norm(delta))
    if distance <= 0:
        raise ValueError('RELEASE_WITHDRAWAL_DISTANCE_NONPOSITIVE')
    speed = min(config.contact_speed_mps, fixture_policy.velocity_limit_mps)
    acceleration = config.push_acceleration_mps2
    # s(u)=10u^3-15u^4+6u^5 has max s'=15/8 and max |s''|=10/sqrt(3).
    duration = max(15. / 8. * distance / speed,
                   math.sqrt(10. / math.sqrt(3.) * distance / acceleration))
    period = min(policy.monitor_period_s, config.sweep_step_m / speed)
    times = np.linspace(0., duration, math.ceil(duration / period) + 1)
    u = times / duration
    progress = 10 * u**3 - 15 * u**4 + 6 * u**5
    rate = (30 * u**2 - 60 * u**3 + 30 * u**4) / duration
    positions = np.asarray(start_q)[None, :] + progress[:, None] * delta
    velocities = rate[:, None] * delta
    if np.max(np.abs(positions)) > fixture_policy.travel_limit_m:
        raise ValueError('RELEASE_CARRIAGE_TRAVEL_LIMIT')
    finish = np.array(start_tool, dtype=float).reshape(4, 4).copy()
    # These coordinates are container absolute; carriage axes were independently
    # checked to be container +X/+Z, regardless of tool yaw in the fixture.
    finish[[0, 2], 3] += delta
    return CarriageMotion(name, tuple(map(float, times)), tuple(map(tuple, positions)),
        tuple(map(tuple, velocities)), tuple(np.asarray(start_tool).ravel()), tuple(finish.ravel()),
        15. / 8. * distance / duration, 10. / math.sqrt(3.) * distance / duration**2)


def make_release_withdrawal(sample, config, policy, fixture_policy):
    """Plan downward separation then outward travel using runtime distances."""
    q = np.array(sample.joint_displacements_xz_m, dtype=float)
    if q.shape != (2,) or not np.isfinite(q).all():
        raise ValueError('INVALID_RELEASE_START_JOINTS')
    down_q = q + [0., -config.release_drop_m]
    down = _motion('separate_down', q, down_q, sample.container_T_tool,
                   config, policy, fixture_policy)
    outward = _motion('withdraw_outward', down_q, down_q + [-config.withdraw_m, 0.],
        down.container_T_tool_end, config, policy, fixture_policy)
    return down, outward
