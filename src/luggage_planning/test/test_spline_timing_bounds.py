"""Continuous caps and preservation of the complete controller curve."""
import math
from copy import deepcopy
import numpy as np
import pytest

from luggage_planning.motion.spline_timing_bounds import derivative_bounds, required_time_scale
from luggage_planning.motion.trajectory_timing import (
    time_parameterize_cartesian, time_parameterize_rest_to_rest, cartesian_joint_limits,
    HARDWARE_DECEL_RAD,
)
from luggage_planning.motion.moveit_state_validity import _quintic_row


def test_standard_quintic_has_known_interior_velocity_and_acceleration_peaks():
    velocity, acceleration = derivative_bounds([[0.],[1.]], [0.,1.], [[0.],[0.]], [[0.],[0.]])
    assert velocity == pytest.approx(1.875)
    assert acceleration == pytest.approx(10/math.sqrt(3))


def test_cartesian_time_dilation_keeps_curve_and_bounds_interior_derivatives():
    positions = [[0.,0.],[.1,.015],[.2,.05],[.21,.06]]
    original = deepcopy(positions)
    v_limit, a_limit = cartesian_joint_limits(.3,.3)
    raw = time_parameterize_rest_to_rest(positions,v_limit,a_limit,HARDWARE_DECEL_RAD)
    raw_v, raw_a = derivative_bounds(positions,raw.times_s,raw.velocities_rad,raw.accelerations_rad)
    assert raw_a > a_limit or raw_v > v_limit
    timed = time_parameterize_cartesian(positions,.3,.3)
    assert timed.time_scale > 1 and positions == original
    maximum_v,maximum_a = derivative_bounds(positions,timed.times_s,timed.velocities_rad,timed.accelerations_rad)
    assert maximum_v <= v_limit and maximum_a <= a_limit
    for index in range(len(positions)-1):
        for s in np.linspace(0.,1.,21):
            def point(profile):
                return _quintic_row(positions[index],positions[index+1],
                    profile.velocities_rad[index],profile.velocities_rad[index+1],
                    profile.accelerations_rad[index],profile.accelerations_rad[index+1],
                    s,profile.times_s[index+1]-profile.times_s[index])
            assert point(timed) == pytest.approx(point(raw),abs=1e-12)
    # The wire uses integer nanoseconds; include that rounding in validation.
    wire_times = [round(t*1e9)/1e9 for t in timed.times_s]
    wire_v, wire_a = derivative_bounds(positions,wire_times,timed.velocities_rad,timed.accelerations_rad)
    assert wire_v <= v_limit and wire_a <= a_limit


def test_already_safe_constant_velocity_curve_is_not_slowed():
    assert required_time_scale([[0.],[1.]],[0.,10.],[[.1],[.1]],[[0.],[0.]],1.,1.) == 1.


@pytest.mark.parametrize('times', [[0.,0.],[0.,float('nan')],[1.,0.]])
def test_invalid_time_data_is_refused(times):
    with pytest.raises(ValueError):
        derivative_bounds([[0.],[1.]],times,[[0.],[0.]],[[0.],[0.]])
