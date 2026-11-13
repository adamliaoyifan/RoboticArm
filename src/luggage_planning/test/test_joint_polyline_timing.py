"""Joint polylines keep exact openings and continuous controller limits."""
import math
import pytest
from luggage_planning.joint_operation.polyline_timing import polyline_vertices,time_parameterize_joint_polyline
from luggage_planning.motion.spline_timing_bounds import derivative_bounds


def q(x,y=0.):return (x,y,0.,0.,0.,0.)


def test_collinear_dense_checks_can_be_removed_but_opening_witness_remains():
    path=[q(i/100) for i in range(101)]
    assert polyline_vertices(path)==(q(0.),q(1.))
    assert polyline_vertices(path,(q(.5),))==(q(0.),q(.5),q(1.))


def test_reversal_and_actual_bends_remain_with_original_winding():
    path=(q(6.),q(5.),q(4.),q(5.),q(5.,.001),q(-6.,.001))
    rows=polyline_vertices(path)
    assert rows==(q(6.),q(4.),q(5.),q(5.,.001),q(-6.,.001))


def test_missing_required_waypoint_is_not_inserted_into_checked_geometry():
    with pytest.raises(ValueError,match='absent'):
        polyline_vertices((q(0.),q(1.)),(q(.5),))


def test_turns_stop_with_bounded_quintic_derivatives_without_whole_leg_slowdown():
    path=(q(0.),q(1.),q(1.,.005),q(2.,.005))
    rows,timed=time_parameterize_joint_polyline(path,.2,.1)
    assert rows==path
    assert 0<timed.duration<20.  # two 1rad spans at 0.314rad/s, plus short turn
    assert all(not any(v) for v in timed.velocities_rad+timed.accelerations_rad)
    velocity,acceleration=derivative_bounds(rows,timed.times_s,timed.velocities_rad,timed.accelerations_rad)
    assert velocity<=timed.v_max and acceleration<=timed.a_max
    assert all(round(t*1e9)==t*1e9 for t in timed.times_s)


def test_stationary_connection_remains_a_finite_two_knot_hold():
    rows,timed=time_parameterize_joint_polyline((q(0.),)*4,.2,.1)
    assert rows==(q(0.),q(0.)) and timed.duration>0
    assert derivative_bounds(rows,timed.times_s,timed.velocities_rad,timed.accelerations_rad)==(0.,0.)


@pytest.mark.parametrize('path',[(q(0.),),(q(0.),q(math.nan))])
def test_invalid_polyline_refuses(path):
    with pytest.raises(ValueError):time_parameterize_joint_polyline(path,.2,.1)
