"""Continuous derivative bounds for the controller's quintic interpolation.

Positions, velocities and accelerations at knots define each polynomial. Knot
limits alone do not bound the polynomial between knots. This module computes
extrema and uniformly dilates time; it does not change joint positions or the
interpolated geometric curve.
"""
import math
import numpy as np
from numpy.polynomial.polynomial import polyval, polyroots, polyder


def _maximum_absolute(coefficients):
    samples = [0., 1.]
    for root in polyroots(polyder(coefficients)):
        if abs(root.imag) <= 1e-8 and 0 < root.real < 1:
            samples.append(float(root.real))
    value = max(abs(float(polyval(s, coefficients))) for s in samples)
    if not math.isfinite(value):
        raise ValueError('nonfinite spline derivative bound')
    return value


def derivative_bounds(positions, times, velocities, accelerations):
    """Return max |dq/dt| and |d²q/dt²| over all joints and intervals."""
    if not (len(positions) == len(times) == len(velocities) == len(accelerations)):
        raise ValueError('spline knot arrays must have equal lengths')
    if not len(positions):
        return 0., 0.
    n_joints = len(positions[0])
    for values in (positions, velocities, accelerations):
        array = np.asarray(values, dtype=float)
        if array.shape != (len(times), n_joints) or not np.isfinite(array).all():
            raise ValueError('invalid spline knot dimensions or values')
    if not all(math.isfinite(t) for t in times):
        raise ValueError('nonfinite spline time')
    maximum_v = max(abs(v) for row in velocities for v in row)
    maximum_a = max(abs(a) for row in accelerations for a in row)
    for i in range(len(times)-1):
        dt = times[i+1]-times[i]
        if dt <= 0:
            raise ValueError('spline times must increase')
        for j in range(n_joints):
            delta = positions[i+1][j]-positions[i][j]
            v0, v1 = velocities[i][j]*dt, velocities[i+1][j]*dt
            a0, a1 = accelerations[i][j]*dt*dt, accelerations[i+1][j]*dt*dt
            c3 = 10*delta-6*v0-4*v1-1.5*a0+.5*a1
            c4 = -15*delta+8*v0+7*v1+1.5*a0-a1
            c5 = 6*delta-3*v0-3*v1-.5*a0+.5*a1
            velocity = np.asarray([v0,a0,3*c3,4*c4,5*c5])/dt
            acceleration = np.asarray([a0,6*c3,12*c4,20*c5])/(dt*dt)
            maximum_v = max(maximum_v, _maximum_absolute(velocity))
            maximum_a = max(maximum_a, _maximum_absolute(acceleration))
    return maximum_v, maximum_a


def required_time_scale(positions, times, velocities, accelerations, v_limit, a_limit):
    if not all(math.isfinite(value) and value > 0 for value in (v_limit,a_limit)):
        raise ValueError('positive finite spline limits required')
    maximum_v, maximum_a = derivative_bounds(positions,times,velocities,accelerations)
    scale = max(1., maximum_v/v_limit, math.sqrt(maximum_a/a_limit))
    # Numerical headroom tightens limits; it never raises them. The caller
    # separately checks the actual nanosecond-rounded wire times.
    return scale*(1.+1e-6) if scale > 1. else 1.
