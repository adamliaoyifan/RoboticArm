#!/usr/bin/env python3
"""Analyze saved native geometry with site retiming; no robot services.

Quintic extrema are evaluated analytically between knots, not only at knots.
This is a timing diagnostic, not route certification or an executable command.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from numpy.polynomial import Polynomial
from rclpy.serialization import deserialize_message
from moveit_msgs.msg import RobotTrajectory
from luggage_planning.motion.trajectory_timing import time_parameterize_cartesian, cartesian_joint_limits


def extrema(poly):
    candidates = [0., 1.]
    candidates.extend(float(root.real) for root in poly.deriv().roots()
                      if abs(root.imag) < 1e-8 and 0 < root.real < 1)
    return max((abs(float(poly(s))), float(s)) for s in candidates)


def analyze(path, velocity_scale, acceleration_scale):
    trajectory = deserialize_message(path.read_bytes(), RobotTrajectory).joint_trajectory
    positions = np.asarray([p.positions for p in trajectory.points])
    timed = time_parameterize_cartesian(positions, velocity_scale, acceleration_scale)
    maximum_v = (0., None)
    maximum_a = (0., None)
    for index in range(len(positions)-1):
        dt = timed.times_s[index+1]-timed.times_s[index]
        for joint in range(positions.shape[1]):
            c0 = positions[index, joint]
            c1 = timed.velocities_rad[index][joint]*dt
            c2 = timed.accelerations_rad[index][joint]*dt*dt/2
            residual = [positions[index+1,joint]-c0-c1-c2,
                        timed.velocities_rad[index+1][joint]*dt-c1-2*c2,
                        timed.accelerations_rad[index+1][joint]*dt*dt-2*c2]
            c3,c4,c5 = np.linalg.solve([[1,1,1],[3,4,5],[6,12,20]],residual)
            poly = Polynomial([c0,c1,c2,c3,c4,c5])
            for derivative, scale, kind in [(1,dt,'v'), (2,dt*dt,'a')]:
                value, s = extrema(poly.deriv(derivative)/scale)
                record = dict(segment=index, joint=trajectory.joint_names[joint], fraction=s)
                if kind=='v' and value>maximum_v[0]: maximum_v=(value,record)
                if kind=='a' and value>maximum_a[0]: maximum_a=(value,record)
    v_limit,a_limit = cartesian_joint_limits(velocity_scale,acceleration_scale)
    return dict(input=str(path), knots=len(positions), duration_s=timed.duration,
        time_scale=timed.time_scale,
        velocity_scale=velocity_scale, acceleration_scale=acceleration_scale,
        velocity_limit=v_limit, acceleration_limit=a_limit,
        max_velocity=maximum_v[0], velocity_location=maximum_v[1],
        max_acceleration=maximum_a[0], acceleration_location=maximum_a[1],
        timing_within_limits=maximum_v[0]<=v_limit*(1+1e-9) and maximum_a[0]<=a_limit*(1+1e-9))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('cases',type=Path)
    parser.add_argument('--velocity-scale',type=float,default=.3)
    parser.add_argument('--acceleration-scale',type=float,default=.3)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    results=[analyze(path,args.velocity_scale,args.acceleration_scale)
             for path in sorted(args.cases.glob('offset-*/geometric.cdr'))]
    destination=args.output or args.cases/'site_retiming.json'
    with destination.open('x') as stream:
        json.dump(dict(scope='geometry retiming diagnostic; not full-route certification',results=results),stream,indent=2)
        stream.write('\n')
    for result in results:
        print(Path(result['input']).parent.name, 'acceleration',result['max_acceleration'],
              'limit',result['acceleration_limit'],'within',result['timing_within_limits'])

if __name__=='__main__': main()
