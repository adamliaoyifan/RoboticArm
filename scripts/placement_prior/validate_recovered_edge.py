#!/usr/bin/env python3
"""Native validation of corrected spline samples on saved private scenes.

Uses the production ValidateRouteV2 endpoint with the normal 2s budget. This
checks one recovered edge, not full pick/place/release or measured grasp proof.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture',type=Path,required=True)
    parser.add_argument('--cases',type=Path,required=True)
    parser.add_argument('--checker',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    import yaml
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.serialization import deserialize_message,serialize_message
    from moveit_msgs.msg import PlanningScene, RobotTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    from luggage_msgs.srv import ValidateRouteV2
    from luggage_msgs.msg import RouteLegV2
    from luggage_planning.routing.lattice import retime_positions
    from luggage_planning.motion.moveit_state_validity import (
        sample_grid,interpolated_positions,DISTANCE_STRIDE,CERT_BUDGET_S,SERVICE_TIMEOUT_S,
    )
    from luggage_planning.motion.trajectory_timing import cartesian_joint_limits

    xml=(args.capture/'robot.urdf').read_text()
    semantic=(args.capture/'robot.srdf').read_text()
    model_hash=hashlib.sha256((xml+semantic).encode()).hexdigest()
    parameters=args.output/'parameters.yaml'
    parameters.write_text(yaml.safe_dump({'validate_route_v2':{'ros__parameters':{
        'robot_description':xml,'robot_description_semantic':semantic,'measure_distance':True}}}))
    report=dict(completed=False,scope='native recovered-edge sample validation only',
        motion=False,simulation=False,measured_grasp=False,budget_s=CERT_BUDGET_S,
        client_timeout_s=SERVICE_TIMEOUT_S,distance_stride=DISTANCE_STRIDE,
        model_sha256=model_hash,checker_sha256=hashlib.sha256(args.checker.read_bytes()).hexdigest(),
        revision=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        dirty=subprocess.check_output(['git','status','--short'],text=True).splitlines(),
        started=time.time(),results=[])
    report_path=args.output/'result.json'
    report_path.write_text(json.dumps(report,indent=2)+'\n')
    proc=node=executor=spin=None
    log=(args.output/'checker.log').open('w')
    try:
        proc=subprocess.Popen([str(args.checker),'--ros-args','--params-file',str(parameters)],
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        rclpy.init();node=rclpy.create_node('prior_recovered_edge_validation')
        executor=MultiThreadedExecutor(num_threads=2);executor.add_node(node)
        spin=threading.Thread(target=executor.spin,daemon=True);spin.start()
        client=node.create_client(ValidateRouteV2,'/validate_route_v2')
        if not client.wait_for_service(timeout_sec=15.):
            raise RuntimeError('native validator startup timeout')
        for offset in (0,20,40,80):
            case=args.cases/('offset-%d'%offset)
            output=args.output/('offset-%d'%offset);output.mkdir()
            geometry=deserialize_message((case/'geometric.cdr').read_bytes(),RobotTrajectory)
            scene_bytes=(case/'scene.cdr').read_bytes()
            scene=deserialize_message(scene_bytes,PlanningScene)
            trajectory=retime_positions([p.positions for p in geometry.joint_trajectory.points],.3,.3)
            times=sample_grid(trajectory);positions=interpolated_positions(trajectory)
            keep=[0]+[i for i in range(1,len(times)) if times[i]>times[i-1]]
            leg=RouteLegV2();leg.edge_id='offset-%d'%offset;leg.edge_class='TRAVERSE'
            leg.carrying=True;leg.load_state_index=0;leg.joint_names=list(trajectory.joint_names)
            leg.n_samples=len(keep);leg.sample_times_ns=[float(times[i]) for i in keep]
            leg.sample_positions_flat=[float(q) for i in keep for q in positions[i]]
            leg.knots.joint_names=list(trajectory.joint_names)
            for q,v,a,stamp in zip(trajectory.positions,trajectory.velocities,
                                   trajectory.accelerations,trajectory.time_from_start_ns):
                point=JointTrajectoryPoint(positions=list(q),velocities=list(v),accelerations=list(a))
                point.time_from_start.sec=int(stamp)//10**9
                point.time_from_start.nanosec=int(stamp)%10**9
                leg.knots.points.append(point)
            request=ValidateRouteV2.Request();request.request_id=leg.edge_id
            request.scene_revision='preserved-native-fixture'
            request.scene_digest=hashlib.sha256(scene_bytes).hexdigest()
            request.route_digest=hashlib.sha256(serialize_message(leg)).hexdigest()
            request.load_state_scenes=[scene];request.legs=[leg]
            request.max_sample_dt_ns=1e7;request.max_sample_step_m=0.
            request.distance_stride=DISTANCE_STRIDE;request.budget_s=CERT_BUDGET_S
            velocity,acceleration=cartesian_joint_limits(.3,.3)
            request.joint_velocity_limits=[velocity]*6
            request.joint_acceleration_limits=[acceleration]*6
            (output/'request.cdr').write_bytes(serialize_message(request))
            ready=threading.Event();future=client.call_async(request)
            future.add_done_callback(lambda f:ready.set())
            if not ready.wait(SERVICE_TIMEOUT_S):raise TimeoutError('validator response; request preserved')
            response=future.result();(output/'response.cdr').write_bytes(serialize_message(response))
            if (response.request_id!=request.request_id or response.route_digest!=request.route_digest
                    or response.scene_digest!=request.scene_digest or response.model_sha256!=model_hash):
                raise ValueError('validator response identity mismatch')
            rows=[]
            for result in response.leg_results:
                distances=[d for d in result.min_distance_m if math.isfinite(d)]
                rows.append(dict(valid=result.valid,reason=result.reason,detail=result.detail,
                    samples=result.n_samples,first_invalid=result.first_invalid_sample,
                    min_distance_m=min(distances) if distances else None,
                    contacts=[dict(a=c.body_1,b=c.body_2) for c in result.contacts]))
            row=dict(offset_mm=offset,result=response.result,all_valid=response.all_valid,
                detail=response.detail,service_time_s=response.service_time_ns/1e9,
                requested_samples=leg.n_samples,legs=rows)
            report['results'].append(row)
            report_path.write_text(json.dumps(report,indent=2)+'\n')
            print(json.dumps(row),flush=True)
        report['completed']=True
    finally:
        if executor is not None:executor.shutdown(timeout_sec=3.)
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        if spin is not None:spin.join(2.)
        if proc is not None and proc.poll() is None:
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=5.)
        log.close()
        report['checker_returncode']=None if proc is None else proc.returncode
        report['finished']=time.time()
        report_path.write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':main()
