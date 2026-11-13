#!/usr/bin/env python3
"""ROS prior-node wiring rehearsal using explicitly synthetic OCC confirmation.

Historical robot/scene geometry exercises native collision screening. This is
NOT evidence of measured occupancy, sealable pickup, or full-route success.
"""
import argparse
import copy
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--probe-log',type=Path,required=True)
    parser.add_argument('--checker',type=Path,required=True)
    parser.add_argument('--prefetch',action='store_true')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    import yaml
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from rclpy.serialization import serialize_message,deserialize_message
    from std_msgs.msg import String
    from rcl_interfaces.srv import GetParameters
    from rcl_interfaces.msg import ParameterValue
    from moveit_msgs.msg import PlanningScene
    from moveit_msgs.srv import GetPlanningScene,GetCartesianPath
    from luggage_msgs.msg import SurfaceContractV2
    from luggage_msgs.srv import PreparePlacementPriorV2
    from luggage_description.frame_diagnostics import BoundaryContext
    from luggage_description.ros_frame_bindings.surfaces import surface_to_message
    from luggage_planning.place.place_service import pose_message
    from luggage_planning.placement_prior.contracts import Transform
    from luggage_planning.placement_prior.pick import transform_from_pose
    from geometry_msgs.msg import Pose
    workspace=Path(__file__).resolve().parents[2]
    rows=args.probe_log.read_text().splitlines()
    box_row=next(row.split() for row in rows if row.startswith('BOX size '))
    bquat=next(row.split()[1:] for row in rows if row.startswith('BOX_QUAT '))
    dims=tuple(map(float,box_row[2:5]))
    box=Transform(tuple(map(float,box_row[6:9])),tuple(map(float,bquat)))
    original=deserialize_message((args.capture/'req-1a/edge-0000.request.cdr').read_bytes(),GetCartesianPath.Request)
    scene=deserialize_message((args.capture/'req-1a/edge-0000.scene.cdr').read_bytes(),PlanningScene)
    tcp=transform_from_pose(original.waypoints[-1]);shifted=Transform((tcp.translation[0]+.02,*tcp.translation[1:]),tcp.quaternion_xyzw)
    def pose(t):
        p=Pose();p.position.x,p.position.y,p.position.z=t.translation
        p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=t.quaternion_xyzw;return p
    parameters=dict(robot_description=(args.capture/'robot.urdf').read_text(),
                    robot_description_semantic=(args.capture/'robot.srdf').read_text())
    config=args.output/'parameters.yaml';config.write_text(yaml.safe_dump({
        'placement_tool_check':{'ros__parameters':parameters},
        'place_prior':{'ros__parameters':dict(parameters,frame_scene_revision='rehearsal',
            container_translation=[0.,0.,0.],container_quaternion_xyzw=[0.,0.,0.,1.],
            prefetch_enabled=args.prefetch,prefetch_box_wdh=[.2,.2,.2],
            prefetch_grasps_flat=[0.,0.,.1,1.,0.,0.,0.])}}))
    surface=dict(contract_version=2,kind='surface',frame_id='container_link',length_unit='m',
        stamp_ns=0,time_basis='scene',scene_revision='rehearsal',request_id='occ',
        geometry_hash='synthetic-wiring-only',map_revision=7,height_datum='container_origin',
        origin_xy_container=[10.,10.],resolution_m=.1,floor_z_container=0.,ceiling_z_container=2.,
        surface_z_container=[[0.]],state=[['free']],confidence=[['sensor']],height_above_floor=[[0.]])
    context=BoundaryContext('PRIOR_OCC','fixture','place_prior','occ','rehearsal','container_link','',
                            'scene',surface['geometry_hash'],7)
    surface_msg=surface_to_message(surface,context)
    confirmation=dict(frame='container_local',map_revision=7,geometry_hash=surface['geometry_hash'],
        state=[['free']],height=[[0.]],map_source='measured',occupancy_confirmed=False,
        acquisition_stamp_ns=123,pending_changes=[],placed_count=0)
    request=PreparePlacementPriorV2.Request();request.request_id='synthetic-prior-rehearsal'
    request.measured=False;request.template_box_wdh=list(dims)
    request.template_box_from_tcp=[pose(box.inverse().compose(t)) for t in (tcp,shifted)]
    request.geometry_hash=surface['geometry_hash'];request.expected_map_revision=7
    request.max_slots=1;request.candidate_limit=2;request.budget_s=10.
    request.slots=[pose_message(dict(contract_version=2,frame_id='container_link',length_unit='m',
        stamp_ns=0,time_basis='scene',scene_revision='rehearsal',request_id=request.request_id,
        reference='box_center',orientation_semantics='box_axis_pi',position=box.translation,quaternion_xyzw=box.quaternion_xyzw))]
    (args.output/'request.cdr').write_bytes(serialize_message(request))
    (args.output/'surface.cdr').write_bytes(serialize_message(surface_msg))
    (args.output/'scene.cdr').write_bytes(serialize_message(scene))
    (args.output/'confirmation.json').write_text(json.dumps(confirmation,indent=2))
    log=(args.output/'checker.log').open('w')
    proc=subprocess.Popen([str(args.checker),'--ros-args','--params-file',str(config)],
                          stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init(args=['--ros-args','--params-file',str(config)])
    spec=importlib.util.spec_from_file_location('prior_node',workspace/'src/luggage_planning/scripts/place_prior_node.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    node=module.PlacePriorNode();fixture=Node('prior_snapshot_fixture',use_global_arguments=False)
    fixture.create_service(GetParameters,'/cargo_volume_mapper/get_parameters',
        lambda req,res: set_parameters(res,ParameterValue))
    fixture.create_service(GetPlanningScene,'/get_planning_scene',lambda req,res:set_scene(res,scene))
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub=fixture.create_publisher(SurfaceContractV2,'/luggage/cargo_map/surface_v2',qos)
    legacy_pub=fixture.create_publisher(String,'/luggage/cargo_map/surface_2d',qos)
    client=fixture.create_client(PreparePlacementPriorV2,'/placement_prior/prepare')
    executor=MultiThreadedExecutor(num_threads=6);executor.add_node(fixture);executor.add_node(node)
    spin=threading.Thread(target=executor.spin,daemon=True);spin.start()
    report={'passed':False,'synthetic_OCC':True,'measured_pick':False,'motion':False,'results':[]}
    def publish():
        pub.publish(surface_msg);legacy_pub.publish(String(data=json.dumps(confirmation)))
    def call(label):
        ready=threading.Event();future=client.call_async(request);future.add_done_callback(lambda f:ready.set())
        if not ready.wait(20.):raise TimeoutError('prior service')
        response=future.result();(args.output/(label+'.cdr')).write_bytes(serialize_message(response))
        report['results'].append(dict(label=label,success=response.success,reason=response.reason,
            choices=[c.candidate_id for c in response.choices],cache_hit=response.cache_hit,elapsed_s=response.elapsed_s))
        return response
    try:
        assert client.wait_for_service(timeout_sec=10.)
        deadline=time.monotonic()+10
        while pub.get_subscription_count()<1 or not node._worker._collision._client.service_is_ready():
            if time.monotonic()>deadline:raise TimeoutError('subscription/checker readiness')
            time.sleep(.02)
        publish();time.sleep(.15)
        assert not call('unconfirmed').success
        confirmation['occupancy_confirmed']=True;publish()
        deadline=time.monotonic()+2
        while node._store.current() is None:
            if time.monotonic()>deadline:raise TimeoutError('confirmed snapshot')
            time.sleep(.01)
        result=call('confirmed');assert result.success, result.reason
        assert [c.candidate_id for c in result.choices]==['template-1']
        assert call('cached').cache_hit
        confirmation['occupancy_confirmed']=False;legacy_pub.publish(String(data=json.dumps(confirmation)))
        deadline=time.monotonic()+2
        while node._store.current() is not None:
            if time.monotonic()>deadline:raise TimeoutError('snapshot invalidation')
            time.sleep(.01)
        assert not call('invalidated').success
        if args.prefetch:
            from luggage_description.container_geometry import cuboid_from_inner_size
            descriptor=cuboid_from_inner_size((1.,1.,1.),floor_z=2.).descriptor()
            surface.update(geometry_hash=descriptor['geometry_hash'],map_revision=8,
                origin_xy_container=[-.5,-.5],floor_z_container=2.,ceiling_z_container=3.,
                surface_z_container=[[2.]*10 for _ in range(10)],
                height_above_floor=[[0.]*10 for _ in range(10)],
                state=[['free']*10 for _ in range(10)],
                confidence=[['sensor']*10 for _ in range(10)])
            context=BoundaryContext('PRIOR_OCC','fixture','place_prior','occ','rehearsal',
                                    'container_link','','scene',surface['geometry_hash'],8)
            surface_msg=surface_to_message(surface,context)
            confirmation.update(geometry_hash=surface['geometry_hash'],map_revision=8,
                occupancy_confirmed=True,geometry_descriptor=descriptor,inner_size=[1.,1.,1.],
                state=surface['state'],height=[[0.]*10 for _ in range(10)])
            (args.output/'prefetch-surface.cdr').write_bytes(serialize_message(surface_msg))
            (args.output/'prefetch-confirmation.json').write_text(json.dumps(confirmation,indent=2))
            started=time.monotonic()
            publish()  # no Prepare call: the node must initiate the computation
            deadline=started+20.
            while node._prefetched is None:
                if time.monotonic()>deadline:raise TimeoutError('automatic template prefetch')
                time.sleep(.02)
            prefetched_request,prefetched_result=node._prefetched
            assert not prefetched_request.measured
            assert prefetched_request.snapshot.map_revision==8 and prefetched_result.choices
            generation=node._worker.poll()[0]
            time.sleep(1.2)
            assert node._worker.poll()[0]==generation, 'duplicate prefetch for same snapshot'
            report['automatic_prefetch']=dict(choices=len(prefetched_result.choices),
                elapsed_s=time.monotonic()-started,duplicate_job=False,measured=False)
            confirmation['occupancy_confirmed']=False;publish()
            deadline=time.monotonic()+2.
            while node._prefetched is not None:
                if time.monotonic()>deadline:raise TimeoutError('automatic prefetch invalidation')
                time.sleep(.01)
            report['automatic_prefetch']['invalidated']=True
        report['passed']=True
    finally:
        executor.shutdown(timeout_sec=3.);node.destroy_node();fixture.destroy_node();rclpy.shutdown();spin.join(2.)
        os.killpg(proc.pid,signal.SIGINT)
        try:proc.wait(timeout=5.)
        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=5.)
        report['checker_returncode']=proc.returncode;log.close()
        (args.output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


def set_parameters(response,typ):
    response.values=[typ(type=1,bool_value=True)];return response

def set_scene(response,scene):
    response.scene=copy.deepcopy(scene);return response

if __name__=='__main__':main()
