#!/usr/bin/env python3
"""One bounded native planning replay from exact Select and PlanningScene CDR.

Starts isolated MoveIt only. Restores the recorded scene in that owned process,
then calls the production Select boundary/producer. No Gazebo or execution.
Captured occupancy and attachment metadata retain their original provenance.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import threading
import time
from types import SimpleNamespace


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture',type=Path,required=True)
    parser.add_argument('--scene-tf',type=Path,required=True)
    parser.add_argument('--robot-poses',type=Path,required=True)
    parser.add_argument('--observe-pose',required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--select-index',type=int,default=0,
                        help='Zero-based captured Select call to replay')
    parser.add_argument('--surface',type=Path,
                        help='Exact matching surface; required for later calls')
    parser.add_argument('--scene',type=Path,
                        help='Exact pre-planning scene; required for later calls')
    parser.add_argument('--attachment-request',type=Path)
    parser.add_argument('--attachment-response',type=Path)
    parser.add_argument('--prior-response',type=Path)
    parser.add_argument('--choice-index',type=int)
    parser.add_argument('--baseline-consumer',action='store_true',
                        help='Run production ComputePlacementV2 then Select, with recorded actual attachment and no prior')
    parser.add_argument('--placement-parameters',type=Path,
                        help='Explicit baseline production placement parameters JSON; required for baseline-consumer')
    parser.add_argument('--measure-resources',action='store_true',
                        help='Record sampled client/producer/MoveIt process-tree resources; no live prior process')
    args=parser.parse_args()
    if args.select_index < 0 or (args.select_index and (not args.surface or not args.scene)):
        parser.error('later Select calls require explicit matching --surface and --scene')
    consumer_inputs=(args.attachment_request,args.attachment_response,args.prior_response,args.choice_index)
    hardware_consumer=any(value is not None for value in consumer_inputs)
    if args.baseline_consumer:
        if not args.attachment_request or not args.attachment_response or not args.placement_parameters:
            parser.error('baseline-consumer requires actual attachment request/response and placement-parameters')
        if args.prior_response is not None or args.choice_index is not None:
            parser.error('baseline-consumer cannot use a prior response or choice')
        hardware_consumer=True
    elif hardware_consumer and any(value is None for value in consumer_inputs):
        parser.error('hardware consumer requires attachment request/response, prior response and choice index')
    if os.environ.get('ROS_DOMAIN_ID')!='8':raise ValueError('requires isolated domain 8')
    # isolated_moveit.launch.py restricts every server to localhost. Apply the
    # same discovery scope to this client before rclpy.init, including env -i.
    os.environ['ROS_LOCALHOST_ONLY']='1'
    from process_guard import dedicated_domain_processes
    if dedicated_domain_processes('8'):raise RuntimeError('domain 8 occupied')
    args.output.mkdir(parents=True,exist_ok=False)
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.serialization import deserialize_message,serialize_message
    from moveit_msgs.msg import PlanningScene
    from moveit_msgs.srv import ApplyPlanningScene
    from rcl_interfaces.srv import GetParameters
    from luggage_msgs.srv import SelectPlacePathV2,GetCargoMapStats,ValidateRouteV2
    from rosidl_runtime_py.convert import message_to_ordereddict
    from luggage_description.scene_tf_config_utils import load_scene_tf_config
    from luggage_planning.place_service import request_dict,fill_select_response_route
    from luggage_planning.route_producer_factory import build_production_route_producer
    b=args.capture/'dumps/typed_service_boundaries'
    rows=[json.loads(line) for line in (b/'boundaries.jsonl').read_text().splitlines()]
    select_indices=[i for i,r in enumerate(rows) if r['boundary']=='/trajectory_selector/select_place_path_v2:request']
    select_index=select_indices[args.select_index]
    request_path=b/rows[select_index]['file']
    request=deserialize_message(request_path.read_bytes(),SelectPlacePathV2.Request)
    stats_row=next(r for r in reversed(rows[:select_index]) if r['boundary']=='/cargo_map/get_stats:response')
    stats=message_to_ordereddict(deserialize_message((b/stats_row['file']).read_bytes(),GetCargoMapStats.Response))
    surface_path=args.surface or next((args.capture/'dumps').glob('trial_*/replay/surface_2d.json'))
    surface=json.loads(surface_path.read_text())
    scene_path=args.scene or args.capture/'portal_boundaries/carrying_joint_00/scene.cdr'
    scene=deserialize_message(scene_path.read_bytes(),PlanningScene)
    document=request_dict(request)
    identity_path=args.capture/'dumps/runtime_identity.json'
    captured_identity=json.loads(identity_path.read_text())
    import yaml
    initial=yaml.safe_load(args.robot_poses.read_text())['poses'][args.observe_pose]['values']
    if len(initial)!=6:raise ValueError('six captured initial joints required')
    xacro_extra=','.join('initial_joint%d:=%.6f'%(i+1,float(value)) for i,value in enumerate(initial))
    if surface['map_revision']!=request.map_revision:raise ValueError('capture map revision mismatch')
    report={'success':False,'scope':'native exact-input diagnostic; not hardware F2 or execution',
            'revision':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
            'dirty':subprocess.check_output(['git','status','--porcelain'],text=True),
            'inputs':{str(p.resolve()):hashlib.sha256(p.read_bytes()).hexdigest()
                      for p in [request_path,b/stats_row['file'],surface_path,scene_path,args.scene_tf,
                                args.robot_poses,identity_path]}}
    report['transport']={'ROS_DOMAIN_ID':'8','ROS_LOCALHOST_ONLY':'1'}
    (args.output/'source.diff').write_bytes(subprocess.check_output(['git','diff']))
    (args.output/'replay_script.py').write_bytes(Path(__file__).read_bytes())
    if hardware_consumer:
        for path in [value for value in consumer_inputs[:3] if value is not None]:
            report['inputs'][str(path.resolve())]=hashlib.sha256(path.read_bytes()).hexdigest()
        report['scope']='captured inputs through hardware consumer and live ROS Select; not hardware deployment F2'
        report['F2']='not_evaluated'
    if args.baseline_consumer:
        report['inputs'][str(args.placement_parameters.resolve())]=hashlib.sha256(args.placement_parameters.read_bytes()).hexdigest()
        report['placement_parameters']=json.loads(args.placement_parameters.read_text())
        report['arm']='baseline_compute_then_select'
    os.environ['ST09_I_CERT_CAPTURE_DIR']=str(args.output/'cert_boundaries')
    os.environ['ST09_PORTAL_CAPTURE_DIR']=str(args.output/'portal_boundaries')
    log=(args.output/'moveit.log').open('w')
    command=['ros2','launch','luggage_planning','isolated_moveit.launch.py',
             'scene_tf_config:='+str(args.scene_tf),'use_rviz:=false','use_gz_sim:=true',
             'hardware_plugin:=gz_ros2_control/GazeboSimSystem','xacro_extra:='+xacro_extra]
    (args.output/'command.json').write_text(json.dumps(command,indent=2))
    proc=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    node=None;executor=None;spin=None;resource_window=None;placement_node=None
    producer=None;work_counters=None
    try:
        rclpy.init();node=rclpy.create_node('exact_route_replay')
        executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
        spin=threading.Thread(target=executor.spin,daemon=True);spin.start()
        def call(client,request,timeout):
            if not client.wait_for_service(timeout_sec=timeout):raise RuntimeError('missing '+client.srv_name)
            future=client.call_async(request);event=threading.Event();future.add_done_callback(lambda f:event.set())
            if not event.wait(timeout):raise RuntimeError('timeout '+client.srv_name)
            return future.result()
        validator=node.create_client(ValidateRouteV2,'/validate_route_v2')
        if not validator.wait_for_service(timeout_sec=30.):raise RuntimeError('validator startup timeout')
        params=node.create_client(GetParameters,'/move_group/get_parameters')
        query=GetParameters.Request()
        query.names=['robot_description','robot_description_semantic','allow_trajectory_execution']
        values=call(params,query,10.).values
        if len(values)!=3 or values[2].type!=1 or values[2].bool_value:
            raise ValueError('execution must be disabled')
        hashes={}
        for index,(name,key) in enumerate([('robot.urdf','urdf_sha256'),('robot.srdf','srdf_sha256')]):
            xml=values[index].string_value
            (args.output/name).write_text(xml)
            hashes[key]=hashlib.sha256(xml.encode()).hexdigest()
        report['model_hashes']=hashes
        if any(value!=captured_identity[key] for key,value in hashes.items()):
            raise ValueError('live model differs from captured runtime identity')
        if hashlib.md5(args.scene_tf.read_bytes()).hexdigest()!=captured_identity['scene_revision']:
            raise ValueError('scene configuration differs from capture')
        apply=node.create_client(ApplyPlanningScene,'/apply_planning_scene')
        message=ApplyPlanningScene.Request();message.scene=scene
        message.scene.is_diff=False
        if not call(apply,message,10.).success:raise RuntimeError('recorded scene restoration refused')
        # Replay clock starts near recorded attachment time and advances with wall
        # time, so production candidate and certification budgets still expire.
        replay_ns=int(request.grasp.identity.stamp_ns)+1_000_000_000
        clock_started=time.monotonic()
        clock=lambda:replay_ns+int((time.monotonic()-clock_started)*1e9)
        original=json.loads((args.capture/('dumps/box_%02d_paths/candidates.json'%args.select_index)).read_text())['rows'][0]
        diag=original.get('atlas') or {}
        if diag.get('loaded'):raise ValueError('this replay requires the captured atlas-unavailable case')
        from luggage_planning.planning_work_counters import PlanningWorkCounters
        work_counters=PlanningWorkCounters()
        producer=build_production_route_producer(node,str(args.scene_tf),lambda:surface,lambda:stats,
            scene=load_scene_tf_config(str(args.scene_tf)),
            scene_tf_hash=hashlib.md5(args.scene_tf.read_bytes()).hexdigest(),
            atlas_cfg={'query_offset':diag.get('query_offset',[])},
            urdf_identity=SimpleNamespace(ready=False,hash=''),
            runtime_atlas=lambda:(None,diag),scene_revision=document['scene_revision'],
            box_count=1,corridor_planner='graph',clock_ns=clock,
            vacuum_state_fn=lambda:{'attached':True,'vacuum_on':True,'received_ns':clock()},
            site_poses_fn=lambda:None,work_counters=work_counters)
        if producer is None or not producer.edge_builder.wait_ready(timeout_sec=30.):
            raise RuntimeError('producer dependencies unavailable')
        if hardware_consumer:
            from luggage_planning.placement_prior_evidence import PriorEvidenceRecorder
            recorder=PriorEvidenceRecorder(args.output/'consumer_boundaries')
            if args.baseline_consumer:
                from baseline_placement import create_baseline_placement
                from luggage_msgs.srv import ComputePlacementV2
                placement_node=create_baseline_placement(args.scene_tf,document['scene_revision'],
                    request.surface,report['placement_parameters'],recorder)
                executor.add_node(placement_node)
                report['effective_placement_parameters']={
                    'solver':placement_node._params,'allowed_yaws':placement_node._allowed_yaws,
                    'hull_margin':placement_node.get_parameter('hull_margin').value,
                    'aperture_margin':placement_node.get_parameter('aperture_margin').value,
                    'atlas_enabled':placement_node._atlas_cfg.get('enabled')}
        if args.measure_resources:
            from luggage_planning.prior_resources import ResourceWindow
            # The producer lives here, and this process owns the MoveIt launch
            # tree. Captured prior hints are data; no prior process is running.
            resource_window=ResourceWindow([os.getpid()],[],interval_s=.05)
            resource_window.__enter__()
        started=time.monotonic()
        if hardware_consumer:
            from rclpy.callback_groups import ReentrantCallbackGroup
            from luggage_msgs.srv import VacuumCommandV2,PreparePlacementPriorV2
            from luggage_planning.hardware_place_replay import replay_place
            from luggage_planning.typed_driver_adapter import TypedDriverAdapter
            received=[]
            def select_callback(wire,reply):
                recorder('select_server:request',wire)
                received.append(wire)
                result=fill_select_response_route(wire,reply,producer,surface_validity=surface)
                recorder('select_server:response',result)
                return result
            service=node.create_service(SelectPlacePathV2,'/trajectory_selector/select_place_path_v2',
                select_callback,callback_group=ReentrantCallbackGroup())
            client=node.create_client(SelectPlacePathV2,'/trajectory_selector/select_place_path_v2',
                callback_group=ReentrantCallbackGroup())
            # This diagnostic supplies transport only. It does not emulate the
            # deployment runtime registry, claim readiness, or authorize motion.
            adapter=object.__new__(TypedDriverAdapter)
            adapter._select=client
            if args.baseline_consumer:
                adapter._compute=node.create_client(ComputePlacementV2,'/placement_planner/compute_placement_v2',
                    callback_group=ReentrantCallbackGroup())
            def transport(port,wire,timeout):
                recorder('hardware_consumer:request',wire)
                # Preserve the server's 60s planning budget. Allow its terminal
                # response to cross ROS after budget exhaustion before teardown.
                grace=5. if port is client else 0.
                if grace:report['diagnostic_select_response_grace_s']=grace
                result=call(port,wire,timeout+grace)
                recorder('hardware_consumer:response',result)
                return result
            adapter._call=transport
            response=replay_place(adapter,request,
                deserialize_message(args.attachment_response.read_bytes(),VacuumCommandV2.Response),
                attachment_request=deserialize_message(args.attachment_request.read_bytes(),VacuumCommandV2.Request),
                scene_revision=document['scene_revision'],
                prior=(deserialize_message(args.prior_response.read_bytes(),PreparePlacementPriorV2.Response)
                       if args.prior_response else None),
                choice_index=args.choice_index)
            report['consumer_request_matches_capture']=len(received)==1 and received[0]==request
            if args.baseline_consumer:
                if len(received)!=1:
                    raise ValueError('baseline did not issue exactly one Select')
                wire=received[0]
                # Baseline recomputes targets and correlation IDs. It must still
                # preserve physical attachment, map, payload and start context.
                fields=('box_wdh','surface','inner_size','geometry_hash','map_revision',
                        'start_joints','container_source_frame','container_target_frame',
                        'container_translation','container_quaternion_xyzw','box_id','generation')
                wire_fields=message_to_ordereddict(wire)
                captured_fields=message_to_ordereddict(request)
                report['baseline_context_preserved']=all(wire_fields[key]==captured_fields[key] for key in fields)
                report['baseline_context_preserved'] &= (wire.grasp.pose==request.grasp.pose
                    and wire.grasp.identity.stamp_ns==request.grasp.identity.stamp_ns
                    and wire.start_tcp.pose==request.start_tcp.pose)
                report['baseline_slot_count']=len(wire.slots)
                if not report['baseline_context_preserved']:
                    raise ValueError('baseline changed captured physical/map context')
            elif not report['consumer_request_matches_capture']:
                raise ValueError('hardware consumer changed captured Select request')
        else:
            response=fill_select_response_route(request,SelectPlacePathV2.Response(),producer,
                surface_validity=surface)
        elapsed=time.monotonic()-started
        if resource_window is not None:
            resource_window.__exit__(None,None,None)
            (args.output/'resources.json').write_text(json.dumps(resource_window.result,indent=2)+'\n')
            resource_window=None
        (args.output/'response.cdr').write_bytes(serialize_message(response))
        (args.output/'rows.json').write_text(response.rows_json or '[]')
        report.update(success=response.success,reason_code=response.reason_code,
                      elapsed_s=elapsed,stage_timings=response.stage_timings_json)
    except Exception as exc:
        import traceback
        report.update(error=str(exc),traceback=traceback.format_exc())
        import faulthandler
        with (args.output/'exception_threads.txt').open('w') as threads:
            faulthandler.dump_traceback(file=threads,all_threads=True)
    finally:
        if producer is not None and work_counters is not None:
            report['work_counters']=work_counters.snapshot(producer.edge_builder)
        if resource_window is not None:
            resource_window.__exit__(None,None,None)
            (args.output/'resources.json').write_text(json.dumps(resource_window.result,indent=2)+'\n')
        (args.output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
        callbacks_stopped=executor.shutdown(timeout_sec=5.) if executor else True
        report['callback_shutdown_complete']=bool(callbacks_stopped)
        if not callbacks_stopped:
            import faulthandler
            with (args.output/'shutdown_threads.txt').open('w') as threads:
                faulthandler.dump_traceback(file=threads,all_threads=True)
        if spin:spin.join(timeout=5.)
        if placement_node:placement_node.destroy_node()
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        subprocess.run(['bash','scripts/stop_sim.sh','--owned-pgid',str(proc.pid)],
            stdout=(args.output/'stop.log').open('w'),stderr=subprocess.STDOUT,check=True)
        proc.wait(timeout=10);log.close()
        residual=dedicated_domain_processes('8')
        (args.output/'domain_after.json').write_text(json.dumps(residual,indent=2))
        if not callbacks_stopped or residual:
            report['success']=False
            report['teardown_incomplete']=True
        (args.output/'result.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k not in ('inputs','traceback')}))
    if not callbacks_stopped:
        # Only this owned diagnostic process is exited, after all external
        # children were stopped and thread/failure artifacts made durable.
        # Python otherwise waits indefinitely on executor worker threads.
        import sys
        sys.stdout.flush();sys.stderr.flush()
        os._exit(1)
    return 0 if report['success'] else 1


if __name__=='__main__':raise SystemExit(main())
