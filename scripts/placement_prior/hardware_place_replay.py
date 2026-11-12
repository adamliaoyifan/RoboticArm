#!/usr/bin/env python3
"""Recorded-input hardware place caller; no motion, vacuum or scene mutations.

Use an already isolated hardware plan-only graph with matching captured identity,
scene, OCC and state. This tool neither creates them nor relabels their provenance.
No-prior runs recompute placement via the production consumer. A supplied choice
keeps its bound target. Exact outbound requests and responses are recorded.
"""
import argparse
import hashlib
import json
from pathlib import Path
import threading
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--select-request',type=Path,required=True)
    parser.add_argument('--attachment-request',type=Path,required=True)
    parser.add_argument('--attachment-response',type=Path,required=True)
    parser.add_argument('--runtime-identity',type=Path,required=True,
                        help='Exact captured runtime identity JSON (model/calibration/graph included)')
    parser.add_argument('--prior-response',type=Path)
    parser.add_argument('--choice-index',type=int)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.choice_index is not None and args.prior_response is None:
        parser.error('--choice-index requires --prior-response')
    args.output.mkdir(parents=True,exist_ok=False)
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.serialization import deserialize_message
    from rcl_interfaces.srv import GetParameters
    from moveit_msgs.srv import GetPlanningScene
    from moveit_msgs.msg import PlanningSceneComponents
    from luggage_msgs.srv import SelectPlacePathV2, VacuumCommandV2, PreparePlacementPriorV2
    from luggage_planning.typed_driver_adapter import TypedDriverAdapter, adopt_runtime_identity
    from luggage_planning.placement_prior.evidence import PriorEvidenceRecorder
    from luggage_planning.hardware_place_replay import replay_place
    report={'completed':False,'F2':'not_evaluated','inputs':{}}
    def read(path, message_type):
        data=path.read_bytes()
        report['inputs'][str(path.resolve())]=hashlib.sha256(data).hexdigest()
        return deserialize_message(data,message_type)
    request=read(args.select_request,SelectPlacePathV2.Request)
    attachment=read(args.attachment_response,VacuumCommandV2.Response)
    attach_request=read(args.attachment_request,VacuumCommandV2.Request)
    prior=read(args.prior_response,PreparePlacementPriorV2.Response) if args.prior_response else None
    identity_bytes=args.runtime_identity.read_bytes()
    expected_identity=json.loads(identity_bytes)
    report['inputs'][str(args.runtime_identity.resolve())]=hashlib.sha256(identity_bytes).hexdigest()
    rclpy.init();node=rclpy.create_node('hardware_pick_driver')
    executor=None;spin=None
    try:
        runtime,_=adopt_runtime_identity(node,peer='hardware_pick_driver')
        report['identity']=runtime.identity
        if runtime.identity!=expected_identity:
            raise ValueError('captured model/calibration/runtime identity does not match live graph')
        adapter=TypedDriverAdapter(node,runtime)
        adapter.set_boundary_recorder(PriorEvidenceRecorder(args.output/'boundaries'))
        executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node)
        spin=threading.Thread(target=executor.spin,daemon=True);spin.start()
        # Query actual live configuration before sending any planning request.
        client=node.create_client(GetParameters,'/move_group/get_parameters')
        if not client.wait_for_service(timeout_sec=5.):
            raise RuntimeError('MoveIt parameter service unavailable')
        query=GetParameters.Request();query.names=['allow_trajectory_execution']
        future=client.call_async(query);ready=threading.Event()
        future.add_done_callback(lambda unused:ready.set())
        if not ready.wait(5.):raise RuntimeError('execution gate query timeout')
        values=future.result().values
        if len(values)!=1 or values[0].type!=1 or values[0].bool_value:
            raise RuntimeError('hardware replay requires MoveIt execution disabled')
        deadline=time.monotonic()+10.
        while True:
            try:
                runtime.require_graph_ready();break
            except Exception:
                if time.monotonic()>=deadline:raise
                time.sleep(.05)
        scene_client=node.create_client(GetPlanningScene,'/get_planning_scene')
        scene_request=GetPlanningScene.Request()
        scene_request.components.components=(PlanningSceneComponents.SCENE_SETTINGS
            | PlanningSceneComponents.ROBOT_STATE | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
            | PlanningSceneComponents.WORLD_OBJECT_NAMES | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.OCTOMAP | PlanningSceneComponents.TRANSFORMS
            | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            | PlanningSceneComponents.LINK_PADDING_AND_SCALING | PlanningSceneComponents.OBJECT_COLORS)
        adapter._call(scene_client,scene_request,5.)
        started=time.monotonic()
        result=replay_place(adapter,request,attachment,
            attachment_request=attach_request,
            scene_revision=runtime.identity['scene_revision'],prior=prior,
            choice_index=args.choice_index)
        report.update(completed=True,success=bool(result and result.success),
                      elapsed_s=time.monotonic()-started,
                      reason_code=getattr(result,'reason_code','NO_RESPONSE'))
    except Exception as exc:
        report['error']=str(exc)
    finally:
        (args.output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
        if executor:executor.shutdown(timeout_sec=5.)
        if spin:spin.join(timeout=5.)
        node.destroy_node();rclpy.shutdown()
    return 0 if report.get('success') else 1


if __name__=='__main__':
    raise SystemExit(main())
