"""Real native whole-chain gate on an isolated model; no deployment claim."""
from dataclasses import replace
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
from types import SimpleNamespace

import pytest

from test_snapshot_joint_path_native import backend as connection_backend

EXECUTABLE = os.environ.get('JOINT_NATIVE_CERTIFIER_EXECUTABLE', '')
pytestmark = pytest.mark.skipif(not EXECUTABLE, reason='explicit native operation-certification gate')


@pytest.fixture(scope='module')
def native_gate():
    import rclpy
    import yaml
    from luggage_msgs.srv import ValidateRouteV2
    from test_snapshot_joint_path_native import robot_fixture
    from luggage_planning.joint_operation.certification import EndpointModel, NativeOperationCertificationPort
    directory = Path(os.environ['JOINT_CERTIFICATION_EVIDENCE_DIR'])
    directory.mkdir(parents=True, exist_ok=False)
    xml, srdf = robot_fixture()
    params = directory/'parameters.yaml'
    params.write_text(yaml.safe_dump({'validate_route_v2': {'ros__parameters': {
        'robot_description': xml, 'robot_description_semantic': srdf}}}))
    log = (directory/'node.log').open('wb')
    process = subprocess.Popen([EXECUTABLE, '--ros-args', '--params-file', str(params)],
        env=dict(os.environ, ROS_DOMAIN_ID='174', ROS_LOCALHOST_ONLY='1'),
        stdout=log, stderr=subprocess.STDOUT)
    context = rclpy.context.Context()
    old_localhost = os.environ.get('ROS_LOCALHOST_ONLY')
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    rclpy.init(context=context, domain_id=174)
    node = rclpy.create_node('operation_certification_gate', context=context)
    if old_localhost is None:
        os.environ.pop('ROS_LOCALHOST_ONLY',None)
    else:
        os.environ['ROS_LOCALHOST_ONLY']=old_localhost
    executor = rclpy.executors.SingleThreadedExecutor(context=context)
    executor.add_node(node)
    worker = threading.Thread(target=executor.spin,daemon=True)
    worker.start()
    records = []
    def record(kind,raw):
        name = '%02d_%s.cdr' % (len(records),kind)
        (directory/name).write_bytes(raw)
        records.append({'kind':kind, 'file':name, 'sha256':hashlib.sha256(raw).hexdigest()})
    model = EndpointModel(xml,srdf,position_tolerance_m=.002,orientation_tolerance_rad=.02)
    port = NativeOperationCertificationPort(node,model,velocity_limits=(.314,)*6,
        acceleration_limits=(.314,)*6,release_drop_m=.05,recorder=record)
    try:
        assert port.client.wait_for_service(timeout_sec=10.) and process.poll() is None
        yield SimpleNamespace(port=port,model=model,directory=directory,records=records,node=node)
    finally:
        (directory/'manifest.json').write_text(json.dumps({'model_sha256':model.model_sha256,
            'scope':'real MoveIt complete-operation certification fixture; no deployment qualification',
            'records':records},indent=2))
        executor.shutdown(timeout_sec=2.)
        worker.join(timeout=2.)
        node.destroy_node()
        context.shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                process.kill();process.wait(timeout=5.)
        log.close()
        manifest=json.loads((directory/'manifest.json').read_text())
        manifest.update(native_exit_code=process.returncode,worker_joined=not worker.is_alive(),
            executable_sha256=hashlib.sha256(Path(EXECUTABLE).read_bytes()).hexdigest())
        (directory/'manifest.json').write_text(json.dumps(manifest,indent=2))
        assert not worker.is_alive(), 'native fixture executor did not stop'
        assert process.returncode == 0, 'native fixture backend did not shut down cleanly'


def operation(model, *, obstacle=False, broad_obstacle_default=False):
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import PlanningScene, CollisionObject
    from shape_msgs.msg import SolidPrimitive
    from rclpy.serialization import serialize_message, deserialize_message
    from scipy.spatial.transform import Rotation
    from luggage_planning.joint_operation.contracts import (
        OperationIdentity, OperationBarrier, JointOperationPlan, OPERATION_JOINTS, seal_operation_plan)
    from luggage_planning.joint_operation.planner import OperationCandidate, timed_operation_leg
    from luggage_planning.joint_operation.scenes import build_operation_scenes
    from luggage_planning.placement_prior.contracts import Snapshot,Transform
    from luggage_planning.planning_contract.attachment_geometry import freeze_geometry
    from luggage_planning.planning_contract.payload_envelope import EnvelopeLimits
    q = [(i*.01,0.,0.,0.,0.,0.) for i in range(7)]
    poses=[]
    for row in q:
        tf=model.urdf.tcp(row,model.path)
        poses.append(Transform(tuple(tf[:3,3]),tuple(Rotation.from_matrix(tf[:3,:3]).as_quat())))
    geometry=freeze_geometry((.2,.15,.1),(0.,0.,-.05),(0.,0.,0.,1.),
        EnvelopeLimits(0.,(0.,0.,0.),(0.,0.),0.))
    grasp=Transform(geometry.t_tool_box_translation,geometry.t_tool_box_quaternion_xyzw)
    pick,release=poses[2].compose(grasp),poses[5].compose(grasp)
    settled=Transform((release.translation[0],release.translation[1],release.translation[2]-.05),
                      release.quaternion_xyzw)
    scene=PlanningScene()
    scene.robot_model_name='connection_fixture'
    scene.robot_state.joint_state.name=list(OPERATION_JOINTS)
    scene.robot_state.joint_state.position=list(q[0])
    source=Snapshot('scene',7,'hull',model.model_sha256,Transform((0.,0.,0.),(0.,0.,0.,1.)),
        b'fixture-observation',serialize_message(scene))
    phases=build_operation_scenes(source,pickup_object_id='pickup_box',geometry=geometry,
        world_from_pick_box=pick,world_from_settled_box=settled)
    if obstacle:
        # This obstacle exists in retreat only: a carrying-scene shortcut would
        # incorrectly certify it. Do not alter the pinned base or live owner.
        retreat=deserialize_message(phases.retreat.scene_cdr,PlanningScene)
        obj=CollisionObject()
        obj.id,obj.header.frame_id,obj.operation='exit_block','world',CollisionObject.ADD
        pose=Pose()
        pose.position.x,pose.position.y,pose.position.z=poses[6].translation
        pose.orientation.w=1.
        obj.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.08]*3)]
        obj.primitive_poses=[pose]
        retreat.world.collision_objects.append(obj)
        if broad_obstacle_default:
            retreat.allowed_collision_matrix.default_entry_names.append('exit_block')
            retreat.allowed_collision_matrix.default_entry_values.append(True)
        raw=serialize_message(retreat)
        phases=replace(phases,retreat=replace(phases.retreat,scene_cdr=raw,
            scene_content_hash=hashlib.sha256(raw).hexdigest()))
    identity=OperationIdentity('native-full','scene',hashlib.sha256(repr(source.identity).encode()).hexdigest(),
        'hull',model.model_sha256,7,'observed-box',12,15,'box',0,'contact')
    candidate=OperationCandidate('native-candidate',pick,poses[2],release,poses[5],settled,
        geometry,phases,poses[1],poses[3],poses[4],poses[6])
    names=('align','pick_contact','pick_lift','carry','place_release','place_retreat')
    leg_phases=('pick','pick','carry','carry','carry','retreat')
    legs=tuple(timed_operation_leg(name,phase,(q[i],q[i+1]),identity,.2,.1)
               for i,(name,phase) in enumerate(zip(names,leg_phases)))
    draft=seal_operation_plan(JointOperationPlan(identity,candidate.candidate_id,'pending-native-certification',
        pick,poses[2],release,poses[5],settled,geometry,q[0],legs,
        (OperationBarrier(2,'attachment_confirmed'),OperationBarrier(5,'release_confirmed')),''))
    return draft,candidate


def test_whole_chain_native_certifies_all_original_legs_and_private_load_states(native_gate):
    from luggage_msgs.srv import ValidateRouteV2
    from rclpy.serialization import deserialize_message
    draft,candidate=operation(native_gate.model)
    result=native_gate.port.certify(draft,candidate,deadline=time.monotonic()+5.,cancelled=lambda:False)
    assert result.passed, result.reason
    assert len(result.certificate_id)==64
    request=deserialize_message((native_gate.directory/native_gate.records[-2]['file']).read_bytes(),ValidateRouteV2.Request)
    assert [leg.load_state_index for leg in request.legs]==[0,0,1,1,1,2]
    assert all(not leg.support_payload_id and not leg.spatial_check_link for leg in request.legs)
    assert tuple(request.legs[-1].knots.points[-1].positions)==draft.legs[-1].knots[-1].positions
    assert request.legs[-1].n_samples>len(draft.legs[-1].knots)


def test_released_world_obstacle_blocks_retreat_in_real_native_gate(native_gate):
    from luggage_msgs.srv import ValidateRouteV2
    from rclpy.serialization import deserialize_message
    draft,candidate=operation(native_gate.model,obstacle=True)
    result=native_gate.port.certify(draft,candidate,deadline=time.monotonic()+5.,cancelled=lambda:False)
    assert not result.passed and not result.certificate_id
    assert result.reason.startswith('NATIVE_CERTIFICATION_REFUSED')
    response=deserialize_message((native_gate.directory/native_gate.records[-1]['file']).read_bytes(),ValidateRouteV2.Response)
    assert all(leg.valid for leg in response.leg_results[:-1])
    assert response.leg_results[-1].edge_id=='place_retreat'
    assert response.leg_results[-1].reason=='COLLISION'


def test_wrong_pick_endpoint_fk_refuses_before_native_request(native_gate):
    from luggage_planning.joint_operation.contracts import seal_operation_plan
    draft,candidate=operation(native_gate.model)
    # Joint-continuous path but a wrong claimed spatial grasp: same G remains
    # valid, so structure/digest alone would accept it.
    from luggage_planning.placement_prior.contracts import Transform
    tool=Transform((4.,4.,4.),draft.world_from_pick_tool.quaternion_xyzw)
    grasp=Transform(draft.predicted_attachment.t_tool_box_translation,
                    draft.predicted_attachment.t_tool_box_quaternion_xyzw)
    draft=seal_operation_plan(replace(draft,world_from_pick_tool=tool,world_from_pick_box=tool.compose(grasp)))
    candidate=replace(candidate,world_from_pick_tool=tool,world_from_pick_box=tool.compose(grasp))
    before=len(native_gate.records)
    result=native_gate.port.certify(draft,candidate,deadline=time.monotonic()+5.,cancelled=lambda:False)
    assert result.reason=='ENDPOINT_FK_MISMATCH' and len(native_gate.records)==before


def test_cancelled_preparation_has_no_backend_submission(native_gate):
    draft,candidate=operation(native_gate.model)
    before=len(native_gate.records)
    result=native_gate.port.certify(draft,candidate,deadline=time.monotonic()+5.,cancelled=lambda:True)
    assert not result.passed and len(native_gate.records)==before


def test_settled_target_must_follow_explicit_container_up_release_policy(native_gate):
    from luggage_planning.placement_prior.contracts import Transform
    from luggage_planning.joint_operation.contracts import seal_operation_plan
    draft,candidate=operation(native_gate.model)
    original=draft.world_from_settled_box
    shifted=Transform((original.translation[0]+.02,*original.translation[1:]),original.quaternion_xyzw)
    draft=seal_operation_plan(replace(draft,world_from_settled_box=shifted))
    candidate=replace(candidate,world_from_settled_box=shifted)
    before=len(native_gate.records)
    result=native_gate.port.certify(draft,candidate,deadline=time.monotonic()+5.,cancelled=lambda:False)
    assert result.reason=='RELEASE_SETTLED_POLICY_MISMATCH' and len(native_gate.records)==before


@pytest.mark.parametrize('kind',['missing_attachment','wrong_dimensions','broad_payload_acm'])
def test_phase_label_cannot_substitute_for_actual_frozen_attachment(native_gate,kind):
    from moveit_msgs.msg import PlanningScene
    from rclpy.serialization import serialize_message,deserialize_message
    draft,candidate=operation(native_gate.model)
    carry=deserialize_message(candidate.scenes.carry.scene_cdr,PlanningScene)
    if kind=='missing_attachment':
        carry.robot_state.attached_collision_objects=[]
    elif kind=='wrong_dimensions':
        carry.robot_state.attached_collision_objects[0].object.primitives[0].dimensions=[.01]*3
    else:
        matrix=carry.allowed_collision_matrix
        matrix.default_entry_values[matrix.default_entry_names.index('pickup_box')]=True
    raw=serialize_message(carry)
    phases=replace(candidate.scenes,carry=replace(candidate.scenes.carry,scene_cdr=raw,
        scene_content_hash=hashlib.sha256(raw).hexdigest()))
    candidate=replace(candidate,scenes=phases)
    before=len(native_gate.records)
    result=native_gate.port.certify(draft,candidate,deadline=time.monotonic()+5.,cancelled=lambda:False)
    assert result.reason.startswith('PHASE_GEOMETRY_MISMATCH') and len(native_gate.records)==before


def test_unknown_world_object_default_cannot_hide_payload_collision(native_gate):
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject,PlanningScene
    from shape_msgs.msg import SolidPrimitive
    from luggage_msgs.srv import ValidateRouteV2
    from rclpy.serialization import serialize_message,deserialize_message
    draft,candidate=operation(native_gate.model)
    carry=deserialize_message(candidate.scenes.carry.scene_cdr,PlanningScene)
    obj=CollisionObject()
    obj.id,obj.header.frame_id,obj.operation='unknown_obstacle','world',CollisionObject.ADD
    pose=Pose()
    xyz=native_gate.model.urdf.tcp(draft.legs[2].knots[0].positions,native_gate.model.path)[:3,3]
    pose.position.x,pose.position.y,pose.position.z=map(float,xyz)
    pose.position.z-=.075  # inside the carried box, clear of tool sphere
    pose.orientation.w=1.
    obj.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.005]*3)]
    obj.primitive_poses=[pose]
    carry.world.collision_objects.append(obj)
    carry.allowed_collision_matrix.default_entry_names.append(obj.id)
    carry.allowed_collision_matrix.default_entry_values.append(True)
    raw=serialize_message(carry)
    scenes=replace(candidate.scenes,carry=replace(candidate.scenes.carry,scene_cdr=raw,
        scene_content_hash=hashlib.sha256(raw).hexdigest()))
    candidate=replace(candidate,scenes=scenes)
    result=native_gate.port.certify(draft,candidate,deadline=time.monotonic()+5.,cancelled=lambda:False)
    assert not result.passed and not result.certificate_id
    response=deserialize_message((native_gate.directory/native_gate.records[-1]['file']).read_bytes(),ValidateRouteV2.Response)
    assert response.leg_results[2].reason=='COLLISION'
    assert any({'pickup_box','unknown_obstacle'}=={c.body_1,c.body_2} for c in response.leg_results[2].contacts)


@pytest.mark.skipif(not os.environ.get('JOINT_NATIVE_CONNECTOR_EXECUTABLE'),
                    reason='combined gate also requires real native connector')
def test_actual_start_and_carry_connections_assemble_then_native_certify(native_gate,connection_backend):
    from rclpy.serialization import serialize_message
    from luggage_planning.joint_operation.connection import NativeSnapshotConnectionPort
    from luggage_planning.joint_operation.planner import JointOperationPlanner,LocalEndpointEvidence,LocalJointBranch
    from luggage_planning.joint_operation.contracts import admit_operation_plan
    draft,candidate=operation(native_gate.model)
    evidence=LocalEndpointEvidence(candidate.candidate_id,native_gate.model.model_sha256,
        candidate.scenes.digests,candidate.predicted_attachment.digest(),True,'FK-checked fixture endpoints',
        (LocalJointBranch('pick',tuple(k.positions for k in draft.legs[1].knots),
                          tuple(k.positions for k in draft.legs[2].knots)),),
        (LocalJointBranch('place',tuple(k.positions for k in draft.legs[4].knots),
                          tuple(k.positions for k in draft.legs[5].knots)),))
    endpoint_port=SimpleNamespace(solve=lambda *args,**kwargs:evidence)
    def record(kind,message):
        name='connection_%s_%02d.cdr'%(kind,len(list(native_gate.directory.glob('connection_*'))))
        (native_gate.directory/name).write_bytes(serialize_message(message))
    connection=NativeSnapshotConnectionPort(connection_backend.node,recorder=record)
    planner=JointOperationPlanner(endpoint_port,connection,native_gate.port,velocity_scale=.2,
        acceleration_scale=.1,certification_reserve_s=2.)
    stop=threading.Event()
    def spin():
        while not stop.is_set():
            connection_backend.executor.spin_once(timeout_sec=.01)
    worker=threading.Thread(target=spin,daemon=True)
    worker.start()
    try:
        result=planner.prepare(draft.identity,draft.start_joints,(candidate,),budget_s=8.)
        assert result.status=='PLAN_READY',result.trace
        admit_operation_plan(result.plan,draft.identity,result.plan.plan_digest)
        assert result.plan.legs[0].knots[0].positions==draft.start_joints
        assert result.plan.legs[3].knots[-1].positions==draft.legs[4].knots[0].positions
        assert len(list(native_gate.directory.glob('connection_*')))==4
        (native_gate.directory/'assembled_plan_summary.json').write_text(json.dumps({
            'status':result.status,'trace':result.trace,'elapsed_s':result.elapsed_s,
            'plan_digest':result.plan.plan_digest,'certificate_id':result.plan.certificate_id,
            'phase_scene_digests':candidate.scenes.digests},indent=2))
    finally:
        stop.set();worker.join(timeout=2.)
