"""Actual native suffix checks; fixtures do not certify a deployment/controller."""
from dataclasses import asdict, replace
from pathlib import Path
import hashlib
import json
import os
import time
from concurrent.futures import Future

import pytest

from test_joint_operation_certification_native import native_gate, operation
from luggage_planning.joint_operation_contracts import seal_operation_plan
from luggage_planning.joint_operation_revalidation import NativeOperationRevalidationPort
from luggage_planning.joint_operation_scenes import build_operation_scenes
from luggage_planning.placement_prior_contracts import Transform
from st09_contracts.attachment_geometry import freeze_geometry
from st09_contracts.payload_envelope import EnvelopeLimits

pytestmark = pytest.mark.skipif(not os.environ.get('JOINT_NATIVE_CERTIFIER_EXECUTABLE'),
    reason='explicit real native revalidation gate')


@pytest.fixture(scope='module')
def issued(native_gate):
    draft,candidate=operation(native_gate.model)
    receipt=native_gate.port.certify(draft,candidate,deadline=time.monotonic()+5.,cancelled=lambda:False)
    assert receipt.passed
    plan=seal_operation_plan(replace(draft,certificate_id=receipt.certificate_id,plan_digest=''))
    return plan,candidate


def port(gate,**changes):
    settings=dict(velocity_limits=(.314,)*6,acceleration_limits=(.314,)*6,
        release_drop_m=.05,start_tolerance_rad=.01,recorder=gate.port.recorder)
    settings.update(changes)
    backend=NativeOperationRevalidationPort(gate.node,gate.model,**settings)
    original=backend.validate
    def captured(plan,**arguments):
        from rclpy.serialization import serialize_message
        from luggage_planning.joint_operation_adapters import operation_plan_to_message
        from luggage_planning.attachment_geometry_transport import geometry_to_message
        name='boundary_'+str(time.monotonic_ns())
        (gate.directory/(name+'_plan.cdr')).write_bytes(serialize_message(
            operation_plan_to_message(plan,plan.identity,plan.plan_digest)))
        (gate.directory/(name+'_scene.cdr')).write_bytes(arguments['snapshot'].scene_cdr)
        if arguments['geometry'] is not None:
            (gate.directory/(name+'_geometry.cdr')).write_bytes(serialize_message(
                geometry_to_message(arguments['geometry'])))
        result=original(plan,**arguments)
        (gate.directory/(name+'_result.json')).write_text(json.dumps({
            'test':os.environ.get('PYTEST_CURRENT_TEST'),
            'actual_joints':arguments['actual_joints'],'next_leg_index':arguments['next_leg_index'],
            'result':asdict(result)},indent=2))
        return result
    backend.validate=captured
    return backend


def context(gate,issued,index=2,geometry=None):
    from moveit_msgs.msg import PlanningScene
    from rclpy.serialization import deserialize_message,serialize_message
    plan,candidate=issued
    geometry=geometry or plan.predicted_attachment
    # All native boundaries are explicit fixture data, not live observed OCC.
    baseline=candidate.scenes.pick
    scene=deserialize_message(baseline.scene_cdr,PlanningScene)
    scene.world.collision_objects=[]
    baseline=replace(baseline,scene_cdr=serialize_message(scene))
    phases=build_operation_scenes(baseline,pickup_object_id='pickup_box',geometry=geometry,
        world_from_pick_box=plan.world_from_pick_box,world_from_settled_box=plan.world_from_settled_box)
    snapshot=phases.for_phase(plan.legs[index].phase)
    scene=deserialize_message(snapshot.scene_cdr,PlanningScene)
    scene.robot_state.joint_state.position=list(plan.legs[index].knots[0].positions)
    raw=serialize_message(scene)
    snapshot=replace(snapshot,scene_cdr=raw,scene_content_hash=hashlib.sha256(raw).hexdigest())
    return dict(expected_identity=plan.identity,expected_digest=plan.plan_digest,snapshot=snapshot,
        pickup_object_id='pickup_box',next_leg_index=index,actual_joints=plan.legs[index].knots[0].positions,
        geometry=None if index<2 else geometry,certificate_current=lambda p:p==plan,
        source_current=lambda snapshot,geometry:True,
        admit_actual_contact=lambda *args,**kwargs:True,
        admit_settled_box=lambda *args,**kwargs:True,
        deadline=time.monotonic()+5.,cancelled=lambda:False)


def run(gate,issued,index=2,geometry=None,**changes):
    plan,_=issued
    arguments=context(gate,issued,index,geometry)
    arguments.update(changes)
    # Exact boundary inputs survive refusals that happen before a native RPC.
    seq=len(gate.records)
    saved=gate.directory/('admission_%03d_%d.json'%(seq,time.monotonic_ns()))
    saved.write_text(json.dumps({'plan_digest':plan.plan_digest,'index':arguments['next_leg_index'],
        'actual_joints':arguments['actual_joints'],'geometry':None if arguments['geometry'] is None else arguments['geometry'].digest(),
        'snapshot_scene_sha256':hashlib.sha256(arguments['snapshot'].scene_cdr).hexdigest()},indent=2))
    return port(gate).validate(plan,**arguments)


@pytest.mark.parametrize('index',[0,1,2,3,4,5])
def test_each_current_phase_checks_original_suffix_without_planning(native_gate,issued,index):
    from luggage_msgs.srv import ValidateRouteV2
    from rclpy.serialization import deserialize_message
    plan,_=issued;before=len(native_gate.records)
    result=run(native_gate,issued,index)
    assert result.receipt.accepted,result.receipt.reason
    assert len(result.native_validation_id)==64
    assert result.receipt.plan_digest==plan.plan_digest
    assert result.receipt.attachment_geometry_digest==(plan.predicted_attachment.digest() if 2<=index<5 else '')
    assert len(native_gate.records)==before+2
    request=deserialize_message((native_gate.directory/native_gate.records[-2]['file']).read_bytes(),ValidateRouteV2.Request)
    assert [leg.edge_id for leg in request.legs]==[leg.name for leg in plan.legs[index:]]
    for message,original in zip(request.legs,plan.legs[index:]):
        assert [tuple(p.positions) for p in message.knots.points]==[k.positions for k in original.knots]
        assert [p.time_from_start.sec*10**9+p.time_from_start.nanosec for p in message.knots.points]==[k.time_from_start_ns for k in original.knots]
        assert all(tuple(p.velocities)==k.velocities and tuple(p.accelerations)==k.accelerations
                   for p,k in zip(message.knots.points,original.knots))


def test_compatible_actual_envelope_change_reuses_original_suffix(native_gate,issued):
    original=issued[0].predicted_attachment
    actual=freeze_geometry(original.measured_dimensions_wdh,original.t_tool_box_translation,
        original.t_tool_box_quaternion_xyzw,EnvelopeLimits(0.,(.001,.001,.001),(0.,0.),0.))
    assert actual.digest()!=original.digest()
    result=run(native_gate,issued,geometry=actual)
    assert result.receipt.accepted,result.receipt.reason
    assert result.receipt.attachment_geometry_digest==actual.digest()


def test_changed_actual_grasp_cannot_silently_move_bound_box_target(native_gate,issued):
    original=issued[0].predicted_attachment
    actual=freeze_geometry(original.measured_dimensions_wdh,(.02,0.,-.05),
        original.t_tool_box_quaternion_xyzw,EnvelopeLimits(0.,(0.,0.,0.),(0.,0.),0.))
    before=len(native_gate.records);result=run(native_gate,issued,geometry=actual)
    assert not result.receipt.accepted
    assert result.receipt.reason=='ACTUAL_GRASP_MISSES_BOUND_BOX_TARGET'
    assert len(native_gate.records)==before


@pytest.mark.parametrize('kind',['map','model','digest','start','geometry_missing',
    'certificate','certificate_truthy','source','source_truthy','slot','slot_truthy','cancelled','deadline'])
def test_identity_freshness_and_admission_fail_before_native(native_gate,issued,kind):
    args=context(native_gate,issued)
    if kind=='map':args['snapshot']=replace(args['snapshot'],map_revision=8)
    elif kind=='model':args['snapshot']=replace(args['snapshot'],model_hash='changed')
    elif kind=='digest':args['expected_digest']='bad'
    elif kind=='start':args['actual_joints']=(.5,0.,0.,0.,0.,0.)
    elif kind=='geometry_missing':args['geometry']=None
    elif kind=='certificate':args['certificate_current']=lambda p:False
    elif kind=='certificate_truthy':args['certificate_current']=lambda p:object()
    elif kind=='source':args['source_current']=lambda *a:False
    elif kind=='source_truthy':args['source_current']=lambda *a:object()
    elif kind=='slot':args['admit_settled_box']=lambda *a,**k:False
    elif kind=='slot_truthy':args['admit_settled_box']=lambda *a,**k:object()
    elif kind=='cancelled':args['cancelled']=lambda:True
    elif kind=='deadline':args['deadline']=time.monotonic()-1.
    before=len(native_gate.records);result=port(native_gate).validate(issued[0],**args)
    assert not result.receipt.accepted
    assert not result.native_validation_id
    assert len(native_gate.records)==before


def changed_scene(arguments,modify):
    from moveit_msgs.msg import PlanningScene
    from rclpy.serialization import deserialize_message,serialize_message
    scene=deserialize_message(arguments['snapshot'].scene_cdr,PlanningScene);modify(scene)
    raw=serialize_message(scene)
    return replace(arguments['snapshot'],scene_cdr=raw,scene_content_hash=hashlib.sha256(raw).hexdigest())


@pytest.mark.parametrize('kind',['missing_attachment','wrong_attachment','extra_attachment','diff','scene_q'])
def test_live_payload_and_robot_state_must_match_actual_provider(native_gate,issued,kind):
    import copy
    args=context(native_gate,issued)
    def modify(scene):
        if kind=='missing_attachment':scene.robot_state.attached_collision_objects=[]
        elif kind=='wrong_attachment':scene.robot_state.attached_collision_objects[0].object.primitives[0].dimensions=[.01]*3
        elif kind=='extra_attachment':
            second=copy.deepcopy(scene.robot_state.attached_collision_objects[0]);second.object.id='unrelated';scene.robot_state.attached_collision_objects.append(second)
        elif kind=='diff':scene.is_diff=True
        else:scene.robot_state.joint_state.position=[1.]*6
    args['snapshot']=changed_scene(args,modify)
    before=len(native_gate.records);result=port(native_gate).validate(issued[0],**args)
    assert not result.receipt.accepted,result
    assert len(native_gate.records)==before


def add_obstacle(scene,xyz):
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import SolidPrimitive
    pose=Pose();pose.position.x,pose.position.y,pose.position.z=xyz;pose.orientation.w=1.
    obj=CollisionObject();obj.id='new_obstacle';obj.header.frame_id='world'
    obj.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.03]*3)];obj.primitive_poses=[pose]
    scene.world.collision_objects.append(obj)


@pytest.mark.parametrize('index',[2,5])
def test_new_carry_or_released_retreat_obstacle_refuses_in_native_gate(native_gate,issued,index):
    plan,_=issued;args=context(native_gate,issued,index)
    if index==2:
        tool=Transform.from_matrix(native_gate.model.urdf.tcp(plan.legs[4].knots[-1].positions,native_gate.model.path))
        g=plan.predicted_attachment
        xyz=tool.compose(Transform(g.t_tool_box_translation,g.t_tool_box_quaternion_xyzw)).translation
    else:
        xyz=Transform.from_matrix(native_gate.model.urdf.tcp(plan.legs[-1].knots[-1].positions,native_gate.model.path)).translation
    args['snapshot']=changed_scene(args,lambda scene:add_obstacle(scene,xyz))
    before=len(native_gate.records);result=port(native_gate).validate(plan,**args)
    assert not result.receipt.accepted,result
    assert 'NATIVE_CERTIFICATION_REFUSED' in result.receipt.reason
    assert len(native_gate.records)==before+2


def test_source_change_after_native_response_discards_positive(native_gate,issued):
    calls=[]
    def current(*args):calls.append(args);return len(calls)==1
    result=run(native_gate,issued,source_current=current)
    assert not result.receipt.accepted
    assert result.receipt.reason=='CERTIFICATE_OR_SOURCE_CHANGED_DURING_REVALIDATION'
    assert len(calls)==2


def test_late_future_remains_owned_and_prevents_overlapping_native_submission(native_gate,issued):
    backend=port(native_gate);backend.inflight=Future()
    before=len(native_gate.records)
    result=backend.validate(issued[0],**context(native_gate,issued))
    assert result.receipt.reason=='REVALIDATION_BACKEND_BUSY'
    assert len(native_gate.records)==before


def attachment_record(plan,geometry):
    from test_joint_operation_execution import confirmed_record
    return replace(confirmed_record(plan,geometry),request_id=plan.identity.request_id,
                   box_id=plan.identity.box_id,scene_revision=plan.identity.scene_revision)


def test_real_native_receipts_drive_complete_consumer_and_reuse_original_knots(native_gate,issued):
    from luggage_planning.joint_operation_execution import JointOperationExecution
    plan,_=issued
    engine=JointOperationExecution(start_tolerance_rad=.01)
    command=engine.start(plan,plan.identity,plan.plan_digest)
    geometry=plan.predicted_attachment
    executed=[];validated=[]
    while command is not None:
        if command.kind in ('validate_start','validate_attached_suffix','apply_release_and_validate_retreat'):
            outcome=run(native_gate,issued,command.leg_index)
            assert outcome.receipt.accepted,outcome.receipt.reason
            validated.append(command.leg_index)
            command=engine.validation_finished(command.token,outcome.receipt)
        elif command.kind=='execute_leg':
            # Controller boundary fixture: no physical execution claimed. A
            # real adapter must make this same per-leg admission before submit.
            if command.leg_index not in validated:
                outcome=run(native_gate,issued,command.leg_index)
                assert outcome.receipt.accepted,outcome.receipt.reason
                validated.append(command.leg_index)
            executed.append(engine.pending_leg(command.token))
            command=engine.motion_finished(command.token,success=True)
        elif command.kind=='confirm_attachment':
            command=engine.attachment_confirmed(command.token,record=attachment_record(plan,geometry),
                geometry=geometry,generation=plan.identity.generation)
        elif command.kind=='confirm_release':
            command=engine.release_confirmed(command.token,success=True)
        else:raise AssertionError(command.kind)
    assert engine.state=='COMPLETE'
    assert validated==list(range(6))
    assert all(a is b for a,b in zip(executed,plan.legs))


def test_real_native_carry_refusal_faults_consumer_without_release(native_gate,issued):
    from luggage_planning.joint_operation_execution import JointOperationExecution
    plan,_=issued;engine=JointOperationExecution(start_tolerance_rad=.01)
    command=engine.start(plan,plan.identity,plan.plan_digest)
    command=engine.validation_finished(command.token,run(native_gate,issued,0).receipt)
    command=engine.motion_finished(command.token,success=True)
    command=engine.motion_finished(command.token,success=True)
    assert command.kind=='confirm_attachment'
    geometry=plan.predicted_attachment
    command=engine.attachment_confirmed(command.token,record=attachment_record(plan,geometry),
        geometry=geometry,generation=plan.identity.generation)
    args=context(native_gate,issued,2)
    tool=Transform.from_matrix(native_gate.model.urdf.tcp(plan.legs[4].knots[-1].positions,native_gate.model.path))
    xyz=tool.compose(Transform(geometry.t_tool_box_translation,geometry.t_tool_box_quaternion_xyzw)).translation
    args['snapshot']=changed_scene(args,lambda scene:add_obstacle(scene,xyz))
    outcome=port(native_gate).validate(plan,**args)
    assert not outcome.receipt.accepted
    assert engine.validation_finished(command.token,outcome.receipt) is None
    assert engine.state=='CARRY_FAULT' and engine.may_have_payload


def test_canonical_named_chain_cannot_move_attachment_barrier_after_pick_lift(native_gate,issued):
    from luggage_planning.joint_operation_contracts import OperationBarrier
    plan,candidate=issued
    legs=list(plan.legs);legs[2]=replace(legs[2],phase='pick')
    wrong=seal_operation_plan(replace(plan,legs=tuple(legs),
        barriers=(OperationBarrier(3,'attachment_confirmed'),OperationBarrier(5,'release_confirmed')),
        plan_digest=''))
    before=len(native_gate.records)
    prepared=native_gate.port.certify(wrong,candidate,deadline=time.monotonic()+5.,cancelled=lambda:False)
    assert not prepared.passed and prepared.reason=='UNSUPPORTED_COMPLETE_CHAIN'
    args=context(native_gate,issued,2);args.update(expected_identity=wrong.identity,expected_digest=wrong.plan_digest)
    result=port(native_gate).validate(wrong,**args)
    assert not result.receipt.accepted and result.receipt.reason=='UNSUPPORTED_COMPLETE_CHAIN'
    assert len(native_gate.records)==before


@pytest.mark.parametrize('kind',['capture','service','initial_source','final_source','late_slot'])
def test_failed_boundary_never_grants_execution_receipt(native_gate,issued,kind):
    backend=port(native_gate);args=context(native_gate,issued)
    def fail(*a,**kw):raise RuntimeError('injected boundary failure')
    if kind=='capture':backend.recorder=fail
    elif kind=='service':
        class BrokenClient:
            def service_is_ready(self):return True
            call_async=staticmethod(fail)
        backend.client=BrokenClient()
    elif kind=='initial_source':args['source_current']=fail
    elif kind=='final_source':
        calls=[]
        def current(*a):
            calls.append(a)
            if len(calls)>1:raise RuntimeError('source read failed after native')
            return True
        args['source_current']=current
    else:
        args['deadline']=time.monotonic()+.01
        def late(*a,**kw):time.sleep(.02);return True
        args['admit_settled_box']=late
    result=backend.validate(issued[0],**args)
    assert not result.receipt.accepted and not result.native_validation_id


def test_sample_cap_prevents_unbounded_controller_sampling(native_gate,issued):
    backend=port(native_gate,max_samples=12)
    before=len(native_gate.records)
    result=backend.validate(issued[0],**context(native_gate,issued,0))
    assert result.receipt.reason=='REVALIDATION_SAMPLE_CAP'
    assert len(native_gate.records)==before


@pytest.mark.parametrize('answer',[False,object(),'exception'])
def test_actual_contact_requires_separate_whole_footprint_admission(native_gate,issued,answer):
    args=context(native_gate,issued)
    def contact(*a,**kw):
        if answer=='exception':raise RuntimeError('actual footprint unavailable')
        return answer
    args['admit_actual_contact']=contact
    before=len(native_gate.records)
    result=port(native_gate).validate(issued[0],**args)
    assert not result.receipt.accepted and not result.native_validation_id
    assert len(native_gate.records)==before
