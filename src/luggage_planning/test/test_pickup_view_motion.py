"""Camera mount/scene joins and plan-only motion failure boundaries."""
from dataclasses import asdict,replace
import json
import math
import os
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np
import pytest
from moveit_msgs.msg import AttachedCollisionObject,PlanningScene
from rclpy.serialization import deserialize_message,serialize_message

from luggage_planning.joint_operation.certification import EndpointModel
from luggage_planning.joint_operation.connection import GeometricConnection
from luggage_planning.joint_operation.ik import SnapshotIKResult
from luggage_planning.joint_operation.view_scene import fixed_tool_from_optical,pickup_view_scene
from luggage_planning.joint_operation.view_motion import PickupViewMotionPreparation
from luggage_planning.joint_operation.view_proposals import PickupCameraProposal
from luggage_planning.joint_operation.view_certification import ViewMotionCertificate,NativeViewCertificationPort
from luggage_planning.placement_prior.contracts import Transform
from test_joint_operation_endpoints_native import robot_fixture
from test_joint_operation_scenes import baseline
from test_joint_operation_observation import measured_case


def camera_robot_fixture():
    xml,srdf=robot_fixture()
    camera='''<link name="actual_optical"><collision><geometry><sphere radius="0.01"/></geometry></collision></link>
    <joint name="optical_mount" type="fixed"><parent link="suction_contact_frame"/>
    <child link="actual_optical"/><origin xyz="0 0.15 0.2" rpy="0 0 0"/></joint>'''
    return xml.replace('</robot>',camera+'</robot>'),srdf


def motion_inputs(model=None):
    if model is None:
        xml,srdf=camera_robot_fixture()
        model=EndpointModel(xml,srdf,position_tolerance_m=.002,orientation_tolerance_rad=.02)
    source=baseline();scene=deserialize_message(source.scene_cdr,PlanningScene)
    scene.robot_model_name='endpoint_fixture'
    scene.world.collision_objects=[o for o in scene.world.collision_objects if o.id!='pickup_box']
    source=replace(source,model_hash=model.model_sha256,scene_cdr=serialize_message(scene))
    domain,_,_,view=measured_case()
    start=(0.,-1.,2.,0.,-1.,math.pi)
    goal=(*start[:5],start[5]+.05)
    camera=Transform.from_matrix(model.urdf.tcp(goal,model.urdf.path_to('actual_optical')))
    proposal=PickupCameraProposal(domain.digest(),view.digest(),'measured-gap',camera,1.,.02,100.)
    return SimpleNamespace(model=model,source=source,domain=domain,view=view,proposal=proposal,start=start,goal=goal)


def static_admission(scene):
    assert any(o.id=='container' for o in scene.world.collision_objects)
    return scene


@pytest.fixture
def capture(request):
    root=os.environ.get('PICKUP_VIEW_MOTION_EVIDENCE')
    directory=Path(root)/request.node.name if root else None
    if directory:directory.mkdir(parents=True,exist_ok=False)
    def record(f,name,value):
        if directory:
            if not (directory/'source_scene.cdr').exists():
                from luggage_perception.ros_suction_domain import domain_to_message
                from luggage_perception.pickup_view_service import view_to_message
                (directory/'source_scene.cdr').write_bytes(f.source.scene_cdr)
                (directory/'domain.cdr').write_bytes(serialize_message(domain_to_message(f.domain)))
                (directory/'view.cdr').write_bytes(serialize_message(view_to_message(f.view)))
                (directory/'proposal.json').write_text(json.dumps(asdict(f.proposal),indent=2))
            if isinstance(value,bytes):(directory/name).write_bytes(value)
            else:(directory/name).write_text(json.dumps(value,indent=2))
    return record


def unit_owner(f,*,certificate=None):
    calls=[]
    def refine(snapshot,target,seed,**kwargs):
        calls.append(('ik',snapshot,target,seed))
        return SnapshotIKResult(True,True,'EXACT_TARGET_CHECKED',f.goal)
    def connect(snapshot,start,goal,**kwargs):
        calls.append(('connection',snapshot,start,goal))
        return GeometricConnection(True,True,'CHECKED',tuple(
            tuple(a+(b-a)*i/10 for a,b in zip(start,goal)) for i in range(11)),11)
    def certify(snapshot,leg,digest,**kwargs):
        calls.append(('certification',snapshot,leg,digest))
        return certificate or ViewMotionCertificate(True,'UNIT_ONLY_RECEIPT','unit-certificate')
    owner=PickupViewMotionPreparation(f.model,SimpleNamespace(refine=refine),SimpleNamespace(connect=connect),
        SimpleNamespace(certify=certify),velocity_scale=.2,acceleration_scale=.1)
    return owner,calls


def prepare(owner,f,**changes):
    settings=dict(request_id='view',robot_state_stamp_ns=f.domain.stamp_ns,pickup_object_id='pickup_box',
        ik_seeds=(f.start,f.goal),max_ik_seeds=2,admit_static_scene=static_admission,
        source_current=lambda source:source is f.source,deadline=time.monotonic()+5.)
    settings.update(changes)
    return owner.prepare(f.source,f.domain,f.view,f.proposal,f.start,**settings)


def test_fixed_nonzero_camera_mount_does_not_change_with_wrist_and_target_round_trips(capture):
    f=motion_inputs()
    first=fixed_tool_from_optical(f.model,'actual_optical',f.start)
    last=fixed_tool_from_optical(f.model,'actual_optical',f.goal)
    capture(f,'mount.json',dict(first=asdict(first),last=asdict(last)))
    assert first.translation==pytest.approx((0.,.15,.2))
    assert last.translation==pytest.approx(first.translation)
    assert last.quaternion_xyzw==pytest.approx(first.quaternion_xyzw)
    tool=f.proposal.world_from_optical.compose(first.inverse())
    assert f.model.matches(f.goal,tool)


@pytest.mark.parametrize('damage',['missing','fixed_base','extra_moving'])
def test_camera_must_share_the_exact_six_joint_rigid_wrist(damage):
    f=motion_inputs();frame='not_in_model' if damage=='missing' else 'base'
    if damage=='extra_moving':
        xml,srdf=camera_robot_fixture()
        xml=xml.replace('name="optical_mount" type="fixed"','name="optical_mount" type="revolute"')
        f.model=EndpointModel(xml,srdf,position_tolerance_m=.002,orientation_tolerance_rad=.02)
        frame='actual_optical'
    with pytest.raises(ValueError,match='SAME_RIGID_WRIST'):fixed_tool_from_optical(f.model,frame,f.start)


def test_private_view_scene_preserves_static_and_unknown_objects_without_touch_exemption(capture):
    from moveit_msgs.msg import LinkPadding
    f=motion_inputs();scene=deserialize_message(f.source.scene_cdr,PlanningScene)
    scene.link_padding=[LinkPadding(link_name='suction_contact_frame',padding=.001)]
    f.source=replace(f.source,scene_cdr=serialize_message(scene))
    original=f.source.scene_cdr
    derived=pickup_view_scene(f.source,f.domain,'pickup_box',admit_static_scene=static_admission)
    actual=deserialize_message(derived.scene_cdr,PlanningScene)
    capture(f,'derived_scene.cdr',derived.scene_cdr)
    assert f.source.scene_cdr==original
    for obj in scene.world.collision_objects:assert obj in actual.world.collision_objects
    assert actual.link_padding==scene.link_padding
    target=next(o for o in actual.world.collision_objects if o.id=='pickup_box')
    assert list(target.primitives[0].dimensions)==list(f.domain.box_wdh)
    matrix=actual.allowed_collision_matrix;index=list(matrix.entry_names).index('pickup_box')
    assert not any(matrix.entry_values[index].enabled)
    assert all(not row.enabled[index] for row in matrix.entry_values)
    assert matrix.default_entry_values[list(matrix.default_entry_names).index('pickup_box')] is False
    assert scene.allowed_collision_matrix.entry_values[2].enabled[3]==matrix.entry_values[2].enabled[3]


@pytest.mark.parametrize('damage',['attachment','conflicting_id','diff','static_refused'])
def test_scene_conflict_or_incomplete_static_evidence_refuses_without_replacement(capture,damage):
    f=motion_inputs();scene=deserialize_message(f.source.scene_cdr,PlanningScene)
    admit=static_admission
    if damage=='attachment':scene.robot_state.attached_collision_objects=[AttachedCollisionObject()]
    elif damage=='conflicting_id':scene.world.collision_objects[0].id='pickup_box'
    elif damage=='diff':scene.is_diff=True
    else:admit=lambda scene:False
    f.source=replace(f.source,scene_cdr=serialize_message(scene));before=f.source.scene_cdr
    capture(f,'failure_input.cdr',before)
    with pytest.raises(ValueError):pickup_view_scene(f.source,f.domain,'pickup_box',admit_static_scene=admit)
    assert f.source.scene_cdr==before


def test_unit_pipeline_binds_scene_source_target_and_timed_empty_leg(capture):
    f=motion_inputs();owner,calls=unit_owner(f)
    result=prepare(owner,f);capture(f,'result.json',asdict(result))
    assert result.reason=='VIEW_PATH_PREPARED' and result.plan is not None
    plan=result.plan
    assert plan.base_snapshot_identity==f.source.identity and plan.view_snapshot_identity!=f.source.identity
    assert plan.leg.phase=='pick' and plan.leg.knots[0].positions==f.start
    assert plan.leg.knots[-1].positions==pytest.approx(f.goal)
    assert plan.world_from_optical==f.proposal.world_from_optical and plan.certificate_id=='unit-certificate'
    assert [call[0] for call in calls]==['ik','connection','certification']
    assert len({call[1].scene_cdr for call in calls})==1


@pytest.mark.parametrize('damage',['model','domain','view','cancelled','deadline','source','seed_budget','empty_seeds'])
def test_unbound_or_stopped_work_does_not_reach_native_backends(capture,damage):
    f=motion_inputs();owner,calls=unit_owner(f);settings={}
    if damage=='model':f.source=replace(f.source,model_hash='different')
    elif damage=='domain':f.proposal=replace(f.proposal,domain_sha256='0'*64)
    elif damage=='view':f.view=replace(f.view,acquisition=replace(f.view.acquisition,stamp_ns=f.domain.stamp_ns-1))
    elif damage=='cancelled':settings['cancelled']=lambda:True
    elif damage=='deadline':settings['deadline']=time.monotonic()-1.
    elif damage=='source':settings['source_current']=lambda source:False
    elif damage=='seed_budget':settings['max_ik_seeds']=1
    else:settings['ik_seeds']=()
    result=prepare(owner,f,**settings);capture(f,'result.json',asdict(result))
    assert result.plan is None and not calls


@pytest.mark.parametrize('failure',['certificate','revoked_source'])
def test_certification_or_final_source_refusal_never_publishes_a_plan(capture,failure):
    f=motion_inputs();owner,calls=unit_owner(f,
        certificate=ViewMotionCertificate(False,'CONSTRUCTED_NATIVE_REFUSAL') if failure=='certificate' else None)
    checks=[]
    def current(source):checks.append(source.identity);return len(checks)==1
    result=prepare(owner,f,**(dict(source_current=current) if failure=='revoked_source' else {}))
    capture(f,'result.json',asdict(result))
    assert result.plan is None and any(call[0]=='certification' for call in calls)
    if failure=='revoked_source':assert result.reason=='VIEW_SOURCE_CHANGED' and len(checks)==2


def test_native_certification_sample_cap_prevents_wire_allocation():
    from luggage_planning.joint_operation.planner import timed_operation_leg
    from test_joint_operation_contracts import example_plan
    f=motion_inputs();port=object.__new__(NativeViewCertificationPort)
    port.model=f.model;port.max_samples=2
    leg=timed_operation_leg('align','pick',(f.start,f.goal),example_plan().identity,.2,.1)
    result=port.certify(f.source,leg,'digest',deadline=time.monotonic()+2.,cancelled=lambda:False)
    assert not result.accepted and result.reason=='VIEW_CERTIFICATION_SAMPLE_CAP'
