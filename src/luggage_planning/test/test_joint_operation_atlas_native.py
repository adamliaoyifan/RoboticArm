"""Qualified synthetic atlas -> real carrying IK/pair connection -> prepare RPC.

The offline fixture uses real native q; it never qualifies deployment artifacts.
"""
from dataclasses import asdict,replace
import copy
import hashlib
import json
import math
import os
import time
from types import SimpleNamespace

import numpy as np
import pytest
import yaml
from rclpy.serialization import deserialize_message,serialize_message

from test_joint_operation_endpoints_native import native_stack,robot_fixture
from test_joint_operation_preparation_native import prepare_stack
from test_atlas_proposals import artifact
from luggage_planning.atlas_proposals import AtlasQualification,artifact_digest
from luggage_planning.joint_operation_atlas import AtlasOperationHints,load_operation_atlas
from luggage_planning.joint_operation_ik import NativeSnapshotIKPort,ik_evidence
from luggage_planning.joint_operation_candidates import propose_contact,candidate_for_contact
from luggage_planning.joint_operation_contacts import contact_query,contact_evidence
from luggage_planning.placement_prior_contracts import PlacementCandidate,Transform
from luggage_planning.placement_prior_snapshot import build_private_snapshot
from luggage_msgs.srv import EvaluateDirectedSuctionV2,RefineSnapshotToolTargetV2

pytestmark=pytest.mark.skipif(not os.environ.get('JOINT_NATIVE_ENDPOINT_EXECUTABLE'),
                             reason='explicit real native atlas gate')


@pytest.fixture(scope='module')
def atlas_stack(prepare_stack):
    fixture=prepare_stack;root=fixture.root
    model=fixture.quality_context._model
    snapshot=build_private_snapshot(root.store.current(),fixture.state.scene,root.model.model_sha256,
        root.container,collision_margin_m=root.margin,volume=fixture.state.volume)
    p=fixture.request.slots[0].pose
    slot=PlacementCandidate('requested-0',Transform((p.position.x,p.position.y,p.position.z),
        (p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w)))
    reference=Transform.from_matrix(root.model.urdf.tcp(list(fixture.request.start_joints),root.model.path))
    proposal=propose_contact(fixture.domain,model,query_id='requested-0:1',
        world_from_place_box=slot.container_from_box,desired_world_from_tool=reference,max_center_checks=256)
    query=contact_query(fixture.domain,fixture.request.detection,[proposal.to_message()],model,budget_s=1.)
    response=fixture.quality.handle(query,EvaluateDirectedSuctionV2.Response())
    contact=contact_evidence(response,query,fixture.domain,model).contacts[0]
    candidate=candidate_for_contact(snapshot,fixture.domain,contact,slot,root.limits,
        pickup_object_id=root.pickup_id,observation_id=fixture.request.observation_id,rank=0)
    ik=NativeSnapshotIKPort(root,root.model,recorder=lambda kind,value:root._record('atlas_ik_'+kind,value))
    assert ik.client.wait_for_service(timeout_sec=5.)
    def refine(target,seed):
        result=ik.refine(candidate.scenes.carry,target,seed,request_id='offline-fixture',
            deadline=time.monotonic()+2.,cancelled=lambda:False)
        assert result.complete and result.success,result.reason
        return result.joints
    contact_q=refine(candidate.world_from_release_tool,fixture.request.start_joints)
    xyz=candidate.world_from_release_tool.translation
    # A predecessor target with lateral displacement, not contact + global lift.
    transit=Transform((xyz[0]-.03,xyz[1]+.01,xyz[2]+.10),candidate.world_from_release_tool.quaternion_xyzw)
    transit_q=refine(transit,contact_q)
    directory=fixture.directory/'atlas';directory.mkdir()
    xml,_=robot_fixture()
    scene_path=directory/'scene.yaml'
    scene_path.write_text(yaml.safe_dump(dict(world_frame='world',static_transforms=[dict(
        parent='world',child='container_link',translation=[0.,0.,0.],rotation_rpy=[0.,0.,0.])])) )
    shash=hashlib.md5(scene_path.read_bytes()).hexdigest();uhash=hashlib.md5(xml.encode()).hexdigest()
    def mutate(data):
        data['contact_seeds'][:]=contact_q;data['transit_seeds'][:]=transit_q
        data['transit_target'][:]=transit.translation
        data['contact_seeds'][...,0,:]=4.  # first contact seed is out of bounds
        data['transit_seeds'][...,2,:]=4.  # third contact passes, transit refuses
    def metadata(meta):
        meta['grid']['origin']=[v-.075 for v in xyz]
        meta['dependencies']['scene_tf_hash']=shash;meta['dependencies']['urdf_hash']=uhash
        meta['contract']['scene_revision']=shash;meta['contract']['urdf_hash']=uhash
        meta['contract']['geometry_hash']=snapshot.geometry_hash
    _,_,npz,meta=artifact(directory,mutate=mutate,meta_mutate=metadata)
    payload_hash=yaml.safe_load(meta.read_text())['contract']['payload_hash']
    qualification=AtlasQualification(artifact_digest(npz.read_bytes(),meta.read_bytes()),
        shash,uhash,snapshot.geometry_hash,payload_hash,'synthetic-native-checked-pair-only')
    receipt=directory/'qualification.json'
    receipt.write_text(json.dumps(dict(qualification=asdict(qualification),scene_revision='scene',
        model_sha256=root.model.model_sha256),indent=2))
    reader=load_operation_atlas(npz,meta,receipt,scene_path,robot_description=xml,
        model_sha256=root.model.model_sha256,scene_revision='scene',world_from_container=root.container)
    provider=AtlasOperationHints(reader,ik,root.preparation.planner.connections,
        scene_revision='scene',model_sha256=root.model.model_sha256)
    return SimpleNamespace(fixture=fixture,candidate=candidate,provider=provider,ik=ik,
        reader=reader,transit=transit,contact_q=contact_q,transit_q=transit_q,directory=directory,
        paths=(npz,meta,receipt,scene_path))


def test_actual_paired_seeds_check_distinct_transit_and_keep_storage_identity(atlas_stack):
    a=atlas_stack
    augmented,trace=a.provider.augment(a.candidate,request_id='pair',deadline=time.monotonic()+4.,
        cancelled=lambda:False,clock=time.monotonic)
    assert len(augmented.place_entry_seeds)==1
    hint=augmented.place_entry_seeds[0]
    assert hint.seed_id.endswith(':1') and hint.seed_id.startswith('atlas-pair:')
    assert a.fixture.root.model.matches(hint.joints,a.transit)
    pairs=next(row for row in trace if row[0]=='atlas_pairs')
    assert pairs[1]=='PAIRS_READY'
    assert [reason for _,reason in pairs[2]]==['CONTACT_REFINE_MISS','ONLINE_PAIR_CHECKED','TRANSIT_REFINE_MISS']
    (a.directory/'pair_trace.json').write_text(json.dumps(trace,indent=2))


def test_real_prepare_consumes_qualified_pair_hint_and_certifies_complete_chain(atlas_stack):
    a=atlas_stack;core=a.fixture.root.preparation;old=core.atlas;core.atlas=a.provider
    try:
        result=a.fixture.call(copy.deepcopy(a.fixture.request))
        assert result.success,result.reason+' '+result.trace_json
        assert len(result.plan.legs)==6 and len(result.plan.barriers)==2
        trace=json.loads(result.trace_json)
        assert any('place:hint:atlas-pair:' in row[0] and row[1]=='PLAN_READY'
                   for row in trace if len(row)>=2 and isinstance(row[0],str))
        assert any(row[:2]==['atlas','QUALIFIED_PROPOSALS'] for row in trace)
        (a.directory/'prepared_response.cdr').write_bytes(serialize_message(result))
    finally:core.atlas=old


@pytest.mark.parametrize('damage',['model','scene','container','receipt','bytes'])
def test_receipt_and_actual_dependency_bytes_are_required(atlas_stack,damage,tmp_path):
    a=atlas_stack;npz,meta,receipt,scene=a.paths
    model=a.fixture.root.model.model_sha256;revision='scene';container=a.fixture.root.container
    if damage=='model':model='0'*64
    elif damage=='scene':revision='new'
    elif damage=='container':container=Transform((.01,0.,0.),(0.,0.,0.,1.))
    elif damage=='receipt':
        receipt=tmp_path/'absent.json';receipt.write_text('{}')
    else:
        npz=tmp_path/'changed.npz';npz.write_bytes(a.paths[0].read_bytes()+b'changed')
    with pytest.raises((ValueError,KeyError)):
        load_operation_atlas(npz,meta,receipt,scene,robot_description=robot_fixture()[0],
            model_sha256=model,scene_revision=revision,world_from_container=container)


@pytest.mark.parametrize('damage',['bounds','obstacle','model','deadline'])
def test_native_ik_refuses_current_invalid_inputs_without_joint_result(atlas_stack,damage):
    from moveit_msgs.msg import PlanningScene,CollisionObject
    from shape_msgs.msg import SolidPrimitive
    from geometry_msgs.msg import Pose
    a=atlas_stack;snapshot=a.candidate.scenes.carry;seed=a.contact_q;deadline=time.monotonic()+1.
    if damage=='bounds':seed=(4.,)*6
    elif damage=='model':snapshot=replace(snapshot,model_hash='0'*64)
    elif damage=='deadline':deadline=time.monotonic()-1.
    else:
        scene=deserialize_message(snapshot.scene_cdr,PlanningScene)
        obj=CollisionObject(id='blocking-world',operation=CollisionObject.ADD);obj.header.frame_id='world'
        shape=SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[3.,3.,3.])
        pose=Pose();pose.orientation.w=1.;pose.position.z=1.
        obj.primitives=[shape];obj.primitive_poses=[pose];scene.world.collision_objects.append(obj)
        snapshot=replace(snapshot,scene_cdr=serialize_message(scene))
    result=a.ik.refine(snapshot,a.candidate.world_from_release_tool,seed,request_id='bad-'+damage,
        deadline=deadline,cancelled=lambda:False)
    assert not result.success and not result.joints


def test_same_tool_pose_obstacle_rejects_carried_box_but_accepts_unloaded_arm(atlas_stack):
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import PlanningScene,CollisionObject
    from shape_msgs.msg import SolidPrimitive
    a=atlas_stack;snapshot=a.candidate.scenes.carry
    scene=deserialize_message(snapshot.scene_cdr,PlanningScene)
    payload=scene.robot_state.attached_collision_objects[0].object.primitive_poses[0]
    tool_box=Transform((payload.position.x,payload.position.y,payload.position.z),
        (payload.orientation.x,payload.orientation.y,payload.orientation.z,payload.orientation.w))
    centre=a.candidate.world_from_release_tool.compose(tool_box)
    obstacle=CollisionObject(id='payload-only-obstacle',operation=CollisionObject.ADD)
    obstacle.header.frame_id='world'
    shape=SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.01,.01,.01])
    pose=Pose();pose.orientation.w=1.
    pose.position.x,pose.position.y,pose.position.z=centre.translation
    obstacle.primitives=[shape];obstacle.primitive_poses=[pose];scene.world.collision_objects.append(obstacle)
    def evaluate(current,label):
        raw=serialize_message(current)
        (a.directory/(label+'_scene.cdr')).write_bytes(raw)
        return a.ik.refine(replace(snapshot,scene_cdr=raw),a.candidate.world_from_release_tool,
            a.contact_q,request_id=label,deadline=time.monotonic()+1.,cancelled=lambda:False)
    loaded=evaluate(scene,'payload_blocked')
    assert loaded.complete and not loaded.success and not loaded.joints
    scene.robot_state.attached_collision_objects=[]
    unloaded=evaluate(scene,'unloaded_clear')
    assert unloaded.complete and unloaded.success


@pytest.mark.parametrize('damage',['request_id','snapshot_id','model_sha256','scene_sha256',
    'partial','changed_q','nan_q','checks','reason','elapsed','negative_q'])
def test_native_ik_response_binding_and_exact_fk_are_independently_admitted(atlas_stack,damage):
    from luggage_planning.joint_operation_scenes import _pose
    a=atlas_stack;snapshot=a.candidate.scenes.carry
    request=RefineSnapshotToolTargetV2.Request(request_id='receipt',
        snapshot_id=hashlib.sha256(repr(snapshot.identity).encode()).hexdigest(),
        model_sha256=snapshot.model_hash,scene_sha256=hashlib.sha256(snapshot.scene_cdr).hexdigest(),
        scene_cdr=list(snapshot.scene_cdr),world_from_tool=_pose(a.candidate.world_from_release_tool),
        seed_joints=list(a.contact_q),budget_s=.2)
    result=a.ik.refine(snapshot,a.candidate.world_from_release_tool,a.contact_q,request_id='receipt',
        deadline=time.monotonic()+1.,cancelled=lambda:False)
    assert result.success
    response=copy.deepcopy(a.ik.inflight.result())
    assert ik_evidence(response,request,a.fixture.root.model).success
    if damage in ('request_id','snapshot_id','model_sha256','scene_sha256'):setattr(response,damage,'changed')
    elif damage=='partial':response.complete=False
    elif damage=='changed_q':response.joints[0]+=.1
    elif damage=='nan_q':response.joints[0]=float('nan')
    elif damage=='checks':response.checked_states=0
    elif damage=='reason':response.reason='OTHER'
    elif damage=='elapsed':response.elapsed_s=.3
    else:response.success=False
    assert not ik_evidence(response,request,a.fixture.root.model).success


def test_atlas_budget_exhaustion_keeps_real_online_prepare_fallback(atlas_stack):
    a=atlas_stack;core=a.fixture.root.preparation;old=core.atlas
    core.atlas=AtlasOperationHints(a.reader,a.ik,a.fixture.root.preparation.planner.connections,
        scene_revision='scene',model_sha256=a.fixture.root.model.model_sha256,max_refinement_s=1e-9)
    try:
        result=a.fixture.call(copy.deepcopy(a.fixture.request))
        assert result.success,result.reason+' '+result.trace_json
        trace=json.loads(result.trace_json)
        pairs=next(row for row in trace if row[0]=='atlas_pairs')
        assert pairs[1]=='CANCELLED_OR_DEADLINE'
        assert any(row[1]=='PLAN_READY' and 'atlas-pair:' not in row[0]
                   for row in trace if len(row)>=2 and isinstance(row[1],str))
    finally:core.atlas=old
