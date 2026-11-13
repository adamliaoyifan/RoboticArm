"""Behavioral source/scene/acquisition wiring with scripted native boundary.

Actual native workers and ROS service publication are covered separately.
"""
from dataclasses import asdict,replace
import json
import math
import os
from pathlib import Path
import threading
from types import SimpleNamespace

import pytest
from moveit_msgs.msg import AttachedCollisionObject,PlanningScene
from rclpy.serialization import deserialize_message,serialize_message
from luggage_msgs.srv import PreparePickupViewV2
from luggage_perception.pickup_view_service import view_to_message
from luggage_perception.ros_suction_domain import domain_to_message
from luggage_planning.joint_operation.contracts import OperationIdentity
from luggage_planning.joint_operation.planner import timed_operation_leg
from luggage_planning.joint_operation.preparation_profile import PreparationBudget
from luggage_planning.joint_operation.view_context import PickupViewContext,PickupViewContextStore
from luggage_planning.joint_operation.view_motion import PreparedPickupView,PickupViewMotionResult
from luggage_planning.joint_operation.view_owner import PickupViewPrepareOwner
from luggage_planning.joint_operation.view_scene import pickup_view_scene,fixed_tool_from_optical
from luggage_planning.placement_prior.contracts import Transform
from test_joint_operation_observer import configuration,snapshot
from test_pickup_view_context import context_case,store
from test_pickup_view_motion import static_admission


@pytest.fixture
def owner_case(tmp_path,request):
    f=context_case()
    root=os.environ.get('PICKUP_VIEW_OWNER_EVIDENCE')
    f.directory=Path(root)/request.node.name if root else tmp_path
    f.directory.mkdir(exist_ok=True,parents=True)
    f.directory.joinpath('original_request.cdr').write_bytes(f.context.request_cdr)
    f.directory.joinpath('original_domain.cdr').write_bytes(serialize_message(domain_to_message(f.domain)))
    f.directory.joinpath('original_probe.cdr').write_bytes(f.context.probes_cdr[0][1])
    f.directory.joinpath('original_view.cdr').write_bytes(serialize_message(view_to_message(f.view)))
    config,_,_=configuration(f.directory,size=(1.,1.,.4))
    f.states=[1]*math.prod(config.dimensions_xyz)
    f.captured=snapshot(config,f.states)
    f.original.geometry_hash=f.captured.source.geometry_hash
    f.context=PickupViewContext.capture(f.original,f.domain,f.observation,[f.probe_message])
    baseline=deserialize_message(f.source.scene_cdr,PlanningScene)
    f.baseline=baseline;f.calls=[];f.records=[];f.current=True;f.retained=True;f.on_motion=None;f.on_scene=None
    def record(name,value):
        raw=value if isinstance(value,bytes) else serialize_message(value)
        path=f.directory/('%04d_%s.bin'%(len(f.records),name));path.write_bytes(raw)
        f.records.append(name)
    def current(req,**kwargs):
        f.calls.append(('current_start',tuple(req.start_joints)))
        if tuple(req.start_joints)!=f.start:raise ValueError('CURRENT_START_STATE_MISMATCH')
    def retain(*args,**kwargs):
        f.calls.append(('retain',kwargs['expected_observation'].binding))
        return SimpleNamespace(retained=f.retained,observation=f.observation)
    def call(client,req,deadline,boundary):
        f.calls.append(('camera',req.domain_sha256,req.detection.id))
        if f.view is None:return SimpleNamespace(success=False,reason='NO_VIEW')
        return SimpleNamespace(success=True,view=view_to_message(f.view))
    def capture(req,deadline,*,latest):
        f.calls.append(('capture',latest,req.expected_map_epoch))
        return f.captured.surface,f.captured.volume,f.captured.source
    def scene(deadline):
        f.calls.append(('scene',))
        if f.on_scene:f.on_scene()
        copy=deserialize_message(serialize_message(f.baseline),PlanningScene)
        return static_admission(copy)
    f.node=SimpleNamespace(observer=SimpleNamespace(planning_current=lambda *a,**k:f.current),
        _active=threading.Lock(),_state_lock=threading.Lock(),joints=(f.domain.stamp_ns,f.start),
        model=f.model,revision='scene',container=Transform((0.,0.,0.),(0.,0.,0.,1.)),
        margin=.02,vertical_margin=0.,pickup_id='pickup_box',budget=PreparationBudget(30.),
        preparation=SimpleNamespace(model=f.contact_model),observed_contacts=SimpleNamespace(retain=retain),
        static_world=SimpleNamespace(admit=static_admission),_record=record,_current=current,
        _call=call,_capture_map=capture,_scene=scene,
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=f.domain.stamp_ns)),
        get_logger=lambda:SimpleNamespace(warning=lambda value:f.calls.append(('warning',value))))
    f.owner=PickupViewPrepareOwner.__new__(PickupViewPrepareOwner)
    f.owner.node=f.node;f.owner.contexts=store();f.owner.contexts.remember(f.context)
    f.owner.max_joint_age_ns=1_000_000_000
    from luggage_msgs.srv import GetPickupViewEvidenceV2
    f.owner.evidence_service=GetPickupViewEvidenceV2;f.owner.evidence_client=object()
    def motion(source,domain,view,proposal,start,**kwargs):
        f.calls.append(('motion',source,kwargs['ik_seeds']))
        if not kwargs['source_current'](source):return PickupViewMotionResult('VIEW_SOURCE_NOT_CURRENT')
        derived=pickup_view_scene(source,domain,'pickup_box',admit_static_scene=static_admission)
        f.node._record('scripted_checked_view_scene',derived.scene_cdr)
        identity=OperationIdentity(kwargs['request_id'],'scene','synthetic',source.geometry_hash,
            f.model.model_sha256,source.map_revision,'obs',domain.stamp_ns,kwargs['robot_state_stamp_ns'],
            domain.box_id,domain.generation,domain.contact_model_hash)
        leg=timed_operation_leg('align','pick',(f.start,f.goal),identity,.2,.1)
        mount=fixed_tool_from_optical(f.model,'actual_optical',f.start)
        plan=PreparedPickupView(kwargs['request_id'],domain.digest(),view.digest(),proposal.query_id,
            source.identity,derived.identity,proposal.world_from_optical,
            proposal.world_from_optical.compose(mount.inverse()),mount,leg,'d'*64,'scripted-certificate',derived.scene_cdr)
        if f.on_motion:f.on_motion()
        if not kwargs['source_current'](source):return PickupViewMotionResult('VIEW_SOURCE_CHANGED')
        return PickupViewMotionResult('VIEW_PATH_PREPARED',plan)
    f.owner.motion=SimpleNamespace(prepare=motion)
    f.request=PreparePickupViewV2.Request(request_id='extra-view',task_request_id='task',
        domain_sha256=f.domain.digest(),query_id=f.probe.query_id,robot_state_stamp_valid=True,
        robot_state_stamp_ns=f.domain.stamp_ns,start_joints=list(f.start),
        tilts_rad=[.1],azimuths_rad=[0.],distance_scales=[1.],
        min_optical_z_m=.1,max_optical_z_m=3.,max_view_proposals=2,max_ik_seeds=2,budget_s=10.)
    f.directory.joinpath('map_input.json').write_text(json.dumps(dict(states=f.states,
        source=asdict(f.captured.source)),indent=2))
    yield f
    f.directory.joinpath('calls.json').write_text(json.dumps([row[:1] for row in f.calls]))
    f.directory.joinpath('final_scene.cdr').write_bytes(serialize_message(f.baseline))


def prepare(f):return f.owner.prepare(f.request,PreparePickupViewV2.Response())


def assert_inactive(result,reason):
    assert not result.success and not result.plan.route_digest and not result.plan.leg.trajectory.trajectory.joint_names,result
    assert reason in result.reason,result.reason


def test_latest_o_original_target_private_scene_and_exact_owner_receipt(owner_case):
    f=owner_case;r=prepare(f)
    assert r.success,r.reason
    receipt=f.owner.contexts.issued('task',r.plan.route_digest)
    from luggage_msgs.msg import PickupViewPlanV2
    assert receipt is not None and deserialize_message(receipt.message_cdr,PickupViewPlanV2)==r.plan
    assert receipt.snapshot.scene_cdr==receipt.plan.view_scene_cdr
    assert receipt.map_source==f.captured.source and r.plan.query_id==f.probe.query_id
    assert ('capture',True,1) in f.calls
    scenes=[row[1] for row in f.calls if row[0]=='motion']
    scene=deserialize_message(scenes[0].scene_cdr,PlanningScene)
    assert any(obj.id=='container' for obj in scene.world.collision_objects)
    scene=deserialize_message(receipt.snapshot.scene_cdr,PlanningScene)
    assert any(obj.id=='pickup_box' for obj in scene.world.collision_objects)
    assert r.plan.leg.name=='align' and r.plan.leg.phase=='pick'
    assert not f.node._active.locked()


def test_newer_o_content_preserves_original_target_and_unknown_occupied_obstacles(owner_case):
    f=owner_case
    f.original.expected_map_revision=6
    f.context=PickupViewContext.capture(f.original,f.domain,f.observation,[f.probe_message])
    f.owner.contexts.forget('task');f.owner.contexts.remember(f.context)
    config,_,_=configuration(f.directory,size=(1.,1.,.4))
    f.states[0]=2;f.states[-1]=0;f.captured=snapshot(config,f.states)
    (f.directory/'updated_map_input.json').write_text(json.dumps(dict(states=f.states,
        source=asdict(f.captured.source)),indent=2))
    r=prepare(f);assert r.success,r.reason
    receipt=f.owner.contexts.issued('task',r.plan.route_digest)
    assert receipt.map_source.map_revision==7 and f.original.expected_map_revision==6
    scene=deserialize_message(receipt.snapshot.scene_cdr,PlanningScene)
    obstacles=[o for o in scene.world.collision_objects if o.id.startswith('placement-prior-occ-')]
    assert len(obstacles)>=2
    assert all(o.primitives[0].dimensions[0]==pytest.approx(.14) for o in obstacles)
    assert all(o.primitives[0].dimensions[2]==pytest.approx(.1) for o in obstacles)
    trace=json.loads(r.trace_json)
    assert any(row[0]=='view_motion' and row[2]=='VIEW_PATH_PREPARED' for row in trace)


def test_no_camera_in_explicit_range_does_not_start_native_search(owner_case):
    f=owner_case;f.request.distance_scales=[10.]
    r=prepare(f);assert_inactive(r,'NO_USEFUL_CAMERA_PROPOSAL')
    assert not any(row[0] in ('motion','capture') for row in f.calls)


def test_issued_receipt_is_independent_and_does_not_extend_context_expiry(owner_case):
    from luggage_msgs.msg import PickupViewPlanV2
    f=owner_case;r=prepare(f);assert r.success,r.reason
    receipt=f.owner.contexts.issued('task',r.plan.route_digest)
    r.plan.leg.name='client-edit'
    assert deserialize_message(receipt.message_cdr,PickupViewPlanV2).leg.name=='align'
    expires=f.owner.contexts.entries['task'].expires_at
    f.owner.contexts.clock=lambda:expires
    assert f.owner.contexts.issued('task',receipt.plan.route_digest) is None


@pytest.mark.parametrize('field,value',[('task_request_id','wrong'),('query_id','invented'),('domain_sha256','wrong')])
def test_unowned_hypothesis_does_not_read_camera_or_plan(owner_case,field,value):
    f=owner_case;setattr(f.request,field,value);r=prepare(f)
    assert_inactive(r,'OWNED_UNRESOLVED_CONTACT_UNAVAILABLE')
    assert not any(row[0] in ('camera','motion','capture') for row in f.calls)


@pytest.mark.parametrize('fault',['expired','no_view','wrong_view','revoked','stale_start','moved_start','g_backend'])
def test_source_start_and_backend_failures_cannot_issue_view(owner_case,fault):
    f=owner_case
    if fault=='expired':f.owner.contexts.forget('task')
    elif fault=='no_view':f.view=None
    elif fault=='wrong_view':f.view=replace(f.view.acquisition,stamp_ns=f.domain.stamp_ns+1).bind(replace(f.domain,stamp_ns=f.domain.stamp_ns+1))
    elif fault=='revoked':f.retained=False
    elif fault=='stale_start':f.node.joints=(f.domain.stamp_ns-2_000_000_000,f.start)
    elif fault=='moved_start':f.request.start_joints[0]+=.2
    else:f.node.observer=None
    r=prepare(f);assert not r.success and not r.plan.route_digest
    assert not any(row[0]=='motion' for row in f.calls)
    assert not f.node._active.locked()


def test_finite_depth_only_refusal_does_not_trigger_path_work(owner_case):
    import numpy as np
    f=owner_case;a=f.view.acquisition
    depth=np.frombuffer(a.depth_mm,dtype=a.depth_dtype).copy();depth[depth==0]=1000.
    f.view=replace(a,depth_mm=depth.tobytes()).bind(f.domain)
    r=prepare(f);assert_inactive(r,'NO_DECISION_RELEVANT_VIEW_GAP')
    assert not any(row[0] in ('motion','capture') for row in f.calls)


@pytest.mark.parametrize('fault',['epoch','unknown_target','no_uld','attached'])
def test_latest_target_and_full_scene_are_hard_boundaries(owner_case,fault):
    f=owner_case
    if fault=='epoch':f.captured=replace(f.captured,source=replace(f.captured.source,epoch=2))
    elif fault=='unknown_target':
        config,_,_=configuration(f.directory,size=(1.,1.,.4));f.states=[0]*len(f.states)
        f.captured=snapshot(config,f.states)
    elif fault=='no_uld':f.baseline.world.collision_objects=[o for o in f.baseline.world.collision_objects if o.id!='container']
    else:f.baseline.robot_state.attached_collision_objects=[AttachedCollisionObject()]
    r=prepare(f);assert not r.success and not r.plan.route_digest
    assert f.owner.contexts.issued('task','d'*64) is None


@pytest.mark.parametrize('fault',['map','world','observation','camera','joints','context'])
def test_changes_during_certification_cannot_publish_success(owner_case,fault):
    f=owner_case
    def change():
        if fault=='map':f.current=False
        elif fault=='world':f.baseline.name='different-static-world'
        elif fault=='observation':f.retained=False
        elif fault=='camera':f.view=None
        elif fault=='joints':f.node.joints=(f.domain.stamp_ns-2_000_000_000,f.start)
        else:f.owner.contexts.forget('task')
    f.on_motion=change
    r=prepare(f);assert not r.success and not r.plan.route_digest
    assert any(row[0]=='motion' for row in f.calls)
    assert f.owner.contexts.issued('task','d'*64) is None


@pytest.mark.parametrize('fault',['capture','retention','deadline','busy','invalid_sampling'])
def test_capture_resources_deadline_and_shared_lock_cannot_leave_live_plan(owner_case,fault):
    f=owner_case;original=f.node._record
    if fault=='capture':
        def broken(name,value):
            original(name,value)
            if name=='pickup_view_prepare_response':raise RuntimeError('capture failure')
        f.node._record=broken
    elif fault=='retention':f.owner.contexts.max_bytes=f.context.size_bytes+10
    elif fault=='deadline':f.request.budget_s=1e-10
    elif fault=='busy':f.node._active.acquire()
    else:f.request.max_view_proposals=0
    try:
        r=prepare(f);assert not r.success and not r.plan.route_digest
        assert f.owner.contexts.issued('task','d'*64) is None
        assert f.node._active.locked()==(fault=='busy')
    finally:
        if f.node._active.locked():f.node._active.release()


@pytest.mark.parametrize('reason',['PLAN_READY','IK_REFUSED','BACKEND_UNAVAILABLE','DEADLINE'])
def test_ordinary_non_gap_results_never_retain_recovery_context(owner_case,reason):
    f=owner_case;f.owner.contexts.forget('task')
    f.owner.remember(f.original,SimpleNamespace(success=reason=='PLAN_READY',reason=reason,
        unresolved_contacts=[]),f.domain,f.observation)
    assert f.owner.contexts.lookup('task',f.domain.digest()) is None and not f.calls


def test_optional_context_failure_does_not_modify_original_response(owner_case):
    f=owner_case;f.owner.contexts.forget('task');f.owner.contexts.max_bytes=1
    response=SimpleNamespace(success=False,reason='SEARCH_INCOMPLETE',unresolved_contacts=[f.probe_message])
    f.owner.remember(f.original,response,f.domain,f.observation)
    assert response.reason=='SEARCH_INCOMPLETE' and response.unresolved_contacts==[f.probe_message]
    assert f.owner.contexts.lookup('task',f.domain.digest()) is None
