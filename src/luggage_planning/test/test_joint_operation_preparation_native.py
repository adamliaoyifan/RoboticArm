"""Generated prepare RPC through measured quality and three native backends.

Synthetic full-pose robot/measurements; no Gazebo, controller or deployment
qualification. Boundary CDR, model, quality samples and teardown are durable.
"""
from dataclasses import replace
import dataclasses
import copy
import json
import math
import os
from pathlib import Path
import threading
import time

import numpy as np
import pytest

from test_joint_operation_endpoints_native import native_stack, robot_fixture

pytestmark = pytest.mark.skipif(not os.environ.get('JOINT_NATIVE_ENDPOINT_EXECUTABLE'),
                                reason='explicit native preparation gate')


@pytest.fixture(scope='module')
def prepare_stack(native_stack,request):
    import yaml
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.serialization import serialize_message
    from rcl_interfaces.msg import ParameterValue
    from rcl_interfaces.srv import GetParameters
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from moveit_msgs.msg import PlanningScene
    from moveit_msgs.srv import GetPlanningScene
    from luggage_msgs.srv import GetCargoVolumeV2, EvaluateDirectedSuctionV2, PrepareJointOperationV2
    from luggage_msgs.msg import SuctionTopDomainV2, SurfaceContractV2
    from luggage_description.container_geometry import cuboid_from_inner_size
    from luggage_description.frame_diagnostics import BoundaryContext
    from luggage_description.ros_frame_bindings.surfaces import surface_to_message
    from luggage_description.suction_contact_model import load_suction_contact_model
    from luggage_perception.observed_suction_domain import ObservedSuctionDomain
    from luggage_perception.suction_height_map import HeightMap
    from luggage_perception.directed_suction_quality import DirectedSuctionContext
    from luggage_perception.directed_suction_service import DirectedSuctionService
    from luggage_perception.frozen_suction_service import register_frozen_suction_services
    from luggage_perception.ros_suction_domain import domain_to_message
    from luggage_planning.joint_operation.prepare_node import JointOperationPrepareNode
    from luggage_planning.joint_operation.contracts import OPERATION_JOINTS
    from luggage_planning.placement_prior.surface import ConfirmedSurface
    from luggage_planning.placement_prior.requests import enumerate_placements
    from luggage_planning.place.place_service import pose_message
    from test_suction_patch_evaluator import make_model
    from types import SimpleNamespace

    stack = native_stack
    backend = getattr(request,'param','grid_g')
    observer = request.getfixturevalue('observer_owner') if backend == 'observer' else None
    directory = stack.directory/'preparation'
    directory.mkdir()
    fields = make_model(.06).as_dict();fields.pop('identity_hash')
    contact_file = directory/'contact.yaml';contact_file.write_text(yaml.safe_dump(fields))
    model = load_suction_contact_model(contact_file)
    q = (0.,-1.,2.,0.,-1.,math.pi)
    tcp = stack.model.urdf.tcp(list(q),stack.model.path)
    x,y,z = (float(v) for v in tcp[:3,3])
    shape = (40,30);origin=(x-.1,y-.075)
    domain = ObservedSuctionDomain('box','instance',3,12_000_000_000,'world',1,
        model.identity_hash,(x,y,z-.05),(0.,0.,0.,1.),(.2,.15,.1),
        (1.,0.,0.),(0.,1.,0.),(0.,0.,1.),-z,origin,.005,shape,bytes([1])*1200)
    hmap = HeightMap(.005,*origin,*shape,np.ones(shape,np.int32),np.zeros(shape),np.zeros(shape),
        np.ones(shape,np.uint8),np.ones(shape,bool),
        np.broadcast_to([0.,0.,1.],shape+(3,)).copy(),np.array([1.,0.,0.]),np.array([0.,1.,0.]),
        np.array([0.,0.,1.]),-z,np.array([],int),np.array([0],int),np.array([],int),
        np.array([],float),np.empty((0,2)))
    quality_context = DirectedSuctionContext(domain,hmap,model)
    quality = DirectedSuctionService();quality.replace_context(quality_context)
    np.savez_compressed(directory/'quality_samples.npz',support=domain.cells(),**{
        f.name:getattr(hmap,f.name) for f in dataclasses.fields(hmap)
        if isinstance(getattr(hmap,f.name),np.ndarray)})
    metadata=dataclasses.asdict(domain);metadata.pop('support')
    (directory/'quality_acquisition.json').write_text(json.dumps(metadata,indent=2))
    descriptor = cuboid_from_inner_size((1.6,1.6,1.),floor_z=.6).descriptor()
    record = dict(contract_version=2,kind='surface',frame_id='container_link',length_unit='m',
        stamp_ns=12_000_000_000,time_basis='scene',scene_revision='scene',request_id='map',
        geometry_hash=descriptor['geometry_hash'],map_revision=7,height_datum='container_origin',
        resolution_m=.1,origin_xy_container=[-.8,-.8],floor_z_container=.6,ceiling_z_container=1.6,
        surface_z_container=[[.6]*16 for _ in range(16)],state=[['free']*16 for _ in range(16)],
        confidence=[['sensor']*16 for _ in range(16)])
    confirmation = dict(geometry_hash=record['geometry_hash'],map_revision=7,frame='container_local',
        map_source='measured',occupancy_confirmed=True,acquisition_stamp_ns=12_000_000_000,
        pending_changes=[],placed_count=0,state=record['state'],height=[[0.]*16 for _ in range(16)],
        inner_size=[1.6,1.6,1.],geometry_descriptor=descriptor)
    volume = GetCargoVolumeV2.Response(success=True,scene_revision='scene',geometry_hash=record['geometry_hash'],
        map_revision=7,frame_id='container_link',acquisition_stamp_ns=12_000_000_000,map_source='measured',
        occupancy_confirmed=True,resolution_m=.1,origin_container=[-.8,-.8,.6],dimensions_xyz=[16,16,10],
        states=[1]*2560,active=[True]*2560)
    confirmed = ConfirmedSurface(json.dumps(record).encode(),b'fixture',json.dumps(confirmation).encode())
    pool = enumerate_placements(confirmed,domain.box_wdh,2048,volume=volume,margin=.05)
    slot = min(pool,key=lambda p:sum((a-b)**2 for a,b in zip(p.container_from_box.translation,(.55,0.,.65))))
    scene = PlanningScene();scene.robot_model_name='endpoint_fixture'
    scene.robot_state.joint_state.name=list(OPERATION_JOINTS)
    scene.robot_state.joint_state.position=list(q)
    owner = stack.node
    frozen = register_frozen_suction_services(owner,context_provider=quality.context_snapshot,
        context_lookup=quality.context_snapshot,
        model_provider=lambda:quality_context._model)
    state = SimpleNamespace(scene=scene,volume=volume,scene_calls=0,on_scene=None)
    def serve_scene(request,response):
        state.scene_calls += 1
        if state.on_scene: state.on_scene(state.scene_calls)
        response.scene=copy.deepcopy(state.scene)
        return response
    def serve_volume(request,response): return copy.deepcopy(state.volume)
    services = [owner.create_service(GetPlanningScene,'/get_planning_scene',serve_scene),
        owner.create_service(EvaluateDirectedSuctionV2,'/luggage/evaluate_directed_suction_v2',quality.handle)]
    if backend == 'grid_g':
        services.extend([owner.create_service(GetCargoVolumeV2,'/cargo_map/get_volume_v2',serve_volume),
            owner.create_service(GetParameters,'/cargo_volume_mapper/get_parameters',
                lambda req,res:GetParameters.Response(values=[ParameterValue(type=1,bool_value=True)]))])
    xml,srdf=robot_fixture()
    from test_joint_operation_scene_admission import fixture_scene_file
    fixture_scene,fixture_mesh,fixture_object=fixture_scene_file(directory)
    state.scene.world.collision_objects.append(fixture_object)
    params=dict(use_sim_time=True,frame_scene_revision='scene',robot_description=xml,robot_description_semantic=srdf,
        scene_tf_config=str(fixture_scene),container_collision_mesh=str(fixture_mesh),
        container_translation=[0.,0.,0.],container_quaternion_xyzw=[0.,0.,0.,1.],
        suction_contact_model_config=str(contact_file),attachment_geometry_config=str(
            Path('src/luggage_planning/config/attachment_geometry.yaml').resolve()),
        geometric_slot_limit=2048,evidence_dir=str(directory/'boundaries'))
    if observer is None:params['map_backend']='grid_g'
    else:params.update(observer_geometry_file=str(observer.geometry),observer_config_file=str(observer.config))
    root=JointOperationPrepareNode(context=owner.context,
        parameter_overrides=[Parameter(k,value=v) for k,v in params.items()])
    # Synthetic acquisitions/joints in this native-only fixture are at12s.
    # Keep the root on that explicit ROS clock instead of the host wall clock.
    from rclpy.time import Time
    root.get_clock().set_ros_time_override(Time(nanoseconds=12_000_000_000))
    executor=MultiThreadedExecutor(context=owner.context,num_threads=4);executor.add_node(root)
    worker=threading.Thread(target=executor.spin,daemon=True);worker.start()
    client=owner.create_client(PrepareJointOperationV2,'/joint_operation/prepare')
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    publishers=[owner.create_publisher(SurfaceContractV2,'/luggage/cargo_map/surface_v2',qos) if observer is None else None,
        owner.create_publisher(String,'/luggage/cargo_map/surface_2d',qos) if observer is None else None,
        owner.create_publisher(SuctionTopDomainV2,'/luggage/suction_top_domain_v2',10),
        owner.create_publisher(JointState,'/joint_states',10)]
    surface=surface_to_message(record,BoundaryContext('TEST','owner','joint_operation','map','scene',
        'container_link','','scene',record['geometry_hash'],7))
    joints=JointState();joints.header.stamp.sec=12;joints.name=list(OPERATION_JOINTS);joints.position=list(q)
    messages=[surface,String(data=json.dumps(confirmation)),domain_to_message(domain),joints]
    request=PrepareJointOperationV2.Request(request_id='native-root',observation_id='observed',generation=3,
        geometry_hash=record['geometry_hash'],expected_map_revision=7,expected_model_sha256=stack.model.model_sha256,
        robot_state_stamp_valid=True,robot_state_stamp_ns=12_000_000_000,start_joints=list(q),candidate_limit=8,budget_s=10.)
    if observer is not None:
        request.expected_map_revision=observer.lease.identity.map_revision
        request.expected_map_epoch=observer.lease.identity.epoch
    box=request.detection;box.id=domain.box_id;box.header.frame_id='world';box.header.stamp.sec=12
    box.pose.position.x,box.pose.position.y,box.pose.position.z=domain.box_xyz_world;box.pose.orientation.w=1.
    box.width,box.depth,box.height=domain.box_wdh;box.top_surface_valid=box.height_valid=True
    box.height_source=box.HEIGHT_SOURCE_MEASURED_SUPPORT
    request.slots=[pose_message(dict(contract_version=2,kind='pose',frame_id='container_link',length_unit='m',
        stamp_ns=0,time_basis='scene',scene_revision='scene',request_id=request.request_id,reference='box_center',
        orientation_semantics='box_axis_pi',position=slot.container_from_box.translation,
        quaternion_xyzw=slot.container_from_box.quaternion_xyzw))]
    for name,message in zip(('surface','confirmation','top_domain','joints'),messages):
        (directory/('original_'+name+'.cdr')).write_bytes(serialize_message(message))
    (directory/'original_detection.cdr').write_bytes(serialize_message(request.detection))
    if observer is None:(directory/'original_volume.cdr').write_bytes(serialize_message(volume))
    (directory/'original_scene.cdr').write_bytes(serialize_message(scene))
    def call(current):
        future=client.call_async(current);limit=time.monotonic()+15.
        while not future.done() and time.monotonic()<limit: time.sleep(.01)
        assert future.done(),'prepare callback did not return within bounded gate'
        return future.result()
    try:
        assert client.wait_for_service(timeout_sec=5.)
        for port in (root.preparation.contacts,root.preparation.planner.endpoints,
                     root.preparation.planner.connections,root.preparation.planner.certification):
            assert port.client.wait_for_service(timeout_sec=5.)
        clients = (root.scene_client,root.volume_client,root.map_parameters) if observer is None else (
            root.scene_client,*root.observer.clients.values())
        for client_ in clients:
            assert client_.wait_for_service(timeout_sec=5.)
        for client_ in (root.observed_contacts.retain_client,root.preparation.contacts.frozen_client):
            assert client_.wait_for_service(timeout_sec=5.)
        limit=time.monotonic()+5.
        while time.monotonic()<limit:
            for index,(pub,msg) in enumerate(zip(publishers,messages)):
                if observer is None or index>=2:pub.publish(msg)
            if (observer is not None or root.store.current()) and root.domain is not None and root.joints is not None:break
            time.sleep(.05)
        assert (observer is not None or root.store.current()) and root.domain and root.joints
        yield SimpleNamespace(request=request,call=call,root=root,state=state,domain=domain,
            publishers=publishers,messages=messages,directory=directory,quality=quality,
            quality_context=quality_context,owner=owner,frozen=frozen,observer=observer)
    finally:
        executor.shutdown(timeout_sec=3.);worker.join(timeout=3.)
        (directory/'root_teardown.json').write_text(json.dumps(dict(worker_stopped=not worker.is_alive(),
            scope='synthetic measured fixture; no controllers/vacuum/scene writes')))
        root.destroy_node()
        owner.destroy_client(client)
        for service in (*services,*frozen.services):owner.destroy_service(service)
        for pub in publishers:
            if pub is not None:owner.destroy_publisher(pub)


def test_generated_prepare_rpc_returns_whole_certified_plan(prepare_stack):
    result=prepare_stack.call(copy.deepcopy(prepare_stack.request))
    assert result.success,result.reason+' '+result.trace_json
    assert result.reason=='PLAN_READY' and len(result.plan.legs)==6 and len(result.plan.barriers)==2
    assert result.elapsed_s<=prepare_stack.request.budget_s
    rounds=[row for row in json.loads(result.trace_json) if row[0]=='contact_round']
    assert rounds and all(row[2]==0 for row in rounds)
    search=_search_evidence(result)
    assert search['domain_sha256']==prepare_stack.domain.digest()
    assert search['furthest_stage']=='NATIVE_CANDIDATE' and search['native_candidates_yielded']>0
    (prepare_stack.directory/'result.json').write_text(json.dumps(dict(reason=result.reason,
        elapsed_s=result.elapsed_s,trace=json.loads(result.trace_json)),indent=2))


def _search_evidence(result):
    records=[row[1] for row in json.loads(result.trace_json) if row[0]=='preparation_search_evidence']
    assert len(records)==1
    assert records[0]['schema']=='preparation_search_evidence_v1'
    assert records[0]['terminal_reason']==result.reason
    assert records[0]['observation_deficiency']=='NOT_ESTABLISHED'
    return records[0]


@pytest.mark.parametrize('fault',['unproven_support','rough_quality','quality_backend'])
def test_prepare_search_receipts_distinguish_support_quality_and_transport(prepare_stack,fault):
    """Constructed measured-input faults; no camera deficiency is fabricated."""
    from rclpy.serialization import serialize_message
    from luggage_perception.directed_suction_quality import DirectedSuctionContext
    from luggage_perception.ros_suction_domain import domain_to_message
    from luggage_planning.joint_operation.contacts import ContactBatchEvidence
    f=prepare_stack
    directory=f.directory/('search_'+fault);directory.mkdir()
    # The fixture clock is frozen at the original acquisition; variants must
    # precede it rather than creating an inadmissible future observation.
    domain=replace(f.domain,stamp_ns=f.domain.stamp_ns-1-['unproven_support','rough_quality','quality_backend'].index(fault))
    samples=copy.deepcopy(f.quality_context._height_map)
    if fault=='unproven_support':
        domain=replace(domain,support=bytes(len(domain.support)))
    elif fault=='rough_quality':
        # Deliberately rough synthetic surface; no calibrated hardware bound.
        heights=(np.indices(samples.shape).sum(axis=0)%2*2-1)*.02
        samples.median_h=heights.copy();samples.smooth_h=heights.copy()
    context=DirectedSuctionContext(domain,samples,f.quality_context._model)
    request=copy.deepcopy(f.request)
    request.request_id+='-search-'+fault
    request.observation_id+='-search-'+fault
    request.detection.header.stamp.sec,request.detection.header.stamp.nanosec=divmod(domain.stamp_ns,10**9)
    f.quality.replace_context(context)
    with f.root._state_lock:f.root.domain_history.remember(domain)
    (directory/'domain.cdr').write_bytes(serialize_message(domain_to_message(domain)))
    np.savez_compressed(directory/'quality_samples.npz',support=domain.cells(),**{
        field.name:getattr(samples,field.name) for field in dataclasses.fields(samples)
        if isinstance(getattr(samples,field.name),np.ndarray)})
    original_evaluate=f.root.preparation.contacts.evaluate
    def unavailable(*args,**kwargs):
        actual=original_evaluate(*args,**kwargs)
        (directory/'quality_injection.json').write_text(json.dumps(dict(
            actual_reason=actual.reason,injected_reason='CONTACT_QUALITY_BACKEND_UNAVAILABLE',
            scope='test-only unavailable boundary receipt after actual frozen quality RPC')))
        return ContactBatchEvidence(False,'CONTACT_QUALITY_BACKEND_UNAVAILABLE')
    if fault=='quality_backend':f.root.preparation.contacts.evaluate=unavailable
    try:
        result=f.call(request)
        (directory/'prepare_response.cdr').write_bytes(serialize_message(result))
        (directory/'result.json').write_text(json.dumps(dict(reason=result.reason,
            trace=json.loads(result.trace_json)),indent=2))
        assert not result.success and result.reason=='SEARCH_INCOMPLETE',result.reason
        assert not result.plan.legs and not result.plan.barriers
        record=_search_evidence(result)
        assert record['domain_sha256']==domain.digest()
        assert record['native_candidates_yielded']==0
        if fault=='unproven_support':
            assert record['supported_cells']==0
            assert record['unproven_cells']==len(domain.support)
            assert record['furthest_stage']=='CONTACT_PROPOSAL'
            assert record['proposals_accepted']==record['quality_batches_complete']==record['quality_batches_incomplete']==0
        elif fault=='rough_quality':
            assert record['furthest_stage']=='DIRECTED_QUALITY'
            assert record['quality_batches_complete']>0 and record['quality_contacts_refused']>0
            assert record['quality_batches_incomplete']==record['quality_contacts_accepted']==0
        else:
            assert record['furthest_stage']=='DIRECTED_QUALITY'
            assert record['quality_batches_incomplete']>0
            assert record['quality_batches_complete']==record['quality_contacts_refused']==0
            assert set(record['quality_batch_reasons'])=={'CONTACT_QUALITY_BACKEND_UNAVAILABLE'}
        assert not f.root._active.locked()
    finally:
        f.root.preparation.contacts.evaluate=original_evaluate
        f.quality.replace_context(f.quality_context)
    # Explicit healthy request after fault; no automatic observation or motion.
    restored=f.call(copy.deepcopy(f.request))
    assert restored.success,restored.reason
    assert _search_evidence(restored)['native_candidates_yielded']>0


def test_prepare_keeps_requested_acquisition_when_newer_topic_precedes_rpc(prepare_stack):
    """Real ROS + native planning reproduction of the saved198ms handoff race."""
    from luggage_perception.directed_suction_quality import DirectedSuctionContext
    from luggage_perception.ros_suction_domain import domain_to_message
    f=prepare_stack
    newer=DirectedSuctionContext(replace(f.domain,stamp_ns=f.domain.stamp_ns+198_000_000),
        f.quality_context._height_map,f.quality_context._model)
    f.quality.replace_context(newer)
    f.publishers[2].publish(domain_to_message(newer.domain))
    deadline=time.monotonic()+2.
    while time.monotonic()<deadline and f.root.domain.stamp_ns!=newer.domain.stamp_ns:
        time.sleep(.01)
    assert f.root.domain.stamp_ns==newer.domain.stamp_ns
    try:
        result=f.call(copy.deepcopy(f.request))
        assert result.success,result.reason
        assert result.plan.observation_stamp_ns==f.domain.stamp_ns
    finally:
        f.quality.replace_context(f.quality_context)
        f.publishers[2].publish(f.messages[2])


@pytest.mark.parametrize('missing,reason',[
    (('observed_domain',),'OBSERVED_DOMAIN_UNAVAILABLE'),
    (('joint_state',),'CURRENT_JOINT_STATE_UNAVAILABLE'),
    (('observed_domain','joint_state'),'OBSERVATION_AND_JOINTS_UNAVAILABLE'),
])
def test_prepare_reports_each_missing_input_before_search_and_recovers(prepare_stack,missing,reason):
    """Generated RPC faults distinguish missing transport inputs from bad views."""
    f=prepare_stack
    records=[]
    original_record=f.root._record
    def capture(boundary,value):
        if boundary=='domain_handoff_unavailable':records.append(json.loads(value))
        return original_record(boundary,value)
    f.root._record=capture
    with f.root._state_lock:
        saved_joints=f.root.joints
        if 'observed_domain' in missing:f.root.domain_history.clear('TEST_MISSING_INPUT')
        if 'joint_state' in missing:f.root.joints=None
    calls_before=f.state.scene_calls
    try:
        result=f.call(copy.deepcopy(f.request))
        assert not result.success and not result.plan.legs and not result.plan.barriers
        assert result.reason=='PREPARATION_REFUSED:ValueError:'+reason
        assert f.state.scene_calls==calls_before
        assert len(records)==1
        assert records[0]['reason']==reason
        assert records[0]['missing_inputs']==list(missing)
        assert records[0]['domain_available']==('observed_domain' not in missing)
        assert records[0]['joints_available']==('joint_state' not in missing)
        assert records[0]['requested_stamp_ns']==f.domain.stamp_ns
        assert not f.root._active.locked()
        (f.directory/('unavailable_'+reason+'.json')).write_text(json.dumps(records[0],indent=2))
    finally:
        f.root._record=original_record
        with f.root._state_lock:
            f.root.joints=saved_joints
            f.root.domain_history.remember(f.domain)
    # Recovery here is another explicit request after restored real inputs,
    # not an automatic view, latest-frame substitution or movement.
    result=f.call(copy.deepcopy(f.request))
    assert result.success,result.reason
    assert result.reason=='PLAN_READY'


def test_prepare_refuses_empty_static_world_before_native_motion_search(prepare_stack):
    saved=copy.deepcopy(prepare_stack.state.scene)
    try:
        prepare_stack.state.scene.world.collision_objects=[]
        result=prepare_stack.call(copy.deepcopy(prepare_stack.request))
        assert not result.success and not result.plan.legs
        assert 'STATIC_CONTAINER_MISSING_OR_DUPLICATE' in result.reason
    finally:
        prepare_stack.state.scene=saved


@pytest.mark.parametrize('allow_alternates',[False,True])
def test_both_primary_contacts_refused_recover_same_target_with_native_alternate(prepare_stack,allow_alternates):
    """Constructed endpoint failure; the alternate still needs real full planning."""
    f=prepare_stack
    endpoint=f.root.preparation.planner.endpoints
    solve=endpoint.solve
    original_limit=f.root.preparation.max_contact_proposals_per_direction
    f.root.preparation.max_contact_proposals_per_direction=2 if allow_alternates else 1
    tried=[]
    def refuse_primary(candidate,**kwargs):
        tried.append(candidate)
        result=solve(candidate,**kwargs)
        if len(tried)<=2:
            f.root._record('test_endpoint_refusal',json.dumps(dict(
                candidate_id=candidate.candidate_id,actual_native_reason=result.reason,
                injected_reason='TEST_PRIMARY_CONTACT_REFUSED')).encode())
            return replace(result,complete=False,reason='TEST_PRIMARY_CONTACT_REFUSED',
                pick_branches=(),place_branches=())
        return result
    endpoint.solve=refuse_primary
    try:
        result=f.call(copy.deepcopy(f.request))
        if not allow_alternates:
            assert not result.success and result.reason=='SEARCH_INCOMPLETE'
            assert not result.plan.legs and len(tried)==2
            search=_search_evidence(result)
            assert search['native_candidates_yielded']==2
            assert search['quality_contacts_accepted']>=2
            assert search['furthest_stage']=='NATIVE_CANDIDATE'
            (f.directory/'prepick_contact_recovery_disabled.json').write_text(json.dumps(dict(
                reason=result.reason,trace=json.loads(result.trace_json)),indent=2))
            return
        assert result.success,result.reason+' '+result.trace_json
        assert len(tried)>=3
        assert len({c.world_from_settled_box for c in tried})==1
        assert np.allclose(tried[0].world_from_pick_tool.translation,
            tried[1].world_from_pick_tool.translation,atol=1e-8,rtol=0)
        assert not np.allclose(tried[2].world_from_pick_tool.translation,
            tried[0].world_from_pick_tool.translation,atol=1e-8,rtol=0)
        assert result.plan.candidate_id==tried[-1].candidate_id
        assert len(result.plan.legs)==6 and len(result.plan.barriers)==2
        rounds=[row for row in json.loads(result.trace_json) if row[0]=='contact_round']
        assert rounds[0][2]==0 and rounds[1][2]==1
        (f.directory/'prepick_contact_recovery.json').write_text(json.dumps(dict(
            reason=result.reason,tried=[dict(candidate_id=c.candidate_id,
            contact=c.world_from_pick_tool.translation,target=c.world_from_settled_box.translation)
            for c in tried],trace=json.loads(result.trace_json)),indent=2))
    finally:
        endpoint.solve=solve
        f.root.preparation.max_contact_proposals_per_direction=original_limit


@pytest.mark.parametrize('damage',['model','generation','map','size','start','budget','no_quality'])
def test_invalid_prepare_inputs_return_inactive_plan(prepare_stack,damage):
    request=copy.deepcopy(prepare_stack.request)
    if damage=='model':request.expected_model_sha256='0'*64
    elif damage=='generation':request.generation+=1
    elif damage=='map':request.expected_map_revision+=1
    elif damage=='size':request.detection.width+=.001
    elif damage=='start':request.start_joints[0]+=.1
    elif damage=='budget':request.budget_s=0.
    else:prepare_stack.quality.replace_context(None)
    try:
        result=prepare_stack.call(request)
        assert not result.success and not result.plan.legs and not result.plan.barriers
    finally:
        prepare_stack.quality.replace_context(prepare_stack.quality_context)


@pytest.mark.parametrize('change',['world','frozen_revoked','occ','joints'])
def test_inputs_changing_during_final_scene_read_refuse_certified_result(prepare_stack,change):
    """Exercise the race after certification, while scene RPC is pending."""
    from std_msgs.msg import String
    fixture=prepare_stack
    original_scene=copy.deepcopy(fixture.state.scene)
    fixture.state.scene_calls=0
    def mutate(count):
        if count!=2:return
        if change=='world':
            # A change in padding changes actual collision semantics.
            from moveit_msgs.msg import LinkPadding
            fixture.state.scene.link_padding=[LinkPadding(link_name='suction_contact_frame',padding=.001)]
            return
        if change=='frozen_revoked':
            fixture.frozen.store.revoke_all()
            return
        elif change=='occ':
            confirmation=json.loads(fixture.messages[1].data);confirmation['pending_changes']=[{'event':'observed_change'}]
            fixture.publishers[1].publish(String(data=json.dumps(confirmation)))
            changed=lambda:fixture.root.store.current() is None
        else:
            message=copy.deepcopy(fixture.messages[3]);message.position[0]+=.1;message.header.stamp.sec=13
            fixture.publishers[3].publish(message)
            changed=lambda:abs(fixture.root.joints[1][0])>.05
        limit=time.monotonic()+1.
        while not changed() and time.monotonic()<limit:time.sleep(.005)
        assert changed(),'test did not deliver the changing input before scene response'
    fixture.state.on_scene=mutate
    try:
        result=fixture.call(copy.deepcopy(fixture.request))
        assert fixture.state.scene_calls==2
        assert not result.success and not result.plan.legs and not result.plan.barriers
        assert 'SNAPSHOT_CHANGED' in result.reason or 'START_STATE_MISMATCH' in result.reason
    finally:
        fixture.state.on_scene=None;fixture.state.scene=original_scene
        for publisher,message in zip(fixture.publishers,fixture.messages):publisher.publish(message)
        limit=time.monotonic()+1.
        while time.monotonic()<limit:
            if (fixture.root.store.current() and fixture.root.domain.digest()==fixture.domain.digest()
                    and fixture.root.joints[1]==tuple(fixture.request.start_joints)):break
            time.sleep(.005)
        assert fixture.root.store.current() and fixture.root.domain.digest()==fixture.domain.digest()
        assert fixture.root.joints[1]==tuple(fixture.request.start_joints)


def test_busy_prepare_refuses_second_rpc_without_overlapping_native_work(prepare_stack):
    fixture=prepare_stack
    port=fixture.root.preparation.contacts
    original=port.evaluate
    entered,release=threading.Event(),threading.Event()
    first=[]
    def hold(*args,**kwargs):
        entered.set()
        assert release.wait(2.),'test did not release first preparation'
        return original(*args,**kwargs)
    port.evaluate=hold
    worker=threading.Thread(target=lambda:first.append(fixture.call(copy.deepcopy(fixture.request))))
    worker.start()
    try:
        assert entered.wait(2.)
        result=fixture.call(copy.deepcopy(fixture.request))
        assert not result.success and result.reason=='OPERATION_PREPARATION_BUSY' and not result.plan.legs
    finally:
        release.set();worker.join(timeout=12.);port.evaluate=original
    assert not worker.is_alive() and len(first)==1 and first[0].success


def test_response_capture_failure_cannot_publish_ready_plan_or_strand_rpc(prepare_stack):
    fixture=prepare_stack;original=fixture.root._record
    def fail(boundary,value):
        if boundary=='prepare_response':raise OSError('synthetic capture device failure')
        return original(boundary,value)
    fixture.root._record=fail
    try:
        result=fixture.call(copy.deepcopy(fixture.request))
        assert not result.success and not result.plan.legs and result.reason=='PREPARATION_EVIDENCE_FAILED:OSError'
        assert not fixture.root._active.locked()
        from rclpy.serialization import serialize_message
        (fixture.directory/'capture_failure_response.cdr').write_bytes(serialize_message(result))
    finally:fixture.root._record=original


def test_one_short_global_deadline_returns_no_partial_plan(prepare_stack):
    request=copy.deepcopy(prepare_stack.request);request.budget_s=.001
    result=prepare_stack.call(request)
    assert not result.success and not result.plan.legs and not result.plan.barriers
