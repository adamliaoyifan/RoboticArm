"""Actual O, ROS prepare owners and native view paths; synthetic acquisition.

No sensor estimator, controller, Gazebo or same-box recovery qualification.
"""
import copy
from dataclasses import replace
import json
import math
import os
import time
from types import SimpleNamespace

import numpy as np
import pytest
from rclpy.serialization import serialize_message,deserialize_message
from luggage_msgs.srv import GetPickupViewEvidenceV2,PreparePickupViewV2
from luggage_perception.directed_suction_quality import DirectedSuctionContext
from luggage_perception.pickup_view_evidence import PickupViewAcquisition
from luggage_perception.pickup_view_service import PickupViewService,view_to_message
from luggage_perception.ros_suction_domain import domain_to_message
from luggage_planning.joint_operation.scenes import _box
from luggage_planning.placement_prior.contracts import Transform
from test_joint_operation_endpoints_native import native_stack,robot_fixture
from test_joint_operation_observer_native import observer_owner
from test_joint_operation_preparation_native import prepare_stack

def camera_model():
    xml,srdf=robot_fixture()
    camera='''<link name="actual_optical"/>
    <joint name="optical_mount" type="fixed"><parent link="suction_contact_frame"/>
    <child link="actual_optical"/><origin xyz="0 0 -0.3"/></joint>'''
    return xml.replace('</robot>',camera+'</robot>'),srdf


pytestmark=[pytest.mark.skipif(not os.environ.get('JOINT_NATIVE_ENDPOINT_EXECUTABLE'),
    reason='explicit actual O/native pickup-view preparation gate'),
    pytest.mark.parametrize('native_stack',[camera_model()],indirect=True,ids=['fixed-camera']),
    pytest.mark.parametrize('prepare_stack',['observer'],indirect=True)]


@pytest.fixture(scope='module')
def view_owner_stack(prepare_stack):
    f=prepare_stack;root=f.root
    directory=f.directory/'pickup_view';directory.mkdir()
    # Measured box remains stationary; current empty tool is above its contact.
    start=(0.,-1.2,2.,0.,-.8,math.pi)
    domain=replace(f.domain,stamp_ns=f.domain.stamp_ns-100,support=bytes(len(f.domain.support)))
    camera=root.model.urdf.tcp(start,root.model.urdf.path_to('actual_optical'))
    shape=(240,320);intrinsics=SimpleNamespace(fx=300.,fy=300.,cx=159.5,cy=119.5)
    ys,xs=np.indices(shape)
    rays=np.stack(((xs-intrinsics.cx)/intrinsics.fx,(ys-intrinsics.cy)/intrinsics.fy,np.ones(shape)),axis=-1)
    directions=rays@camera[:3,:3].T
    normal=np.asarray(domain.normal_world)
    distance=-(normal@camera[:3,3]+domain.plane_offset_world_m)/(directions@normal)
    depth=(distance*1000.).astype(np.float32)
    # Declared deterministic raw-depth gaps. They exercise decision recovery,
    # not the simulator sensor or generic perception estimator's error model.
    depth[::5,:]=0.;depth[:,::5]=0.
    acquisition=PickupViewAcquisition.capture(depth,np.ones(shape,bool),intrinsics,camera,
        stamp_ns=domain.stamp_ns,optical_frame_id='actual_optical',instance_id=domain.instance_id,
        generation=domain.generation,region_source='instance_mask')
    view=acquisition.bind(domain)
    context=DirectedSuctionContext(domain,copy.deepcopy(f.quality_context._height_map),
        f.quality_context._model,pickup_view=view)
    f.quality.replace_context(context)
    with root._state_lock:
        root.domain_history.remember(domain)
        root.joints=(f.domain.stamp_ns,start)
    initial=copy.deepcopy(f.request);initial.request_id='view-owner-task';initial.observation_id='view-owner-acquisition'
    initial.detection.header.stamp.sec,initial.detection.header.stamp.nanosec=divmod(domain.stamp_ns,10**9)
    initial.start_joints=list(start)
    # Rebind the frozen slot's request identity to this actual prepare call.
    for slot in initial.slots:slot.identity.request_id=initial.request_id
    service=PickupViewService(context_lookup=f.quality.context_snapshot)
    server=f.owner.create_service(GetPickupViewEvidenceV2,'/luggage/get_pickup_view_evidence_v2',service.get)
    client=f.owner.create_client(PreparePickupViewV2,'/joint_operation/prepare_pickup_view')
    assert client.wait_for_service(timeout_sec=5.)
    for port in (root.pickup_views.motion.ik,root.pickup_views.motion.certifier):
        assert port.client.wait_for_service(timeout_sec=5.)
    (directory/'initial_request.cdr').write_bytes(serialize_message(initial))
    (directory/'domain.cdr').write_bytes(serialize_message(domain_to_message(domain)))
    (directory/'view.cdr').write_bytes(serialize_message(view_to_message(view)))
    failed=f.call(initial)
    (directory/'whole_prepare_response.cdr').write_bytes(serialize_message(failed))
    assert not failed.success and failed.reason=='SEARCH_INCOMPLETE' and failed.unresolved_contacts,failed.reason
    probe=failed.unresolved_contacts[0]
    assert root.pickup_views.contexts.lookup(initial.request_id,domain.digest()) is not None
    request=PreparePickupViewV2.Request(request_id='view-owner-recovery',task_request_id=initial.request_id,
        domain_sha256=domain.digest(),query_id=probe.query_id,
        robot_state_stamp_valid=True,robot_state_stamp_ns=f.domain.stamp_ns,start_joints=list(start),
        tilts_rad=[0.],azimuths_rad=[0.],distance_scales=[1.],
        min_optical_z_m=.05,max_optical_z_m=3.,max_view_proposals=1,max_ik_seeds=2,budget_s=20.)
    def call(request,name):
        path=directory/name;path.mkdir()
        (path/'request.cdr').write_bytes(serialize_message(request))
        (path/'scene.cdr').write_bytes(serialize_message(f.state.scene))
        future=client.call_async(request);deadline=time.monotonic()+25.
        while not future.done() and time.monotonic()<deadline:time.sleep(.01)
        assert future.done(),'pickup-view prepare callback deadline'
        result=future.result()
        (path/'response.cdr').write_bytes(serialize_message(result))
        (path/'result.json').write_text(json.dumps(dict(success=result.success,reason=result.reason,
            trace=json.loads(result.trace_json),elapsed_s=result.elapsed_s),indent=2))
        return result
    try:yield SimpleNamespace(f=f,root=root,domain=domain,view=view,request=request,call=call,directory=directory)
    finally:
        f.owner.destroy_service(server);f.owner.destroy_client(client)


def test_o_owner_generated_service_issues_native_view(view_owner_stack):
    s=view_owner_stack;r=s.call(s.request,'healthy')
    assert r.success and r.reason=='PICKUP_VIEW_PREPARED',r.reason
    receipt=s.root.pickup_views.contexts.issued(s.request.task_request_id,r.plan.route_digest)
    assert receipt is not None and receipt.plan.certificate_id==r.plan.certificate_id
    assert receipt.map_source.epoch==s.f.observer.lease.identity.epoch
    assert r.plan.model_sha256==s.root.model.model_sha256
    assert receipt.plan.view_scene_cdr==receipt.snapshot.scene_cdr
    assert any(row[0]=='view_motion' and row[2]=='VIEW_PATH_PREPARED' for row in json.loads(r.trace_json))


@pytest.mark.parametrize('where',['start','goal'])
def test_o_native_obstacle_refuses_view_after_same_input_control(view_owner_stack,where):
    s=view_owner_stack;req=copy.deepcopy(s.request);req.request_id+='-'+where
    control=s.call(req,'control-'+where)
    assert control.success,control.reason
    leg=control.plan.leg.trajectory.trajectory
    q=tuple(req.start_joints) if where=='start' else tuple(leg.points[-1].positions)
    tool=Transform.from_matrix(s.root.model.urdf.tcp(q,s.root.model.path))
    other=tuple(leg.points[-1].positions) if where=='start' else tuple(req.start_joints)
    other_tool=Transform.from_matrix(s.root.model.urdf.tcp(other,s.root.model.path))
    separation=tool.inverse().compose(other_tool).translation
    obstacle_size=.006
    distance=float(np.linalg.norm(np.maximum(np.abs(separation)-obstacle_size/2,0.)))
    radius=float(s.root.model.urdf.root.find(
        "link[@name='suction_contact_frame']/collision/geometry/sphere").get('radius'))
    (s.directory/('isolation-'+where+'.json')).write_text(json.dumps(dict(
        obstacle_size_m=obstacle_size,other_tool_to_obstacle_distance_m=distance,
        tool_collision_radius_m=radius,endpoint_tolerance_m=s.root.model.position_tolerance_m)))
    assert distance>radius+s.root.model.position_tolerance_m,'fixture must isolate the two endpoints'
    original=copy.deepcopy(s.f.state.scene)
    s.f.state.scene.world.collision_objects.append(_box('blocked-view',(obstacle_size,)*3,tool,'world'))
    try:
        result=s.call(req,'blocked-'+where)
        assert not result.success and not result.plan.route_digest,result.reason
        rows=[row for row in json.loads(result.trace_json) if row[0]=='view_motion']
        assert rows
        if where=='start':
            assert any(item[0]=='connection' and item[2]=='STATE_COLLISION' for row in rows for item in row[3]),rows
        else:assert any(item[0]=='ik' and item[2]=='NO_VALID_IK' for row in rows for item in row[3]),rows
    finally:s.f.state.scene=original


def test_actual_native_certificate_then_source_change_never_issues_new_view(view_owner_stack):
    s=view_owner_stack;native=s.root.pickup_views.motion.certifier
    certify=native.certify;scene=copy.deepcopy(s.f.state.scene);observed=[]
    def changed(*args,**kwargs):
        result=certify(*args,**kwargs);observed.append(result.accepted)
        if result.accepted:s.f.state.scene.name='changed-after-native'
        return result
    native.certify=changed
    try:
        req=copy.deepcopy(s.request);req.request_id+='-changed'
        result=s.call(req,'source-changed')
        assert observed and any(observed),'actual native certification did not trigger'
        assert not result.success and not result.plan.route_digest,result.reason
        assert any(row[0]=='view_motion' and row[2]=='VIEW_SOURCE_CHANGED' for row in json.loads(result.trace_json))
    finally:native.certify=certify;s.f.state.scene=scene
