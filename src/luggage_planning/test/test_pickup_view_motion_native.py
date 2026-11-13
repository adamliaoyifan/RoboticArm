"""Actual private-scene IK/OMPL/timed certification for empty-arm camera views.

Synthetic full-pose six-axis fixture with a fixed optical link. No controller,
Gazebo, measured-information-gain or deployment qualification is claimed.
"""
from dataclasses import asdict,replace
import json
import os
import time

import numpy as np
import pytest
from moveit_msgs.msg import PlanningScene
from rclpy.serialization import serialize_message,deserialize_message

from luggage_planning.joint_operation.ik import NativeSnapshotIKPort
from luggage_planning.joint_operation.view_certification import NativeViewCertificationPort
from luggage_planning.joint_operation.view_motion import PickupViewMotionPreparation
from luggage_planning.joint_operation.view_scene import pickup_view_scene
from luggage_planning.joint_operation.scenes import _box
from luggage_planning.placement_prior.contracts import Transform
from test_joint_operation_endpoints_native import native_stack
from test_pickup_view_motion import camera_robot_fixture,motion_inputs,prepare,static_admission

pytestmark=[pytest.mark.skipif(not os.environ.get('JOINT_NATIVE_ENDPOINT_EXECUTABLE'),reason='explicit native camera-view gate'),
    pytest.mark.parametrize('native_stack',[camera_robot_fixture()],indirect=True,ids=['fixed-camera'])]


@pytest.fixture(scope='module')
def view_stack(native_stack):
    stack=native_stack
    record=lambda port:lambda kind,raw:stack.record(port,kind,raw if isinstance(raw,bytes) else serialize_message(raw))
    ik=NativeSnapshotIKPort(stack.node,stack.model,max_query_s=1.,recorder=record('view_ik'))
    assert ik.client.wait_for_service(timeout_sec=10.)
    certifier=NativeViewCertificationPort(stack.node,stack.model,velocity_limits=(.314,)*6,
        acceleration_limits=(.314,)*6,max_samples=50000,recorder=record('view_certificate'))
    assert certifier.client.wait_for_service(timeout_sec=10.)
    owner=PickupViewMotionPreparation(stack.model,ik,stack.connections,certifier,
        velocity_scale=.2,acceleration_scale=.1)
    return stack,owner


def scenario(stack,name):
    f=motion_inputs(stack.model)
    # The synthetic pickup is below the viewing tool. No truth/query bypass:
    # the same explicitly constructed measured box reaches the native scene.
    f.domain=replace(f.domain,box_xyz_world=(.6,0.,.55),box_wdh=(.2,.15,.1),plane_offset_world_m=-.6)
    f.view=f.view.acquisition.bind(f.domain)
    f.proposal=replace(f.proposal,domain_sha256=f.domain.digest(),view_sha256=f.view.digest())
    directory=stack.directory/name;directory.mkdir()
    from luggage_perception.ros_suction_domain import domain_to_message
    from luggage_perception.pickup_view_service import view_to_message
    (directory/'domain.cdr').write_bytes(serialize_message(domain_to_message(f.domain)))
    (directory/'view.cdr').write_bytes(serialize_message(view_to_message(f.view)))
    (directory/'proposal.json').write_text(json.dumps(asdict(f.proposal),indent=2))
    return f,directory


def run_case(stack,owner,f,directory,**settings):
    (directory/'scene.cdr').write_bytes(f.source.scene_cdr)
    (directory/'actual_proposal.json').write_text(json.dumps(asdict(f.proposal),indent=2))
    (directory/'start.json').write_text(json.dumps(dict(joints=f.start,ik_seeds=(f.start,f.goal),
        model_sha256=stack.model.model_sha256,scope='synthetic native fixture, not deployment sensor accuracy')))
    derived=pickup_view_scene(f.source,f.domain,'pickup_box',admit_static_scene=static_admission)
    (directory/'view_scene.cdr').write_bytes(derived.scene_cdr)
    result=prepare(owner,f,request_id='view-'+'_'.join(directory.relative_to(stack.directory).parts),
        deadline=time.monotonic()+8.,**settings)
    (directory/'result.json').write_text(json.dumps(asdict(result),indent=2))
    return result


def test_complete_camera_view_path_uses_fixed_mount_full_scene_and_independent_native_certificate(view_stack):
    stack,owner=view_stack;f,directory=scenario(stack,'healthy')
    original=f.source.scene_cdr
    result=run_case(stack,owner,f,directory)
    assert result.reason=='VIEW_PATH_PREPARED' and result.plan is not None,result
    plan=result.plan
    assert plan.base_snapshot_identity==f.source.identity and f.source.scene_cdr==original
    camera=Transform.from_matrix(stack.model.urdf.tcp(plan.leg.knots[-1].positions,stack.model.urdf.path_to('actual_optical')))
    assert np.linalg.norm(np.array(camera.translation)-f.proposal.world_from_optical.translation)<=stack.model.position_tolerance_m
    assert plan.certificate_id and plan.route_digest and plan.leg.knots[-1].time_from_start_ns>0
    assert [stage[0] for stage in result.trace]==['ik','connection','certification']
    assert stack.records and any(r['port']=='view_certificate' and r['kind'].endswith('response') for r in stack.records)


@pytest.mark.parametrize('obstacle',['goal','start'])
def test_full_private_scene_obstacle_prevents_view_motion(view_stack,obstacle):
    stack,owner=view_stack;f,directory=scenario(stack,'blocked_'+obstacle)
    # Blocking native tool states demonstrates scene consumption. A tiny J6
    # change leaves the spherical tool centre nearly unchanged, so for the goal
    # control first choose a distinct reachable camera target by moving J1.
    f.goal=(f.goal[0]+.4,*f.goal[1:])
    camera=Transform.from_matrix(stack.model.urdf.tcp(f.goal,stack.model.urdf.path_to('actual_optical')))
    f.proposal=replace(f.proposal,world_from_optical=camera)
    control=directory/'control';control.mkdir()
    control_result=run_case(stack,owner,f,control)
    assert control_result.plan is not None,control_result
    q=f.goal if obstacle=='goal' else f.start
    pose=Transform.from_matrix(stack.model.urdf.tcp(q,stack.model.path))
    scene=deserialize_message(f.source.scene_cdr,PlanningScene)
    scene.world.collision_objects.append(_box('source_obstacle',(.15,.15,.15),pose,'world'))
    f.source=replace(f.source,scene_cdr=serialize_message(scene))
    result=run_case(stack,owner,f,directory)
    assert result.plan is None and result.reason=='VIEW_SEARCH_INCOMPLETE',result
    assert any(row[0]==('ik' if obstacle=='goal' else 'connection') for row in result.trace),result.trace


def test_unreachable_camera_target_cannot_produce_a_plan(view_stack):
    stack,owner=view_stack;f,directory=scenario(stack,'unreachable')
    f.proposal=replace(f.proposal,world_from_optical=Transform((30.,0.,2.),f.proposal.world_from_optical.quaternion_xyzw))
    result=run_case(stack,owner,f,directory)
    assert result.plan is None and result.reason=='VIEW_SEARCH_INCOMPLETE',result
    assert all(row[0]=='ik' for row in result.trace)


def test_source_changed_after_native_certificate_discards_the_complete_view_path(view_stack):
    stack,owner=view_stack;f,directory=scenario(stack,'revoked_source')
    reads=[]
    def current(source):reads.append(source.identity);return len(reads)==1
    result=run_case(stack,owner,f,directory,source_current=current)
    assert result.plan is None and result.reason=='VIEW_SOURCE_CHANGED',result
    assert any(row[0]=='certification' and row[-1]=='NATIVE_VIEW_PATH_CHECKED' for row in result.trace)
    assert len(reads)==2


@pytest.mark.parametrize('derivative',['velocity','acceleration'])
def test_native_derivative_limit_refusal_cannot_issue_a_view_plan(view_stack,derivative):
    stack,owner=view_stack;f,directory=scenario(stack,derivative+'_refused')
    original=owner.certifier
    # A deliberately incompatible certification policy tests the real native
    # timing gate; it is a fixture fault, not a proposed robot limit/default.
    port=NativeViewCertificationPort(stack.node,stack.model,
        velocity_limits=(.0001 if derivative=='velocity' else .314,)*6,
        acceleration_limits=(.0001 if derivative=='acceleration' else .314,)*6,max_samples=50000,
        recorder=lambda kind,raw:stack.record('view_limits',kind,raw))
    assert port.client.wait_for_service(timeout_sec=5.)
    owner.certifier=port
    try:
        result=run_case(stack,owner,f,directory)
        assert result.plan is None and result.reason=='VIEW_SEARCH_INCOMPLETE',result
        assert any(row[0]=='certification' and row[-1].startswith('NATIVE_CERTIFICATION_REFUSED:')
            for row in result.trace),result.trace
        from luggage_msgs.srv import ValidateRouteV2
        responses=[r for r in stack.records if r['port']=='view_limits' and r['kind'].endswith('response')]
        response=deserialize_message((stack.directory/responses[-1]['file']).read_bytes(),ValidateRouteV2.Response)
        assert any(leg.reason==derivative.upper()+'_LIMIT' for leg in response.leg_results)
    finally:owner.certifier=original
