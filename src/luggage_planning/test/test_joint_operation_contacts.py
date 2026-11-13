"""Real perception quality witnesses consumed as immutable joint-plan grasps."""
from dataclasses import asdict, replace
import json
import math
import os
from pathlib import Path
import threading
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest
from luggage_msgs.msg import DirectedSuctionQueryV2
from luggage_msgs.srv import EvaluateDirectedSuctionV2
from rclpy.serialization import deserialize_message, serialize_message

from luggage_perception.directed_suction_quality import DirectedSuctionContext
from luggage_perception.directed_suction_service import DirectedSuctionService
from luggage_perception.ros_suction_domain import domain_to_message
from luggage_planning.joint_operation.contacts import (
    contact_query, contact_evidence, grasp_for_contact, DirectedContactPort,
)
from luggage_planning.placement_prior.contracts import Transform
from test_directed_suction_quality import context_inputs
from test_directed_suction_service import request_and_context


def evaluate(request, context):
    handler = DirectedSuctionService(clock=lambda:0.)
    handler.replace_context(context)
    request = deserialize_message(serialize_message(request), EvaluateDirectedSuctionV2.Request)
    response = handler.handle(request, EvaluateDirectedSuctionV2.Response())
    return deserialize_message(serialize_message(response), EvaluateDirectedSuctionV2.Response())


@pytest.fixture
def consume(request):
    counter = 0
    def run(wire, query, context):
        nonlocal counter
        root = os.environ.get('JOINT_CONTACT_CONSUMER_EVIDENCE')
        if root:
            p = Path(root)/request.node.name/str(counter)
            p.mkdir(parents=True, exist_ok=False)
            (p/'request.cdr').write_bytes(serialize_message(query))
            (p/'response.cdr').write_bytes(serialize_message(wire))
            (p/'domain.cdr').write_bytes(serialize_message(domain_to_message(context.domain)))
            (p/'policy.json').write_text(json.dumps(context._model.as_dict(), indent=2))
            # Synthetic measurement stage boundary; no simulator/GT input.
            hmap = context._height_map
            np.savez_compressed(p/'quality_samples.npz', **{
                name:value for name,value in hmap.__dict__.items() if isinstance(value,np.ndarray)})
        counter += 1
        return contact_evidence(wire, query, context.domain, context._model)
    return run


def test_real_service_cdr_yields_one_contact_and_preserves_refusal_diagnostics(consume):
    original, context = request_and_context()
    query = contact_query(context.domain, original.detection, original.queries, context._model,
        budget_s=1., expected_policy_sha256=context.policy_sha256)
    original.detection.width = 99.  # Request is an independent snapshot.
    assert query.detection.width == context.domain.box_wdh[0]
    response = evaluate(query, context)
    result = consume(response, query, context)
    assert result.complete and len(result.contacts) == 1
    assert [row[1] for row in result.verdicts] == [True, False]
    assert result.contacts[0].world_from_contact.translation == pytest.approx((.0013, -.0027, 1.))
    assert result.quality_policy_sha256 == context.policy_sha256
    assert result.samples_sha256 == context.samples_sha256


@pytest.mark.parametrize('box_yaw', [0., math.pi/2, math.pi, -math.pi/2])
@pytest.mark.parametrize('uv', [(0.,0.),(.0413,0.),(-.0413,0.),(0.,.0227),(0.,-.0227)])
@pytest.mark.parametrize('height', [.2, .4])
def test_directed_grasp_same_G_at_both_ends_for_measured_offsets_heights_and_box_yaws(
        consume, box_yaw, uv, height):
    domain, hmap, model = context_inputs()
    c, s = math.cos(box_yaw), math.sin(box_yaw)
    domain = replace(domain, box_xyz_world=(0.,0.,1.-height/2),
        box_quaternion_xyzw=(0.,0.,math.sin(box_yaw/2),math.cos(box_yaw/2)),
        box_wdh=(.7,.45,height), u_axis_world=(c,s,0.), v_axis_world=(-s,c,0.))
    hmap.u_axis, hmap.v_axis = np.array(domain.u_axis_world), np.array(domain.v_axis_world)
    context = DirectedSuctionContext(domain,hmap,model)
    request, _ = request_and_context()
    box = request.detection
    box.pose.position.z = domain.box_xyz_world[2]
    box.pose.orientation.z, box.pose.orientation.w = domain.box_quaternion_xyzw[2:]
    box.height = height
    request.queries = [DirectedSuctionQueryV2(query_id=str(i),center_uv_top_m=list(uv),yaw_rad=yaw)
        for i,yaw in enumerate((0.,math.pi/2,math.pi,-math.pi/2))]
    query = contact_query(domain,box,request.queries,model,budget_s=1.)
    result = consume(evaluate(query,context),query,context)
    assert result.complete and len(result.contacts) == 4, result.reason
    for contact in result.contacts:
        grasp = grasp_for_contact(domain,contact,observation_id='observed',rank=0)
        world_box = Transform(domain.box_xyz_world,domain.box_quaternion_xyzw)
        recovered = world_box.compose(grasp.box_from_tcp)
        assert recovered.translation == pytest.approx(contact.world_from_contact.translation,abs=1e-12)
        assert abs(sum(a*b for a,b in zip(recovered.quaternion_xyzw,contact.world_from_contact.quaternion_xyzw))) == pytest.approx(1.)
        for target_yaw in (0.,math.pi/2,math.pi,-math.pi/2):
            target_box = Transform((.4,-.8,1.2),(0.,0.,math.sin(target_yaw/2),math.cos(target_yaw/2)))
            target_tool = target_box.compose(grasp.box_from_tcp)
            restored = target_tool.compose(grasp.box_from_tcp.inverse())
            assert restored.translation == pytest.approx(target_box.translation,abs=1e-12)
            assert abs(sum(a*b for a,b in zip(restored.quaternion_xyzw,target_box.quaternion_xyzw))) == pytest.approx(1.)
        assert grasp.pick_references[2] == contact.world_from_contact
        assert all(ref.quaternion_xyzw == contact.world_from_contact.quaternion_xyzw for ref in grasp.pick_references)


def test_tilted_quality_normal_survives_grasp_and_conservative_attachment(consume):
    from luggage_planning.planning_contract.attachment_geometry import freeze_geometry
    from luggage_planning.planning_contract.payload_envelope import EnvelopeLimits
    domain,hmap,model = context_inputs()
    angle = .02
    domain = replace(domain, v_axis_world=(0.,math.cos(angle),math.sin(angle)),
        normal_world=(0.,-math.sin(angle),math.cos(angle)))
    hmap.v_axis,hmap.normal = np.array(domain.v_axis_world),np.array(domain.normal_world)
    hmap.local_normal[:] = domain.normal_world
    context = DirectedSuctionContext(domain,hmap,model)
    request,_ = request_and_context()
    query = contact_query(domain,request.detection,request.queries[:1],model,budget_s=1.)
    result = consume(evaluate(query,context),query,context)
    assert result.complete and len(result.contacts) == 1
    contact = result.contacts[0]
    grasp = grasp_for_contact(domain,contact,observation_id='obs',rank=0)
    normal = np.array(domain.normal_world)
    for ref,distance in zip(grasp.pick_references,(.30,.25,0.,.35)):
        assert np.array(ref.translation)-contact.world_from_contact.translation == pytest.approx(normal*distance,abs=1e-12)
    t = grasp.box_from_tcp.inverse()
    geometry = freeze_geometry(domain.box_wdh,t.translation,t.quaternion_xyzw,
        EnvelopeLimits(5.,(.1,.1,.1),(0.,0.),.005))
    assert geometry.t_tool_box_quaternion_xyzw == t.quaternion_xyzw
    damaged = replace(contact,world_from_contact=Transform((99.,0.,0.),(0.,0.,0.,1.)))
    with pytest.raises(ValueError,match='receipt'):
        grasp_for_contact(domain,damaged,observation_id='obs',rank=0)


@pytest.mark.parametrize('damage', ['domain','stamp','frame','policy','samples','query_id','query_uv',
    'query_yaw','order','missing','pose','metrics','digest','refinements','elapsed','negative_pose',
    'bad_matrix_rehashed','moved_pose_rehashed','rotated_pose_rehashed'])
def test_changed_witness_never_exposes_partial_contacts(consume, damage):
    query,context = request_and_context()
    query.expected_quality_policy_sha256 = context.policy_sha256
    response = evaluate(query,context)
    first = response.contacts[0]
    if damage == 'domain': response.domain_sha256 = '0'*64
    elif damage == 'stamp': response.header.stamp.nanosec += 1
    elif damage == 'frame': response.header.frame_id = 'base_link'
    elif damage == 'policy': response.quality_policy_sha256 = '0'*64
    elif damage == 'samples': response.samples_sha256 = '0'*64
    elif damage == 'query_id': first.query_id = 'other'
    elif damage == 'query_uv': first.center_uv_top_m[0] += .001
    elif damage == 'query_yaw': first.yaw_rad += .1
    elif damage == 'order': response.contacts = list(reversed(response.contacts))
    elif damage == 'missing': response.contacts = response.contacts[:1]
    elif damage == 'pose': first.world_from_contact_frame[3] += .001
    elif damage == 'metrics': first.quality_metrics_json = '{}'
    elif damage == 'digest': first.evidence_sha256 = '0'*64
    elif damage == 'refinements': first.refinement_count = 4
    elif damage == 'elapsed': response.elapsed_s = query.budget_s
    elif damage == 'negative_pose': response.contacts[1].world_from_contact_frame[0] = 1.
    else:
        valid = consume(response,query,context).contacts[0].quality
        matrix = np.array(valid.world_from_contact_frame).reshape(4,4)
        if damage == 'bad_matrix_rehashed': matrix[0,0] = 2.
        elif damage == 'moved_pose_rehashed': matrix[0,3] += .02
        else: matrix[:3,:2] *= -1.
        altered = replace(valid,world_from_contact_frame=tuple(matrix.ravel()))
        first.world_from_contact_frame = list(altered.world_from_contact_frame)
        first.evidence_sha256 = altered.evidence_digest()
    result = consume(response,query,context)
    assert not result.complete and not result.contacts and not result.verdicts


def test_missing_observed_support_refuses_even_a_rehashed_quality_witness(consume):
    query,context = request_and_context()
    response = evaluate(query,context)
    valid = consume(response,query,context).contacts[0].quality
    support = bytearray(context.domain.support)
    support[70*90+45] = 0
    changed = replace(context.domain,support=bytes(support))
    query.domain_sha256 = response.domain_sha256 = changed.digest()
    altered = replace(valid,domain_sha256=changed.digest())
    response.contacts = response.contacts[:1]; query.queries = query.queries[:1]
    response.contacts[0].evidence_sha256 = altered.evidence_digest()
    changed_context = DirectedSuctionContext(changed,context._height_map,context._model)
    result = consume(response,query,changed_context)
    assert not result.complete and result.reason.startswith('CONTACT_QUALITY_SUPPORT_MISMATCH')


def test_all_negative_batch_is_a_complete_evaluation(consume):
    query,context = request_and_context()
    query.queries = query.queries[1:]
    result = consume(evaluate(query,context),query,context)
    assert result.complete and not result.contacts and len(result.verdicts) == 1


def test_busy_cancelled_and_invalid_deadline_ports_submit_nothing():
    port = object.__new__(DirectedContactPort)
    port.inflight = NS(done=lambda:False)
    kwargs = dict(deadline=time.monotonic()+1.,cancelled=lambda:False)
    assert port.evaluate(None,None,None,None,**kwargs).reason == 'CONTACT_QUALITY_BACKEND_BUSY'
    kwargs['cancelled'] = lambda:True
    assert port.evaluate(None,None,None,None,**kwargs).reason == 'CANCELLED_OR_DEADLINE'
    kwargs['deadline'] = float('nan')
    assert port.evaluate(None,None,None,None,**kwargs).reason == 'INVALID_CONTACT_QUALITY_DEADLINE'


@pytest.mark.parametrize('axis', [(1.,0.,0.),(0.,1.,0.),(0.,0.,1.),(1.,2.,3.)])
@pytest.mark.parametrize('angle', [0., .2, math.pi-1e-9, math.pi])
def test_measured_matrix_conversion_preserves_rotation_near_each_half_turn(axis,angle):
    from luggage_description.frame_transforms import rotate_vector
    axis = np.array(axis)/np.linalg.norm(axis)
    cross = np.array(((0.,-axis[2],axis[1]),(axis[2],0.,-axis[0]),(-axis[1],axis[0],0.)))
    matrix = np.eye(4)
    matrix[:3,:3] = np.eye(3)*math.cos(angle)+(1.-math.cos(angle))*np.outer(axis,axis)+math.sin(angle)*cross
    matrix[:3,3] = (.1,-.2,.3)
    result = Transform.from_matrix(matrix)
    for basis in np.eye(3):
        assert rotate_vector(result.quaternion_xyzw,tuple(float(v) for v in basis)) == pytest.approx(matrix[:3,:3]@basis,abs=1e-12)
    assert result.translation == (.1,-.2,.3)


@pytest.mark.parametrize('damage', ['reflection','scale','last_row','nan'])
def test_matrix_conversion_does_not_repair_invalid_measurements(damage):
    matrix = np.eye(4)
    if damage == 'reflection': matrix[0,0] = -1.
    elif damage == 'scale': matrix[1,1] = 1.01
    elif damage == 'last_row': matrix[3,0] = .1
    else: matrix[0,3] = float('nan')
    with pytest.raises(ValueError,match='proper finite'):
        Transform.from_matrix(matrix)


def test_late_backend_retains_inflight_slot_until_terminal_and_cannot_leak_contact():
    from concurrent.futures import Future
    query,context = request_and_context()
    calls, late = [], Future()
    def call(request):
        calls.append(request)
        if len(calls) == 1:
            return late
        done = Future()
        done.set_result(evaluate(request,context))
        return done
    client = NS(service_is_ready=lambda:True,call_async=call)
    port = DirectedContactPort(NS(create_client=lambda *_:client))
    def run():
        return port.evaluate(context.domain,query.detection,query.queries,context._model,
            deadline=time.monotonic()+.02,cancelled=lambda:False)
    timed_out = run()
    assert not timed_out.complete and not timed_out.contacts
    assert timed_out.reason == 'CANCELLED_OR_DEADLINE'
    assert run().reason == 'CONTACT_QUALITY_BACKEND_BUSY' and len(calls) == 1
    late.set_result(evaluate(calls[0],context))
    recovered = run()
    assert recovered.complete and len(recovered.contacts) == 1 and len(calls) == 2


@pytest.mark.skipif(not os.environ.get('JOINT_CONTACT_CONSUMER_ROS_DOMAIN_ID'),reason='isolated ROS transport opt-in')
def test_native_port_consumes_service_and_terminates_executor():
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import MultiThreadedExecutor
    root = Path(os.environ['JOINT_CONTACT_CONSUMER_NATIVE_EVIDENCE'])
    root.mkdir(parents=True,exist_ok=False)
    query,snapshot = request_and_context()
    (root/'domain.cdr').write_bytes(serialize_message(domain_to_message(snapshot.domain)))
    (root/'policy.json').write_text(json.dumps(snapshot._model.as_dict(),indent=2))
    np.savez_compressed(root/'quality_samples.npz',**{
        key:value for key,value in snapshot._height_map.__dict__.items() if isinstance(value,np.ndarray)})
    handler = DirectedSuctionService(); handler.replace_context(snapshot)
    context = Context()
    rclpy.init(context=context,domain_id=int(os.environ['JOINT_CONTACT_CONSUMER_ROS_DOMAIN_ID']))
    server = rclpy.create_node('joint_contact_server',context=context)
    node = rclpy.create_node('joint_contact_consumer',context=context)
    executor = MultiThreadedExecutor(num_threads=2,context=context)
    executor.add_node(server); executor.add_node(node)
    server.create_service(EvaluateDirectedSuctionV2,'/luggage/evaluate_directed_suction_v2',handler.handle)
    case = ['positive']
    def record(kind,raw): (root/(case[0]+'_'+kind+'.cdr')).write_bytes(raw)
    port = DirectedContactPort(node,recorder=record)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    try:
        assert port.client.wait_for_service(timeout_sec=3.)
        for name,available in [('positive',True),('missing_context',False)]:
            case[0] = name
            if not available: handler.replace_context(None)
            result = port.evaluate(snapshot.domain,query.detection,query.queries,snapshot._model,
                deadline=time.monotonic()+2.,cancelled=lambda:False,expected_policy_sha256=snapshot.policy_sha256)
            (root/(name+'_result.json')).write_text(json.dumps(asdict(result),indent=2))
            assert result.complete is available
            assert len(result.contacts) == (1 if available else 0)
    finally:
        executor.shutdown(timeout_sec=3.); thread.join(timeout=3.)
        server.destroy_node(); node.destroy_node(); context.shutdown()
        terminal = not context.ok() and not thread.is_alive()
        (root/'teardown.json').write_text(json.dumps(dict(context_terminal=terminal,external_processes_launched=0)))
        assert terminal
