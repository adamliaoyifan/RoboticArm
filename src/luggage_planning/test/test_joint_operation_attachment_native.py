"""Real O preparation/frozen owner + actual vacuum adapter + provider freeze.

Actuator is synthetic. No controller or vacuum hardware is addressed. The
qualification positive is not native admission of a changed actual suffix.
"""
from dataclasses import asdict, replace
from pathlib import Path
from types import SimpleNamespace
import copy
import json
import os
import time

import pytest

from test_joint_operation_preparation_native import prepare_stack, native_stack
from test_joint_operation_observer_native import observer_owner

pytestmark = [
    pytest.mark.skipif(not os.environ.get('JOINT_NATIVE_ENDPOINT_EXECUTABLE'),
                      reason='explicit native stationary attachment gate'),
    pytest.mark.parametrize('prepare_stack', ['observer'], indirect=True),
]


@pytest.fixture
def seal_case(prepare_stack, request):
    from rclpy.serialization import serialize_message
    from luggage_msgs.srv import VacuumCommandV2
    from luggage_planning.attachment_provider import MeasuredAttachmentProvider
    from luggage_planning.attachment_frame_adapters import handle_vacuum_command_v2
    from luggage_planning.joint_operation.actual_contact import RetainedContactObservation
    from luggage_planning.joint_operation.attachment import StationaryOperationAttachmentPort
    from luggage_planning.place_service import pose_message
    from luggage_planning.placement_prior.contracts import Transform

    f = prepare_stack
    query = copy.deepcopy(f.request)
    query.request_id = request.node.name
    result = f.call(query)
    assert result.success, result.reason
    plan = f.root.issued.lookup(result.plan.plan_digest)
    observation = RetainedContactObservation(f.root.issued.contact_binding(plan), f.domain)
    directory = f.directory / request.node.name
    directory.mkdir()
    counter = []
    def record(name, data):
        path = directory / ('%03d_' % len(counter) + name)
        path.write_bytes(data)
        counter.append(path.name)
    provider = MeasuredAttachmentProvider(plan.identity.scene_revision, 'fixture-calibration')
    now = [f.domain.stamp_ns + 100_000_000]
    port = StationaryOperationAttachmentPort(provider, f.root.observed_contacts,
        certificate_current=f.root.issued.current, now_ns=lambda:now[0], recorder=record)
    enable = VacuumCommandV2.Request(enable=True, request_id=plan.identity.request_id,
        scene_revision=plan.identity.scene_revision, box_id=f.domain.box_id,
        generation=f.domain.generation, measured_box=copy.deepcopy(query.detection))
    box = plan.world_from_pick_box
    enable.box_pose = pose_message(dict(contract_version=2,kind='pose',frame_id='world',
        length_unit='m',stamp_ns=f.domain.stamp_ns,time_basis='acquisition',
        scene_revision=plan.identity.scene_revision,request_id=plan.identity.request_id,
        reference='box_center',orientation_semantics='box_axis_pi',
        position=box.translation,quaternion_xyzw=box.quaternion_xyzw))
    effects = []
    def response_for(world_tcp=None, stamp_ns=None, success=True):
        tcp = world_tcp or plan.world_from_pick_tool
        tool = dict(contract_version=2,kind='pose',frame_id='world',length_unit='m',
            stamp_ns=now[0] if stamp_ns is None else stamp_ns,time_basis='acquisition',
            scene_revision=plan.identity.scene_revision,request_id=plan.identity.request_id,
            reference='suction_contact_frame',orientation_semantics='tool_quaternion',
            position=tcp.translation,quaternion_xyzw=tcp.quaternion_xyzw)
        backend = SimpleNamespace(attach=lambda payload:(effects.append(payload) or success,'synthetic-actuator'))
        response = handle_vacuum_command_v2(enable, VacuumCommandV2.Response(),
            scene_revision=plan.identity.scene_revision,generation=f.domain.generation,
            resolve_tool=lambda _:tool,backend=backend,geometry_policy=f.root.limits)
        record('adapter_request.cdr',serialize_message(enable))
        record('adapter_response.cdr',serialize_message(response))
        record('actual_tool.json',json.dumps(tool,default=list).encode())
        assert response.success == success, response.message
        return response
    response = response_for()
    def qualify(reply=None, req=None):
        try:
            value = port.qualify(plan,observation,query.detection,req or enable,reply or response,
                deadline=time.monotonic()+5.,cancelled=lambda:False)
            record('record.json',json.dumps(value.record.to_dict(),sort_keys=True).encode())
            return value
        except Exception as error:
            record('refusal.json',json.dumps(dict(type=type(error).__name__,reason=str(error),
                record_present=provider.record_for(plan.identity.request_id) is not None)).encode())
            raise
    state = SimpleNamespace(f=f,plan=plan,observation=observation,detection=query.detection,
        provider=provider,port=port,request=enable,response=response,response_for=response_for,
        qualify=qualify,record=record,now=now,effects=effects)
    yield state
    record('case_manifest.json',json.dumps(dict(files=counter,box_stamp_ns=f.domain.stamp_ns,
        confirmation_clock_ns=now[0],plan_sha256=plan.plan_digest,map_backend='observer',
        hardware='synthetic actuator only; no controller',motion_effects=0,map_writes=0,
        release_effects=0)).encode())


@pytest.mark.parametrize('contact', ['planned', 'center'])
def test_real_stationary_seal_preserves_two_stamps_and_frozen_actual_footprint(seal_case, contact):
    from luggage_planning.placement_prior.contracts import Transform
    c = seal_case
    if contact == 'center':
        G = c.plan.world_from_pick_box.inverse().compose(c.plan.world_from_pick_tool)
        centered = Transform((0.,0.,c.f.domain.box_wdh[2]/2),G.quaternion_xyzw)
        c.response = c.response_for(c.plan.world_from_pick_box.compose(centered))
    actual = c.qualify()
    assert actual.record.source.route_grade
    assert actual.record.acquisition_stamp_ns == c.f.domain.stamp_ns
    assert actual.evidence.tcp_confirmation_stamp_ns == c.now[0] > actual.record.acquisition_stamp_ns
    assert actual.evidence.frozen_binding_sha256 == c.observation.binding.digest()
    assert actual.evidence.geometry_sha256 == actual.geometry.digest()
    assert actual.record.t_tcp_box_translation == actual.geometry.t_tool_box_translation
    assert c.qualify().record is actual.record
    assert c.provider.generation_for(c.plan.identity.request_id) == c.f.domain.generation
    # Changing a seal after freezing, even to another valid full-face contact,
    # does not change the existing record. Native target admission is separate.
    changed = c.response_for(stamp_ns=c.now[0]+1)
    c.now[0] += 1
    with pytest.raises(ValueError,match='SEALED_ATTACHMENT_REBIND_REFUSED'): c.qualify(changed)
    assert c.provider.latest_record(c.plan.identity.request_id) is actual.record


@pytest.mark.parametrize('damage', ['failed','unmeasured','request','scene','dimension',
    'future','reversed','stale','outside_face','wrong_request_box','wrong_generation','wrong_observation',
    'request_version','response_units','request_time_basis','response_time_basis'])
def test_real_seal_negative_never_freezes_a_record(seal_case, damage):
    from luggage_planning.placement_prior.contracts import Transform
    c = seal_case
    reply = copy.deepcopy(c.response)
    request = copy.deepcopy(c.request)
    if damage == 'failed': reply.success = False
    elif damage == 'unmeasured': reply.has_measured_attachment = False
    elif damage == 'request': reply.measured_grasp.identity.request_id = 'other'
    elif damage == 'scene': reply.measured_grasp.identity.scene_revision = 'other'
    elif damage == 'dimension': reply.attached_box_wdh[0] += .01
    elif damage == 'future': c.now[0] -= 1
    elif damage == 'reversed': reply.measured_grasp.identity.stamp_ns = c.f.domain.stamp_ns-1
    elif damage == 'stale': c.now[0] = c.f.domain.stamp_ns+c.provider.max_age_ns+1
    elif damage == 'outside_face':
        tcp = c.plan.world_from_pick_tool
        reply = c.response_for(Transform((tcp.translation[0]+.12,*tcp.translation[1:]),tcp.quaternion_xyzw))
    elif damage == 'wrong_request_box': request.box_id = 'other'
    elif damage == 'wrong_generation': request.generation += 1
    elif damage == 'wrong_observation': request.box_pose.identity.stamp_ns -= 1
    elif damage == 'request_version': request.box_pose.identity.contract_version = 1
    elif damage == 'response_units': reply.measured_grasp.identity.length_unit = 'mm'
    elif damage == 'request_time_basis': request.box_pose.identity.time_basis = 'scene'
    elif damage == 'response_time_basis': reply.measured_grasp.identity.time_basis = 'acquisition'
    c.record('damaged_case.json',json.dumps(dict(damage=damage,now_ns=c.now[0])).encode())
    from rclpy.serialization import serialize_message
    c.record('damaged_request.cdr',serialize_message(request))
    c.record('damaged_response.cdr',serialize_message(reply))
    with pytest.raises(ValueError): c.qualify(reply,request)
    assert c.provider.record_for(c.plan.identity.request_id) is None


@pytest.mark.parametrize('damage', ['revoked','model','expired'])
def test_actual_frozen_owner_revocation_before_seal_qualification_is_refused(seal_case, damage):
    c = seal_case
    old_clock = c.f.frozen.store.clock
    old_model = c.f.frozen.model_provider
    changed = copy.deepcopy(c.f.quality_context._model)
    changed.boundary_margin_m += .001
    try:
        if damage == 'revoked': c.f.frozen.store.revoke_all()
        elif damage == 'expired': c.f.frozen.store.clock = lambda:time.monotonic()+1000.
        else: c.f.frozen.model_provider = lambda:changed
        with pytest.raises(ValueError): c.qualify()
        assert c.provider.record_for(c.plan.identity.request_id) is None
    finally:
        c.f.frozen.store.clock = old_clock
        c.f.frozen.model_provider = old_model
