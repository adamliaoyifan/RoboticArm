"""Actual root/native preparation bound to perception-owned frozen evidence."""
from dataclasses import replace
import copy
import json
import os
import time

import pytest

from test_joint_operation_preparation_native import prepare_stack,native_stack
from test_joint_operation_issuance_native import check

pytestmark=pytest.mark.skipif(not os.environ.get('JOINT_NATIVE_ENDPOINT_EXECUTABLE'),
    reason='explicit real native prepared-observation gate')


def test_new_camera_frame_during_native_preparation_keeps_original_samples_and_binding(prepare_stack):
    from luggage_perception.directed_suction_quality import DirectedSuctionContext
    from luggage_perception.ros_suction_domain import domain_to_message
    from luggage_perception.frozen_suction_service import binding_from_message
    f=prepare_stack;f.state.scene_calls=0
    different=replace(f.domain,stamp_ns=f.domain.stamp_ns+100_000_000,
        support=bytes(len(f.domain.support)))
    new_context=DirectedSuctionContext(different,f.quality_context._height_map,f.quality_context._model)
    def update(count):
        if count!=1:return
        f.quality.replace_context(new_context)
        message=domain_to_message(different)
        from rclpy.serialization import serialize_message
        (f.directory/'new_camera_top_domain.cdr').write_bytes(serialize_message(message))
        f.publishers[2].publish(message)
        limit=time.monotonic()+1.
        while f.root.domain.digest()!=different.digest() and time.monotonic()<limit:time.sleep(.005)
        assert f.root.domain.digest()==different.digest()
    f.state.on_scene=update
    try:
        request=copy.deepcopy(f.request);request.request_id='frozen-new-camera'
        result=f.call(request)
        assert result.success,result.reason
        receipt=check(f,result.plan)
        assert receipt.current
        binding=binding_from_message(receipt.frozen_observation)
        assert binding==f.root.issued.contact_binding(f.root.issued.lookup(result.plan.plan_digest))
        assert binding.domain_sha256==f.domain.digest()
        assert binding.samples_sha256==f.quality_context.samples_sha256
        assert binding.samples_sha256!=new_context.samples_sha256
        assert binding.stamp_ns==f.domain.stamp_ns
        assert len(result.plan.legs)==6 and len(result.plan.barriers)==2
    finally:
        f.state.on_scene=None;f.quality.replace_context(f.quality_context)
        f.publishers[2].publish(f.messages[2])
        deadline=time.monotonic()+1.
        while f.root.domain.digest()!=f.domain.digest() and time.monotonic()<deadline:time.sleep(.005)
        assert f.root.domain.digest()==f.domain.digest()


@pytest.mark.parametrize('kind',['revoked','expired','model'])
def test_original_evidence_invalidated_after_native_certification_cannot_issue(prepare_stack,kind):
    f=prepare_stack;f.state.scene_calls=0
    original_clock=f.frozen.store.clock;original_model=f.frozen.model_provider
    changed_model=copy.deepcopy(f.quality_context._model);changed_model.boundary_margin_m+=.001
    def update(count):
        if count!=2:return
        (f.directory/('source_invalidation_'+kind+'.json')).write_text(json.dumps({
            'scene_call':count,'kind':kind,'clock_offset_s':1000. if kind=='expired' else 0.,
            'model':changed_model.as_dict() if kind=='model' else f.quality_context._model.as_dict()},indent=2))
        if kind=='revoked':f.frozen.store.revoke_all()
        elif kind=='expired':f.frozen.store.clock=lambda:time.monotonic()+1000.
        else:f.frozen.model_provider=lambda:changed_model
    f.state.on_scene=update
    try:
        request=copy.deepcopy(f.request);request.request_id='invalidated-frozen-'+kind
        result=f.call(request)
        assert f.state.scene_calls==2,'case must reach post-native validation'
        assert not result.success and not result.plan.legs
        assert not f.root._active.locked()
        assert all(entry.plan.identity.request_id!=request.request_id
            for entry in f.root.issued._entries.values())
    finally:
        f.state.on_scene=None;f.frozen.store.clock=original_clock;f.frozen.model_provider=original_model


def test_missing_frozen_quality_owner_never_falls_back_to_current_service(prepare_stack,monkeypatch):
    f=prepare_stack;request=copy.deepcopy(f.request);request.request_id='frozen-quality-missing'
    original=f.root.preparation.contacts.frozen_client.service_is_ready
    monkeypatch.setattr(f.root.preparation.contacts.frozen_client,'service_is_ready',lambda:False)
    before=f.state.scene_calls
    result=f.call(request)
    assert not result.success and not result.plan.legs
    assert f.state.scene_calls==before+1
    assert f.quality.context_snapshot() is f.quality_context


def test_forged_retention_reply_cannot_issue_a_plan(prepare_stack,monkeypatch):
    f=prepare_stack;original=f.root.observed_contacts._call
    def damaged(*a,**kw):
        from rclpy.serialization import serialize_message
        value=original(*a,**kw);value.binding.binding_sha256='forged'
        (f.directory/'injected_frozen_reply.cdr').write_bytes(serialize_message(value))
        return value
    monkeypatch.setattr(f.root.observed_contacts,'_call',damaged)
    before=f.state.scene_calls
    request=copy.deepcopy(f.request);request.request_id='forged-frozen'
    result=f.call(request)
    assert not result.success and not result.plan.legs
    assert f.state.scene_calls==before


def test_occ_change_while_remote_observation_check_is_pending_refuses_plan(prepare_stack,monkeypatch):
    from std_msgs.msg import String
    from rclpy.serialization import serialize_message
    f=prepare_stack;original=f.root.observed_contacts._call;changed=[]
    def update(client,request,**kw):
        value=original(client,request,**kw)
        if request.expected_binding.binding_sha256:
            record=json.loads(f.messages[1].data);record['pending_changes']=[{'event':'observed_change'}]
            message=String(data=json.dumps(record))
            (f.directory/'occ_during_retention_check.cdr').write_bytes(serialize_message(message))
            f.publishers[1].publish(message)
            limit=time.monotonic()+1.
            while f.root.store.current() is not None and time.monotonic()<limit:time.sleep(.005)
            assert f.root.store.current() is None
            changed.append(True)
        return value
    monkeypatch.setattr(f.root.observed_contacts,'_call',update)
    try:
        request=copy.deepcopy(f.request);request.request_id='late-frozen-check-occ'
        result=f.call(request)
        assert changed
        assert not result.success and not result.plan.legs
    finally:
        f.publishers[1].publish(f.messages[1])
        limit=time.monotonic()+1.
        while f.root.store.current() is None and time.monotonic()<limit:time.sleep(.005)
        assert f.root.store.current() is not None
