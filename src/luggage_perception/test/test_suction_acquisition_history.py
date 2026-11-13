"""Acquisition transport races: exact old evidence, never latest substitution."""
from dataclasses import replace
import threading
from types import SimpleNamespace

import pytest

from luggage_perception.directed_suction_quality import DirectedSuctionContext
from luggage_perception.directed_suction_service import DirectedSuctionService
from luggage_perception.frozen_suction_observation import FrozenSuctionObservations
from luggage_perception.frozen_suction_service import FrozenSuctionService
from luggage_perception.suction_acquisition_history import SuctionAcquisitionHistory
from luggage_msgs.srv import RetainSuctionObservationV2
from test_directed_suction_service import request_and_context


def successive_contexts():
    request, old = request_and_context()
    newer_domain = replace(old.domain, stamp_ns=old.domain.stamp_ns+198_000_000)
    newer = DirectedSuctionContext(newer_domain, old._height_map, old._model)
    return request, old, newer


def test_named_published_context_survives_next_frame_and_uses_original_samples():
    request, old, newer = successive_contexts()
    owner = DirectedSuctionService(clock=lambda:0.)
    owner.replace_context(old)
    owner.replace_context(newer)
    handler = FrozenSuctionService(
        FrozenSuctionObservations(max_records=3,max_bytes=8*1024**2,lifetime_s=10.,clock=lambda:0.),
        context_provider=owner.context_snapshot,context_lookup=owner.context_snapshot,
        model_provider=lambda:old._model,clock=lambda:0.)
    query = RetainSuctionObservationV2.Request(request_id='operation',observation_id='selected-view',
        domain_sha256=old.domain_sha256,detection=request.detection,generation=old.domain.generation)
    result = handler.retain(query,RetainSuctionObservationV2.Response())
    assert result.retained,result.reason
    assert result.binding.header.stamp.sec*10**9+result.binding.header.stamp.nanosec == old.domain.stamp_ns
    assert result.binding.domain_sha256 == old.domain_sha256
    assert result.binding.samples_sha256 == old.samples_sha256
    assert owner.context_snapshot() is newer
    owner.replace_context(None)
    query.request_id = 'other-operation'
    assert not handler.retain(query,RetainSuctionObservationV2.Response()).retained


def test_resource_eviction_expiry_and_identity_change_never_borrow_another_frame():
    _, old, newer = successive_contexts()
    now=[0.]
    history = SuctionAcquisitionHistory(max_records=1,lifetime_s=1.,clock=lambda:now[0])
    history.remember(old.domain,context=old)
    history.remember(newer.domain,context=newer)
    assert history.context_for_digest(old.domain_sha256) is None
    assert history.context_for_digest(newer.domain_sha256) is newer
    now[0]=.9
    history.remember(newer.domain,context=newer)
    now[0]=1.
    assert history.context_for_digest(newer.domain_sha256) is None
    history.remember(old.domain,context=old)
    history.remember(replace(newer.domain,generation=newer.domain.generation+1))
    assert history.context_for_digest(old.domain_sha256) is None
    small=SuctionAcquisitionHistory(max_bytes=1)
    small.remember(old.domain)
    assert small.domain_for_acquisition(small.acquisition_key(old.domain)) is None


def test_same_acquisition_with_conflicting_geometry_invalidates_handoff():
    _, old, _ = successive_contexts()
    history = SuctionAcquisitionHistory()
    history.remember(old.domain)
    with pytest.raises(ValueError,match='CONFLICTING_SUCTION_ACQUISITION'):
        history.remember(replace(old.domain,box_wdh=(.8,.5,.2)))
    assert history.domain_for_acquisition(history.acquisition_key(old.domain)) is None
    diagnostic=history.diagnostics()
    assert diagnostic['records']==[] and diagnostic['last_clear']['reason']=='CONFLICTING_ACQUISITION'
    assert diagnostic['last_clear']['discarded']==[history.acquisition_key(old.domain)]


def test_prepare_selects_requested_acquisition_despite_newer_topic_and_still_checks_geometry():
    from luggage_planning.joint_operation.prepare_node import JointOperationPrepareNode
    from luggage_planning.planning_contract.attachment import ATTACHMENT_MAX_AGE_NS
    request, old, newer = successive_contexts()
    history=SuctionAcquisitionHistory()
    history.remember(old.domain)
    history.remember(newer.domain)
    now=[newer.domain.stamp_ns]
    node=SimpleNamespace(domain=newer.domain,domain_history=history,
        joints=(13_000_000_000,(0.,)*6),_state_lock=threading.Lock(),
        preparation=SimpleNamespace(model=SimpleNamespace(identity_hash=old.domain.contact_model_hash)),
        _record=lambda *_:None,get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=now[0])),
        start_tolerance=.01)
    prepare_request=SimpleNamespace(detection=request.detection,generation=old.domain.generation,
        robot_state_stamp_valid=True,robot_state_stamp_ns=12_000_000_000,start_joints=(0.,)*6)
    assert JointOperationPrepareNode._current(node,prepare_request) is old.domain
    prepare_request.detection.width+=.002
    with pytest.raises(ValueError,match='independent detection disagree'):
        JointOperationPrepareNode._current(node,prepare_request)
    prepare_request.detection.width-=.002
    now[0]=old.domain.stamp_ns+ATTACHMENT_MAX_AGE_NS+1
    with pytest.raises(ValueError,match='OBSERVATION_AGE_REFUSED'):
        JointOperationPrepareNode._current(node,prepare_request)
    now[0]=newer.domain.stamp_ns
    history.clear()
    with pytest.raises(ValueError,match='OBSERVED_DOMAIN_UNAVAILABLE'):
        JointOperationPrepareNode._current(node,prepare_request)
