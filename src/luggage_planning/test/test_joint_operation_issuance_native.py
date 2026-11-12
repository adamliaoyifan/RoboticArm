"""Actual prepare/check RPC lineage; no controller or source-validity claim."""
from dataclasses import replace
import copy
import os
import time

import pytest

from test_joint_operation_preparation_native import prepare_stack, native_stack

pytestmark = pytest.mark.skipif(not os.environ.get('JOINT_NATIVE_ENDPOINT_EXECUTABLE'),
    reason='explicit real native issuance gate')


def check(fixture,message):
    from luggage_msgs.srv import CheckPreparedOperationV2
    from rclpy.serialization import serialize_message
    client=fixture.owner.create_client(CheckPreparedOperationV2,'/joint_operation/check_prepared')
    try:
        assert client.wait_for_service(timeout_sec=5.)
        request=CheckPreparedOperationV2.Request(plan=message)
        name='issuance_'+str(time.monotonic_ns())
        (fixture.directory/(name+'_request.cdr')).write_bytes(serialize_message(request))
        future=client.call_async(request);deadline=time.monotonic()+5.
        while not future.done() and time.monotonic()<deadline:time.sleep(.005)
        assert future.done(),'bounded check RPC did not return'
        response=future.result()
        (fixture.directory/(name+'_response.cdr')).write_bytes(serialize_message(response))
        return response
    finally:fixture.owner.destroy_client(client)


def prepare(fixture,label):
    request=copy.deepcopy(fixture.request);request.request_id='issued-'+label
    response=fixture.call(request)
    assert response.success,response.reason
    return response.plan


def test_real_prepare_issues_plan_and_readonly_check_retains_original(prepare_stack):
    fixture=prepare_stack;message=prepare(fixture,'positive')
    response=check(fixture,message)
    assert response.current and response.reason=='OWNER_ISSUANCE_CURRENT'
    assert fixture.root.issued.lookup(message.plan_digest).certificate_id==message.certificate_id


@pytest.mark.parametrize('damage',['certificate','identity','knots','stamp','resealed'])
def test_check_rpc_refuses_client_changes_even_when_client_reseals(prepare_stack,damage):
    from luggage_planning.joint_operation.adapters import operation_plan_to_message
    from luggage_planning.joint_operation.contracts import seal_operation_plan
    fixture=prepare_stack;original=prepare(fixture,damage);message=copy.deepcopy(original)
    if damage=='certificate':message.certificate_id='client'
    elif damage=='identity':message.generation+=1
    elif damage=='knots':message.legs[0].trajectory.trajectory.points[-1].positions[0]+=.01
    elif damage=='stamp':message.robot_state_stamp_valid=False
    else:
        retained=fixture.root.issued.lookup(message.plan_digest)
        forged=seal_operation_plan(replace(retained,certificate_id='client',plan_digest=''))
        message=operation_plan_to_message(forged,forged.identity,forged.plan_digest)
    assert not check(fixture,message).current
    assert check(fixture,original).current


def test_response_capture_failure_never_enters_owner_registry(prepare_stack):
    fixture=prepare_stack;original=fixture.root._record;captured=[]
    def fail(boundary,value):
        if boundary=='prepare_response' and value.success:
            captured.append(copy.deepcopy(value.plan))
            original('failed_issuance_candidate',value)
            raise OSError('fixture evidence device refusal')
        return original(boundary,value)
    fixture.root._record=fail
    request=copy.deepcopy(fixture.request);request.request_id='not-issued-capture'
    try:
        response=fixture.call(request)
        assert not response.success and not response.plan.legs and len(captured)==1
        assert not check(fixture,captured[0]).current
        assert not fixture.root._active.locked()
    finally:fixture.root._record=original


def test_failed_new_prepare_keeps_previous_issued_plan_current(prepare_stack):
    fixture=prepare_stack;message=prepare(fixture,'preserve')
    request=copy.deepcopy(fixture.request);request.request_id=message.request_id;request.budget_s=0.
    response=fixture.call(request)
    assert not response.success and not response.plan.legs
    assert check(fixture,message).current


def test_owner_restart_expiry_and_capacity_eviction_refuse(prepare_stack):
    from luggage_planning.joint_operation.issuance import IssuedOperationRegistry
    fixture=prepare_stack;retained=fixture.root.issued
    message=prepare(fixture,'expired')
    try:
        fixture.root.issued=IssuedOperationRegistry(max_plans=1,max_bytes=16*1024*1024,lifetime_s=5.)
        assert not check(fixture,message).current
        now=[10.];fixture.root.issued.clock=lambda:now[0]
        current=prepare(fixture,'new-owner')
        assert check(fixture,current).current
        next_message=prepare(fixture,'evicts-prior')
        assert not check(fixture,current).current and check(fixture,next_message).current
        now[0]=15.;assert not check(fixture,next_message).current
    finally:fixture.root.issued=retained


def test_successful_same_request_reprepare_retires_previous(prepare_stack):
    fixture=prepare_stack;message=prepare(fixture,'replacement')
    from luggage_perception.ros_suction_domain import domain_to_message
    from luggage_perception.directed_suction_quality import DirectedSuctionContext
    observed=replace(fixture.domain,generation=fixture.domain.generation+1)
    fixture.quality.replace_context(DirectedSuctionContext(observed,
        fixture.quality_context._height_map,fixture.quality_context._model))
    # The production detector invalidates evidence at this identity epoch.
    fixture.frozen.store.revoke_all()
    fixture.publishers[2].publish(domain_to_message(observed))
    try:
        deadline=time.monotonic()+2.
        while fixture.root.domain.generation!=observed.generation and time.monotonic()<deadline:time.sleep(.005)
        assert fixture.root.domain.generation==observed.generation
        request=copy.deepcopy(fixture.request);request.request_id=message.request_id
        request.generation=observed.generation
        response=fixture.call(request)
        assert response.success,response.reason
        assert response.plan.plan_digest!=message.plan_digest
        assert not check(fixture,message).current and check(fixture,response.plan).current
    finally:
        fixture.quality.replace_context(fixture.quality_context)
        fixture.publishers[2].publish(fixture.messages[2])
        deadline=time.monotonic()+2.
        while fixture.root.domain.generation!=fixture.domain.generation and time.monotonic()<deadline:time.sleep(.005)
        assert fixture.root.domain.generation==fixture.domain.generation


def test_issuance_failure_returns_inactive_response_and_keeps_owner_available(prepare_stack):
    fixture=prepare_stack;original=fixture.root.issued.issue
    def fail(plan,**kwargs):raise RuntimeError('fixture registry storage unavailable')
    fixture.root.issued.issue=fail
    request=copy.deepcopy(fixture.request);request.request_id='not-issued-registry'
    try:
        response=fixture.call(request)
        assert not response.success and not response.plan.legs
        assert response.reason=='PREPARATION_ISSUANCE_FAILED:RuntimeError'
        assert not fixture.root._active.locked()
    finally:fixture.root.issued.issue=original
