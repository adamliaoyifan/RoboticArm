"""Generated and actual ROS source-bound view queries; no robot ports."""
import copy
from dataclasses import replace
import hashlib
import json
import os
from pathlib import Path
import threading
import time

import pytest

from luggage_msgs.srv import GetPickupViewEvidenceV2
from luggage_perception.directed_suction_service import DirectedSuctionService
from luggage_perception.pickup_view_service import PickupViewService,view_from_message
from luggage_perception.suction_acquisition_history import SuctionAcquisitionHistory
from test_pickup_view_evidence import view_context


@pytest.fixture
def fixture(request):
    original,context,_,_=view_context()
    now=[0.]
    owner=DirectedSuctionService(acquisition_history=SuctionAcquisitionHistory(lifetime_s=1.,clock=lambda:now[0]))
    owner.replace_context(context)
    handler=PickupViewService(context_lookup=owner.context_snapshot)
    directory=None
    if os.environ.get('PICKUP_VIEW_EVIDENCE_DIR'):
        directory=Path(os.environ['PICKUP_VIEW_EVIDENCE_DIR'])/request.node.name;directory.mkdir(parents=True)
    query=GetPickupViewEvidenceV2.Request(domain_sha256=context.domain_sha256,
        detection=original.detection,generation=context.domain.generation)
    records=[]
    def capture(name,message):
        from rclpy.serialization import serialize_message
        raw=serialize_message(message)
        records.append(dict(name=name,sha256=hashlib.sha256(raw).hexdigest(),bytes=len(raw)))
        if directory is not None:(directory/(name+'.cdr')).write_bytes(raw)
    def call(message):
        from rclpy.serialization import serialize_message,deserialize_message
        capture('request_'+str(len(records)),message)
        result=handler.get(deserialize_message(serialize_message(message),GetPickupViewEvidenceV2.Request),
            GetPickupViewEvidenceV2.Response())
        capture('response_'+str(len(records)),result)
        return deserialize_message(serialize_message(result),GetPickupViewEvidenceV2.Response)
    from types import SimpleNamespace
    yield SimpleNamespace(owner=owner,handler=handler,context=context,now=now,query=query,
                          call=call,capture=capture,directory=directory)
    if directory is not None:(directory/'manifest.json').write_text(json.dumps(dict(records=records),indent=2))


def test_exact_old_acquisition_survives_newer_publication_without_renewal(fixture):
    f=fixture
    _,new,_,_=view_context(stamp_ns=f.context.domain.stamp_ns+1)
    f.owner.replace_context(new)
    result=f.call(f.query)
    assert result.success,result.reason
    assert view_from_message(result.view,f.context.domain)==f.context.pickup_view
    assert f.owner.context_snapshot() is new
    f.now[0]=1.
    refused=f.call(f.query)
    assert not refused.success and not refused.view.evidence_sha256 and not refused.view.depth_mm.data


@pytest.mark.parametrize('damage',['missing','stamp','generation','dimensions','no_view','revoked','identity'])
def test_refusal_has_no_partial_view_or_latest_fallback(fixture,damage):
    f=fixture;query=copy.deepcopy(f.query)
    if damage=='missing':query.domain_sha256='0'*64
    elif damage=='stamp':query.detection.header.stamp.nanosec+=1
    elif damage=='generation':query.generation+=1
    elif damage=='dimensions':query.detection.width+=.001
    elif damage=='no_view':
        f.owner.replace_context(None)
        f.owner.replace_context(f.context.without_pickup_view('PICKUP_VIEW_NOT_CAPTURED'))
    elif damage=='revoked':f.owner.replace_context(None)
    else:
        domain=replace(f.context.domain,instance_id='other')
        f.owner._history.remember(domain)
    result=f.call(query)
    assert not result.success and result.reason.startswith('PICKUP_VIEW_REFUSED:')
    assert not result.view.evidence_sha256 and not result.view.depth_mm.data


def test_revocation_during_payload_copy_cannot_return_partial_evidence(fixture):
    f=fixture;calls=[]
    def lookup(digest):
        calls.append(digest)
        return f.context if len(calls)==1 else None
    f.handler.context_lookup=lookup
    result=f.call(f.query)
    assert not result.success and 'SOURCE_CHANGED_DURING_READ' in result.reason
    assert not result.view.evidence_sha256 and not result.view.depth_mm.data


def test_view_only_contradiction_refuses_view_but_keeps_quality_source(fixture):
    from luggage_perception.directed_suction_quality import DirectedSuctionContext
    f=fixture
    changed=replace(f.context.pickup_view.acquisition,region_source='detection_bbox').bind(f.context.domain)
    other=DirectedSuctionContext(f.context.domain,f.context._height_map,f.context._model,pickup_view=changed)
    f.owner.replace_context(other)
    result=f.call(f.query)
    assert not result.success and 'PICKUP_VIEW_CONFLICTING_ACQUISITION' in result.reason
    assert not result.view.depth_mm.data
    retained=f.owner.context_snapshot(f.context.domain_sha256)
    assert retained is not None and retained.samples_sha256==f.context.samples_sha256


def test_real_ros_client_server_returns_bound_payload_and_terminal_teardown(fixture):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    f=fixture;context=rclpy.context.Context()
    rclpy.init(context=context,domain_id=179)
    server=rclpy.create_node('pickup_view_test_owner',context=context)
    client_node=rclpy.create_node('pickup_view_test_client',context=context)
    service=server.create_service(GetPickupViewEvidenceV2,'/test/get_pickup_view_evidence',f.handler.get)
    client=client_node.create_client(GetPickupViewEvidenceV2,'/test/get_pickup_view_evidence')
    executor=SingleThreadedExecutor(context=context);executor.add_node(server);executor.add_node(client_node)
    worker=threading.Thread(target=executor.spin);worker.start()
    try:
        assert client.wait_for_service(timeout_sec=3.)
        _,new,_,_=view_context(stamp_ns=f.context.domain.stamp_ns+1)
        f.owner.replace_context(new)
        def call(name,query):
            f.capture('ros_'+name+'_request',query)
            future=client.call_async(query);deadline=time.monotonic()+3.
            while not future.done() and time.monotonic()<deadline:time.sleep(.01)
            assert future.done()
            response=future.result();f.capture('ros_'+name+'_response',response)
            return response
        response=call('exact',f.query)
        assert response.success,response.reason
        assert view_from_message(response.view,f.context.domain)==f.context.pickup_view
        for fault in ('missing','generation','expired'):
            query=copy.deepcopy(f.query)
            if fault=='missing':query.domain_sha256='0'*64
            elif fault=='generation':query.generation+=1
            else:f.now[0]=1.
            refused=call(fault,query)
            assert not refused.success and not refused.view.evidence_sha256 and not refused.view.depth_mm.data
    finally:
        executor.shutdown(timeout_sec=3.);worker.join(timeout=3.)
        client_node.destroy_client(client);server.destroy_service(service)
        server.destroy_node();client_node.destroy_node();context.shutdown()
        if f.directory is not None:(f.directory/'teardown.json').write_text(json.dumps(dict(worker_stopped=not worker.is_alive())))
    assert not worker.is_alive()
