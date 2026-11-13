"""Named view consumption survives later refusal without becoming permission."""
import copy
from dataclasses import asdict
import json
from types import SimpleNamespace

import numpy as np
import pytest
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.convert import message_to_ordereddict

from luggage_msgs.srv import GetPickupViewEvidenceV2
from luggage_perception.pickup_view_service import PickupViewService, view_from_message, view_to_message
from pickup_continuation_fixtures import (
    case_archive, candidate, fixture_policy, planar_view,
)
from test_adopted_suction_capture import adopted, CapturedTop, CapturedEvaluation
from test_frozen_pickup_view import camera_case, retained
from test_pickup_view_service import (
    fixture as service_fixture,
    test_refusal_has_no_partial_view_or_latest_fallback as assert_existing_refusal,
)


def read_port(f):
    handler = PickupViewService(
        context_lookup=lambda digest:f.current.context if f.current.context.domain_sha256==digest else None,
        frozen_context_lookup=f.handler.pickup_context,
        on_read=lambda domain:f.detector.LuggageDetector._capture_selected_suction_input(f.node,domain))
    count = [0]
    def call(request):
        raw = serialize_message(request)
        index = count[0]; count[0] += 1
        (f.root/('view_%02d_request.cdr'%index)).write_bytes(raw)
        result = handler.get(deserialize_message(raw,GetPickupViewEvidenceV2.Request),
                             GetPickupViewEvidenceV2.Response())
        raw = serialize_message(result)
        (f.root/('view_%02d_response.cdr'%index)).write_bytes(raw)
        return deserialize_message(raw,GetPickupViewEvidenceV2.Response)
    return handler,call


def query(retention_request, binding=None):
    result = GetPickupViewEvidenceV2.Request(domain_sha256=retention_request.domain_sha256,
        detection=retention_request.detection,generation=retention_request.generation)
    if binding is not None:
        result.binding = binding
    return result


def test_unretained_named_views_pin_exact_inputs_and_bound_reads_deduplicate(adopted):
    f = adopted
    handler,call = read_port(f)
    request,context,depth,labels = f.stage(12_079_000_000)
    saved = f.retain(request)
    original = query(request,saved.binding)
    for i in range(16):
        f.stage(12_080_000_000+i)
        result = call(original)
        assert result.success
        assert view_from_message(result.view,context.domain)==context.pickup_view
    for stamp in (40_326_000_000,51_018_000_000):
        next_request,new,_,_ = f.stage(stamp)
        assert call(query(next_request)).success
        # No later retain or association success is necessary for this capture.
        assert len(f.store._entries)==1
    assert len(list((f.root/'inputs').glob('case-*')))==3
    captured = np.load(f.root/'inputs/case-000/inputs.npz')
    np.testing.assert_array_equal(captured['depth_mm'],depth)
    np.testing.assert_array_equal(captured['plane_labels'],labels)
    for stamp in (60_000_000_000,61_000_000_000):
        next_request,_,_,_ = f.stage(stamp)
        assert call(query(next_request)).success
    assert len(list((f.root/'inputs').glob('case-*')))==4
    assert f.warnings==['selected suction input capture: capture_limit']
    records=[json.loads(p.read_text()) for p in sorted((f.root/'inputs').glob('case-*/record.json'))]
    assert [r['stamp_ns'] for r in records]==[12_079_000_000,40_326_000_000,51_018_000_000,60_000_000_000]


def test_actual_ambiguous_association_does_not_lose_consumed_evaluator_input(adopted):
    from luggage_perception.stationary_pickup_association import associate_stationary_pickup
    from luggage_perception.ros_suction_domain import domain_to_message
    f = adopted
    _,original_context,_,_ = f.stage(12_000_000_000)
    before_domain,before = planar_view()
    domain,view = planar_view(newer=True,candidates=(candidate(),candidate()))
    acquisition = view.acquisition
    depth = np.frombuffer(acquisition.depth_mm,dtype=acquisition.depth_dtype).reshape(acquisition.shape)
    region = np.frombuffer(acquisition.instance_region,dtype=np.uint8).reshape(acquisition.shape).astype(bool)
    intr = SimpleNamespace(**dict(zip(('fx','fy','cx','cy'),acquisition.intrinsics_fx_fy_cx_cy)))
    labels = np.arange(depth.size,dtype=np.int32).reshape(depth.shape)
    f.capture.stage(depth,intr,np.asarray(acquisition.world_from_optical).reshape(4,4),
        CapturedTop(labels),region,original_context._model,
        CapturedEvaluation(domain.instance_id,domain.generation,domain.stamp_ns/1e9,'world'),
        stamp_ns=domain.stamp_ns)
    f.current.context = SimpleNamespace(domain=domain,domain_sha256=domain.digest(),
        pickup_view=view,pickup_view_reason='PICKUP_VIEW_AVAILABLE')
    request = GetPickupViewEvidenceV2.Request(domain_sha256=domain.digest(),generation=domain.generation)
    detection = request.detection
    detection.id = domain.box_id
    detection.header.frame_id = domain.frame_id
    detection.header.stamp.sec,detection.header.stamp.nanosec=divmod(domain.stamp_ns,10**9)
    detection.pose.position.x,detection.pose.position.y,detection.pose.position.z=domain.box_xyz_world
    detection.pose.orientation.x,detection.pose.orientation.y,detection.pose.orientation.z,detection.pose.orientation.w=domain.box_quaternion_xyzw
    detection.width,detection.depth,detection.height=domain.box_wdh
    detection.height_valid=detection.top_surface_valid=True
    detection.height_source=detection.HEIGHT_SOURCE_MEASURED_SUPPORT
    _,call = read_port(f)
    result = call(request)
    assert result.success
    after = view_from_message(result.view,domain)
    association = associate_stationary_pickup(before_domain,before,domain,after,
        settled_stamp_ns=2_000_000_000,stationary=True,policy=fixture_policy())
    assert not association.accepted and association.reason=='AMBIGUOUS_REPORTED_CANDIDATES'
    (f.root/'before_view.cdr').write_bytes(serialize_message(view_to_message(before)))
    (f.root/'before_domain.cdr').write_bytes(serialize_message(domain_to_message(before_domain)))
    (f.root/'after_domain.cdr').write_bytes(serialize_message(domain_to_message(domain)))
    (f.root/'association.json').write_text(json.dumps(asdict(association),indent=2))
    captured=np.load(f.root/'inputs/case-000/inputs.npz')
    np.testing.assert_array_equal(captured['depth_mm'],depth)
    np.testing.assert_array_equal(captured['plane_labels'],labels)
    assert len(list((f.root/'inputs').glob('case-*')))==1


@pytest.mark.parametrize('damage',['missing','stamp','generation','dimensions','no_view','revoked','identity'])
def test_failed_named_view_reads_never_select(service_fixture,damage):
    calls=[]
    service_fixture.handler.on_read=calls.append
    assert_existing_refusal(service_fixture,damage)
    assert calls==[]


@pytest.mark.parametrize('damage',['expiry','malformed','domain','revoked_during_copy'])
def test_invalid_bound_read_never_selects(camera_case,damage,monkeypatch):
    f=camera_case
    receipt,binding=retained(f)
    request=f.query(binding)
    calls=[]
    f.handler.on_read=calls.append
    if damage=='expiry':
        f.now[0]=10.
    elif damage=='malformed':
        request.binding.binding_sha256='invalid'
    elif damage=='domain':
        request.domain_sha256='0'*64
    else:
        import luggage_perception.pickup_view_service as module
        original=module.view_to_message
        def revoke(view):
            result=original(view)
            f.store.revoke_all()
            return result
        monkeypatch.setattr(module,'view_to_message',revoke)
    response=f.call('view',request)
    assert not response.success and not response.view.depth_mm.data
    assert calls==[]


@pytest.mark.parametrize('failure',['missing','evicted','io','callback','disabled'])
def test_capture_failure_does_not_change_successful_view_wire(adopted,failure,caplog):
    f=adopted
    request,context,_,_=f.stage(12_000_000_000)
    handler,call=read_port(f)
    expected=view_to_message(context.pickup_view)
    (f.root/'view_expected.cdr').write_bytes(serialize_message(expected))
    if failure=='missing':
        f.capture._pending.clear()
    elif failure=='evicted':
        for i in range(9):f.stage(12_000_000_001+i)
        f.current.context=context
    elif failure=='io':
        def broken(*args):raise OSError('diagnostic disk unavailable')
        f.capture.pin=broken
    elif failure=='callback':
        def broken(domain):raise OSError('optional callback failed')
        handler.on_read=broken
    else:
        handler.on_read=None
    result=call(query(request))
    assert result.success
    # CDR alignment padding is not a wire field. Compare every field and the
    # source-bound digest, including exact image payload bytes, after transport.
    assert message_to_ordereddict(result.view)==message_to_ordereddict(expected)
    assert view_from_message(result.view,context.domain)==context.pickup_view
    assert bytes(result.view.depth_mm.data)==bytes(expected.depth_mm.data)
    assert not list((f.root/'inputs').glob('case-*'))
    if failure in ('missing','evicted'):assert 'exact_input_missing' in f.warnings[0]
    elif failure=='io':assert 'disk unavailable' in f.warnings[0]
    elif failure=='callback':assert 'optional callback failed' in caplog.text


def test_detector_constructor_wires_successful_named_reads_to_capture(adopted,monkeypatch):
    import rclpy
    f=adopted
    request,context,_,_=f.stage(12_000_000_000)
    # No sensor stream or estimator work is needed to exercise actual port wiring.
    monkeypatch.setattr(f.detector.LuggageDetector,'_estimate_worker',lambda self:None)
    node=None
    rclpy.init(domain_id=175,args=['--ros-args','-p','use_semantic:=true',
        '-p','top_surface_mode:=dynamic','-p','publish_suction_top_domain:=true'])
    try:
        node=f.detector.LuggageDetector()
        node._suction_input_capture=f.capture
        node._pickup_view_service.context_lookup=lambda digest:context
        result=node._pickup_view_service.get(query(request),GetPickupViewEvidenceV2.Response())
        assert result.success
        assert len(list((f.root/'inputs').glob('case-*')))==1
    finally:
        if node is not None:
            node._estimate_thread.join(timeout=1.)
            node.destroy_node()
        rclpy.shutdown()
