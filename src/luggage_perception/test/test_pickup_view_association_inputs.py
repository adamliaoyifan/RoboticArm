"""Exact optional association inputs; constructed conformance, no identity claim."""
from dataclasses import replace
import importlib.util
import os
from pathlib import Path
from types import SimpleNamespace
import threading
import time

import numpy as np
import pytest

from luggage_perception.pickup_view_association import (
    BufferedPickupRgb, MAX_REPORTED_CANDIDATES, ReportedPickupCandidate,
)
from luggage_perception.ros_pickup_association import association_inputs_from_detection
from test_pickup_view_evidence import view_context


def source_inputs():
    _, context, _, _ = view_context()
    a = context.pickup_view.acquisition
    image = np.arange(8*12*3, dtype=np.uint8).reshape(8, 12, 3)
    rgb = BufferedPickupRgb.capture(image, stamp_ns=a.stamp_ns,
        optical_frame_id=a.optical_frame_id, encoding='rgb8')
    def candidate(confidence, bbox, held=False):
        return SimpleNamespace(label=2, prompt='box', confidence=confidence, bbox=list(bbox), held=held)
    message = SimpleNamespace(header=SimpleNamespace(frame_id=a.optical_frame_id,
        stamp=SimpleNamespace(sec=a.stamp_ns//10**9, nanosec=a.stamp_ns%10**9)),
        image_height=8, image_width=12, frame_seq=7, generation=a.generation,
        instance_id=a.instance_id, backend='constructed-fixture', detections=[
            candidate(.5, (0,0,4,4)), candidate(.9, (1,1,9,7)),
            candidate(.9, (1,1,9,7), True)])
    return context, image, rgb, message


def capture(message, rgb, context):
    a = context.pickup_view.acquisition
    return association_inputs_from_detection(message, rgb, stamp_ns=a.stamp_ns,
        optical_frame_id=a.optical_frame_id, image_shape=a.shape, selected_bbox=(1,1,9,7))


def associated_view():
    context, image, rgb, message = source_inputs()
    inputs, reason = capture(message, rgb, context)
    assert inputs is not None, reason
    return context, replace(context.pickup_view.acquisition,
        association_inputs=inputs, association_reason=reason).bind(context.domain)


@pytest.mark.parametrize('encoding', ['rgb8','bgr8'])
def test_owned_rgb_and_all_reported_competitors_survive_source_mutation(encoding):
    context, image, _, message = source_inputs()
    expected = image[:, :, ::-1].copy() if encoding == 'bgr8' else image.copy()
    a = context.pickup_view.acquisition
    rgb = BufferedPickupRgb.capture(image, stamp_ns=a.stamp_ns,
        optical_frame_id=a.optical_frame_id, encoding=encoding)
    inputs, reason = capture(message, rgb, context)
    assert reason == 'PICKUP_ASSOCIATION_INPUTS_AVAILABLE'
    image[:] = 0
    message.detections[1].bbox[0] = 8
    message.detections.clear()
    assert np.array_equal(rgb.array(), expected)
    assert not rgb.array().flags.writeable
    assert len(inputs.candidates) == 3 and inputs.selected_index == 1
    assert inputs.candidates[1].bbox == inputs.candidates[2].bbox == (1,1,9,7)
    assert inputs.candidates[2].held and not inputs.candidates[1].held


@pytest.mark.parametrize('case', ['confidence','area','tie'])
def test_selected_roi_index_matches_actual_detector_order(case):
    from luggage_perception.pickup_view_association import selected_candidate_index
    _, _, _, message = source_inputs()
    if case == 'confidence':message.detections[0].confidence = .99
    elif case == 'area':message.detections[2].bbox = [0,0,12,8]
    pool = tuple(ReportedPickupCandidate(c.label,c.prompt,c.confidence,tuple(c.bbox),c.held)
                 for c in message.detections)
    path = Path(__file__).parents[1]/'scripts/luggage_detector_node.py'
    spec = importlib.util.spec_from_file_location('association_roi_order',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    selected=selected_candidate_index(pool)
    assert selected == {'confidence':0,'area':2,'tie':1}[case]
    assert pool[selected].bbox == module.LuggageDetector._select_instance_bbox(None,message)


@pytest.mark.parametrize('damage', ['missing_rgb','rgb_stamp','rgb_frame','detector_stamp',
    'detector_frame','image_shape','pool_count','candidate','selected_bbox'])
def test_optional_refusal_has_no_substitution_and_keeps_independent_view(damage):
    context, _, rgb, message = source_inputs()
    if damage == 'missing_rgb': rgb = None
    elif damage == 'rgb_stamp': rgb = replace(rgb, stamp_ns=rgb.stamp_ns+1)
    elif damage == 'rgb_frame': rgb = replace(rgb, optical_frame_id='another-optical')
    elif damage == 'detector_stamp': message.header.stamp.nanosec += 1
    elif damage == 'detector_frame': message.header.frame_id = 'another-optical'
    elif damage == 'image_shape': message.image_width += 1
    elif damage == 'pool_count': message.detections *= MAX_REPORTED_CANDIDATES
    elif damage == 'candidate': message.detections[0].confidence = float('nan')
    else: message.detections[1].bbox = [0,0,12,8]
    inputs, reason = capture(message, rgb, context)
    assert inputs is None and reason.startswith('PICKUP_ASSOCIATION_UNAVAILABLE:')
    base = context.pickup_view.acquisition
    retained = replace(base, association_inputs=inputs, association_reason=reason).bind(context.domain)
    assert retained.acquisition.depth_mm == base.depth_mm
    assert retained.acquisition.instance_region == base.instance_region
    assert retained.domain_sha256 == context.domain_sha256


@pytest.mark.parametrize('encoding', ['mono8','rgba8',''])
def test_unsupported_encoding_refuses_optional_rgb(encoding):
    with pytest.raises(ValueError):
        BufferedPickupRgb.capture(np.zeros((2,3,3),np.uint8), stamp_ns=1,
            optical_frame_id='measured', encoding=encoding)


def test_optional_resource_and_candidate_limits_never_truncate(monkeypatch):
    import luggage_perception.pickup_view_association as module
    context, _, rgb, message = source_inputs()
    monkeypatch.setattr(module, 'MAX_ASSOCIATION_BYTES', len(rgb.rgb8))
    inputs, reason = capture(message, rgb, context)
    assert inputs is None and 'OVER_BUDGET' in reason
    assert len(message.detections) == 3
    with pytest.raises(ValueError):
        BufferedPickupRgb.capture(np.zeros((8,13,3),np.uint8), stamp_ns=1,
            optical_frame_id='measured',encoding='rgb8')


def test_detector_rgb_callback_retains_actual_source_and_eight_entry_bound():
    from sensor_msgs.msg import Image
    path = Path(__file__).parents[1]/'scripts/luggage_detector_node.py'
    spec = importlib.util.spec_from_file_location('association_detector_boundary',path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    fake = SimpleNamespace(_view_lock=threading.RLock())
    for sec in range(10):
        message = Image(height=2,width=3,encoding='bgr8',step=9,data=bytes(range(18)))
        message.header.frame_id='actual-optical';message.header.stamp.sec=sec
        module.LuggageDetector._rgb_cb(fake,message)
    assert len(fake._rgb_by_stamp) == 8 and (1,0) not in fake._rgb_by_stamp
    retained = fake._rgb_by_stamp[(9,0)]
    assert retained.stamp_ns == 9*10**9 and retained.optical_frame_id == 'actual-optical'
    assert np.array_equal(retained.array()[0,0], [2,1,0])


def test_wire_roundtrip_binds_all_exact_inputs():
    from rclpy.serialization import serialize_message, deserialize_message
    from luggage_msgs.msg import PickupViewEvidenceV2
    from luggage_perception.pickup_view_service import view_to_message, view_from_message
    context, view = associated_view()
    message = view_to_message(view)
    raw = serialize_message(message)
    result = view_from_message(deserialize_message(raw,PickupViewEvidenceV2),context.domain)
    assert result == view and result.digest() == view.digest()
    assert len(result.acquisition.association_inputs.candidates) == 3
    if os.environ.get('PICKUP_VIEW_ASSOCIATION_EVIDENCE'):
        folder = Path(os.environ['PICKUP_VIEW_ASSOCIATION_EVIDENCE']);folder.mkdir(exist_ok=True,parents=True)
        (folder/'complete_view.cdr').write_bytes(raw)


@pytest.mark.parametrize('damage', ['rgb_bytes','rgb_stamp','rgb_step','pool_stamp','pool_size',
    'pool_candidate','pool_id','pool_generation','pool_backend','pool_frame_seq','pool_prompt',
    'pool_held','selected_index','availability','reason'])
def test_wire_modified_association_inputs_cannot_pass_binding(damage):
    from luggage_perception.pickup_view_service import view_to_message, view_from_message
    context, view = associated_view(); message = view_to_message(view)
    if damage == 'rgb_bytes':message.rgb.data[0] ^= 1
    elif damage == 'rgb_stamp':message.rgb.header.stamp.nanosec += 1
    elif damage == 'rgb_step':message.rgb.step += 1
    elif damage == 'pool_stamp':message.candidate_pool.header.stamp.nanosec += 1
    elif damage == 'pool_size':message.candidate_pool.image_height += 1
    elif damage == 'pool_candidate':message.candidate_pool.detections.pop()
    elif damage == 'pool_id':message.candidate_pool.instance_id = 'other'
    elif damage == 'pool_generation':message.candidate_pool.generation += 1
    elif damage == 'pool_backend':message.candidate_pool.backend = 'changed'
    elif damage == 'pool_frame_seq':message.candidate_pool.frame_seq += 1
    elif damage == 'pool_prompt':message.candidate_pool.detections[0].prompt = 'changed'
    elif damage == 'pool_held':message.candidate_pool.detections[0].held = True
    elif damage == 'selected_index':message.selected_candidate_index = 2
    elif damage == 'availability':message.association_inputs_available = False
    else:message.association_inputs_reason = 'changed'
    with pytest.raises(ValueError):view_from_message(message,context.domain)


@pytest.mark.parametrize('rgb_available', [True,False])
def test_detector_hands_exact_optional_inputs_to_evaluator_without_changing_output(monkeypatch,rgb_available):
    context, _, rgb, message = source_inputs()
    path = Path(__file__).parents[1]/'scripts/luggage_detector_node.py'
    spec = importlib.util.spec_from_file_location('association_handoff_boundary',path)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    depth=np.full((8,12),1000.);mask=np.ones(depth.shape,bool);top=SimpleNamespace(reason='ok')
    calls=[];output=object()
    evaluator=SimpleNamespace(update=lambda *a,**kw:calls.append(kw),copy_output=lambda:output)
    header=message.header
    fake=SimpleNamespace(_pop_raw_depth=lambda key:((depth,SimpleNamespace(header=header)),object()),
        _peek_instance_mask=lambda key:mask,_select_instance_bbox=lambda m:(1,1,9,7),
        _component_config=None,_dynamic_config=None,_tf_buffer=None,_world_frame='world',
        _timing={},_suction_evaluator=evaluator,_view_lock=threading.RLock(),
        _publish_observed_suction_domain=True,
        _rgb_by_stamp={(header.stamp.sec,header.stamp.nanosec):rgb} if rgb_available else {})
    monkeypatch.setattr(module,'isolate_depth_component',lambda *a,**kw:SimpleNamespace(ok=True,mask=mask))
    monkeypatch.setattr(module,'_lookup_transform_matrix',lambda *a:(np.eye(4),None))
    monkeypatch.setattr(module,'estimate_dynamic_top_surface',lambda *a,**kw:top)
    found,quality=module.LuggageDetector._estimate_dynamic_top(fake,SimpleNamespace(header=header),message,object(),12.)
    assert found is top and quality is output and len(calls)==1
    assert (calls[0]['association_inputs'] is not None) == rgb_available
    if rgb_available:
        assert calls[0]['association_inputs'].rgb is rgb
        assert len(calls[0]['association_inputs'].candidates)==3
    else:assert 'EXACT_RGB_UNAVAILABLE' in calls[0]['association_reason']


def test_wire_real_service_returns_complete_rgb_pool_and_terminal_teardown():
    import json
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.serialization import serialize_message
    from luggage_msgs.srv import GetPickupViewEvidenceV2
    from luggage_perception.directed_suction_quality import DirectedSuctionContext
    from luggage_perception.pickup_view_service import PickupViewService,view_from_message
    original,view=associated_view()
    observed=DirectedSuctionContext(original.domain,original._height_map,original._model,pickup_view=view)
    handler=PickupViewService(context_lookup=lambda digest:observed if digest==observed.domain_sha256 else None)
    request,_,_,_=view_context()
    query=GetPickupViewEvidenceV2.Request(domain_sha256=observed.domain_sha256,
        detection=request.detection,generation=observed.domain.generation)
    context=rclpy.context.Context();rclpy.init(context=context,domain_id=178)
    owner=rclpy.create_node('association_inputs_owner',context=context)
    caller=rclpy.create_node('association_inputs_caller',context=context)
    service=owner.create_service(GetPickupViewEvidenceV2,'/test/association_inputs',handler.get)
    client=caller.create_client(GetPickupViewEvidenceV2,'/test/association_inputs')
    executor=SingleThreadedExecutor(context=context);executor.add_node(owner);executor.add_node(caller)
    worker=threading.Thread(target=executor.spin);worker.start()
    directory=Path(os.environ['PICKUP_VIEW_ASSOCIATION_EVIDENCE']) if os.environ.get('PICKUP_VIEW_ASSOCIATION_EVIDENCE') else None
    if directory:directory.mkdir(exist_ok=True,parents=True)
    try:
        assert client.wait_for_service(timeout_sec=3.)
        future=client.call_async(query);deadline=time.monotonic()+3.
        while not future.done() and time.monotonic()<deadline:time.sleep(.01)
        assert future.done()
        response=future.result()
        if directory:
            (directory/'native_request.cdr').write_bytes(serialize_message(query))
            (directory/'native_response.cdr').write_bytes(serialize_message(response))
        assert response.success,response.reason
        assert view_from_message(response.view,observed.domain)==view
    finally:
        executor.shutdown(timeout_sec=3.);worker.join(timeout=3.)
        caller.destroy_client(client);owner.destroy_service(service)
        owner.destroy_node();caller.destroy_node();context.shutdown()
        if directory:(directory/'teardown.json').write_text(json.dumps(dict(worker_stopped=not worker.is_alive())))
    assert not worker.is_alive()
