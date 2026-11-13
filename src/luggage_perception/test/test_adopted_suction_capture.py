"""Adoption owns bounded diagnostics; detection reads and I/O are not permits."""
import copy
from dataclasses import dataclass
import importlib.util
import json
import os
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from rclpy.serialization import deserialize_message, serialize_message

from luggage_msgs.srv import RetainSuctionObservationV2
from luggage_perception.frozen_suction_observation import FrozenSuctionObservations
from luggage_perception.frozen_suction_service import (
    FrozenSuctionService, binding_from_message, register_frozen_suction_services,
)
from luggage_perception.suction_input_capture import SuctionInputCapture
from pickup_continuation_fixtures import case_archive
from test_pickup_view_evidence import view_context


@dataclass
class CapturedTop:
    plane_labels: object


@dataclass
class CapturedEvaluation:
    instance_id: str
    generation: int
    stamp: float
    frame: str


@pytest.fixture
def adopted(case_archive):
    path = Path(os.environ.get('ELFIN_TEST_DETECTOR_SCRIPT',
        Path(__file__).parents[1]/'scripts/luggage_detector_node.py'))
    spec = importlib.util.spec_from_file_location('adopted_capture_detector', path)
    detector = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(detector)
    capture = SuctionInputCapture(case_archive.directory/'inputs')
    warnings = []
    node = SimpleNamespace(_suction_input_capture=capture, _warn_throttled=warnings.append,
        _stamp_age_sec=lambda stamp:0., _cloud_max_age=1., _use_semantic=False,
        _acquisition_unsettled=lambda stamp:False)
    current = SimpleNamespace(context=None)
    store = FrozenSuctionObservations(max_records=8, max_bytes=8*1024**2,
                                    lifetime_s=10., clock=lambda:0.)
    handler = FrozenSuctionService(store, context_provider=lambda:current.context,
        model_provider=lambda:current.context._model,
        on_retained=lambda domain:detector.LuggageDetector._capture_retained_suction_input(node, domain))
    count = [0]

    def stage(stamp):
        original, context, depth, region = view_context(stamp_ns=stamp)
        current.context = context
        domain = context.domain
        top = CapturedTop(np.arange(depth.size, dtype=np.int32).reshape(depth.shape))
        evaluation = CapturedEvaluation(domain.instance_id, domain.generation, stamp/1e9, 'world')
        capture.stage(depth, SimpleNamespace(fx=120., fy=121., cx=5.5, cy=3.5),
            np.eye(4), top, region, context._model, evaluation, stamp_ns=stamp)
        request = RetainSuctionObservationV2.Request(request_id='task', observation_id=str(stamp),
            domain_sha256=context.domain_sha256, detection=original.detection, generation=domain.generation)
        return request, context, depth.copy(), top.plane_labels.copy()

    def retain(request):
        index = count[0]; count[0] += 1
        raw = serialize_message(request)
        (case_archive.directory/('%02d_request.cdr'%index)).write_bytes(raw)
        response = handler.retain(deserialize_message(raw, RetainSuctionObservationV2.Request),
                                  RetainSuctionObservationV2.Response())
        output = serialize_message(response)
        (case_archive.directory/('%02d_response.cdr'%index)).write_bytes(output)
        return deserialize_message(output, RetainSuctionObservationV2.Response)

    return SimpleNamespace(capture=capture, handler=handler, node=node, detector=detector,
        warnings=warnings, current=current, store=store, stage=stage, retain=retain,
        root=case_archive.directory)


def test_read_checks_preserve_slots_for_exact_adopted_acquisitions(adopted):
    f = adopted
    request, context, depth, labels = f.stage(12_079_000_000)
    assert context.domain.box_id != context.domain.instance_id
    initial = f.retain(request)
    assert initial.retained
    binding = binding_from_message(initial.binding)
    quality = f.store.lookup(binding)
    check = copy.deepcopy(request)
    check.expected_binding = initial.binding
    for stamp in range(12_080_000_000, 12_080_000_016):
        _, incidental, _, _ = f.stage(stamp)
        box = object()
        frame = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=12,nanosec=stamp%10**9)),
            pca_valid=True,pca_reason='ok',pca_confidence=.9,box=box)
        result, confidence = f.detector.LuggageDetector._frame_to_detect(f.node, frame)
        assert result is box and confidence == .9
        assert f.retain(check).retained
    assert f.store.lookup(binding) is quality
    assert len(list((f.root/'inputs').glob('case-*'))) == 1
    saved = np.load(f.root/'inputs/case-000/inputs.npz')
    np.testing.assert_array_equal(saved['depth_mm'], depth)
    np.testing.assert_array_equal(saved['plane_labels'], labels)

    for stamp in (28_314_000_000, 30_000_000_000, 31_000_000_000):
        request, context, _, _ = f.stage(stamp)
        assert f.retain(request).retained
    events = list(map(json.loads,(f.root/'inputs/selection_events.jsonl').read_text().splitlines()))
    assert [r['stamp_ns'] for r in events if r['status']=='saved'] == [
        12_079_000_000,28_314_000_000,30_000_000_000,31_000_000_000]
    request, _, _, _ = f.stage(32_000_000_000)
    assert f.retain(request).retained
    assert f.warnings == ['selected suction input capture: capture_limit']
    assert len(list((f.root/'inputs').glob('case-*'))) == 4


@pytest.mark.parametrize('damage',['detection','policy','expired'])
def test_failed_retention_does_not_pin_diagnostics(adopted, damage):
    f = adopted
    request, _, _, _ = f.stage(12_000_000_000)
    if damage == 'detection':
        request.detection.width += .1
    elif damage == 'policy':
        request.expected_quality_policy_sha256 = 'incorrect'
    else:
        good = f.retain(request)
        request.expected_binding = good.binding
        f.store.revoke_all()
    before = len(list((f.root/'inputs').glob('case-*')))
    calls = []
    f.handler.on_retained = calls.append
    response = f.retain(request)
    assert not response.retained
    assert calls == [] and len(list((f.root/'inputs').glob('case-*'))) == before


@pytest.mark.parametrize('field',['instance','generation','stamp'])
def test_nearby_identity_cannot_substitute_for_exact_capture(adopted, field):
    f = adopted
    request, context, _, _ = f.stage(12_000_000_000)
    values = [context.domain.instance_id, context.domain.generation, context.domain.stamp_ns]
    index = ['instance','generation','stamp'].index(field)
    values[index] = 'another-instance' if field == 'instance' else values[index]+1
    assert f.capture.pin(*values)['status'] == 'exact_input_missing'
    assert not list((f.root/'inputs').glob('case-*'))
    assert f.retain(request).retained
    record = json.loads((f.root/'inputs/case-000/record.json').read_text())
    assert record['identity'] == [context.domain.instance_id, context.domain.generation]
    assert record['stamp_ns'] == context.domain.stamp_ns


@pytest.mark.parametrize('failure',['missing','evicted','io','callback','disabled'])
def test_optional_capture_failure_preserves_retention_and_quality(adopted, failure, caplog):
    f = adopted
    request, context, _, _ = f.stage(12_000_000_000)
    if failure == 'missing':
        f.capture._pending.clear()
    elif failure == 'evicted':
        for i in range(9):
            f.stage(12_000_000_001+i)
        f.current.context = context
    elif failure == 'io':
        def broken(*args):
            raise OSError('diagnostic disk unavailable')
        f.capture.pin = broken
    elif failure == 'callback':
        def broken(domain):
            raise OSError('optional callback failed')
        f.handler.on_retained = broken
    else:
        del f.node._suction_input_capture
    response = f.retain(request)
    assert response.retained and response.reason == 'MEASURED_OBSERVATION_RETAINED'
    binding = binding_from_message(response.binding)
    assert f.store.lookup(binding).domain_sha256 == context.domain_sha256
    assert response.pickup_view_available
    assert not list((f.root/'inputs').glob('case-*'))
    if failure in ('missing','evicted'):
        assert 'exact_input_missing' in f.warnings[0]
    elif failure == 'io':
        assert 'disk unavailable' in f.warnings[0]
    elif failure == 'callback':
        assert 'optional callback failed' in caplog.text
    else:
        assert f.warnings == []


def test_real_registration_forwards_adoption_callback(adopted):
    f = adopted
    request, context, _, _ = f.stage(12_000_000_000)
    services = []
    node = SimpleNamespace(declare_parameter=lambda name,value:SimpleNamespace(value=value),
        create_service=lambda kind,name,handler:services.append((kind,name,handler)))
    calls = []
    handler = register_frozen_suction_services(node, context_provider=lambda:context,
        model_provider=lambda:context._model, on_retained=calls.append)
    response = services[0][2](request, RetainSuctionObservationV2.Response())
    assert response.retained and calls == [context.domain]
    assert handler.store.lookup(binding_from_message(response.binding)) is not None
