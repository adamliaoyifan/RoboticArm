"""Frame-local unavailability, explicit revocation, and exact retained evidence."""
from dataclasses import replace
import gzip
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import threading
from types import SimpleNamespace

import pytest
from rclpy.serialization import deserialize_message, serialize_message
from luggage_msgs.msg import SuctionTopDomainV2
from luggage_msgs.srv import RetainSuctionObservationV2
from luggage_perception.directed_suction_service import DirectedSuctionService
from luggage_perception.frozen_suction_observation import FrozenSuctionObservations
from luggage_perception.frozen_suction_service import FrozenSuctionService
from luggage_perception.ros_suction_domain import acquisition_metadata_from_message, domain_to_message
from luggage_perception.suction_acquisition_history import SuctionAcquisitionHistory
from test_suction_acquisition_history import successive_contexts


@pytest.fixture
def capture(request):
    root = os.environ.get('ACQUISITION_UNAVAILABLE_EVIDENCE')
    directory = Path(root)/request.node.name if root else None
    records = []
    if directory: directory.mkdir(parents=True, exist_ok=False)

    def record(name, value):
        raw = value if isinstance(value, bytes) else json.dumps(value, indent=2).encode()
        if directory: (directory/name).write_bytes(raw)
        records.append(dict(file=name, sha256=hashlib.sha256(raw).hexdigest(), bytes=len(raw)))

    yield record
    if directory:
        (directory/'case.json').write_text(json.dumps(dict(case=request.node.name, records=records), indent=2))


def unavailable(domain):
    message = domain_to_message(domain)
    message.valid, message.reason = False, 'OBSERVED_TOP_DOMAIN_UNAVAILABLE:fixture frame'
    message.support, message.domain_sha256 = [], ''
    return message


def metadata(domain):
    return acquisition_metadata_from_message(domain_to_message(domain))


def test_later_unavailable_frame_preserves_exact_named_quality_and_no_current_permission(capture):
    request, old, later = successive_contexts()
    owner = DirectedSuctionService(clock=lambda:0.)
    owner.replace_context(old)
    result = owner.unavailable_acquisition(**metadata(later.domain), reason='fixture')
    capture('transition.json', result)
    assert owner.context_snapshot() is None
    assert owner.context_snapshot(old.domain_sha256) is old
    assert owner.context_snapshot(later.domain_sha256) is None
    handler = FrozenSuctionService(FrozenSuctionObservations(
        max_records=3, max_bytes=8*1024**2, lifetime_s=10., clock=lambda:0.),
        context_provider=owner.context_snapshot, context_lookup=owner.context_snapshot,
        model_provider=lambda:old._model, clock=lambda:0.)
    query = RetainSuctionObservationV2.Request(request_id='task', observation_id='selected',
        domain_sha256=old.domain_sha256, detection=request.detection, generation=old.domain.generation)
    answer = handler.retain(query, RetainSuctionObservationV2.Response())
    capture('retained.cdr', serialize_message(answer))
    assert answer.retained and answer.binding.samples_sha256 == old.samples_sha256
    query.domain_sha256 = later.domain_sha256
    assert not handler.retain(query, RetainSuctionObservationV2.Response()).retained
    owner.replace_context(None)
    assert owner.context_snapshot(old.domain_sha256) is None


def test_same_acquisition_unavailable_removes_only_that_record(capture):
    _, old, later = successive_contexts()
    history = SuctionAcquisitionHistory()
    history.remember(old.domain, context=old)
    history.remember(later.domain, context=later)
    history.unavailable(**metadata(old.domain), reason='contradiction')
    capture('history.json', history.diagnostics())
    assert history.context_for_digest(old.domain_sha256) is None
    assert history.context_for_digest(later.domain_sha256) is later


@pytest.mark.parametrize('field,value', [('box_id','other'),('instance_id','other'),
    ('generation',50),('contact_model_hash','f'*64),('frame_id','other'),('frame_id',''),
    ('contact_model_hash',''),('stamp_ns',-1)])
def test_changed_identity_or_incomplete_scope_revokes_history(field, value, capture):
    _, old, later = successive_contexts()
    history = SuctionAcquisitionHistory()
    history.remember(old.domain)
    scope = metadata(later.domain); scope[field] = value
    history.unavailable(**scope, reason='fixture')
    capture('history.json', history.diagnostics())
    assert history.domain_for_acquisition(history.acquisition_key(old.domain)) is None


def test_unavailable_publication_does_not_renew_retention(capture):
    _, old, later = successive_contexts()
    now = [0.]
    history = SuctionAcquisitionHistory(lifetime_s=1., clock=lambda:now[0])
    history.remember(old.domain)
    now[0] = .99
    history.unavailable(**metadata(later.domain), reason='fixture')
    assert history.domain_for_acquisition(history.acquisition_key(old.domain)) is old.domain
    now[0] = 1.
    assert history.domain_for_acquisition(history.acquisition_key(old.domain)) is None
    capture('expired.json', history.diagnostics())


def test_actual_producer_exports_loaded_identity_and_preserves_old_named_context(capture):
    from luggage_msgs.msg import DetectionFrame
    from test_observed_suction_domain import detection_and_evaluation
    from luggage_perception.ros_suction_domain import observed_domain_for_detection
    detection, evaluation = detection_and_evaluation()
    domain = observed_domain_for_detection(detection, evaluation)
    _, samples, _ = successive_contexts()
    context = SimpleNamespace(domain=domain, domain_sha256=domain.digest(),
        _height_map=samples._height_map, _model=samples._model, samples_sha256=samples.samples_sha256)
    service = DirectedSuctionService(); service.replace_context(context)
    path = Path(__file__).parents[1]/'scripts/luggage_detector_node.py'
    spec = importlib.util.spec_from_file_location('detector_unavailable_acquisition', path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    outputs = []
    fake = SimpleNamespace(_suction_top_domain_pub=SimpleNamespace(publish=outputs.append),
        _last_suction_eval=evaluation, _directed_suction_service=service,
        _suction_evaluator=SimpleNamespace(contact_model=SimpleNamespace(
            model_version=domain.contact_model_version, identity_hash=domain.contact_model_hash)))
    frame = DetectionFrame(header=detection.header, box=detection, pca_valid=False,
        instance_id=domain.instance_id, generation=domain.generation)
    # Separate header: do not mutate original detection/acquisition.
    import copy
    frame.header = copy.deepcopy(detection.header)
    frame.header.stamp.nanosec += 1
    module.LuggageDetector._publish_suction_top_domain(fake, frame)
    message = outputs[-1]
    capture('producer_invalid.cdr', serialize_message(message))
    capture('history.json', service._history.diagnostics())
    assert not message.valid and not message.support and service.context_snapshot() is None
    assert message.contact_model_hash == domain.contact_model_hash
    assert message.contact_model_version == domain.contact_model_version
    assert service.context_snapshot(domain.digest()) is context


def test_prepare_exact_old_acquisition_and_latest_rejection_capture(capture):
    from luggage_planning.joint_operation.prepare_node import JointOperationPrepareNode
    request, old, later = successive_contexts()
    records = []
    node = SimpleNamespace(domain_history=SuctionAcquisitionHistory(), _state_lock=threading.Lock(),
        domain=None, joints=(13_000_000_000,(0.,)*6), start_tolerance=.01,
        preparation=SimpleNamespace(model=SimpleNamespace(identity_hash=old.domain.contact_model_hash)),
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=later.domain.stamp_ns)),
        _record=lambda name,value:records.append((name,value)))
    JointOperationPrepareNode._domain(node, domain_to_message(old.domain))
    rejected = unavailable(later.domain)
    JointOperationPrepareNode._domain(node, rejected)
    query = SimpleNamespace(detection=request.detection, generation=old.domain.generation,
        robot_state_stamp_valid=True, robot_state_stamp_ns=12_000_000_000, start_joints=(0.,)*6)
    assert JointOperationPrepareNode._current(node, query) == old.domain
    raw = next(value for name,value in records if name=='latest_rejected_domain_input')
    transition = json.loads(next(value for name,value in records if name=='latest_rejected_domain_transition'))
    capture('rejected_input.cdr', raw); capture('transition.json', transition)
    assert deserialize_message(raw,SuctionTopDomainV2) == rejected
    assert len(transition['before']['records']) == len(transition['after']['records']) == 1
    # Malformed claimed-valid content still revokes, never ordinary unavailability.
    rejected.valid = True
    JointOperationPrepareNode._domain(node, rejected)
    with pytest.raises(ValueError, match='OBSERVED_DOMAIN_UNAVAILABLE'):
        JointOperationPrepareNode._current(node, query)
    assert not node.domain_history.diagnostics()['records']


def test_unchanged_legacy_replay_without_model_identity_remains_unscoped(capture):
    source = os.environ.get('ACQUISITION_UNAVAILABLE_REPLAY')
    if not source: pytest.skip('requires saved regression07 raw two-frame case')
    from luggage_planning.joint_operation.prepare_node import JointOperationPrepareNode
    node = SimpleNamespace(domain_history=SuctionAcquisitionHistory(), _state_lock=threading.Lock())
    hashes = []
    for filename in ('0.cdr.gz','1.cdr.gz'):
        stored = (Path(source)/filename).read_bytes(); raw = gzip.decompress(stored)
        hashes.append(hashlib.sha256(stored).hexdigest())
        message = deserialize_message(raw,SuctionTopDomainV2)
        JointOperationPrepareNode._domain(node, message)
        capture(filename+'.json', node.domain_history.diagnostics())
    capture('source.json', dict(directory=source, stored_sha256=hashes))
    assert not message.valid and not message.contact_model_hash
    assert not node.domain_history.diagnostics()['records']
    assert node.domain_history.diagnostics()['last_unavailable']['action'] == 'REVOKED_UNSCOPED'
