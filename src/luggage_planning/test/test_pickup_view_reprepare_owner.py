"""Actual owner/core/history contracts; scripted ordinary-planning boundary."""
from dataclasses import asdict, replace
import json
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest
from rclpy.serialization import deserialize_message, serialize_message
from luggage_msgs.msg import DetectedLuggage, JointOperationPlanV2, PickupViewPlanV2, StationaryPickupPolicyV2
from luggage_msgs.srv import GetPickupViewEvidenceV2, PrepareJointOperationV2, ReprepareAfterPickupViewV2
from luggage_description.suction_contact_model import load_suction_contact_model
from luggage_perception.observed_contact_footprint import project_contact_footprint
from luggage_perception.pickup_view_service import view_to_message
from luggage_perception.ros_suction_domain import domain_to_message
from luggage_perception.suction_acquisition_history import SuctionAcquisitionHistory
from luggage_planning.joint_operation.contact_observation import contact_matrix, unresolved_probe
from luggage_planning.joint_operation.prepare_node import JointOperationPrepareNode
from luggage_planning.joint_operation.preparation_profile import PreparationBudget
from luggage_planning.joint_operation.view_context import IssuedPickupView, PickupViewContext
from luggage_planning.joint_operation.view_reprepare import PickupViewReprepareOwner
from pickup_continuation_fixtures import candidate, case_archive, fixture_policy, planar_view
from test_pickup_view_context import context_case, store


def detection_for(domain):
    detection = DetectedLuggage(id=domain.box_id)
    detection.header.frame_id = domain.frame_id
    detection.header.stamp.sec, detection.header.stamp.nanosec = divmod(domain.stamp_ns, 10**9)
    detection.width, detection.depth, detection.height = domain.box_wdh
    detection.pose.position.x, detection.pose.position.y, detection.pose.position.z = domain.box_xyz_world
    (detection.pose.orientation.x, detection.pose.orientation.y,
        detection.pose.orientation.z, detection.pose.orientation.w) = domain.box_quaternion_xyzw
    detection.top_surface_valid = detection.height_valid = True
    detection.height_source = detection.HEIGHT_SOURCE_MEASURED_SUPPORT
    return detection


@pytest.fixture
def reprepare_case(case_archive):
    from pathlib import Path
    f = context_case(); f.archive = case_archive
    model = load_suction_contact_model(str(Path('src/luggage_description/config/suction_contact_model.yaml').resolve()))
    support = np.ones((10, 10), np.uint8); support[5, 5] = 0
    depth = np.full((16, 16), 1000, np.uint16); depth[6:10, 6:10] = 0
    f.domain, f.view = planar_view(depth=depth, domain_changes=dict(
        support=support.tobytes(), contact_model_hash=model.identity_hash, contact_model_version=model.model_version))
    f.new_domain, f.new_view = planar_view(newer=True, domain_changes=dict(
        contact_model_hash=model.identity_hash, contact_model_version=model.model_version))
    f.original.detection = detection_for(f.domain)
    f.original.generation = f.domain.generation
    f.original.observation_id = f.domain.digest()
    f.original.start_joints = [0.]*6
    f.original.robot_state_stamp_ns = f.domain.stamp_ns
    vertices = project_contact_footprint(f.domain, model, contact_matrix(f.domain, (0., 0.), 0.))
    f.probe = unresolved_probe(f.domain, model, 'original-physical-contact', (0., 0.), 0., vertices)
    assert f.probe is not None
    f.probe_message = f.probe.to_message(f.placement, f.source, f.original.request_id, 8)
    binding = replace(f.observation.binding, observation_id=f.domain.digest(), box_id=f.domain.box_id,
        generation=f.domain.generation, stamp_ns=f.domain.stamp_ns, domain_sha256=f.domain.digest(),
        contact_model_hash=model.identity_hash)
    f.observation = SimpleNamespace(binding=binding)
    f.context = PickupViewContext.capture(f.original, f.domain, f.observation, [f.probe_message])
    f.retention_clock = [10.]
    contexts = store(clock=lambda: f.retention_clock[0]); contexts.remember(f.context)
    f.issued = PickupViewPlanV2(request_id='issued-empty-view', task_request_id=f.original.request_id,
        domain_sha256=f.domain.digest(), view_sha256=f.view.digest(), query_id=f.probe.query_id,
        model_sha256=f.model.model_sha256, route_digest='d'*64, certificate_id='scripted-view-only')
    f.receipt = IssuedPickupView(SimpleNamespace(route_digest=f.issued.route_digest), None,
        SimpleNamespace(scene_cdr=b'scripted-view-scene'), f.view, bytes(serialize_message(f.issued)), 'b'*64)
    contexts.issue(f.context, f.receipt)
    f.calls, f.records, f.fail_record = [], [], None
    f.q, f.now_ns = (.1, .2, .3, .4, .5, .6), 3_000_000_100
    history = SuctionAcquisitionHistory(); history.remember(f.new_domain)
    def record(name, value):
        raw = value if isinstance(value, bytes) else serialize_message(value)
        path = case_archive.directory/(f'{len(f.records):04d}_'+name+'.bin')
        path.write_bytes(raw); f.records.append(path.name)
        if f.fail_record == name: raise OSError('constructed capture failure')
    def call(client, query, deadline, boundary):
        record('new_acquisition_query', query)
        result = GetPickupViewEvidenceV2.Response(success=True, view=view_to_message(f.new_view))
        record('new_acquisition_result', result)
        return result
    def prepare(request, response, *, latest_map=False):
        assert f.node._active.acquire(blocking=False), 'continuation stranded ordinary lock'
        f.node._active.release()
        f.calls.append((deserialize_message(serialize_message(request), PrepareJointOperationV2.Request), latest_map))
        record('ordinary_prepare_input', request)
        response.success, response.reason = True, 'SCRIPTED_INDEPENDENT_PREPARE_PASS'
        response.plan = JointOperationPlanV2(request_id=request.request_id, plan_digest='e'*64)
        response.trace_json = '[["scripted_ordinary_boundary","not_full_motion_qualification"]]'
        record('ordinary_prepare_output', response)
        return response
    f.node = SimpleNamespace(observer=object(), _active=threading.Lock(), _state_lock=threading.Lock(),
        joints=(3_000_000_001, f.q), domain_history=history, start_tolerance=.01,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=f.now_ns)),
        preparation=SimpleNamespace(model=model), budget=PreparationBudget(30.),
        _record=record, _call=call, _prepare=prepare)
    f.node._current = lambda request: JointOperationPrepareNode._current(f.node, request)
    f.views = SimpleNamespace(node=f.node, contexts=contexts, max_joint_age_ns=1_000_000_000,
                             evidence_client=object(), evidence_service=GetPickupViewEvidenceV2)
    f.owner = PickupViewReprepareOwner.__new__(PickupViewReprepareOwner)
    f.owner.node, f.owner.views = f.node, f.views
    f.request = ReprepareAfterPickupViewV2.Request(request_id='new-independent-operation',
        previous_view=f.issued, settled_stamp_valid=True, settled_stamp_ns=2_000_000_000,
        pickup_stationary=True, detection=detection_for(f.new_domain), generation=f.new_domain.generation,
        association_policy=StationaryPickupPolicyV2(**asdict(fixture_policy())),
        information_sample_shape=[4, 4], budget_s=10.)
    case_archive.save('original_and_new_raw_inputs', (f.domain, f.view, f.new_domain, f.new_view))
    record('original_request', f.original); record('original_probe', f.probe_message)
    record('issued_view', f.issued); record('original_domain', domain_to_message(f.domain))
    case_archive.save('actual_node_joints', dict(stamp_ns=f.node.joints[0], positions=f.q, now_ns=f.now_ns))
    try:
        yield f
    finally:
        case_archive.save('handoff_summary', dict(calls=len(f.calls), latest_map=[v for _, v in f.calls],
                          records=f.records, preparation_lock_held=f.node._active.locked()))


def run(f):
    f.node._record('test_request', f.request)
    output = f.owner.handle(f.request, ReprepareAfterPickupViewV2.Response())
    f.node._record('test_result', output)
    return output


def test_unique_new_id_reprepares_exact_original_target_from_actual_joints(reprepare_case):
    f = reprepare_case; original = f.context.request_cdr
    result = run(f)
    assert result.success and result.associated, result.reason
    assert len(f.calls) == 1
    renewed, latest = f.calls[0]
    assert latest and tuple(renewed.start_joints) == f.q
    assert renewed.robot_state_stamp_ns == f.node.joints[0]
    assert renewed.request_id == f.request.request_id and renewed.observation_id == f.new_domain.digest()
    assert renewed.detection == f.request.detection and renewed.generation == f.new_domain.generation
    assert len(renewed.slots) == 1 and renewed.slots[0].pose == f.probe_message.placement.pose
    assert renewed.placement_task_mode == '' and renewed.candidate_limit == 1
    assert renewed.expected_map_epoch == f.original.expected_map_epoch
    assert renewed.expected_model_sha256 == f.original.expected_model_sha256
    assert 0 < renewed.budget_s <= f.request.budget_s
    assert f.context.request_cdr == original and f.context.probe(f.probe.query_id) == f.probe_message
    assert json.loads(result.information_json)['gained_samples'] > 0
    assert json.loads(result.association_json)['reason'] == 'CONDITIONAL_UNIQUE_STATIONARY_MATCH'


def test_zero_gain_does_not_invent_support_or_skip_independent_preparation(reprepare_case):
    f = reprepare_case
    depth = np.full((16, 16), 1000, np.uint16); depth[6:10, 4:8] = 0
    f.new_view = replace(f.new_view, acquisition=replace(f.new_view.acquisition, depth_mm=depth.tobytes()))
    f.archive.save('changed_new_view', f.new_view)
    result = run(f)
    assert result.success and result.associated and len(f.calls) == 1
    assert json.loads(result.information_json)['gained_samples'] == 0
    assert f.new_domain.cells().all()


@pytest.mark.parametrize('damage,reason', [
    ('duplicate', 'AMBIGUOUS_REPORTED_CANDIDATES'), ('rgb_missing', 'EXACT_ASSOCIATION_INPUTS_UNAVAILABLE'),
    ('receipt', 'EXACT_RETAINED_VIEW_REQUIRED'), ('query', 'EXACT_RETAINED_VIEW_REQUIRED'),
    ('settle', 'POST_SETTLE_NEW_ACQUISITION_REQUIRED'), ('settle_invalid', 'INPUT_INVALID'),
    ('stationary', 'STATIONARY_PREMISE_REQUIRED'), ('policy', 'EXPLICIT_STATIONARY'),
    ('grid', 'BOUNDED_REGION_SAMPLE'), ('budget', 'INPUT_INVALID'), ('id', 'INPUT_INVALID'),
    ('generation', 'OBSERVED_DOMAIN_UNAVAILABLE'), ('missing_joints', 'JOINTS_UNAVAILABLE_OR_STALE'),
    ('stale_joints', 'JOINTS_UNAVAILABLE_OR_STALE'), ('expired', 'EXACT_RETAINED_VIEW_REQUIRED'),
    ('backend', 'REQUIRES_O_BACKEND'), ('need', 'NO_DECISION_RELEVANT_VIEW_GAP'),
])
def test_refused_inputs_never_invoke_ordinary_planning(reprepare_case, damage, reason):
    f = reprepare_case
    if damage == 'duplicate':
        a = f.new_view.acquisition
        f.new_view = replace(f.new_view, acquisition=replace(a, association_inputs=replace(
            a.association_inputs, candidates=(candidate(), candidate(held=True)))))
    elif damage == 'rgb_missing':
        f.new_view = replace(f.new_view, acquisition=replace(f.new_view.acquisition, association_inputs=None))
    elif damage == 'receipt': f.request.previous_view.certificate_id = 'another-receipt'
    elif damage == 'query': f.request.previous_view.query_id = 'invented-region'
    elif damage == 'settle': f.request.settled_stamp_ns = f.new_domain.stamp_ns
    elif damage == 'settle_invalid': f.request.settled_stamp_valid = False
    elif damage == 'stationary': f.request.pickup_stationary = False
    elif damage == 'policy': f.request.association_policy.depth_residual_m = 0.
    elif damage == 'grid': f.request.information_sample_shape = [0, 4]
    elif damage == 'budget': f.request.budget_s = 0.
    elif damage == 'id': f.request.request_id = f.original.request_id
    elif damage == 'generation': f.request.generation += 1
    elif damage == 'missing_joints': f.node.joints = None
    elif damage == 'stale_joints': f.node.joints = (1, f.q)
    elif damage == 'expired': f.retention_clock[0] = 15.
    elif damage == 'backend': f.node.observer = None
    elif damage == 'need':
        view = replace(f.view, acquisition=replace(f.view.acquisition,
                       depth_mm=np.full((16, 16), 1000, np.uint16).tobytes()))
        # Keep a fully matching issued receipt, but no localized missing data.
        f.views.contexts.entries[f.original.request_id] = replace(
            f.views.contexts.entries[f.original.request_id], issued=replace(f.receipt, view=view))
    f.archive.save('case_changes', dict(damage=damage, new_view=f.new_view, joints=f.node.joints,
                                      now_ns=f.now_ns, retention_clock=f.retention_clock[0]))
    result = run(f)
    assert not result.success and not result.plan.plan_digest and reason in result.reason, result.reason
    assert not f.calls and not f.node._active.locked()


@pytest.mark.parametrize('failure', ['busy', 'association_capture', 'response_capture', 'independent_refusal'])
def test_owner_or_evidence_failure_cannot_leave_active_plan(reprepare_case, failure):
    f = reprepare_case
    if failure == 'busy': f.node._active.acquire()
    elif failure == 'association_capture': f.fail_record = 'pickup_view_continuation_association.json'
    elif failure == 'response_capture': f.fail_record = 'pickup_view_continuation_response'
    else:
        original_prepare = f.node._prepare
        def refuse(*args, **kwargs):
            result = original_prepare(*args, **kwargs)
            result.success, result.reason, result.plan = False, 'SCRIPTED_CONTACT_REFUSED', JointOperationPlanV2()
            return result
        f.node._prepare = refuse
    try:
        result = run(f)
        assert not result.success and not result.plan.plan_digest
        assert len(f.calls) == (1 if failure in ('response_capture', 'independent_refusal') else 0)
        if failure != 'busy': assert not f.node._active.locked()
    finally:
        if f.node._active.locked(): f.node._active.release()


def test_retained_view_counts_optional_rgb_bytes(reprepare_case):
    f = reprepare_case
    a = f.view.acquisition
    assert f.receipt.size_bytes >= f.view.size_bytes+len(f.receipt.message_cdr)
    assert f.view.size_bytes > len(a.depth_mm)+len(a.instance_region)+len(a.association_inputs.rgb.rgb8)


def test_native_ros_service_transport_and_terminal_owner_teardown(reprepare_case):
    """Actual service wire/handler; ordinary six-leg planning is scripted above."""
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    f = reprepare_case
    context = Context(); rclpy.init(context=context)
    server = Node('pickup_reprepare_fixture', context=context)
    client_node = Node('pickup_reprepare_client', context=context)
    executor = MultiThreadedExecutor(num_threads=2, context=context)
    executor.add_node(server); executor.add_node(client_node)
    service = server.create_service(ReprepareAfterPickupViewV2, '/fixture/reprepare_pickup_view', f.owner.handle)
    client = client_node.create_client(ReprepareAfterPickupViewV2, '/fixture/reprepare_pickup_view')
    worker = threading.Thread(target=executor.spin); worker.start()
    try:
        assert client.wait_for_service(timeout_sec=5.)
        f.node._record('native_request', f.request)
        future = client.call_async(f.request)
        deadline = time.monotonic()+10.
        while not future.done() and time.monotonic() < deadline: time.sleep(.005)
        assert future.done()
        result = future.result(); f.node._record('native_response', result)
        assert result.success and result.associated and len(f.calls) == 1, result.reason
    finally:
        executor.shutdown(timeout_sec=5.); worker.join(timeout=5.)
        server.destroy_node(); client_node.destroy_node(); context.shutdown()
        f.archive.save('native_teardown', dict(worker_stopped=not worker.is_alive(),
                                              scripted_ordinary_planning=True))
        assert not worker.is_alive()
