"""Actual settlement/adoption boundaries; scripted motion and ordinary planning."""
from concurrent.futures import Future
from dataclasses import asdict, replace
import json
import threading
import time
from types import SimpleNamespace

import pytest
from rclpy.serialization import deserialize_message, serialize_message
from luggage_msgs.action import ExecutePickupViewV2
from luggage_msgs.msg import JointOperationPlanV2
from luggage_msgs.srv import ReprepareAfterPickupViewV2
from luggage_planning.joint_operation.adapters import operation_plan_to_message
from luggage_planning.joint_operation.contracts import seal_operation_plan
from luggage_planning.joint_operation.view_budget import PickupViewBudgetLedger
from luggage_planning.joint_operation.view_continuation import (
    PickupViewRuntimeContinuation, retain_terminal_view,
)
from luggage_planning.motion.motion_ownership import MotionOwnership
from pickup_continuation_fixtures import candidate, case_archive, planar_case
from test_joint_operation_contracts import example_plan
from test_pickup_view_execution import lifecycle
from test_pickup_view_reprepare_owner import reprepare_case


@pytest.fixture
def ledger_case(case_archive):
    clock = [100.]
    ledger = PickupViewBudgetLedger(case_archive.directory/'budget.json', clock=lambda: clock[0],
                                    max_terminal_receipt_bytes=32)
    ledger.open('original-task', 'd'*64, max_motions=2, total_budget_s=10.)
    case_archive.ledger = (ledger.path, clock)
    case_archive.save('ledger_initial', ledger.rows)
    receipt = dict(plan_cdr=b'constructed-cdr', route_digest='a'*64, domain_sha256='d'*64,
                   settled_stamp_ns=2_000_000_000)
    case_archive.save('receipt_input', receipt)
    return ledger, clock, receipt


def test_settled_receipt_roundtrip_replacement_and_retry_never_renew(ledger_case, case_archive):
    ledger, clock, receipt = ledger_case
    ledger.dispatch('original-task'); ledger.record_settled_view('original-task', **receipt)
    before = ledger.path.read_bytes(); clock[0] = 101.
    ledger.record_settled_view('original-task', **receipt)
    assert ledger.path.read_bytes() == before
    restored = PickupViewBudgetLedger(ledger.path, clock=lambda: clock[0])
    key = {k: v for k, v in receipt.items() if k != 'plan_cdr'}
    assert restored.settled_view(**key) == ('original-task', receipt['plan_cdr'])
    restored.dispatch('original-task')
    next_receipt = dict(receipt, route_digest='b'*64, plan_cdr=b'later-cdr', settled_stamp_ns=3_000_000_000)
    case_archive.save('later_receipt_input', next_receipt)
    restored.record_settled_view('original-task', **next_receipt)
    with pytest.raises(ValueError, match='EXACT_RUNTIME'):
        restored.settled_view(**key)
    assert restored.rows['original-task']['opened_wall_s'] == 100.
    assert restored.status('original-task') == (2, 1.)
    assert restored.continuation_remaining('original-task') == 9.
    with pytest.raises(ValueError, match='MOTION_LIMIT'): restored.dispatch('original-task')


@pytest.mark.parametrize('damage', ['uncharged', 'domain', 'route', 'size', 'stamp', 'type', 'same_count', 'backwards'])
def test_invalid_or_rebound_terminal_receipt_never_replaces_owned_event(ledger_case, case_archive, damage):
    ledger, _, receipt = ledger_case
    if damage != 'uncharged': ledger.dispatch('original-task')
    if damage in ('same_count', 'backwards'):
        ledger.record_settled_view('original-task', **receipt)
        if damage == 'backwards': ledger.dispatch('original-task')
        receipt = dict(receipt, route_digest='b'*64,
                       settled_stamp_ns=receipt['settled_stamp_ns']-1 if damage == 'backwards' else 3_000_000_000)
    elif damage == 'domain': receipt['domain_sha256'] = 'f'*64
    elif damage == 'route': receipt['route_digest'] = 'not-a-digest'
    elif damage == 'size': receipt['plan_cdr'] = b'x'*33
    elif damage == 'stamp': receipt['settled_stamp_ns'] = 0
    elif damage == 'type': receipt['plan_cdr'] = bytearray(b'not-exact-bytes')
    case_archive.save('attempted_receipt', dict(receipt, plan_cdr=bytes(receipt['plan_cdr'])))
    before = ledger.path.read_bytes()
    with pytest.raises(ValueError): ledger.record_settled_view('original-task', **receipt)
    assert ledger.path.read_bytes() == before


@pytest.mark.parametrize('now,reason', [(110., 'TIME_LIMIT'), (99., 'CLOCK_ROLLBACK')])
def test_continuation_does_not_renew_expired_or_reversed_clock(ledger_case, now, reason):
    ledger, clock, _ = ledger_case
    before = ledger.path.read_bytes(); clock[0] = now
    with pytest.raises(ValueError, match=reason): ledger.continuation_remaining('original-task')
    assert ledger.path.read_bytes() == before


def test_failed_alias_or_receipt_save_is_not_an_in_memory_commit(case_archive, monkeypatch):
    before_domain, _, after_domain, _ = case = planar_case()
    proof = case_archive.associate(case)
    clock = [100.]
    ledger = PickupViewBudgetLedger(case_archive.directory/'budget.json', clock=lambda: clock[0])
    case_archive.ledger = (ledger.path, clock)
    ledger.open('task', before_domain.digest(), max_motions=2, total_budget_s=10.)
    ledger.dispatch('task'); before = ledger.path.read_bytes(); rows = json.loads(before)
    save = ledger._save
    def fail(): raise OSError('constructed write failure')
    monkeypatch.setattr(ledger, '_save', fail)
    with pytest.raises(OSError): ledger.adopt_acquisition('task', proof)
    assert ledger.rows == rows and ledger.path.read_bytes() == before
    with pytest.raises(OSError):
        ledger.record_settled_view('task', plan_cdr=b'cdr', route_digest='a'*64,
            domain_sha256=before_domain.digest(), settled_stamp_ns=2_000_000_000)
    assert ledger.rows == rows and ledger.path.read_bytes() == before
    monkeypatch.setattr(ledger, '_save', save)
    ledger.adopt_acquisition('task', proof)
    assert json.loads(ledger.path.read_text())['task']['acquisitions'][after_domain.digest()] == proof.digest()


def fixture_plan(f, request):
    plan = example_plan()
    identity = replace(plan.identity, request_id=request.request_id,
        observation_id=f.new_domain.digest(), observation_stamp_ns=f.new_domain.stamp_ns,
        robot_state_stamp_ns=f.node.joints[0], box_id=f.new_domain.box_id,
        generation=f.new_domain.generation, contact_model_hash=f.new_domain.contact_model_hash,
        model_sha256=f.issued.model_sha256)
    legs = tuple(replace(leg, trajectory_request_id=identity.request_id,
        robot_state_stamp_ns=identity.robot_state_stamp_ns,
        knots=tuple(replace(knot, positions=tuple(q+offset for q, offset in zip(knot.positions, f.q)))
                    for knot in leg.knots)) for leg in plan.legs)
    plan = seal_operation_plan(replace(plan, identity=identity, legs=legs, start_joints=f.q))
    return operation_plan_to_message(plan, plan.identity, plan.plan_digest)


@pytest.fixture
def runtime_case(reprepare_case):
    f = reprepare_case
    f.wall_clock = [100.]
    f.ledger = PickupViewBudgetLedger(f.archive.directory/'runtime_budget.json', clock=lambda: f.wall_clock[0])
    f.archive.ledger = (f.ledger.path, f.wall_clock)
    f.ledger.open('original-runtime-task', f.domain.digest(), max_motions=1, total_budget_s=10.)
    f.ledger.dispatch('original-runtime-task')
    f.terminal = ExecutePickupViewV2.Result(success=True, submitted=True, motion_terminal=True,
        settled_stamp_valid=True, settled_stamp_ns=f.request.settled_stamp_ns, plan=f.issued)
    f.node._record('constructed_actual_action_terminal', f.terminal)
    retain_terminal_view(f.ledger, 'original-runtime-task', f.terminal)
    f.wall_clock[0] = 101.
    f.preparation_refusal = False
    def ordinary(request, response, *, latest_map=False):
        f.calls.append((deserialize_message(serialize_message(request), type(request)), latest_map))
        f.node._record('ordinary_input', request)
        response.success = not f.preparation_refusal
        response.reason = 'SEARCH_INCOMPLETE' if f.preparation_refusal else 'SCRIPTED_ORDINARY_COMPLETE_PLAN'
        response.plan = JointOperationPlanV2() if f.preparation_refusal else fixture_plan(f, request)
        response.trace_json = '[["fixture","not_native_motion_certification"]]'
        if f.preparation_refusal:
            probe = deserialize_message(serialize_message(f.probe_message), type(f.probe_message))
            probe.domain_sha256, probe.contact_model_hash = f.new_domain.digest(), f.new_domain.contact_model_hash
            probe.placement.identity.request_id = request.request_id
            response.unresolved_contacts = [probe]
        f.node._record('ordinary_output', response)
        return response
    f.node._prepare = ordinary
    f.parent = SimpleNamespace(_active=threading.Lock(), session=None, owner=MotionOwnership(),
        ports=SimpleNamespace(rpc=SimpleNamespace(unresolved=lambda: False)), max_budget=30.,
        node=SimpleNamespace(get_parameter=lambda name: SimpleNamespace(value=30.)),
        evidence_root=f.archive.directory/'runtime', evidence_limit=64*1024*1024)
    f.continuation = PickupViewRuntimeContinuation.__new__(PickupViewRuntimeContinuation)
    f.continuation.parent = f.parent
    f.continuation.view = SimpleNamespace(ledger=f.ledger)
    f.continuation.wire = ReprepareAfterPickupViewV2
    f.continuation.rpc = None
    f.delayed, f.pending, f.modifier = False, None, lambda value: value
    f.on_dispatch = lambda: None
    f.rpc_inputs = []
    def actual_owner(request):
        response = f.owner.handle(request, ReprepareAfterPickupViewV2.Response())
        return f.modifier(response)
    f.actual_owner = actual_owner
    class Client:
        def service_is_ready(self): return True
        def call_async(self, request):
            f.node._record('runtime_rpc_request', request)
            f.rpc_inputs.append(deserialize_message(serialize_message(request), type(request)))
            future = Future()
            if f.delayed:
                f.pending = future
                f.on_dispatch()
            else: future.set_result(actual_owner(request))
            return future
    f.continuation.client = Client()
    try:
        yield f
    finally:
        f.archive.save('runtime_terminal_state', dict(ledger=f.ledger.rows,
            lock_held=f.parent._active.locked(), rpc_unresolved=f.continuation.rpc.unresolved()
                if f.continuation.rpc else False, rpc_calls=len(f.rpc_inputs), ordinary_calls=len(f.calls)))


def run(f):
    f.node._record('runtime_external_request', f.request)
    response = f.continuation.handle(f.request, ReprepareAfterPickupViewV2.Response())
    f.node._record('runtime_external_response', response)
    return response


def test_exact_runtime_receipt_keeps_last_view_count_and_adopts_actual_new_id(runtime_case):
    f = runtime_case; before = dict(f.ledger.rows['original-runtime-task'])
    response = run(f)
    assert response.success and response.associated, response.reason
    row = json.loads(f.ledger.path.read_text())['original-runtime-task']
    assert row['dispatched'] == before['dispatched'] == 1
    assert row['opened_wall_s'] == before['opened_wall_s'] == 100.
    assert row['total_budget_s'] == before['total_budget_s'] == 10.
    assert f.new_domain.digest() in row['acquisitions']
    assert f.rpc_inputs[0].budget_s == 9.
    assert tuple(response.plan.start_joints) == f.q
    assert response.plan.observation_id == f.new_domain.digest() and len(response.plan.legs) == 3
    assert not f.parent._active.locked()
    with pytest.raises(ValueError, match='MOTION_LIMIT'): f.ledger.dispatch('original-runtime-task')
    restored = PickupViewBudgetLedger(f.ledger.path, clock=lambda: 102.)
    assert restored.continuation_remaining('original-runtime-task') == 8.


def test_independent_refusal_still_adopts_verified_measurement_for_next_view(runtime_case):
    f = runtime_case; f.preparation_refusal = True
    response = run(f)
    assert not response.success and response.associated and response.unresolved_contacts
    assert not response.plan.plan_digest and f.new_domain.digest() in f.ledger.rows['original-runtime-task']['acquisitions']


def test_no_gain_remains_information_not_a_motion_permission(runtime_case):
    import numpy as np
    f = runtime_case
    depth = np.full((16, 16), 1000, np.uint16); depth[6:10, 4:8] = 0
    f.new_view = replace(f.new_view, acquisition=replace(f.new_view.acquisition, depth_mm=depth.tobytes()))
    f.archive.save('changed_new_view', f.new_view)
    response = run(f)
    assert response.success and response.associated and len(f.calls) == 1
    assert json.loads(response.information_json)['gained_samples'] == 0
    assert f.ledger.rows['original-runtime-task']['dispatched'] == 1


@pytest.mark.parametrize('boundary', ['record', 'close'])
def test_continuation_evidence_fault_never_publishes_plan_or_strands_lock(runtime_case, monkeypatch, boundary):
    from luggage_planning.joint_operation import view_continuation
    f = runtime_case
    original = view_continuation.RosBoundaryRecorder
    class Recorder(original):
        def record(self, name, message, **kwargs):
            if boundary == 'record' and name == 'continuation_response':
                raise OSError('constructed response write failure')
            return super().record(name, message, **kwargs)
        def close(self):
            if boundary == 'close': raise OSError('constructed health write failure')
            return super().close()
    monkeypatch.setattr(view_continuation, 'RosBoundaryRecorder', Recorder)
    f.archive.save('capture_fault', dict(boundary=boundary))
    response = run(f)
    assert not response.success and not response.plan.plan_digest and not response.unresolved_contacts
    assert 'EVIDENCE' in response.reason and not f.parent._active.locked()
    # Completed adoption is not rolled back or renewed by a response-capture fault.
    assert f.new_domain.digest() in json.loads(f.ledger.path.read_text())['original-runtime-task']['acquisitions']


@pytest.mark.parametrize('damage', ['plan_receipt', 'settle', 'missing', 'budget', 'expired', 'busy',
    'active_motion', 'session', 'pending', 'duplicate', 'rgb', 'policy', 'proof_source', 'proof_policy',
    'proof_view', 'proof_after_view', 'information_binding', 'new_detection', 'plan_binding', 'plan_digest', 'write'])
def test_refused_runtime_boundaries_cannot_publish_or_renew(runtime_case, monkeypatch, damage):
    f = runtime_case
    if damage == 'plan_receipt': f.request.previous_view.certificate_id = 'not-executed'
    elif damage == 'settle': f.request.settled_stamp_ns += 1
    elif damage == 'missing':
        f.ledger.rows['original-runtime-task'].pop('settled_view'); f.ledger._save()
    elif damage == 'budget': f.request.budget_s = 0.
    elif damage == 'expired': f.wall_clock[0] = 110.
    elif damage == 'busy': f.parent._active.acquire()
    elif damage == 'active_motion': f.parent.owner.motion_lock.acquire()
    elif damage == 'session': f.parent.session = object()
    elif damage == 'pending': f.parent.ports.rpc.unresolved = lambda: True
    elif damage == 'duplicate':
        a = f.new_view.acquisition
        f.new_view = replace(f.new_view, acquisition=replace(a, association_inputs=replace(
            a.association_inputs, candidates=(candidate(), candidate(held=True)))))
    elif damage == 'rgb': f.new_view = replace(f.new_view, acquisition=replace(f.new_view.acquisition, association_inputs=None))
    elif damage == 'policy': f.request.association_policy.depth_residual_m = 0.
    elif damage.startswith('proof_'):
        def modify(response):
            proof = json.loads(response.association_json)
            key = {'proof_source': 'after_domain_sha256', 'proof_policy': 'policy_sha256',
                   'proof_view': 'before_view_sha256', 'proof_after_view': 'after_view_sha256'}[damage]
            proof[key] = 'f'*64; response.association_json = json.dumps(proof); return response
        f.modifier = modify
    elif damage == 'information_binding':
        def modify(response):
            info = json.loads(response.information_json)
            info['query_id'] = 'other-region'; response.information_json = json.dumps(info); return response
        f.modifier = modify
    elif damage == 'new_detection':
        def modify(response): response.detection.width += .01; return response
        f.modifier = modify
    elif damage == 'plan_binding':
        def modify(response): response.plan.box_id = 'other-box'; return response
        f.modifier = modify
    elif damage == 'plan_digest':
        def modify(response): response.plan.plan_digest = 'f'*64; return response
        f.modifier = modify
    elif damage == 'write':
        def fail(): raise OSError('constructed ledger write failure')
        monkeypatch.setattr(f.ledger, '_save', fail)
    f.archive.save('runtime_fault', dict(damage=damage, wall_clock=f.wall_clock[0]))
    before = f.ledger.path.read_bytes(); rows = json.loads(before)
    try:
        response = run(f)
        assert not response.success and not response.plan.plan_digest and not response.unresolved_contacts, response.reason
        assert f.ledger.path.read_bytes() == before and f.ledger.rows == rows
        assert f.parent._active.locked() is (damage == 'busy')
    finally:
        if f.parent._active.locked(): f.parent._active.release()
        if f.parent.owner.motion_lock.locked(): f.parent.owner.motion_lock.release()


def test_late_success_is_captured_but_never_adopted_or_returned(runtime_case, monkeypatch):
    from luggage_planning.joint_operation import rpc
    f = runtime_case; f.delayed = True; f.request.budget_s = 1.
    clock = [time.monotonic()]
    # Only the RPC port sees this clock. Advance after actual pending transport,
    # so durable pre-call recording cannot accidentally bypass the tested path.
    monkeypatch.setattr(rpc, 'time', SimpleNamespace(monotonic=lambda: clock[0]))
    def dispatched():
        clock[0] += 10.
        f.archive.save('late_rpc_clock_after_dispatch', dict(monotonic_s=clock[0], pending=True))
    f.on_dispatch = dispatched
    f.archive.save('late_rpc_clock_before', dict(monotonic_s=clock[0], budget_s=f.request.budget_s))
    before = f.ledger.path.read_bytes()
    response = run(f)
    assert not response.success and f.pending is not None and f.continuation.rpc.unresolved()
    f.pending.set_result(f.actual_owner(f.rpc_inputs[-1]))
    assert not f.continuation.rpc.unresolved() and f.ledger.path.read_bytes() == before
    assert not response.plan.plan_digest
    health = list(f.parent.evidence_root.glob('*/health.json'))
    assert len(health) == 1 and json.loads(health[0].read_text())['closed']
    # The recorder's full manifest, not index spelling, is audited by the gate.
    assert any(b'measured_reprepare_late_response' in p.read_bytes() for p in f.parent.evidence_root.rglob('*.json'))


def test_actual_action_registers_terminal_before_success_reply(lifecycle, monkeypatch):
    f = lifecycle; original = f.view.ledger.record_settled_view
    called = []
    def retain(*args, **kwargs):
        assert f.outcomes == []
        called.append(kwargs); return original(*args, **kwargs)
    monkeypatch.setattr(f.view.ledger, 'record_settled_view', retain)
    response = f.view.execute(f.handle())
    assert response.success and len(called) == 1
    row = json.loads(f.view.ledger.path.read_text())['stable-box']
    assert row['settled_view']['settled_stamp_ns'] == response.settled_stamp_ns


@pytest.mark.parametrize('field,value', [('success', False), ('submitted', False), ('motion_terminal', False),
                                        ('may_have_payload', True), ('settled_stamp_valid', False)])
def test_nonterminal_or_loaded_result_cannot_register_receipt(runtime_case, field, value):
    f = runtime_case; setattr(f.terminal, field, value)
    f.node._record('attempted_terminal_result', f.terminal)
    before = f.ledger.path.read_bytes()
    with pytest.raises(ValueError, match='EMPTY_SETTLED_ACTION'):
        retain_terminal_view(f.ledger, 'original-runtime-task', f.terminal)
    assert f.ledger.path.read_bytes() == before


def test_native_two_service_continuation_chain_and_teardown(runtime_case):
    import rclpy
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    f = runtime_case
    rclpy.init()
    node = Node('pickup_runtime_continuation_native')
    node._group = ReentrantCallbackGroup()
    node.declare_parameter('max_preparation_budget_s', 30.)
    f.parent.node = node
    ordinary = node.create_service(ReprepareAfterPickupViewV2,
        '/joint_operation/reprepare_after_pickup_view', f.owner.handle, callback_group=node._group)
    f.continuation = PickupViewRuntimeContinuation(SimpleNamespace(parent=f.parent, ledger=f.ledger))
    client = node.create_client(ReprepareAfterPickupViewV2,
        '/joint_operation/continue_after_pickup_view', callback_group=node._group)
    executor = MultiThreadedExecutor(num_threads=4); executor.add_node(node)
    worker = threading.Thread(target=executor.spin); worker.start()
    try:
        assert client.wait_for_service(timeout_sec=3.)
        assert f.continuation.client.wait_for_service(timeout_sec=3.)
        f.node._record('native_runtime_request', f.request)
        future = client.call_async(f.request)
        deadline = time.monotonic()+5.
        while not future.done() and time.monotonic() < deadline: time.sleep(.01)
        assert future.done(), 'native service did not terminate'
        result = future.result(); f.node._record('native_runtime_response', result)
        assert result.success and result.associated, result.reason
        assert f.new_domain.digest() in f.ledger.rows['original-runtime-task']['acquisitions']
    finally:
        executor.shutdown(timeout_sec=3.); worker.join(timeout=3.)
        node.destroy_node(); rclpy.shutdown()
        f.archive.save('native_runtime_teardown', dict(worker_stopped=not worker.is_alive()))
    assert not worker.is_alive()
