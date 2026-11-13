"""Selected camera lifetime under stream/byte pressure; no sensor-error model."""
import copy
from dataclasses import replace
from types import SimpleNamespace
import threading

import numpy as np
import pytest
from rclpy.serialization import deserialize_message, serialize_message

from luggage_msgs.srv import GetPickupViewEvidenceV2, RetainSuctionObservationV2
from luggage_perception.frozen_suction_observation import FrozenSuctionObservations
from luggage_perception.frozen_suction_service import (
    FrozenSuctionService, binding_from_message, binding_to_message,
)
from luggage_perception.pickup_view_service import PickupViewService, view_from_message
from luggage_perception.suction_acquisition_history import SuctionAcquisitionHistory
from pickup_continuation_fixtures import case_archive
from test_pickup_view_evidence import view_context


@pytest.fixture
def camera_case(case_archive):
    original, context, depth, region = view_context()
    now = [0.]
    history = SuctionAcquisitionHistory(max_view_bytes=context.pickup_view.size_bytes,
                                       clock=lambda: now[0])
    history.remember(context.domain, context=context)
    store = FrozenSuctionObservations(max_records=4, max_bytes=8*1024**2,
                                    lifetime_s=10., clock=lambda: now[0])
    model = copy.deepcopy(context._model)
    frozen = FrozenSuctionService(store, context_provider=lambda: context,
        context_lookup=history.context_for_digest, model_provider=lambda: model)
    view = PickupViewService(context_lookup=history.context_for_digest,
                             frozen_context_lookup=frozen.pickup_context)
    request = RetainSuctionObservationV2.Request(request_id='task', observation_id='view',
        domain_sha256=context.domain_sha256, detection=original.detection,
        generation=context.domain.generation)
    counter = [0]

    def call(service, message):
        service_type, handler = {
            'retain': (RetainSuctionObservationV2, frozen.retain),
            'view': (GetPickupViewEvidenceV2, view.get),
        }[service]
        index = counter[0]; counter[0] += 1
        raw = serialize_message(message)
        (case_archive.directory / ('%02d_%s_request.cdr' % (index, service))).write_bytes(raw)
        result = handler(deserialize_message(raw, service_type.Request), service_type.Response())
        output = serialize_message(result)
        (case_archive.directory / ('%02d_%s_response.cdr' % (index, service))).write_bytes(output)
        return deserialize_message(output, service_type.Response)

    def pressure():
        for i in range(1, 4):
            _, newer, _, _ = view_context(stamp_ns=context.domain.stamp_ns+i)
            history.remember(newer.domain, context=newer)
        case_archive.save('rolling_pressure', history.diagnostics())

    def query(binding=None):
        result = GetPickupViewEvidenceV2.Request(domain_sha256=context.domain_sha256,
            detection=original.detection, generation=context.domain.generation)
        if binding is not None:
            result.binding = binding_to_message(binding)
        return result

    np.savez_compressed(case_archive.directory/'input_samples.npz', depth=depth, region=region,
                        support=context.domain.cells())
    case_archive.save('input', dict(domain=context.domain, policy=context._model.as_dict(),
                                  view=context.pickup_view))
    return SimpleNamespace(context=context, now=now, history=history, store=store,
        model=model, frozen=frozen, handler=view, request=request, call=call,
        pressure=pressure, query=query, archive=case_archive)


def retained(case):
    result = case.call('retain', case.request)
    assert result.retained, result.reason
    return result, binding_from_message(result.binding)


def assert_no_view(result):
    assert not result.success
    assert not result.view.evidence_sha256 and not result.view.depth_mm.data


def test_frozen_camera_survives_rolling_pressure_and_owner_forwards_binding(camera_case):
    f = camera_case
    receipt, binding = retained(f)
    assert receipt.pickup_view_available
    assert receipt.retained_observation_bytes >= f.context.pickup_view.size_bytes
    quality = f.store.lookup(binding)
    f.pressure()
    unbound = f.call('view', f.query())
    assert_no_view(unbound)
    assert 'RESOURCE_EVICTED' in unbound.reason
    bound = f.call('view', f.query(binding))
    assert bound.success, bound.reason
    assert view_from_message(bound.view, f.context.domain) == f.context.pickup_view
    assert f.store.lookup(binding) is quality

    # Exercise the real prepare owner's acquisition method, not a spelling check.
    from luggage_planning.joint_operation.view_owner import PickupViewPrepareOwner
    owner = PickupViewPrepareOwner.__new__(PickupViewPrepareOwner)
    owner.evidence_service, owner.evidence_client = GetPickupViewEvidenceV2, object()
    forwarded = []
    def call(client, query, deadline, label):
        assert query.binding == receipt.binding
        forwarded.append(query.binding.binding_sha256)
        return f.call('view', query)
    owner.node = SimpleNamespace(_call=call)
    context = SimpleNamespace(domain=f.context.domain, request=lambda: f.request,
                              observation=SimpleNamespace(binding=binding))
    assert owner._acquisition(context, 100.) == f.context.pickup_view
    assert owner._acquisition(context, 100.) == f.context.pickup_view
    assert len(forwarded) == 2


def test_camera_missing_before_freeze_never_reacquired(camera_case):
    f = camera_case; f.pressure()
    receipt, binding = retained(f)
    assert not receipt.pickup_view_available
    assert receipt.pickup_view_reason == 'PICKUP_VIEW_RESOURCE_EVICTED'
    assert f.store.lookup(binding) is not None
    assert_no_view(f.call('view', f.query(binding)))
    # An exact source becoming available again cannot rewrite an existing token.
    f.history.clear(); f.history.remember(f.context.domain, context=f.context)
    f.request.expected_binding = receipt.binding
    again, _ = retained(f)
    assert not again.pickup_view_available and again.binding == receipt.binding
    assert_no_view(f.call('view', f.query(binding)))


def test_duplicate_retention_and_camera_reads_do_not_renew(camera_case):
    f = camera_case; receipt, binding = retained(f)
    f.now[0] = 9.
    f.request.expected_binding = receipt.binding
    again, _ = retained(f)
    assert again.binding == receipt.binding
    assert f.call('view', f.query(binding)).success
    f.now[0] = 10.
    assert f.store.lookup(binding) is None
    assert_no_view(f.call('view', f.query(binding)))
    assert not f.call('retain', f.request).retained


@pytest.mark.parametrize('damage', ['revoke', 'identity', 'policy', 'binding',
                                  'hash', 'domain', 'generation', 'dimensions', 'partial'])
def test_bound_read_refuses_without_unbound_fallback(camera_case, damage):
    f = camera_case; receipt, binding = retained(f); query = f.query(binding)
    if damage == 'revoke':
        f.store.revoke(binding.request_id, binding.observation_id)
    elif damage == 'identity':
        f.store.revoke_all()
    elif damage == 'policy':
        f.model.cell_size_m *= 2
    elif damage == 'binding':
        query.binding = binding_to_message(replace(binding, retention_id='another'))
    elif damage == 'hash':
        query.binding.binding_sha256 = '0'*64
    elif damage == 'domain':
        query.domain_sha256 = '0'*64
    elif damage == 'generation':
        query.generation += 1
    elif damage == 'dimensions':
        query.detection.width += .001
    else:
        from luggage_msgs.msg import FrozenSuctionObservationV2
        query.binding = FrozenSuctionObservationV2(request_id='partial')
    result = f.call('view', query)
    f.archive.save('refusal', dict(damage=damage, reason=result.reason))
    assert_no_view(result)


def test_revocation_during_bound_payload_copy_returns_empty(camera_case):
    f = camera_case; _, binding = retained(f)
    lookup = f.handler.frozen_context_lookup
    calls = []
    def revoke_on_second(token):
        calls.append(token.digest())
        if len(calls) == 2:
            f.store.revoke_all()
        return lookup(token)
    f.handler.frozen_context_lookup = revoke_on_second
    result = f.call('view', f.query(binding))
    assert_no_view(result)
    assert 'SOURCE_CHANGED_DURING_READ' in result.reason


def quality_bytes(context):
    store = FrozenSuctionObservations(max_records=4, max_bytes=8*1024**2, lifetime_s=10.)
    token = store.retain('first', 'view', context.without_pickup_view('NOT_CAPTURED'))
    return store.pickup_context(token).retained_observation_bytes


@pytest.mark.parametrize('remaining', [0, -1])
def test_exact_total_byte_limit_and_optional_camera_admission(camera_case, remaining):
    f = camera_case; q = quality_bytes(f.context); v = f.context.pickup_view.size_bytes
    store = FrozenSuctionObservations(max_records=4, max_bytes=q+v+remaining, lifetime_s=10.)
    token = store.retain('first', 'view', f.context)
    result = store.pickup_context(token)
    f.archive.save('byte_boundary', dict(quality=q, camera=v, limit=store.max_bytes,
                                       retained=result.retained_observation_bytes))
    assert store.lookup(token) is not None and store._bytes <= store.max_bytes
    assert (result.pickup_view is not None) == (remaining == 0)
    if remaining == -1:
        assert result.pickup_view_reason == 'FROZEN_PICKUP_VIEW_BYTE_LIMIT'


def test_new_camera_cannot_evict_quality_row(camera_case):
    f = camera_case; q = quality_bytes(f.context); v = f.context.pickup_view.size_bytes
    store = FrozenSuctionObservations(max_records=4, max_bytes=2*q+v-1, lifetime_s=10.)
    first = store.retain('first', 'view', f.context.without_pickup_view('NOT_CAPTURED'))
    context = store.lookup(first)
    second = store.retain('other', 'view', f.context)
    assert store.lookup(first) is context and store.lookup(second) is not None
    assert store.pickup_context(second).pickup_view is None
    assert store._bytes <= store.max_bytes


def test_quality_pressure_drops_only_optional_camera_and_keeps_quality_identity(camera_case):
    f = camera_case; q = quality_bytes(f.context)
    store = FrozenSuctionObservations(max_records=4, max_bytes=2*q, lifetime_s=10.)
    first = store.retain('first', 'view', f.context)
    assert store.pickup_context(first).pickup_view is not None
    context = store.lookup(first)
    second = store.retain('other', 'view', f.context.without_pickup_view('NOT_CAPTURED'))
    assert store.lookup(first) is context and store.lookup(second) is not None
    result = store.pickup_context(first)
    assert result.pickup_view is None
    assert result.pickup_view_reason == 'FROZEN_PICKUP_VIEW_RESOURCE_EVICTED'
    assert store._bytes <= store.max_bytes


def test_actual_ros_bound_read_after_pressure_and_terminal_cleanup(camera_case):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    f = camera_case; receipt, binding = retained(f); f.pressure()
    context = Context(); rclpy.init(context=context, domain_id=173)
    server = rclpy.create_node('frozen_pickup_owner', context=context)
    client = rclpy.create_node('frozen_pickup_reader', context=context)
    service = server.create_service(GetPickupViewEvidenceV2, '/frozen_pickup_test', f.handler.get)
    port = client.create_client(GetPickupViewEvidenceV2, '/frozen_pickup_test')
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(server); executor.add_node(client)
    worker = threading.Thread(target=executor.spin, daemon=True); worker.start()
    try:
        assert port.wait_for_service(timeout_sec=3.)
        query = f.query(binding)
        (f.archive.directory/'actual_ros_request.cdr').write_bytes(serialize_message(query))
        future = port.call_async(query)
        complete = threading.Event()
        future.add_done_callback(lambda _future: complete.set())
        assert complete.wait(timeout=3.)
        assert future.done()
        result = future.result()
        (f.archive.directory/'actual_ros_response.cdr').write_bytes(serialize_message(result))
        assert result.success, result.reason
        assert view_from_message(result.view, f.context.domain) == f.context.pickup_view
    finally:
        executor.shutdown(timeout_sec=2.); worker.join(timeout=2.)
        client.destroy_client(port)
        server.destroy_service(service); server.destroy_node(); client.destroy_node()
        rclpy.shutdown(context=context)
        f.archive.save('ros_teardown', dict(worker_alive=worker.is_alive(), context_ok=context.ok()))
    assert not worker.is_alive() and not context.ok()
