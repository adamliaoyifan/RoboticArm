"""Terminal RPC evidence after its parent stops waiting; no controller ports."""
import hashlib
import json
import os
from pathlib import Path
import re
import threading
import time
from types import SimpleNamespace as NS

import pytest
from rclpy.serialization import serialize_message
from rclpy.task import Future
from luggage_msgs.srv import PlanSnapshotJointPathV2 as Service
from luggage_planning.joint_operation.connection import NativeSnapshotConnectionPort


@pytest.fixture
def capture(request, tmp_path):
    root = Path(os.environ.get('CONNECTION_COMPLETION_EVIDENCE', str(tmp_path)))
    directory = root/re.sub(r'[^A-Za-z0-9_.-]', '_', request.node.name)
    directory.mkdir(parents=True, exist_ok=False)
    rows, events, lock = [], [], threading.RLock()
    def record(kind, value):
        raw = value if isinstance(value, bytes) else bytes(serialize_message(value))
        with lock:
            name = f'{len(rows):03d}_{kind}.cdr'
            (directory/name).write_bytes(raw)
            rows.append(dict(kind=kind, file=name, bytes=len(raw), sha256=hashlib.sha256(raw).hexdigest(),
                request_id=getattr(value, 'request_id', None), monotonic_ns=time.monotonic_ns()))
    result = NS(directory=directory, rows=rows, events=events, record=record)
    yield result
    (directory/'case.json').write_text(json.dumps(dict(records=rows, events=events), indent=2)+'\n')


def snapshot():
    return NS(identity='one-private-scene', model_hash='model', scene_cdr=b'fixture-scene')


def reply(query, *, success=False):
    response = Service.Response(request_id=query.request_id, snapshot_id=query.snapshot_id,
        model_sha256=query.model_sha256, scene_sha256=query.scene_sha256,
        complete=True, success=success, reason='CHECKED' if success else 'NO_CONNECTION')
    if success:
        from trajectory_msgs.msg import JointTrajectoryPoint
        from luggage_planning.joint_operation.contracts import OPERATION_JOINTS
        response.geometric_path.joint_names = list(OPERATION_JOINTS)
        response.geometric_path.points = [JointTrajectoryPoint(positions=list(q))
            for q in (query.start_joints, [.01]*6, query.goal_joints)]
        response.checked_states = 3
    return response


class RegisteredFuture(Future):
    def __init__(self):
        super().__init__()
        self.registered = threading.Event()

    def add_done_callback(self, callback):
        super().add_done_callback(callback)
        self.registered.set()


class Client:
    def __init__(self, on_send=None):
        self.future = RegisteredFuture()
        self.queries = []
        self.on_send = on_send

    def service_is_ready(self):
        return True

    def call_async(self, query):
        self.queries.append(query)
        if self.on_send:
            self.on_send(self, query)
        return self.future


def port(capture, client):
    result = object.__new__(NativeSnapshotConnectionPort)
    result.service, result.client, result.recorder = Service, client, capture.record
    result.inflight, result.max_edge_s = None, 1.
    result.logger = NS(error=lambda text: capture.events.append(dict(error=text)))
    return result


def connect(port, *, deadline=None, cancelled=lambda: False):
    return port.connect(snapshot(), (0.,)*6, (.02,)*6, request_id='original-request',
        deadline=time.monotonic()+1. if deadline is None else deadline, cancelled=cancelled)


@pytest.mark.parametrize('already_done', [True, False])
def test_terminal_response_is_recorded_once_and_validated(capture, already_done):
    client = Client()
    worker = None
    if already_done:
        client.on_send = lambda c, query: c.future.set_result(reply(query, success=True))
    else:
        def produce():
            assert client.future.registered.wait(1.)
            client.future.set_result(reply(client.queries[0], success=True))
        worker = threading.Thread(target=produce); worker.start()
    try:
        outcome = connect(port(capture, client))
        assert outcome.success and outcome.complete and len(outcome.positions) == 3
        assert [r['kind'] for r in capture.rows] == ['request', 'response']
        assert capture.rows[0]['request_id'] == capture.rows[1]['request_id'] == 'original-request'
    finally:
        if worker is not None:
            worker.join(timeout=2.)
            assert not worker.is_alive()


@pytest.mark.parametrize('stop', ['deadline', 'cancel'])
def test_late_success_remains_evidence_and_cannot_promote_stopped_call(capture, stop):
    cancelled = threading.Event()
    client = Client(on_send=(lambda c,q: cancelled.set()) if stop == 'cancel' else None)
    owner = port(capture, client)
    outcome = connect(owner, deadline=time.monotonic()+.02, cancelled=cancelled.is_set)
    capture.events.append(dict(caller_return=outcome.reason))
    assert outcome.reason == 'CANCELLED_OR_DEADLINE' and not outcome.complete and not outcome.positions
    assert [r['kind'] for r in capture.rows] == ['request']
    busy = connect(owner)
    assert busy.reason == 'CONNECTION_BACKEND_BUSY' and len(client.queries) == 1
    client.future.set_result(reply(client.queries[0], success=True))
    assert [r['kind'] for r in capture.rows] == ['request', 'response']
    assert not outcome.success and not outcome.positions and len(client.queries) == 1


@pytest.mark.parametrize('fault', ['transport', 'cancelled_future', 'recorder'])
def test_completion_errors_are_diagnosed_and_release_the_caller(capture, fault):
    def send(client, query):
        if fault == 'transport': client.future.set_exception(RuntimeError('transport failed'))
        elif fault == 'cancelled_future': client.future.cancel()
        else: client.future.set_result(reply(query))
    owner = port(capture, Client(on_send=send))
    if fault == 'recorder':
        def record(kind, value):
            if kind == 'response': raise OSError('capture failed')
            capture.record(kind, value)
        owner.recorder = record
    with pytest.raises((RuntimeError, OSError)):
        connect(owner)
    assert [r['kind'] for r in capture.rows] == ['request', 'error.json']
    assert any('CONNECTION_COMPLETION_FAILED' in e.get('error', '') for e in capture.events)


def test_slow_async_capture_survives_parent_deadline(capture):
    client = Client(); owner = port(capture, client)
    entered, release = threading.Event(), threading.Event()
    def record(kind, value):
        if kind == 'response':
            entered.set()
            assert release.wait(2.)
        capture.record(kind, value)
    owner.recorder = record
    def produce():
        assert client.future.registered.wait(1.)
        client.future.set_result(reply(client.queries[0], success=True))
    worker = threading.Thread(target=produce); worker.start()
    try:
        outcome = connect(owner, deadline=time.monotonic()+.05)
        assert entered.is_set() and outcome.reason == 'CANCELLED_OR_DEADLINE'
        assert [r['kind'] for r in capture.rows] == ['request']
    finally:
        release.set(); worker.join(timeout=2.)
    assert not worker.is_alive()
    assert [r['kind'] for r in capture.rows] == ['request', 'response']


@pytest.mark.parametrize('stop', ['deadline', 'cancel'])
def test_real_ros_service_reply_is_retained_after_parent_return(capture, stop):
    import rclpy
    context = rclpy.context.Context()
    old_localhost = os.environ.get('ROS_LOCALHOST_ONLY')
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    rclpy.init(context=context, domain_id=177)
    if old_localhost is None: os.environ.pop('ROS_LOCALHOST_ONLY', None)
    else: os.environ['ROS_LOCALHOST_ONLY'] = old_localhost
    server = rclpy.create_node('connection_delayed_service', context=context)
    caller = rclpy.create_node('connection_completion_client', context=context)
    entered, release, recorded, cancelled = [threading.Event() for _ in range(4)]
    requests = []
    def serve(query, response):
        requests.append(query.request_id); entered.set()
        assert release.wait(2.), 'FIXTURE_REPLY_NOT_RELEASED'
        return reply(query, success=True)
    service = server.create_service(Service, '/joint_operation/plan_connection', serve)
    executor = rclpy.executors.SingleThreadedExecutor(context=context)
    executor.add_node(server); executor.add_node(caller)
    worker = threading.Thread(target=executor.spin); worker.start()
    def record(kind, value):
        capture.record(kind, value)
        if kind == 'response': recorded.set()
    owner = NativeSnapshotConnectionPort(caller, recorder=record)
    trigger = None
    try:
        assert owner.client.wait_for_service(timeout_sec=3.)
        if stop == 'cancel':
            def cancel_after_delivery():
                if entered.wait(1.): cancelled.set()
            trigger = threading.Thread(target=cancel_after_delivery); trigger.start()
        outcome = connect(owner, deadline=time.monotonic()+(.15 if stop == 'deadline' else 1.),
                          cancelled=cancelled.is_set)
        capture.events.append(dict(caller_return=outcome.reason, response_recorded=recorded.is_set()))
        assert outcome.reason == 'CANCELLED_OR_DEADLINE' and not outcome.positions
        assert entered.wait(1.) and not recorded.is_set()
        assert connect(owner).reason == 'CONNECTION_BACKEND_BUSY'
        release.set()
        assert recorded.wait(2.)
        assert requests == ['original-request']
        assert [r['kind'] for r in capture.rows] == ['request', 'response']
        assert capture.rows[-1]['request_id'] == 'original-request' and not outcome.success
    finally:
        release.set()
        if trigger is not None: trigger.join(timeout=2.)
        executor.shutdown(timeout_sec=3.); worker.join(timeout=3.)
        caller.destroy_node(); server.destroy_service(service); server.destroy_node(); context.shutdown()
        (capture.directory/'teardown.json').write_text(json.dumps(dict(worker_stopped=not worker.is_alive(),
            cancel_worker_stopped=trigger is None or not trigger.is_alive())))
    assert not worker.is_alive()
