"""Non-motion ROS graph tests: real actions/services, synthetic data only."""
import errno
import json
import os
from pathlib import Path
import threading
import time

import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rclpy.task import Future
from control_msgs.action import FollowJointTrajectory as FJT
from moveit_msgs.action import ExecuteTrajectory as Execute
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from unique_identifier_msgs.msg import UUID

from luggage_planning.async_boundary_capture import AsyncBoundaryCapture, read_stream
from luggage_planning.diagnostic_action_capture import CaptureNode, EXECUTE, FJT as FJT_NAME, PREFIX
from luggage_planning.diagnostic_capture_routes import action_remappings

def cli_remaps(source):
    return [value for pair in action_remappings(source) for value in ('-r', pair[0] + ':=' + pair[1]) ]


def output(tmp_path, name):
    root = Path(os.environ.get('CAPTURE_WIRING_EVIDENCE', tmp_path))
    root.mkdir(exist_ok=True, parents=True)
    return root / name


def until(predicate, timeout=8.):
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= deadline:
            raise AssertionError('non-motion graph deadline exceeded')
        time.sleep(.005)


def result(future):
    until(future.done)
    return future.result()


@pytest.fixture
def graph():
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=8)
    nodes = []
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    def add(node):
        nodes.append(node)
        executor.add_node(node)
        return node
    yield add
    executor.shutdown(timeout_sec=5.)
    spinner.join(timeout=5.)
    for node in reversed(nodes):
        node.destroy_node()
    rclpy.shutdown()


@pytest.mark.parametrize('mode', ['success', 'abort', 'reject', 'cancel', 'capture_fault'])
def test_actual_fjt_uuid_and_result(graph, tmp_path, mode):
    capture = graph(CaptureNode(output(tmp_path, 'action_' + mode), 'synthetic-' + mode))
    backend = graph(Node('fake_movegroup', cli_args=['--ros-args'] + cli_remaps(FJT_NAME)))
    controller_node = graph(Node('fake_controller'))
    group = ReentrantCallbackGroup()
    seen = []
    client_fjt = ActionClient(backend, FJT, FJT_NAME, callback_group=group)

    async def controller(handle):
        seen.append((handle.goal_id, handle.request))
        if mode in ('cancel', 'capture_fault'):
            while not handle.is_cancel_requested:
                future = Future()
                timer = backend.create_timer(.01, lambda: future.set_result(True),
                                              callback_group=group)
                await future
                backend.destroy_timer(timer)
            handle.canceled()
            return FJT.Result(error_code=-1, error_string='synthetic canceled')
        if mode == 'abort':
            handle.abort()
            return FJT.Result(error_code=-4, error_string='synthetic abort')
        handle.publish_feedback(FJT.Feedback(joint_names=list(handle.request.trajectory.joint_names)))
        handle.succeed()
        return FJT.Result(error_code=0)

    controller_server = ActionServer(controller_node, FJT, FJT_NAME, controller,
        goal_callback=lambda _: GoalResponse.REJECT if mode == 'reject' else GoalResponse.ACCEPT,
        cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group)

    forwarding = {}
    def cancel_movegroup(handle):
        downstream = forwarding.get('fjt')
        if downstream is not None:
            downstream.cancel_goal_async()
        return CancelResponse.ACCEPT

    async def movegroup(handle):
        # Deliberately differs from executor input: proves FJT isn't a copied cert.
        goal = FJT.Goal(trajectory=handle.request.trajectory.joint_trajectory)
        goal.trajectory.points[0].positions = [.875, -.25]
        goal.trajectory.header.frame_id = 'fake-movegroup-output'
        downstream = await client_fjt.send_goal_async(goal, goal_uuid=UUID(uuid=[7]*16))
        forwarding['fjt'] = downstream
        if not downstream.accepted:
            handle.abort()
            return Execute.Result()
        wrapped = await downstream.get_result_async()
        outcome = Execute.Result()
        outcome.error_code.val = 1 if wrapped.status == 4 else -4
        if wrapped.status == 4:
            handle.succeed()
        elif handle.is_cancel_requested:
            handle.canceled()
        else:
            handle.abort()
        return outcome

    execute_server = ActionServer(backend, Execute, EXECUTE, movegroup,
        cancel_callback=cancel_movegroup, callback_group=group)
    caller = graph(Node('fake_executor', cli_args=['--ros-args'] + cli_remaps(EXECUTE)))
    client = ActionClient(caller, Execute, EXECUTE)
    until(lambda: client.server_is_ready() and client_fjt.server_is_ready()
          and capture.execute_capture.client.server_is_ready()
          and capture.fjt_capture.client.server_is_ready())
    goal = Execute.Goal()
    trajectory = goal.trajectory.joint_trajectory
    trajectory.joint_names = ['j2', 'j1']
    trajectory.header.stamp.sec, trajectory.header.stamp.nanosec = 8, 987654321
    trajectory.points = [JointTrajectoryPoint(positions=[.1, .2], velocities=[-.1, .1],
                                              accelerations=[], effort=[])]
    trajectory.points[0].time_from_start.nanosec = 999999999
    handle = result(client.send_goal_async(goal, goal_uuid=UUID(uuid=[3]*16)))
    assert handle.accepted
    if mode in ('cancel', 'capture_fault'):
        until(lambda: bool(seen))
        if mode == 'cancel':
            result(handle.cancel_goal_async())
        else:
            from std_msgs.msg import String
            publisher = caller.create_publisher(String, PREFIX + '/fault', 10)
            until(lambda: publisher.get_subscription_count() == 1)
            publisher.publish(String(data=json.dumps(dict(
                run_id='synthetic-' + mode, name='CAPTURE_WRITE_FAILED',
                detail='synthetic driver disk fault during execution'))))
    wrapped = result(handle.get_result_async())
    assert wrapped.status == (4 if mode == 'success' else 5 if mode == 'cancel' else 6)
    until(lambda: not capture.execute_capture.identities() and not capture.fjt_capture.identities())
    time.sleep(.1)
    health = capture.close_capture()
    entries = list(read_stream(capture.recorder.root))
    by_boundary = {}
    for metadata, payload in entries:
        by_boundary.setdefault(metadata['boundary'], []).append((metadata, payload))
    execute_request = by_boundary[EXECUTE + ':send'][0]
    assert execute_request[0]['goal_id'] == '03'*16
    original = deserialize_message(execute_request[1], Execute.Impl.SendGoalService.Request)
    assert original.goal.trajectory.joint_trajectory.points[0].positions[0] == .1
    received = by_boundary[FJT_NAME + ':receive_goal'][0]
    actual = deserialize_message(received[1], FJT.Impl.SendGoalService.Request)
    assert received[0]['goal_id'] == '07'*16
    assert actual.goal.trajectory.header.stamp.nanosec == 987654321
    assert actual.goal.trajectory.header.frame_id == 'fake-movegroup-output'
    assert list(actual.goal.trajectory.points[0].positions) == [.875, -.25]
    assert list(actual.goal.trajectory.points[0].accelerations) == []
    assert actual.goal.trajectory.points[0].time_from_start.nanosec == 999999999
    sent = by_boundary[FJT_NAME + ':send'][0]
    assert sent[0]['correlation']['executor_goal_ids'] == ['03'*16]
    assert deserialize_message(sent[1], FJT.Impl.SendGoalService.Request) == actual
    if mode not in ('reject', 'capture_fault'):
        assert seen[0][0] == actual.goal_id
        assert seen[0][1] == actual.goal
        assert by_boundary[FJT_NAME + ':result'][0][0]['goal_id'] == '07'*16
        assert by_boundary[EXECUTE + ':result'][0][0]['goal_id'] == '03'*16
    assert bool(health['errors']) == (mode == 'capture_fault')
    (capture.recorder.root / 'assertions.json').write_text(json.dumps(dict(
        synthetic_only=True, mode=mode, status=wrapped.status,
        executor_uuid='03'*16, fjt_uuid='07'*16,
        source='actual_stub_action_boundary', health=health), indent=2))
    client.destroy()
    client_fjt.destroy()
    execute_server.destroy()
    controller_server.destroy()


def test_disk_block_never_blocks_callback_and_overflow_is_named(tmp_path):
    entered, release = threading.Event(), threading.Event()
    def blocked_open(*args):
        entered.set()
        release.wait(3.)
        return open(*args)
    recorder = AsyncBoundaryCapture(output(tmp_path, 'queue_full'), run_id='blocked',
                                     capacity=2, opener=blocked_open)
    assert entered.wait(1.)
    started = time.monotonic()
    assert recorder.record('synthetic', JointState())
    assert recorder.record('synthetic', JointState())
    assert not recorder.record('synthetic', JointState())
    assert time.monotonic() - started < .1  # disk remains blocked throughout
    release.set()
    health = recorder.close()
    assert health['errors'][0]['name'] == 'CAPTURE_QUEUE_FULL'
    assert (health['received'], health['written'], health['dropped']) == (3, 2, 1)


def test_write_failure_is_named_with_counts(tmp_path):
    def fail_open(*args):
        raise OSError(errno.ENOSPC, 'synthetic no space')
    recorder = AsyncBoundaryCapture(output(tmp_path, 'disk_fault'), run_id='disk-fault',
                                     opener=fail_open)
    until(lambda: bool(recorder.health()['errors']))
    assert not recorder.record('synthetic', JointState())
    health = recorder.close()
    assert health['errors'][0]['name'] == 'CAPTURE_WRITE_FAILED'
    assert 'no space' in health['errors'][0]['detail']
    assert health['received'] == health['written'] + health['failed'] + health['dropped']


def test_synthetic_production_rate_ros_subscriptions(graph, tmp_path):
    from control_msgs.msg import JointTrajectoryControllerState
    from rosgraph_msgs.msg import Clock
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    directory = output(tmp_path, 'rate_1750hz')
    capture = graph(CaptureNode(directory, 'synthetic-1750hz'))
    recorder = capture.recorder
    node = graph(Node('synthetic_high_rate'))
    callback_ns = []
    original = recorder.record
    def measured(*args, **kwargs):
        started = time.monotonic_ns()
        accepted = original(*args, **kwargs)
        callback_ns.append(time.monotonic_ns() - started)
        return accepted
    recorder.record = measured
    heartbeats = []
    kinds = [('/clock', Clock), ('/joint_states', JointState),
             ('/elfin_arm_controller/controller_state', JointTrajectoryControllerState)]
    publishers = [node.create_publisher(kind, topic, 4096) for topic, kind in kinds]
    timer = node.create_timer(.005, lambda: heartbeats.append(time.monotonic_ns()))
    until(lambda: all(pub.get_subscription_count() == 1 for pub in publishers))
    count, rate = 7000, 1750
    published = [0, 0, 0]
    start_ns = time.monotonic_ns()
    start = start_ns / 1e9
    for index in range(count):
        kind_index = 0 if index % 7 < 5 else index % 7 - 4
        if kind_index == 0:
            message = Clock()
            stamp = message.clock
        elif kind_index == 1:
            message = JointState(name=['j%d' % i for i in range(6)],
                position=[float(index)]*6, velocity=[.01]*6, effort=[])
            stamp = message.header.stamp
        else:
            message = JointTrajectoryControllerState(joint_names=['j%d' % i for i in range(6)])
            message.desired.positions = [float(index)]*6
            message.actual.positions = [.125 + index]*6
            message.error.positions = [.125]*6
            stamp = message.header.stamp
        stamp.sec, stamp.nanosec = 123, index * 100003 + 17
        publishers[kind_index].publish(message)
        published[kind_index] += 1
        delay = start + (index + 1) / rate - time.monotonic()
        if delay > 0:
            time.sleep(delay)
    end_ns = time.monotonic_ns()
    elapsed = (end_ns - start_ns) / 1e9
    until(lambda: len(callback_ns) == count)
    health = capture.close_capture()
    entries = list(read_stream(directory))
    assert health['received'] == health['written'] == count
    assert health['dropped'] == health['failed'] == 0
    assert health['recorder_io_complete']
    by_topic = {topic: [] for topic, _ in kinds}
    for metadata, payload in entries:
        by_topic[metadata['boundary']].append(metadata['source_stamp_ns'])
    for kind_index, (topic, _) in enumerate(kinds):
        expected = [123*10**9 + index*100003 + 17 for index in range(count)
                    if (0 if index % 7 < 5 else index % 7 - 4) == kind_index]
        assert sorted(by_topic[topic]) == expected
    p99 = sorted(callback_ns)[int(count*.99)] / 1e6
    assert p99 < 1000 / rate
    workload_beats = [stamp for stamp in heartbeats if start_ns <= stamp <= end_ns]
    probe_times = [start_ns] + workload_beats + [end_ns]
    max_probe_gap_ms = max(b-a for a,b in zip(probe_times, probe_times[1:])) / 1e6
    assert len(workload_beats) >= 600
    assert max_probe_gap_ms < 50.
    report = dict(synthetic_only=True, published=count, per_topic_published=published,
                  elapsed_s=elapsed, offered_hz=count/elapsed, target_hz=rate,
                  target_topic_hz=dict(zip([p[0] for p in kinds], [1250,250,250])),
                  callback_p99_ms=p99, callback_max_ms=max(callback_ns)/1e6,
                  execution_heartbeats=len(workload_beats), probe_max_gap_ms=max_probe_gap_ms,
                  measurement_window='publication workload only; excludes setup/drain', health=health)
    (directory / 'rate_result.json').write_text(json.dumps(report, indent=2))


def test_certificate_request_and_response_through_actual_remap(graph, tmp_path):
    from luggage_msgs.srv import ValidateRouteV2
    from luggage_msgs.msg import RouteLegV2
    target = '/validate_route_v2'
    capture = graph(CaptureNode(output(tmp_path, 'certificate'), 'synthetic-certificate'))
    capture.bridge_service(ValidateRouteV2, target)
    backend = graph(Node('fake_certifier'))
    seen = []
    def certify(request, response):
        seen.append(request)
        response.request_id = request.request_id
        response.route_digest = request.route_digest
        response.scene_digest = request.scene_digest
        response.all_valid = True
        return response
    server = backend.create_service(ValidateRouteV2, target, certify)
    caller = graph(Node('fake_certificate_client', cli_args=[
        '--ros-args', '-r', target + ':=' + PREFIX + target]))
    client = caller.create_client(ValidateRouteV2, target)
    until(lambda: client.service_is_ready() and capture.service_bridges[0][0].service_is_ready())
    request = ValidateRouteV2.Request(request_id='same-run-cert-1', route_digest='synthetic-route')
    leg = RouteLegV2()
    leg.knots.joint_names = ['j2', 'j1']
    leg.knots.points = [JointTrajectoryPoint(positions=[.25, -.3], velocities=[],
        accelerations=[], effort=[])]
    leg.knots.points[0].time_from_start.nanosec = 123456789
    request.legs = [leg]
    response = result(client.call_async(request))
    assert response.all_valid
    assert seen == [request]
    assert capture.close_capture()['recorder_io_complete']
    records = list(read_stream(capture.recorder.root))
    first = next(r for r in records if r[0]['boundary'] == target + ':request')
    last = next(r for r in records if r[0]['boundary'] == target + ':response')
    assert first[0]['correlation'] == last[0]['correlation']
    assert deserialize_message(first[1], ValidateRouteV2.Request) == request
    assert deserialize_message(last[1], ValidateRouteV2.Response) == response


def test_mid_write_failure_preserves_accounting_and_names_error(tmp_path):
    entered, release = threading.Event(), threading.Event()
    class BrokenDisk:
        def __enter__(self):
            entered.set()
            release.wait(2.)
            return self
        def __exit__(self, *args):
            pass
        def write(self, frame):
            raise OSError(errno.ENOSPC, 'synthetic mid-write failure')
    recorder = AsyncBoundaryCapture(output(tmp_path, 'mid_write_fault'), run_id='mid-write',
                                    opener=lambda *args: BrokenDisk())
    assert entered.wait(1.)
    assert recorder.record('pending-1', JointState())
    assert recorder.record('pending-2', JointState())
    release.set()
    until(lambda: bool(recorder.health()['errors']))
    health = recorder.close()
    assert health['errors'][0]['name'] == 'CAPTURE_WRITE_FAILED'
    assert 'mid-write failure' in health['errors'][0]['detail']
    assert health['received'] == health['failed'] == 2
    assert health['written'] == health['pending'] == health['queued_bytes'] == 0
