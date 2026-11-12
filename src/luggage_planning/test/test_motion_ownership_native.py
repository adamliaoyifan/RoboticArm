"""Actual motion-node action admission and ROS transports; synthetic controller.

The legacy planning body is replaced by a bounded transport fixture. Its real
registered execute callbacks, ownership and three real action clients remain.
The bound leg uses actual native suffix validation. This gate certifies motion
ownership wiring, not the still separate vacuum/observation execution adoption.
"""
from dataclasses import asdict
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import pytest

from test_joint_operation_motion_native import controller
from test_joint_operation_revalidation_native import native_gate, issued, context, port
from luggage_planning.joint_operation.motion import StrictOperationMotionPort

pytestmark = pytest.mark.skipif(not os.environ.get('JOINT_NATIVE_CERTIFIER_EXECUTABLE'),
    reason='explicit isolated native/controller ownership gate')


def until(predicate, timeout=3.):
    end = time.monotonic()+timeout
    while time.monotonic()<end:
        if predicate():
            return
        time.sleep(.005)
    assert predicate(), 'bounded condition did not occur'


def result(future, timeout=3.):
    event = threading.Event()
    future.add_done_callback(lambda _: event.set())
    assert event.wait(timeout), 'bounded fixture RPC did not finish'
    return future.result()


@pytest.fixture
def driver(native_gate, controller, request):
    import rclpy
    from control_msgs.action import FollowJointTrajectory
    from moveit_msgs.action import ExecuteTrajectory, MoveGroup
    from rclpy.action import ActionClient, ActionServer
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.parameter import Parameter
    from rclpy.serialization import serialize_message
    from trajectory_msgs.msg import JointTrajectoryPoint
    from luggage_msgs.action import GoToRobotPose, PlanMotion, PlanMotionV2
    from luggage_planning.joint_operation.contracts import OPERATION_JOINTS
    source = Path(__file__).parents[1]/'scripts/motion_planner_node.py'
    spec = importlib.util.spec_from_file_location('owned_motion_node', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    scene = Path(__file__).parents[2]/'luggage_description/config/scene_tf_deploy_update.yaml'
    node = module.MotionPlannerNode(context=native_gate.node.context, parameter_overrides=[
        Parameter('scene_tf_config', value=str(scene)), Parameter('atlas_enabled', value=False),
        Parameter('debug_capture', value=False)])
    c = controller
    c.state.mode = 'success'
    c.state.accept_gate.set()
    c.state.execution_gate.clear()
    state = SimpleNamespace(mode='wait', session=None, outcomes=[], transports=[], workers=[])
    def record(kind, message):
        c.record('owner_'+kind, serialize_message(message))
    def execute_fjt(handle):
        record('fjt_goal', handle.request)
        handle.succeed()
        response = FollowJointTrajectory.Result()
        record('fjt_result', response)
        return response
    def execute_movegroup(handle):
        record('movegroup_goal', handle.request)
        handle.succeed()
        response = MoveGroup.Result()
        response.error_code.val = 1
        record('movegroup_result', response)
        return response
    servers = [ActionServer(node, FollowJointTrajectory,
        '/elfin_arm_controller/follow_joint_trajectory', execute_fjt,
        callback_group=ReentrantCallbackGroup()),
        ActionServer(node, MoveGroup, '/move_action', execute_movegroup,
        callback_group=ReentrantCallbackGroup())]
    interfaces = dict(v1=(PlanMotion, '/motion_planner/plan_motion'),
        v2=(PlanMotionV2, '/motion_planner/plan_motion_v2'),
        pose=(GoToRobotPose, '/motion_planner/go_to_robot_pose'))
    clients = {name: ActionClient(node, typ, endpoint, callback_group=ReentrantCallbackGroup())
               for name, (typ, endpoint) in interfaces.items()}
    def handler(kind):
        def callback(handle):
            # Exercise the actual wrapped clients installed by MotionPlannerNode.
            record(kind+'_admitted', handle.request)
            def capture_acceptance(future):
                inner = future.result()
                c.record('owner_transport_binding', json.dumps(dict(
                    outer_goal_uuid=bytes(handle.goal_id.uuid).hex(),
                    inner_goal_uuid=bytes(inner.goal_id.uuid).hex(), kind=kind,
                    accepted=inner.accepted)).encode())
            if kind == 'pose':
                goal = FollowJointTrajectory.Goal()
                client = node._fjt
            else:
                if kind == 'v1':
                    planning = MoveGroup.Goal()
                    planning.planning_options.plan_only = True
                    pending = node._executor_client._move_group.send_goal_async(planning)
                    pending.add_done_callback(capture_acceptance)
                    first = result(pending)
                    assert result(first.get_result_async()).status == 4
                goal = ExecuteTrajectory.Goal()
                goal.trajectory.joint_trajectory.joint_names = list(OPERATION_JOINTS)
                goal.trajectory.joint_trajectory.points = [JointTrajectoryPoint(positions=[0.]*6)]
                client = node._executor_client._execute
            state.transports.append(kind)
            future = client.send_goal_async(goal)
            future.add_done_callback(capture_acceptance)
            response = interfaces[kind][0].Result()
            if state.mode == 'return_before_accept':
                handle.abort()
                response.message = 'synthetic acceptance timeout'
                return response
            accepted = result(future)
            if state.mode == 'return_before_result':
                handle.abort()
                response.message = 'synthetic result timeout'
                return response
            wrapped = result(accepted.get_result_async())
            response.success = wrapped.status == 4
            handle.succeed() if response.success else handle.abort()
            return response
        return callback
    node._execute_plan_motion_owned = handler('v1')
    node._execute_plan_motion_v2_owned = handler('v2')
    node._execute_goto_pose_owned = handler('pose')
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=8, context=native_gate.node.context)
    executor.add_node(node)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    strict = StrictOperationMotionPort(node, motion_lock=node._motion_owner.motion_lock,
        start_tolerance_rad=.01, end_tolerance_rad=.01, max_joint_age_ns=10**9, recorder=c.record)
    def start(kind):
        typ, _ = interfaces[kind]
        goal = typ.Goal()
        record(kind+'_submitted', goal)
        accepted = result(clients[kind].send_goal_async(goal))
        assert accepted.accepted
        return accepted
    def send(kind):
        return start(kind).get_result_async()
    try:
        for client in clients.values():
            assert client.wait_for_server(timeout_sec=3.)
        for client in (node._fjt, node._executor_client._move_group, node._executor_client._execute):
            assert client.wait_for_server(timeout_sec=3.)
        yield SimpleNamespace(node=node, c=c, state=state, strict=strict, send=send,
                              start=start, record=record, owner=node._motion_owner)
    finally:
        # Persist the triggering boundary before any cleanup/unblocking.
        case = hashlib.sha256(request.node.nodeid.encode()).hexdigest()[:12]
        artifact = native_gate.directory/('ownership_'+case+'.json')
        captured = dict(case=request.node.nodeid, before_cleanup=node._motion_owner.snapshot(),
            outcomes=state.outcomes, transports=state.transports,
            scope='actual registered motion-node ownership, synthetic legacy planning/controller')
        artifact.write_text(json.dumps(captured, indent=2))
        c.state.accept_gate.set()
        c.state.execution_gate.set()
        strict.cancel()
        def motion_stopped():
            strict.poll_terminal()
            return not node._motion_owner.motion_lock.locked()
        until(motion_stopped)
        for execution_worker in state.workers:
            execution_worker.join(3.)
            assert not execution_worker.is_alive()
        if state.session is not None:
            node._motion_owner.finish(state.session)
        until(lambda: node._motion_owner.snapshot()['active'] is None)
        captured['after_cleanup'] = node._motion_owner.snapshot()
        for server in servers+[node._plan_action, node._plan_v2_action, node._pose_action]:
            server.destroy()
        for client in clients.values():
            client.destroy()
        strict.client.destroy()
        executor.shutdown(timeout_sec=3.)
        worker.join(3.)
        node.destroy_node()
        captured['worker_joined'] = not worker.is_alive()
        artifact.write_text(json.dumps(captured, indent=2))
        assert not worker.is_alive()


@pytest.mark.parametrize('kind', ['v1', 'v2', 'pose'])
def test_actual_registered_entries_and_wrapped_clients_complete(driver, kind):
    d = driver
    observed = result(d.send(kind))
    d.state.outcomes.append(dict(kind=kind, success=observed.result.success, status=observed.status))
    assert observed.status == 4 and observed.result.success
    until(lambda: d.owner.snapshot()['active'] is None)
    assert d.state.transports == [kind]


@pytest.mark.parametrize('mode', ['return_before_accept', 'return_before_result'])
def test_legacy_timeout_excludes_all_entries_until_real_controller_terminal(driver, mode):
    d = driver
    d.state.mode = mode
    d.c.state.mode = 'hold'
    if mode == 'return_before_accept':
        d.c.state.accept_gate.clear()
    before = len(d.c.state.cancelled)
    response = result(d.send('v2'))
    assert response.status == 6 and not response.result.success
    assert d.owner.motion_lock.locked()
    for kind in ('v1', 'v2', 'pose'):
        refused = result(d.send(kind))
        d.state.outcomes.append(dict(kind=kind, success=refused.result.success, reason=refused.result.message))
        assert refused.status == 6 and refused.result.message == 'SHARED_MOTION_BUSY'
    assert d.state.transports == ['v2']
    d.c.state.accept_gate.set()
    until(lambda: len(d.c.state.cancelled) > before)
    assert d.owner.motion_lock.locked()  # Cancel acknowledgement is insufficient.
    d.c.state.execution_gate.set()
    until(lambda: d.owner.snapshot()['active'] is None)


def bound_options(d, native_gate, issued):
    plan = issued[0]
    d.c.update(plan.legs[0].knots[0].positions)
    arguments = context(native_gate, issued, 0)
    native = port(native_gate)
    def validate(plan, index, **limits):
        values = dict(arguments)
        values.update(limits)
        values['actual_joints'] = d.c.state.q
        return native.validate(plan, **values)
    return plan, dict(expected_identity=plan.identity, expected_digest=plan.plan_digest,
        geometry=None, validate_current=validate, source_current=lambda *a: True,
        read_joints=d.c.sample, deadline=time.monotonic()+5., cancelled=lambda: False)


def test_bound_session_uses_same_submission_lock_and_excludes_legacy_between_legs(driver, native_gate, issued):
    d = driver
    d.state.session = d.owner.begin('native bound-operation fixture')
    assert d.state.session is not None
    plan, options = bound_options(d, native_gate, issued)
    outcome = d.strict.execute(plan, 0, **options)
    d.state.outcomes.append(asdict(outcome))
    assert outcome.success and outcome.terminal
    assert not d.owner.motion_lock.locked()
    for kind in ('v1', 'v2', 'pose'):
        observed = result(d.send(kind))
        assert observed.result.message == 'SHARED_MOTION_BUSY'
    assert d.state.transports == []
    assert d.owner.finish(d.state.session)
    observed = result(d.send('pose'))
    assert observed.result.success


def test_outer_cancel_reaches_real_inner_controller_and_does_not_release_ownership(driver):
    d = driver
    d.c.state.mode = 'hold'
    before = len(d.c.state.cancelled)
    goals = len(d.c.state.goals)
    handle = d.start('v2')
    until(lambda: len(d.c.state.goals) > goals)
    acknowledgement = result(handle.cancel_goal_async())
    d.record('outer_cancel_ack', acknowledgement)
    d.state.outcomes.append(dict(external_goal_id=bytes(handle.goal_id.uuid).hex(),
        external_cancel_accepted=bool(acknowledgement.goals_canceling)))
    assert acknowledgement.goals_canceling
    until(lambda: len(d.c.state.cancelled) > before, timeout=1.)
    assert d.owner.motion_lock.locked()
    refused = result(d.send('pose'))
    assert refused.result.message == 'SHARED_MOTION_BUSY'
    d.c.state.execution_gate.set()
    stopped = result(handle.get_result_async())
    assert not stopped.result.success
    until(lambda: d.owner.snapshot()['active'] is None)


def test_bound_nonterminal_cancel_cannot_release_operation_session(driver, native_gate, issued):
    d = driver
    d.state.session = d.owner.begin('bound cancellation fixture')
    plan, options = bound_options(d, native_gate, issued)
    d.c.state.mode = 'hold'
    outcomes = []
    cancelled = threading.Event()
    options['cancelled'] = cancelled.is_set
    count = len(d.c.state.goals)
    worker = threading.Thread(target=lambda: outcomes.append(d.strict.execute(plan, 0, **options)))
    d.state.workers.append(worker)
    worker.start()
    until(lambda: len(d.c.state.goals) > count)
    cancelled.set()
    worker.join(2.)
    assert not worker.is_alive() and len(outcomes) == 1
    outcome = outcomes[0]
    d.state.outcomes.append(asdict(outcome))
    assert not outcome.terminal and d.owner.motion_lock.locked()
    assert not d.owner.finish(d.state.session)
    observed = result(d.send('pose'))
    assert observed.result.message == 'SHARED_MOTION_BUSY'
    d.c.state.execution_gate.set()
    terminal = []
    def poll():
        value = d.strict.poll_terminal()
        if value is not None and value.terminal:
            terminal.append(value)
        return bool(terminal)
    until(poll)
    assert d.owner.snapshot()['active'] is not None
    assert d.owner.finish(d.state.session)
