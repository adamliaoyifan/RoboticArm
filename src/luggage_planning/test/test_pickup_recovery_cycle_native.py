"""Actual Action/two-service workflow; synthetic acquisition and motion/planning.

This verifies default helper transport composition, not native kinematics,
continuous collision certification or deployment-model recovery.
"""
from dataclasses import replace
import json
import threading
import time
from types import SimpleNamespace

from rclpy.serialization import deserialize_message, serialize_message
from luggage_msgs.action import ExecutePickupViewV2
from luggage_msgs.srv import PrepareJointOperationV2, ReprepareAfterPickupViewV2
from luggage_perception.ros_suction_domain import domain_to_message
from luggage_planning.joint_operation.pickup_recovery import (
    PickupPreparationAttempt, PickupPreparationRecovery, PickupRecoveryPorts,
)
from luggage_planning.joint_operation.view_budget import PickupViewBudgetLedger
from luggage_planning.joint_operation.view_continuation import PickupViewRuntimeContinuation, retain_terminal_view
from pickup_continuation_fixtures import case_archive
from test_pickup_recovery_cycle import settings
from test_pickup_view_reprepare_owner import reprepare_case
from test_pickup_view_runtime_continuation import runtime_case


def complete(future):
    done = threading.Event(); future.add_done_callback(lambda _: done.set())
    assert done.wait(5.), 'native workflow future did not terminate'
    return future.result()


def test_native_pickup_workflow_action_acquisition_and_runtime_chain(runtime_case):
    import rclpy
    from rclpy.action import ActionClient, ActionServer
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    f = runtime_case
    f.ledger = PickupViewBudgetLedger(f.archive.directory/'actual_workflow_budget.json', clock=lambda: f.wall_clock[0])
    f.archive.ledger = (f.ledger.path, f.wall_clock)
    task = 'native-cycle-task'
    f.ledger.open(task, f.domain.digest(), max_motions=1, total_budget_s=10.)
    initial = PickupPreparationAttempt(f.original, PrepareJointOperationV2.Response(
        reason='SEARCH_INCOMPLETE', unresolved_contacts=[f.probe_message]), f.original.detection,
        domain_to_message(f.domain))
    f.node._record('cycle_initial_request', initial.request)
    f.node._record('cycle_initial_response', initial.response)
    f.archive.save('native_workflow_scope', dict(actuator='synthetic terminal Action',
        acquisition='calibrated raw plane fixture supplied after settle callback; not physical sensing',
        preparation='real matching/reprepare owners with scripted complete-plan boundary'))
    rclpy.init()
    node = Node('pickup_preparation_cycle_native'); node._group = ReentrantCallbackGroup()
    node.declare_parameter('max_preparation_budget_s', 30.)
    f.parent.node = node
    node.create_service(ReprepareAfterPickupViewV2, '/joint_operation/reprepare_after_pickup_view',
                        f.owner.handle, callback_group=node._group)
    f.continuation = PickupViewRuntimeContinuation(SimpleNamespace(parent=f.parent, ledger=f.ledger))
    continued = node.create_client(ReprepareAfterPickupViewV2, '/joint_operation/continue_after_pickup_view',
                                   callback_group=node._group)
    events = []
    def execute(handle):
        f.node._record('cycle_actual_action_goal', handle.request)
        assert handle.request.pickup_task_id == task
        f.ledger.dispatch(task)
        result = deserialize_message(serialize_message(f.terminal), ExecutePickupViewV2.Result)
        search = handle.request.preparation
        result.plan.request_id, result.plan.task_request_id = search.request_id, search.task_request_id
        result.plan.domain_sha256, result.plan.query_id = search.domain_sha256, search.query_id
        result.dispatched_motions = f.ledger.status(task)[0]
        f.issued = result.plan
        f.views.contexts.issue(f.context, replace(f.receipt, message_cdr=bytes(serialize_message(result.plan))))
        retain_terminal_view(f.ledger, task, result)
        f.node._record('cycle_actual_action_result', result)
        events.append('action_terminal')
        handle.succeed()
        return result
    server = ActionServer(node, ExecutePickupViewV2, '/joint_operation/execute_pickup_view',
                          execute_callback=execute, callback_group=node._group)
    action = ActionClient(node, ExecutePickupViewV2, '/joint_operation/execute_pickup_view', callback_group=node._group)
    executor = MultiThreadedExecutor(num_threads=4); executor.add_node(node)
    worker = threading.Thread(target=executor.spin); worker.start()
    def view(goal, deadline):
        handle = complete(action.send_goal_async(goal)); assert handle.accepted
        wrapped = complete(handle.get_result_async())
        f.node._record('cycle_actual_action_wrapped', wrapped)
        return wrapped
    def observe(stamp, deadline):
        assert events == ['action_terminal'] and stamp == f.terminal.settled_stamp_ns
        events.append('new_fixture_acquisition')
        f.node._record('cycle_new_detection', f.request.detection)
        f.node._record('cycle_new_domain', domain_to_message(f.new_domain))
        return f.request.detection, domain_to_message(f.new_domain)
    def reprepare(request, deadline):
        assert events == ['action_terminal', 'new_fixture_acquisition']
        f.node._record('cycle_actual_continue_request', request)
        result = complete(continued.call_async(request))
        f.node._record('cycle_actual_continue_response', result)
        events.append('runtime_continuation')
        return result
    def fallback(*args): raise AssertionError('successful recovery needs no fallback')
    try:
        assert action.wait_for_server(timeout_sec=3.)
        assert continued.wait_for_service(timeout_sec=3.)
        assert f.continuation.client.wait_for_service(timeout_sec=3.)
        workflow = PickupPreparationRecovery(settings(1), PickupRecoveryPorts(view, observe, reprepare, fallback),
            record=f.node._record)
        plan, detection, domain = workflow.run(initial, pickup_task_id=task)
        f.node._record('cycle_returned_plan', plan)
        assert plan.plan_digest and domain.domain_sha256 == f.new_domain.digest()
        assert detection == f.request.detection
        assert events == ['action_terminal', 'new_fixture_acquisition', 'runtime_continuation']
        assert f.ledger.rows[task]['dispatched'] == 1
        assert f.new_domain.digest() in f.ledger.rows[task]['acquisitions']
    finally:
        executor.shutdown(timeout_sec=3.); worker.join(timeout=3.)
        action.destroy(); server.destroy(); node.destroy_node(); rclpy.shutdown()
        f.archive.save('native_workflow_teardown', dict(worker_stopped=not worker.is_alive(), events=events,
            active_lock=f.parent._active.locked()))
    assert not worker.is_alive()
