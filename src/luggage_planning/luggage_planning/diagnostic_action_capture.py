"""Opt-in action/service boundary adapters for a *new* diagnostic run.

Two action hops expose what the executor submits and what MoveGroup actually
sends to FJT, even when those trajectories differ. Each hop preserves its UUID
at downstream submission. This adds transport context and must be disclosed in
any later diagnosis. It is not a passive historical-goal reconstruction.
"""
import json
import threading
import uuid

from action_msgs.msg import GoalStatus, GoalStatusArray
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from moveit_msgs.action import ExecuteTrajectory
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from luggage_planning.async_boundary_capture import AsyncBoundaryCapture
from luggage_planning.ros_boundary_capture import _goal_id

from luggage_planning.diagnostic_capture_routes import PREFIX, EXECUTE, FJT


class BoundaryActionServer(ActionServer):
    """Observe Humble's taken SendGoal/Cancel/GetResult before dispatch.

    This Waitable adapter is pinned by ROS integration tests; changing rclpy's
    taken-data schema requires revalidation. Unlike goal_callback it sees UUIDs
    even for rejected requests.
    """
    def __init__(self, *args, capture, boundary, **kwargs):
        self.capture, self.boundary = capture, boundary
        super().__init__(*args, **kwargs)

    async def execute(self, taken_data):
        for name in ('goal', 'cancel', 'result'):
            if name in taken_data:
                self.capture(self.boundary + ':receive_' + name, taken_data[name][1])
        await super().execute(taken_data)


class CapturedAction:
    """Asynchronous forwarding; all data comes from real requests/results.

    No synchronous future waits occur on executor callbacks. Capture failure
    requests ordinary action cancellation; it never freezes or kills a node.
    Rejection cannot be mirrored after upstream acceptance, so it is recorded
    explicitly and returned as an aborted goal (diagnostic transport difference).
    """
    def __init__(self, node, action_type, target, recorder, parents=lambda: []):
        self.node, self.kind, self.target = node, action_type, target
        self.recorder, self.parents = recorder, parents
        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.active = {}
        self.client = ActionClient(node, action_type, target, callback_group=self.group)
        self.server = BoundaryActionServer(node, action_type, PREFIX + target,
                                   capture=recorder.record, boundary=target,
                                   execute_callback=self.execute,
                                   goal_callback=self.admit,
                                   cancel_callback=self.cancel,
                                   callback_group=self.group)
        self.status = node.create_subscription(
            GoalStatusArray, target + '/_action/status',
            lambda msg: recorder.record(target + ':status', msg), 100,
            callback_group=self.group)

    def identities(self):
        with self.lock:
            return list(self.active)

    def admit(self, request):
        if not self.recorder.healthy or not self.client.server_is_ready():
            # goal_callback lacks UUID; retain rejected contents without inventing one.
            self.recorder.record(self.target + ':admission_rejected', request)
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel(self, upstream):
        self.recorder.record(self.target + ':cancel_requested', upstream.request,
                             goal_id=upstream.goal_id)
        with self.lock:
            handle = self.active.get(_goal_id(upstream.goal_id))
        if handle is not None:
            handle.cancel_goal_async()
        return CancelResponse.ACCEPT

    def cancel_on_capture_fault(self):
        if self.recorder.healthy:
            return
        with self.lock:
            handles = list(self.active.values())
        for handle in handles:
            if handle is not None:
                handle.cancel_goal_async()

    async def execute(self, upstream):
        identity = _goal_id(upstream.goal_id)
        parent_ids = self.parents()
        correlation = {'executor_goal_ids': parent_ids,
                       'parent_unambiguous': len(parent_ids) == 1}
        with self.lock:
            self.active[identity] = None
        request = self.kind.Impl.SendGoalService.Request(
            goal_id=upstream.goal_id, goal=upstream.request)
        result = self.kind.Result()
        try:
            if not self.recorder.record(self.target + ':received', request,
                                        correlation=correlation):
                raise RuntimeError('CAPTURE_NOT_HEALTHY_BEFORE_SEND')
            # This is the actual downstream send boundary, not a certified copy.
            if not self.recorder.record(self.target + ':send', request,
                                        correlation=correlation):
                raise RuntimeError('CAPTURE_NOT_HEALTHY_BEFORE_SEND')

            def feedback(wrapper):
                self.recorder.record(self.target + ':feedback', wrapper,
                                     correlation=correlation)
                upstream.publish_feedback(wrapper.feedback)

            handle = await self.client.send_goal_async(
                upstream.request, goal_uuid=upstream.goal_id,
                feedback_callback=feedback)
            with self.lock:
                self.active[identity] = handle
            response = self.kind.Impl.SendGoalService.Response(accepted=handle.accepted, stamp=handle.stamp)
            self.recorder.record(self.target + ':send_response', response,
                                 goal_id=upstream.goal_id, correlation=correlation)
            if not handle.accepted:
                raise RuntimeError('DOWNSTREAM_GOAL_REJECTED')
            if upstream.is_cancel_requested or not self.recorder.healthy:
                await handle.cancel_goal_async()
            wrapped = await handle.get_result_async()
            self.recorder.record(self.target + ':result', wrapped,
                                 goal_id=upstream.goal_id, correlation=correlation)
            result = wrapped.result
            if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
                upstream.succeed()
            elif (wrapped.status == GoalStatus.STATUS_CANCELED
                  and upstream.is_cancel_requested):
                upstream.canceled()
            else:
                upstream.abort()
            return result
        except Exception as error:
            self.node.get_logger().error('DIAGNOSTIC_ACTION_FAILED: ' + str(error))
            if self.kind is FollowJointTrajectory:
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = str(error)
            else:
                result.error_code.val = -4  # MoveIt CONTROL_FAILED
            upstream.abort()
            return result
        finally:
            with self.lock:
                self.active.pop(identity, None)


class CaptureNode(Node):
    def __init__(self, directory, run_id):
        super().__init__('diagnostic_boundary_capture')
        self.recorder = AsyncBoundaryCapture(
            directory, run_id=run_id,
            on_error=lambda name, detail: self.get_logger().error(name + ': ' + detail))
        self.execute_capture = CapturedAction(self, ExecuteTrajectory, EXECUTE, self.recorder)
        self.fjt_capture = CapturedAction(self, FollowJointTrajectory, FJT, self.recorder,
                                          self.execute_capture.identities)
        self.group = ReentrantCallbackGroup()
        self.subscriptions_owned = []
        for topic, kind in [('/joint_states', JointState),
                            ('/elfin_arm_controller/controller_state', JointTrajectoryControllerState),
                            ('/clock', Clock)]:
            self.subscriptions_owned.append(self.create_subscription(
                kind, topic, lambda msg, topic=topic: self.recorder.record(topic, msg),
                QoSProfile(depth=4096, reliability=ReliabilityPolicy.BEST_EFFORT),
                callback_group=self.group))
        self.publisher = self.create_publisher(String, PREFIX + '/health', 10)
        self.driver_fault_subscription = self.create_subscription(
            String, PREFIX + '/fault', self.driver_fault, 10, callback_group=self.group)
        # Wall timer intentionally independent of a stopped simulation clock.
        self.timer = self.create_timer(.1, self.publish_health, callback_group=self.group)
        self.service_bridges = []

    def bridge_service(self, service_type, target):
        if target != '/validate_route_v2':
            raise ValueError('only ValidateRouteV2 diagnostic bridging is supported')
        client = self.create_client(service_type, target, callback_group=self.group)

        async def call(request, response):
            correlation = {'service_call_id': uuid.uuid4().hex}
            if not self.recorder.record(target + ':request', request,
                                        correlation=correlation):
                response.result = 1  # ValidateRouteV2 INVALID_REQUEST
                response.detail = 'DIAGNOSTIC_CAPTURE_UNHEALTHY'
                return response
            if not client.service_is_ready():
                response.result = 1
                response.detail = 'DIAGNOSTIC_CERT_SERVICE_UNAVAILABLE'
                self.recorder.record(target + ':response', response,
                                     correlation=correlation)
                return response
            actual = await client.call_async(request)
            self.recorder.record(target + ':response', actual, correlation=correlation)
            return actual

        server = self.create_service(service_type, PREFIX + target, call,
                                     callback_group=self.group)
        self.service_bridges.append((client, server))

    def driver_fault(self, message):
        try:
            data = json.loads(message.data)
            if data.get('run_id') != self.recorder.run_id:
                return
            self.recorder.record('driver:capture_fault', message)
            self.recorder.fault('CAPTURE_DRIVER_FAULT', data)
        except (ValueError, TypeError, AttributeError) as error:
            self.recorder.fault('CAPTURE_FAULT_MESSAGE_INVALID', error)

    def publish_health(self):
        self.execute_capture.cancel_on_capture_fault()
        self.fjt_capture.cancel_on_capture_fault()
        status = self.recorder.health()
        status['targets_ready'] = (self.execute_capture.client.server_is_ready()
            and self.fjt_capture.client.server_is_ready()
            and all(client.service_is_ready() for client, _ in self.service_bridges))
        self.publisher.publish(String(data=json.dumps(status)))

    def close_capture(self):
        return self.recorder.close()
