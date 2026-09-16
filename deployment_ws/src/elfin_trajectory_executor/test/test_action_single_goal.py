"""ROS action integration for exclusive executor goal admission."""

import threading
import unittest

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from trajectory_msgs.msg import JointTrajectoryPoint

from elfin_trajectory_executor.trajectory_executor_node import (
    TrajectoryExecutorNode,
)


ACTION_NAME = "/test/elfin_arm_controller/follow_joint_trajectory"
JOINT_NAMES = ["elfin_joint%s" % index for index in range(1, 7)]


def _wait(future, timeout=5.0):
    event = threading.Event()
    future.add_done_callback(lambda _future: event.set())
    if not event.wait(timeout):
        raise AssertionError("ROS future timed out")
    return future.result()


def _goal(position, duration_sec):
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = list(JOINT_NAMES)
    point = JointTrajectoryPoint()
    point.positions = [float(position)] + [0.0] * 5
    point.velocities = [0.0] * 6
    nanoseconds = int(round(float(duration_sec) * 1e9))
    point.time_from_start = Duration(
        sec=nanoseconds // 1_000_000_000,
        nanosec=nanoseconds % 1_000_000_000,
    )
    goal.trajectory.points = [point]
    return goal


class ActionSingleGoalTest(unittest.TestCase):
    def test_overlap_is_rejected_and_ownership_recovers(self):
        rclpy.init(args=[
            "--ros-args",
            "-p", "mode:=sim",
            "-p", "action_name:=%s" % ACTION_NAME,
        ])
        executor_node = TrajectoryExecutorNode()
        client_node = rclpy.create_node("single_goal_integration_client")
        client = ActionClient(client_node, FollowJointTrajectory, ACTION_NAME)
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(executor_node)
        executor.add_node(client_node)
        spinner = threading.Thread(target=executor.spin)
        spinner.start()

        try:
            self.assertTrue(client.wait_for_server(timeout_sec=5.0))
            for iteration in range(10):
                target = 0.01 if iteration % 2 == 0 else -0.01
                owner = _wait(client.send_goal_async(_goal(target, 0.2)))
                self.assertTrue(owner.accepted)

                overlapping = _wait(
                    client.send_goal_async(_goal(-target, 0.2)))
                self.assertFalse(overlapping.accepted)

                wrapped = _wait(owner.get_result_async())
                self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
                self.assertEqual(
                    wrapped.result.error_code,
                    FollowJointTrajectory.Result.SUCCESSFUL,
                )

            cancellable = _wait(client.send_goal_async(_goal(0.02, 1.0)))
            self.assertTrue(cancellable.accepted)
            cancel_response = _wait(cancellable.cancel_goal_async())
            self.assertTrue(cancel_response.goals_canceling)
            canceled = _wait(cancellable.get_result_async())
            self.assertEqual(canceled.status, GoalStatus.STATUS_CANCELED)

            original_execute = executor_node._iface.execute

            def raise_backend_error(*_args, **_kwargs):
                raise RuntimeError("injected backend failure")

            executor_node._iface.execute = raise_backend_error
            failing = _wait(client.send_goal_async(_goal(0.01, 0.1)))
            self.assertTrue(failing.accepted)
            failed = _wait(failing.get_result_async())
            self.assertEqual(failed.status, GoalStatus.STATUS_ABORTED)
            executor_node._iface.execute = original_execute

            after_cancel = _wait(client.send_goal_async(_goal(0.0, 0.1)))
            self.assertTrue(after_cancel.accepted)
            completed = _wait(after_cancel.get_result_async())
            self.assertEqual(completed.status, GoalStatus.STATUS_SUCCEEDED)
        finally:
            executor.shutdown()
            spinner.join(timeout=5.0)
            client_node.destroy_node()
            executor_node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
