#!/usr/bin/env python3
"""Send a conservative FollowJointTrajectory goal, then return to start."""
import argparse
import math
import sys
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = [
    "elfin_joint1",
    "elfin_joint2",
    "elfin_joint3",
    "elfin_joint4",
    "elfin_joint5",
    "elfin_joint6",
]
START = [0.0, -0.5, 0.0, 0.0, 0.0, 0.0]
GOAL = [0.3, -1.0, 0.5, -0.8, 0.4, 0.3]


def max_abs_err(actual, target):
    by_name = dict(zip(actual.name, actual.position))
    return max(abs(by_name[n] - t) for n, t in zip(JOINTS, target))


class TrajectoryClient(Node):
    def __init__(self):
        super().__init__("send_joint_trajectory")
        self._client = ActionClient(
            self, FollowJointTrajectory, "/elfin_arm_controller/follow_joint_trajectory"
        )
        self._joint_state = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

    def _on_js(self, msg):
        self._joint_state = msg

    def wait_ready(self, timeout=30.0):
        if not self._client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError("FollowJointTrajectory server not available")
        deadline = time.time() + timeout
        while self._joint_state is None and time.time() < deadline:
            time.sleep(0.05)
        if self._joint_state is None:
            raise RuntimeError("/joint_states not received")

    def wait_near(self, positions, max_error, timeout=2.0):
        deadline = time.time() + timeout
        last = float("inf")
        while time.time() < deadline:
            if self._joint_state is not None:
                last = max_abs_err(self._joint_state, positions)
                if last <= max_error:
                    return last
            time.sleep(0.02)
        return last

    def current_positions(self):
        by_name = dict(zip(self._joint_state.name, self._joint_state.position))
        return [float(by_name[name]) for name in JOINTS]

    def send(self, positions, duration_sec):
        start_positions = self.current_positions()
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = list(JOINTS)
        start_point = JointTrajectoryPoint()
        start_point.positions = start_positions
        start_point.velocities = [0.0] * 6
        start_point.time_from_start = Duration(sec=0, nanosec=0)
        goal_point = JointTrajectoryPoint()
        goal_point.positions = [float(v) for v in positions]
        goal_point.velocities = [0.0] * 6
        sec = int(duration_sec)
        nsec = int(round((duration_sec - sec) * 1e9))
        goal_point.time_from_start = Duration(sec=sec, nanosec=nsec)
        traj.points = [start_point, goal_point]
        goal.trajectory = traj
        # Jazzy JTC honours this field (Humble ignored it). Keep it aligned
        # with elfin_controllers_sim.yaml goal_time so large observe→goal
        # motions are not aborted 1s after time_from_start.
        goal.goal_time_tolerance = Duration(sec=15, nanosec=0)

        send_future = self._client.send_goal_async(goal)
        handle = self._wait_future(send_future, 10.0)
        if handle is None or not handle.accepted:
            raise RuntimeError("trajectory goal rejected")
        wrapped = self._wait_future(
            handle.get_result_async(), float(duration_sec) + 15.0
        )
        if wrapped is None:
            raise RuntimeError("trajectory result timed out")
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError("trajectory status=%s error_code=%s" % (wrapped.status, wrapped.result.error_code))
        return wrapped.result

    @staticmethod
    def _wait_future(future, timeout):
        """rclpy Futures on Humble have no result(timeout); poll instead."""
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        if not future.done():
            return None
        return future.result()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--max-error", type=float, default=0.001)
    parser.add_argument("--duration", type=float, default=3.0)
    args = parser.parse_args()

    rclpy.init()
    node = TrajectoryClient()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    failures = 0
    worst = 0.0
    try:
        node.wait_ready()
        for i in range(args.repeat):
            t0 = time.time()
            go_res = node.send(GOAL, args.duration)
            goal_err = node.wait_near(GOAL, args.max_error)
            back_res = node.send(START, args.duration)
            start_err = node.wait_near(START, args.max_error)
            dt = time.time() - t0
            worst = max(worst, goal_err, start_err)
            ok = (
                go_res.error_code == 0
                and back_res.error_code == 0
                and goal_err <= args.max_error
                and start_err <= args.max_error
            )
            if not ok:
                failures += 1
            node.get_logger().info(
                "repeat %d ok=%s error_codes=(%s,%s) goal_err=%.6f start_err=%.6f dt=%.3fs"
                % (i, ok, go_res.error_code, back_res.error_code, goal_err, start_err, dt)
            )
            if not math.isfinite(worst):
                failures += 1
    except Exception as exc:
        node.get_logger().error(str(exc))
        failures += 1
    finally:
        node.get_logger().info(
            "done repeats=%d failures=%d worst_abs_err=%.6f" % (args.repeat, failures, worst)
        )
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
