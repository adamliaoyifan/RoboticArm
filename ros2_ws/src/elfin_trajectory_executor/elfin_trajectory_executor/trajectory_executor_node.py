"""
trajectory_executor_node.py
~~~~~~~~~~~~~~~~~~~~~~~~~~~
ROS2 Humble trajectory execution node for the Elfin S20.

Implements a FollowJointTrajectory action server — the standard interface
consumed by MoveIt2 and most ROS2 motion planners — backed by either a
pure-software simulation (no hardware) or the HuayanRobot CPSClient SDK.

Topic / action summary
----------------------
Action servers (offered):
    /follow_joint_trajectory   [control_msgs/action/FollowJointTrajectory]

Publishers:
    /joint_states              [sensor_msgs/msg/JointState]       100 Hz
    /trajectory_executor/status [std_msgs/msg/String]             on change

Parameters (all in the 'trajectory_executor' namespace):
    mode               string   'sim' | 'real'      default: 'sim'
    robot_ip           string                        default: '192.168.0.10'
    robot_port         int                           default: 10003
    default_velocity_deg  float                      default: 30.0
    max_velocity_deg      float                      default: 60.0
    joint_names        string[] (must match planner)

Usage
-----
Simulation:
    ros2 launch elfin_trajectory_executor sim.launch.py

Real hardware:
    ros2 launch elfin_trajectory_executor real.launch.py robot_ip:=192.168.0.10
"""

from __future__ import annotations

import math
import threading
from typing import List, Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .sim_interface import SimInterface, RESULT_SUCCESSFUL, RESULT_PREEMPTED, RESULT_INVALID_GOAL
from .huayan_interface import HuayanInterface, RESULT_ERROR


class TrajectoryExecutorNode(Node):
    """
    ROS2 node that exposes a FollowJointTrajectory action server for the
    Elfin S20 robot arm.
    """

    JOINT_NAMES: List[str] = [
        'elfin_joint1', 'elfin_joint2', 'elfin_joint3',
        'elfin_joint4', 'elfin_joint5', 'elfin_joint6',
    ]

    def __init__(self) -> None:
        super().__init__('trajectory_executor')

        # ----------------------------------------------------------------
        # Parameters
        # ----------------------------------------------------------------
        self.declare_parameter('mode', 'sim')
        self.declare_parameter('robot_ip', '192.168.0.10')
        self.declare_parameter('robot_port', 10003)
        self.declare_parameter('default_velocity_deg', 30.0)
        self.declare_parameter('max_velocity_deg', 60.0)
        self.declare_parameter('joint_names', self.JOINT_NAMES)

        mode = self.get_parameter('mode').get_parameter_value().string_value
        robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        robot_port = self.get_parameter('robot_port').get_parameter_value().integer_value
        default_vel = self.get_parameter('default_velocity_deg').get_parameter_value().double_value
        max_vel = self.get_parameter('max_velocity_deg').get_parameter_value().double_value
        self._joint_names: List[str] = (
            self.get_parameter('joint_names').get_parameter_value().string_array_value
            or self.JOINT_NAMES
        )

        self.get_logger().info(
            f'[executor] Starting in {mode.upper()} mode '
            f'(robot_ip={robot_ip}:{robot_port})'
        )

        # ----------------------------------------------------------------
        # Backend
        # ----------------------------------------------------------------
        if mode == 'real':
            self._iface = HuayanInterface(
                node=self,
                robot_ip=robot_ip,
                robot_port=robot_port,
                default_velocity_deg=default_vel,
                max_velocity_deg=max_vel,
            )
            if not self._iface.connect():
                self.get_logger().error(
                    '[executor] Failed to connect to robot. '
                    'Will retry on first goal.'
                )
        else:
            self._iface = SimInterface(node=self)

        # ----------------------------------------------------------------
        # Publishers
        # ----------------------------------------------------------------
        # Reentrant callback group so action + timer can run concurrently.
        self._cb_group = ReentrantCallbackGroup()

        self._js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self._status_pub = self.create_publisher(
            String, '/trajectory_executor/status', 10
        )

        # 100 Hz joint-state publisher timer.
        self._js_timer = self.create_timer(
            0.01, self._publish_joint_states,
            callback_group=self._cb_group,
        )

        # ----------------------------------------------------------------
        # Action server
        # ----------------------------------------------------------------
        # Use a separate callback group so the action execute callback can
        # block without starving the joint-state timer.
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'follow_joint_trajectory',
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            execute_callback=self._execute_callback,
            callback_group=self._cb_group,
        )

        # Cancel flag shared between the action server and the backend.
        self._cancel_flag = threading.Event()

        self._publish_status('idle')
        self.get_logger().info('[executor] Ready.')

    # ------------------------------------------------------------------
    # Action server callbacks
    # ------------------------------------------------------------------

    def _goal_callback(self, goal_request) -> GoalResponse:
        """Accept or reject incoming goals."""
        traj = goal_request.trajectory

        # Validate joint names — reorder if planner uses a different order.
        error = self._validate_joint_names(traj.joint_names)
        if error:
            self.get_logger().warn(f'[executor] Goal rejected: {error}')
            return GoalResponse.REJECT

        # Reject if real backend is in error state.
        if isinstance(self._iface, HuayanInterface) and not self._iface.is_ready:
            self.get_logger().warn('[executor] Goal rejected: robot not ready.')
            return GoalResponse.REJECT

        self.get_logger().info(
            f'[executor] Goal accepted: {len(traj.points)} waypoints.'
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, cancel_request) -> CancelResponse:
        """Accept cancellation and set the cancel flag."""
        self.get_logger().info('[executor] Cancel request received.')
        self._cancel_flag.set()
        return CancelResponse.ACCEPT

    async def _execute_callback(self, goal_handle) -> FollowJointTrajectory.Result:
        """
        Main execution callback.  Runs in a separate thread (MultiThreadedExecutor).

        1. Reorder waypoints to match our joint name order.
        2. Delegate to the backend.
        3. Map return code → action result.
        """
        self._cancel_flag.clear()
        self._publish_status('executing')

        traj = goal_handle.request.trajectory
        ordered_traj = self._reorder_trajectory(traj)

        def feedback_fn(positions: List[float]) -> None:
            """Called by the backend at ~100 Hz with current positions."""
            if goal_handle.is_active:
                fb = FollowJointTrajectory.Feedback()
                fb.joint_names = self._joint_names
                fb.actual.positions = list(positions)
                fb.actual.time_from_start = self._ros_time_offset()
                goal_handle.publish_feedback(fb)

        ret = self._iface.execute(ordered_traj, feedback_fn, self._cancel_flag)

        result = FollowJointTrajectory.Result()

        if ret == RESULT_SUCCESSFUL:
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            result.error_string = ''
            goal_handle.succeed()
            self._publish_status('idle')
            self.get_logger().info('[executor] Goal succeeded.')

        elif ret == RESULT_PREEMPTED:
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            result.error_string = 'Preempted by cancel request.'
            goal_handle.canceled()
            self._publish_status('idle')
            self.get_logger().info('[executor] Goal cancelled.')

        elif ret == RESULT_INVALID_GOAL:
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = 'Trajectory validation failed (check joint limits).'
            goal_handle.abort()
            self._publish_status('error')
            self.get_logger().error('[executor] Goal aborted: invalid goal.')

        else:  # RESULT_ERROR
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = 'Hardware execution error. Check robot logs.'
            goal_handle.abort()
            self._publish_status('error')
            self.get_logger().error('[executor] Goal aborted: hardware error.')

        return result

    # ------------------------------------------------------------------
    # Joint state publisher
    # ------------------------------------------------------------------

    def _publish_joint_states(self) -> None:
        """Publish current joint positions at 100 Hz for RViz."""
        positions = self._iface.current_positions  # always in radians

        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._joint_names
        msg.position = list(positions)
        msg.velocity = [0.0] * 6
        msg.effort = [0.0] * 6
        self._js_pub.publish(msg)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _validate_joint_names(self, incoming: List[str]) -> Optional[str]:
        """
        Check that the incoming joint list is a permutation of our list.
        Returns error string or None.
        """
        if not incoming:
            return 'Trajectory has no joint_names.'
        missing = set(self._joint_names) - set(incoming)
        if missing:
            return f'Missing joints: {missing}'
        extra = set(incoming) - set(self._joint_names)
        if extra:
            return f'Unknown joints: {extra}'
        return None

    def _reorder_trajectory(self, traj: JointTrajectory) -> JointTrajectory:
        """
        Return a new JointTrajectory whose joint order matches self._joint_names.

        If the incoming order already matches, the original object is returned.
        """
        if list(traj.joint_names) == self._joint_names:
            return traj

        # Build an index map: our_idx → incoming_idx
        incoming_idx = {name: i for i, name in enumerate(traj.joint_names)}
        order = [incoming_idx[name] for name in self._joint_names]

        new_traj = JointTrajectory()
        new_traj.header = traj.header
        new_traj.joint_names = self._joint_names

        for pt in traj.points:
            new_pt = JointTrajectoryPoint()
            new_pt.time_from_start = pt.time_from_start
            new_pt.positions = [pt.positions[i] for i in order]
            if pt.velocities:
                new_pt.velocities = [pt.velocities[i] for i in order]
            if pt.accelerations:
                new_pt.accelerations = [pt.accelerations[i] for i in order]
            new_traj.points.append(new_pt)

        return new_traj

    def _publish_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)

    def _ros_time_offset(self) -> Duration:
        """Return current ROS time as a Duration (used for feedback stamp)."""
        t = self.get_clock().now()
        secs = int(t.nanoseconds // 1_000_000_000)
        nsecs = int(t.nanoseconds % 1_000_000_000)
        d = Duration()
        d.sec = secs
        d.nanosec = nsecs
        return d

    def destroy_node(self) -> None:
        """Clean up resources on shutdown."""
        if isinstance(self._iface, HuayanInterface):
            self._iface.disconnect()
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)

    node = TrajectoryExecutorNode()

    # MultiThreadedExecutor allows the action execute callback to block while
    # the joint-state timer and other callbacks keep running.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
