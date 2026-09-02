"""Blocking FollowJointTrajectory client: wait for the executor result.

Callers must keep spinning the node (background executor thread) while
``execute_positions`` waits. Success is the only ``ready_for_next`` signal.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .execution_contract import DEFAULT_ACTION_NAME

JOINT_NAMES = [
    "elfin_joint1",
    "elfin_joint2",
    "elfin_joint3",
    "elfin_joint4",
    "elfin_joint5",
    "elfin_joint6",
]


@dataclass(frozen=True)
class ExecutionOutcome:
    succeeded: bool
    ready_for_next: bool
    status: int
    error_code: int
    error_string: str
    goal_id: str

    @property
    def message(self) -> str:
        if self.ready_for_next:
            return "EXECUTION_COMPLETE ready_for_next=true"
        return (
            "EXECUTION_FAILED ready_for_next=false "
            "status=%s error_code=%s error_string=%r"
            % (self.status, self.error_code, self.error_string)
        )


class FollowJointTrajectoryClient:
    """Action client that treats the FJT result as the step barrier."""

    def __init__(self, node: Node, action_name: str = DEFAULT_ACTION_NAME):
        self._node = node
        self._action_name = str(action_name)
        self._client = ActionClient(node, FollowJointTrajectory, self._action_name)
        self._joint_state: Optional[JointState] = None
        node.create_subscription(JointState, "/joint_states", self._on_js, 10)

    def _on_js(self, msg: JointState) -> None:
        self._joint_state = msg

    def wait_ready(self, timeout_sec: float = 30.0) -> None:
        if not self._client.wait_for_server(timeout_sec=timeout_sec):
            raise RuntimeError(
                "FollowJointTrajectory server not available at %s" % self._action_name
            )
        deadline = time.time() + timeout_sec
        while self._joint_state is None and time.time() < deadline:
            time.sleep(0.05)
        if self._joint_state is None:
            raise RuntimeError("/joint_states not received from the executor")

    def current_positions(self) -> List[float]:
        if self._joint_state is None:
            raise RuntimeError("/joint_states not received")
        by_name = dict(zip(self._joint_state.name, self._joint_state.position))
        missing = [name for name in JOINT_NAMES if name not in by_name]
        if missing:
            raise RuntimeError("joint_states missing %s" % missing)
        return [float(by_name[name]) for name in JOINT_NAMES]

    def execute_positions(
        self,
        positions: Sequence[float],
        duration_sec: float,
        *,
        result_timeout_sec: Optional[float] = None,
    ) -> ExecutionOutcome:
        start = self.current_positions()
        target = [float(v) for v in positions]
        if len(target) != 6:
            raise ValueError("expected 6 joint positions, got %d" % len(target))

        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = list(JOINT_NAMES)

        p0 = JointTrajectoryPoint()
        p0.positions = start
        p0.velocities = [0.0] * 6
        p0.time_from_start = Duration(sec=0, nanosec=0)

        p1 = JointTrajectoryPoint()
        p1.positions = target
        p1.velocities = [0.0] * 6
        p1.time_from_start = _duration_from_sec(duration_sec)
        traj.points = [p0, p1]
        goal.trajectory = traj
        goal.goal_time_tolerance = Duration(sec=2, nanosec=0)

        send_future = self._client.send_goal_async(goal)
        handle = _wait_future(send_future, 10.0)
        if handle is None:
            return ExecutionOutcome(
                False, False, GoalStatus.STATUS_UNKNOWN, -1,
                "goal send timeout", "",
            )
        if not handle.accepted:
            return ExecutionOutcome(
                False, False, GoalStatus.STATUS_ABORTED, -1,
                "goal rejected", "",
            )

        goal_id = _uuid_hex(handle.goal_id)
        timeout = (
            float(result_timeout_sec)
            if result_timeout_sec is not None
            else float(duration_sec) + 60.0
        )
        wrapped = _wait_future(handle.get_result_async(), timeout)
        if wrapped is None:
            handle.cancel_goal()
            return ExecutionOutcome(
                False, False, GoalStatus.STATUS_UNKNOWN, -1,
                "result timeout", goal_id,
            )

        result = wrapped.result
        error_code = int(getattr(result, "error_code", -1))
        error_string = str(getattr(result, "error_string", "") or "")
        status = int(wrapped.status)
        succeeded = (
            status == GoalStatus.STATUS_SUCCEEDED
            and error_code == FollowJointTrajectory.Result.SUCCESSFUL
        )
        return ExecutionOutcome(
            succeeded=succeeded,
            ready_for_next=succeeded,
            status=status,
            error_code=error_code,
            error_string=error_string,
            goal_id=goal_id,
        )


def _duration_from_sec(duration_sec: float) -> Duration:
    duration_sec = max(0.1, float(duration_sec))
    sec = int(duration_sec)
    nsec = int(round((duration_sec - sec) * 1e9))
    if nsec >= 1000000000:
        sec += 1
        nsec -= 1000000000
    return Duration(sec=sec, nanosec=nsec)


def _uuid_hex(goal_id) -> str:
    try:
        return bytes(goal_id.uuid).hex()
    except Exception:
        return ""


def _wait_future(future, timeout_sec: float):
    """Works on Humble (no Future.result timeout) and Jazzy."""
    event = threading.Event()
    future.add_done_callback(lambda _f: event.set())
    if not event.wait(timeout=timeout_sec):
        return None
    return future.result()
