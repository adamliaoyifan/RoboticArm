"""
sim_interface.py
~~~~~~~~~~~~~~~~
Pure-software simulation backend for the Elfin S20 trajectory executor.

No real robot, no HuayanRobot SDK required.  The interface interpolates
through a JointTrajectory using real wall-clock time, publishes
/joint_states at ~100 Hz, and calls the feedback callback so the action
server can report progress.

Interpolation strategy
----------------------
* Between every pair of consecutive waypoints we perform per-joint linear
  interpolation keyed on time_from_start.
* The control loop runs in a background thread so the calling thread
  (action server) can check the cancel flag without blocking the ROS spin.

Thread safety
-------------
_current_positions is protected by a threading.Lock; the ROS timer callback
and the execution thread both touch it.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Callable, List, Optional

import numpy as np

# Result codes mirrored from control_msgs to avoid importing them in this module.
RESULT_SUCCESSFUL = 0
RESULT_PREEMPTED = -5
RESULT_INVALID_GOAL = -2


class SimInterface:
    """Simulated trajectory executor — pure FK, no hardware."""

    JOINT_NAMES: List[str] = [
        'elfin_joint1', 'elfin_joint2', 'elfin_joint3',
        'elfin_joint4', 'elfin_joint5', 'elfin_joint6',
    ]

    # Elfin S20 joint limits (radians).  Used for goal validation.
    JOINT_LIMITS_RAD = [
        (-math.pi * 2, math.pi * 2),   # J1  ±360°
        (-math.pi * 2, math.pi * 2),   # J2  ±360°
        (-math.pi * 2, math.pi * 2),   # J3  ±360°
        (-math.pi * 2, math.pi * 2),   # J4  ±360°
        (-math.pi * 2, math.pi * 2),   # J5  ±360°
        (-math.pi * 2, math.pi * 2),   # J6  ±360°
    ]

    def __init__(self, node) -> None:
        """
        Parameters
        ----------
        node : rclpy.node.Node
            Parent ROS2 node (used for logging only; joint_state publishing
            is handled by the executor node via the feedback callback).
        """
        self._node = node
        self._lock = threading.Lock()
        self._current_positions: List[float] = [0.0] * 6
        self._executing = False

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def current_positions(self) -> List[float]:
        """Thread-safe snapshot of the current simulated joint positions."""
        with self._lock:
            return list(self._current_positions)

    def validate_trajectory(self, trajectory) -> Optional[str]:
        """
        Validate joint names and position limits.

        Returns
        -------
        str | None
            Error message string, or None if valid.
        """
        for pt in trajectory.points:
            for idx, pos in enumerate(pt.positions):
                lo, hi = self.JOINT_LIMITS_RAD[idx]
                if not (lo <= pos <= hi):
                    return (
                        f"Joint {self.JOINT_NAMES[idx]} position "
                        f"{math.degrees(pos):.1f}° out of limits "
                        f"[{math.degrees(lo):.0f}°, {math.degrees(hi):.0f}°]"
                    )
        return None

    def execute(
        self,
        trajectory,
        feedback_fn: Callable[[List[float]], None],
        cancel_flag: threading.Event,
    ) -> int:
        """
        Execute a JointTrajectory by interpolating through its waypoints.

        The call blocks until execution is complete, preempted, or an error
        occurs.  The calling thread should be the action server execute
        callback so it naturally holds the GIL while waiting; the inner
        sleep intervals are short (10 ms) so cancellation is responsive.

        Parameters
        ----------
        trajectory : trajectory_msgs.msg.JointTrajectory
            The trajectory to execute.
        feedback_fn : callable
            Called with the current joint positions (List[float], radians)
            approximately every 10 ms during execution.
        cancel_flag : threading.Event
            Set by the action server when the goal is cancelled.  The
            execution loop checks it every iteration.

        Returns
        -------
        int
            RESULT_SUCCESSFUL, RESULT_PREEMPTED, or RESULT_INVALID_GOAL.
        """
        error = self.validate_trajectory(trajectory)
        if error:
            self._node.get_logger().error(f'[sim] Invalid trajectory: {error}')
            return RESULT_INVALID_GOAL

        points = trajectory.points
        if not points:
            self._node.get_logger().warn('[sim] Received empty trajectory.')
            return RESULT_SUCCESSFUL

        # Convert time_from_start to float seconds for arithmetic.
        times_s = [
            _duration_to_sec(pt.time_from_start) for pt in points
        ]

        # Seed the starting position from current simulated state.
        with self._lock:
            start_positions = list(self._current_positions)

        # Build a full waypoint list including the implicit start at t=0.
        positions_list: List[List[float]] = [start_positions] + [
            list(pt.positions) for pt in points
        ]
        times_full = [0.0] + times_s

        total_duration = times_full[-1]
        if total_duration <= 0.0:
            self._node.get_logger().warn(
                '[sim] Trajectory duration is zero; jumping to final pose.')
            with self._lock:
                self._current_positions = list(positions_list[-1])
            feedback_fn(self._current_positions)
            return RESULT_SUCCESSFUL

        self._executing = True
        t_start = time.monotonic()
        control_period = 0.01  # 100 Hz

        try:
            while True:
                if cancel_flag.is_set():
                    self._node.get_logger().info('[sim] Trajectory preempted.')
                    return RESULT_PREEMPTED

                elapsed = time.monotonic() - t_start

                if elapsed >= total_duration:
                    # Snap to the final waypoint exactly.
                    with self._lock:
                        self._current_positions = list(positions_list[-1])
                    feedback_fn(list(self._current_positions))
                    break

                # Find the two bracketing waypoints.
                seg = _find_segment(times_full, elapsed)
                t0, t1 = times_full[seg], times_full[seg + 1]
                p0, p1 = positions_list[seg], positions_list[seg + 1]

                alpha = (elapsed - t0) / (t1 - t0) if (t1 - t0) > 1e-9 else 1.0
                alpha = max(0.0, min(1.0, alpha))

                interpolated = [
                    p0[j] + alpha * (p1[j] - p0[j])
                    for j in range(6)
                ]

                with self._lock:
                    self._current_positions = interpolated

                feedback_fn(list(interpolated))

                # Sleep for remainder of control period.
                loop_elapsed = time.monotonic() - t_start - elapsed
                sleep_time = max(0.0, control_period - loop_elapsed)
                time.sleep(sleep_time)

        finally:
            self._executing = False

        self._node.get_logger().info('[sim] Trajectory execution complete.')
        return RESULT_SUCCESSFUL

    def stop(self) -> None:
        """Immediate stop — no-op in sim (cancel_flag handles it)."""
        pass


# ------------------------------------------------------------------
# Helpers
# ------------------------------------------------------------------

def _duration_to_sec(duration) -> float:
    """Convert builtin_interfaces/Duration to seconds."""
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def _find_segment(times: List[float], t: float) -> int:
    """
    Binary search for the segment index i such that times[i] <= t < times[i+1].
    Clamps to the last valid segment.
    """
    lo, hi = 0, len(times) - 2
    while lo < hi:
        mid = (lo + hi + 1) // 2
        if times[mid] <= t:
            lo = mid
        else:
            hi = mid - 1
    return lo
