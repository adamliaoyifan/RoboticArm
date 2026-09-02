"""
huayan_interface.py
~~~~~~~~~~~~~~~~~~~
Real-hardware backend for the Elfin S20 trajectory executor.

Wraps the HuayanRobot CPSClient Python SDK (CPS.so / CPS.pyd, V1.0.11.0).
Handles the full connection lifecycle, trajectory execution, and error
recovery so the action server can treat hardware interaction as a simple
blocking call.

Connection lifecycle
--------------------
connect()
    HRIF_Connect
    → HRIF_Connect2Box
    → HRIF_Electrify
    → HRIF_Connect2Controller
    → poll HRIF_IsControllerStarted (up to 30 s)
    → HRIF_GrpReset
    → HRIF_GrpEnable

execute(trajectory, feedback_fn, cancel_flag)
    For each waypoint:
        HRIF_WayPoint (joint move, blending on intermediates)
        poll HRIF_IsBlendingDone / HRIF_IsMotionDone
        call feedback_fn with live joint positions

disconnect()
    HRIF_GrpDisable → HRIF_BlackOut → HRIF_DisConnect

Error / reconnect strategy
--------------------------
* HRIF_ calls that return nRet > 0 are treated as hard errors: execution
  stops, HRIF_GrpStop is called, and RESULT_ERROR is returned.
* If IsConnected returns False mid-execution a reconnect is attempted up
  to MAX_RECONNECT_RETRIES times with exponential backoff.
* Incoming goals are rejected while the connection is not READY.
"""

from __future__ import annotations

import math
import threading
import time
from enum import Enum, auto
from typing import Callable, List, Optional, Tuple

# ---------------------------------------------------------------------------
# Result codes (mirrors control_msgs/action/FollowJointTrajectory constants)
# ---------------------------------------------------------------------------
RESULT_SUCCESSFUL = 0
RESULT_PREEMPTED = -5
RESULT_INVALID_GOAL = -2
RESULT_ERROR = -6

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
BOX_ID = 0
RBT_ID = 0
TCP_NAME = 'TCP'
UCS_NAME = 'Base'

MAX_RECONNECT_RETRIES = 3
CONTROLLER_START_TIMEOUT_S = 30.0
POLL_INTERVAL_S = 0.05          # 20 Hz feedback polling
BLEND_RADIUS_MM = 5.0           # blending on intermediate waypoints
FINAL_BLEND_RADIUS_MM = 0.0     # exact stop at last waypoint
DEFAULT_VELOCITY_DEG = 30.0
DEFAULT_ACCEL_DEG = 60.0        # must be > velocity per HuayanRobot constraint
MAX_VELOCITY_DEG = 60.0


class ConnectionState(Enum):
    DISCONNECTED = auto()
    CONNECTING = auto()
    READY = auto()
    EXECUTING = auto()
    ERROR = auto()


class HuayanInterface:
    """
    Real-hardware backend for the Elfin S20 via HuayanRobot CPSClient SDK.

    Parameters
    ----------
    node : rclpy.node.Node
        Parent node for logging.
    robot_ip : str
        Controller IP address (default '192.168.0.10').
    robot_port : int
        Controller port (default 10003).
    default_velocity_deg : float
        Fallback joint velocity in °/s when trajectory time hints are absent.
    max_velocity_deg : float
        Upper clamp on computed velocity.
    """

    JOINT_NAMES: List[str] = [
        'elfin_joint1', 'elfin_joint2', 'elfin_joint3',
        'elfin_joint4', 'elfin_joint5', 'elfin_joint6',
    ]

    # Elfin S20 joint limits in degrees (matches URDF ±360°)
    JOINT_LIMITS_DEG: List[Tuple[float, float]] = [
        (-360.0, 360.0),
        (-360.0, 360.0),
        (-360.0, 360.0),
        (-360.0, 360.0),
        (-360.0, 360.0),
        (-360.0, 360.0),
    ]

    def __init__(
        self,
        node,
        robot_ip: str = '192.168.0.10',
        robot_port: int = 10003,
        default_velocity_deg: float = DEFAULT_VELOCITY_DEG,
        max_velocity_deg: float = MAX_VELOCITY_DEG,
    ) -> None:
        self._node = node
        self._ip = robot_ip
        self._port = robot_port
        self._default_vel = default_velocity_deg
        self._max_vel = max_velocity_deg

        self._cps = None          # CPSClient instance (imported lazily)
        self._state = ConnectionState.DISCONNECTED
        self._state_lock = threading.Lock()
        self._current_positions_deg: List[float] = [0.0] * 6

    # ------------------------------------------------------------------
    # State helpers
    # ------------------------------------------------------------------

    @property
    def is_ready(self) -> bool:
        with self._state_lock:
            return self._state == ConnectionState.READY

    def _set_state(self, state: ConnectionState) -> None:
        with self._state_lock:
            self._state = state
        self._node.get_logger().info(f'[huayan] State → {state.name}')

    # ------------------------------------------------------------------
    # Connection lifecycle
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        """
        Establish connection and bring robot to enabled/ready state.

        Returns True on success, False otherwise.
        """
        self._set_state(ConnectionState.CONNECTING)
        try:
            self._cps = self._import_cps()
        except ImportError as exc:
            self._node.get_logger().error(
                f'[huayan] Cannot import CPS SDK: {exc}\n'
                'Ensure CPS_python3_Linux.so is on PYTHONPATH.'
            )
            self._set_state(ConnectionState.ERROR)
            return False

        if not self._do_connect():
            self._set_state(ConnectionState.ERROR)
            return False

        self._set_state(ConnectionState.READY)
        return True

    def disconnect(self) -> None:
        """Graceful shutdown: disable servo, power off, disconnect."""
        if self._cps is None:
            return
        try:
            self._node.get_logger().info('[huayan] Disconnecting...')
            self._cps.HRIF_GrpDisable(BOX_ID, RBT_ID)
            time.sleep(0.5)
            self._cps.HRIF_BlackOut(BOX_ID)
            time.sleep(0.3)
            self._cps.HRIF_DisConnect(BOX_ID)
        except Exception as exc:
            self._node.get_logger().warn(f'[huayan] Disconnect error: {exc}')
        finally:
            self._set_state(ConnectionState.DISCONNECTED)

    # ------------------------------------------------------------------
    # Trajectory execution
    # ------------------------------------------------------------------

    @property
    def current_positions(self) -> List[float]:
        """Current joint positions in radians (read from hardware)."""
        return [math.radians(d) for d in self._current_positions_deg]

    def validate_trajectory(self, trajectory) -> Optional[str]:
        """Return error string or None if trajectory is valid."""
        for pt in trajectory.points:
            for idx, pos in enumerate(pt.positions):
                pos_deg = math.degrees(pos)
                lo, hi = self.JOINT_LIMITS_DEG[idx]
                if not (lo <= pos_deg <= hi):
                    return (
                        f"Joint {self.JOINT_NAMES[idx]} at {pos_deg:.1f}° "
                        f"out of limits [{lo:.0f}°, {hi:.0f}°]"
                    )
        return None

    def execute(
        self,
        trajectory,
        feedback_fn: Callable[[List[float]], None],
        cancel_flag: threading.Event,
    ) -> int:
        """
        Execute a JointTrajectory on the real robot.

        Sends each trajectory point as a HRIF_WayPoint (joint-space move)
        with blending on intermediate points and an exact stop at the final
        waypoint.  Polls HRIF_IsBlendingDone (intermediate) and
        HRIF_IsMotionDone (final) for completion.

        Parameters
        ----------
        trajectory : trajectory_msgs.msg.JointTrajectory
        feedback_fn : callable
            Called with current joint positions (radians) during execution.
        cancel_flag : threading.Event
            Set by the action server on preemption.

        Returns
        -------
        int  RESULT_SUCCESSFUL | RESULT_PREEMPTED | RESULT_ERROR | RESULT_INVALID_GOAL
        """
        error = self.validate_trajectory(trajectory)
        if error:
            self._node.get_logger().error(f'[huayan] {error}')
            return RESULT_INVALID_GOAL

        if not self._ensure_connected():
            return RESULT_ERROR

        self._set_state(ConnectionState.EXECUTING)

        points = trajectory.points
        if not points:
            self._set_state(ConnectionState.READY)
            return RESULT_SUCCESSFUL

        # Convert trajectory times to seconds for velocity estimation.
        times_s = [_duration_to_sec(pt.time_from_start) for pt in points]

        try:
            for i, pt in enumerate(points):
                if cancel_flag.is_set():
                    self._node.get_logger().info('[huayan] Trajectory preempted.')
                    self._safe_stop()
                    return RESULT_PREEMPTED

                is_last = (i == len(points) - 1)
                joints_deg = [math.degrees(a) for a in pt.positions]

                # --- Velocity estimation ---
                vel_deg = self._estimate_velocity(
                    i, points, times_s, joints_deg
                )
                accel_deg = max(DEFAULT_ACCEL_DEG, vel_deg * 2.0)
                radius = FINAL_BLEND_RADIUS_MM if is_last else BLEND_RADIUS_MM

                self._node.get_logger().debug(
                    f'[huayan] WP {i}/{len(points)-1}  '
                    f'J={[f"{d:.1f}" for d in joints_deg]}°  '
                    f'vel={vel_deg:.1f}°/s  r={radius}mm'
                )

                nRet = self._cps.HRIF_WayPoint(
                    BOX_ID, RBT_ID,
                    0,                         # nMoveType=0: joint move
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # PCS target (unused)
                    joints_deg[0], joints_deg[1], joints_deg[2],
                    joints_deg[3], joints_deg[4], joints_deg[5],
                    TCP_NAME, UCS_NAME,
                    vel_deg,
                    accel_deg,
                    radius,
                    1,                          # nIsUseJoint=1
                    0, 0, 0,                    # nIsSeek, nIOBit, nIOState
                    str(i),                     # strCmdID
                )

                if nRet != 0:
                    msg = self._get_error_str(nRet)
                    self._node.get_logger().error(
                        f'[huayan] HRIF_WayPoint failed (code {nRet}): {msg}'
                    )
                    self._safe_stop()
                    return RESULT_ERROR

                # Poll for completion and publish feedback.
                result = self._wait_for_waypoint(
                    is_last=is_last,
                    feedback_fn=feedback_fn,
                    cancel_flag=cancel_flag,
                )
                if result != RESULT_SUCCESSFUL:
                    return result

        except Exception as exc:
            self._node.get_logger().error(f'[huayan] Execution exception: {exc}')
            self._safe_stop()
            self._set_state(ConnectionState.ERROR)
            return RESULT_ERROR

        self._node.get_logger().info('[huayan] Trajectory execution complete.')
        self._set_state(ConnectionState.READY)
        return RESULT_SUCCESSFUL

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _do_connect(self) -> bool:
        """Low-level connection sequence. Returns True on success."""
        log = self._node.get_logger()

        log.info(f'[huayan] Connecting to {self._ip}:{self._port}...')
        nRet = self._cps.HRIF_Connect(BOX_ID, self._ip, self._port)
        if nRet != 0:
            log.error(f'[huayan] HRIF_Connect failed: {self._get_error_str(nRet)}')
            return False

        nRet = self._cps.HRIF_Connect2Box(BOX_ID)
        if nRet != 0:
            log.error(f'[huayan] HRIF_Connect2Box failed: {self._get_error_str(nRet)}')
            return False

        nRet = self._cps.HRIF_Electrify(BOX_ID)
        if nRet != 0:
            log.error(f'[huayan] HRIF_Electrify failed: {self._get_error_str(nRet)}')
            return False

        nRet = self._cps.HRIF_Connect2Controller(BOX_ID)
        if nRet != 0:
            log.error(
                f'[huayan] HRIF_Connect2Controller failed: {self._get_error_str(nRet)}'
            )
            return False

        # Poll until controller is fully started.
        log.info('[huayan] Waiting for controller to start...')
        deadline = time.monotonic() + CONTROLLER_START_TIMEOUT_S
        while time.monotonic() < deadline:
            result = []
            nRet = self._cps.HRIF_IsControllerStarted(BOX_ID, result)
            if nRet == 0 and result and result[0] == '1':
                break
            time.sleep(0.5)
        else:
            log.error('[huayan] Controller did not start within timeout.')
            return False

        log.info('[huayan] Controller started. Resetting and enabling...')
        nRet = self._cps.HRIF_GrpReset(BOX_ID, RBT_ID)
        if nRet != 0:
            log.error(f'[huayan] HRIF_GrpReset failed: {self._get_error_str(nRet)}')
            return False

        time.sleep(0.3)

        nRet = self._cps.HRIF_GrpEnable(BOX_ID, RBT_ID)
        if nRet != 0:
            log.error(f'[huayan] HRIF_GrpEnable failed: {self._get_error_str(nRet)}')
            return False

        # Give servo time to stabilise.
        time.sleep(0.5)

        # Read initial joint positions.
        self._refresh_positions()

        log.info('[huayan] Robot ready.')
        return True

    def _ensure_connected(self) -> bool:
        """
        Check connection health; reconnect with backoff if needed.
        Returns True when ready, False after exhausting retries.
        """
        if self._cps is None:
            return False

        if self._cps.HRIF_IsConnected(BOX_ID):
            return True

        self._node.get_logger().warn('[huayan] Connection lost. Attempting reconnect...')
        for attempt in range(1, MAX_RECONNECT_RETRIES + 1):
            wait = 2 ** attempt
            self._node.get_logger().info(
                f'[huayan] Reconnect attempt {attempt}/{MAX_RECONNECT_RETRIES} '
                f'in {wait}s...'
            )
            time.sleep(wait)
            if self._do_connect():
                self._set_state(ConnectionState.READY)
                return True

        self._node.get_logger().error('[huayan] Reconnect failed after all retries.')
        self._set_state(ConnectionState.ERROR)
        return False

    def _wait_for_waypoint(
        self,
        is_last: bool,
        feedback_fn: Callable[[List[float]], None],
        cancel_flag: threading.Event,
    ) -> int:
        """
        Poll for waypoint/motion completion.

        For intermediate points we check HRIF_IsBlendingDone (waypoint queue
        consumed) since the robot keeps moving during blending.
        For the last point we wait for HRIF_IsMotionDone (full stop).
        """
        while True:
            if cancel_flag.is_set():
                self._safe_stop()
                return RESULT_PREEMPTED

            self._refresh_positions()
            feedback_fn(self.current_positions)

            result = []
            if is_last:
                nRet = self._cps.HRIF_IsMotionDone(BOX_ID, RBT_ID, result)
                done = (nRet == 0 and result and result[0] is True)
            else:
                nRet = self._cps.HRIF_IsBlendingDone(BOX_ID, RBT_ID, result)
                done = (nRet == 0 and result and result[0] is True)

            if done:
                return RESULT_SUCCESSFUL

            time.sleep(POLL_INTERVAL_S)

    def _refresh_positions(self) -> None:
        """Read actual joint positions from hardware and cache them."""
        result = []
        nRet = self._cps.HRIF_ReadActJointPos(BOX_ID, RBT_ID, result)
        if nRet == 0 and len(result) >= 6:
            self._current_positions_deg = [float(r) for r in result[:6]]

    def _safe_stop(self) -> None:
        """Best-effort emergency stop."""
        try:
            if self._cps is not None:
                self._cps.HRIF_GrpStop(BOX_ID, RBT_ID)
                time.sleep(0.1)
                self._cps.HRIF_GrpReset(BOX_ID, RBT_ID)
        except Exception as exc:
            self._node.get_logger().warn(f'[huayan] Stop error: {exc}')
        finally:
            with self._state_lock:
                if self._state == ConnectionState.EXECUTING:
                    self._state = ConnectionState.READY

    def _get_error_str(self, code: int) -> str:
        """Return human-readable description for an HRIF error code."""
        try:
            result = []
            self._cps.HRIF_GetErrorCodeStr(BOX_ID, code, result)
            if result:
                return result[0]
        except Exception:
            pass
        return f'(unknown error {code})'

    def _estimate_velocity(
        self,
        idx: int,
        points,
        times_s: List[float],
        joints_deg: List[float],
    ) -> float:
        """
        Estimate the velocity for waypoint idx.

        Strategy (in priority order):
        1. Use trajectory.points[idx].velocities if provided and non-zero.
        2. Compute from position delta / time delta to next waypoint.
        3. Fall back to default_velocity_deg.

        The result is clamped to [1.0, max_velocity_deg].
        """
        pt = points[idx]

        # 1) Use provided velocities.
        if pt.velocities:
            max_vel = max(abs(math.degrees(v)) for v in pt.velocities)
            if max_vel > 0.0:
                return min(max_vel, self._max_vel)

        # 2) Derive from position delta / time delta.
        if idx + 1 < len(points):
            next_joints_deg = [math.degrees(a) for a in points[idx + 1].positions]
            dt = times_s[idx + 1] - times_s[idx]
            if dt > 1e-6:
                max_delta = max(
                    abs(next_joints_deg[j] - joints_deg[j]) for j in range(6)
                )
                vel = max_delta / dt
                if vel > 0.0:
                    return min(vel, self._max_vel)

        # 3) Default.
        return min(self._default_vel, self._max_vel)

    @staticmethod
    def _import_cps():
        """
        Lazily import the CPSClient class from the Huayan SDK.

        The SDK .so file must be on sys.path / PYTHONPATH.  We rename
        CPS_python3_Linux.so → CPS.so (or CPS.pyd on Windows) as per the
        official documentation so the import works as `from CPS import CPSClient`.
        """
        from CPS import CPSClient  # noqa: PLC0415
        return CPSClient()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _duration_to_sec(duration) -> float:
    """Convert builtin_interfaces/Duration to float seconds."""
    return float(duration.sec) + float(duration.nanosec) * 1e-9
