#!/usr/bin/env python3
from __future__ import annotations

import math
import threading
import time


BOX_ID = 0
RBT_ID = 0
TCP_NAME = "TCP"
UCS_NAME = "Base"
DEFAULT_ACCEL_DEG = 60.0
STATE_REFUSE = 20018
FSM_ENABLING = 23
FSM_STANDBY = 33
FSM_DISABLE = 24
FSM_BLACKOUT = 7
FSM_ELECTRIFYING = 8
FSM_ELECTRIC_BOX_DISCONNECT = 2
FSM_ELECTRIC_BOX_CONNECTING = 3


class HuayanBackend:
    """Small ROS-free wrapper around the Huayan CPS.py SDK."""

    def __init__(
        self,
        logger,
        robot_ip="192.168.0.10",
        robot_port=10003,
        default_velocity_deg=30.0,
        max_velocity_deg=60.0,
        poll_interval_s=0.05,
        blend_radius_mm=5.0,
        final_blend_radius_mm=0.0,
        controller_start_timeout_s=30.0,
        power_off_on_disconnect=False,
    ):
        self._log = logger
        self._ip = robot_ip
        self._port = int(robot_port)
        self._default_vel = float(default_velocity_deg)
        self._max_vel = float(max_velocity_deg)
        self._poll_interval = float(poll_interval_s)
        self._blend_radius = float(blend_radius_mm)
        self._final_blend_radius = float(final_blend_radius_mm)
        self._controller_start_timeout = float(controller_start_timeout_s)
        self._power_off_on_disconnect = bool(power_off_on_disconnect)
        self._cps = None
        self._ready = False
        self._lock = threading.Lock()
        self._current_positions_deg = [0.0] * 6

    @property
    def is_ready(self):
        with self._lock:
            return self._ready

    @property
    def current_positions(self):
        with self._lock:
            return [math.radians(v) for v in self._current_positions_deg]

    def connect(self):
        try:
            from CPS import CPSClient  # pylint: disable=import-outside-toplevel
        except ImportError as exc:
            self._log("error", "Cannot import Huayan CPS SDK: %s" % exc)
            self._set_ready(False)
            return False

        self._cps = CPSClient()
        if not self._do_connect():
            self._set_ready(False)
            return False

        self._set_ready(True)
        return True

    def disconnect(self):
        if self._cps is None:
            return
        try:
            if self._is_enabled():
                self._cps.HRIF_GrpDisable(BOX_ID, RBT_ID)
                time.sleep(0.2)
            if self._power_off_on_disconnect:
                self._cps.HRIF_BlackOut(BOX_ID)
                time.sleep(0.2)
            self._cps.HRIF_DisConnect(BOX_ID)
        except Exception as exc:  # pragma: no cover - hardware path
            self._log("warn", "Disconnect error: %s" % exc)
        finally:
            self._set_ready(False)

    def refresh_positions(self):
        if self._cps is None:
            return False
        result = []
        n_ret = self._cps.HRIF_ReadActJointPos(BOX_ID, RBT_ID, result)
        if n_ret == 0 and len(result) >= 6:
            with self._lock:
                self._current_positions_deg = [float(v) for v in result[:6]]
            return True
        return False

    def execute_trajectory(self, trajectory, should_cancel, feedback_cb=None):
        if not self.is_ready and not self.connect():
            return False, "Robot is not ready"

        if not trajectory.points:
            return True, ""

        times_s = [_duration_to_sec(pt.time_from_start) for pt in trajectory.points]

        for idx, point in enumerate(trajectory.points):
            if should_cancel():
                self.stop()
                return False, "Trajectory preempted"

            is_last = idx == len(trajectory.points) - 1
            joints_deg = [math.degrees(v) for v in point.positions]
            velocity = self._estimate_velocity(idx, trajectory.points, times_s, joints_deg)
            accel = max(DEFAULT_ACCEL_DEG, velocity * 2.0)
            radius = self._final_blend_radius if is_last else self._blend_radius

            n_ret = self._cps.HRIF_WayPoint(
                BOX_ID,
                RBT_ID,
                0,
                [0.0] * 6,
                joints_deg,
                TCP_NAME,
                UCS_NAME,
                velocity,
                accel,
                radius,
                1,
                0,
                0,
                0,
                str(idx),
            )
            if n_ret != 0:
                self.stop()
                return False, "HRIF_WayPoint failed: %s" % self._error_string(n_ret)

            ok, message = self._wait_for_waypoint(is_last, should_cancel, feedback_cb)
            if not ok:
                return ok, message

        return True, ""

    def stop(self):
        try:
            if self._cps is not None:
                self._cps.HRIF_GrpStop(BOX_ID, RBT_ID)
                time.sleep(0.1)
                self._cps.HRIF_GrpReset(BOX_ID, RBT_ID)
        except Exception as exc:  # pragma: no cover - hardware path
            self._log("warn", "Stop error: %s" % exc)

    def _do_connect(self):
        self._log("info", "Connecting to Huayan controller %s:%s" % (self._ip, self._port))
        n_ret = self._cps.HRIF_Connect(BOX_ID, self._ip, self._port)
        if n_ret != 0:
            self._log("error", "HRIF_Connect failed: %s" % self._error_string(n_ret))
            return False

        fsm_id, fsm_desc = self._read_fsm()
        if fsm_id is not None:
            self._log("info", "Current robot FSM: %s (%s)" % (fsm_id, fsm_desc))

        if fsm_id in (5, 22):
            self._log(
                "error",
                "Robot is in fault/e-stop state (%s). Clear faults on the teach pendant first."
                % fsm_id,
            )
            return False

        connect2box_allow_refuse = fsm_id is None or fsm_id not in (
            FSM_ELECTRIC_BOX_DISCONNECT,
            FSM_ELECTRIC_BOX_CONNECTING,
        )
        electrify_allow_refuse = fsm_id is not None and fsm_id >= FSM_ENABLING

        if not self._run_step(
            "HRIF_Connect2Box",
            lambda: self._cps.HRIF_Connect2Box(BOX_ID),
            allow_state_refuse=connect2box_allow_refuse,
        ):
            return False

        if not self._run_step(
            "HRIF_Electrify",
            lambda: self._cps.HRIF_Electrify(BOX_ID),
            allow_state_refuse=electrify_allow_refuse,
        ):
            return False

        if fsm_id in (FSM_BLACKOUT, FSM_ELECTRIFYING, 6):
            self._log("info", "Waiting for robot to finish powering up...")
            if not self._wait_for_controller_boot(self._controller_start_timeout):
                self._log("error", "Robot did not finish powering up within timeout")
                return False
            fsm_id, fsm_desc = self._read_fsm()
            if fsm_id is not None:
                self._log("info", "Robot FSM after power-up: %s (%s)" % (fsm_id, fsm_desc))

        connect_controller_allow_refuse = (
            self._controller_started()
            or (fsm_id is not None and fsm_id >= FSM_ENABLING)
        )
        if not self._run_step(
            "HRIF_Connect2Controller",
            lambda: self._cps.HRIF_Connect2Controller(BOX_ID),
            allow_state_refuse=connect_controller_allow_refuse,
        ):
            return False

        if not self._controller_started():
            if not self._wait_for_controller_boot(self._controller_start_timeout):
                self._log("error", "Controller did not start within timeout")
                return False

        if self._is_enabled():
            self._log("info", "Robot servos already enabled")
        else:
            if fsm_id == FSM_DISABLE:
                self._run_step(
                    "HRIF_GrpReset",
                    lambda: self._cps.HRIF_GrpReset(BOX_ID, RBT_ID),
                    allow_state_refuse=True,
                )
                time.sleep(0.3)
            if not self._run_step(
                "HRIF_GrpEnable",
                lambda: self._cps.HRIF_GrpEnable(BOX_ID, RBT_ID),
                allow_state_refuse=False,
            ):
                return False
            time.sleep(0.3)

        self.refresh_positions()
        self._log("info", "Huayan controller ready")
        return True

    def _wait_for_controller_boot(self, timeout_s):
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if self._controller_started():
                return True
            fsm_id, _ = self._read_fsm()
            if fsm_id in (5, 22):
                return False
            time.sleep(0.5)
        return self._controller_started()

    def _read_fsm(self):
        result = []
        n_ret = self._cps.HRIF_ReadCurFSM(BOX_ID, RBT_ID, result)
        if n_ret != 0 or not result:
            return None, None
        try:
            fsm_id = int(result[0])
        except (TypeError, ValueError):
            return None, result[0]
        desc = result[1] if len(result) > 1 else str(result[0])
        return fsm_id, desc

    def _controller_started(self):
        result = []
        n_ret = self._cps.HRIF_IsControllerStarted(BOX_ID, result)
        return n_ret == 0 and result and str(result[0]) == "1"

    def _is_enabled(self):
        result = []
        n_ret = self._cps.HRIF_ReadRobotState(BOX_ID, RBT_ID, result)
        return n_ret == 0 and len(result) > 1 and str(result[1]) == "1"

    def _run_step(self, name, fn, allow_state_refuse=False):
        n_ret = fn()
        if n_ret == 0:
            return True
        if allow_state_refuse and n_ret == STATE_REFUSE:
            self._log(
                "info",
                "%s skipped: %s" % (name, self._error_string(n_ret)),
            )
            return True
        self._log("error", "%s failed: %s" % (name, self._error_string(n_ret)))
        return False

    def _wait_for_waypoint(self, is_last, should_cancel, feedback_cb):
        while True:
            if should_cancel():
                self.stop()
                return False, "Trajectory preempted"

            self.refresh_positions()
            if feedback_cb:
                feedback_cb(self.current_positions)

            result = []
            if is_last:
                n_ret = self._cps.HRIF_IsMotionDone(BOX_ID, RBT_ID, result)
            else:
                n_ret = self._cps.HRIF_IsBlendingDone(BOX_ID, RBT_ID, result)

            if n_ret != 0:
                return False, "Motion state query failed: %s" % self._error_string(n_ret)
            if result and bool(result[0]):
                return True, ""
            time.sleep(self._poll_interval)

    def _estimate_velocity(self, idx, points, times_s, joints_deg):
        point = points[idx]
        if point.velocities:
            max_vel = max(abs(math.degrees(v)) for v in point.velocities)
            if max_vel > 0.0:
                return min(max_vel, self._max_vel)

        if idx + 1 < len(points):
            next_joints_deg = [math.degrees(v) for v in points[idx + 1].positions]
            dt = times_s[idx + 1] - times_s[idx]
            if dt > 1e-6:
                max_delta = max(abs(next_joints_deg[j] - joints_deg[j]) for j in range(6))
                if max_delta > 0.0:
                    return min(max_delta / dt, self._max_vel)

        return min(self._default_vel, self._max_vel)

    def _error_string(self, code):
        try:
            result = []
            self._cps.HRIF_GetErrorCodeStr(BOX_ID, int(code), result)
            if result:
                return "%s (%s)" % (result[0], code)
        except Exception:
            pass
        return "error code %s" % code

    def _set_ready(self, ready):
        with self._lock:
            self._ready = ready


def _duration_to_sec(duration):
    return float(duration.secs) + float(duration.nsecs) * 1e-9
