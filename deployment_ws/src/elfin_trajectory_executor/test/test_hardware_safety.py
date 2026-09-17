"""Fail-closed tests for CPS polling and controller-limit readiness."""

import threading
import unittest

from elfin_trajectory_executor.huayan_interface import (
    ConnectionState,
    HuayanInterface,
    RESULT_ERROR,
    RESULT_INVALID_GOAL,
    RESULT_PREEMPTED,
    RESULT_SUCCESSFUL,
)


class _Logger:
    def __init__(self):
        self.errors = []
        self.warnings = []

    def info(self, _message):
        pass

    def error(self, message):
        self.errors.append(str(message))

    def warn(self, message):
        self.warnings.append(str(message))


class _Node:
    def __init__(self):
        self.logger = _Logger()

    def get_logger(self):
        return self.logger


def _polling_interface(cps):
    iface = HuayanInterface.__new__(HuayanInterface)
    iface._node = _Node()
    iface._cps = cps
    iface._current_positions_deg = [0.0] * 6
    iface._refresh_positions = lambda: None
    iface._get_error_str = lambda code: "decoded-%s" % code
    iface.stop_calls = 0
    iface.states = []

    def stop():
        iface.stop_calls += 1

    iface._safe_stop = stop
    iface._set_state = iface.states.append
    return iface


class HardwareSafetyTest(unittest.TestCase):
    def test_blending_error_stops_without_advancing(self):
        class _Cps:
            def __init__(self):
                self.calls = 0

            def HRIF_IsBlendingDone(self, _box, _robot, _result):
                self.calls += 1
                return 40083

        cps = _Cps()
        iface = _polling_interface(cps)
        result = iface._wait_for_waypoint(
            is_last=False,
            feedback_fn=lambda _positions: None,
            cancel_flag=threading.Event(),
        )

        self.assertEqual(result, RESULT_ERROR)
        self.assertEqual(cps.calls, 1)
        self.assertEqual(iface.stop_calls, 1)
        self.assertEqual(iface.states, [ConnectionState.ERROR])
        self.assertIn("HRIF_IsBlendingDone", iface._node.logger.errors[0])
        self.assertIn("40083", iface._node.logger.errors[0])

    def test_motion_done_malformed_success_stops_and_enters_error(self):
        class _Cps:
            def HRIF_IsMotionDone(self, _box, _robot, _result):
                return 0

        iface = _polling_interface(_Cps())
        result = iface._wait_for_waypoint(
            is_last=True,
            feedback_fn=lambda _positions: None,
            cancel_flag=threading.Event(),
        )

        self.assertEqual(result, RESULT_ERROR)
        self.assertEqual(iface.stop_calls, 1)
        self.assertEqual(iface.states, [ConnectionState.ERROR])
        self.assertIn(
            "malformed success payload", iface._node.logger.errors[0])

    def test_motion_done_nonzero_return_stops_and_enters_error(self):
        class _Cps:
            def HRIF_IsMotionDone(self, _box, _robot, _result):
                return 20018

        iface = _polling_interface(_Cps())
        result = iface._wait_for_waypoint(
            is_last=True,
            feedback_fn=lambda _positions: None,
            cancel_flag=threading.Event(),
        )

        self.assertEqual(result, RESULT_ERROR)
        self.assertEqual(iface.stop_calls, 1)
        self.assertEqual(iface.states, [ConnectionState.ERROR])
        self.assertIn("20018", iface._node.logger.errors[0])

    def test_valid_motion_done_response_succeeds_without_stop(self):
        class _Cps:
            def HRIF_IsMotionDone(self, _box, _robot, result):
                result.append(True)
                return 0

        iface = _polling_interface(_Cps())
        result = iface._wait_for_waypoint(
            is_last=True,
            feedback_fn=lambda _positions: None,
            cancel_flag=threading.Event(),
        )

        self.assertEqual(result, RESULT_SUCCESSFUL)
        self.assertEqual(iface.stop_calls, 0)
        self.assertEqual(iface.states, [])

    def test_required_limits_fail_closed_but_jerk_is_diagnostic(self):
        class _Cps:
            def HRIF_ReadJointMaxVel(self, _box, _robot, result):
                result.extend([100.0] * 6)
                return 0

            def HRIF_ReadJointMaxAcc(self, _box, _robot, result):
                result.extend([80.0] * 6)
                return 0

        iface = HuayanInterface.__new__(HuayanInterface)
        iface._node = _Node()
        iface._cps = _Cps()
        iface._controller_limit_fraction = 0.8

        self.assertTrue(iface._read_controller_motion_limits())
        self.assertIsNone(iface._controller_max_jerk_deg)
        self.assertTrue(iface._node.logger.warnings)

        delattr(_Cps, "HRIF_ReadJointMaxAcc")
        self.assertFalse(iface._read_controller_motion_limits())

    def test_limit_fraction_cannot_exceed_safety_maximum(self):
        with self.assertRaisesRegex(ValueError, "controller_limit_fraction"):
            HuayanInterface(_Node(), controller_limit_fraction=0.81)

    def test_servo_j_uses_start_servo_and_push_servo_j_only(self):
        class _Cps:
            def __init__(self):
                self.calls = []

            def HRIF_StartServo(self, box, robot, servo_time, lookahead):
                self.calls.append(
                    ("StartServo", box, robot, servo_time, lookahead))
                return 0

            def HRIF_PushServoJ(self, box, robot, joints):
                self.calls.append(("PushServoJ", box, robot, list(joints)))
                return 0

        iface = _polling_interface(_Cps())
        iface._monitor_only = False
        iface._ensure_connected = lambda: True
        iface._refresh_positions = lambda: None
        iface._current_positions_deg = [0.0] * 6

        result = iface.execute_servo_j_path(
            [[0.0] * 6, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]],
            feedback_fn=lambda _positions: None,
            cancel_flag=threading.Event(),
            servo_time=0.001,
            lookahead_time=0.01,
        )

        self.assertEqual(result, RESULT_SUCCESSFUL)
        self.assertEqual(iface._cps.calls[0][0], "StartServo")
        self.assertEqual([call[0] for call in iface._cps.calls[1:]], [
            "PushServoJ", "PushServoJ"])
        self.assertEqual(iface.states, [
            ConnectionState.EXECUTING, ConnectionState.READY])

    def test_servo_j_cancel_calls_grp_stop_path(self):
        iface = _polling_interface(object())
        iface._monitor_only = False
        iface._ensure_connected = lambda: True
        cancel = threading.Event()
        cancel.set()

        result = iface.execute_servo_j_path(
            [[0.0] * 6],
            feedback_fn=lambda _positions: None,
            cancel_flag=cancel,
            servo_time=0.001,
            lookahead_time=0.01,
        )

        self.assertEqual(result, RESULT_PREEMPTED)
        self.assertEqual(iface.stop_calls, 1)

    def test_servo_j_rejects_bad_path_before_motion(self):
        iface = _polling_interface(object())
        iface._monitor_only = False

        result = iface.execute_servo_j_path(
            [[999.0] * 6],
            feedback_fn=lambda _positions: None,
            cancel_flag=threading.Event(),
        )

        self.assertEqual(result, RESULT_INVALID_GOAL)
        self.assertEqual(iface.stop_calls, 0)


if __name__ == "__main__":
    unittest.main()
