"""实机代码 (site / real-cell). Not part of the Gazebo simulation stack."""

from __future__ import annotations

import math
import threading
import unittest
from types import SimpleNamespace

from elfin_trajectory_executor.huayan_interface import (
    BACKEND_SERVO_ESJ,
    ConnectionState,
    HuayanInterface,
    RESULT_INVALID_GOAL,
    RESULT_PREEMPTED,
    RESULT_SUCCESSFUL,
)
from elfin_trajectory_executor.waypoint_profile import PREFLIGHT_FAILED_LOG


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, msg):
        self.lines.append(("info", str(msg)))

    def warn(self, msg):
        self.lines.append(("warn", str(msg)))

    warning = warn

    def error(self, msg):
        self.lines.append(("error", str(msg)))


class FakeNode:
    def __init__(self):
        self._log = _Log()

    def get_logger(self):
        return self._log


class FakeCPS:
    def __init__(self):
        self.calls = []
        self.waypoint_args = []
        self.push_batches = []
        self.set_calls = []
        self.stopped = False
        self.commands_after_stop = 0
        self.positions = [0.0] * 6
        self.joint_max_vel = [120.0] * 6
        self.joint_max_acc = [80.0] * 6
        self.joint_max_jerk = [200.0] * 6
        self.blend_n = 0
        self.motion_n = 0
        self.servo_state = ["0"]
        self.waypoint_ret = 0
        self.push_ret = 0
        self.motion_done_after_pushes = None
        self.cancel_on_push = None

    def _name(self, name):
        self.calls.append(name)

    def HRIF_IsConnected(self, box):
        self._name("HRIF_IsConnected")
        return True

    def HRIF_Connect(self, *a):
        self._name("HRIF_Connect")
        return 0

    def HRIF_ReadCurFSM(self, box, rbt, result):
        self._name("HRIF_ReadCurFSM")
        result.clear()
        result.extend([33, "StandBy"])
        return 0

    def HRIF_Connect2Box(self, box):
        self._name("HRIF_Connect2Box")
        return 0

    def HRIF_Electrify(self, box):
        self._name("HRIF_Electrify")
        return 0

    def HRIF_Connect2Controller(self, box):
        self._name("HRIF_Connect2Controller")
        return 0

    def HRIF_IsControllerStarted(self, box, result):
        self._name("HRIF_IsControllerStarted")
        result.clear()
        result.append("1")
        return 0

    def HRIF_ReadRobotState(self, box, rbt, result):
        self._name("HRIF_ReadRobotState")
        result.clear()
        result.extend(["0", "1"])
        return 0

    def HRIF_ReadVersion(self, box, rbt, result):
        self._name("HRIF_ReadVersion")
        result.clear()
        result.extend(["V1", "CPS", "Codesys", "S20"])
        return 0

    def HRIF_ReadJointMaxVel(self, box, rbt, result):
        self._name("HRIF_ReadJointMaxVel")
        result.clear()
        result.extend(str(v) for v in self.joint_max_vel)
        return 0

    def HRIF_ReadJointMaxAcc(self, box, rbt, result):
        self._name("HRIF_ReadJointMaxAcc")
        result.clear()
        result.extend(str(v) for v in self.joint_max_acc)
        return 0

    def HRIF_ReadJointMaxJerk(self, box, rbt, result):
        self._name("HRIF_ReadJointMaxJerk")
        result.clear()
        result.extend(str(v) for v in self.joint_max_jerk)
        return 0

    def HRIF_SetJointMaxVel(self, *a):
        self.set_calls.append("HRIF_SetJointMaxVel")
        self._name("HRIF_SetJointMaxVel")
        return 1

    def HRIF_SetJointMaxAcc(self, *a):
        self.set_calls.append("HRIF_SetJointMaxAcc")
        self._name("HRIF_SetJointMaxAcc")
        return 1

    def HRIF_SetJointMaxJerk(self, *a):
        self.set_calls.append("HRIF_SetJointMaxJerk")
        self._name("HRIF_SetJointMaxJerk")
        return 1

    def HRIF_ReadActPos(self, box, rbt, result):
        self._name("HRIF_ReadActPos")
        result.clear()
        result.extend(self.positions)
        result.extend([0.0, 0.0, 400.0, 180.0, 0.0, 180.0])
        return 0

    def HRIF_ReadActJointVel(self, box, rbt, result):
        result.clear()
        result.extend(["0"] * 6)
        return 0

    def HRIF_ReadActJointCur(self, box, rbt, result):
        result.clear()
        result.extend(["0"] * 6)
        return 0

    def HRIF_ReadCmdJointPos(self, box, rbt, result):
        result.clear()
        result.extend(str(v) for v in self.positions)
        return 0

    def HRIF_ReadBoxDI(self, box, bit, result):
        result.clear()
        result.append("0")
        return 0

    def HRIF_ReadBoxDO(self, box, bit, result):
        result.clear()
        result.append("0")
        return 0

    def HRIF_WayPoint(self, *args):
        self._name("HRIF_WayPoint")
        if self.stopped:
            self.commands_after_stop += 1
        self.waypoint_args.append(args)
        return self.waypoint_ret

    def HRIF_IsBlendingDone(self, box, rbt, result):
        self.blend_n += 1
        result.clear()
        result.append(self.blend_n % 2 == 0)
        return 0

    def HRIF_IsMotionDone(self, box, rbt, result):
        self.motion_n += 1
        result.clear()
        if self.motion_done_after_pushes is not None:
            result.append(len(self.push_batches) >= self.motion_done_after_pushes)
        else:
            result.append(self.motion_n % 2 == 0)
        return 0

    def HRIF_GrpStop(self, *a):
        self._name("HRIF_GrpStop")
        self.stopped = True
        return 0

    def HRIF_GrpReset(self, *a):
        self._name("HRIF_GrpReset")
        return 0

    def HRIF_GrpEnable(self, *a):
        self._name("HRIF_GrpEnable")
        return 0

    def HRIF_GrpDisable(self, *a):
        self._name("HRIF_GrpDisable")
        return 0

    def HRIF_GetErrorCodeStr(self, box, code, result):
        result.clear()
        result.append("err-%s" % code)
        return 0

    def HRIF_InitServoEsJ(self, *a):
        self._name("HRIF_InitServoEsJ")
        return 0

    def HRIF_StartServoEsJ(self, *a):
        self._name("HRIF_StartServoEsJ")
        if self.stopped:
            self.commands_after_stop += 1
        return 0

    def HRIF_PushServoEsJ(self, box, rbt, n_points, s_points):
        self._name("HRIF_PushServoEsJ")
        if self.stopped:
            self.commands_after_stop += 1
        self.push_batches.append((n_points, list(s_points)))
        if self.cancel_on_push is not None:
            self.cancel_on_push.set()
        return self.push_ret

    def HRIF_ReadServoEsJState(self, box, rbt, result):
        self._name("HRIF_ReadServoEsJState")
        result.clear()
        result.extend(self.servo_state)
        return 0


def _traj(points_deg, times_s):
    pts = []
    for q, t in zip(points_deg, times_s):
        sec = int(t)
        nsec = int(round((t - sec) * 1e9))
        pts.append(
            SimpleNamespace(
                positions=[math.radians(v) for v in q],
                velocities=[],
                accelerations=[],
                time_from_start=SimpleNamespace(sec=sec, nanosec=nsec),
            )
        )
    return SimpleNamespace(points=pts)


def _q(j2: float):
    q = [0.0] * 6
    q[1] = j2
    return q


def _iface(**kwargs):
    node = FakeNode()
    iface = HuayanInterface(
        node,
        poll_interval_s=0.0,
        blend_start_timeout_s=0.0,
        stop_settle_s=0.0,
        **kwargs,
    )
    cps = FakeCPS()
    iface._cps = cps
    iface._set_state(ConnectionState.READY)
    iface._read_controller_motion_limits()
    return iface, cps, node


class FakeCpsExecuteTest(unittest.TestCase):
    def test_connect_reads_limits_never_sets(self):
        iface, cps, node = _iface()
        iface._set_state(ConnectionState.DISCONNECTED)
        self.assertTrue(iface._do_connect())
        joined = ",".join(cps.calls)
        self.assertIn("HRIF_ReadJointMaxVel", joined)
        self.assertIn("HRIF_ReadJointMaxAcc", joined)
        self.assertIn("HRIF_ReadJointMaxJerk", joined)
        self.assertIn("HRIF_ReadVersion", joined)
        self.assertEqual(cps.set_calls, [])
        self.assertIn("Controller limits vel=", "".join(m for _, m in node._log.lines))

    def test_h1_accel_rejected_zero_waypoints(self):
        iface, cps, _ = _iface(command_acceleration_deg=0.5, max_velocity_deg=20.0)
        traj = _traj([_q(0.0), _q(5.0)], [0.0, 1.0])
        ret = iface.execute(traj, lambda _p: None, threading.Event())
        self.assertEqual(ret, RESULT_INVALID_GOAL)
        self.assertEqual(sum(1 for c in cps.calls if c == "HRIF_WayPoint"), 0)
        self.assertIn(PREFLIGHT_FAILED_LOG, iface.last_error_string)

    def test_invalid_later_waypoint_zero_calls(self):
        iface, cps, _ = _iface()
        bad = _q(400.0)
        traj = _traj([_q(0.0), bad], [0.0, 1.0])
        ret = iface.execute(traj, lambda _p: None, threading.Event())
        self.assertEqual(ret, RESULT_INVALID_GOAL)
        self.assertEqual(sum(1 for c in cps.calls if c == "HRIF_WayPoint"), 0)

    def test_legal_54_style_profile_sends_accel_60(self):
        iface, cps, _ = _iface(
            command_acceleration_deg=60.0,
            max_velocity_deg=60.0,
            default_velocity_deg=54.0,
        )
        traj = _traj([_q(0.0), _q(25.0)], [0.0, 1.0])
        ret = iface.execute(traj, lambda _p: None, threading.Event())
        self.assertEqual(ret, RESULT_SUCCESSFUL)
        self.assertGreaterEqual(sum(1 for c in cps.calls if c == "HRIF_WayPoint"), 1)
        for args in cps.waypoint_args:
            vel = args[7]
            accel = args[8]
            self.assertLessEqual(accel, 60.0 + 1e-9)
            self.assertLessEqual(vel, accel - 1.0 + 1e-9)
        self.assertEqual(cps.set_calls, [])

    def test_cancel_stops_and_sends_no_waypoint_after_grpstop(self):
        iface, cps, _ = _iface()
        cancel = threading.Event()

        def feedback(_p):
            cancel.set()

        traj = _traj([_q(0.0), _q(25.0), _q(50.0)], [0.0, 1.0, 2.0])
        ret = iface.execute(traj, feedback, cancel)
        self.assertEqual(ret, RESULT_PREEMPTED)
        self.assertIn("HRIF_GrpStop", cps.calls)
        self.assertEqual(cps.commands_after_stop, 0)
        self.assertLess(sum(1 for c in cps.calls if c == "HRIF_WayPoint"), 3)

    def test_servo_esj_request_uses_waypoint(self):
        iface, cps, node = _iface(execution_backend=BACKEND_SERVO_ESJ)
        self.assertEqual(iface._execution_backend, "waypoint")
        self.assertTrue(
            any("waypoint-only" in msg for _, msg in node._log.lines)
        )
        traj = _traj([_q(0.0), _q(10.0)], [0.0, 1.0])
        ret = iface.execute(traj, lambda _p: None, threading.Event())
        self.assertEqual(ret, RESULT_SUCCESSFUL)
        self.assertGreaterEqual(sum(1 for c in cps.calls if c == "HRIF_WayPoint"), 1)
        self.assertEqual(sum(1 for c in cps.calls if c == "HRIF_PushServoEsJ"), 0)

    def test_launch_files_wire_accel_and_backend(self):
        from pathlib import Path
        root = Path(__file__).resolve().parents[1]
        jazzy = (root / "launch" / "jazzy_real.launch.py").read_text(encoding="utf-8")
        rec = (root / "launch" / "record_site.launch.py").read_text(encoding="utf-8")
        yaml = (root / "config" / "executor.yaml").read_text(encoding="utf-8")
        self.assertIn("command_acceleration_deg", jazzy)
        self.assertIn("execution_backend", jazzy)
        self.assertIn('default_value="waypoint"', jazzy)
        self.assertIn("servo_esj is rejected", jazzy)
        self.assertIn("command_acceleration_deg", rec)
        self.assertIn('default_value="waypoint"', rec)
        self.assertIn('"execution_backend"', rec)
        self.assertIn("servo_esj is rejected", rec)
        self.assertIn("servo_j_servo_time", rec)
        self.assertIn("command_acceleration_deg: 60.0", yaml)
        pick = (
            Path(__file__).resolve().parents[4]
            / "src" / "luggage_planning" / "launch" / "hardware_pick.launch.py"
        )
        self.assertTrue(pick.is_file(), pick)
        src = pick.read_text(encoding="utf-8")
        self.assertIn("command_acceleration_deg", src)
        self.assertIn('"max_velocity_deg": "60.0"', src)


if __name__ == "__main__":
    unittest.main()
