"""实机代码 (site / real-cell). Not part of the Gazebo simulation stack."""

import math
import threading
import unittest

from elfin_trajectory_executor.cps_parse import (
    acs_from_read_act_pos,
    as_bit,
    as_float_list,
    choose_joint_vel_deg,
    finite_diff_deg_s,
    joints_moved_deg,
    rpy_deg_to_quat_xyzw,
    tcp_from_read_act_pos,
)
from elfin_trajectory_executor.huayan_interface import (
    FSM_BOX_DISCONNECT,
    FSM_DISABLE,
    FSM_ENABLING,
    FSM_STANDBY,
    STATE_REFUSE,
    HuayanInterface,
    RESULT_INVALID_GOAL,
    already_motion_ready,
    connect2box_allow_refuse,
    cps_step_ok,
    decimate_joint_waypoints,
    electrify_allow_refuse,
    hrif_waypoint_joint,
    read_positive_joint_limits,
    safe_waypoint_profile,
)
from elfin_trajectory_executor.vacuum_io import snapshot as vacuum_snapshot


class CpsParseTest(unittest.TestCase):
    def test_as_bit_cps_strings_and_bools(self):
        self.assertEqual(as_bit(["1"]), 1)
        self.assertEqual(as_bit(["0"]), 0)
        self.assertEqual(as_bit([True]), 1)
        self.assertEqual(as_bit([False]), 0)
        self.assertEqual(as_bit(["true"]), 1)
        self.assertIsNone(as_bit([]))
        self.assertIsNone(as_bit(["2"]))
        self.assertIsNone(as_bit(None))

    def test_as_float_list_accepts_cps_strings(self):
        raw = ["1.5", "2", "-3.25", "0", "10", "20", "ignored"]
        self.assertEqual(
            as_float_list(raw, 6),
            [1.5, 2.0, -3.25, 0.0, 10.0, 20.0],
        )

    def test_as_float_list_rejects_short_or_garbage(self):
        self.assertIsNone(as_float_list(["1", "2"], 6))
        self.assertIsNone(as_float_list(["a"] * 6, 6))
        self.assertIsNone(as_float_list(None, 6))

    def test_tcp_slice_matches_read_act_tcp_pos(self):
        joints = [10, 20, 30, 40, 50, 60]
        tcp = [100.0, 200.0, 300.0, 1.0, 2.0, 3.0]
        rest = [0] * 12
        self.assertEqual(tcp_from_read_act_pos(joints + tcp + rest), tcp)
        self.assertEqual(acs_from_read_act_pos(joints + tcp + rest), joints)
        self.assertIsNone(tcp_from_read_act_pos(joints))

    def test_finite_diff_vel(self):
        prev = [0.0] * 6
        curr = [10.0, 0, 0, 0, 0, 0]
        self.assertEqual(finite_diff_deg_s(prev, curr, 0.1)[0], 100.0)
        self.assertIsNone(finite_diff_deg_s(prev, curr, 1e-6))

    def test_choose_vel_falls_back_when_cps_stuck_at_zero(self):
        cps = [0.0] * 6
        fd = [12.0, 0, 0, 0, 0, 0]
        vel, src = choose_joint_vel_deg(cps, fd, moved=True)
        self.assertEqual(vel[0], 12.0)
        self.assertEqual(src, "finite_diff")
        vel, src = choose_joint_vel_deg(cps, fd, moved=False)
        self.assertEqual(vel, cps)
        self.assertEqual(src, "cps")
        vel, src = choose_joint_vel_deg([3.0] + [0] * 5, fd, moved=True)
        self.assertEqual(vel[0], 3.0)
        self.assertEqual(src, "cps")
        self.assertTrue(joints_moved_deg([0] * 6, [0.2] + [0] * 5))
        self.assertFalse(joints_moved_deg([0] * 6, [0] * 6))

    def test_rpy_identity_and_90_yaw(self):
        qx, qy, qz, qw = rpy_deg_to_quat_xyzw(0.0, 0.0, 0.0)
        self.assertAlmostEqual(qx, 0.0)
        self.assertAlmostEqual(qy, 0.0)
        self.assertAlmostEqual(qz, 0.0)
        self.assertAlmostEqual(qw, 1.0)
        qx, qy, qz, qw = rpy_deg_to_quat_xyzw(0.0, 0.0, 90.0)
        self.assertAlmostEqual(qx, 0.0)
        self.assertAlmostEqual(qy, 0.0)
        self.assertAlmostEqual(qz, math.sin(math.radians(45.0)))
        self.assertAlmostEqual(qw, math.cos(math.radians(45.0)))

    def test_vacuum_snapshot_maps_bits(self):
        class _Iface:
            last_io_ok = True
            vacuum_io_kind = "box"
            vacuum_di0 = 1
            vacuum_do0 = 1
            vacuum_do1 = 0

        payload = vacuum_snapshot(_Iface())
        self.assertEqual(payload["DI0"], 1)
        self.assertTrue(payload["suction_ok"])
        self.assertTrue(payload["vacuum_on"])
        self.assertFalse(payload["de_vacuum"])

    def test_cps_step_ok_accepts_state_refuse_when_allowed(self):
        self.assertTrue(cps_step_ok(0, False))
        self.assertFalse(cps_step_ok(STATE_REFUSE, False))
        self.assertTrue(cps_step_ok(STATE_REFUSE, True))
        self.assertFalse(cps_step_ok(1, True))

    def test_standby_skips_cold_start_and_allows_connect2box_refuse(self):
        self.assertTrue(already_motion_ready(FSM_STANDBY))
        self.assertFalse(already_motion_ready(FSM_ENABLING))
        self.assertFalse(already_motion_ready(FSM_DISABLE))
        self.assertFalse(already_motion_ready(None))
        self.assertTrue(connect2box_allow_refuse(FSM_STANDBY))
        self.assertFalse(connect2box_allow_refuse(FSM_BOX_DISCONNECT))
        # Disable is already powered; Electrify 20018 is already-done.
        self.assertTrue(electrify_allow_refuse(FSM_STANDBY))
        self.assertTrue(electrify_allow_refuse(FSM_DISABLE))
        self.assertTrue(electrify_allow_refuse(FSM_ENABLING))
        self.assertFalse(electrify_allow_refuse(None))

    def test_hrif_waypoint_joint_uses_python_sdk_lists(self):
        class _Cps:
            def __init__(self):
                self.calls = []

            def HRIF_WayPoint(
                    self, boxID, rbtID, type, points, RawACSpoints, tcp, ucs,
                    speed, Acc, radius, isJoint, isSeek, bit, state, cmdID):
                self.calls.append({
                    "boxID": boxID,
                    "rbtID": rbtID,
                    "type": type,
                    "points": list(points),
                    "RawACSpoints": list(RawACSpoints),
                    "tcp": tcp,
                    "ucs": ucs,
                    "speed": speed,
                    "Acc": Acc,
                    "radius": radius,
                    "isJoint": isJoint,
                    "isSeek": isSeek,
                    "bit": bit,
                    "state": state,
                    "cmdID": cmdID,
                })
                return 0

        cps = _Cps()
        joints = [10.0, -20.0, 30.0, 40.0, 50.0, 60.0]
        n_ret = hrif_waypoint_joint(cps, joints, 15.0, 60.0, 0.0, "1")
        self.assertEqual(n_ret, 0)
        call = cps.calls[0]
        self.assertEqual(call["type"], 0)
        self.assertEqual(call["points"], [0.0] * 6)
        self.assertEqual(call["RawACSpoints"], joints)
        self.assertEqual(call["isJoint"], 1)
        self.assertEqual(call["cmdID"], "1")
        with self.assertRaises(TypeError):
            cps.HRIF_WayPoint(
                0, 0, 0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                *joints,
                "TCP", "Base", 15.0, 60.0, 0.0, 1, 0, 0, 0, "1",
            )

    def test_decimate_drops_totg_chatter_keeps_goal(self):
        start = [0.0] * 6
        chatter = []
        for d in (0.0, 0.3, -0.2, 0.4, 0.1):
            q = [0.0] * 6
            q[1] = d
            chatter.append(q)
        goal = [0.0] * 6
        goal[1] = 25.0
        chatter.append(goal)
        kept = decimate_joint_waypoints(chatter, min_delta_deg=2.0, start_deg=start)
        self.assertEqual(kept, [len(chatter) - 1])

    def test_decimate_keeps_real_via_and_last(self):
        pts = []
        for j2 in (0.0, 0.4, 12.0, 12.3, 40.0):
            q = [0.0] * 6
            q[1] = j2
            pts.append(q)
        kept = decimate_joint_waypoints(pts, min_delta_deg=2.0, start_deg=[0.0] * 6)
        self.assertEqual(kept, [2, 4])

    def test_reads_positive_controller_joint_limits(self):
        class _Cps:
            def HRIF_ReadJointMaxAcc(self, box_id, robot_id, result):
                result.extend(["80", "81", "82", "83", "84", "85"])
                return 0

        self.assertEqual(
            read_positive_joint_limits(_Cps(), "HRIF_ReadJointMaxAcc"),
            [80.0, 81.0, 82.0, 83.0, 84.0, 85.0],
        )
        self.assertIsNone(read_positive_joint_limits(_Cps(), "missing"))

        class _BadCps:
            def HRIF_ReadJointMaxAcc(self, box_id, robot_id, result):
                result.extend([80, 80, 0, 80, 80, 80])
                return 0

        self.assertIsNone(
            read_positive_joint_limits(_BadCps(), "HRIF_ReadJointMaxAcc")
        )

    def test_safe_waypoint_profile_uses_proven_acceleration_and_caps_velocity(self):
        self.assertEqual(
            safe_waypoint_profile(
                54.0, 60.0, 60.0, [100.0] * 6, [100.0] * 6),
            (54.0, 60.0),
        )
        self.assertEqual(
            safe_waypoint_profile(
                70.0, 60.0, 60.0, [100.0] * 6, [100.0] * 6),
            (59.0, 60.0),
        )
        self.assertEqual(
            safe_waypoint_profile(
                54.0, 60.0, 60.0,
                controller_max_velocity_deg=[20.0] * 6,
                controller_max_acceleration_deg=[50.0] * 6,
            ),
            (16.0, 40.0),
        )
        with self.assertRaises(ValueError):
            safe_waypoint_profile(
                10.0, 0.5, 20.0, [100.0] * 6, [100.0] * 6)
        with self.assertRaisesRegex(ValueError, "velocity limits are unavailable"):
            safe_waypoint_profile(10.0, 60.0, 20.0, None, [100.0] * 6)
        with self.assertRaisesRegex(ValueError, "exceeds safety maximum"):
            safe_waypoint_profile(
                10.0, 60.0, 20.0, [100.0] * 6, [100.0] * 6, 0.81)

    def test_profile_preflight_rejects_before_first_waypoint(self):
        class _Logger:
            def info(self, message):
                pass

            def error(self, message):
                pass

        class _Node:
            def get_logger(self):
                return _Logger()

        class _Cps:
            def __init__(self):
                self.waypoint_calls = 0

            def HRIF_WayPoint(self, *args):
                self.waypoint_calls += 1
                return 0

        class _Duration:
            def __init__(self, seconds):
                self.sec = int(seconds)
                self.nanosec = int(round((seconds - self.sec) * 1e9))

        class _Point:
            def __init__(self, degrees, seconds):
                self.positions = [math.radians(v) for v in degrees]
                self.velocities = []
                self.accelerations = []
                self.time_from_start = _Duration(seconds)

        class _Trajectory:
            def __init__(self):
                self.points = [
                    _Point([0, 3, 0, 0, 0, 0], 0.2),
                    _Point([0, 6, 0, 0, 0, 0], 0.4),
                ]

        iface = HuayanInterface.__new__(HuayanInterface)
        iface._monitor_only = False
        iface._node = _Node()
        iface._cps = _Cps()
        iface._current_positions_deg = [0.0] * 6
        iface._default_vel = 10.0
        iface._max_vel = 20.0
        iface._command_accel = 0.5
        iface._controller_max_velocity_deg = None
        iface._controller_max_acceleration_deg = None
        iface._controller_limit_fraction = 0.8
        iface._ensure_connected = lambda: True
        iface._refresh_positions = lambda: None
        iface._set_state = lambda state: None

        result = iface.execute(
            _Trajectory(), lambda positions: None, threading.Event()
        )
        self.assertEqual(result, RESULT_INVALID_GOAL)
        self.assertEqual(iface._cps.waypoint_calls, 0)


if __name__ == "__main__":
    unittest.main()
