import math
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
    already_motion_ready,
    connect2box_allow_refuse,
    cps_step_ok,
    electrify_allow_refuse,
    hrif_waypoint_joint,
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


if __name__ == "__main__":
    unittest.main()
