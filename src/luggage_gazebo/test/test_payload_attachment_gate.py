#!/usr/bin/env python3
"""Vacuum-attached is a carrying-leg precondition, not a return-leg one.

Run15b aborted PLACE_LOST_PAYLOAD on the unloaded ``return`` leg after a
successful detach. Carrying legs still fail closed when the box is gone.
Settle/verify runs only after retreat and return have been admitted.
"""

import os
import sys
import unittest

_SCRIPTS = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from geometry_msgs.msg import Pose  # noqa: E402
from luggage_msgs.msg import MotionSegment  # noqa: E402

from place_smoke_driver import (  # noqa: E402
    CANONICAL_ARM_JOINTS,
    PlaceSmokeDriver,
    payload_attachment_gate,
    segment_requires_payload_attached,
)
from luggage_description.scene_tf_config_utils import (  # noqa: E402
    load_scene_tf_config,
)
from luggage_gazebo.place_metrics import PlaceTrial  # noqa: E402

_SCENE = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "luggage_description", "config", "scene_tf_deploy.yaml"))
# Slot 656 from g2 run15b: accepted inside the deploy inner box.
_BASE = (0.5850054914376829, 1.5049977409687083, -0.20642825678177834)
_WORLD = (-1.5049998898037118, 0.5849999632683536, 0.6535717432182216)


class _Seg(object):
    def __init__(self, name, requires=None, load_state=None):
        self.name = name
        self.type = "joint_path"
        self.keep_tool_down = True
        self.keep_camera_down = False
        self.lock_wrist = False
        self.allow_ompl_fallback = False
        pose = Pose()
        pose.orientation.w = 1.0
        self.target_pose = pose
        if requires is not None:
            self.requires_payload_attached = requires
        if load_state is not None:
            self.load_state = load_state


class _Args(object):
    use_vacuum = True
    plan_only = False
    drift_wait = 0.0
    plan_timeout = 5.0


class _Log(object):
    def info(self, *_args, **_kwargs):
        return None


def _held(attached=True, reason=""):
    return {
        "attached": attached,
        "vacuum_on": attached,
        "fail_reason": reason,
    }


class TestPayloadAttachmentGate(unittest.TestCase):
    def test_carrying_names_require_attachment_before_detach(self):
        for name in ("transit", "traverse", "insert", "descend"):
            ok, code = payload_attachment_gate(
                _Seg(name), _held(False), detached=False)
            self.assertFalse(ok, name)
            self.assertEqual(code, "PLACE_LOST_PAYLOAD", name)
            ok_held, code_held = payload_attachment_gate(
                _Seg(name), _held(True), detached=False)
            self.assertTrue(ok_held, name)
            self.assertEqual(code_held, "", name)

    def test_retreat_and_return_allow_detached_vacuum(self):
        for name in ("retreat", "return", "home"):
            ok, code = payload_attachment_gate(
                _Seg(name), _held(False), detached=False)
            self.assertTrue(ok, name)
            self.assertEqual(code, "", name)

    def test_any_leg_after_successful_detach_allows_unattached(self):
        ok, code = payload_attachment_gate(
            _Seg("transit"), _held(False), detached=True)
        self.assertTrue(ok)
        self.assertEqual(code, "")

    def test_explicit_metadata_overrides_the_name(self):
        self.assertTrue(segment_requires_payload_attached(
            _Seg("return", requires=True), detached=True))
        self.assertFalse(segment_requires_payload_attached(
            _Seg("transit", requires=False), detached=False))
        ok, code = payload_attachment_gate(
            _Seg("transit", requires=False), _held(False), detached=False)
        self.assertTrue(ok)
        self.assertEqual(code, "")

    def test_load_state_overrides_the_name_before_detach(self):
        ok, code = payload_attachment_gate(
            _Seg("return"), _held(False), detached=False,
            load_state="carrying")
        self.assertFalse(ok)
        self.assertEqual(code, "PLACE_LOST_PAYLOAD")
        ok_free, code_free = payload_attachment_gate(
            _Seg("transit"), _held(False), detached=False,
            load_state="unloaded")
        self.assertTrue(ok_free)
        self.assertEqual(code_free, "")

    def test_fail_reason_after_detach_is_a_vacuum_failure(self):
        ok, code = payload_attachment_gate(
            _Seg("return"), _held(False, reason="seal_lost"),
            detached=True)
        self.assertFalse(ok)
        self.assertTrue(code.startswith("VACUUM_DETACH:"))
        self.assertNotEqual(code, "PLACE_LOST_PAYLOAD")

    def test_carrying_fail_reason_stays_a_lost_payload(self):
        ok, code = payload_attachment_gate(
            _Seg("descend"), _held(True, reason="follow_timeout"),
            detached=False)
        self.assertFalse(ok)
        self.assertEqual(code, "PLACE_LOST_PAYLOAD")


class _Flow(object):
    """Real place loop, stubbed graph. Records leg submission vs settle."""

    def __init__(self, attached=True, detach_reason=""):
        self._args = _Args()
        self._vacuum_state = _held(attached)
        self._payload_detached = False
        self._detach_reason = detach_reason
        self._probe_joints = None
        self._segments_log = []
        self._timeline = []
        self._place_state = "INIT"
        self._box_model = "pickup_box_0001_carryon"
        self._keep_placed = True
        self._follow_skipped0 = 0
        self._scene_config = load_scene_tf_config(_SCENE)
        self._typed = None
        self._plan = None
        self._place_touch = None
        self._add_placed = None
        self._exit_yaw = {}
        self._frame_motion_segments = []
        self._frame_motion_index = 0
        self._segment_exec_name = None
        self._segment_exec_t0 = None
        self._joint_ring_horizon_sec = 1.0
        self._last_joint_state = {
            "stamp": 2.0,
            "name": list(CANONICAL_ARM_JOINTS),
            "position": [0.0, -1.0, 0.0, 0.0, 0.0, 0.0],
        }
        self.events = []
        self._rest = list(_WORLD) + [0.0, 0.0, 3.1415926535]

    def ros_now_sec(self):
        return 1.0

    def get_logger(self):
        return _Log()

    def _follow_skipped(self):
        return 0

    def _clear_smoke_slot(self, _slot, delete_model=False):
        return None

    def _set_state(self, new_state, guard="", note=""):
        self.events.append("state:%s" % new_state)
        self._place_state = new_state

    def _sample_tf_box(self, label="", query_gz=False):
        self.events.append(label)
        return {
            "suction": None,
            "box_gz": None,
            "vacuum": dict(self._vacuum_state),
        }

    def suction_xyz(self):
        return None, ""

    def vacuum_command(self, enable, pick_msg=None):
        self.events.append("vacuum:%s" % int(bool(enable)))
        self._vacuum_state = _held(
            bool(enable), self._detach_reason if not enable else "")
        return True, "detached pickup_box_0001_carryon"

    def _release_as_dynamic(self):
        self.events.append("dynamic")
        return True, "dynamic", list(self._rest)

    def _transition_admission_unloaded(self, _evidence):
        self.events.append("phase")
        return True, "OK"

    def _gz_box_pose(self):
        return list(self._rest)

    def _poses_after_settle(self):
        self.events.append("settle")
        pose = list(self._rest)
        return pose, pose, {
            "settled": True, "pose": list(_WORLD), "window_move_m": 0.0}

    def _capture_success_debug(self, *_args, **_kwargs):
        self.events.append("verify_debug")
        return {}

    def _dump_place(self, *_args, **_kwargs):
        self.events.append("dump")

    def _home_arm(self):
        self.events.append("home")
        return True, "ok", None

    def call_srv(self, *_args, **_kwargs):
        return type("Resp", (), {"success": True, "message": "ok"})()

    def send_action(self, _client, goal, timeout=0, name=""):
        self.events.append("exec:%s" % goal.segment.name)
        result = type("Result", (), {
            "fraction": 1.0,
            "used_ompl_fallback": False,
            "moveit_error_code": 1,
            "settle_json": "probe",
        })()
        return True, "timed joint path", result

    _check_i1 = PlaceSmokeDriver._check_i1
    _joint_sample = PlaceSmokeDriver._joint_sample
    _timed_leg = PlaceSmokeDriver._timed_leg
    _await_retreat_start_contract = (
        PlaceSmokeDriver._await_retreat_start_contract)
    _peek_leg_load_state = PlaceSmokeDriver._peek_leg_load_state
    _payload_attachment_gate = PlaceSmokeDriver._payload_attachment_gate
    _leg_execution_admission = PlaceSmokeDriver._leg_execution_admission
    _execute_segment = PlaceSmokeDriver._execute_segment
    _set_place_touch = PlaceSmokeDriver._set_place_touch
    _abort = PlaceSmokeDriver._abort
    _slot_world_msg = PlaceSmokeDriver._slot_world_msg
    run_place_from_carry = PlaceSmokeDriver.run_place_from_carry


def _slot():
    slot = type("Slot", (), {})()
    slot.layer = slot.row = slot.col = 0
    slot.width, slot.depth, slot.height = 0.50, 0.40, 0.25
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = _BASE
    pose.orientation.w = 1.0
    slot.place_pose = pose
    meta = {
        "planning_frame": "world",
        "pose_world": {"position": list(_WORLD), "yaw": 3.1415926535},
        "pose_base_link": {"position": list(_BASE), "yaw": 1.5707963},
        "source": "test",
    }
    return slot, meta


def _motion(name):
    segment = MotionSegment()
    segment.name = name
    segment.type = "joint_path"
    segment.keep_tool_down = True
    segment.target_pose.orientation.w = 1.0
    return segment


def _timed_leg(name):
    point = {
        "positions": [0.0, -1.0, 0.0, 0.0, 0.0, 0.0],
    }
    return type("Leg", (), {
        "name": name,
        "joint_trajectory": {
            "joint_names": list(CANONICAL_ARM_JOINTS),
            "points": [point, dict(point)],
        },
    })()


def _route():
    return [_motion(name) for name in (
        "transit", "insert", "descend", "retreat", "return")]


def _route_v2():
    return [_timed_leg(name) for name in (
        "transit", "insert", "descend", "retreat", "return")]


class TestSettleFollowsReturn(unittest.TestCase):
    def test_carrying_segment_does_not_submit_when_unattached(self):
        driver = _Flow(attached=False)
        trial = PlaceTrial(index=0)
        ok, code, rec = driver._execute_segment(_motion("transit"), trial)
        self.assertFalse(ok)
        self.assertEqual(code, "PLACE_LOST_PAYLOAD")
        self.assertIsNone(rec)
        self.assertFalse(any(item.startswith("exec:") for item in driver.events))

    def test_return_submits_when_vacuum_is_already_off(self):
        driver = _Flow(attached=False)
        driver._payload_detached = True
        trial = PlaceTrial(index=0)
        ok, code, rec = driver._execute_segment(_motion("return"), trial)
        self.assertTrue(ok, code)
        self.assertEqual(code, "")
        self.assertIn("exec:return", driver.events)
        self.assertIsNotNone(rec)

    def test_retreat_submits_when_vacuum_is_already_off(self):
        driver = _Flow(attached=False)
        driver._payload_detached = True
        trial = PlaceTrial(index=0)
        ok, _code, _rec = driver._execute_segment(_motion("retreat"), trial)
        self.assertTrue(ok)
        self.assertIn("exec:retreat", driver.events)

    def test_fail_reason_after_detach_refuses_return_before_settle(self):
        driver = _Flow(attached=True, detach_reason="seal_lost")
        driver._selected_carry_v1 = _route()
        driver._selected_carry_v2 = _route_v2()
        trial = PlaceTrial(index=0)
        result = driver.run_place_from_carry(None, trial, *_slot())
        self.assertTrue(str(result.fail_code).startswith("VACUUM_DETACH"))
        self.assertNotEqual(result.fail_code, "PLACE_LOST_PAYLOAD")
        self.assertNotIn("settle", driver.events)
        self.assertNotIn("exec:return", driver.events)
        self.assertIsNone(result.err_xy)

    def test_return_completes_before_settle_and_verify_fields(self):
        driver = _Flow(attached=True)
        driver._selected_carry_v1 = _route()
        driver._selected_carry_v2 = _route_v2()
        trial = PlaceTrial(index=0)
        result = driver.run_place_from_carry(None, trial, *_slot())
        self.assertEqual(result.fail_code, "", result.fail_code)
        submitted = [item for item in driver.events if item.startswith("exec:")]
        self.assertEqual(submitted, [
            "exec:transit", "exec:insert", "exec:descend",
            "exec:retreat", "exec:return",
        ])
        settle_at = driver.events.index("settle")
        return_at = driver.events.index("exec:return")
        self.assertGreater(settle_at, return_at)
        self.assertIsNotNone(result.err_xy)
        self.assertIsNotNone(result.err_z)
        self.assertIsNotNone(result.roll)
        self.assertIsNotNone(result.pitch)
        self.assertIsNotNone(result.drift)
        self.assertIs(result.inside_inner_box, True)
        self.assertLess(result.err_xy, 0.04)
        self.assertLess(abs(result.err_z), 0.03)


if __name__ == "__main__":
    unittest.main()
