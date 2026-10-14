"""No certified admission must not crash the typed execution branch.

The pack driver's SelectPlacePathV2 response carries no
ValidateSelectedRoute record, so ``_execution_admission`` is None on the
live pack path. ``_execute_segment`` used to call
``leg_admission.get(...)`` unguarded and every successful route died as
``DRIVER_EXC_AttributeError``. These tests pin the no-admission
semantics: no crash, the carrying checks fall back to the segment
contract, and a lost vacuum still refuses the leg by name before any
controller submission.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), ".."))

from place_smoke_driver import CANONICAL_ARM_JOINTS, PlaceSmokeDriver  # noqa: E402

from luggage_gazebo.place_metrics import PlaceTrial  # noqa: E402
from luggage_msgs.msg import MotionSegment  # noqa: E402


class _Log(object):
    def info(self, *_args, **_kwargs):
        return None

    def error(self, *_args, **_kwargs):
        return None


class _Args(object):
    use_vacuum = True
    plan_only = False
    plan_timeout = 5.0


class _Typed(object):
    def __init__(self):
        self.submitted = []
        self.admissions = []
        self.aborts = []

    def plan_motion(self, motion_segment, timeout_sec, name="", admission=None,
                    should_abort=None):
        self.submitted.append(str(motion_segment.name))
        self.admissions.append(admission)
        self.aborts.append(should_abort)
        result = type("Result", (), {
            "fraction": 1.0, "used_ompl_fallback": False,
            "moveit_error_code": 1, "settle_json": "", "success": True,
        })()
        return True, "timed joint path", result


def _knots(first, last):
    return {
        "joint_names": list(CANONICAL_ARM_JOINTS),
        "points": [{"positions": list(first)}, {"positions": list(last)}],
    }


TARGET = [1.745, -1.732, -1.454, 4.757, 1.577, -1.414]


class _Leg(object):
    def __init__(self, name, first, last):
        self.name = name
        self.joint_trajectory = _knots(first, last)
        self.scene_revision = "rev"


def _segment(name):
    segment = MotionSegment()
    segment.name = name
    segment.type = "joint_path"
    segment.keep_tool_down = True
    segment.target_pose.orientation.w = 1.0
    return segment


def _driver(legs, vacuum_state):
    driver = type("Driver", (), {})()
    driver._execute_segment = PlaceSmokeDriver._execute_segment.__get__(driver)
    driver._args = _Args()
    driver._typed = _Typed()
    driver._vacuum_state = dict(vacuum_state)
    driver._payload_detached = False
    driver._segments_log = []
    driver._frame_motion_segments = list(legs)
    driver._frame_motion_index = 0
    driver._joint_ring = None
    driver._last_joint_state = None
    driver._handoff_policy = None
    driver._last_submitted_leg = None
    driver._handoff_failure_record = None
    driver._admission_check_client = None
    driver.ros_now_sec = lambda: 5.0
    driver.get_logger = lambda: _Log()
    driver._payload_attachment_gate = (
        PlaceSmokeDriver._payload_attachment_gate.__get__(driver))
    driver._check_i1 = lambda: (True, "")
    driver._peek_leg_load_state = (
        PlaceSmokeDriver._peek_leg_load_state.__get__(driver))
    driver._leg_execution_admission = (
        PlaceSmokeDriver._leg_execution_admission.__get__(driver))
    driver._carrying_vacuum_holds = (
        PlaceSmokeDriver._carrying_vacuum_holds.__get__(driver))
    driver._await_leg_handoff = lambda segment, name, admission, trial: (
        (True, "") if driver._last_submitted_leg is None
        else (True, ""))
    driver._sample_tf_box = lambda label="", query_gz=False: {
        "suction": None, "box_gz": None,
        "vacuum": dict(driver._vacuum_state)}
    driver._joint_ring_horizon_sec = 20.0
    driver._segment_exec_name = None
    driver._segment_exec_t0 = None

    def _note(motion_segment, name, ok, message=None):
        return None

    driver._note_leg_submitted = _note
    driver._record_submitted_leg = (
        lambda trial, motion_segment, name, ok, message, times: None)
    # The regression: no certified admission scope at all.
    driver._execution_admission = None
    return driver


def _feeding_driver(legs, vacuum_state):
    """First leg has no handoff to wait for (no submitted leg before it)."""
    driver = _driver(legs, vacuum_state)
    driver._handoff_policy = type("Policy", (), {
        "timeout_sec": 0.2, "hold_sec": 0.02,
        "max_sample_gap_sec": 5.0, "max_feedback_age_sec": 5.0})()
    return driver


class TestNoAdmissionTypedExecution(unittest.TestCase):
    def test_carrying_leg_without_admission_executes_and_aborts_armed(self):
        leg = _Leg("transit", TARGET, TARGET)
        driver = _feeding_driver([leg], {
            "attached": True, "vacuum_on": True, "fail_reason": ""})
        trial = PlaceTrial(index=0)
        ok, code, _rec = driver._execute_segment(_segment("transit"), trial)
        self.assertTrue(ok, code)
        self.assertEqual(driver._typed.submitted, ["transit"])
        # No admission claim is sent, and the mid-leg payload-loss abort
        # stays armed through the segment-contract fallback.
        self.assertIsNone(driver._typed.admissions[0])
        self.assertIsNotNone(driver._typed.aborts[0])

    def test_carrying_leg_without_admission_refuses_lost_vacuum(self):
        leg = _Leg("transit", TARGET, TARGET)
        driver = _feeding_driver([leg], {
            "attached": False, "vacuum_on": False,
            "fail_reason": "vacuum_released"})
        trial = PlaceTrial(index=0)
        ok, code, _rec = driver._execute_segment(_segment("transit"), trial)
        self.assertFalse(ok)
        self.assertEqual(code, "PAYLOAD_LOST:vacuum_released")
        self.assertEqual(driver._typed.submitted, [])

    def test_unloaded_leg_after_detach_needs_no_vacuum(self):
        leg = _Leg("retreat", TARGET, TARGET)
        driver = _feeding_driver([leg], {
            "attached": False, "vacuum_on": False, "fail_reason": ""})
        driver._payload_detached = True
        trial = PlaceTrial(index=0)
        ok, code, _rec = driver._execute_segment(_segment("retreat"), trial)
        self.assertTrue(ok, code)
        self.assertEqual(driver._typed.submitted, ["retreat"])
        self.assertIsNone(driver._typed.aborts[0])


if __name__ == "__main__":
    unittest.main()
