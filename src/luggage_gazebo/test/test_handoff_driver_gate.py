#!/usr/bin/env python3
"""The real ``_execute_segment`` typed branch gates leg N+1 on leg N arrival.

Each refusal must leave zero submissions of the next leg; success exactly
one. A background thread plays the joint-state feed into the driver ring with
real receive times. The policy is shortened for test speed only.
"""

import os
import sys
import threading
import time
import unittest

import rclpy

_SCRIPTS = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from collections import deque  # noqa: E402

from luggage_msgs.msg import MotionSegment  # noqa: E402

from luggage_gazebo.handoff_settle import HandoffPolicy  # noqa: E402
from luggage_gazebo.place_metrics import PlaceTrial  # noqa: E402
from place_smoke_driver import (  # noqa: E402
    CANONICAL_ARM_JOINTS,
    QP_START_TOLERANCE_RAD,
    PlaceSmokeDriver,
    trajectory_dump,
)

TARGET = [-2.218311, -1.212007, 1.87346, -2.237272, -1.567003, -0.64752]


def setUpModule():
    if not rclpy.ok():
        rclpy.init()


class _Log(object):
    def info(self, *_args, **_kwargs):
        return None


class _Args(object):
    use_vacuum = True
    plan_only = False
    plan_timeout = 5.0


class _Typed(object):
    """Records submissions; the arm "executes" instantly and succeeds."""

    def __init__(self):
        self.submitted = []
        self.timeouts = []

    def plan_motion(self, motion_segment, timeout_sec, name="", admission=None,
                    should_abort=None):
        self.submitted.append(str(motion_segment.name))
        self.timeouts.append(timeout_sec)
        result = type("Result", (), {
            "fraction": 1.0, "used_ompl_fallback": False,
            "moveit_error_code": 1, "settle_json": "", "success": True,
        })()
        return True, "timed joint path", result


def _knots(first, last):
    from types import SimpleNamespace as NS
    return NS(trajectory=NS(joint_names=list(CANONICAL_ARM_JOINTS), points=[
        NS(positions=list(q), velocities=[], time_from_start=NS(sec=t, nanosec=0))
        for q, t in ((first, 0), (last, 117))]))


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


def _offset(joint, value):
    values = list(TARGET)
    values[joint] += value
    return values


class _Driver(object):
    def _release_capture_event(self, event, **_k):
        pass  # prior-g1 provenance hook (env-gated no-op)

    def _stop_release_capture(self):
        pass

    _execute_segment = PlaceSmokeDriver._execute_segment
    _await_leg_handoff = PlaceSmokeDriver._await_leg_handoff
    _note_leg_submitted = PlaceSmokeDriver._note_leg_submitted
    _handoff_samples_reader = PlaceSmokeDriver._handoff_samples_reader
    _handoff_admission_reason = PlaceSmokeDriver._handoff_admission_reason
    _controller_state_window = PlaceSmokeDriver._controller_state_window
    _on_controller_state = PlaceSmokeDriver._on_controller_state
    _record_submitted_leg = PlaceSmokeDriver._record_submitted_leg
    _controller_point = staticmethod(PlaceSmokeDriver._controller_point)
    _on_joint_state = PlaceSmokeDriver._on_joint_state
    _peek_leg_load_state = PlaceSmokeDriver._peek_leg_load_state
    _payload_attachment_gate = PlaceSmokeDriver._payload_attachment_gate
    _leg_execution_admission = PlaceSmokeDriver._leg_execution_admission
    _carrying_vacuum_holds = PlaceSmokeDriver._carrying_vacuum_holds
    _check_i1 = PlaceSmokeDriver._check_i1
    _abort = PlaceSmokeDriver._abort
    CONTROLLER_STATE_TOPIC = PlaceSmokeDriver.CONTROLLER_STATE_TOPIC
    _CONTROLLER_STATE_HORIZON_SEC = 30.0
    _CONTROLLER_STATE_SUCCESS_ROWS = 30
    _HANDOFF_ADMISSION_POLL_SEC = 0.1
    _HANDOFF_ADMISSION_TIMEOUT_SEC = 0.5

    def __init__(self, legs, carrying=True, timeout=1.0):
        self._args = _Args()
        self._typed = _Typed()
        self._vacuum_state = {
            "attached": True, "vacuum_on": True, "fail_reason": ""}
        self._payload_detached = False
        self._segments_log = []
        self._timeline = []
        self._tf_trace = []
        self._place_state = "INIT"
        self._follow_skipped0 = 0
        self._frame_motion_segments = list(legs)
        self._frame_motion_index = 0
        self._segment_exec_name = None
        self._segment_exec_t0 = None
        self._joint_ring = deque()
        self._joint_ring_horizon_sec = 20.0
        self._last_joint_state = None
        self._handoff_policy = HandoffPolicy(
            timeout_sec=timeout, hold_sec=0.1, max_sample_gap_sec=0.2,
            max_feedback_age_sec=0.3)
        self._last_submitted_leg = None
        self._handoff_failure_record = None
        self._controller_state_ring = deque()
        self._controller_state_count = 0
        self._controller_state_import_error = ""
        self._admission_check_client = None
        self.admission_live = True
        self._execution_admission = {
            "route_digest": "route", "certification_id": "cert",
            "admission_map_revision": 7,
            "leg_digests": [leg.name for leg in legs],
            "leg_load_states": [
                "carrying" if carrying else "unloaded" for _ in legs],
        }
        self.admission_calls = 0

    # -- stubs the typed branch touches --------------------------------
    def _leg_trajectory_digest(self, segment):
        return segment.name

    def get_logger(self):
        return _Log()

    def ros_now_sec(self):
        return 5.0

    def _sample_tf_box(self, label="", query_gz=False):
        return {"suction": None, "box_gz": None,
                "vacuum": dict(self._vacuum_state)}

    def _set_state(self, new_state, guard="", note=""):
        self._place_state = new_state

    def _home_arm(self):
        return True, "ok", None

    def _follow_skipped(self):
        return 0

    def _dump_place(self, trial, extra_files=None, extra=None, debug=None):
        self.dumped = dict(extra or {})

    def create_client(self, *_args, **_kwargs):
        return object()

    def call_srv(self, _client, request, timeout):
        self.admission_calls += 1
        reason = "" if self.admission_live else "MAP_REVOKED"
        return type("Resp", (), {
            "live": self.admission_live, "reason_code": reason,
            "detail": ""})()


class _Feeder(object):
    """Plays /joint_states into the driver ring in real time, 100 Hz."""

    def __init__(self, driver, position, velocity=None, start_stamp=50.0):
        self.driver = driver
        self.position = position
        self.velocity = velocity or (lambda t: [0.0] * 6)
        self.start_stamp = start_stamp
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.t0 = None

    def start(self):
        self.t0 = time.monotonic()
        self.thread.start()
        return self

    def _run(self):
        k = 0
        while not self.stop.is_set():
            t = time.monotonic() - self.t0
            msg = type("Msg", (), {})()
            msg.name = list(CANONICAL_ARM_JOINTS)
            msg.position = self.position(t)
            vel = self.velocity(t)
            msg.velocity = [] if vel is None else vel
            msg.header = type("H", (), {})()
            msg.header.stamp = type("S", (), {})()
            stamp = self.start_stamp + t
            msg.header.stamp.sec = int(stamp)
            msg.header.stamp.nanosec = int((stamp - int(stamp)) * 1e9)
            self.driver._on_joint_state(msg)
            k += 1
            time.sleep(0.01)

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1.0)


def _two_legs(second_first=None):
    first = _Leg("transit", _offset(1, 0.3), TARGET)
    second = _Leg("traverse", second_first or TARGET, _offset(0, 0.4))
    return first, second


class TestHandoffGate(unittest.TestCase):
    def setUp(self):
        self.feeder = None

    def tearDown(self):
        if self.feeder is not None:
            self.feeder.close()

    def _run_two(self, driver, position, velocity=None):
        trial = PlaceTrial(index=0)
        ok1, code1, _ = driver._execute_segment(_segment("transit"), trial)
        self.assertTrue(ok1, code1)
        self.feeder = _Feeder(driver, position, velocity).start()
        ok2, code2, rec2 = driver._execute_segment(
            _segment("traverse"), trial)
        return trial, ok2, code2, rec2

    def test_first_leg_has_nothing_to_wait_for(self):
        driver = _Driver(_two_legs())
        trial = PlaceTrial(index=0)
        ok, code, _rec = driver._execute_segment(_segment("transit"), trial)
        self.assertTrue(ok, code)
        self.assertEqual(driver._typed.submitted, ["transit"])
        self.assertNotIn("handoffs", trial.extras)
        self.assertEqual(driver._typed.timeouts, [122.0])

    def test_settled_arm_submits_exactly_one_next_leg(self):
        driver = _Driver(_two_legs())
        trial, ok, code, rec = self._run_two(driver, lambda t: list(TARGET))
        self.assertTrue(ok, code)
        self.assertEqual(driver._typed.submitted, ["transit", "traverse"])
        handoff = trial.extras["handoffs"][0]
        self.assertEqual(handoff["status"], "ready")
        self.assertEqual(handoff["previous_leg"], "transit")
        self.assertEqual(handoff["next_leg"], "traverse")
        self.assertTrue(handoff["final_start_check"]["within_tolerance"])
        self.assertFalse(handoff["controller_state"]["available"])
        self.assertIn("no controller_state",
                      handoff["controller_state"]["unavailable_reason"])

    def test_creeping_arm_is_not_submitted_until_it_arrives(self):
        driver = _Driver(_two_legs())

        def creep(t):
            return _offset(1, -max(0.0, 0.3 - 0.5 * t))  # arrives at 0.6 s

        trial, ok, code, _rec = self._run_two(
            driver, creep, velocity=lambda t: [0.0, -0.0007, 0, 0, 0, 0])
        self.assertTrue(ok, code)
        self.assertEqual(driver._typed.submitted, ["transit", "traverse"])
        handoff = trial.extras["handoffs"][0]
        self.assertGreaterEqual(handoff["waited_sec"], 0.6)

    def test_never_converging_arm_submits_zero_and_reports_timeout(self):
        driver = _Driver(_two_legs(), timeout=0.6)
        trial, ok, code, rec = self._run_two(
            driver, lambda t: _offset(1, -0.05))
        self.assertFalse(ok)
        self.assertEqual(code, "HANDOFF_SETTLE_TIMEOUT")
        self.assertIsNone(rec)
        self.assertEqual(driver._typed.submitted, ["transit"])
        full = driver._handoff_failure_record
        self.assertEqual(full["status"], "timeout")
        self.assertGreater(len(full["samples"]), 10)
        self.assertEqual(full["policy"]["tolerance_rad"],
                         QP_START_TOLERANCE_RAD)
        self.assertFalse(full["controller_state"]["available"])

    def test_abort_writes_the_full_handoff_record(self):
        driver = _Driver(_two_legs(), timeout=0.5)
        trial, ok, code, _rec = self._run_two(
            driver, lambda t: _offset(1, -0.05))
        self.assertFalse(ok)
        driver._abort(trial, code, True, extra={"slot.json": {}})
        self.assertIn("handoff.json", driver.dumped)
        self.assertEqual(driver.dumped["handoff.json"]["code"], code)
        self.assertIn("slot.json", driver.dumped)
        self.assertIsNone(driver._handoff_failure_record)

    def test_endpoint_mismatch_is_not_a_timeout_and_submits_zero(self):
        driver = _Driver(_two_legs(second_first=_offset(2, 0.05)),
                         timeout=5.0)
        started = time.monotonic()
        trial, ok, code, _rec = self._run_two(
            driver, lambda t: list(TARGET))
        self.assertFalse(ok)
        self.assertEqual(code, "HANDOFF_ENDPOINT_MISMATCH")
        self.assertLess(time.monotonic() - started, 1.0)
        self.assertEqual(driver._typed.submitted, ["transit"])
        self.assertEqual(
            driver._handoff_failure_record["continuity"]["status"],
            "discontinuous")

    def test_vacuum_loss_during_the_wait_stops_the_handoff(self):
        driver = _Driver(_two_legs(), timeout=5.0)
        threading.Timer(0.25, lambda: driver._vacuum_state.update(
            attached=False, fail_reason="")).start()
        trial, ok, code, _rec = self._run_two(
            driver, lambda t: _offset(1, -0.05))
        self.assertFalse(ok)
        self.assertEqual(code, "PAYLOAD_LOST:vacuum_released")
        self.assertEqual(driver._typed.submitted, ["transit"])
        self.assertEqual(driver._handoff_failure_record["status"], "aborted")

    def test_admission_revoked_during_the_wait_stops_the_handoff(self):
        driver = _Driver(_two_legs(), timeout=5.0)
        threading.Timer(0.25, lambda: setattr(
            driver, "admission_live", False)).start()
        trial, ok, code, _rec = self._run_two(
            driver, lambda t: _offset(1, -0.05))
        self.assertFalse(ok)
        self.assertEqual(code, "HANDOFF_ADMISSION_MAP_REVOKED")
        self.assertEqual(driver._typed.submitted, ["transit"])
        self.assertGreaterEqual(driver.admission_calls, 1)

    def test_leg_without_a_timed_endpoint_is_refused_before_submission(self):
        first = _Leg("transit", _offset(1, 0.3), TARGET)
        second = _Leg("traverse", TARGET, TARGET)
        second.joint_trajectory = None
        driver = _Driver([first, second])
        trial, ok, code, _rec = self._run_two(
            driver, lambda t: _offset(1, -0.2))
        self.assertFalse(ok)
        self.assertIn("INVALID_CERTIFIED_TIMING", code)
        self.assertEqual(driver._typed.submitted, ["transit"])
        handoff = trial.extras["handoffs"][0]
        self.assertEqual(handoff["status"], "skipped")
        self.assertEqual(handoff["reason"], "no_timed_endpoint")
        self.assertFalse(handoff["next_first_knot_present"])

    def test_failed_previous_leg_leaves_nothing_to_wait_for(self):
        driver = _Driver(_two_legs())
        driver._note_leg_submitted(_two_legs()[0], "transit", False)
        self.assertIsNone(driver._last_submitted_leg)

    def test_controller_reference_is_captured_when_published(self):
        driver = _Driver(_two_legs())
        trial = PlaceTrial(index=0)
        ok1, code1, _ = driver._execute_segment(_segment("transit"), trial)
        self.assertTrue(ok1, code1)
        self.feeder = _Feeder(driver, lambda t: list(TARGET)).start()

        from control_msgs.msg import JointTrajectoryControllerState
        from trajectory_msgs.msg import JointTrajectoryPoint

        def publish():
            for k in range(40):
                # Real message type: field names are the humble ones.
                msg = JointTrajectoryControllerState()
                msg.joint_names = list(CANONICAL_ARM_JOINTS)
                for field, values in (
                        ("reference", TARGET), ("feedback", TARGET),
                        ("error", [0.0] * 6), ("output", TARGET)):
                    point = JointTrajectoryPoint()
                    point.positions = list(values)
                    setattr(msg, field, point)
                msg.header.stamp.sec = 50
                msg.header.stamp.nanosec = k * 1000000
                driver._on_controller_state(msg)
                time.sleep(0.01)

        thread = threading.Thread(target=publish)
        thread.start()
        ok2, code2, _rec = driver._execute_segment(
            _segment("traverse"), trial)
        thread.join()
        self.assertTrue(ok2, code2)
        window = trial.extras["handoffs"][0]["controller_state"]
        self.assertTrue(window["available"])
        self.assertEqual(window["rows"][0]["reference"], [
            round(value, 6) for value in TARGET])
        self.assertEqual(window["rows"][0]["error"], [0.0] * 6)


class TestSubmittedTrajectoryCapture(unittest.TestCase):
    """The V2 path writes no boundary dump; this is the only record."""

    def _message_leg(self):
        from luggage_msgs.msg import MotionSegmentV2
        from trajectory_msgs.msg import JointTrajectoryPoint

        segment = MotionSegmentV2()
        segment.name = "transit"
        trajectory = segment.joint_trajectory.trajectory
        trajectory.joint_names = list(CANONICAL_ARM_JOINTS)
        for k in range(5):
            point = JointTrajectoryPoint()
            point.positions = [0.1 * k] * 6
            point.velocities = [0.01 * k] * 6
            point.time_from_start.sec = k
            point.time_from_start.nanosec = 250000000
            trajectory.points.append(point)
        return segment

    def test_message_form_keeps_every_point_with_time_parameters(self):
        dump = trajectory_dump(self._message_leg())
        self.assertEqual(dump["n_points"], 5)
        self.assertFalse(dump["truncated"])
        self.assertEqual(dump["joint_names"], list(CANONICAL_ARM_JOINTS))
        self.assertEqual([p["t"] for p in dump["points"]],
                         [0.25, 1.25, 2.25, 3.25, 4.25])
        self.assertEqual(dump["points"][2]["q"], [0.2] * 6)
        self.assertEqual(dump["points"][2]["v"], [0.02] * 6)

    def test_absent_trajectory_is_none_not_empty(self):
        self.assertIsNone(trajectory_dump(None))
        leg = _Leg("x", TARGET, TARGET)
        leg.joint_trajectory = None
        self.assertIsNone(trajectory_dump(leg))

    def test_truncation_is_marked(self):
        dump = trajectory_dump(self._message_leg(), max_points=3)
        self.assertTrue(dump["truncated"])
        self.assertEqual(len(dump["points"]), 3)
        self.assertEqual(dump["n_points"], 5)

    def test_each_executed_leg_is_recorded_with_times(self):
        driver = _Driver(_two_legs())
        trial = PlaceTrial(index=0)
        ok, code, _ = driver._execute_segment(_segment("transit"), trial)
        self.assertTrue(ok, code)
        row = trial.extras["submitted_legs"][0]
        self.assertEqual(row["name"], "transit")
        self.assertTrue(row["ok"])
        self.assertLessEqual(row["submit_wall"], row["result_wall"])
        self.assertEqual(row["trajectory"]["n_points"], 2)


class TestControllerStateTopic(unittest.TestCase):
    def test_topic_is_the_one_with_a_live_publisher(self):
        # /S20/elfin_arm_controller/controller_state has no publisher
        # (handoff_verify01); the bag records the same topic the driver
        # subscribes to.
        from luggage_gazebo.capture_budget import bag_topics
        self.assertEqual(
            PlaceSmokeDriver.CONTROLLER_STATE_TOPIC,
            "/elfin_arm_controller/controller_state")
        self.assertIn(PlaceSmokeDriver.CONTROLLER_STATE_TOPIC, bag_topics())


class TestJointStateVelocityIngestion(unittest.TestCase):
    def _shell(self):
        return _Driver(_two_legs())

    def test_velocity_is_aligned_by_name(self):
        driver = self._shell()
        msg = type("Msg", (), {})()
        msg.name = ["elfin_joint1", "elfin_joint2", "elfin_joint5",
                    "elfin_joint3", "elfin_joint4", "elfin_joint6"]
        msg.position = [1.0, 2.0, 5.0, 3.0, 4.0, 6.0]
        msg.velocity = [0.1, 0.2, 0.5, 0.3, 0.4, 0.6]
        msg.header = type("H", (), {})()
        msg.header.stamp = type("S", (), {"sec": 1, "nanosec": 0})()
        driver._on_joint_state(msg)
        record = driver._last_joint_state
        self.assertEqual(record["position"], [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
        self.assertEqual(record["velocity"], [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        self.assertIn("rx_mono", record)

    def test_missing_velocity_stays_none_not_zero(self):
        driver = self._shell()
        msg = type("Msg", (), {})()
        msg.name = list(CANONICAL_ARM_JOINTS)
        msg.position = [0.0] * 6
        msg.velocity = []
        msg.header = type("H", (), {})()
        msg.header.stamp = type("S", (), {"sec": 1, "nanosec": 0})()
        driver._on_joint_state(msg)
        self.assertIsNone(driver._last_joint_state["velocity"])


if __name__ == "__main__":
    unittest.main()
