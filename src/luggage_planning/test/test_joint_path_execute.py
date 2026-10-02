#!/usr/bin/env python3
"""joint_path execution does not call OMPL."""

from types import SimpleNamespace
import unittest

from luggage_planning.motion_executor import MotionExecutor, cartesian_hops
from luggage_planning.place_selection_client import apply_carry_segment


def _executor():
    executor = object.__new__(MotionExecutor)
    executor._occupancy = None
    executor._occupancy_object_count = 0
    executor._joint_path_continuity = 1.2
    executor._planned_traj = None
    executor.executed = []

    def _execute(traj, name, _fraction, _feedback, _timeout):
        executor.executed.append(name)
        executor.n_points = len(traj.joint_trajectory.points)
        return executor._result(True, "ok", 1.0)

    executor._execute_robot_trajectory = _execute
    return executor


class _Pose(object):
    def __init__(self):
        self.position = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.orientation = SimpleNamespace()


class _Segment(object):
    def __init__(self):
        self.name = "traverse"
        self.type = "cartesian"
        self.allow_ompl_fallback = True
        self.target_pose = _Pose()
        self.waypoints = []


class _SegmentWithJointField(_Segment):
    """Stub mimicking a REBUILT MotionSegment (joint_waypoints installed).

    Humble generated messages expose get_fields_and_field_types() -> dict;
    the degradation gate must key off that dict, not a non-existent
    get_fields() -- otherwise joint_path stays downgraded forever.
    """

    def __init__(self):
        _Segment.__init__(self)
        self.joint_waypoints = []

    def get_fields_and_field_types(self):
        return {"joint_waypoints": "trajectory_msgs/JointTrajectoryPoint[]"}


class TestCarryHandoff(unittest.TestCase):
    def test_joint_path_downgrades_without_message_field(self):
        segment = _Segment()
        selection = {"segments": {"traverse": {
            "type": "joint_path",
            "waypoints": [(0.0, 0.0, 1.0), (0.2, 0.0, 1.0)],
            "joint_knots": [[0.0] * 6, [0.1] + [0.0] * 5],
            "allow_ompl_fallback": False,
        }}}
        apply_carry_segment(segment, selection)
        self.assertEqual(segment.type, "cartesian")
        self.assertTrue(segment.allow_ompl_fallback)
        self.assertAlmostEqual(segment.target_pose.position.x, 0.2)

    def test_joint_path_survives_with_message_field(self):
        segment = _SegmentWithJointField()
        selection = {"segments": {"traverse": {
            "type": "joint_path",
            "waypoints": [(0.0, 0.0, 1.0), (0.2, 0.0, 1.0)],
            "joint_knots": [[0.0] * 6, [0.1] + [0.0] * 5],
            "allow_ompl_fallback": False,
        }}}
        apply_carry_segment(segment, selection)
        self.assertEqual(segment.type, "joint_path")
        self.assertFalse(segment.allow_ompl_fallback)
        self.assertEqual(segment._joint_knots, [[0.0] * 6, [0.1] + [0.0] * 5])

    def test_waypoints_without_spec_become_cartesian(self):
        segment = _Segment()
        segment.name = "transit"
        segment.type = "pose_target"
        segment.allow_ompl_fallback = False
        apply_carry_segment(segment, {
            "waypoint_frame": "world",
            "waypoints": [(0.86, -0.38, 1.5), (1.005, -0.535, 0.827)],
        })
        self.assertEqual(segment.type, "cartesian")
        self.assertTrue(segment.allow_ompl_fallback)
        self.assertAlmostEqual(segment.target_pose.position.x, 1.005)
        self.assertEqual(len(segment.waypoints), 1)
        self.assertAlmostEqual(segment.waypoints[0].position.x, 0.86)


class TestJointPathExecute(unittest.TestCase):
    def test_dense_path_executes_without_ompl(self):
        executor = _executor()
        segment = SimpleNamespace(
            type="joint_path", name="traverse", joint_waypoints=[],
            _joint_knots=[[0.0] * 6, [0.05, 0.0, 0.0, 0.0, 0.0, 0.0]])
        result = executor._run_joint_path(segment, None, 5.0)
        self.assertTrue(result.success)
        self.assertFalse(result.used_ompl_fallback)
        self.assertEqual(executor.executed, ["traverse"])
        self.assertGreater(executor.n_points, 2)

    def test_branch_jump_does_not_execute(self):
        executor = _executor()
        segment = SimpleNamespace(
            type="joint_path", name="transit", joint_waypoints=[],
            _joint_knots=[[0.0] * 6, [3.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
        result = executor._run_joint_path(segment, None, 5.0)
        self.assertFalse(result.success)
        self.assertIn("joint_branch_jump", result.message)
        self.assertIn("PLACE_PATH_INFEASIBLE", result.message)
        self.assertEqual(executor.executed, [])


def _excursion_plan(joint="elfin_joint6"):
    return SimpleNamespace(
        excursion_rejected=True, occupancy_blocked=False,
        reason="cartesian joint excursion 14.143 rad/m")


class TestWristAlignThenRetry(unittest.TestCase):
    """Wrist-only excursion on a descent pays the yaw in place first.

    The 2026-09-29 v3 trial1 signature: carry arrives 0.837 rad of j6 off
    the insert heading; the descent refuses (unwind), the joint fallback
    refuses (branch jump) -- the align-then-retry step rotates at the
    current EE position onto the goal orientation and re-runs the descent.
    """

    def _executor(self, joint="elfin_joint6", align_ok=True, retry_plan=None):
        executor = object.__new__(MotionExecutor)
        executor._occupancy = None
        executor._occupancy_object_count = 0
        executor._joint_path_continuity = 1.2
        executor._excursion_limit = 3.0
        executor._min_fraction = 0.95
        executor._planned_traj = None
        executor._cartesian_joint_fallback = None
        executor._cartesian_fallback_record = None
        executor._align_retry_record = None
        executor._align_retry_enabled = True
        executor._cartesian_excursion = {
            "joint": joint, "excursion_rad": 0.837}
        executor.calls = []

        def _fk(names, joints):
            executor.calls.append("fk")
            return (1.2, -0.4, 1.3)

        executor._fk_xyz = _fk

        def _pose_target(segment, fb, timeout, joints):
            executor.calls.append("align:%s" % segment.name)
            return executor._result(align_ok, "align ok" if align_ok
                                    else "align failed")

        executor._run_pose_target = _pose_target
        plans = iter([(0.27, _excursion_plan(joint)),
                      retry_plan if retry_plan is not None
                      else (1.0, SimpleNamespace(
                          excursion_rejected=False, occupancy_blocked=False))])
        executor._plan_cartesian = lambda seg, fb: next(plans)
        executor.executed = []
        executor.n_points = 0

        def _execute(traj, name, _fraction, _feedback, _timeout):
            executor.executed.append(name)
            executor.n_points = len(traj.joint_trajectory.points)
            return executor._result(True, "ok", 1.0)

        executor._execute_robot_trajectory = _execute

        def _execute_traj(plan, seg, frac, fb, timeout):
            executor.executed.append("descent")
            return executor._result(True, "ok", frac)

        executor._execute_trajectory = _execute_traj
        executor._piecewise_cartesian_retry = False
        executor._execute_seeded_hop = (
            lambda *args, **kwargs: executor._result(False, "hop skipped", 0.0))
        return executor

    def _segment(self, name="insert"):
        segment = _Segment()
        segment.name = name
        segment.allow_ompl_fallback = False
        segment.keep_tool_down = True
        return segment

    def test_wrist_excursion_aligns_then_retries(self):
        executor = self._executor()
        result = executor._execute_segment_inner(
            self._segment(), None, 5.0, [0.0] * 6)
        self.assertTrue(result.success)
        self.assertIn("align-then-retry", result.message)
        self.assertEqual(executor.calls[0], "fk")
        self.assertEqual(executor.calls[1], "align:wrist_align")
        self.assertEqual(executor.executed, ["descent"])
        record = executor._align_retry_record
        self.assertTrue(record["align_ok"] and record["retry_ok"])

    def test_arm_joint_excursion_does_not_align(self):
        executor = self._executor(joint="elfin_joint2")
        result = executor._execute_segment_inner(
            self._segment(), None, 5.0, [0.0] * 6)
        self.assertFalse(result.success)
        self.assertIn("PLACE_PATH_EXCURSION", result.message)
        self.assertFalse(any(str(call).startswith("align") for call in executor.calls))
        self.assertIsNone(executor._align_retry_record)

    def test_failed_align_keeps_original_code(self):
        executor = self._executor(align_ok=False)
        result = executor._execute_segment_inner(
            self._segment(), None, 5.0, [0.0] * 6)
        self.assertFalse(result.success)
        self.assertIn("PLACE_PATH_EXCURSION", result.message)
        self.assertEqual(
            executor._align_retry_record["reason"], "align_failed")

    def test_non_descent_segment_does_not_align(self):
        executor = self._executor()
        result = executor._execute_segment_inner(
            self._segment(name="retreat"), None, 5.0, [0.0] * 6)
        self.assertFalse(result.success)  # no align hook for retreat
        self.assertFalse(any(str(call).startswith("align") for call in executor.calls))


class TestCartesianRefineFallback(unittest.TestCase):
    """Rejected no-fallback cartesian legs re-solve deterministically.

    The 2026-09-27 n=3 insert failure class: GetCartesianPath drifts to a
    different IK branch, stalls (fraction 0.244) and the excursion gate
    refuses -- with OMPL off there was no exit. The fallback re-solves the
    goal offline from the live joints and must clear the SAME excursion
    bar before executing; every failure keeps the original code.
    """

    def _executor(self, provider):
        executor = object.__new__(MotionExecutor)
        executor._occupancy = None
        executor._occupancy_object_count = 0
        executor._joint_path_continuity = 1.2
        executor._excursion_limit = 3.0
        executor._min_fraction = 0.95
        executor._planned_traj = None
        executor._cartesian_joint_fallback = provider
        executor._cartesian_fallback_record = None
        executor._align_retry_enabled = False  # these tests target the
        executor._cartesian_excursion = None   # joint fallback directly
        executor._piecewise_cartesian_retry = False
        executor._fk_xyz = lambda *_args, **_kwargs: None
        executor.executed = []

        def _execute(traj, name, _fraction, _feedback, _timeout):
            executor.executed.append(name)
            executor.n_points = len(traj.joint_trajectory.points)
            return executor._result(True, "ok", 1.0)

        executor._execute_robot_trajectory = _execute
        return executor

    def _run(self, executor):
        executor._plan_cartesian = lambda seg, fb: (0.244, _excursion_plan())
        segment = _Segment()
        segment.name = "insert"
        segment.allow_ompl_fallback = False  # insert/descend/retreat class
        return executor._execute_segment_inner(segment, None, 5.0, [0.0] * 6)

    def test_excursion_rejection_executes_joint_fallback(self):
        executor = self._executor(lambda seg, live: {
            "knots": [[0.0] * 6, [0.05, 0.0, 0.0, 0.0, 0.0, 0.0]],
            "fk_path_len": 0.10})
        result = self._run(executor)
        self.assertTrue(result.success)
        self.assertIn("cartesian joint fallback", result.message)
        self.assertFalse(result.used_ompl_fallback)
        self.assertEqual(executor.executed, ["insert"])
        self.assertGreater(executor.n_points, 2)
        record = executor._cartesian_fallback_record
        self.assertTrue(record["used"])
        self.assertEqual(record["rejected_for"],
                         "cartesian joint excursion 14.143 rad/m")

    def test_occupancy_skips_ompl_on_carry_excursion(self):
        """Occupancy-injected transit must not pose_target fly-through the hull."""
        ompl = []
        hops = []
        executor = self._executor(lambda seg, live: None)
        executor._occupancy = object()
        executor._notify = lambda *args, **kwargs: None
        executor._run_pose_target = (
            lambda *args, **kwargs: ompl.append("ompl") or executor._result(
                False, "PLACE_PATH_INFEASIBLE outside_hull", 1.0))
        executor._piecewise_cartesian = (
            lambda *args, **kwargs: hops.append("piecewise") or executor._result(
                True, "piecewise portal hops", 1.0))
        executor._plan_cartesian = lambda seg, fb: (1.0, _excursion_plan())
        segment = _Segment()
        segment.name = "transit"
        segment.allow_ompl_fallback = True
        result = executor._execute_segment_inner(
            segment, None, 5.0, [0.0] * 6)
        self.assertEqual(ompl, [])
        self.assertEqual(hops, ["piecewise"])
        self.assertTrue(result.success)
        self.assertFalse(result.used_ompl_fallback)

    def test_refine_failure_keeps_original_code(self):
        executor = self._executor(lambda seg, live: None)
        result = self._run(executor)
        self.assertFalse(result.success)
        self.assertIn("PLACE_PATH_EXCURSION", result.message)
        self.assertEqual(executor.executed, [])
        self.assertEqual(
            executor._cartesian_fallback_record["reason"], "refine_failed")

    def test_branch_jump_fallback_is_fail_closed(self):
        executor = self._executor(lambda seg, live: {
            "knots": [[0.0] * 6, [3.0, 0.0, 0.0, 0.0, 0.0, 0.0]],
            "fk_path_len": 0.10})
        result = self._run(executor)
        self.assertFalse(result.success)
        self.assertIn("PLACE_PATH_EXCURSION", result.message)
        self.assertEqual(executor.executed, [])
        self.assertIn("joint_branch_jump",
                      executor._cartesian_fallback_record["reason"])

    def test_excursion_recheck_rejects_unwind(self):
        # 1.0 rad of joint1 over a 0.10 m path = 10 rad/m > 3.0: the
        # fallback must not legitimise the unwind the gate just refused.
        executor = self._executor(lambda seg, live: {
            "knots": [[0.0] * 6, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]],
            "fk_path_len": 0.10})
        result = self._run(executor)
        self.assertFalse(result.success)
        self.assertIn("PLACE_PATH_EXCURSION", result.message)
        self.assertEqual(executor.executed, [])
        self.assertEqual(
            executor._cartesian_fallback_record["reason"],
            "excursion_recheck_joint1")


class CartesianOccupancyRetryTest(unittest.TestCase):
    def test_transit_retries_cartesian_after_hull_sweep(self):
        executor = object.__new__(MotionExecutor)
        executor._occupancy = object()
        executor._occupancy_object_count = 0
        executor._occupancy_cartesian_retry = False
        executor._start_joints = [0.0] * 6
        calls = []

        def _inner(segment, _fb, _timeout, _joints):
            calls.append(str(segment.type))
            self.assertEqual(segment.name, "transit")
            return executor._result(True, "cartesian ok", 1.0)

        executor._execute_segment_inner = _inner
        segment = _Segment()
        segment.name = "transit"
        segment.type = "pose_target"
        result = executor._cartesian_after_occupancy(
            segment, "outside_hull", None, 5.0)
        self.assertTrue(result.success)
        self.assertIn("cartesian after occupancy outside_hull", result.message)
        self.assertEqual(calls, ["cartesian"])
        self.assertEqual(segment.type, "pose_target")

    def test_non_transit_does_not_retry(self):
        executor = object.__new__(MotionExecutor)
        segment = _Segment()
        segment.name = "insert"
        segment.type = "pose_target"
        self.assertIsNone(executor._cartesian_after_occupancy(
            segment, "outside_hull", None, 5.0))


class PiecewiseCartesianTest(unittest.TestCase):
    def test_hops_split_long_line(self):
        hops = cartesian_hops((-1.0, 0.0, 1.46), (0.86, -0.38, 1.5), 0.30)
        self.assertGreaterEqual(len(hops), 5)
        self.assertAlmostEqual(hops[-1][0], 0.86)
        self.assertAlmostEqual(hops[-1][1], -0.38)

    def test_short_line_is_one_hop(self):
        hops = cartesian_hops((0.0, 0.0, 1.0), (0.10, 0.0, 1.0), 0.30)
        self.assertEqual(len(hops), 1)

    def test_short_retreat_is_one_seeded_hop(self):
        executor = object.__new__(MotionExecutor)
        executor._occupancy = None
        executor._occupancy_object_count = 0
        executor._piecewise_cartesian_retry = False
        executor._planned_traj = None
        executor._fk_xyz = lambda _names, _joints: (0.2, 0.0, 1.0)
        targets = []

        def _hop(segment, _joints, _fb, _timeout):
            targets.append(float(segment.target_pose.position.z))
            return executor._result(True, "hop ok", 1.0)

        executor._execute_seeded_hop = _hop
        segment = _Segment()
        segment.name = "retreat"
        segment.target_pose.position.x = 0.2
        segment.target_pose.position.y = 0.0
        segment.target_pose.position.z = 1.055
        result = executor._piecewise_cartesian(
            segment, None, 5.0, [0.0] * 6, "excursion", 1.0)
        self.assertTrue(result.success)
        self.assertEqual(targets, [1.055])

    def test_wrist_singularity_splits_before_giving_up(self):
        executor = object.__new__(MotionExecutor)
        executor._occupancy = None
        executor._occupancy_object_count = 0
        executor._piecewise_cartesian_retry = False
        executor._planned_traj = None
        executor._fk_xyz = lambda _names, _joints: (0.0, 0.0, 1.0)
        calls = []

        def _hop(segment, _joints, _fb, _timeout):
            calls.append(float(segment.target_pose.position.z))
            if len(calls) == 1:
                return executor._result(
                    False, "wrist_singularity margin=0.100 crosses=True", 0.0)
            return executor._result(True, "hop ok", 1.0)

        executor._execute_seeded_hop = _hop
        segment = _Segment()
        segment.name = "place_exit"
        segment.target_pose.position.x = 0.0
        segment.target_pose.position.y = 0.0
        segment.target_pose.position.z = 1.40
        result = executor._piecewise_cartesian(
            segment, None, 5.0, [0.0] * 6, "wrist", 1.0)
        self.assertTrue(result.success)
        self.assertGreater(len(calls), 1)

    def test_transit_piecewise_restores_target(self):
        executor = object.__new__(MotionExecutor)
        executor._occupancy = None
        executor._occupancy_object_count = 0
        executor._piecewise_cartesian_retry = False
        executor._planned_traj = None
        executor._fk_xyz = lambda _names, _joints: (-1.0, 0.0, 1.46)
        targets = []

        def _hop(segment, _joints, _fb, _timeout):
            targets.append((
                float(segment.target_pose.position.x),
                float(segment.target_pose.position.y),
                float(segment.target_pose.position.z)))
            return executor._result(True, "hop ok", 1.0)

        executor._execute_seeded_hop = _hop
        segment = _Segment()
        segment.name = "transit"
        segment.target_pose.position.x = 0.86
        segment.target_pose.position.y = -0.38
        segment.target_pose.position.z = 1.5
        result = executor._piecewise_cartesian(
            segment, None, 5.0, [0.0] * 6, "excursion", 1.0)
        self.assertTrue(result.success)
        self.assertIn("piecewise portal hops after excursion", result.message)
        self.assertGreaterEqual(len(targets), 2)
        self.assertAlmostEqual(targets[-1][0], 0.86)
        self.assertAlmostEqual(segment.target_pose.position.x, 0.86)
        self.assertAlmostEqual(segment.target_pose.position.y, -0.38)

    def test_hops_retarget_admitted_fk_pose(self):
        from luggage_planning.motion_executor import _V2_ADMITTED
        from luggage_planning.motion_frame_contracts import AdmittedSegment

        executor = object.__new__(MotionExecutor)
        executor._occupancy = None
        executor._occupancy_object_count = 0
        executor._piecewise_cartesian_retry = False
        executor._planned_traj = None
        executor._fk_xyz = lambda _names, _joints: (-1.0, 0.0, 1.46)
        seen = []

        def _hop(segment, _joints, _fb, _timeout):
            current = _V2_ADMITTED.get()
            seen.append(list(current.target_pose["position"]))
            return executor._result(True, "hop ok", 1.0)

        executor._execute_seeded_hop = _hop
        segment = _Segment()
        segment.name = "transit"
        segment.target_pose.position.x = 0.86
        segment.target_pose.position.y = -0.38
        segment.target_pose.position.z = 1.5
        admitted = AdmittedSegment(
            segment_type="cartesian", name="transit",
            target_link="suction_contact_frame",
            request_id="req", scene_revision="scene", stamp_ns=0,
            time_basis="scene", keep_tool_down=True, keep_camera_down=False,
            lock_wrist=False, allow_ompl_fallback=False,
            required_cartesian_fraction=0.95,
            target_pose={"position": [0.86, -0.38, 1.5],
                         "quaternion_xyzw": [1.0, 0.0, 0.0, 0.0]},
            waypoints=(), joint_names=None, joint_points=None)
        token = _V2_ADMITTED.set(admitted)
        try:
            result = executor._piecewise_cartesian(
                segment, None, 5.0, [0.0] * 6, "excursion", 1.0)
        finally:
            _V2_ADMITTED.reset(token)
        self.assertTrue(result.success)
        self.assertGreaterEqual(len(seen), 2)
        self.assertNotAlmostEqual(seen[0][0], 0.86, places=2)
        self.assertAlmostEqual(seen[-1][0], 0.86)
        self.assertAlmostEqual(admitted.target_pose["position"][0], 0.86)
        self.assertAlmostEqual(admitted.target_pose["position"][1], -0.38)

    def test_seeded_hop_interpolates_without_movegroup(self):
        executor = object.__new__(MotionExecutor)
        executor._occupancy = None
        executor._occupancy_object_count = 0
        executor._joint_path_continuity = 1.2
        executor._planned_traj = None
        executor.fk_submissions = []
        executor._ik_joints = (
            lambda pose, joints, keep_tool_down=False: (
                [joints[0] + 0.1] + list(joints[1:])))
        executor._v2_fk_gate = lambda _positions: None
        executor._notify = lambda *args, **kwargs: None
        executed = []

        def _exec(traj, name, _frac, _fb, _timeout):
            executed.append((name, len(traj.joint_trajectory.points)))
            return executor._result(True, "ok", 1.0)

        executor._execute_robot_trajectory = _exec
        result = executor._execute_seeded_hop(
            _Segment(), [0.0, 0.0, 0.0, 0.0, 1.2, 0.0], None, 5.0)
        self.assertTrue(result.success, result.message)
        self.assertEqual(executed[0][0], "portal hop")
        self.assertGreater(executed[0][1], 2)

    def test_seeded_hop_prefers_offline_knots(self):
        executor = object.__new__(MotionExecutor)
        executor._occupancy = None
        executor._occupancy_object_count = 0
        executor._joint_path_continuity = 1.2
        executor._planned_traj = None
        executor.fk_submissions = []
        executor._v2_fk_gate = lambda _positions: None
        executor._notify = lambda *args, **kwargs: None
        executor._ik_joints = lambda *args, **kwargs: (_ for _ in ()).throw(
            AssertionError("KDL IK must not run when offline knots exist"))
        safe = [0.0, 0.0, 0.0, 0.0, 1.2, 0.0]
        executor._cartesian_joint_fallback = lambda _seg, _live: {
            "knots": [list(safe), [0.04, 0.0, 0.0, 0.0, 1.2, 0.0],
                      [0.08, 0.0, 0.0, 0.0, 1.2, 0.0]]}
        executed = []

        def _exec(traj, name, _frac, _fb, _timeout):
            executed.append((name, len(traj.joint_trajectory.points)))
            return executor._result(True, "ok", 1.0)

        executor._execute_robot_trajectory = _exec
        result = executor._execute_seeded_hop(
            _Segment(), [0.0, 0.0, 0.0, 0.0, 1.2, 0.0], None, 5.0)
        self.assertTrue(result.success, result.message)
        self.assertEqual(executed[0][0], "portal hop")
        self.assertGreater(executed[0][1], 3)


if __name__ == "__main__":
    unittest.main()
