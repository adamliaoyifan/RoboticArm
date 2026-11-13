#!/usr/bin/env python3
"""joint_path execution does not call OMPL."""

from types import SimpleNamespace
import unittest

from luggage_planning.motion.motion_executor import MotionExecutor, cartesian_hops


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


class TestGenerationBypassRemoved(unittest.TestCase):
    """[ST09-I] The executor is a pure executor.

    Cartesian/pose_target segment specs were the legacy generation
    bypass (GetCartesianPath solves, OMPL fallback, seeded re-solves,
    piecewise/align retries). Production motion now arrives as
    certified joint_path route legs; a generation-type segment is
    refused by name with zero execution side effects.
    """

    def _executor(self):
        executor = object.__new__(MotionExecutor)
        executor._occupancy = None
        executor._min_fraction = 0.95
        executor._align_retry_enabled = False
        executor.executed = []
        executor._planned_traj = None
        executor._cartesian_excursion = None
        executor._wrist_measurement = None
        executor._j6_measurement = None
        executor._cartesian_fallback_record = None
        executor._occupancy_object_count = 0

        def _execute(*_args, **_kwargs):
            executor.executed.append("submitted")
            raise AssertionError("generation segment must not submit")

        executor._execute_trajectory = _execute
        executor._execute_robot_trajectory = _execute
        return executor

    def test_cartesian_segment_is_refused_without_submission(self):
        executor = self._executor()
        segment = SimpleNamespace(
            type="cartesian", name="insert", allow_ompl_fallback=True,
            required_cartesian_fraction=0.9, target_pose=_Pose(),
            waypoints=[], keep_tool_down=True)
        result = executor._execute_segment_inner(segment, None, 5.0, None)
        self.assertFalse(result.success)
        self.assertIn("SEGMENT_GENERATION_REMOVED", result.message)
        self.assertEqual(executor.executed, [])

    def test_pose_target_segment_is_refused_without_submission(self):
        executor = self._executor()
        segment = SimpleNamespace(
            type="pose_target", name="transit",
            allow_ompl_fallback=True, target_pose=_Pose(),
            waypoints=[], keep_tool_down=True)
        result = executor._execute_segment_inner(segment, None, 5.0, None)
        self.assertFalse(result.success)
        self.assertIn("SEGMENT_GENERATION_REMOVED", result.message)
        self.assertEqual(executor.executed, [])

    def test_generation_methods_are_gone(self):
        for name in ("_plan_cartesian", "_run_pose_target", "_ik_joints",
                     "_query_ik", "_align_then_retry",
                     "_cartesian_refine_fallback", "_piecewise_cartesian",
                     "_execute_seeded_hop", "_tool_down_constraint",
                     "_build_goal_constraints"):
            self.assertFalse(hasattr(MotionExecutor, name), name)

    def test_generation_clients_are_gone(self):
        executor = self._executor()
        self.assertFalse(hasattr(executor, "_cartesian"))
        self.assertFalse(hasattr(executor, "_ik"))

    def test_probe_refuses_generation_types(self):
        executor = self._executor()
        executor._reset_boundary_measurements = lambda: None
        executor._wrist_check = lambda *_a, **_k: None
        executor._j6_check = lambda *_a, **_k: None
        record = executor.probe_segment(SimpleNamespace(
            type="cartesian", name="descend", target_pose=_Pose()), None)
        self.assertIn("SEGMENT_GENERATION_REMOVED", record["refused"])


class CartesianHopsGeometryTest(unittest.TestCase):
    """The pure hop-splitting geometry stays available for diagnostics."""

    def test_hops_split_long_line(self):
        # Intermediate hop points only; the endpoints belong to the caller.
        hops = cartesian_hops((0.0, 0.0, 1.0), (0.9, 0.0, 1.0), step_m=0.30)
        self.assertEqual(hops[0], [0.3, 0.0, 1.0])
        self.assertEqual(hops[-1], [0.9, 0.0, 1.0])
        self.assertTrue(all(
            ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2
             + (b[2] - a[2]) ** 2) ** 0.5 <= 0.30 + 1e-9
            for a, b in zip(hops, hops[1:])))

    def test_short_line_is_one_hop(self):
        self.assertEqual(
            cartesian_hops((0.0, 0.0, 1.0), (0.1, 0.0, 1.0), step_m=0.30),
            [[0.1, 0.0, 1.0]])


if __name__ == "__main__":
    unittest.main()
