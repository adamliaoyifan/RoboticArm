#!/usr/bin/env python3
"""MoveIt 2 client wrapper for one motion segment at a time.

Node-layer code (per docs/architecture/perception_architecture.md): it
imports moveit_msgs at the top level by design; algorithm modules must not
import it. Not a Node subclass - construct with an already-initialized node
(so the caller owns executors/callback groups) and call ``execute_segment``.

Segment routing (docs/plans/closed_loop_pick_retreat_nodes.md):

    pose_target  -> MoveGroup plan_only, then rest-to-rest ExecuteTrajectory
                    (MoveIt TOTG execute stays off; Huayan CollisionStop)
    cartesian    -> GetCartesianPath geometry, wrist/excursion gates,
                    occupancy sweep, then rest-to-rest ExecuteTrajectory
    joint_path   -> dense the selector's joint knots, FK-sweep occupancy,
                    then rest-to-rest ExecuteTrajectory. No RRTConnect.
    named pose   -> handled by the node via plain FJT (see motion_planner_node)

``keep_tool_down`` adds an orientation constraint (tool Z down).
``keep_camera_down`` / ``lock_wrist`` are NOT implemented on purpose; the
result message says so instead of silently ignoring the flags.
"""

from __future__ import division

import contextvars
import math
import threading
import time
from dataclasses import dataclass

_V2_ADMITTED = contextvars.ContextVar("motion_v2_admitted", default=None)

from luggage_planning.joint6_hold import (
    joint6_value,
    pick_holds_joint6,
    pin_joint6_trajectory,
)
from luggage_planning.motion_boundary import robot_traj_to_dict
from luggage_planning.ros_clock_wait import wait_event
from luggage_planning.trajectory_timing import time_parameterize_cartesian

from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import Pose as PoseMsg, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
    PositionIKRequest,
    RobotTrajectory,
)
from moveit_msgs.srv import GetCartesianPath, GetPositionFK, GetPositionIK
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint

# Tool-Z (suction normal) pointing at world -Z: quaternion (x=1, w=0).
TOOL_DOWN_QUAT = (1.0, 0.0, 0.0, 0.0)
JOINTS = ["elfin_joint1", "elfin_joint2", "elfin_joint3",
          "elfin_joint4", "elfin_joint5", "elfin_joint6"]
# Match S20 URDF revolute limits (±2π). Used only to pick the nearest
# equivalent wrap of an IK solution so OMPL does not command a 4 rad spin.
_JOINT_LIMIT = 6.28
_VEL_SCALE = 0.3
_ACC_SCALE = 0.3
# elfin joint limits are uniform (URDF velocity 1.57 rad/s, elfin_moveit_config
# joint_limits.yaml acceleration 3.14 rad/s^2).
_JOINT_VEL_LIMIT = 1.57
_JOINT_ACC_LIMIT = 3.14
# A cartesian solution may be smooth in joint space and still unwind the arm
# through a near-singular reconfiguration: motion_occ/2026-09-21_1838
# place_exit spent 6.58 rad of elfin_joint1 on a 1.39 m hop (4.72 rad/m,
# 19.3 s) and the open-loop controller missed the goal by 0.91 rad. Every
# legitimate cartesian segment in that run stayed at 0.93-1.50 rad/m, so the
# bound sits above the accepted set with 2x margin. jump_threshold does not
# catch this: the rejected path's largest adjacent step was 0.157 rad.
_CARTESIAN_EXCURSION_RAD_PER_M = 3.0
# Below this Cartesian length the ratio is dominated by its denominator (a
# 9 mm traverse reads 2.07 rad/m for 0.02 rad of motion), so short hops are
# judged against the floor instead.
_CARTESIAN_EXCURSION_MIN_PATH_M = 0.05
# j5 and j6 are not refuse gates. A keep_tool_down segment only requires
# the suction normal to stay within tool_down_abs_tol of world -Z; yaw
# about that axis is free, so either wrist branch may be used.
# wrist_margin and j6_rotation stay in the boundary dump.
_WRIST_JOINT_INDEX = 4
_WRIST_SINGULARITY_MARGIN_RAD = 0.50
_J6_JOINT_INDEX = 5
_J6_ROTATION_LIMIT_RAD = 2.50
_TOOL_DOWN_ABS_TOL = 0.05
# Diagnostic only: max per-joint |seed-goal|. Not a refuse gate.
_POSE_TARGET_MAX_JOINT_DELTA_RAD = 1.50
_WRIST_FREE_SEGMENTS_DEFAULT = ()


@dataclass
class SegmentExecResult:
    success: bool
    message: str
    fraction: float = 0.0
    used_ompl_fallback: bool = False
    moveit_error_code: int = 0
    occupancy_checked: bool = False
    occupancy_object_count: int = 0
    trajectory: object = None
    start_joints: object = None


class _OccupancyBlockedPlan(object):
    occupancy_blocked = True
    solution = None


class _ExcursionRejectedPlan(object):
    """A cartesian solution whose joint excursion is implausible.

    Carries the measurement so the caller reports why the path was refused
    instead of reporting it as a low fraction.
    """

    excursion_rejected = True
    solution = None

    def __init__(self, reason, measurement):
        self.reason = str(reason)
        self.measurement = dict(measurement)


def cartesian_excursion(joint_names, points, path_len_m,
                        limit_rad_per_m=_CARTESIAN_EXCURSION_RAD_PER_M,
                        min_path_m=_CARTESIAN_EXCURSION_MIN_PATH_M):
    """Joint excursion per metre of Cartesian travel for one solution.

    ``points`` are objects with ``positions`` (a JointTrajectory) and
    ``path_len_m`` is the straight-line length of the requested waypoint
    chain. Returns ``None`` when there is nothing to judge, otherwise a dict
    with the worst joint, its total variation, the ratio, and ``ok``.
    """
    rows = [list(getattr(p, "positions", p)) for p in (points or [])]
    rows = [r for r in rows if r]
    if len(rows) < 2 or path_len_m is None:
        return None
    width = min(len(r) for r in rows)
    if width <= 0:
        return None
    names = list(joint_names or JOINTS)
    totals = [0.0] * width
    for before, after in zip(rows, rows[1:]):
        for index in range(width):
            totals[index] += abs(float(after[index]) - float(before[index]))
    worst = max(range(width), key=lambda index: totals[index])
    denominator = max(float(path_len_m), float(min_path_m))
    ratio = totals[worst] / denominator if denominator > 0 else float("inf")
    return {
        "joint": names[worst] if worst < len(names) else "joint%d" % worst,
        "excursion_rad": round(totals[worst], 4),
        "path_len_m": round(float(path_len_m), 4),
        "rad_per_m": round(ratio, 3),
        "limit_rad_per_m": float(limit_rad_per_m),
        "ok": ratio <= float(limit_rad_per_m),
    }


def _wrist_cell(value):
    """Which [k*pi, (k+1)*pi) band joint5 sits in.

    Band boundaries are the degenerate configurations, so a change of cell
    between two consecutive waypoints means the solution reconfigured the
    wrist through a singularity rather than moving within one branch.
    """
    return math.floor(float(value) / math.pi)


def wrist_margin(points, margin_rad=_WRIST_SINGULARITY_MARGIN_RAD,
                 joint_index=_WRIST_JOINT_INDEX):
    """Closest approach of joint5 to k*pi, plus a branch-crossing flag.

    ``points`` are objects with ``positions`` (a JointTrajectory) or plain
    joint vectors. Returns ``None`` when there is nothing to judge, otherwise
    a dict with the closest approach to k*pi, whether the track crosses a
    branch boundary, and ``ok``. ``ok`` requires both staying at least
    ``margin_rad`` from every k*pi and not changing [k*pi, (k+1)*pi) cell:
    a two-point seed-to-goal pair parked at +/-pi/2 has a large closest
    approach but still reconfigures through the singularity.
    """
    rows = [list(getattr(p, "positions", p)) for p in (points or [])]
    track = [float(r[joint_index]) for r in rows if len(r) > joint_index]
    if len(track) < 2:
        return None
    margin = min(abs(v - round(v / math.pi) * math.pi) for v in track)
    crosses = any(
        _wrist_cell(a) != _wrist_cell(b) for a, b in zip(track, track[1:]))
    return {
        "joint": (JOINTS[joint_index] if joint_index < len(JOINTS)
                  else "joint%d" % joint_index),
        "margin_rad": round(margin, 4),
        "margin_limit_rad": float(margin_rad),
        "delta_rad": round(abs(track[-1] - track[0]), 4),
        "crosses_branch": bool(crosses),
        "ok": (not crosses) and margin >= float(margin_rad),
    }


# Name used in the first draft of this gate; tests and callers use wrist_margin.
wrist_singularity = wrist_margin


def j6_rotation(points, limit_rad=_J6_ROTATION_LIMIT_RAD,
                joint_index=_J6_JOINT_INDEX):
    """Total |d j6| along a track. Dump only; not a refuse gate."""
    rows = [list(getattr(p, "positions", p)) for p in (points or [])]
    track = [float(r[joint_index]) for r in rows if len(r) > joint_index]
    if len(track) < 2:
        return None
    travel = sum(abs(b - a) for a, b in zip(track, track[1:]))
    net = abs(track[-1] - track[0])
    return {
        "joint": (JOINTS[joint_index] if joint_index < len(JOINTS)
                  else "joint%d" % joint_index),
        "travel_rad": round(travel, 4),
        "net_rad": round(net, 4),
        "limit_rad": float(limit_rad),
        "ok": travel <= float(limit_rad),
    }


def pose_target_joint_delta(seed, goal,
                            limit_rad=_POSE_TARGET_MAX_JOINT_DELTA_RAD,
                            joint_names=None):
    """Max per-joint |delta| from seed to an IK goal.

    Dump diagnostic only. j5 and j6 travel are not refuse reasons.
    """
    if not seed or not goal or len(seed) != len(goal):
        return None
    names = list(joint_names or JOINTS)
    deltas = [abs(float(a) - float(b)) for a, b in zip(seed, goal)]
    worst = max(range(len(deltas)), key=lambda index: deltas[index])
    value = deltas[worst]
    return {
        "joint": names[worst] if worst < len(names) else "joint%d" % worst,
        "max_rad": round(value, 4),
        "limit_rad": float(limit_rad),
        "ok": value <= float(limit_rad),
    }


def _quat_mul(a, b):
    """Hamilton product of (x, y, z, w) tuples."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def equivalent_tool_down_pose(pose):
    """Same position, tool yaw + pi (rectangular payload is invariant).

    Rotate 180 deg about the tool Z axis so keep_tool_down IK can try both
    wrist branches without changing the payload heading.
    """
    q = (
        float(pose.orientation.x), float(pose.orientation.y),
        float(pose.orientation.z), float(pose.orientation.w),
    )
    x, y, z, w = _quat_mul(q, (0.0, 0.0, 1.0, 0.0))
    out = type(pose)()
    out.position.x = float(pose.position.x)
    out.position.y = float(pose.position.y)
    out.position.z = float(pose.position.z)
    out.orientation.x = x
    out.orientation.y = y
    out.orientation.z = z
    out.orientation.w = w
    return out


def _sec_to_duration(sec):
    nsec_total = int(round(float(sec) * 1e9))
    if nsec_total < 0:
        nsec_total = 0
    return DurationMsg(
        sec=nsec_total // 1000000000,
        nanosec=nsec_total % 1000000000,
    )


def apply_rest_to_rest_to_joint_trajectory(joint_trajectory, vel_scale, acc_scale):
    """Replace TOTG or linear times with the site rest-to-rest trapezoid.

    Keeps the geometric knots. Writes time_from_start, velocities, and
    accelerations capped to 60 deg/s and 20 deg/s^2 on both ramps.
    """
    points = list(joint_trajectory.points)
    if len(points) < 2:
        return None
    timed = time_parameterize_cartesian(
        [list(pt.positions) for pt in points], vel_scale, acc_scale)
    for point, t_s, vel, acc in zip(
            points, timed.times_s, timed.velocities_rad,
            timed.accelerations_rad):
        point.time_from_start = _sec_to_duration(t_s)
        point.velocities = list(vel)
        point.accelerations = list(acc)
    return timed


def _wrap_near(current, target, lower=-_JOINT_LIMIT, upper=_JOINT_LIMIT):
    """Choose target + k·2π inside limits that is closest to current."""
    best = target
    best_err = abs(target - current)
    for k in range(-3, 4):
        cand = target + k * 2.0 * math.pi
        if cand < lower - 1e-6 or cand > upper + 1e-6:
            continue
        err = abs(cand - current)
        if err < best_err:
            best, best_err = cand, err
    return best


class MotionExecutor:

    def __init__(self, node, group_name="elfin_arm",
                 link_name="suction_contact_frame",
                 planner_id="RRTConnect",
                 allowed_planning_time=5.0,
                 num_planning_attempts=10,
                 planning_frame="world",
                 cartesian_max_step=0.01,
                 cartesian_min_fraction=0.95,
                 cartesian_avoid_collisions=True,
                 cartesian_excursion_rad_per_m=_CARTESIAN_EXCURSION_RAD_PER_M,
                 wrist_singularity_margin_rad=_WRIST_SINGULARITY_MARGIN_RAD,
                 wrist_free_segments=_WRIST_FREE_SEGMENTS_DEFAULT,
                 pose_target_max_joint_delta_rad=_POSE_TARGET_MAX_JOINT_DELTA_RAD,
                 j6_rotation_limit_rad=_J6_ROTATION_LIMIT_RAD,
                 tool_down_abs_tol=_TOOL_DOWN_ABS_TOL,
                 atlas_seed_provider=None,
                 atlas_mode="miss-retry",
                 atlas_max_seeds=3,
                 atlas_fk_verify_tol_m=0.03,
                 atlas_submit_mode="reseed",
                 velocity_scaling=_VEL_SCALE,
                 acceleration_scaling=_ACC_SCALE,
                 joint_path_continuity_rad=1.2,
                 cartesian_joint_fallback=None,
                 align_retry_enabled=True,
                 fk_verifier=None):
        self._node = node
        self._group = str(group_name)
        self._link = str(link_name)
        self._planner_id = str(planner_id)
        self._planning_time = float(allowed_planning_time)
        self._attempts = int(num_planning_attempts)
        self._frame = str(planning_frame)
        self._max_step = float(cartesian_max_step)
        self._min_fraction = float(cartesian_min_fraction)
        self._avoid_collisions = bool(cartesian_avoid_collisions)
        self._excursion_limit = float(cartesian_excursion_rad_per_m)
        self._wrist_margin = float(wrist_singularity_margin_rad)
        if wrist_free_segments is None:
            wrist_free_segments = _WRIST_FREE_SEGMENTS_DEFAULT
        self._wrist_free = {str(name).strip()
                            for name in wrist_free_segments}
        self._wrist_free.discard("")
        self._pose_target_max_delta = float(pose_target_max_joint_delta_rad)
        self._j6_limit = float(j6_rotation_limit_rad)
        # Reachability-atlas seed chain (see luggage_planning.atlas_runtime):
        # miss-retry re-seeds KDL after an unseeded miss; always-first tries
        # seeds first; joint_goal additionally allows an FK-verified seed to
        # become the goal without an IK service hit.
        self._atlas_provider = atlas_seed_provider
        self._atlas_mode = str(atlas_mode or "miss-retry")
        self._atlas_max_seeds = int(atlas_max_seeds)
        self._atlas_fk_tol = float(atlas_fk_verify_tol_m)
        self._atlas_submit_mode = str(atlas_submit_mode or "reseed")
        self._vel_scale = float(velocity_scaling)
        self._acc_scale = float(acceleration_scaling)
        self._joint_path_continuity = float(joint_path_continuity_rad)
        # Deterministic re-solve for no-fallback cartesian segments rejected
        # by the excursion gate or a sub-threshold fraction (see
        # _cartesian_refine_fallback). Node-injected because the refine
        # needs the URDF/scene; the executor stays frame-agnostic.
        self._cartesian_joint_fallback = cartesian_joint_fallback
        self._align_retry_enabled = bool(align_retry_enabled)
        self._fk_verifier = fk_verifier
        self._v2_lock = threading.Lock()
        self.fk_submissions = []
        self._tool_down_abs_tol = float(tool_down_abs_tol)
        # Set per execute_segment call (goals are serialized: the stack
        # never runs two motion segments at once, move_group would refuse
        # the second). When the outer action goal is cancelled, inner
        # action handles are cancelled so the controller stops instead of
        #         grinding on past the caller's timeout.
        self._cancel_check = None
        self._last_ik_code = 0

        import rclpy
        self._rclpy = rclpy
        # MoveIt 2 Humble serves the MoveGroup action at /move_action
        # (node info on the move_group node); /move_group/action/move_group
        # has no server in this distro.
        self._move_group = rclpy.action.ActionClient(
            node, MoveGroup, "/move_action")
        self._execute = rclpy.action.ActionClient(
            node, ExecuteTrajectory, "/execute_trajectory")
        self._cartesian = node.create_client(
            GetCartesianPath, "/compute_cartesian_path")
        self._ik = node.create_client(GetPositionIK, "/compute_ik")
        self._fk = node.create_client(GetPositionFK, "/compute_fk")
        self._occupancy = None
        self._payload_wdh = None
        self._world_to_map = None
        self._occupancy_exempt = []
        self._arm_radius = 0.08
        self._inflate_m = 0.05
        self._occupancy_object_count = 0
        self._planned_traj = None
        self._start_joints = []
        self._last_boundary = {}
        self._cartesian_excursion = None
        self._wrist_measurement = None
        self._j6_measurement = None
        self._pose_target_joint_delta = None
        self._atlas_record = None

    # ------------------------------------------------------------------

    def wait_ready(self, timeout_sec=60.0):
        """True when the MoveGroup/ExecuteTrajectory servers and the
        cartesian service are all available."""
        ok = self._move_group.wait_for_server(timeout_sec=timeout_sec)
        if not ok:
            return False
        if not self._execute.wait_for_server(timeout_sec=5.0):
            return False
        if not self._cartesian.wait_for_service(timeout_sec=5.0):
            return False
        return self._ik.wait_for_service(timeout_sec=5.0)

    def set_occupancy(self, snapshot, payload_wdh=None, world_to_map=None,
                      arm_radius=None, inflate_m=None, object_count=0,
                      exempt_locals=None):
        """Pin cargo occupancy for the next segment. ``snapshot=None`` clears.

        ``exempt_locals`` are grid-local landing-footprint AABBs
        (``exempt_footprint_locals``): cargo cells inside them do not
        collide, so approach/insertion segments aimed at their own slot are
        not blocked by the slot's support and inflated neighbours.
        """
        self._occupancy = snapshot
        self._payload_wdh = None if payload_wdh is None else [
            float(v) for v in payload_wdh]
        self._world_to_map = world_to_map
        if arm_radius is not None:
            self._arm_radius = float(arm_radius)
        if inflate_m is not None:
            self._inflate_m = float(inflate_m)
        self._occupancy_object_count = int(object_count)
        self._occupancy_exempt = list(exempt_locals or [])

    def _result(self, success, message, fraction=0.0, used_ompl=False,
                error_code=0):
        return SegmentExecResult(
            bool(success), str(message), float(fraction), bool(used_ompl),
            int(error_code),
            occupancy_checked=self._occupancy is not None,
            occupancy_object_count=self._occupancy_object_count,
        )

    # ------------------------------------------------------------------

    def last_boundary(self):
        return dict(self._last_boundary or {})

    def _reset_boundary_measurements(self):
        self._cartesian_excursion = None
        self._wrist_measurement = None
        self._j6_measurement = None
        self._pose_target_joint_delta = None
        self._atlas_record = None
        self._cartesian_fallback_record = None
        self._align_retry_record = None

    def _stamp_boundary(self, result, name):
        traj = robot_traj_to_dict(self._planned_traj)
        result.trajectory = traj
        result.start_joints = list(self._start_joints or [])
        self._last_boundary = {
            "name": str(name or ""),
            "success": bool(result.success),
            "message": str(result.message),
            "fraction": float(result.fraction),
            "used_ompl_fallback": bool(result.used_ompl_fallback),
            "moveit_error_code": int(result.moveit_error_code),
            "occupancy_checked": bool(result.occupancy_checked),
            "occupancy_object_count": int(result.occupancy_object_count),
            "start_joints": list(self._start_joints or []),
            "trajectory": traj,
        }
        if self._cartesian_excursion is not None:
            self._last_boundary["cartesian_excursion"] = dict(
                self._cartesian_excursion)
        if self._wrist_measurement is not None:
            self._last_boundary["wrist_margin"] = dict(self._wrist_measurement)
        if self._j6_measurement is not None:
            self._last_boundary["j6_rotation"] = dict(self._j6_measurement)
        if self._pose_target_joint_delta is not None:
            self._last_boundary["pose_target_joint_delta"] = dict(
                self._pose_target_joint_delta)
        atlas_record = getattr(self, "_atlas_record", None)
        if atlas_record is not None:
            self._last_boundary["atlas"] = dict(atlas_record)
        fallback_record = getattr(self, "_cartesian_fallback_record", None)
        if fallback_record is not None:
            self._last_boundary["cartesian_joint_fallback"] = dict(
                fallback_record)
        align_record = getattr(self, "_align_retry_record", None)
        if align_record is not None:
            self._last_boundary["align_retry"] = dict(align_record)
        if self._occupancy is not None:
            from luggage_planning.occupancy_place_paths import (
                occupancy_collision_boxes)
            boxes = occupancy_collision_boxes(
                self._occupancy, max_objects=64,
                exempt=self._occupancy_exempt)
            self._last_boundary["occupancy_boxes"] = [
                {"id": box["id"], "xyz": list(box["xyz"]),
                 "size": list(box["size"])}
                for box in boxes]
        return result

    def _v2_fk_gate(self, joints):
        """Reject a v2 Cartesian-derived joint goal before any controller call."""
        admitted = _V2_ADMITTED.get()
        if admitted is None or admitted.target_pose is None:
            return None
        from luggage_planning.motion_frame_contracts import fk_budget
        ok, message = fk_budget(
            self._fk_verifier, joints,
            admitted.target_pose["position"], admitted.target_pose["quaternion_xyzw"])
        if ok:
            self.fk_submissions.append([float(v) for v in joints])
            return None
        return self._result(False, message, 0.0)

    def _execute_timed_joint_v2(self, admitted, feedback_cb, execute_timeout):
        """Submit the admitted timed trajectory. Do not densify or retime it."""
        from luggage_planning.joint_path import JointPathError, validate_joint_path
        from luggage_planning.motion_frame_adapters import timed_trajectory
        from moveit_msgs.msg import RobotTrajectory

        names, points = timed_trajectory(admitted)
        knots = [[float(v) for v in point.positions] for point in points]
        continuity = float(getattr(self, "_joint_path_continuity", 1.2))
        try:
            validate_joint_path(knots, continuity)
        except JointPathError as exc:
            return self._result(False, "PLACE_PATH_INFEASIBLE %s" % exc.reason, 0.0)
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = list(names)
        trajectory.joint_trajectory.points = points
        self._planned_traj = trajectory
        hook = getattr(self, "_timed_submit", None)
        if hook is not None:
            return hook(trajectory)
        self._notify(feedback_cb, "executing", admitted.name, 1.0)
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        if not self._execute.wait_for_server(timeout_sec=5.0):
            return self._result(False, "execute_trajectory server unavailable", 1.0)
        future = self._execute.send_goal_async(goal)
        handle = self._wait_future(future, 10.0)
        if handle is None or not handle.accepted:
            return self._result(False, "ExecuteTrajectory goal rejected", 1.0)
        wrapped = self._wait_goal(handle, execute_timeout)
        if wrapped is None:
            handle.cancel_goal()
            return self._result(False, "ExecuteTrajectory timeout", 1.0)
        result = getattr(wrapped, "result", wrapped)
        code = int(result.error_code.val)
        if code != MoveItErrorCodes.SUCCESS:
            return self._result(False, "ExecuteTrajectory error_code=%s" % code, 1.0)
        return self._result(True, "timed joint path", 1.0, error_code=code)

    def execute_segment_v2(self, record, context, feedback_cb=None,
                           execute_timeout=45.0, current_joints=None):
        """Admit, then execute. Joint paths keep their wire timing."""
        from luggage_description.frame_contracts import ContractError
        from luggage_planning.motion_frame_adapters import segment_namespace
        from luggage_planning.motion_frame_contracts import (
            MOTION_FRAME, assert_canonical_joints, admit_motion_segment,
        )
        assert_canonical_joints(JOINTS)
        admitted = admit_motion_segment(record, context)
        if self._frame != MOTION_FRAME or self._link != admitted.target_link:
            raise ContractError(
                "TARGET_LINK_MISMATCH", "executor",
                [MOTION_FRAME, admitted.target_link], [self._frame, self._link])
        if admitted.segment_type == "joint_path":
            with self._v2_lock:
                self._cancel_check = None
                try:
                    return self._execute_timed_joint_v2(
                        admitted, feedback_cb, execute_timeout)
                finally:
                    self._cancel_check = None
        token = _V2_ADMITTED.set(admitted)
        try:
            with self._v2_lock:
                return self.execute_segment(
                    segment_namespace(admitted), feedback_cb, execute_timeout,
                    current_joints)
        finally:
            _V2_ADMITTED.reset(token)

    def probe_segment_v2(self, record, context, start_joints=None):
        """Admit and probe. Execution and valve clients are not called."""
        from luggage_description.frame_contracts import ContractError
        from luggage_planning.motion_frame_adapters import segment_namespace
        from luggage_planning.motion_frame_contracts import (
            MOTION_FRAME, admit_motion_segment, fk_budget,
        )
        admitted = admit_motion_segment(record, context)
        if self._frame != MOTION_FRAME or self._link != admitted.target_link:
            raise ContractError(
                "TARGET_LINK_MISMATCH", "executor",
                [MOTION_FRAME, admitted.target_link], [self._frame, self._link])
        if admitted.segment_type == "joint_path":
            return {"ik_ok": False, "fraction": None, "message": "joint_path admitted",
                    "success": True}
        token = _V2_ADMITTED.set(admitted)
        try:
            with self._v2_lock:
                probed = self.probe_segment(segment_namespace(admitted), start_joints)
        finally:
            _V2_ADMITTED.reset(token)
        joints = probed.get("ik_joints") if isinstance(probed, dict) else None
        if admitted.segment_type == "pose_target" and joints:
            ok, message = fk_budget(
                self._fk_verifier, joints,
                admitted.target_pose["position"], admitted.target_pose["quaternion_xyzw"])
            if not ok:
                probed = dict(probed)
                probed["ik_ok"] = False
                probed["message"] = message
                probed["success"] = False
                return probed
        if isinstance(probed, dict):
            probed = dict(probed)
            probed.setdefault("success", bool(probed.get("ik_ok")))
            probed.setdefault("message", "probe")
        return probed

    def execute_segment(self, segment_msg, feedback_cb=None,
                        execute_timeout=45.0, current_joints=None,
                        cancel_check=None):
        """Execute one luggage_msgs/MotionSegment.

        Returns ``SegmentExecResult``. ``fraction`` is 1.0 for pose-target
        paths and the computed Cartesian fraction otherwise.
        ``cancel_check`` is polled while inner action goals run; a True
        return cancels the inner goal so the controller stops.
        """
        self._cancel_check = cancel_check
        self._planned_traj = None
        self._reset_boundary_measurements()
        self._start_joints = list(current_joints or [])
        try:
            result = self._execute_segment_inner(
                segment_msg, feedback_cb, execute_timeout, current_joints)
            return self._stamp_boundary(
                result, getattr(segment_msg, "name", ""))
        finally:
            self._cancel_check = None

    def _robot_trajectory(self, knots):
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = list(JOINTS)
        for knot in knots:
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in knot]
            trajectory.joint_trajectory.points.append(point)
        return trajectory

    def _run_joint_path(self, segment_msg, feedback_cb, execute_timeout):
        """Execute selector joint knots. Sweep failures do not fall back to RRT."""
        from luggage_planning.joint_path import JointPathError, validate_joint_path
        from luggage_planning.place_selection_client import read_joint_knots

        name = str(getattr(segment_msg, "name", "") or "")
        knots = read_joint_knots(segment_msg)
        continuity = float(getattr(self, "_joint_path_continuity", 1.2))

        def fk(joints):
            return self._fk_xyz(JOINTS, joints)

        def sweep(points):
            return self._occupancy_ok_points(points)

        try:
            if self._occupancy is None:
                dense = validate_joint_path(knots, continuity)
            else:
                dense = validate_joint_path(
                    knots, continuity, fk_fn=fk, sweep_fn=sweep, fk_stride=4)
        except JointPathError as exc:
            return self._result(
                False, "PLACE_PATH_INFEASIBLE %s" % exc.reason, 0.0)
        trajectory = self._robot_trajectory(dense)
        self._planned_traj = trajectory
        return self._execute_robot_trajectory(
            trajectory, name, 1.0, feedback_cb, execute_timeout)

    def _cartesian_refine_fallback(self, segment_msg, feedback_cb,
                                   execute_timeout, current_joints,
                                   why, fraction):
        """Deterministic re-solve for a rejected no-fallback cartesian leg.

        GetCartesianPath is one seeded solve with jump detection off, so it
        can stall mid-descent after drifting to a different IK branch (the
        2026-09-27 n=3 insert: j6 0.771 rad over 5.4 cm at fraction 0.244
        while a continuous branch from the same state needs j6 -0.017). The
        node-injected provider re-solves the goal offline from the live
        joints -- in effect choosing the branch continuous with the current
        arm -- and the dense path must clear the SAME excursion bar before
        execution, so the fallback cannot legitimise a bad re-solve.

        Returns ``None`` to keep the original failure (fail-closed).
        """
        from luggage_planning.joint_path import JointPathError, validate_joint_path

        record = {"used": False, "rejected_for": str(why),
                  "fraction": round(float(fraction), 4)}
        self._cartesian_fallback_record = record
        try:
            info = self._cartesian_joint_fallback(segment_msg, current_joints)
        except Exception as exc:  # noqa: BLE001 - provider boundary
            record["reason"] = "provider_error"
            record["error"] = str(exc)[:200]
            return None
        if not info or not info.get("knots"):
            record["reason"] = "refine_failed"
            return None
        knots = [[float(v) for v in row] for row in info["knots"]]
        continuity = float(getattr(self, "_joint_path_continuity", 1.2))

        def fk(joints):
            return self._fk_xyz(JOINTS, joints)

        def sweep(points):
            return self._occupancy_ok_points(points)

        try:
            if self._occupancy is None:
                dense = validate_joint_path(knots, continuity)
            else:
                dense = validate_joint_path(
                    knots, continuity, fk_fn=fk, sweep_fn=sweep, fk_stride=4)
        except JointPathError as exc:
            record["reason"] = "joint_path_error:%s" % exc.reason
            return None
        path_len = max(
            float(info.get("fk_path_len") or 0.0),
            _CARTESIAN_EXCURSION_MIN_PATH_M)
        for idx in range(len(JOINTS)):
            variation = sum(
                abs(b[idx] - a[idx]) for a, b in zip(dense, dense[1:]))
            if variation / path_len > self._excursion_limit:
                record["reason"] = "excursion_recheck_joint%d" % (idx + 1)
                record["fk_path_len_m"] = round(path_len, 4)
                return None
        record.update({"used": True, "n_dense": len(dense),
                       "fk_path_len_m": round(path_len, 4)})
        gate = self._v2_fk_gate(dense[-1])
        if gate is not None:
            record["reason"] = gate.message
            return gate
        self._notify(feedback_cb, "planning", segment_msg.name, fraction,
                     "%s; cartesian joint fallback" % why)
        trajectory = self._robot_trajectory(dense)
        self._planned_traj = trajectory
        result = self._execute_robot_trajectory(
            trajectory, str(getattr(segment_msg, "name", "") or ""), 1.0,
            feedback_cb, execute_timeout)
        result.message = "%s (cartesian joint fallback from %s)" % (
            result.message, why)
        result.fraction = float(fraction)
        return result

    def _align_then_retry(self, segment_msg, feedback_cb,
                          execute_timeout, current_joints, why, fraction):
        """Rotate the wrist onto the goal heading in place, then re-descend.

        The carry does not constrain the arrival branch, so the tool can
        reach the descent start with its heading ~0.8 rad off the insert
        goal's place_yaw; with j5 pinned near pi/2 for tool-down that
        difference is pure j6, and paying it inside a 5-20 cm cartesian
        descent is exactly the unwind the excursion gate refuses
        (2026-09-29 v3 trial1: 0.837 rad, 14.3 rad/m; the joint fallback
        also correctly refuses it as a branch jump). This step pays it as
        an in-place rotation (pose_target at the CURRENT end-effector
        position with the goal orientation), then re-runs the descent.

        Returns ``None`` to keep the original failure (fail-closed).
        """
        from types import SimpleNamespace

        excursion = self._cartesian_excursion or {}
        record = {
            "trigger": str(why),
            "joint": excursion.get("joint"),
            "excursion_rad": excursion.get("excursion_rad"),
            "fraction": round(float(fraction), 4),
        }
        self._align_retry_record = record
        pos = self._fk_xyz(JOINTS, [float(v) for v in (current_joints or [])])
        if not pos:
            record["reason"] = "fk_unavailable"
            return None
        goal = segment_msg.target_pose
        orientation = (
            float(getattr(goal.orientation, "x", 0.0)),
            float(getattr(goal.orientation, "y", 0.0)),
            float(getattr(goal.orientation, "z", 0.0)),
            float(getattr(goal.orientation, "w", 1.0)))
        align_pose = type(goal)()
        try:
            align_pose.position.x = float(pos[0])
            align_pose.position.y = float(pos[1])
            align_pose.position.z = float(pos[2])
            align_pose.orientation.x = orientation[0]
            align_pose.orientation.y = orientation[1]
            align_pose.orientation.z = orientation[2]
            align_pose.orientation.w = orientation[3]
        except AttributeError:
            # Stand-in pose objects (tests) have no typed sub-fields.
            align_pose = SimpleNamespace(
                position=SimpleNamespace(
                    x=float(pos[0]), y=float(pos[1]), z=float(pos[2])),
                orientation=SimpleNamespace(
                    x=orientation[0], y=orientation[1],
                    z=orientation[2], w=orientation[3]))
        align = SimpleNamespace(
            name="wrist_align", type="pose_target", target_pose=align_pose,
            keep_tool_down=True, allow_ompl_fallback=False)
        self._notify(feedback_cb, "planning", segment_msg.name, fraction,
                     "%s; wrist align-then-retry" % why)
        align_result = self._run_pose_target(
            align, feedback_cb, execute_timeout, current_joints)
        record["align_ok"] = bool(align_result.success)
        if not align_result.success:
            record["reason"] = "align_failed"
            return None
        fraction2, plan2 = self._plan_cartesian(segment_msg, feedback_cb)
        record["retry_fraction"] = (
            None if plan2 is None else round(float(fraction2), 4))
        required = max(
            self._min_fraction,
            float(getattr(segment_msg,
                          "required_cartesian_fraction", 0.0) or 0.0))
        if (plan2 is None or getattr(plan2, "excursion_rejected", False)
                or getattr(plan2, "occupancy_blocked", False)
                or fraction2 < required):
            record["retry_ok"] = False
            return None
        record["retry_ok"] = True
        result = self._execute_trajectory(
            plan2, segment_msg, fraction2, feedback_cb, execute_timeout)
        result.message = "%s (wrist align-then-retry from %s)" % (
            result.message, why)
        return result

    def _execute_segment_inner(self, segment_msg, feedback_cb,
                               execute_timeout, current_joints):
        seg_type = str(segment_msg.type)
        if seg_type == "joint_path":
            return self._run_joint_path(
                segment_msg, feedback_cb, execute_timeout)
        if seg_type == "pose_target":
            return self._run_pose_target(segment_msg, feedback_cb,
                                         execute_timeout, current_joints)
        if seg_type == "cartesian":
            fraction, plan = self._plan_cartesian(segment_msg, feedback_cb)
            if plan is None:
                return self._result(False, "compute_cartesian_path timeout", 0.0)
            occupancy_blocked = bool(getattr(plan, "occupancy_blocked", False))
            excursion_rejected = bool(
                getattr(plan, "excursion_rejected", False))
            required = max(
                self._min_fraction,
                float(getattr(segment_msg,
                              "required_cartesian_fraction", 0.0) or 0.0))
            need_fallback = (
                occupancy_blocked or excursion_rejected
                or fraction < required)
            if need_fallback:
                if occupancy_blocked:
                    why = "occupancy blocked"
                elif excursion_rejected:
                    why = plan.reason
                else:
                    why = "cartesian %.3f < %.3f" % (
                        fraction, self._min_fraction)
                if (excursion_rejected and self._align_retry_enabled
                        and str(getattr(segment_msg, "name", "")) in (
                            "insert", "descend")
                        and bool(getattr(segment_msg, "keep_tool_down", False))
                        and str((self._cartesian_excursion or {}).get(
                            "joint", "")) in ("elfin_joint5", "elfin_joint6")):
                    # Wrist-only unwind on a descent: the carry left the
                    # tool's heading off the goal's; pay the rotation in
                    # place first instead of inside the descent.
                    aligned = self._align_then_retry(
                        segment_msg, feedback_cb, execute_timeout,
                        current_joints, why, fraction)
                    if aligned is not None:
                        return aligned
                if (segment_msg.allow_ompl_fallback
                        and required <= self._min_fraction):
                    self._notify(feedback_cb, "planning", segment_msg.name,
                                 fraction, "%s; OMPL fallback" % why)
                    fallback = self._run_pose_target(
                        segment_msg, feedback_cb, execute_timeout,
                        current_joints)
                    if fallback.success:
                        fallback.message = "%s (fallback from %s)" % (
                            fallback.message, why)
                        fallback.fraction = fraction
                        fallback.used_ompl_fallback = True
                        return fallback
                    # OMPL failed too (e.g. 2026-09-27 box3 transit: KDL
                    # -31 with no atlas seeds): the deterministic polyline
                    # re-solve is the last exit before failing the leg.
                    if (not occupancy_blocked
                            and callable(self._cartesian_joint_fallback)
                            and current_joints):
                        joint = self._cartesian_refine_fallback(
                            segment_msg, feedback_cb, execute_timeout,
                            current_joints,
                            "%s; OMPL fallback failed (%s)" % (
                                why, fallback.message),
                            fraction)
                        if joint is not None:
                            return joint
                    fallback.message = "%s (fallback from %s)" % (
                        fallback.message, why)
                    fallback.fraction = fraction
                    fallback.used_ompl_fallback = True
                    return fallback
                if (not occupancy_blocked
                        and callable(self._cartesian_joint_fallback)
                        and current_joints):
                    # insert/descend/retreat class: the cartesian solve
                    # stalled or drifted branches, OMPL is off by design --
                    # re-solve deterministically from the live joints.
                    fallback = self._cartesian_refine_fallback(
                        segment_msg, feedback_cb, execute_timeout,
                        current_joints, why, fraction)
                    if fallback is not None:
                        return fallback
                if occupancy_blocked:
                    return self._result(
                        False, "PLACE_PATH_INFEASIBLE occupancy blocked cartesian",
                        fraction)
                if excursion_rejected:
                    return self._result(
                        False, "PLACE_PATH_EXCURSION %s" % why, fraction)
                return self._result(
                    False,
                    "cartesian fraction %.3f below %.3f and no OMPL fallback"
                    % (fraction, required),
                    fraction)
            if self._occupancy is not None:
                ok, sweep_msg = self._occupancy_ok_plan(plan, segment_msg)
                if not ok:
                    return self._result(
                        False, "PLACE_PATH_INFEASIBLE %s" % sweep_msg, fraction)
            return self._execute_trajectory(plan, segment_msg, fraction,
                                            feedback_cb, execute_timeout)
        return SegmentExecResult(
            False, "unknown segment type %r" % seg_type, 0.0)

    def probe_segment(self, segment_msg, start_joints=None):
        """IK + optional cartesian fraction, no execution (dry-run)."""
        self._reset_boundary_measurements()
        ik = self._ik_joints(
            segment_msg.target_pose, start_joints,
            keep_tool_down=bool(getattr(segment_msg, "keep_tool_down", False)))
        record = {
            "name": str(segment_msg.name),
            "type": str(segment_msg.type),
            "ik_ok": ik is not None,
            "ik_joints": ik,
            "fraction": None,
            "cartesian_ok": None,
        }
        if self._pose_target_joint_delta is not None:
            record["pose_target_joint_delta"] = dict(
                self._pose_target_joint_delta)
        atlas_record = getattr(self, "_atlas_record", None)
        if atlas_record is not None:
            record["atlas"] = dict(atlas_record)
        if self._j6_measurement is not None:
            record["j6_rotation"] = dict(self._j6_measurement)
        if start_joints and ik is not None:
            measurement = self._wrist_check(
                [list(start_joints), list(ik)], JOINTS)
            if measurement is not None:
                record["wrist_margin"] = dict(measurement)
            j6 = self._j6_check(
                [list(start_joints), list(ik)], JOINTS)
            if j6 is not None:
                record["j6_rotation"] = dict(j6)
        if str(segment_msg.type) == "cartesian":
            fraction, plan = self._plan_cartesian(
                segment_msg, None, start_joints=start_joints)
            record["fraction"] = fraction if plan is not None else 0.0
            record["cartesian_ok"] = (
                plan is not None
                and not getattr(plan, "excursion_rejected", False)
                and fraction >= self._min_fraction)
            if self._cartesian_excursion is not None:
                record["cartesian_excursion"] = dict(self._cartesian_excursion)
            if self._wrist_measurement is not None:
                record["wrist_margin"] = dict(self._wrist_measurement)
            if self._j6_measurement is not None:
                record["j6_rotation"] = dict(self._j6_measurement)
        return record

    # ------------------------------------------------------------------
    # pose_target

    def _build_goal_constraints(self, segment_msg):
        target = segment_msg.target_pose
        constraints = Constraints()

        pos = PositionConstraint()
        pos.header.frame_id = self._frame
        pos.link_name = self._link
        box = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.005]  # 5 mm position tolerance
        box.primitives = [primitive]
        center = PoseMsg()
        center.position.x = float(target.position.x)
        center.position.y = float(target.position.y)
        center.position.z = float(target.position.z)
        center.orientation.w = 1.0
        box.primitive_poses = [center]
        pos.constraint_region = box
        pos.weight = 1.0
        constraints.position_constraints = [pos]

        orient = OrientationConstraint()
        orient.header.frame_id = self._frame
        orient.link_name = self._link
        orient.orientation.x = float(target.orientation.x)
        orient.orientation.y = float(target.orientation.y)
        orient.orientation.z = float(target.orientation.z)
        orient.orientation.w = float(target.orientation.w)
        orient.absolute_x_axis_tolerance = 0.05
        orient.absolute_y_axis_tolerance = 0.05
        orient.absolute_z_axis_tolerance = 0.05
        orient.weight = 1.0
        if segment_msg.keep_tool_down:
            # Roll/pitch within tool_down_abs_tol of world -Z. Yaw is free,
            # so j5/j6 may change branch as long as the suction normal
            # stays down.
            constraints.orientation_constraints = [
                self._tool_down_constraint()]
        else:
            constraints.orientation_constraints = [orient]

        return constraints

    def _tool_down_constraint(self):
        down = OrientationConstraint()
        down.header.frame_id = self._frame
        down.link_name = self._link
        down.orientation.x, down.orientation.y = TOOL_DOWN_QUAT[:2]
        down.orientation.z, down.orientation.w = TOOL_DOWN_QUAT[2:]
        down.absolute_x_axis_tolerance = self._tool_down_abs_tol
        down.absolute_y_axis_tolerance = self._tool_down_abs_tol
        down.absolute_z_axis_tolerance = math.pi
        down.weight = 1.0
        return down

    def _run_pose_target(self, segment_msg, feedback_cb, execute_timeout,
                         current_joints=None):
        self._notify(feedback_cb, "planning", segment_msg.name, 0.0)
        ik_joints = self._ik_joints(
            segment_msg.target_pose, current_joints,
            keep_tool_down=bool(getattr(segment_msg, "keep_tool_down", False)))
        if ik_joints is not None:
            result = self._run_joint_goal(
                ik_joints, feedback_cb, execute_timeout, "IK joint goal",
                seed_joints=current_joints, segment_msg=segment_msg)
            # A FK-verified atlas goal can still lose to a narrow-passage
            # transit (pickup_box clipping the container on the way in,
            # 2026-09-24 smoke): each seed is a different final posture,
            # so try the remaining verified goals before giving up.
            queue = list(getattr(self, "_atlas_joint_goal_queue", None) or [])

            def _cancelled():
                check = getattr(self, "_cancel_check", None)
                return check is not None and check()

            while not result.success and queue and not _cancelled():
                alt = queue.pop(0)
                note = "atlas joint goal fk_err=%.3f" % alt["fk_error_m"]
                result = self._run_joint_goal(
                    alt["joints"], feedback_cb, execute_timeout, note,
                    seed_joints=current_joints, segment_msg=segment_msg)
                if result.success:
                    record = getattr(self, "_atlas_record", None) or {}
                    record["outcome"] = "joint_goal_alt"
                    record["fk_error_m"] = alt["fk_error_m"]
                    self._atlas_record = record
            self._atlas_joint_goal_queue = []
            return result
        # Pose-constraint OMPL cannot sample a pose KDL already rejected
        # and can wrap a joint by ±2π.
        return self._result(
            False,
            "IK failed error_code=%s (no pose-constraint fallback)"
            % self._last_ik_code,
            error_code=int(self._last_ik_code or 0))

    def execute_joints(self, positions, feedback_cb=None, execute_timeout=45.0,
                       current_joints=None):
        """MoveIt joint-space plan/execute to ``positions`` (rad, JOINTS order)."""
        wrapped = list(positions)
        self._planned_traj = None
        self._start_joints = list(current_joints or [])
        if current_joints and len(current_joints) == len(JOINTS):
            wrapped = [
                _wrap_near(cur, tgt) for cur, tgt in zip(current_joints, wrapped)]
        result = self._run_joint_goal(
            wrapped, feedback_cb, execute_timeout, "named joint goal",
            seed_joints=current_joints)
        return self._stamp_boundary(result, "named_joint")

    def _run_joint_goal(self, positions, feedback_cb, execute_timeout, note,
                        seed_joints=None, segment_msg=None):
        """Plan/execute a joint goal.

        j5 and j6 travel are recorded, not refused. A keep_tool_down
        pose target already selected an IK solution whose suction normal
        is down. Pick segments still pin J6 before execution.
        """
        gate = self._v2_fk_gate(positions)
        if gate is not None:
            return gate
        if seed_joints and len(seed_joints) == len(positions):
            self._wrist_check([list(seed_joints), list(positions)], JOINTS)
            self._j6_check([list(seed_joints), list(positions)], JOINTS)
        self._notify(feedback_cb, "planning", note, 0.0)
        goal = MoveGroup.Goal()
        goal.request.group_name = self._group
        goal.request.num_planning_attempts = self._attempts
        goal.request.allowed_planning_time = self._planning_time
        goal.request.planner_id = self._planner_id
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints = [self._joint_constraints(positions)]
        goal.planning_options.plan_only = True
        goal.planning_options.replan = False
        if not self._move_group.wait_for_server(timeout_sec=5.0):
            return self._result(False, "move_group action server unavailable")
        future = self._move_group.send_goal_async(
            goal, feedback_callback=self._movegroup_feedback(feedback_cb, note))
        handle = self._wait_future(future, 5.0 + self._planning_time)
        if handle is None or not handle.accepted:
            return self._result(False, "MoveGroup goal rejected/timeout")
        wrapped = self._wait_goal(handle, execute_timeout)
        if wrapped is None:
            handle.cancel_goal()
            return self._result(False, "MoveGroup execution timeout")
        result = getattr(wrapped, "result", wrapped)
        return self._finish_move_group(
            result, note, segment_msg, feedback_cb, execute_timeout, "")

    def _ik_joints(self, pose, current_joints, keep_tool_down=False):
        """IK seeded at the current arm, then wrap each joint nearest current.

        For a keep_tool_down target, also solve the equivalent yaw+pi
        quaternion (identical for a rectangular payload) and keep the
        solution with the smaller max per-joint delta from the seed.
        j5 and j6 are not filtered. ``_wrap_near`` only folds k*2pi.

        OMPL pose-constraint sampling otherwise returns a +/-2pi equivalent of
        joint1 (limits are +/-6.28) and controller times out 4 rad away.

        Atlas chain (``atlas_seed_provider``): miss-retry re-solves through
        the provider's seeds when the unseeded pass finds nothing;
        always-first runs the seeds before the unseeded pass. When every IK
        pass is empty and submit_mode allows joint_goal, a seed whose FK
        lands within ``atlas_fk_verify_tol_m`` of the target position is
        returned as the joint goal directly (the descend/traverse legs
        absorb the residual, as the executed 1838 transit showed). Every
        step lands in the boundary dump as ``atlas``.
        """
        poses = [pose]
        if keep_tool_down:
            poses.append(equivalent_tool_down_pose(pose))
        provider = getattr(self, "_atlas_provider", None)
        record = {
            "tried": False, "outcome": "", "status": None,
            "status_name": None, "cell": None, "n_seeds": 0,
        }
        self._atlas_record = record if callable(provider) else None
        cached_seeds = []

        def _provider_seeds():
            if cached_seeds or not callable(provider):
                return cached_seeds
            try:
                info = provider(pose) or {}
            except Exception as exc:  # noqa: BLE001 - atlas must never break IK
                info = {}
                node = getattr(self, "_node", None)
                logger = getattr(node, "get_logger", None)
                if logger is not None:
                    logger().warning("atlas seed provider error: %s" % exc)
            rows = [list(row) for row in (info.get("seeds") or [])
                    if row and len(row) == len(JOINTS)]
            if info.get("refined"):
                # Node-side refine curates the list (atlas branches + the
                # live-arm family); truncating here would drop the family
                # whose hull corridor actually plans (2026-09-24 smoke22).
                cached_seeds.extend(rows)
            else:
                cached_seeds.extend(rows[:max(0, int(self._atlas_max_seeds))])
            record["tried"] = True
            record["refined"] = bool(info.get("refined"))
            record["status"] = info.get("status")
            record["status_name"] = info.get("status_name")
            record["cell"] = info.get("seed_cell") or info.get("cell")
            record["n_seeds"] = len(cached_seeds)
            if not cached_seeds:
                record["outcome"] = "no_seeds"
            return cached_seeds

        solutions = []

        def _solve(label, seeds):
            hits = 0
            for seed in seeds:
                for candidate in poses:
                    solved = self._query_ik(candidate, seed)
                    if solved is not None:
                        solutions.append(solved)
                        hits += 1
            if hits:
                record["outcome"] = label
            return hits

        if callable(provider) and self._atlas_mode == "always-first":
            _solve("always_first", _provider_seeds())
        if not solutions:
            _solve("unseeded", [current_joints])
        if (not solutions and callable(provider)
                and self._atlas_mode != "always-first"):
            _solve("seeded", _provider_seeds())
        if not solutions and not record["outcome"]:
            record["outcome"] = "no_solution"

        self._pose_target_joint_delta = {
            "n_solutions": 0,
            "ok": False,
            "max_rad": None,
            "limit_rad": float(self._pose_target_max_delta),
        }
        self._j6_measurement = None
        if not solutions and callable(provider) and \
                "joint_goal" in self._atlas_submit_mode:
            target = (float(pose.position.x), float(pose.position.y),
                      float(pose.position.z))
            verified = []
            # Offline-refined seeds (node-side scipy) already carry their FK
            # residual; the service round-trip only loses seeds under load
            # (2026-09-24 smoke22: the working live-family goal dropped by
            # an _fk_xyz timeout).
            refined = bool(record.get("refined"))
            for seed in _provider_seeds():
                if refined:
                    verified.append({"joints": list(seed),
                                     "fk_error_m": 0.0})
                    continue
                fk = self._fk_xyz(JOINTS, seed)
                if fk is None:
                    continue
                error = math.sqrt(sum(
                    (fk[i] - target[i]) ** 2 for i in range(3)))
                if error <= float(self._atlas_fk_tol):
                    verified.append({"joints": list(seed),
                                     "fk_error_m": round(error, 4)})
            # Fold every goal onto the wrap nearest the live arm: a
            # k*2pi-distant j5/j6 is the same pose but a 6 rad unwind for
            # the next cartesian leg (2026-09-24 smoke18: insert rejected
            # j6 6.0 rad over 7 cm purely from the wrap).
            if current_joints and len(current_joints) == len(JOINTS):
                for item in verified:
                    item["joints"] = [
                        _wrap_near(cur, tgt) for cur, tgt in zip(
                            current_joints, item["joints"])]
            verified.sort(key=lambda item: item["fk_error_m"])
            self._atlas_joint_goal_queue = verified[1:]
            if verified:
                record["outcome"] = "joint_goal"
                record["fk_error_m"] = verified[0]["fk_error_m"]
                record["n_joint_goals"] = len(verified)
                solutions.append(verified[0]["joints"])
        if not solutions:
            return None
        if not current_joints or len(current_joints) != len(JOINTS):
            self._pose_target_joint_delta["n_solutions"] = len(solutions)
            return solutions[0]

        def _max_delta(sol):
            return max(
                abs(float(a) - float(b))
                for a, b in zip(current_joints, sol))

        chosen = min(solutions, key=_max_delta)
        measurement = pose_target_joint_delta(
            current_joints, chosen, self._pose_target_max_delta)
        if measurement is not None:
            measurement["n_solutions"] = len(solutions)
        self._pose_target_joint_delta = measurement
        self._j6_check([list(current_joints), list(chosen)], JOINTS)
        self._wrist_check([list(current_joints), list(chosen)], JOINTS)
        return chosen

    def _query_ik(self, pose, current_joints):
        """One GetPositionIK call, wrapped nearest the seed."""
        self._ik_t0 = time.monotonic()
        self._ik_target = (float(pose.position.x), float(pose.position.y),
                           float(pose.position.z))
        return self._query_ik_inner(pose, current_joints)

    def _log_ik_call(self, seed, verdict):
        node = getattr(self, "_node", None)
        logger = getattr(node, "get_logger", None)
        if logger is None:
            return
        logger().info(
            "compute_ik %s seed=%s target=(%.3f, %.3f, %.3f) %.2fs"
            % (verdict,
               [round(float(v), 2) for v in (seed or [])][:6],
               getattr(self, "_ik_target", (0, 0, 0))[0],
               getattr(self, "_ik_target", (0, 0, 0))[1],
               getattr(self, "_ik_target", (0, 0, 0))[2],
               time.monotonic() - getattr(self, "_ik_t0", time.monotonic())))

    def _query_ik_inner(self, pose, current_joints):
        if not self._ik.wait_for_service(timeout_sec=2.0):
            self._last_ik_code = -1
            return None
        request = GetPositionIK.Request()
        request.ik_request.group_name = self._group
        request.ik_request.ik_link_name = self._link
        request.ik_request.avoid_collisions = True
        request.ik_request.robot_state.is_diff = True
        if current_joints and len(current_joints) == len(JOINTS):
            request.ik_request.robot_state.joint_state.name = list(JOINTS)
            request.ik_request.robot_state.joint_state.position = [
                float(v) for v in current_joints]
        request.ik_request.timeout = DurationMsg(sec=1, nanosec=0)
        stamped = PoseStamped()
        stamped.header.frame_id = self._frame
        gm = PoseMsg()
        gm.position.x = float(pose.position.x)
        gm.position.y = float(pose.position.y)
        gm.position.z = float(pose.position.z)
        gm.orientation.x = float(pose.orientation.x)
        gm.orientation.y = float(pose.orientation.y)
        gm.orientation.z = float(pose.orientation.z)
        gm.orientation.w = float(pose.orientation.w)
        stamped.pose = gm
        request.ik_request.pose_stamped = stamped
        future = self._ik.call_async(request)
        response = self._wait_future(future, 3.0)
        if response is None:
            self._last_ik_code = -1
            self._log_ik_call(current_joints, "future_timeout")
            return None
        self._last_ik_code = int(response.error_code.val)
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self._log_ik_call(current_joints, "code=%d" % self._last_ik_code)
            return None
        by_name = dict(zip(
            response.solution.joint_state.name,
            response.solution.joint_state.position))
        if not all(name in by_name for name in JOINTS):
            self._last_ik_code = -2
            return None
        raw = [float(by_name[name]) for name in JOINTS]
        if not current_joints or len(current_joints) != len(JOINTS):
            return raw
        return [
            _wrap_near(cur, tgt) for cur, tgt in zip(current_joints, raw)]

    @staticmethod
    def _joint_constraints(positions):
        constraints = Constraints()
        for name, value in zip(JOINTS, positions):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(value)
            joint.tolerance_above = 0.02
            joint.tolerance_below = 0.02
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        return constraints

    def _finish_move_group(self, result, note, segment_msg, feedback_cb,
                           execute_timeout, unimplemented):
        code = int(result.error_code.val)
        traj = getattr(result, "planned_trajectory", None)
        if traj is not None:
            self._planned_traj = traj
        if code != MoveItErrorCodes.SUCCESS:
            return self._result(
                False, "MoveGroup error_code=%s (%s)" % (code, note),
                error_code=code)
        # j5 proximity and j6 travel are recorded. Neither refuses.
        joint_traj = getattr(traj, "joint_trajectory", None)
        if joint_traj is not None:
            points = list(getattr(joint_traj, "points", None) or [])
            names = getattr(joint_traj, "joint_names", None)
            self._wrist_check(points, names)
            self._j6_check(points, names)
        if self._occupancy is not None:
            ok, sweep_msg = self._occupancy_ok_robot_traj(traj)
            if not ok:
                return self._result(
                    False, "PLACE_PATH_INFEASIBLE %s" % sweep_msg,
                    fraction=1.0, error_code=code)
        self._hold_pick_joint6(traj, segment_msg)
        exec_result = self._execute_robot_trajectory(
            traj, note, 1.0, feedback_cb, execute_timeout)
        if exec_result.success:
            exec_result.message = "pose_target ok (%s; rest-to-rest)" % note
            if unimplemented:
                exec_result.message += "; " + unimplemented
        return exec_result

    def _occupancy_ok_points(self, points):
        if self._occupancy is None:
            return True, ""
        if not self._payload_wdh:
            return False, "payload size missing"
        mapped = []
        convert = self._world_to_map
        for point in points:
            xyz = (float(point[0]), float(point[1]), float(point[2]))
            if convert is not None:
                xyz = tuple(float(v) for v in convert(xyz))
            mapped.append(xyz)
        from luggage_planning.occupancy_place_paths import sweep_polyline
        sweep = sweep_polyline(
            self._occupancy, mapped, self._payload_wdh,
            arm_radius=self._arm_radius,
            exempt=self._occupancy_exempt)
        if sweep.collides:
            return False, sweep.reason or "occupancy_collision"
        return True, ""

    def _occupancy_ok_plan(self, plan_response, segment_msg):
        traj = getattr(getattr(plan_response, "solution", None),
                       "joint_trajectory", None)
        if traj is not None and traj.points:
            return self._occupancy_ok_robot_traj(plan_response.solution)
        points = []
        for wp in list(getattr(segment_msg, "waypoints", None) or []):
            points.append((wp.position.x, wp.position.y, wp.position.z))
        tgt = segment_msg.target_pose
        points.append((tgt.position.x, tgt.position.y, tgt.position.z))
        return self._occupancy_ok_points(points)

    def _occupancy_ok_robot_traj(self, robot_traj):
        if robot_traj is None:
            return False, "empty planned trajectory"
        points = self._fk_polyline(robot_traj)
        if not points:
            return False, "FK polyline empty"
        return self._occupancy_ok_points(points)

    def _fk_polyline(self, robot_traj):
        joint_traj = getattr(robot_traj, "joint_trajectory", robot_traj)
        names = list(getattr(joint_traj, "joint_names", None) or JOINTS)
        pts = list(getattr(joint_traj, "points", None) or [])
        if not pts:
            return []
        step = max(1, len(pts) // 40)
        polyline = []
        if not self._fk.wait_for_service(timeout_sec=2.0):
            return []
        from moveit_msgs.msg import RobotState
        for point in pts[::step] + ([pts[-1]] if pts else []):
            request = GetPositionFK.Request()
            request.header.frame_id = self._frame
            request.fk_link_names = [self._link]
            state = RobotState()
            state.joint_state.name = list(names)
            state.joint_state.position = [float(v) for v in point.positions]
            request.robot_state = state
            future = self._fk.call_async(request)
            response = self._wait_future(future, 2.0)
            if response is None or int(response.error_code.val) != int(
                    MoveItErrorCodes.SUCCESS):
                continue
            if not response.pose_stamped:
                continue
            pos = response.pose_stamped[0].pose.position
            polyline.append((pos.x, pos.y, pos.z))
        return polyline

    def _fk_xyz(self, joint_names, positions):
        """End-effector position for one joint vector, or None."""
        if not self._fk.wait_for_service(timeout_sec=2.0):
            return None
        from moveit_msgs.msg import RobotState
        request = GetPositionFK.Request()
        request.header.frame_id = self._frame
        request.fk_link_names = [self._link]
        state = RobotState()
        state.joint_state.name = list(joint_names or JOINTS)
        state.joint_state.position = [float(v) for v in positions]
        request.robot_state = state
        response = self._wait_future(self._fk.call_async(request), 2.0)
        if response is None or int(response.error_code.val) != int(
                MoveItErrorCodes.SUCCESS):
            return None
        if not response.pose_stamped:
            return None
        pos = response.pose_stamped[0].pose.position
        return (pos.x, pos.y, pos.z)

    def _requested_cartesian_length(self, waypoints, solution):
        """Straight-line length of the requested waypoint chain, or None.

        ``compute_cartesian_path`` interpolates linearly between waypoints, so
        the chain length is the Cartesian distance the solution covers. The
        start pose is not in the request; take it from the first point of the
        returned solution.
        """
        joint_traj = getattr(solution, "joint_trajectory", solution)
        points = list(getattr(joint_traj, "points", None) or [])
        if not points or not waypoints:
            return None
        start = self._fk_xyz(
            getattr(joint_traj, "joint_names", None) or JOINTS,
            points[0].positions)
        if start is None:
            return None
        chain = [start] + [
            (wp.position.x, wp.position.y, wp.position.z) for wp in waypoints]
        return sum(
            math.sqrt(sum((b[i] - a[i]) ** 2 for i in range(3)))
            for a, b in zip(chain, chain[1:]))

    def _hold_pick_joint6(self, robot_traj, segment_msg):
        """Keep the suction-cup roll at the segment-start J6 during pick.

        J6 is coaxial with the panel. Pick segments pin every knot to the
        joints captured at ``execute_segment``; place segments are unchanged.
        """
        name = str(getattr(segment_msg, "name", "") or "")
        if robot_traj is None or not pick_holds_joint6(name):
            return
        held = joint6_value(self._start_joints)
        if held is None:
            self._node.get_logger().warning(
                "pick %s: J6 hold skipped, current joints missing" % name)
            return
        joint_traj = getattr(robot_traj, "joint_trajectory", robot_traj)
        pin_joint6_trajectory(joint_traj, held)
        self._node.get_logger().info(
            "pick %s: holding elfin_joint6 at %.4f rad" % (name, held))

    def _retime_robot_trajectory(self, robot_trajectory):
        """Drop MoveIt TOTG or linear times; write the site rest-to-rest profile."""
        joint_trajectory = getattr(robot_trajectory, "joint_trajectory", None)
        if joint_trajectory is None:
            return None
        timed = apply_rest_to_rest_to_joint_trajectory(
            joint_trajectory, self._vel_scale, self._acc_scale)
        if timed is not None:
            self._node.get_logger().info(
                "rest-to-rest duration=%.3fs path=%.4f rad "
                "v_peak=%.3f rad/s a_acc=%.3f a_decel=%.3f rad/s^2 "
                "(site accel 60 deg/s^2, decel 20 deg/s^2)"
                % (timed.duration, timed.path_length, timed.v_peak,
                   timed.a_max, timed.a_decel)
            )
        return timed

    def _execute_robot_trajectory(self, robot_traj, name, fraction,
                                  feedback_cb, execute_timeout):
        self._notify(feedback_cb, "executing", name, fraction)
        if robot_traj is not None:
            self._retime_robot_trajectory(robot_traj)
            self._planned_traj = robot_traj
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = robot_traj
        if not self._execute.wait_for_server(timeout_sec=5.0):
            return self._result(
                False, "execute_trajectory server unavailable", fraction)
        future = self._execute.send_goal_async(goal)
        handle = self._wait_future(future, 10.0)
        if handle is None or not handle.accepted:
            return self._result(
                False, "ExecuteTrajectory goal rejected", fraction)
        wrapped = self._wait_goal(handle, execute_timeout)
        if wrapped is None:
            handle.cancel_goal()
            return self._result(False, "ExecuteTrajectory timeout", fraction)
        result = getattr(wrapped, "result", wrapped)
        code = int(result.error_code.val)
        if code != MoveItErrorCodes.SUCCESS:
            return self._result(
                False, "ExecuteTrajectory error_code=%s" % code,
                fraction, error_code=code)
        return self._result(
            True, "ompl ok (%s)" % name, fraction, error_code=code)

    # ------------------------------------------------------------------
    # cartesian

    def _plan_cartesian(self, segment_msg, feedback_cb, start_joints=None):
        self._notify(feedback_cb, "planning", segment_msg.name, 0.0)
        waypoints = list(getattr(segment_msg, "waypoints", None) or [])
        waypoints.append(segment_msg.target_pose)
        if self._occupancy is not None:
            ee = [(wp.position.x, wp.position.y, wp.position.z)
                  for wp in waypoints]
            ok, _msg = self._occupancy_ok_points(ee)
            if not ok:
                return 0.0, _OccupancyBlockedPlan()
        request = GetCartesianPath.Request()
        request.header.frame_id = self._frame
        request.group_name = self._group
        request.link_name = self._link
        request.start_state.is_diff = True
        if start_joints is not None:
            request.start_state.joint_state.name = list(JOINTS)
            request.start_state.joint_state.position = [
                float(v) for v in start_joints]
        request.waypoints = waypoints
        request.max_step = self._max_step
        request.jump_threshold = 0.0
        request.avoid_collisions = self._avoid_collisions
        future = self._cartesian.call_async(request)
        response = self._wait_future(future, 15.0)
        if response is None:
            return 0.0, None
        fraction = float(response.fraction)
        # Pick locks J6 before the excursion gate. A Cartesian solution
        # may spin the coaxial cup almost half a turn over a few
        # centimetres; that ratio is not the path that will be sent.
        self._hold_pick_joint6(
            getattr(response, "solution", None), segment_msg)
        return self._judge_cartesian_response(
            segment_msg, waypoints, response, fraction)

    def _judge_cartesian_response(self, segment_msg, waypoints, response,
                                  fraction):
        """Apply the excursion gate. j5 and j6 are recorded, not refused."""
        rejected_exc = self._reject_for_excursion(
            waypoints, response, fraction)
        self._record_wrist(response)
        self._record_j6(response)
        if rejected_exc is not None:
            return fraction, rejected_exc
        return fraction, response

    def _reject_for_excursion(self, waypoints, response, fraction):
        """``_ExcursionRejectedPlan`` when the solution unwinds the arm.

        Records the measurement either way so a boundary dump shows the ratio
        of an accepted path, not only of a refused one.
        """
        self._cartesian_excursion = None
        solution = getattr(response, "solution", None)
        if solution is None:
            return None
        path_len = self._requested_cartesian_length(waypoints, solution)
        if path_len is None:
            self._cartesian_excursion = {"skipped": "fk_unavailable"}
            return None
        joint_traj = getattr(solution, "joint_trajectory", solution)
        measurement = cartesian_excursion(
            getattr(joint_traj, "joint_names", None) or JOINTS,
            list(getattr(joint_traj, "points", None) or []),
            path_len * max(0.0, min(1.0, fraction)),
            limit_rad_per_m=self._excursion_limit)
        if measurement is None:
            return None
        self._cartesian_excursion = measurement
        if measurement["ok"]:
            return None
        reason = (
            "cartesian joint excursion %.3f rad/m > %.3f (%s, %.3f rad over "
            "%.3f m)" % (
                measurement["rad_per_m"], measurement["limit_rad_per_m"],
                measurement["joint"], measurement["excursion_rad"],
                measurement["path_len_m"]))
        return _ExcursionRejectedPlan(reason, measurement)

    def _record_wrist(self, response):
        """Dump-only j5 measurement. Never refuses."""
        self._wrist_measurement = None
        solution = getattr(response, "solution", None)
        if solution is None:
            return None
        joint_traj = getattr(solution, "joint_trajectory", solution)
        return self._wrist_check(
            list(getattr(joint_traj, "points", None) or []),
            getattr(joint_traj, "joint_names", None))

    def _record_j6(self, response):
        """Dump-only j6 travel. Never refuses."""
        self._j6_measurement = None
        solution = getattr(response, "solution", None)
        if solution is None:
            return None
        joint_traj = getattr(solution, "joint_trajectory", solution)
        return self._j6_check(
            list(getattr(joint_traj, "points", None) or []),
            getattr(joint_traj, "joint_names", None))

    def _wrist_check(self, points, joint_names=None):
        """``wrist_margin`` over ``points``, resolving the joint index.

        Dump-only. A trajectory that does not carry joint5 is not judged.
        """
        names = list(joint_names or [])
        index = _WRIST_JOINT_INDEX
        if names:
            wrist = JOINTS[_WRIST_JOINT_INDEX]
            if wrist not in names:
                return None
            index = names.index(wrist)
        measurement = wrist_margin(
            points, margin_rad=self._wrist_margin, joint_index=index)
        if measurement is not None:
            self._wrist_measurement = measurement
        return measurement

    def _j6_check(self, points, joint_names=None):
        """``j6_rotation`` over ``points``, resolving the joint index."""
        names = list(joint_names or [])
        index = _J6_JOINT_INDEX
        if names:
            flange = JOINTS[_J6_JOINT_INDEX]
            if flange not in names:
                return None
            index = names.index(flange)
        measurement = j6_rotation(
            points, limit_rad=self._j6_limit, joint_index=index)
        if measurement is not None:
            self._j6_measurement = measurement
        return measurement

    def _retime_cartesian_solution(self, plan_response):
        """Time a GetCartesianPath solution with the site rest-to-rest cap.

        GetCartesianPath returns geometry without a Huayan-safe time law.
        MoveIt TOTG (joint_limits 3.14 rad/s^2 scaled) trips CollisionStop.
        """
        self._retime_robot_trajectory(plan_response.solution)
        return plan_response

    def _execute_trajectory(self, plan_response, segment_msg, fraction,
                            feedback_cb, execute_timeout):
        self._notify(feedback_cb, "executing", segment_msg.name, fraction)
        self._hold_pick_joint6(
            getattr(plan_response, "solution", None), segment_msg)
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = self._retime_cartesian_solution(
            plan_response).solution
        self._planned_traj = goal.trajectory
        if not self._execute.wait_for_server(timeout_sec=5.0):
            return SegmentExecResult(
                False, "execute_trajectory server unavailable", fraction)
        future = self._execute.send_goal_async(goal)
        handle = self._wait_future(future, 10.0)
        if handle is None or not handle.accepted:
            return SegmentExecResult(
                False, "ExecuteTrajectory goal rejected", fraction)
        wrapped = self._wait_goal(handle, execute_timeout)
        if wrapped is None:
            handle.cancel_goal()
            return SegmentExecResult(
                False, "ExecuteTrajectory timeout", fraction)
        result = getattr(wrapped, "result", wrapped)
        code = int(result.error_code.val)
        if code != MoveItErrorCodes.SUCCESS:
            return SegmentExecResult(
                False, "ExecuteTrajectory error_code=%s" % code,
                fraction, moveit_error_code=code)
        message = "cartesian ok (fraction %.3f)" % fraction
        unimplemented = self._unimplemented_flags_note(segment_msg)
        if unimplemented:
            message += "; " + unimplemented
        return SegmentExecResult(
            True, message, fraction, moveit_error_code=code)

    # ------------------------------------------------------------------
    # helpers

    @staticmethod
    def _unimplemented_flags_note(segment_msg):
        missing = []
        if segment_msg.keep_camera_down:
            missing.append("keep_camera_down")
        if segment_msg.lock_wrist:
            missing.append("lock_wrist")
        if not missing:
            return ""
        return "NOT_IMPLEMENTED: %s" % ", ".join(missing)

    def _notify(self, feedback_cb, stage, segment_name, fraction, note=""):
        if feedback_cb is None:
            return
        try:
            feedback_cb(stage, segment_name, fraction, note)
        except Exception:  # noqa: BLE001 - feedback must never break execution
            pass

    def _movegroup_feedback(self, feedback_cb, segment_name):
        # MoveGroup feedback carries only a state string (MONITORING_PLANNING,
        # PLANNING, ...), no numeric fraction; map planning states to the
        # "planning" stage and everything else to "executing".
        def _cb(_feedback_msg):
            # rclpy hands the callback a wrapper whose .feedback is the
            # MoveGroup.Feedback carrying the state string.
            raw = getattr(_feedback_msg, "feedback", _feedback_msg)
            state = str(getattr(raw, "state", ""))
            stage = "planning" if "PLAN" in state.upper() else "executing"
            self._notify(feedback_cb, stage, segment_name, 0.0)
        return _cb

    def _wait_future(self, future, timeout_sec):
        event = threading.Event()
        future.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout=timeout_sec):
            return None
        return future.result()

    def _wait_goal(self, handle, timeout_sec):
        """Wait for an accepted goal. Timeout is sim seconds when use_sim_time."""
        event = threading.Event()
        future = handle.get_result_async()
        future.add_done_callback(lambda _f: event.set())
        watcher = None
        if self._cancel_check is not None:
            def _watch():
                while not event.wait(0.2):
                    try:
                        if self._cancel_check():
                            handle.cancel_goal()
                            return
                    except Exception:  # noqa: BLE001 - watcher must not leak
                        return
            watcher = threading.Thread(target=_watch, daemon=True)
            watcher.start()
        reached, _reason = wait_event(
            event, timeout_sec, clock=self._node.get_clock())
        if not reached:
            if watcher is not None and watcher.is_alive():
                try:
                    handle.cancel_goal()
                except Exception:  # noqa: BLE001 - best-effort cancel
                    pass
            return None
        return future.result()
