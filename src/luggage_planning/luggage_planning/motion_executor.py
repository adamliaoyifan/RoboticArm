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
# Timed-path start gate. place_smoke_driver.QP_START_TOLERANCE_RAD must
# stay equal to this; the driver waits for it and does not widen it.
QP_START_TOLERANCE_RAD = 0.01
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
    singularity_rejected = False
    solution = None

    def __init__(self, reason, measurement):
        self.reason = str(reason)
        self.measurement = dict(measurement)


class _SingularityRejectedPlan(object):
    """A cartesian solution whose wrist track crosses or nears a singularity.

    The executor replans from the live joints instead of executing the track.
    """

    excursion_rejected = False
    singularity_rejected = True
    solution = None

    def __init__(self, reason, measurement):
        self.reason = str(reason)
        self.measurement = dict(measurement)


def cartesian_excursion(joint_names, points, path_len_m,
                        limit_rad_per_m=_CARTESIAN_EXCURSION_RAD_PER_M,
                        min_path_m=_CARTESIAN_EXCURSION_MIN_PATH_M,
                        ignore_joints=None):
    """Joint excursion per metre of Cartesian travel for one solution.

    ``points`` are objects with ``positions`` (a JointTrajectory) and
    ``path_len_m`` is the straight-line length of the requested waypoint
    chain. Returns ``None`` when there is nothing to judge, otherwise a dict
    with the worst joint, its total variation, the ratio, and ``ok``.

    ``ignore_joints`` (default none) are measured but do not refuse. j6 is
    coaxial with the suction cup: keep_tool_down frees yaw, so a long
    carry can accumulate several turns without being an arm unwind.
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
    ignored = set(ignore_joints or ())
    scored = [index for index in range(width)
              if (names[index] if index < len(names) else "") not in ignored]
    if not scored:
        scored = list(range(width))
    worst = max(scored, key=lambda index: totals[index])
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
    # v2 segments arrive as namespaces. type(pose)() does not grow the
    # nested position/orientation fields a Pose message has.
    out = PoseMsg()
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


def cartesian_hops(start, end, step_m=0.30):
    """Split a TCP line into hops of about ``step_m``.

    One 1.9 m pick-to-portal cartesian lets KDL change IK family
    (rehearsal-32: elfin_joint3 9.08 rad / 1.90 m). Short hops re-seed
    from the live arm so each solve stays in the current branch.
    """
    origin = [float(start[0]), float(start[1]), float(start[2])]
    dest = [float(end[0]), float(end[1]), float(end[2])]
    delta = [dest[i] - origin[i] for i in range(3)]
    length = math.sqrt(sum(v * v for v in delta))
    step = max(float(step_m), 1e-3)
    if length <= step * 1.25:
        return [dest]
    count = max(2, int(math.ceil(length / step)))
    hops = []
    for index in range(1, count + 1):
        frac = float(index) / float(count)
        hops.append([origin[i] + delta[i] * frac for i in range(3)])
    return hops


def execute_wait_timeout_sec(execute_timeout, points):
    """Wall wait for one leg's execution, aware of its own duration.

    A fixed 45 s wall deadline is shorter than a ~45 s nominal transit
    leg and fails even at RTF 1.0 (rehearsal_07: ExecuteTrajectory
    timeout at 45.99 s wall on a fully planned, healthy leg). The
    leg's own trajectory duration floors the wait with headroom for
    real-time-factor degradation; short legs keep the caller's timeout.
    """
    try:
        duration_s = max(
            float(point.time_from_start.sec)
            + float(point.time_from_start.nanosec) * 1e-9
            for point in points)
    except (AttributeError, TypeError, ValueError):
        return float(execute_timeout)
    if duration_s <= 0.0 or not math.isfinite(duration_s):
        return float(execute_timeout)
    return max(float(execute_timeout), duration_s * 2.0 + 10.0)


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
        # [ST09-I] /compute_cartesian_path and /compute_ik clients are
        # gone with the generation bypasses: geometry generation lives
        # in the route's MoveIt edge builder; this executor only submits
        # certified joint trajectories (plus the preserved GoToRobotPose
        # named-joint path) and verifies FK.
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
        """True when the MoveGroup/ExecuteTrajectory servers are up."""
        ok = self._move_group.wait_for_server(timeout_sec=timeout_sec)
        if not ok:
            return False
        return self._execute.wait_for_server(timeout_sec=5.0)

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

    def _admitted_position_swap(self, xyz):
        """Point the v2 FK gate at an intermediate hop. Restore with the return."""
        admitted = _V2_ADMITTED.get()
        pose = None if admitted is None else getattr(admitted, "target_pose", None)
        if not isinstance(pose, dict):
            return None
        saved = list(pose.get("position") or [])
        if len(saved) != 3:
            return None
        pose["position"] = [float(xyz[0]), float(xyz[1]), float(xyz[2])]
        return saved

    def _v2_fk_gate(self, joints):
        """Reject a v2 Cartesian-derived joint goal before any controller call."""
        admitted = _V2_ADMITTED.get()
        if admitted is None or admitted.target_pose is None:
            return None
        from luggage_planning.motion_frame_contracts import fk_budget
        ok, message = fk_budget(
            self._fk_verifier, joints,
            admitted.target_pose["position"], admitted.target_pose["quaternion_xyzw"],
            keep_tool_down=bool(admitted.keep_tool_down))
        if ok:
            self.fk_submissions.append([float(v) for v in joints])
            return None
        return self._result(False, message, 0.0)

    def _execute_timed_joint_v2(self, admitted, feedback_cb, execute_timeout,
                                current_joints=None):
        """Submit the admitted timed trajectory. Do not densify or retime it."""
        from luggage_planning.joint_path import JointPathError, validate_joint_path
        from luggage_planning.motion_frame_adapters import timed_trajectory
        from moveit_msgs.msg import RobotTrajectory

        names, points = timed_trajectory(admitted)
        knots = [[float(v) for v in point.positions] for point in points]
        # Revolute wrap: a full cycle can leave the live arm at a 2*pi
        # equivalent (live j6 = -7.7 rad) while the certified knots carry
        # the principal value — the same physical trajectory. Shift each
        # joint by a constant whole turn so the submitted knots sit next
        # to the LIVE joints (never command a full-turn spin, never let
        # bookkeeping drift past the joint limits).
        if current_joints and len(current_joints) == len(knots[0]):
            import math as _math

            shifts = []
            for live, first in zip(current_joints, knots[0]):
                turns = round((float(first) - float(live))
                              / (2.0 * _math.pi))
                shifts.append(turns * 2.0 * _math.pi)
            if any(shifts):
                knots = [[float(v) - s for v, s in zip(row, shifts)]
                         for row in knots]
                points = [type(point)(
                    positions=[float(v) - s for v, s in
                               zip(point.positions, shifts)],
                    velocities=list(point.velocities),
                    accelerations=list(getattr(point, "accelerations", [])
                                       or []),
                    effort=list(getattr(point, "effort", []) or []),
                    time_from_start=point.time_from_start)
                    for point in points]
        continuity = float(getattr(self, "_joint_path_continuity", 1.2))
        try:
            validate_joint_path(knots, continuity)
        except JointPathError as exc:
            return self._result(False, "PLACE_PATH_INFEASIBLE %s" % exc.reason, 0.0)
        # QP start-state gate: the timed trajectory was certified from the
        # planned start joints; a robot that moved since must replan.
        # (Angular distance: 2*pi equivalents of one revolute joint are
        # the same physical state.)
        if current_joints and len(current_joints) == len(knots[0]):
            import math as _math

            deltas = [
                min(abs(float(a) - float(b)),
                    2.0 * _math.pi - abs(float(a) - float(b)))
                for a, b in zip(current_joints, knots[0])]
            drift = max(deltas) if deltas else 0.0
            if drift > QP_START_TOLERANCE_RAD:
                if isinstance(self._last_boundary, dict):
                    self._last_boundary["qp_start_state_mismatch"] = {
                        "drift_rad": round(drift, 6),
                        "tolerance_rad": QP_START_TOLERANCE_RAD,
                        "joint_names": list(names),
                        "current_joints": [
                            round(float(v), 6) for v in current_joints],
                        "trajectory_first_joints": [
                            round(float(v), 6) for v in knots[0]],
                        "delta_per_joint": [round(v, 6) for v in deltas],
                    }
                return self._result(False, "QP_START_STATE_MISMATCH", 0.0)
        if getattr(self, "_occupancy", None) is not None \
                and getattr(self, "_payload_wdh", None):
            ok, reason = self._timed_knots_occupancy_ok(names, points)
            if not ok:
                return self._result(False, "PLACE_PATH_INFEASIBLE %s" % reason, 0.0)
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
        wrapped = self._wait_goal(handle, execute_wait_timeout_sec(
            execute_timeout, points))
        if wrapped is None:
            handle.cancel_goal()
            return self._result(False, "ExecuteTrajectory timeout", 1.0)
        result = getattr(wrapped, "result", wrapped)
        code = int(result.error_code.val)
        if code != MoveItErrorCodes.SUCCESS:
            return self._result(False, "ExecuteTrajectory error_code=%s" % code, 1.0)
        return self._result(True, "timed joint path", 1.0, error_code=code)

    def _timed_knots_occupancy_ok(self, names, points):
        """Offline-FK occupancy sweep of the timed knots.

        The QP certified these knots against the map it planned with; this
        re-sweeps against the executor's live snapshot. Offline FK (no
        /compute_fk round-trips) keeps the gate fast.
        """
        from luggage_planning.offline_ik import JOINTS, load_urdf
        order = [names.index(name) for name in JOINTS if name in names]
        knots = [[float(point.positions[i]) for i in order] for point in points]
        step = max(1, len(knots) // 20)
        sampled = knots[::step]
        if sampled[-1] is not knots[-1]:
            sampled.append(knots[-1])
        try:
            urdf = load_urdf()
            path = urdf.path_to(self._link)
        except Exception:  # noqa: BLE001 - sweep needs the live snapshot
            return True, ""
        polyline = [tuple(float(v) for v in urdf.tcp(q, path)[:3, 3])
                    for q in sampled]
        return self._occupancy_ok_points(polyline)

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
                        admitted, feedback_cb, execute_timeout,
                        current_joints=current_joints)
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
                admitted.target_pose["position"], admitted.target_pose["quaternion_xyzw"],
                keep_tool_down=bool(admitted.keep_tool_down))
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
        """Execute joint-path knots. Sweep failures do not fall back to RRT."""
        from luggage_planning.joint_path import JointPathError, validate_joint_path

        def read_joint_knots(segment):
            # [ST09-I] the place_selection_client knot carrier is gone;
            # knots come from the message field or the rebuilt attr.
            points = list(getattr(segment, "joint_waypoints", None) or [])
            if points and hasattr(points[0], "positions"):
                return [[float(v) for v in point.positions]
                        for point in points]
            return [list(knot) for knot in
                    (getattr(segment, "_joint_knots", None) or [])]

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
    # [ST09-I] _cartesian_refine_fallback removed with the generation bypasses.
    # [ST09-I] _align_then_retry removed with the generation bypasses.

    def _execute_segment_inner(self, segment_msg, feedback_cb,
                               execute_timeout, current_joints):
        """Pure execution (ST09-I): joint paths only, no generation.

        Production motion comes from the certified route legs
        (``route_producer`` -> ``SelectPlacePathV2`` -> typed joint_path
        ``MotionSegmentV2``); Cartesian/OMPL generation lives inside the
        route's MoveIt edge builder. A pose/cartesian segment spec here
        is a legacy bypass attempt and is refused with a named code and
        zero execution side effects.
        """
        seg_type = str(segment_msg.type)
        if seg_type == "joint_path":
            return self._run_joint_path(
                segment_msg, feedback_cb, execute_timeout)
        if seg_type in ("pose_target", "cartesian"):
            self._notify(feedback_cb, "refused", segment_msg.name, 0.0,
                         "segment generation removed")
            return self._result(
                False,
                "SEGMENT_GENERATION_REMOVED %s segments are a legacy "
                "bypass; motion comes from the certified route"
                % seg_type, 0.0)
        return SegmentExecResult(
            False, "unknown segment type %r" % seg_type, 0.0)

    def probe_segment(self, segment_msg, start_joints=None):
        """Validation-only dry-run; no generation (ST09-I).

        Joint-path specs validate their knots (times, sizes, wrist/j6
        diagnostics). Pose/cartesian specs are legacy generation probes
        and report a named refusal — the route's MoveIt edge builder is
        the only geometry generator.
        """
        self._reset_boundary_measurements()
        seg_type = str(segment_msg.type)
        record = {
            "name": str(segment_msg.name),
            "type": seg_type,
        }
        if seg_type == "joint_path":
            knots = list(getattr(segment_msg, "joint_waypoints", []) or [])
            record["knots"] = len(knots)
            record["times_ok"] = all(
                b.time_from_start.sec * 10 ** 9 + b.time_from_start.nanosec
                > a.time_from_start.sec * 10 ** 9 + a.time_from_start.nanosec
                for a, b in zip(knots, knots[1:]))
            if start_joints and knots:
                last = knots[-1]
                measurement = self._wrist_check(
                    [list(start_joints), list(last.positions)], JOINTS)
                if measurement is not None:
                    record["wrist_margin"] = dict(measurement)
                j6 = self._j6_check(
                    [list(start_joints), list(last.positions)], JOINTS)
                if j6 is not None:
                    record["j6_rotation"] = dict(j6)
            return record
        record["refused"] = "SEGMENT_GENERATION_REMOVED %s probes are a " \
            "legacy bypass; routes are certified upstream" % seg_type
        return record

    # ------------------------------------------------------------------
    # pose_target
    # [ST09-I] _build_goal_constraints removed with the generation bypasses.
    # [ST09-I] _tool_down_constraint removed with the generation bypasses.
    # [ST09-I] _run_pose_target removed with the generation bypasses.

    def _finish_move_group(self, result, note, segment_msg, feedback_cb,
                           execute_timeout, unimplemented):
        """Close out a MoveGroup result (kept for the preserved
        ``GoToRobotPose`` named-joint-goal path; route legs do not use
        MoveGroup execution)."""
        code = int(result.error_code.val)
        traj = getattr(result, "planned_trajectory", None)
        if traj is not None:
            self._planned_traj = traj
        if code != MoveItErrorCodes.SUCCESS:
            return self._result(
                False, "MoveGroup error_code=%s (%s)" % (code, note),
                error_code=code)
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
    # [ST09-I] _ik_joints removed with the generation bypasses.
    # [ST09-I] _query_ik removed with the generation bypasses.
    # [ST09-I] _log_ik_call removed with the generation bypasses.
    # [ST09-I] _query_ik_inner removed with the generation bypasses.

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
    # [ST09-I] _finish_move_group removed with the generation bypasses.
    # [ST09-I] _cartesian_after_occupancy removed with the generation bypasses.

    def _traj_end_joints(self, robot_traj):
        joint_traj = getattr(robot_traj, "joint_trajectory", robot_traj)
        points = list(getattr(joint_traj, "points", None) or [])
        if not points:
            return None
        names = list(getattr(joint_traj, "joint_names", None) or JOINTS)
        by_name = dict(zip(names, points[-1].positions))
        if not all(name in by_name for name in JOINTS):
            return None
        return [float(by_name[name]) for name in JOINTS]
    # [ST09-I] _execute_seeded_hop removed with the generation bypasses.
    # [ST09-I] _piecewise_cartesian removed with the generation bypasses.

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
    # [ST09-I] _occupancy_ok_plan removed with the generation bypasses.

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
    # [ST09-I] _requested_cartesian_length removed with the generation bypasses.

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

    @staticmethod
    def _as_moveit_pose(pose):
        """GetCartesianPath requires geometry_msgs/Pose, not a pose namespace."""
        if isinstance(pose, PoseMsg):
            return pose
        out = PoseMsg()
        out.position.x = float(pose.position.x)
        out.position.y = float(pose.position.y)
        out.position.z = float(pose.position.z)
        out.orientation.x = float(pose.orientation.x)
        out.orientation.y = float(pose.orientation.y)
        out.orientation.z = float(pose.orientation.z)
        out.orientation.w = float(pose.orientation.w)
        return out
    # [ST09-I] _plan_cartesian removed with the generation bypasses.
    # [ST09-I] _judge_cartesian_response removed with the generation bypasses.
    # [ST09-I] _reject_for_excursion removed with the generation bypasses.

    def _record_wrist(self, response):
        """j5 measurement. The caller refuses a track that is not ok."""
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
