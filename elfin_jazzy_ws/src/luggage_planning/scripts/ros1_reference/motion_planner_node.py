#!/usr/bin/env python3
"""Plan and execute motions via MoveIt (observe reset + container aim)."""

import json
import os
import sys
import threading
import math
import time

import actionlib
import numpy as np
import rospy
import rospkg
import tf2_ros
import yaml
import moveit_commander
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_commander.exception import MoveItCommanderException
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveGroupAction,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import (
    GetPositionFK,
    GetPositionFKRequest,
    GetStateValidity,
    GetStateValidityRequest,
)
from shape_msgs.msg import SolidPrimitive
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    JointTrajectoryControllerState,
)
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Empty, SetBool, Trigger

from luggage_msgs.srv import (
    AimCameraAtContainer,
    AimCameraAtContainerRequest,
    AimCameraAtContainerResponse,
    GoToJointValues,
    GoToJointValuesResponse,
    GoToRobotPose,
    GoToRobotPoseResponse,
    PlanMotion,
    PlanMotionResponse,
    ValidateMotionSequence,
    ValidateMotionSequenceResponse,
)

PLANNING_ROOT = rospkg.RosPack().get_path("luggage_planning")
if os.path.join(PLANNING_ROOT, "scripts") not in sys.path:
    sys.path.insert(0, os.path.join(PLANNING_ROOT, "scripts"))

DESC_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_description"), "scripts")
if DESC_SCRIPTS not in sys.path:
    sys.path.insert(0, DESC_SCRIPTS)

from container_aim_utils import (  # noqa: E402
    build_joint_seeds,
    optical_pose_look_at,
    pick_closest_joint_solution,
    view_axis_alignment_error,
)
from scene_tf_config_utils import (  # noqa: E402
    container_opening_target_point,
    load_scene_tf_config,
    resolve_scene_tf_config_path,
)
from joint_angle_utils import (  # noqa: E402
    WRAP_EQUIVALENT_JOINTS,
    format_rewrites,
    normalize_joint_targets,
)
from exploration_config_utils import (  # noqa: E402
    default_exploration_path,
    downward_constraints_config,
    load_exploration_config,
)
from downward_constraint_utils import (  # noqa: E402
    compute_downward_orientations,
    feasibility_check,
    link_z_tilt_deg,
    validate_downward_tilts,
)
from settle_criterion import (  # noqa: E402
    DISPLACEMENT,
    STRICT_QUANTILE,
    SettleTracker,
    format_diagnostics,
)

DEFAULT_JOINT_NAMES = [
    "elfin_joint1",
    "elfin_joint2",
    "elfin_joint3",
    "elfin_joint4",
    "elfin_joint5",
    "elfin_joint6",
]

OPTICAL_FRAME = "camera_depth_optical_frame"
LINK6_FRAME = "elfin_link6"
BASE_FRAME = "elfin_base_link"


def _load_poses_config():
    path = rospy.get_param(
        "~robot_poses_config",
        os.path.join(
            rospkg.RosPack().get_path("luggage_description"),
            "config",
            "robot_poses.yaml.example",
        ),
    )
    with open(path, "r") as handle:
        return yaml.safe_load(handle)


class _ExecutionJointLogger:
    """Periodically log per-joint desired/actual/error while a trajectory runs.

    Subscribes to JointTrajectoryControllerState (already published by
    elfin_arm_controller) and emits one log line per tick at *rate_hz*.
    The format prints each joint inline so a single execution shows up as a
    contiguous block in /rosout — easy to grep & plot afterwards.

    Lifecycle:
        sampler = _ExecutionJointLogger(...)
        sampler.start()      # before group.execute()
        # ... execute runs ...
        sampler.stop()       # after execute returns
    """

    def __init__(self, topic, rate_hz):
        self._topic = topic
        self._rate_hz = max(0.1, float(rate_hz))
        self._period = 1.0 / self._rate_hz
        self._latest = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._sub = None
        self._thread = None
        self._tick = 0
        self._t0 = None

    def _on_state(self, msg):
        with self._lock:
            self._latest = msg

    def start(self):
        if self._rate_hz <= 0.0:
            return
        self._stop_event.clear()
        self._latest = None
        self._tick = 0
        self._t0 = rospy.Time.now()
        self._sub = rospy.Subscriber(
            self._topic, JointTrajectoryControllerState, self._on_state, queue_size=1
        )
        self._thread = threading.Thread(target=self._run, name="exec_joint_logger")
        self._thread.daemon = True
        self._thread.start()

    def stop(self):
        if self._thread is None:
            return
        self._stop_event.set()
        self._thread.join(timeout=self._period + 0.5)
        if self._sub is not None:
            self._sub.unregister()
            self._sub = None
        self._thread = None

    def _run(self):
        # rospy.Rate is the right primitive even from a non-main thread; it
        # uses rospy.sleep which honours simulated /clock when use_sim_time=true.
        rate = rospy.Rate(self._rate_hz)
        while not self._stop_event.is_set() and not rospy.is_shutdown():
            self._emit()
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break
        # One final sample on stop so the last known state is in the log.
        self._emit(suffix=" [final]")

    def _emit(self, suffix=""):
        with self._lock:
            msg = self._latest
        if msg is None:
            return
        self._tick += 1
        elapsed = (rospy.Time.now() - self._t0).to_sec() if self._t0 else 0.0
        parts = []
        for i, name in enumerate(msg.joint_names):
            try:
                des = msg.desired.positions[i]
                act = msg.actual.positions[i]
                err = msg.error.positions[i]
            except IndexError:
                continue
            parts.append("%s d=%+.4f a=%+.4f e=%+.4f" % (name, des, act, err))
        # Per-tick joint tracking dump. Valuable when diagnosing controller
        # error, useless in a run log; ~exec_log_rate still limits the rate.
        rospy.logdebug(
            "[exec_log t=%.2fs #%d]%s %s",
            elapsed, self._tick, suffix, " | ".join(parts),
        )


class MotionPlanner:
    def __init__(self):
        self._config = _load_poses_config()
        defaults = self._config.get("defaults", {})
        self._default_observe_pose = defaults.get("observe_pose", "observe")
        self._default_tolerance = float(
            self._config.get("poses", {})
            .get(self._default_observe_pose, {})
            .get("tolerance", 0.02)
        )
        self._planning_time = float(defaults.get("planning_time", 10.0))
        self._max_vel = float(defaults.get("max_velocity_scaling", 0.3))
        self._max_acc = float(defaults.get("max_acceleration_scaling", 0.3))
        self._robot_description = rospy.get_param("~robot_description", "robot_description")
        self._move_group_action = rospy.get_param("~move_group_action", "/move_group")
        self._move_group_wait_timeout = float(rospy.get_param("~move_group_wait_timeout", 60.0))
        self._execute_timeout = float(rospy.get_param("~execute_timeout", 45.0))
        self._release_settle_vel_tol = float(rospy.get_param(
            "~release_settle_velocity_tolerance", 0.03))
        self._release_settle_timeout = float(rospy.get_param(
            "~release_settle_timeout", 3.0))
        # Which signal proves "not moving", not how fast is too fast.
        # displacement (default) measures the movement itself; velocity reads
        # /joint_states.velocity, which under Gazebo's kinematic position mode
        # carries a static, pose-dependent bias up to ~0.03 rad/s on a
        # provably motionless arm (phase 8). Keep velocity only for comparing
        # against the historical behaviour.
        self._settle_criterion = rospy.get_param(
            "~settle_criterion", DISPLACEMENT)
        self._settle_quantile = float(rospy.get_param(
            "~settle_quantile", STRICT_QUANTILE))
        self._exec_log_rate = float(rospy.get_param("~exec_log_rate", 1.0))
        self._exec_log_state_topic = rospy.get_param(
            "~exec_log_controller_state_topic", "/S20/elfin_arm_controller/state"
        )
        self._tf_wait_timeout = float(rospy.get_param("~tf_wait_timeout", 5.0))
        self._scene_tf_config_path = rospy.get_param(
            "~scene_tf_config",
            rospy.get_param("/luggage/scene_tf_config", resolve_scene_tf_config_path()),
        )
        self._optical_frame = rospy.get_param("~optical_frame", OPTICAL_FRAME)
        self._link6_frame = rospy.get_param("~link6_frame", LINK6_FRAME)
        self._pose_target_link = rospy.get_param("~pose_target_link", self._link6_frame)
        self._pick_pose_target_link = rospy.get_param(
            "~pick_pose_target_link", "suction_contact_frame"
        )
        self._pick_segments_use_contact_frame = set(
            s.strip()
            for s in rospy.get_param(
                "~pick_segments_use_contact_frame", "pre_grasp,approach,attach,pick_retreat,stage_mid,stage_late,stage,retreat"
                ",transit,traverse,insert,descend"
            ).split(",")
            if s.strip()
        )
        self._pose_target_frame = rospy.get_param("~pose_target_frame", "world")
        self._base_frame = rospy.get_param("~base_frame", BASE_FRAME)
        # Cartesian (straight-line) descent params for pick approach/attach/retreat.
        # eef_step = max EE interpolation step (m); min_fraction = required
        # fraction-reached to accept the cartesian path, else fall back to OMPL.
        # (This Noetic moveit_commander's compute_cartesian_path has no
        # jump_threshold arg; the C++ binding uses its default.)
        self._cartesian_eef_step = float(rospy.get_param("~cartesian_eef_step", 0.01))
        self._cartesian_min_fraction = float(
            rospy.get_param("~cartesian_min_fraction", 0.95)
        )
        self._replan_max_attempts = int(rospy.get_param("~replan_max_attempts", 2))
        self._replan_delay = float(rospy.get_param("~replan_delay", 1.0))
        self._tool_down_rp_tolerance = float(rospy.get_param("~tool_down_roll_pitch_tolerance", 0.15))
        self._tool_down_yaw_tolerance = float(rospy.get_param("~tool_down_yaw_tolerance", 0.10))
        self._tool_down_planning_time = float(rospy.get_param("~tool_down_planning_time", 15.0))
        self._constrained_planning_time_cap = float(rospy.get_param(
            "~constrained_planning_time_cap", 15.0))
        self._tool_down_yaw_relax_steps = [
            float(v) for v in rospy.get_param("~tool_down_yaw_relax_steps", [0.20, 0.35])
        ]
        self._wrist_max_rotation_threshold = float(rospy.get_param("~wrist_max_rotation_threshold", 1.5))
        self._wrist_j5_j6_rotation_threshold = float(rospy.get_param("~wrist_j5_j6_rotation_threshold", 1.0))
        self._wrist_max_step_threshold = float(rospy.get_param("~wrist_max_step_threshold", 0.5))
        # Relaxed wrist thresholds for dual-down segments (smart explore,
        # place transit): the camera/suction orientation constraints are the
        # real flip-guard, so allow larger reorientation/yaw swings (up to pi)
        # while still rejecting genuine 2pi branch-jump loops.
        self._downward_wrist_j4_threshold = float(
            rospy.get_param("~downward_wrist_j4_threshold", 2.5))
        self._downward_wrist_j5_j6_threshold = float(
            rospy.get_param("~downward_wrist_j5_j6_threshold", math.pi))
        self._downward_wrist_step_threshold = float(
            rospy.get_param("~downward_wrist_step_threshold", 1.0))
        self._camera_down_rp_tolerance = float(rospy.get_param("~camera_down_rp_tolerance", 0.12))
        self._camera_down_yaw_tolerance = float(rospy.get_param("~camera_down_yaw_tolerance", 0.10))
        self._camera_down_planning_time = float(rospy.get_param("~camera_down_planning_time", 15.0))
        self._camera_down_yaw_relax_steps = [
            float(v) for v in rospy.get_param("~camera_down_yaw_relax_steps", [0.20, 0.35])
        ]
        self._lock_wrist_joints = [
            s.strip()
            for s in rospy.get_param(
                "~lock_wrist_joints", "elfin_joint4,elfin_joint5,elfin_joint6"
            ).split(",")
        ]
        self._lock_wrist_tolerance_j4 = float(rospy.get_param("~lock_wrist_tolerance_j4", 0.40))
        self._lock_wrist_tolerance_j5_j6 = float(rospy.get_param("~lock_wrist_tolerance_j5_j6", 0.20))
        self._lock_wrist_relax_steps = [
            float(v) for v in rospy.get_param("~lock_wrist_relax_steps", [0.35, 0.50])
        ]
        # Dual camera-down + suction-down path constraints (smart explore +
        # place transit). Loaded from the exploration config so this planner
        # and cargo_exploration_planner share one source of truth.
        exploration_path = rospy.get_param(
            "~exploration_config", default_exploration_path()
        )
        try:
            self._downward_cfg = downward_constraints_config(
                load_exploration_config(exploration_path)
            )
        except Exception as exc:
            rospy.logwarn(
                "downward_constraints: cannot load exploration config: %s", exc)
            self._downward_cfg = downward_constraints_config({})
        self._suction_frame = rospy.get_param("~suction_frame", "suction_contact_frame")
        self._downward_primary_constraint = str(
            self._downward_cfg.get("primary_constraint", "suction")).lower()
        if self._downward_primary_constraint != "suction":
            rospy.logwarn(
                "Unsupported downward primary_constraint=%s; using suction",
                self._downward_primary_constraint,
            )
            self._downward_primary_constraint = "suction"
        self._strict_downward = bool(
            rospy.get_param("~strict_downward", self._downward_cfg["strict"])
        )
        self._downward_validate = bool(
            rospy.get_param(
                "~downward_validate_trajectory",
                self._downward_cfg["validate_trajectory"],
            )
        )
        self._camera_max_tilt_rad = math.radians(
            self._downward_cfg["camera_max_tilt_deg"])
        self._suction_max_tilt_rad = math.radians(
            self._downward_cfg["suction_max_tilt_deg"])
        self._compute_fk_service = rospy.get_param("~compute_fk_service", "/compute_fk")
        self._downward_fk_stride = int(rospy.get_param("~downward_fk_stride", 5))
        self._downward_orientations_cache = None
        self._fk_proxy = None
        self._refresh_dynamic_scene_enabled = bool(
            rospy.get_param("~refresh_dynamic_scene_enabled", True)
        )
        self._pick_segments_skip_dynamic_refresh = set(
            s.strip()
            for s in rospy.get_param(
                "~pick_segments_skip_dynamic_refresh", "pre_grasp,approach,attach,pick_retreat"
            ).split(",")
            if s.strip()
        )
        self._current_segment_name = ""
        self._group = None
        self._move_group_ready = False

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._target_pub = rospy.Publisher(
            "/luggage/debug/planning_targets", MarkerArray, queue_size=1, latch=True
        )
        self._fail_pub = rospy.Publisher(
            "/luggage/debug/planning_failures", MarkerArray, queue_size=1, latch=True
        )
        # MoveIt never renders the path constraints it is given. Publish both
        # the raw message (echo-able, bag-able) and a geometric rendering so a
        # failed plan can be explained after the fact.
        self._constraints_pub = rospy.Publisher(
            "/luggage/debug/active_path_constraints", Constraints,
            queue_size=1, latch=True,
        )
        self._constraint_marker_pub = rospy.Publisher(
            "/luggage/debug/constraint_markers", MarkerArray,
            queue_size=1, latch=True,
        )
        self._diagnostics_pub = rospy.Publisher(
            "/luggage/debug/planning_diagnostics_json", String,
            queue_size=1, latch=True,
        )
        self._marker_id = 0
        self._last_tilt_profile = {}
        rospy.set_param("/luggage/trajectory_tilt_history", [])
        self._state_validity_srv = None
        # Segment-scoped suction_panel<->box ACM toggle. Segments after pre_grasp
        # (approach/attach/retreat) ask scene_manager to ALLOW the touch so the
        # panel can descend to / lift off the box; pre_grasp and everything else
        # keep the panel colliding with the box (default enforced).
        self._set_pickup_touch_srv = rospy.ServiceProxy(
            "/scene_manager/set_pickup_touch", SetBool
        )
        self._set_place_support_touch_srv = rospy.ServiceProxy(
            "/scene_manager/set_place_support_touch", SetBool
        )
        self._pickup_touch_segments = set(
            s.strip()
            for s in rospy.get_param(
                "~pickup_touch_segments", "approach,attach,pick_retreat"
            ).split(",")
            if s.strip()
        )
        self._place_support_touch_segments = set(
            s.strip() for s in rospy.get_param(
                "~place_support_touch_segments", "descend").split(",")
            if s.strip()
        )
        # Segments that need a fresh octomap right before they run: the contact
        # descent (approach/attach) must reach the box top, but the octomap
        # (2Hz, cumulative) keeps re-inserting residual box-surface voxels even
        # after pickup_box_pointcloud_filter strips most of them (handle ridges,
        # OBB-edge points, 2.5cm voxel quantization). Clearing the octomap
        # immediately before the descent removes those stale voxels so the cup
        # is not blocked from contacting the box. The box itself stays a valid
        # collision object via current_pickup_box (+ ACM for the panel).
        self._clear_octomap_segments = set(
            s.strip()
            for s in rospy.get_param(
                "~clear_octomap_segments", "approach,attach"
            ).split(",")
            if s.strip()
        )
        self._clear_octomap_srv = rospy.ServiceProxy("/clear_octomap", Empty)

        self._run_startup_wrap_diagnostics()

    def _run_startup_wrap_diagnostics(self):
        """Warn if joint_states / controller desired sit on different 2π branches.

        When the spawn pose, observe target and controller desired drift onto
        different equivalent branches for the wrap joints (elfin_joint1/4/5/6),
        ros_control follows the numerical delta and spins the joint a whole
        turn. We snapshot both sources and compare against the configured
        observe pose so the mismatch is loud at startup instead of showing up
        as a "PID problem" later.
        """
        topic = rospy.get_param(
            "~startup_diag_controller_state",
            "/S20/elfin_arm_controller/state",
        )
        timeout = float(rospy.get_param("~startup_diag_timeout", 5.0))
        try:
            js = rospy.wait_for_message("/joint_states", JointState, timeout=timeout)
        except rospy.ROSException as exc:
            rospy.logdebug("Wrap diag: /joint_states unavailable: %s", exc)
            return
        cs = self._snapshot_controller_state(topic=topic, timeout=timeout)

        try:
            observe_cfg = self._pose_config(self._default_observe_pose)
        except KeyError:
            observe_cfg = None

        actual = dict(zip(js.name, js.position))
        rows = []
        wrap_window = 2.0 * math.pi
        garbage_desired_threshold = 20.0
        for name in WRAP_EQUIVALENT_JOINTS:
            if name not in actual:
                continue
            desired_raw = cs[name]["desired"] if cs and name in cs else None
            if desired_raw is not None and abs(desired_raw) > garbage_desired_threshold:
                desired_val = None
            else:
                desired_val = desired_raw
            row = {
                "joint": name,
                "actual": actual[name],
                "desired": desired_val,
                "observe": None,
            }
            if observe_cfg is not None:
                try:
                    idx = observe_cfg["joints"].index(name)
                    row["observe"] = float(observe_cfg["values"][idx])
                except (ValueError, KeyError):
                    pass

            mismatch = False
            for key in ("desired", "observe"):
                ref = row[key]
                if ref is None:
                    continue
                if abs(abs(row["actual"] - ref) - wrap_window) < 0.3:
                    mismatch = True
                    break
            row["mismatch"] = mismatch
            rows.append(row)

        any_mismatch = any(r["mismatch"] for r in rows)
        if any_mismatch:
            rospy.logwarn(
                "Wrap-branch mismatch detected on startup — controller will spin "
                "this joint a full turn even though pose is identical. Re-check "
                "robot_poses.yaml.example and gazebo_robot_spawner OBSERVE_JOINTS."
            )
        for row in rows:
            level = rospy.logwarn if row["mismatch"] else rospy.logdebug
            if row["desired"] is None and cs and row["joint"] in cs:
                desired_str = "uninitialized"
            elif row["desired"] is not None:
                desired_str = "%.4f" % row["desired"]
            else:
                desired_str = "n/a"
            level(
                "wrap-diag %s actual=%.4f desired=%s observe=%s %s",
                row["joint"],
                row["actual"],
                desired_str,
                ("%.4f" % row["observe"]) if row["observe"] is not None else "n/a",
                "MISMATCH" if row["mismatch"] else "ok",
            )

    def _next_marker_id(self):
        self._marker_id += 1
        return self._marker_id

    def _refresh_dynamic_scene(self):
        """Ask dynamic_scene_manager to sync latest obstacles before replanning."""
        if not self._refresh_dynamic_scene_enabled:
            rospy.loginfo("Dynamic scene refresh skipped: disabled by parameter")
            return False
        if self._current_segment_name in self._pick_segments_skip_dynamic_refresh:
            rospy.loginfo(
                "Dynamic scene refresh skipped for pick segment %s; using explicit pickup box scene",
                self._current_segment_name,
            )
            return False
        try:
            sync = rospy.ServiceProxy("/dynamic_scene_manager/sync_dynamic_scene", Trigger)
            sync.wait_for_service(timeout=2.0)
            resp = sync()
            rospy.loginfo("Dynamic scene refreshed: %s", resp.message)
            return True
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn("Cannot refresh dynamic scene: %s", exc)
            return False

    def _publish_target_marker(self, pose, segment_name, link_name, frame_id,
                               success, extra_text=""):
        """Publish a target marker (green if success, red if failure)."""
        ma = MarkerArray()
        stamp = rospy.Time.now()
        color = (
            ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9)
            if success
            else ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9)
        )

        sphere = Marker()
        sphere.header.frame_id = frame_id
        sphere.header.stamp = stamp
        sphere.ns = "plan_target"
        sphere.id = self._next_marker_id()
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose = pose
        sphere.scale.x = 0.09
        sphere.scale.y = 0.09
        sphere.scale.z = 0.09
        sphere.color = color
        sphere.lifetime = rospy.Duration(60.0)
        ma.markers.append(sphere)

        label = Marker()
        label.header.frame_id = frame_id
        label.header.stamp = stamp
        label.ns = "plan_target_label"
        label.id = self._next_marker_id()
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position = Point(
            x=pose.position.x, y=pose.position.y, z=pose.position.z + 0.16
        )
        label.pose.orientation = Quaternion(w=1.0)
        label.scale.z = 0.07
        label.text = "%s [%s]" % (segment_name, link_name)
        if extra_text:
            label.text += "\n%s" % extra_text
        label.color = color
        label.lifetime = rospy.Duration(60.0)
        ma.markers.append(label)

        self._target_pub.publish(ma)

    def _publish_failure_marker(self, pose, segment_name, frame_id, error_code, message):
        """Publish failure marker with error information."""
        ma = MarkerArray()
        stamp = rospy.Time.now()
        red = ColorRGBA(r=1.0, g=0.1, b=0.1, a=1.0)

        sphere = Marker()
        sphere.header.frame_id = frame_id
        sphere.header.stamp = stamp
        sphere.ns = "plan_fail"
        sphere.id = self._next_marker_id()
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose = pose
        sphere.scale.x = 0.12
        sphere.scale.y = 0.12
        sphere.scale.z = 0.12
        sphere.color = red
        sphere.lifetime = rospy.Duration(120.0)
        ma.markers.append(sphere)

        label = Marker()
        label.header.frame_id = frame_id
        label.header.stamp = stamp
        label.ns = "plan_fail_label"
        label.id = self._next_marker_id()
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position = Point(
            x=pose.position.x, y=pose.position.y, z=pose.position.z + 0.22
        )
        label.pose.orientation = Quaternion(w=1.0)
        label.scale.z = 0.08
        label.text = "FAIL %s err=%s\n%s" % (
            segment_name, error_code, message[:220])
        label.color = red
        label.lifetime = rospy.Duration(120.0)
        ma.markers.append(label)

        self._fail_pub.publish(ma)

    def _check_state_validity(self, joint_names, joint_values, group_name="elfin_arm"):
        """Check if a robot state is valid (collision-free). Returns (valid, contacts_info)."""
        if self._state_validity_srv is None:
            try:
                rospy.wait_for_service("/check_state_validity", timeout=2.0)
                self._state_validity_srv = rospy.ServiceProxy(
                    "/check_state_validity", GetStateValidity
                )
            except rospy.ROSException:
                return True, "state validity service unavailable"
        try:
            req = GetStateValidityRequest()
            req.robot_state.joint_state.name = list(joint_names)
            req.robot_state.joint_state.position = list(joint_values)
            req.group_name = group_name
            resp = self._state_validity_srv(req)
            if resp.valid:
                return True, "state valid"
            contact_names = []
            for contact in resp.contacts[:5]:
                contact_names.append(
                    "%s <-> %s" % (contact.contact_body_1, contact.contact_body_2)
                )
            info = "state invalid: %s" % "; ".join(contact_names) if contact_names else "state invalid (no contact details)"
            return False, info
        except rospy.ServiceException as exc:
            rospy.logwarn("check_state_validity call failed: %s", exc)
            return True, "check failed: %s" % exc

    def _publish_validity_marker(self, pose, frame_id, info):
        """Publish a text marker with state validity diagnostic."""
        ma = MarkerArray()
        stamp = rospy.Time.now()
        label = Marker()
        label.header.frame_id = frame_id
        label.header.stamp = stamp
        label.ns = "state_validity"
        label.id = self._next_marker_id()
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position = Point(
            x=pose.position.x, y=pose.position.y, z=pose.position.z + 0.30
        )
        label.pose.orientation = Quaternion(w=1.0)
        label.scale.z = 0.07
        label.text = info[:80]
        label.color = ColorRGBA(r=1.0, g=0.3, b=0.3, a=1.0)
        label.lifetime = rospy.Duration(60.0)
        ma.markers.append(label)
        self._fail_pub.publish(ma)

    @staticmethod
    def _rotate_vector(quaternion, vector):
        """Rotate a vector by a geometry_msgs Quaternion."""
        q = np.array([
            quaternion.x, quaternion.y, quaternion.z, quaternion.w],
            dtype=float)
        norm = float(np.linalg.norm(q))
        if norm < 1e-12:
            return np.array(vector, dtype=float)
        x, y, z, w = q / norm
        matrix = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        return matrix.dot(np.array(vector, dtype=float))

    def _link_position(self, link_name, frame_id):
        """Return the current link origin in frame_id, or None when TF is cold."""
        try:
            transform = self._tf_buffer.lookup_transform(
                frame_id, link_name, rospy.Time(0), rospy.Duration(0.3))
        except tf2_ros.TransformException:
            return None
        translation = transform.transform.translation
        return np.array(
            [translation.x, translation.y, translation.z], dtype=float)

    def _append_constraint_cone(
            self, ma, stamp, frame_id, apex, axis, half_angle, ns, marker_id,
            color, length=0.30, segments=24):
        """Draw the admissible-tilt cone of an OrientationConstraint."""
        axis = np.array(axis, dtype=float)
        norm = float(np.linalg.norm(axis))
        if norm < 1e-9:
            return
        axis = axis / norm
        # Beyond ~85 deg the cone degenerates into a half space; clamp so the
        # rendering stays finite while still reading as "almost unconstrained".
        half_angle = min(max(float(half_angle), 0.0), math.radians(85.0))
        radius = length * math.tan(half_angle)
        reference = (
            np.array([1.0, 0.0, 0.0])
            if abs(axis[2]) > 0.9 else np.array([0.0, 0.0, 1.0]))
        side = np.cross(axis, reference)
        side /= float(np.linalg.norm(side))
        other = np.cross(axis, side)
        center = apex + axis * length

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation = Quaternion(w=1.0)
        marker.scale.x = 0.004
        marker.color = color
        marker.lifetime = rospy.Duration(120.0)
        rim = []
        for i in range(segments):
            angle = 2.0 * math.pi * i / float(segments)
            rim.append(
                center
                + side * (radius * math.cos(angle))
                + other * (radius * math.sin(angle)))
        for i, point in enumerate(rim):
            if i % 3 == 0:
                marker.points.append(
                    Point(x=apex[0], y=apex[1], z=apex[2]))
                marker.points.append(
                    Point(x=point[0], y=point[1], z=point[2]))
            nxt = rim[(i + 1) % segments]
            marker.points.append(Point(x=point[0], y=point[1], z=point[2]))
            marker.points.append(Point(x=nxt[0], y=nxt[1], z=nxt[2]))
        ma.markers.append(marker)

    def _publish_constraint_markers(
            self, constraints, segment_name, mode_desc, level_idx, anchor_pose,
            anchor_frame):
        """Render orientation cones, position regions and joint limits."""
        ma = MarkerArray()
        stamp = rospy.Time.now()
        clear = Marker()
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)
        if constraints is None:
            self._constraint_marker_pub.publish(ma)
            return

        cone_color = ColorRGBA(r=0.2, g=0.7, b=1.0, a=0.9)
        free_yaw_color = ColorRGBA(r=1.0, g=0.9, b=0.2, a=0.8)
        for idx, oc in enumerate(constraints.orientation_constraints):
            frame_id = oc.header.frame_id or self._base_frame
            apex = self._link_position(oc.link_name, frame_id)
            if apex is None:
                continue
            axis = self._rotate_vector(oc.orientation, (0.0, 0.0, 1.0))
            half_angle = max(
                float(oc.absolute_x_axis_tolerance),
                float(oc.absolute_y_axis_tolerance))
            self._append_constraint_cone(
                ma, stamp, frame_id, apex, axis, half_angle,
                "constraint_orientation", idx, cone_color)
            yaw_free = float(oc.absolute_z_axis_tolerance) >= math.radians(179.0)
            self._append_text_marker(
                ma, stamp, frame_id, apex + axis * 0.34,
                "%s +Z <= %.1fdeg\nyaw %s" % (
                    oc.link_name, math.degrees(half_angle),
                    "free" if yaw_free
                    else "<= %.1fdeg" % math.degrees(
                        oc.absolute_z_axis_tolerance)),
                "constraint_orientation_label", idx, 0.045,
                free_yaw_color if yaw_free else cone_color)

        region_color = ColorRGBA(r=1.0, g=0.4, b=0.8, a=0.25)
        marker_id = 0
        for pc in constraints.position_constraints:
            frame_id = pc.header.frame_id or self._base_frame
            region = pc.constraint_region
            for primitive, primitive_pose in zip(
                    region.primitives, region.primitive_poses):
                marker = Marker()
                marker.header.frame_id = frame_id
                marker.header.stamp = stamp
                marker.ns = "constraint_position"
                marker.id = marker_id
                marker.action = Marker.ADD
                marker.pose = primitive_pose
                marker.color = region_color
                marker.lifetime = rospy.Duration(120.0)
                if primitive.type == SolidPrimitive.BOX:
                    marker.type = Marker.CUBE
                    marker.scale.x = float(primitive.dimensions[0])
                    marker.scale.y = float(primitive.dimensions[1])
                    marker.scale.z = float(primitive.dimensions[2])
                elif primitive.type == SolidPrimitive.SPHERE:
                    marker.type = Marker.SPHERE
                    diameter = 2.0 * float(primitive.dimensions[0])
                    marker.scale.x = diameter
                    marker.scale.y = diameter
                    marker.scale.z = diameter
                else:
                    continue
                ma.markers.append(marker)
                marker_id += 1

        lines = ["%s level=%d\n%s" % (segment_name, level_idx, mode_desc)]
        for jc in constraints.joint_constraints:
            lines.append("%s: %.3f +%.3f/-%.3f" % (
                jc.joint_name, jc.position,
                jc.tolerance_above, jc.tolerance_below))
        anchor = np.array([
            anchor_pose.position.x,
            anchor_pose.position.y,
            anchor_pose.position.z + 0.40,
        ])
        self._append_text_marker(
            ma, stamp, anchor_frame, anchor, "\n".join(lines),
            "constraint_summary", 0, 0.05,
            ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0))
        self._constraint_marker_pub.publish(ma)

    def _append_text_marker(
            self, ma, stamp, frame_id, position, text, ns, marker_id, scale,
            color, lifetime=120.0):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position = Point(
            x=float(position[0]), y=float(position[1]), z=float(position[2]))
        marker.pose.orientation = Quaternion(w=1.0)
        marker.scale.z = scale
        marker.color = color
        marker.text = text
        marker.lifetime = rospy.Duration(lifetime)
        ma.markers.append(marker)

    def _publish_active_constraints(
            self, constraints, segment_name, mode_desc, level_idx, anchor_pose,
            anchor_frame):
        """Publish the raw Constraints message plus its geometric rendering."""
        try:
            self._constraints_pub.publish(
                constraints if constraints is not None else Constraints())
            self._publish_constraint_markers(
                constraints, segment_name, mode_desc, level_idx, anchor_pose,
                anchor_frame)
        except Exception as exc:  # visualization must never break planning
            rospy.logwarn_throttle(
                5.0, "constraint visualization failed: %s", exc)

    def _publish_planning_diagnostics(self, diagnostics):
        """Publish one structured planning record for bags and offline triage."""
        try:
            self._diagnostics_pub.publish(
                String(data=json.dumps(diagnostics, sort_keys=True)))
        except Exception as exc:
            rospy.logwarn_throttle(
                5.0, "planning diagnostics publish failed: %s", exc)

    @staticmethod
    def _endpoint_classification(endpoint_diag):
        """Pull the classification token out of the endpoint diagnostic text."""
        if not endpoint_diag or not endpoint_diag.startswith("classification="):
            return "unconstrained"
        return endpoint_diag.split(";", 1)[0].split("=", 1)[1].strip()

    def _diagnose_start_state(self, group, label, err_val):
        """Log start-state contacts when a plan fails with START_STATE_IN_COLLISION.

        Called from joint- and pose-target planning paths whenever MoveIt
        returns a negative error code (notably -10 = START_STATE_IN_COLLISION).
        Queries /check_state_validity for the *current* joint state and logs
        the contact body pairs so the operator can tell whether the invalid
        state comes from robot self-collision, static scene geometry
        (pedestal/container), dynamic obstacles (dyn_obs_*/octomap), or the
        pickup box.

        Returns a short string suitable for appending to the failure message:
        ``"start-state contacts: A <-> B; C <-> D"`` or ``""`` when the state
        is valid or the service is unavailable.
        """
        try:
            contacts_only = (err_val == MoveItErrorCodes.START_STATE_IN_COLLISION)
        except (AttributeError, TypeError):
            contacts_only = False
        try:
            err_int = int(err_val)
        except (TypeError, ValueError):
            err_int = 0
        # Always diagnose on START_STATE_IN_COLLISION; for other negative
        # errors (e.g. GOAL_IN_COLLISION=-12) we still check the start state
        # because MoveIt sometimes reports a goal error when the start state
        # itself is the real offender.
        if not contacts_only and err_int >= 0:
            return ""

        joint_names = list(group.get_active_joints())
        joint_values = list(group.get_current_joint_values())
        if len(joint_names) != len(joint_values):
            return ""
        valid, info = self._check_state_validity(joint_names, joint_values)
        if valid:
            return ""
        rospy.logwarn(
            "[%s] Start-state invalid (plan error=%s): %s",
            label, err_val, info,
        )
        return " [start-state: %s]" % info

    def _move_group_startup_hint(self):
        return (
            "Check: rosnode list | grep move_group; "
            "rosaction list | grep move_group; "
            "rosparam get /move_group/robot_description | head -3. "
            "Start stack: roslaunch luggage_bringup inspect_container.launch"
        )

    def _wait_for_move_group_action(self):
        if self._move_group_ready:
            return
        client = actionlib.SimpleActionClient(self._move_group_action, MoveGroupAction)
        rospy.loginfo(
            "Waiting for move_group action server %s (timeout %.0fs) ...",
            self._move_group_action,
            self._move_group_wait_timeout,
        )
        if not client.wait_for_server(rospy.Duration(self._move_group_wait_timeout)):
            raise MoveItCommanderException(
                "%s\n%s"
                % (
                    "move_group action server %s not available within %.0fs"
                    % (self._move_group_action, self._move_group_wait_timeout),
                    self._move_group_startup_hint(),
                )
            )
        self._move_group_ready = True
        rospy.loginfo("move_group action server ready")

    def _ensure_move_group(self, group_name="elfin_arm"):
        if self._group is not None:
            return self._group
        self._wait_for_move_group_action()
        moveit_commander.roscpp_initialize([])
        self._group = moveit_commander.MoveGroupCommander(
            group_name, robot_description=self._robot_description
        )
        self._group.set_planning_time(self._planning_time)
        self._group.set_num_planning_attempts(3)
        self._group.set_max_velocity_scaling_factor(self._max_vel)
        self._group.set_max_acceleration_scaling_factor(self._max_acc)
        return self._group

    def _pose_config(self, pose_name):
        poses = self._config.get("poses", {})
        if pose_name not in poses:
            raise KeyError("Unknown pose '%s'" % pose_name)
        return poses[pose_name]

    @staticmethod
    def _ordered_joint_values(group, joint_names):
        current = dict(zip(group.get_active_joints(), group.get_current_joint_values()))
        return [current.get(name, 0.0) for name in joint_names]

    @staticmethod
    def _within_tolerance(current, target, tolerance):
        if len(current) != len(target):
            return False
        return all(abs(c - t) <= tolerance for c, t in zip(current, target))

    def _nearest_equivalent_targets(self, joint_names, current_values, target_values, label):
        adjusted, rewrites = normalize_joint_targets(
            joint_names,
            current_values,
            target_values,
            wrap_joints=WRAP_EQUIVALENT_JOINTS,
        )
        if rewrites:
            rospy.loginfo(
                "Adjusted wrapped joint targets for %s: %s",
                label,
                format_rewrites(rewrites),
            )
        return adjusted

    def _lookup_xyz(self, target_frame, source_frame=None):
        source_frame = source_frame or self._base_frame
        transform = self._tf_buffer.lookup_transform(
            source_frame,
            target_frame,
            rospy.Time(0),
            rospy.Duration(2.0),
        )
        t = transform.transform.translation
        return [t.x, t.y, t.z]

    def _resolve_opening_xyz(self, container_frame):
        """Return opening center [x,y,z] in elfin_base_link (TF first, then scene config)."""
        frame = container_frame.strip() if container_frame else "container_opening_frame"
        try:
            if self._tf_buffer.can_transform(
                self._base_frame,
                frame,
                rospy.Time(0),
                rospy.Duration(self._tf_wait_timeout),
            ):
                xyz = self._lookup_xyz(frame)
                rospy.loginfo("Opening from TF %s in %s: %s", frame, self._base_frame, xyz)
                return xyz
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            rospy.logwarn("TF lookup for opening failed: %s", exc)

        if frame in ("container_opening_frame", "container_link"):
            scene_config = load_scene_tf_config(self._scene_tf_config_path)
            xyz = container_opening_target_point(scene_config)
            rospy.logwarn(
                "Using opening xyz from scene config %s (no TF %s -> %s)",
                self._scene_tf_config_path,
                self._base_frame,
                frame,
            )
            return xyz

        raise tf2_ros.LookupException(
            "Cannot resolve frame '%s' (start container_tf_publisher or use container_opening_frame)"
            % frame
        )

    def _lookup_optical_z_axis(self):
        transform = self._tf_buffer.lookup_transform(
            self._base_frame,
            self._optical_frame,
            rospy.Time(0),
            rospy.Duration(2.0),
        )
        q = transform.transform.rotation
        # Rotate unit Z by quaternion.
        x, y, z, w = q.x, q.y, q.z, q.w
        return [
            2.0 * (x * z + w * y),
            2.0 * (y * z - w * x),
            1.0 - 2.0 * (x * x + y * y),
        ]

    def _robot_state_from_joints(self, joint_names, values):
        state = RobotState()
        state.joint_state.name = list(joint_names)
        state.joint_state.position = list(values)
        return state

    def _build_link6_position_constraint(self, xyz, xy_tol, z_tol):
        constraint = PositionConstraint()
        constraint.header.frame_id = self._base_frame
        constraint.link_name = self._link6_frame
        constraint.target_point_offset.x = 0.0
        constraint.target_point_offset.y = 0.0
        constraint.target_point_offset.z = 0.0
        region = constraint.constraint_region
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [xy_tol * 2.0, xy_tol * 2.0, z_tol * 2.0]
        region.primitives.append(box)
        pose = Pose()
        pose.position = Point(x=xyz[0], y=xyz[1], z=xyz[2])
        pose.orientation = Quaternion(w=1.0)
        region.primitive_poses.append(pose)
        constraint.weight = 1.0
        return constraint

    def _build_joint_constraints(self, observe_values, joint_names, joint1_tol, other_tol):
        constraints = []
        for name, ref in zip(joint_names, observe_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = ref
            tol = joint1_tol if name == "elfin_joint1" else other_tol
            jc.tolerance_above = tol
            jc.tolerance_below = tol
            jc.weight = 1.0
            constraints.append(jc)
        return constraints

    def _ik_from_seed(self, group, target_pose, seed_values, joint_names):
        state = self._robot_state_from_joints(joint_names, seed_values)
        group.set_start_state(state)
        try:
            group.set_joint_value_target(
                target_pose.pose,
                self._optical_frame,
                True,
            )
        except MoveItCommanderException as exc:
            rospy.logdebug("IK failed for seed %s: %s", [round(v, 3) for v in seed_values], exc)
            group.set_start_state_to_current_state()
            return None
        target_map = dict(zip(group.get_active_joints(), group.get_joint_value_target()))
        solution = [float(target_map.get(name, 0.0)) for name in joint_names]
        group.set_start_state_to_current_state()
        return solution

    def _snapshot_controller_state(self, topic="/S20/elfin_arm_controller/state", timeout=1.0):
        """Capture one controller state message for diagnostics."""
        try:
            state = rospy.wait_for_message(topic, JointTrajectoryControllerState, timeout=timeout)
            errors = {}
            for i, name in enumerate(state.joint_names):
                errors[name] = {
                    "desired": round(state.desired.positions[i], 5),
                    "actual": round(state.actual.positions[i], 5),
                    "error": round(state.error.positions[i], 5),
                }
            return errors
        except (rospy.ROSException, IndexError):
            return None

    def _log_controller_diagnostics(self, context):
        """Log controller state snapshot after an execution failure."""
        errors = self._snapshot_controller_state()
        if errors is None:
            rospy.logwarn("[%s] Controller state unavailable for diagnostics", context)
            return
        max_err_joint = max(errors, key=lambda n: abs(errors[n]["error"]))
        max_err = errors[max_err_joint]
        rospy.logwarn(
            "[%s] Controller state: worst joint %s desired=%.4f actual=%.4f error=%.4f",
            context, max_err_joint, max_err["desired"], max_err["actual"], max_err["error"],
        )
        for name, vals in errors.items():
            if abs(vals["error"]) > 0.01:
                rospy.logwarn(
                    "[%s]   %s: desired=%.4f actual=%.4f error=%.4f",
                    context, name, vals["desired"], vals["actual"], vals["error"],
                )

    def _execute_plan(self, group, plan):
        result = {"done": False, "ok": False}

        def _run():
            result["ok"] = bool(group.execute(plan, wait=True))
            result["done"] = True

        sampler = _ExecutionJointLogger(
            topic=self._exec_log_state_topic,
            rate_hz=self._exec_log_rate,
        )
        sampler.start()
        thread = threading.Thread(target=_run)
        thread.daemon = True
        thread.start()
        thread.join(self._execute_timeout)
        sampler.stop()
        if not result["done"]:
            try:
                group.stop()
            except MoveItCommanderException:
                pass
            self._log_controller_diagnostics("exec_timeout")
            return False, "Execution timed out after %.0fs (is Gazebo + elfin_arm_controller running?)" % self._execute_timeout
        if not result["ok"]:
            self._log_controller_diagnostics("exec_rejected")
            return False, "Execution rejected by move_group (CONTROL_FAILED)"
        return True, "Reached target"

    def _send_hold_trajectory(self, joint_names, positions, duration=0.5):
        """Send a 1-point trajectory that holds the arm at *positions*.

        Used as an emergency brake when settle detects escalating oscillation.
        Returns True if the action server was reachable and the goal was sent.
        """
        action_ns = rospy.get_param(
            "~arm_controller_action",
            "/S20/elfin_arm_controller/follow_joint_trajectory",
        )
        client = actionlib.SimpleActionClient(action_ns, FollowJointTrajectoryAction)
        if not client.wait_for_server(rospy.Duration(2.0)):
            rospy.logwarn("Emergency hold: controller action server unreachable")
            return False
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = list(joint_names)
        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        pt.velocities = [0.0] * len(positions)
        pt.time_from_start = rospy.Duration(duration)
        goal.trajectory.points = [pt]
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration(duration + 5.0))
        rospy.loginfo("Emergency hold trajectory sent to %s", action_ns)
        return True

    def _wait_robot_settled(self, vel_tol=0.005, timeout=3.0, hold_time=0.15,
                            quantile=None, diagnostics_out=None):
        """Block until all joint velocities stay below vel_tol for hold_time.

        Without this, the arm drifts slightly (Gazebo gravity settling, residual
        oscillation after a prior trajectory) between set_start_state_to_current_state()
        and the subsequent execute(), and move_group rejects the trajectory with
        "start point deviates from current robot state more than allowed_start_tolerance".

        If velocities are *escalating* (3 consecutive readings each higher than
        the previous) AND exceed the joint velocity limit (1.57 rad/s), send a
        hold-current-position trajectory as an emergency brake to prevent
        runaway oscillation. The threshold is set at the joint limit rather
        than a low 0.5 rad/s because normal PID settling can briefly spike
        above 0.5 while still decaying — calling hold on every such spike
        injects a new trajectory that itself excites oscillation, creating
        a hold-oscillate-hold loop.

        Returns (settled, elapsed_sec, max_velocity) so callers can surface the
        settle cost in their status messages for observability.

        The decision itself lives in settle_criterion.SettleTracker, which the
        offline replay also uses, so a recorded trace and a live run cannot be
        judged by two different rules. Pass ``diagnostics_out`` (a dict) to get
        the peak joint, the tail velocity and the below-tolerance fraction back
        -- a single peak number cannot distinguish a decaying residual from a
        flat one, which is exactly the question EX1 turns on.
        """
        # getattr with defaults: several callers construct a MotionPlanner
        # without running __init__, and settling must not depend on that.
        if quantile is None:
            quantile = getattr(self, "_settle_quantile", STRICT_QUANTILE)
        criterion = getattr(self, "_settle_criterion", DISPLACEMENT)
        tracker = SettleTracker(vel_tol, hold_time, quantile, criterion)
        t0 = rospy.Time.now()
        recent_v = []
        hold_sent = False
        escalate_window = 3
        # Joint velocity limit is 1.57 rad/s for all elfin joints; only
        # escalate when clearly above the limit (with margin) to avoid
        # false positives during normal settling.
        escalate_threshold = 1.8

        while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < timeout:
            try:
                js = rospy.wait_for_message("/joint_states", JointState, timeout=0.5)
            except rospy.ROSException:
                continue
            elapsed = (rospy.Time.now() - t0).to_sec()
            velocities = dict(zip(js.name, js.velocity))
            positions = dict(zip(js.name, js.position))
            settled = tracker.update(elapsed, velocities, positions)
            max_v = tracker.last_peak

            recent_v.append(max_v)
            if len(recent_v) > escalate_window:
                recent_v.pop(0)

            if (
                not hold_sent
                and len(recent_v) == escalate_window
                and all(
                    recent_v[i] > recent_v[i - 1]
                    for i in range(1, escalate_window)
                )
                and recent_v[-1] > escalate_threshold
            ):
                rospy.logwarn(
                    "Velocity escalating (%.3f -> %.3f -> %.3f); "
                    "sending emergency hold trajectory",
                    recent_v[0], recent_v[1], recent_v[2],
                )
                arm_joints = list(js.name)
                arm_positions = list(js.position)
                if self._send_hold_trajectory(arm_joints, arm_positions):
                    hold_sent = True
                    recent_v.clear()
                    tracker = SettleTracker(
                        vel_tol, hold_time, quantile, criterion)
                    # Give the hold trajectory time to take effect before
                    # re-sampling; otherwise the next reading may still
                    # show the pre-hold velocity and re-trigger logic.
                    time.sleep(0.2)

            if settled:
                if diagnostics_out is not None:
                    diagnostics_out.update(tracker.diagnostics())
                return True, (rospy.Time.now() - t0).to_sec(), max_v
        if diagnostics_out is not None:
            diagnostics_out.update(tracker.diagnostics())
        return False, (rospy.Time.now() - t0).to_sec(), tracker.peak_velocity

    def _ensure_release_settled(self, segment):
        """Fail closed unless the payload is motionless before vacuum release."""
        if segment.name != "descend":
            return True, "release settle not required"
        diagnostics = {}
        settled, elapsed, max_velocity = self._wait_robot_settled(
            vel_tol=self._release_settle_vel_tol,
            timeout=self._release_settle_timeout,
            hold_time=0.25,
            diagnostics_out=diagnostics,
        )
        if settled:
            return True, "release settle ok %.2fs/%.4f" % (
                elapsed, max_velocity)
        try:
            joint_state = rospy.wait_for_message(
                "/joint_states", JointState, timeout=1.0)
        except rospy.ROSException:
            return False, "release settle failed: joint state unavailable"
        if not self._send_hold_trajectory(
                joint_state.name, joint_state.position, duration=0.5):
            return False, (
                "release settle timeout %.2fs/%.4f; hold failed [%s]"
                % (elapsed, max_velocity, format_diagnostics(diagnostics)))
        retry_diagnostics = {}
        settled, retry_elapsed, retry_max_velocity = self._wait_robot_settled(
            vel_tol=self._release_settle_vel_tol,
            timeout=self._release_settle_timeout,
            hold_time=0.25,
            diagnostics_out=retry_diagnostics,
        )
        if not settled:
            # The diagnostics are the point: a flat tail_ratio says more time
            # will not help, a decaying one says the timeout is too short.
            return False, (
                "release settle failed after hold %.2fs/%.4f [%s]"
                % (retry_elapsed, retry_max_velocity,
                   format_diagnostics(retry_diagnostics)))
        return True, "release settle recovered by hold %.2fs/%.4f" % (
            retry_elapsed, retry_max_velocity)

    def _settle_before_plan(self, group):
        """Wait for the arm to be still before planning.

        Called at the top of each plan attempt so the planned start state matches
        the robot state at execute time. Returns a short summary string for the
        caller to fold into its status message (e.g. "settle 0.12s/0.003").

        NOTE: we deliberately do NOT call group.stop() here. After a trajectory
        ABORTs, JointTrajectoryController holds the last setpoint (arm stable).
        Calling group.stop() cancels that hold -> no holding torque -> the arm
        swings under gravity at up to ~2.9 rad/s (above the 1.57 velocity limit),
        which group.stop cannot arrest because no trajectory is active. The
        start-point drift this was meant to fix is already covered by
        allowed_start_tolerance=0.1 (moveit_with_camera.launch).
        """
        settled, elapsed, max_v = self._wait_robot_settled()
        flag = "ok" if settled else "TIMEOUT"
        return "settle %s %.2fs/%.4f" % (flag, elapsed, max_v)

    def _fold_trajectory_wrap_joints(self, plan):
        """Continuously unwrap wrap joints so the executed path is the short one.

        RRT can emit trajectory points where a wrap joint (elfin_joint1/4/5/6)
        jumps by ~2pi between consecutive points to an equivalent angle on a
        different branch; ros_control's JointTrajectoryController interpolates
        linearly between positions, so it would execute the long way around (a
        full spin). Fold each wrap joint's points to be continuous with the
        start state (each point within +/-pi of the previous) so the controller
        takes the short equivalent path. Velocities/accelerations stay
        consistent because consecutive folded points remain on the same branch.
        """
        joint_names = plan.joint_trajectory.joint_names
        points = plan.joint_trajectory.points
        if len(points) < 2:
            return
        two_pi = 2.0 * math.pi
        wrap_indices = [
            i for i, n in enumerate(joint_names) if n in WRAP_EQUIVALENT_JOINTS
        ]
        for idx in wrap_indices:
            prev = points[0].positions[idx]
            for pt in points[1:]:
                # MoveIt may return positions as tuples; normalize to a list so
                # we can fold wrap joints in place.
                pos = list(pt.positions)
                raw = pos[idx]
                k = round((raw - prev) / two_pi)
                folded = raw - k * two_pi
                pos[idx] = folded
                pt.positions = pos
                prev = folded

    def _check_wrist_trajectory_quality(self, plan, constraint_mode="", dual_down=False):
        """Reject trajectories where wrist joints rotate excessively.

        Checks both total rotation and single-step jumps. Uses tighter
        thresholds for J5/J6 to prevent camera/end-effector spin. Expects the
        trajectory to have been wrap-folded first (_fold_trajectory_wrap_joints)
        so consecutive points are on the same branch. For dual-down segments
        the orientation constraints already keep the EE from flipping, so the
        wrist thresholds are relaxed to admit legitimate reorientation/yaw
        swings while still rejecting genuine 2pi branch-jump loops.
        Returns (ok, info_string).
        """
        wrist_indices = {}
        joint_names = plan.joint_trajectory.joint_names
        for name in ("elfin_joint4", "elfin_joint5", "elfin_joint6"):
            if name in joint_names:
                wrist_indices[name] = joint_names.index(name)

        points = plan.joint_trajectory.points
        if len(points) < 2:
            return True, "too few points"

        violations = []
        for name, idx in wrist_indices.items():
            total_rotation = 0.0
            max_step = 0.0
            max_step_idx = 0
            for i in range(1, len(points)):
                delta = abs(points[i].positions[idx] - points[i - 1].positions[idx])
                total_rotation += delta
                if delta > max_step:
                    max_step = delta
                    max_step_idx = i

            if dual_down:
                rot_threshold = (
                    self._downward_wrist_j5_j6_threshold
                    if name in ("elfin_joint5", "elfin_joint6")
                    else self._downward_wrist_j4_threshold
                )
                step_threshold = self._downward_wrist_step_threshold
            else:
                rot_threshold = (
                    self._wrist_j5_j6_rotation_threshold
                    if name in ("elfin_joint5", "elfin_joint6")
                    else self._wrist_max_rotation_threshold
                )
                step_threshold = self._wrist_max_step_threshold
            if total_rotation > rot_threshold:
                violations.append(
                    "%s total=%.2frad(>%.2f)" % (name, total_rotation, rot_threshold)
                )
            if max_step > step_threshold:
                violations.append(
                    "%s step=%.2frad@pt%d(>%.2f)"
                    % (name, max_step, max_step_idx, step_threshold)
                )

        if violations:
            return False, "wrist quality: %s [%s]" % (
                ", ".join(violations), constraint_mode,
            )
        return True, "ok"

    def _move_to_joint_target(self, joint_names, target_values, tolerance, label, path_constraints=None):
        try:
            group = self._ensure_move_group()
        except MoveItCommanderException as exc:
            return False, False, "MoveIt not ready: %s" % exc

        current_values = self._ordered_joint_values(group, joint_names)
        adjusted_targets = self._nearest_equivalent_targets(
            joint_names, current_values, target_values, label
        )
        if self._within_tolerance(current_values, adjusted_targets, tolerance):
            rospy.loginfo("Already at target: %s", label)
            return True, True, "Already at target: %s" % label

        for attempt in range(1 + self._replan_max_attempts):
            settle_summary = self._settle_before_plan(group)
            group.set_start_state_to_current_state()
            if path_constraints is not None:
                group.set_path_constraints(path_constraints)
            group.set_joint_value_target(dict(zip(joint_names, adjusted_targets)))
            rospy.loginfo("Planning move to %s (attempt %d, %s) ...", label, attempt + 1, settle_summary)
            success, plan, _planning_time, error_code = group.plan()
            if path_constraints is not None:
                group.clear_path_constraints()

            if not success or not plan.joint_trajectory.points:
                err_val = error_code.val if hasattr(error_code, "val") else error_code
                if attempt < self._replan_max_attempts:
                    rospy.logwarn(
                        "Planning failed for %s (error %s), refreshing scene and retrying...",
                        label, err_val,
                    )
                    self._refresh_dynamic_scene()
                    rospy.sleep(self._replan_delay)
                    group.set_start_state_to_current_state()
                    continue
                contact_diag = self._diagnose_start_state(group, label, err_val)
                fail_pose = Pose(position=Point(x=0, y=0, z=0), orientation=Quaternion(w=1.0))
                self._publish_failure_marker(
                    fail_pose, label, self._base_frame, err_val, "Joint planning failed"
                )
                return False, False, "Planning failed for %s (error %s) after %d attempts [%s]%s" % (
                    label, err_val, attempt + 1, settle_summary, contact_diag
                )

            exec_ok, exec_message = self._execute_plan(group, plan)
            if not exec_ok:
                if attempt < self._replan_max_attempts:
                    rospy.logwarn(
                        "Execution failed for %s (%s), refreshing scene and retrying...",
                        label, exec_message,
                    )
                    self._refresh_dynamic_scene()
                    rospy.sleep(self._replan_delay)
                    group.set_start_state_to_current_state()
                    continue
                return False, False, "%s [%s]" % (exec_message, settle_summary)

            rospy.loginfo("Reached %s [%s]", label, settle_summary)
            return True, False, "Reached %s [%s]" % (label, settle_summary)

        return False, False, "Planning exhausted all attempts for %s" % label

    # ── Constraint builders for camera-down and wrist-lock ──────────────

    def _lookup_camera_reference_orientation(self):
        """Return a fixed "optical-z points to world -z" target quaternion.

        Previously this returned the *current* TF pose of camera_depth_optical_frame
        in the base frame, which made keep_camera_down lock the camera to whatever
        attitude it happened to have when the segment started — not necessarily
        "lens facing the ground". Combined with the cam_mount rpy=(π/2, -π/2, 0),
        that caused the IK solver to rotate J6 just to match the snapshot pose.

        Now we return the canonical down-facing target: rotate base_link by π
        around its X axis, which sends the optical frame's +Z (the camera view
        direction) to world -Z while leaving the in-image roll (rotation around
        the view axis) free. Pair this with a large absolute_z_axis_tolerance on
        the OrientationConstraint so the solver can pick any J6 angle.
        """
        # rpy = (π, 0, 0) → quaternion (x, y, z, w) = (1, 0, 0, 0)
        return Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)

        # --- Original implementation (kept for reference) -------------------
        # def _lookup_camera_reference_orientation(self):
        #     """Get current camera_depth_optical_frame orientation in base frame."""
        #     try:
        #         transform = self._tf_buffer.lookup_transform(
        #             self._base_frame, self._optical_frame,
        #             rospy.Time(0), rospy.Duration(2.0),
        #         )
        #         return transform.transform.rotation
        #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
        #             tf2_ros.ExtrapolationException) as exc:
        #         rospy.logwarn("Cannot lookup camera TF for constraint: %s", exc)
        #         return None

    def _downward_orientations(self):
        """Cache and return the TF-derived downward target quaternions.

        Returns the dict from ``compute_downward_orientations``
        (``camera_down_quat``, ``suction_down_quat``, ``inter_axis_deg``) or
        None when the fixed camera<->suction transform is unavailable or the
        mount is infeasible. The transform is pose-independent (both links are
        rigid children of suction_panel).
        """
        if self._downward_orientations_cache is not None:
            return self._downward_orientations_cache
        try:
            orientations = compute_downward_orientations(
                self._tf_buffer, self._optical_frame, self._suction_frame,
                self._base_frame, timeout=self._tf_wait_timeout,
            )
        except Exception as exc:
            rospy.logwarn("downward_constraints: camera<->suction TF unavailable: %s", exc)
            return None
        ok, msg = feasibility_check(
            orientations["inter_axis_deg"],
            self._downward_cfg["camera_max_tilt_deg"],
            self._downward_cfg["suction_max_tilt_deg"],
        )
        if not ok:
            rospy.logerr("downward_constraints: %s; dual-down disabled", msg)
            return None
        self._downward_orientations_cache = orientations
        rospy.loginfo("downward_constraints: %s", msg)
        return orientations

    def _ensure_fk(self):
        """Lazily connect to /compute_fk for per-point trajectory tilt checks."""
        if self._fk_proxy is not None:
            return self._fk_proxy
        try:
            rospy.wait_for_service(self._compute_fk_service, timeout=2.0)
        except rospy.ROSException:
            rospy.logwarn(
                "downward validation: %s unavailable; tilt checks skipped",
                self._compute_fk_service,
            )
            return None
        self._fk_proxy = rospy.ServiceProxy(self._compute_fk_service, GetPositionFK)
        return self._fk_proxy

    def _fk_downward_tilts(self, joint_names, joint_values):
        """Return (camera_tilt_deg, suction_tilt_deg) for one robot state."""
        fk = self._ensure_fk()
        if fk is None:
            return None
        req = GetPositionFKRequest()
        req.header.frame_id = self._base_frame
        req.fk_link_names = [self._optical_frame, self._suction_frame]
        req.robot_state = RobotState(
            joint_state=JointState(
                name=list(joint_names), position=list(joint_values)))
        try:
            resp = fk(req)
        except rospy.ServiceException as exc:
            rospy.logwarn("downward diagnostics: compute_fk failed: %s", exc)
            return None
        if (
                resp.error_code.val != MoveItErrorCodes.SUCCESS
                or len(resp.pose_stamped) < 2):
            return None
        cam_q = resp.pose_stamped[0].pose.orientation
        suc_q = resp.pose_stamped[1].pose.orientation
        return (
            link_z_tilt_deg((cam_q.x, cam_q.y, cam_q.z, cam_q.w)),
            link_z_tilt_deg((suc_q.x, suc_q.y, suc_q.z, suc_q.w)),
        )

    def _format_downward_state_diagnostic(
            self, label, joint_names, joint_values):
        """Describe collision and downward-envelope status for one state."""
        valid, contacts = self._check_state_validity(joint_names, joint_values)
        tilts = self._fk_downward_tilts(joint_names, joint_values)
        collision = "valid" if valid else "COLLISION(%s)" % contacts
        if tilts is None:
            return "%s: %s tilt=unavailable" % (label, collision)
        cam, suc = tilts
        cam_ok = cam <= self._downward_cfg["camera_max_tilt_deg"] + 1e-6
        suc_ok = suc <= self._downward_cfg["suction_max_tilt_deg"] + 1e-6
        return (
            "%s: %s camera=%.2fdeg(%s<=%.2f) suction=%.2fdeg(%s<=%.2f)"
            % (
                label, collision,
                cam, "ok" if cam_ok else "FAIL",
                self._downward_cfg["camera_max_tilt_deg"],
                suc, "ok" if suc_ok else "FAIL",
                self._downward_cfg["suction_max_tilt_deg"],
            )
        )

    def _pose_goal_joint_solution(self, group, pose, target_link):
        """Resolve a pose goal once for diagnostics without executing it."""
        joint_names = list(group.get_active_joints())
        current_values = list(group.get_current_joint_values())
        try:
            group.set_start_state(
                self._robot_state_from_joints(joint_names, current_values))
            group.set_joint_value_target(pose.pose, target_link, True)
            target_map = dict(zip(
                group.get_active_joints(), group.get_joint_value_target()))
            return joint_names, [
                float(target_map.get(name, current_values[i]))
                for i, name in enumerate(joint_names)
            ]
        except (MoveItCommanderException, IndexError) as exc:
            rospy.logwarn(
                "downward diagnostics: goal IK failed for %s: %s",
                target_link, exc,
            )
            return None
        finally:
            try:
                group.set_start_state_to_current_state()
            except MoveItCommanderException:
                pass

    def _downward_endpoint_diagnostics(self, group, pose, target_link):
        """Return start/goal collision + tilt diagnostics for constrained plans."""
        joint_names = list(group.get_active_joints())
        current_values = list(group.get_current_joint_values())
        start_valid, _ = self._check_state_validity(
            joint_names, current_values)
        start_tilts = self._fk_downward_tilts(
            joint_names, current_values)
        reports = [
            self._format_downward_state_diagnostic(
                "start", joint_names, current_values)
        ]
        goal = self._pose_goal_joint_solution(group, pose, target_link)
        if goal is None:
            classification = "goal_ik_unavailable"
            reports.append("goal: IK unavailable")
        else:
            goal_valid, _ = self._check_state_validity(goal[0], goal[1])
            goal_tilts = self._fk_downward_tilts(goal[0], goal[1])
            reports.append(self._format_downward_state_diagnostic(
                "goal", goal[0], goal[1]))
            start_outside = (
                start_tilts is not None and (
                    start_tilts[0] > self._downward_cfg["camera_max_tilt_deg"]
                    or start_tilts[1] > self._downward_cfg["suction_max_tilt_deg"]
                )
            )
            goal_outside = (
                goal_tilts is not None and (
                    goal_tilts[0] > self._downward_cfg["camera_max_tilt_deg"]
                    or goal_tilts[1] > self._downward_cfg["suction_max_tilt_deg"]
                )
            )
            if not start_valid or not goal_valid:
                classification = "collision_endpoint"
            elif start_outside:
                classification = "start_outside"
            elif goal_outside:
                classification = "goal_outside"
            else:
                classification = "sampler_or_collision_disconnected"
        return "classification=%s; %s" % (
            classification, "; ".join(reports))

    def _validate_downward_trajectory(self, plan, segment):
        """Per-point FK tilt check for every payload-carry segment.

        Any ``keep_tool_down`` segment is safety-critical: suction and camera
        budgets are checked over all sampled points and FK failures fail closed.
        """
        if not self._downward_validate:
            return True, "validation disabled"
        if not bool(segment.keep_tool_down):
            return True, "not payload carry"
        points = plan.joint_trajectory.points
        joint_names = plan.joint_trajectory.joint_names
        if len(points) < 2:
            return True, "too few points"
        fk = self._ensure_fk()
        if fk is None:
            return False, "compute_fk unavailable"
        stride = max(1, self._downward_fk_stride)
        indices = list(range(0, len(points), stride))
        if indices[-1] != len(points) - 1:
            indices.append(len(points) - 1)
        camera_tilts = []
        suction_tilts = []
        camera_points = []
        for i in indices:
            req = GetPositionFKRequest()
            req.header.frame_id = self._base_frame
            req.fk_link_names = [self._optical_frame, self._suction_frame]
            robot_state = RobotState()
            robot_state.joint_state = JointState(
                name=list(joint_names), position=list(points[i].positions))
            req.robot_state = robot_state
            try:
                resp = fk(req)
            except rospy.ServiceException as exc:
                rospy.logwarn("downward validation: compute_fk failed: %s", exc)
                return False, "compute_fk call failed"
            if resp.error_code.val != MoveItErrorCodes.SUCCESS:
                return False, "compute_fk error %d" % resp.error_code.val
            if len(resp.pose_stamped) < 2:
                return False, "compute_fk returned too few poses"
            cam_q = resp.pose_stamped[0].pose.orientation
            suc_q = resp.pose_stamped[1].pose.orientation
            camera_tilts.append(link_z_tilt_deg((cam_q.x, cam_q.y, cam_q.z, cam_q.w)))
            suction_tilts.append(link_z_tilt_deg((suc_q.x, suc_q.y, suc_q.z, suc_q.w)))
            camera_points.append(resp.pose_stamped[0].pose.position)
        ok, max_cam, max_suc, worst = validate_downward_tilts(
            camera_tilts, suction_tilts,
            self._downward_cfg["camera_max_tilt_deg"],
            self._downward_cfg["suction_max_tilt_deg"],
        )
        self._publish_trajectory_tilt(
            segment.name, camera_points, camera_tilts, suction_tilts, worst)
        self._last_tilt_profile = {
            "segment": segment.name,
            "max_camera_tilt_deg": max_cam,
            "max_suction_tilt_deg": max_suc,
            "worst_point_index": int(worst),
            "sampled_points": len(camera_tilts),
        }
        history = rospy.get_param("/luggage/trajectory_tilt_history", [])
        history.append(dict(self._last_tilt_profile))
        rospy.set_param("/luggage/trajectory_tilt_history", history[-200:])
        return ok, "downward tilt max_cam=%.1f max_suc=%.1f deg (worst pt %d)" % (
            max_cam, max_suc, worst)

    def _publish_trajectory_tilt(
            self, segment_name, positions, camera_tilts, suction_tilts, worst):
        """Draw the planned path colored by how close it runs to the tilt budget."""
        if len(positions) < 2:
            return
        camera_limit = max(1e-6, self._downward_cfg["camera_max_tilt_deg"])
        suction_limit = max(1e-6, self._downward_cfg["suction_max_tilt_deg"])
        stamp = rospy.Time.now()
        ma = MarkerArray()

        line = Marker()
        line.header.frame_id = self._base_frame
        line.header.stamp = stamp
        line.ns = "trajectory_tilt"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation = Quaternion(w=1.0)
        line.scale.x = 0.008
        line.lifetime = rospy.Duration(120.0)
        for position, camera, suction in zip(
                positions, camera_tilts, suction_tilts):
            usage = min(1.0, max(
                camera / camera_limit, suction / suction_limit))
            line.points.append(
                Point(x=position.x, y=position.y, z=position.z))
            line.colors.append(ColorRGBA(
                r=min(1.0, 2.0 * usage),
                g=min(1.0, 2.0 * (1.0 - usage)),
                b=0.1,
                a=1.0,
            ))
        ma.markers.append(line)

        if 0 <= worst < len(positions):
            marker = Marker()
            marker.header.frame_id = self._base_frame
            marker.header.stamp = stamp
            marker.ns = "trajectory_tilt_worst"
            marker.id = 1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = positions[worst]
            marker.pose.orientation = Quaternion(w=1.0)
            marker.scale.x = marker.scale.y = marker.scale.z = 0.035
            marker.color = ColorRGBA(r=1.0, g=0.2, b=0.0, a=1.0)
            marker.lifetime = rospy.Duration(120.0)
            ma.markers.append(marker)
            self._append_text_marker(
                ma, stamp, self._base_frame,
                (positions[worst].x, positions[worst].y,
                 positions[worst].z + 0.08),
                "%s worst tilt\ncamera %.1f/%.1f deg\nsuction %.1f/%.1f deg" % (
                    segment_name,
                    camera_tilts[worst], camera_limit,
                    suction_tilts[worst], suction_limit),
                "trajectory_tilt_worst_label", 2, 0.045,
                ColorRGBA(r=1.0, g=0.6, b=0.2, a=1.0))
        self._target_pub.publish(ma)

    def _build_wrist_lock_joint_constraints(self, j4_tolerance, j5_j6_tolerance):
        """Build JointConstraints to keep J4/J5/J6 near their current angles."""
        constraints = []
        try:
            js = rospy.wait_for_message("/joint_states", JointState, timeout=2.0)
        except rospy.ROSException:
            rospy.logwarn("Cannot get joint_states for wrist lock constraints")
            return constraints

        current = dict(zip(js.name, js.position))
        for joint_name in self._lock_wrist_joints:
            if joint_name not in current:
                continue
            ref = float(current[joint_name])
            tol = j4_tolerance if joint_name == "elfin_joint4" else j5_j6_tolerance
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = ref
            jc.tolerance_above = tol
            jc.tolerance_below = tol
            jc.weight = 1.0
            constraints.append(jc)
            rospy.loginfo("Wrist-lock: %s ref=%.4f tol=%.3f", joint_name, ref, tol)
        return constraints

    def _build_segment_constraints(self, segment, tool_yaw, cam_rp, cam_yaw,
                                   wrist_tols, camera_ref_q=None):
        """Build combined Constraints for a segment.

        Returns (Constraints_or_None, mode_description).

        Dual-down segments (keep_tool_down + keep_camera_down with a usable
        camera<->suction TF) constrain ``suction_contact_frame`` +Z and
        ``camera_depth_optical_frame`` +Z toward base -Z using the TF-derived
        suction-down target quaternions and the downward_constraints budgets.
        This fixes the old keep_tool_down bug that constrained ``elfin_link6``
        in ``world`` to the segment's own goal orientation.
        """
        constraints = Constraints()
        modes = []
        dual_down = bool(segment.keep_tool_down) and bool(
            getattr(segment, "keep_camera_down", False))
        # A tool-down-only segment still needs the TF-derived suction target;
        # otherwise the legacy fallback constrains elfin_link6 with a
        # suction-frame quaternion. Camera-down controls only whether the
        # additional camera guard is requested.
        orientations = (
            self._downward_orientations()
            if segment.keep_tool_down else None)

        if segment.keep_tool_down and tool_yaw is not None:
            oc = OrientationConstraint()
            oc.weight = 1.0
            if orientations is not None:
                q = orientations["suction_down_quat"]
                oc.header.frame_id = self._base_frame
                oc.link_name = self._suction_frame
                oc.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
                rp_tolerance = (
                    self._suction_max_tilt_rad
                    if dual_down else self._tool_down_rp_tolerance)
                oc.absolute_x_axis_tolerance = rp_tolerance
                oc.absolute_y_axis_tolerance = rp_tolerance
                oc.absolute_z_axis_tolerance = tool_yaw
                modes.append("tool_down(suction rp=%.3f,yaw=%.3f)" % (
                    rp_tolerance, tool_yaw))
            else:
                oc.header.frame_id = self._pose_target_frame
                oc.link_name = self._pose_target_link
                oc.orientation = segment.target_pose.orientation
                oc.absolute_x_axis_tolerance = self._tool_down_rp_tolerance
                oc.absolute_y_axis_tolerance = self._tool_down_rp_tolerance
                oc.absolute_z_axis_tolerance = tool_yaw
                modes.append("tool_down(link6 rp=%.3f,yaw=%.3f)" % (
                    self._tool_down_rp_tolerance, tool_yaw))
            constraints.orientation_constraints.append(oc)

        if getattr(segment, 'keep_camera_down', False) and cam_rp is not None:
            cam_q = None
            if dual_down and orientations is not None:
                # Camera and suction are rigidly coupled. The tighter suction
                # cone already bounds camera tilt to inter-axis+suction budget
                # (12.25+5 < 18 deg on the current mount). Adding a second
                # OrientationConstraint on the same serial chain makes OMPL
                # sample the intersection of two redundant manifolds and is a
                # major source of "no valid goal states" failures. Keep camera
                # as an FK-validated safety guard instead of a path constraint.
                modes.append("camera_guard(FK<=%.3f)" % self._camera_max_tilt_rad)
            elif camera_ref_q is not None:
                cam_q = camera_ref_q
                eff_rp = cam_rp
                eff_yaw = cam_yaw
            if cam_q is not None:
                oc = OrientationConstraint()
                oc.header.frame_id = self._base_frame
                oc.link_name = self._optical_frame
                oc.orientation = cam_q
                oc.absolute_x_axis_tolerance = eff_rp
                oc.absolute_y_axis_tolerance = eff_rp
                oc.absolute_z_axis_tolerance = eff_yaw
                oc.weight = 1.0
                constraints.orientation_constraints.append(oc)
                modes.append("camera_down(rp=%.3f,yaw=%.3f)" % (eff_rp, eff_yaw))
            else:
                rospy.logwarn("Skipping camera-down constraint (orientation unavailable)")

        if getattr(segment, 'lock_wrist', False) and wrist_tols is not None:
            j4_tol, j5j6_tol = wrist_tols
            jcs = self._build_wrist_lock_joint_constraints(j4_tol, j5j6_tol)
            constraints.joint_constraints.extend(jcs)
            if jcs:
                modes.append("wrist_lock(j4=%.3f,j5j6=%.3f)" % (j4_tol, j5j6_tol))

        if not modes:
            return None, "unconstrained"
        return constraints, "+".join(modes)

    def _build_constraint_levels(self, segment):
        """Build fallback constraint configurations from tight to relaxed.

        Each level is (tool_yaw_tol, cam_rp_tol, cam_yaw_tol, wrist_tols).
        Dual-down segments (keep_tool_down + keep_camera_down with a usable
        camera<->suction TF) use a single strict level: the fixed roll/pitch
        budgets from downward_constraints and free yaw (pi), since the
        suction-down orientation already satisfies both axes jointly and must
        not be relaxed away.
        """
        tool_down = segment.keep_tool_down
        camera_down = getattr(segment, 'keep_camera_down', False)
        wrist_lock = getattr(segment, 'lock_wrist', False)

        dual_down = (
            tool_down and camera_down
            and self._downward_orientations() is not None
        )
        if dual_down:
            w = (
                (self._lock_wrist_tolerance_j4, self._lock_wrist_tolerance_j5_j6)
                if wrist_lock else None
            )
            # (tool_yaw=pi free, cam_rp=camera budget, cam_yaw=pi free, wrist)
            return [(math.pi, self._camera_max_tilt_rad, math.pi, w)]

        if not (tool_down or camera_down or wrist_lock):
            return [(None, None, None, None)]

        if tool_down and segment.name in (
                "stage_mid", "stage_late", "stage",
                "transit", "traverse"):
            # Payload safety constrains roll/pitch; yaw is free along the path.
            # The goal pose still enforces the requested box yaw.
            t_steps = [math.pi]
        else:
            t_steps = (
                [self._tool_down_yaw_tolerance]
                + list(self._tool_down_yaw_relax_steps)
                if tool_down else [None]
            )
        c_rp = self._camera_down_rp_tolerance if camera_down else None
        c_steps = (
            [self._camera_down_yaw_tolerance] + list(self._camera_down_yaw_relax_steps)
            if camera_down else [None]
        )
        w_steps = (
            [(self._lock_wrist_tolerance_j4, self._lock_wrist_tolerance_j5_j6)]
            + [(s * 1.5, s) for s in self._lock_wrist_relax_steps]
            if wrist_lock else [None]
        )

        n = max(len(t_steps), len(c_steps), len(w_steps))
        levels = []
        for i in range(n):
            t = t_steps[min(i, len(t_steps) - 1)]
            c = c_steps[min(i, len(c_steps) - 1)]
            w = w_steps[min(i, len(w_steps) - 1)]
            level = (t, c_rp, c, w)
            if not levels or level != levels[-1]:
                levels.append(level)
        return levels

    @staticmethod
    def _optical_frame_segments():
        """Segment names whose target_pose is a camera_depth_optical_frame pose
        in elfin_base_link (probe insertions + smart explore views)."""
        return (
            "align_down", "pre_opening", "enter_opening", "probe_inside",
            "retreat_opening",
            "explore_view", "smart_phase0", "smart_phase1",
        )

    @staticmethod
    def _place_segments_in_base_frame():
        """Place slot poses from bin_packer / placement_planner are in base_link.

        scene_manager.add_placed_box uses the same frame. Pick poses stay in
        world (spawner ground truth); only place segments need base_link here.
        """
        return (
            "stage_mid", "stage_late", "stage",
            "transit", "traverse", "insert", "descend")

    def _segment_pose_target_link(self, segment):
        if segment.name in self._optical_frame_segments():
            return self._optical_frame
        if segment.name in self._pick_segments_use_contact_frame:
            return self._pick_pose_target_link
        return self._pose_target_link

    def _segment_pose_reference_frame(self, segment):
        """Return the frame in which an unframed MotionSegment pose is expressed."""
        if segment.name in self._optical_frame_segments():
            return self._base_frame
        if segment.name in self._place_segments_in_base_frame():
            return self._base_frame
        # place retreat carries keep_tool_down; pick retreat does not.
        if segment.name == "retreat" and bool(getattr(segment, "keep_tool_down", False)):
            return self._base_frame
        return self._pose_target_frame

    def _execute_pose_target(self, segment):
        try:
            group = self._ensure_move_group()
        except MoveItCommanderException as exc:
            return False, "MoveIt not ready: %s" % exc

        reference_frame = self._segment_pose_reference_frame(segment)
        pose = PoseStamped()
        pose.header.frame_id = reference_frame
        pose.header.stamp = rospy.Time(0)
        pose.pose = segment.target_pose

        target_link = self._segment_pose_target_link(segment)

        self._publish_target_marker(
            segment.target_pose, segment.name,
            target_link, reference_frame, success=True,
        )

        previous_link = group.get_end_effector_link()
        original_planning_time = None
        try:
            group.set_end_effector_link(target_link)
            group.set_pose_reference_frame(reference_frame)

            # ── Build constraint levels for this segment ──
            constraint_levels = self._build_constraint_levels(segment)
            any_constrained = any(
                level != (None, None, None, None) for level in constraint_levels
            )
            if any_constrained:
                original_planning_time = group.get_planning_time()

            camera_ref_q = None
            if getattr(segment, 'keep_camera_down', False):
                camera_ref_q = self._lookup_camera_reference_orientation()
                if camera_ref_q is not None:
                    rospy.loginfo(
                        "Camera-down reference: frame=%s quat=(%.3f,%.3f,%.3f,%.3f)",
                        self._optical_frame,
                        camera_ref_q.x, camera_ref_q.y, camera_ref_q.z, camera_ref_q.w,
                    )

            endpoint_diag = ""
            if any_constrained:
                endpoint_diag = self._downward_endpoint_diagnostics(
                    group, pose, target_link)
                rospy.loginfo(
                    "Constraint endpoints for %s: %s",
                    segment.name, endpoint_diag,
                )
            classification = self._endpoint_classification(endpoint_diag)

            last_fail_message = "Pose planning exhausted all attempts for %s" % segment.name
            self._last_tilt_profile = {}
            diagnostics = {
                "segment": segment.name,
                "target_link": target_link,
                "reference_frame": reference_frame,
                "endpoint_classification": classification,
                "endpoint_diagnostics": endpoint_diag,
                "constrained": bool(any_constrained),
                "constraint_levels": len(constraint_levels),
            }

            for level_idx, level in enumerate(constraint_levels):
                tool_yaw, cam_rp, cam_yaw, wrist_tols = level
                constraints, mode_desc = self._build_segment_constraints(
                    segment, tool_yaw, cam_rp, cam_yaw, wrist_tols,
                    camera_ref_q=camera_ref_q,
                )
                self._publish_active_constraints(
                    constraints, segment.name, mode_desc, level_idx,
                    segment.target_pose, reference_frame)
                self._publish_target_marker(
                    segment.target_pose, segment.name, target_link,
                    reference_frame, success=True,
                    extra_text="level=%d mode=%s\n%s" % (
                        level_idx, mode_desc, classification),
                )
                diagnostics.update({
                    "level": level_idx,
                    "constraint_mode": mode_desc,
                })

                if constraints is not None:
                    group.set_path_constraints(constraints)
                    constrained_time = max(
                        self._tool_down_planning_time if segment.keep_tool_down else 0,
                        self._camera_down_planning_time
                        if getattr(segment, 'keep_camera_down', False) else 0,
                        self._planning_time,
                    )
                    if self._constrained_planning_time_cap > 0.0:
                        constrained_time = min(
                            constrained_time,
                            self._constrained_planning_time_cap,
                        )
                    group.set_planning_time(constrained_time)

                if level_idx > 0:
                    rospy.logwarn(
                        "Relaxing constraints for segment %s (level %d): %s",
                        segment.name, level_idx, mode_desc,
                    )
                elif any_constrained:
                    rospy.loginfo(
                        "Constraints for segment %s: %s", segment.name, mode_desc,
                    )

                for attempt in range(1 + self._replan_max_attempts):
                    settle_summary = self._settle_before_plan(group)
                    group.set_start_state_to_current_state()
                    group.set_pose_target(pose, target_link)
                    rospy.loginfo(
                        "Planning pose_target segment %s (level %d attempt %d, %s) "
                        "for %s in %s: xyz=(%.3f, %.3f, %.3f)",
                        segment.name, level_idx, attempt + 1, settle_summary,
                        target_link, reference_frame,
                        pose.pose.position.x, pose.pose.position.y,
                        pose.pose.position.z,
                    )
                    success, plan, _planning_time, error_code = group.plan()
                    group.clear_pose_targets()
                    diagnostics["attempt"] = attempt + 1
                    if not success or not plan.joint_trajectory.points:
                        err_val = error_code.val if hasattr(error_code, "val") else error_code
                        diagnostics["error_code"] = err_val
                        if attempt < self._replan_max_attempts:
                            rospy.logwarn(
                                "Pose planning failed for %s (error %s), refreshing scene...",
                                segment.name, err_val,
                            )
                            if segment.name in self._clear_octomap_segments:
                                self._clear_octomap(segment.name)
                            self._refresh_dynamic_scene()
                            rospy.sleep(self._replan_delay)
                            continue
                        contact_diag = self._diagnose_start_state(
                            group, segment.name, err_val
                        )
                        last_fail_message = (
                            "Planning failed for %s (error %s) after %d attempts "
                            "[%s %s; %s]%s"
                            % (segment.name, err_val, attempt + 1, settle_summary,
                               mode_desc, endpoint_diag, contact_diag)
                        )
                        break

                    # Fold wrap-joint points onto the start branch so the
                    # controller executes the short equivalent path (no 2pi
                    # spins) before we measure wrist rotation.
                    self._fold_trajectory_wrap_joints(plan)

                    if any_constrained or segment.name == "align_down":
                        dual_down = (
                            bool(segment.keep_tool_down)
                            and self._downward_orientations() is not None
                        )
                        wrist_ok, wrist_info = self._check_wrist_trajectory_quality(
                            plan, mode_desc, dual_down=dual_down,
                        )
                        diagnostics["wrist_quality"] = wrist_info
                        if not wrist_ok:
                            rospy.logwarn(
                                "Wrist trajectory quality rejected for %s: %s (attempt %d)",
                                segment.name, wrist_info, attempt + 1,
                            )
                            if attempt < self._replan_max_attempts:
                                if segment.name in self._clear_octomap_segments:
                                    self._clear_octomap(segment.name)
                                self._refresh_dynamic_scene()
                                rospy.sleep(self._replan_delay)
                                continue
                            last_fail_message = (
                                "Wrist quality rejected for %s: %s [%s]"
                                % (segment.name, wrist_info, settle_summary)
                            )
                            break

                    # Downward tilt validation (dual-down strict mode). Runs
                    # per-point FK on camera/suction +Z vs base -Z; rejects
                    # trajectories that violate the budgets when strict.
                    down_ok, down_info = self._validate_downward_trajectory(
                        plan, segment)
                    diagnostics["downward_validation"] = down_info
                    diagnostics["tilt_profile"] = dict(self._last_tilt_profile)
                    if not down_ok:
                        rospy.logwarn(
                            "Downward quality rejected for %s: %s (attempt %d)",
                            segment.name, down_info, attempt + 1,
                        )
                        if self._strict_downward:
                            if attempt < self._replan_max_attempts:
                                if segment.name in self._clear_octomap_segments:
                                    self._clear_octomap(segment.name)
                                self._refresh_dynamic_scene()
                                rospy.sleep(self._replan_delay)
                                continue
                            last_fail_message = (
                                "Downward quality rejected for %s: %s [%s]"
                                % (segment.name, down_info, settle_summary)
                            )
                            break
                        rospy.logwarn(
                            "Downward tilt exceeded but strict=False; "
                            "executing %s anyway", segment.name,
                        )

                    self._publish_target_marker(
                        segment.target_pose, segment.name,
                        target_link, reference_frame, success=True,
                        extra_text="level=%d mode=%s\n%s" % (
                            level_idx, mode_desc, classification),
                    )
                    exec_ok, exec_message = self._execute_plan(group, plan)
                    if not exec_ok:
                        if attempt < self._replan_max_attempts:
                            rospy.logwarn(
                                "Pose exec failed for %s (%s), refreshing scene...",
                                segment.name, exec_message,
                            )
                            if segment.name in self._clear_octomap_segments:
                                self._clear_octomap(segment.name)
                            self._refresh_dynamic_scene()
                            rospy.sleep(self._replan_delay)
                            continue
                        self._publish_failure_marker(
                            segment.target_pose, segment.name,
                            reference_frame, "exec", exec_message,
                        )
                        diagnostics.update({
                            "outcome": "execution_failed",
                            "message": exec_message,
                        })
                        self._publish_planning_diagnostics(diagnostics)
                        return False, "%s failed: %s [%s]" % (
                            segment.name, exec_message, settle_summary,
                        )
                    release_ok, release_info = self._ensure_release_settled(
                        segment)
                    diagnostics["release_settle"] = release_info
                    if not release_ok:
                        diagnostics.update({
                            "outcome": "release_settle_failed",
                            "message": release_info,
                        })
                        self._publish_planning_diagnostics(diagnostics)
                        return False, "%s failed: %s" % (
                            segment.name, release_info)
                    diagnostics.update({
                        "outcome": "success",
                        "message": settle_summary,
                    })
                    self._publish_planning_diagnostics(diagnostics)
                    return True, "Reached pose segment %s [%s %s]" % (
                        segment.name, settle_summary, mode_desc,
                    )

                # Inner attempt loop exhausted — try next constraint level
                if constraints is not None:
                    try:
                        group.clear_path_constraints()
                    except MoveItCommanderException:
                        pass

            # All constraint levels exhausted
            self._publish_failure_marker(
                segment.target_pose, segment.name,
                reference_frame, "exhausted", last_fail_message,
            )
            current_joints = group.get_active_joints()
            current_values = group.get_current_joint_values()
            valid, info = self._check_state_validity(current_joints, current_values)
            if not valid:
                self._publish_validity_marker(
                    segment.target_pose, reference_frame, info,
                )
                rospy.logwarn("State validity: %s", info)
            diagnostics.update({
                "outcome": "levels_exhausted",
                "message": last_fail_message,
                "current_state_valid": bool(valid),
                "current_state_info": info,
            })
            self._publish_planning_diagnostics(diagnostics)
            return False, last_fail_message
        finally:
            if original_planning_time is not None:
                try:
                    group.clear_path_constraints()
                except MoveItCommanderException:
                    pass
                try:
                    group.set_planning_time(original_planning_time)
                except MoveItCommanderException:
                    pass
            try:
                group.clear_pose_targets()
                group.set_start_state_to_current_state()
                if previous_link:
                    group.set_end_effector_link(previous_link)
            except MoveItCommanderException:
                pass

    def _execute_cartesian_target(self, segment):
        """Plan + execute a straight-line Cartesian descent/lift to the segment
        target, holding the EE link (suction_contact_frame for pick segments)
        fixed over the box so the forearm cannot swing into the box.

        Used for pick approach/attach/retreat: a free-space OMPL plan to a
        point directly above the box can swing the forearm/elbow through the
        box volume between collision-checked waypoints (Bug #1). A Cartesian
        path along the EE line evolves the arm config continuously, so no such
        swing is possible. Falls back to OMPL pose_target (_execute_pose_target)
        if the Cartesian path cannot reach the required fraction.
        """
        try:
            group = self._ensure_move_group()
        except MoveItCommanderException as exc:
            return False, "MoveIt not ready: %s" % exc

        target_link = self._segment_pose_target_link(segment)
        reference_frame = self._segment_pose_reference_frame(segment)
        previous_link = group.get_end_effector_link()
        delegated = False
        self._last_tilt_profile = {}
        diagnostics = {
            "segment": segment.name,
            "target_link": target_link,
            "reference_frame": reference_frame,
            "plan_type": "cartesian",
            "outcome": "cartesian_incomplete",
        }
        try:
            group.set_end_effector_link(target_link)
            group.set_pose_reference_frame(reference_frame)
            self._publish_target_marker(
                segment.target_pose, segment.name,
                target_link, reference_frame, success=True,
                extra_text="cartesian",
            )

            settle_summary = self._settle_before_plan(group)
            group.set_start_state_to_current_state()
            is_probe_insertion = segment.name in (
                "enter_opening", "probe_inside", "retreat_opening")
            if is_probe_insertion:
                current_names = group.get_active_joints()
                current_values = group.get_current_joint_values()
                valid, info = self._check_state_validity(
                    current_names, current_values
                )
                if not valid:
                    self._publish_validity_marker(
                        segment.target_pose, reference_frame, info
                    )
                    if segment.name != "retreat_opening":
                        return False, "%s start state invalid: %s" % (
                            segment.name, info,
                        )
                    rospy.logwarn(
                        "Retreating from invalid probe state: %s", info
                    )
            waypoints = [segment.target_pose]
            path_constraints = None
            if getattr(segment, "keep_camera_down", False) or segment.keep_tool_down:
                # Dual-down cartesian segments inherit the same camera+suction
                # constraints as pose_target; camera-only segments (probe) keep
                # the straight-down camera constraint.
                dual_down = bool(segment.keep_tool_down) and bool(
                    getattr(segment, "keep_camera_down", False))
                if dual_down and self._downward_orientations() is not None:
                    levels = self._build_constraint_levels(segment)
                    tool_yaw, cam_rp, cam_yaw, wrist_tols = levels[0]
                else:
                    tool_yaw = None
                    cam_rp = self._camera_down_rp_tolerance
                    cam_yaw = self._camera_down_yaw_tolerance
                    wrist_tols = None
                path_constraints, _mode = self._build_segment_constraints(
                    segment,
                    tool_yaw=tool_yaw,
                    cam_rp=cam_rp,
                    cam_yaw=cam_yaw,
                    wrist_tols=wrist_tols,
                    camera_ref_q=self._lookup_camera_reference_orientation(),
                )
                diagnostics["constraint_mode"] = _mode
                self._publish_active_constraints(
                    path_constraints, segment.name, _mode, 0,
                    segment.target_pose, reference_frame)
            rospy.loginfo(
                "Cartesian %s -> %s in %s xyz=(%.3f, %.3f, %.3f) [%s]",
                segment.name, target_link, reference_frame,
                segment.target_pose.position.x, segment.target_pose.position.y,
                segment.target_pose.position.z, settle_summary,
            )
            try:
                # NOTE: this Noetic moveit_commander signature is
                #   compute_cartesian_path(waypoints, eef_step,
                #                           avoid_collisions=True,
                #                           path_constraints=None)
                # There is NO jump_threshold param here (it raised
                # "unknown constraint type <class 'bool'>" when a threshold was
                # passed positionally and True landed in path_constraints).
                plan, fraction = group.compute_cartesian_path(
                    waypoints,
                    eef_step=self._cartesian_eef_step,
                    avoid_collisions=True,
                    path_constraints=path_constraints,
                )
                rospy.loginfo(
                    "Cartesian %s fraction=%.3f (need %.2f) points=%d",
                    segment.name, fraction, self._cartesian_min_fraction,
                    len(plan.joint_trajectory.points),
                )
                diagnostics["cartesian_fraction"] = float(fraction)
                if (
                    fraction >= self._cartesian_min_fraction
                    and plan.joint_trajectory.points
                ):
                    self._fold_trajectory_wrap_joints(plan)
                    dual_down = (
                        bool(segment.keep_tool_down)
                        and self._downward_orientations() is not None
                    )
                    if path_constraints is not None:
                        wrist_ok, wrist_info = self._check_wrist_trajectory_quality(
                            plan, _mode, dual_down=dual_down)
                        diagnostics["wrist_quality"] = wrist_info
                        if not wrist_ok:
                            diagnostics["outcome"] = "wrist_rejected"
                            return False, (
                                "Cartesian %s wrist quality rejected: %s"
                                % (segment.name, wrist_info)
                            )
                    down_ok, down_info = self._validate_downward_trajectory(
                        plan, segment)
                    diagnostics["downward_validation"] = down_info
                    diagnostics["tilt_profile"] = dict(self._last_tilt_profile)
                    if not down_ok and self._strict_downward:
                        diagnostics["outcome"] = "downward_rejected"
                        return False, (
                            "Cartesian %s downward quality rejected: %s"
                            % (segment.name, down_info)
                        )
                    if not down_ok:
                        rospy.logwarn(
                            "Cartesian %s downward tilt exceeded with "
                            "strict=False: %s", segment.name, down_info,
                        )
                    exec_ok, exec_message = self._execute_plan(group, plan)
                    if exec_ok:
                        release_ok, release_info = (
                            self._ensure_release_settled(segment))
                        diagnostics["release_settle"] = release_info
                        if not release_ok:
                            diagnostics.update({
                                "outcome": "release_settle_failed",
                                "message": release_info,
                            })
                            return False, (
                                "Cartesian %s %s"
                                % (segment.name, release_info))
                        if is_probe_insertion:
                            final_names = group.get_active_joints()
                            final_values = group.get_current_joint_values()
                            valid, info = self._check_state_validity(
                                final_names, final_values
                            )
                            if not valid:
                                self._publish_validity_marker(
                                    segment.target_pose,
                                    reference_frame,
                                    info,
                                )
                                diagnostics.update({
                                    "outcome": "final_state_invalid",
                                    "message": info,
                                })
                                return False, "%s final state invalid: %s" % (
                                    segment.name, info,
                                )
                        diagnostics["outcome"] = "success"
                        return True, "Reached cartesian %s (frac=%.3f) [%s]" % (
                            segment.name, fraction, settle_summary,
                        )
                    rospy.logwarn(
                        "Cartesian exec failed for %s (%s)",
                        segment.name, exec_message,
                    )
                    diagnostics.update({
                        "outcome": "execution_failed",
                        "message": exec_message,
                    })
                else:
                    rospy.logwarn(
                        "Cartesian %s fraction %.3f < %.2f",
                        segment.name, fraction, self._cartesian_min_fraction,
                    )
            except Exception as exc:
                rospy.logwarn(
                    "Cartesian %s raised %s",
                    segment.name, exc,
                )
                diagnostics.update({
                    "outcome": "cartesian_exception",
                    "message": str(exc),
                })

            if not getattr(segment, "allow_ompl_fallback", False):
                self._publish_failure_marker(
                    segment.target_pose, segment.name, reference_frame,
                    -31, "Cartesian path incomplete; OMPL fallback disabled",
                )
                diagnostics["outcome"] = "cartesian_incomplete_no_fallback"
                return False, (
                    "Cartesian %s did not reach required fraction; "
                    "OMPL fallback disabled" % segment.name
                )

            # Fallback: restore the EE link FIRST so _execute_pose_target
            # captures the true previous_link and restores it correctly itself.
            try:
                group.clear_pose_targets()
                group.set_start_state_to_current_state()
                if previous_link:
                    group.set_end_effector_link(previous_link)
            except MoveItCommanderException:
                pass
            delegated = True
            return self._execute_pose_target(segment)
        finally:
            if not delegated:
                self._publish_planning_diagnostics(diagnostics)
                try:
                    group.clear_pose_targets()
                    group.set_start_state_to_current_state()
                    if previous_link:
                        group.set_end_effector_link(previous_link)
                except MoveItCommanderException:
                    pass

    def handle_go_to_pose(self, req):
        pose_name = req.pose_name.strip() if req.pose_name else self._default_observe_pose
        try:
            pose_cfg = self._pose_config(pose_name)
        except KeyError as exc:
            return GoToRobotPoseResponse(
                success=False, already_there=False, message=str(exc)
            )

        joint_names = pose_cfg["joints"]
        target_values = pose_cfg["values"]
        tolerance = float(pose_cfg.get("tolerance", self._default_tolerance))
        group_name = pose_cfg.get("group", "elfin_arm")

        try:
            self._ensure_move_group(group_name)
        except MoveItCommanderException as exc:
            return GoToRobotPoseResponse(
                success=False,
                already_there=False,
                message="MoveIt not ready: %s" % exc,
            )

        success, already_there, message = self._move_to_joint_target(
            joint_names, target_values, tolerance, "pose '%s'" % pose_name
        )
        return GoToRobotPoseResponse(
            success=success, already_there=already_there, message=message
        )

    def handle_go_to_joint_values(self, req):
        joint_names = list(req.joint_names) if req.joint_names else list(DEFAULT_JOINT_NAMES)
        target_values = list(req.values)

        if len(joint_names) != len(target_values):
            return GoToJointValuesResponse(
                success=False,
                already_there=False,
                message="joint_names and values length mismatch (%d vs %d)"
                % (len(joint_names), len(target_values)),
            )

        success, already_there, message = self._move_to_joint_target(
            joint_names,
            target_values,
            self._default_tolerance,
            "joint target",
        )
        return GoToJointValuesResponse(
            success=success, already_there=already_there, message=message
        )

    def handle_aim_camera_at_container(self, req):
        rospy.loginfo(
            "aim_camera_at_container: frame=%s execute=%s xy_tol=%.3f",
            req.container_frame or "container_opening_frame",
            bool(req.execute),
            float(req.link6_xy_tolerance) if req.link6_xy_tolerance > 0.0 else 0.03,
        )

        container_frame = req.container_frame.strip() if req.container_frame else "container_opening_frame"
        xy_tol = float(req.link6_xy_tolerance) if req.link6_xy_tolerance > 0.0 else 0.03
        z_tol = float(req.link6_z_tolerance) if req.link6_z_tolerance > 0.0 else 0.15
        execute = bool(req.execute)

        try:
            opening_xyz = self._resolve_opening_xyz(container_frame)
            eye = self._lookup_xyz(self._optical_frame)
            link6_xyz = self._lookup_xyz(self._link6_frame)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            return AimCameraAtContainerResponse(
                success=False,
                already_there=False,
                joint_values=[],
                message="TF unavailable: %s" % exc,
            )

        rospy.loginfo(
            "aim targets: opening=%s eye=%s link6=%s",
            [round(v, 3) for v in opening_xyz],
            [round(v, 3) for v in eye],
            [round(v, 3) for v in link6_xyz],
        )

        aim_pose = Pose(
            position=Point(x=opening_xyz[0], y=opening_xyz[1], z=opening_xyz[2]),
            orientation=Quaternion(w=1.0),
        )
        self._publish_target_marker(
            aim_pose, "aim_camera", self._optical_frame, self._base_frame, success=True
        )

        try:
            group = self._ensure_move_group()
        except MoveItCommanderException as exc:
            return AimCameraAtContainerResponse(
                success=False,
                already_there=False,
                joint_values=[],
                message="MoveIt not ready: %s" % exc,
            )

        target_pose = optical_pose_look_at(eye, opening_xyz, frame_id=self._base_frame)
        target_pose.header.stamp = rospy.Time(0)

        observe_cfg = self._pose_config(self._default_observe_pose)
        joint_names = observe_cfg["joints"]
        observe_values = observe_cfg["values"]
        current_values = self._ordered_joint_values(group, joint_names)
        observe_values = self._nearest_equivalent_targets(
            joint_names, current_values, observe_values, "observe seed"
        )

        err_deg = view_axis_alignment_error(eye, opening_xyz, self._lookup_optical_z_axis())
        if err_deg < 5.0:
            rospy.loginfo("Camera already aligned (%.1f deg)", err_deg)
            return AimCameraAtContainerResponse(
                success=True,
                already_there=True,
                joint_values=current_values,
                message="Camera already aligned (%.1f deg)" % err_deg,
            )

        group.set_end_effector_link(self._optical_frame)
        seeds = build_joint_seeds(current_values, observe_values, opening_xyz[:2], link6_xyz[:2])
        ik_candidates = []
        for seed in seeds:
            solution = self._ik_from_seed(group, target_pose, seed, joint_names)
            if solution is not None:
                ik_candidates.append(solution)
                rospy.loginfo("IK seed ok: %s", [round(v, 3) for v in solution])

        best_joints, score = pick_closest_joint_solution(ik_candidates, current_values)
        if best_joints is None:
            return AimCameraAtContainerResponse(
                success=False,
                already_there=False,
                joint_values=[],
                message="No IK solution for container aim (check camera_depth_optical_frame in URDF)",
            )

        rospy.loginfo("Best IK joint delta norm: %.3f", score if score is not None else -1.0)

        if not execute:
            return AimCameraAtContainerResponse(
                success=True,
                already_there=False,
                joint_values=best_joints,
                message="IK ready (execute=false)",
            )

        fallback_levels = [
            (None, None, None, None),
            (xy_tol, z_tol, 1.8, 0.55),
            (xy_tol, z_tol, 1.2, 0.35),
        ]

        last_message = "Planning failed"
        for level_xy, level_z, j1_tol, other_tol in fallback_levels:
            path_constraints = None
            if level_xy is not None:
                constraints = Constraints()
                constraints.position_constraints.append(
                    self._build_link6_position_constraint(link6_xyz, level_xy, level_z)
                )
                constraints.joint_constraints = self._build_joint_constraints(
                    observe_values, joint_names, j1_tol, other_tol
                )
                path_constraints = constraints
                rospy.loginfo("Planning with link6 XY box +/-%.0fmm ...", level_xy * 1000.0)
            else:
                rospy.loginfo("Planning without path constraints ...")

            success, already_there, message = self._move_to_joint_target(
                joint_names,
                best_joints,
                self._default_tolerance,
                "container aim",
                path_constraints=path_constraints,
            )
            if success:
                return AimCameraAtContainerResponse(
                    success=True,
                    already_there=already_there,
                    joint_values=best_joints,
                    message=message,
                )
            last_message = message
            rospy.logwarn("Container aim attempt failed: %s", message)

        return AimCameraAtContainerResponse(
            success=False,
            already_there=False,
            joint_values=best_joints,
            message=last_message,
        )

    def _aim_service_wrapper(self, req):
        try:
            return self.handle_aim_camera_at_container(req)
        except Exception as exc:
            rospy.logerr("aim_camera_at_container failed: %s", exc, exc_info=True)
            return AimCameraAtContainerResponse(
                success=False,
                already_there=False,
                joint_values=[],
                message="Internal error: %s" % exc,
            )

    def _set_pickup_touch(self, allowed):
        """Toggle suction_panel<->box ACM via scene_manager (best-effort)."""
        try:
            self._set_pickup_touch_srv.wait_for_service(timeout=1.0)
            self._set_pickup_touch_srv(allowed)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn(
                "set_pickup_touch(%s) failed: %s (scene_manager down?)",
                allowed, exc,
            )

    def _set_place_support_touch(self, allowed):
        try:
            self._set_place_support_touch_srv.wait_for_service(timeout=1.0)
            self._set_place_support_touch_srv(allowed)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn(
                "set_place_support_touch(%s) failed: %s",
                allowed, exc)

    @staticmethod
    def _trajectory_end_state(plan):
        """Build a diff RobotState from the final point of a planned trajectory."""
        trajectory = getattr(plan, "joint_trajectory", None)
        points = list(getattr(trajectory, "points", []) or [])
        names = list(getattr(trajectory, "joint_names", []) or [])
        if not points or not names:
            return None
        state = RobotState()
        state.is_diff = True
        state.joint_state.name = names
        state.joint_state.position = list(points[-1].positions)
        return state

    @staticmethod
    def _planned_joint_margin(plan):
        """Return the minimum configured joint-limit margin in radians."""
        trajectory = getattr(plan, "joint_trajectory", None)
        points = list(getattr(trajectory, "points", []) or [])
        names = list(getattr(trajectory, "joint_names", []) or [])
        
        if not points or not names:
            return -1.0
        limits = rospy.get_param(
            "/robot_description_planning/joint_limits", {})
        minimum = None
        for point in points:
            for name, value in zip(names, point.positions):
                cfg = limits.get(name, {}) or {}
                if not (
                        cfg.get("has_position_limits", False)
                        and "min_position" in cfg
                        and "max_position" in cfg):
                    continue
                margin = min(
                    float(value) - float(cfg["min_position"]),
                    float(cfg["max_position"]) - float(value),
                )
                minimum = margin if minimum is None else min(minimum, margin)
        return -1.0 if minimum is None else float(minimum)

    @staticmethod
    def _planned_manipulability(group, plan):
        """Return Yoshikawa manipulability at the planned endpoint."""
        trajectory = getattr(plan, "joint_trajectory", None)
        points = list(getattr(trajectory, "points", []) or [])
        if not points:
            return -1.0
        try:
            jacobian = np.asarray(group.get_jacobian_matrix(
                list(points[-1].positions)), dtype=float)
            return float(np.prod(np.linalg.svd(
                jacobian, compute_uv=False)))
        except Exception:
            return -1.0

    def _validation_constraints(self, segment):
        if not (
                getattr(segment, "keep_camera_down", False)
                or getattr(segment, "keep_tool_down", False)):
            return None
        dual_down = bool(getattr(segment, "keep_tool_down", False)) and bool(
            getattr(segment, "keep_camera_down", False))
        if dual_down and self._downward_orientations() is not None:
            levels = self._build_constraint_levels(segment)
            tool_yaw, cam_rp, cam_yaw, wrist_tols = levels[0]
        else:
            tool_yaw = None
            cam_rp = self._camera_down_rp_tolerance
            cam_yaw = self._camera_down_yaw_tolerance
            wrist_tols = None
        constraints, _mode = self._build_segment_constraints(
            segment,
            tool_yaw=tool_yaw,
            cam_rp=cam_rp,
            cam_yaw=cam_yaw,
            wrist_tols=wrist_tols,
            camera_ref_q=self._lookup_camera_reference_orientation(),
        )
        return constraints

    def handle_validate_motion_sequence(self, req):
        """Plan a constrained sequence without executing or mutating scene state."""
        segments = list(req.segments or [])
        if not segments:
            return ValidateMotionSequenceResponse(
                success=False,
                message="validation sequence is empty",
                rejection_reason="empty_sequence",
                minimum_cartesian_fraction=0.0,
                minimum_joint_margin=-1.0,
                manipulability=-1.0,
                retreat_feasible=False,
            )
        try:
            group = self._ensure_move_group()
        except MoveItCommanderException as exc:
            return ValidateMotionSequenceResponse(
                success=False,
                message="MoveIt not ready: %s" % exc,
                rejection_reason="moveit_unavailable",
                minimum_cartesian_fraction=0.0,
                minimum_joint_margin=-1.0,
                manipulability=-1.0,
                retreat_feasible=False,
            )

        previous_link = group.get_end_effector_link()
        start_state = None
        min_fraction = 1.0
        min_margin = None
        min_manipulability = None
        retreat_planned = False
        try:
            for segment in segments:
                target_link = self._segment_pose_target_link(segment)
                reference_frame = self._segment_pose_reference_frame(segment)
                group.set_end_effector_link(target_link)
                group.set_pose_reference_frame(reference_frame)
                group.clear_pose_targets()
                group.clear_path_constraints()
                if start_state is None:
                    group.set_start_state_to_current_state()
                else:
                    group.set_start_state(start_state)
                constraints = self._validation_constraints(segment)
                if constraints is not None:
                    group.set_path_constraints(constraints)

                if segment.type == "cartesian":
                    plan, fraction = group.compute_cartesian_path(
                        [segment.target_pose],
                        eef_step=self._cartesian_eef_step,
                        avoid_collisions=True,
                        path_constraints=constraints,
                    )
                    min_fraction = min(min_fraction, float(fraction))
                    valid_plan = (
                        fraction >= self._cartesian_min_fraction
                        and bool(plan.joint_trajectory.points)
                    )
                    reason = "cartesian_fraction"
                elif segment.type == "pose_target":
                    group.set_pose_target(segment.target_pose, target_link)
                    success, plan, _planning_time, _error_code = group.plan()
                    valid_plan = bool(
                        success and plan.joint_trajectory.points)
                    reason = "pose_planning_failed"
                else:
                    return ValidateMotionSequenceResponse(
                        success=False,
                        message="unsupported validation segment type %s"
                        % segment.type,
                        rejection_reason="unsupported_segment",
                        minimum_cartesian_fraction=min_fraction,
                        minimum_joint_margin=(
                            -1.0 if min_margin is None else min_margin),
                        manipulability=-1.0,
                        retreat_feasible=False,
                    )

                if not valid_plan:
                    return ValidateMotionSequenceResponse(
                        success=False,
                        message="validation failed at %s" % segment.name,
                        rejection_reason="%s:%s" % (segment.name, reason),
                        minimum_cartesian_fraction=min_fraction,
                        minimum_joint_margin=(
                            -1.0 if min_margin is None else min_margin),
                        manipulability=-1.0,
                        retreat_feasible=False,
                    )
                down_ok, down_info = self._validate_downward_trajectory(
                    plan, segment)
                if not down_ok and self._strict_downward:
                    return ValidateMotionSequenceResponse(
                        success=False,
                        message="validation failed at %s: %s"
                        % (segment.name, down_info),
                        rejection_reason="%s:downward_constraint"
                        % segment.name,
                        minimum_cartesian_fraction=min_fraction,
                        minimum_joint_margin=(
                            -1.0 if min_margin is None else min_margin),
                        manipulability=-1.0,
                        retreat_feasible=False,
                    )
                margin = self._planned_joint_margin(plan)
                if margin >= 0.0:
                    min_margin = (
                        margin if min_margin is None
                        else min(min_margin, margin))
                manipulability = self._planned_manipulability(group, plan)
                if manipulability >= 0.0:
                    min_manipulability = (
                        manipulability
                        if min_manipulability is None
                        else min(min_manipulability, manipulability))
                start_state = self._trajectory_end_state(plan)
                if start_state is None:
                    return ValidateMotionSequenceResponse(
                        success=False,
                        message="validation produced empty trajectory at %s"
                        % segment.name,
                        rejection_reason="%s:empty_trajectory" % segment.name,
                        minimum_cartesian_fraction=min_fraction,
                        minimum_joint_margin=(
                            -1.0 if min_margin is None else min_margin),
                        manipulability=-1.0,
                        retreat_feasible=False,
                    )
                retreat_planned = segment.name == "retreat_opening"

            return ValidateMotionSequenceResponse(
                success=True,
                message="validated %d motion segments" % len(segments),
                rejection_reason="",
                minimum_cartesian_fraction=min_fraction,
                minimum_joint_margin=(
                    -1.0 if min_margin is None else min_margin),
                manipulability=(
                    -1.0 if min_manipulability is None
                    else min_manipulability),
                retreat_feasible=retreat_planned,
            )
        except Exception as exc:
            rospy.logwarn("Motion sequence validation failed: %s", exc)
            return ValidateMotionSequenceResponse(
                success=False,
                message="validation exception: %s" % exc,
                rejection_reason="validation_exception",
                minimum_cartesian_fraction=min_fraction,
                minimum_joint_margin=(
                    -1.0 if min_margin is None else min_margin),
                manipulability=-1.0,
                retreat_feasible=False,
            )
        finally:
            try:
                group.clear_pose_targets()
                group.clear_path_constraints()
                group.set_start_state_to_current_state()
                if previous_link:
                    group.set_end_effector_link(previous_link)
            except MoveItCommanderException:
                pass

    def handle_plan_motion(self, req):
        self._current_segment_name = req.segment.name
        try:
            if req.segment.type == "aim_camera":
                aim_req = AimCameraAtContainerRequest()
                aim_req.container_frame = "container_opening_frame"
                aim_req.link6_xy_tolerance = 0.03
                aim_req.link6_z_tolerance = 0.15
                aim_req.execute = True
                resp = self.handle_aim_camera_at_container(aim_req)
                return PlanMotionResponse(success=resp.success, message=resp.message)

            if req.segment.type == "pose_target":
                self._set_pickup_touch(self._is_pickup_touch_segment(req.segment))
                self._set_place_support_touch(
                    req.segment.name in self._place_support_touch_segments)
                success, message = self._execute_pose_target(req.segment)
                return PlanMotionResponse(success=success, message=message)

            if req.segment.type == "cartesian":
                self._set_pickup_touch(self._is_pickup_touch_segment(req.segment))
                self._set_place_support_touch(
                    req.segment.name in self._place_support_touch_segments)
                if req.segment.name in self._clear_octomap_segments:
                    self._clear_octomap(req.segment.name)
                success, message = self._execute_cartesian_target(req.segment)
                return PlanMotionResponse(success=success, message=message)
        finally:
            self._current_segment_name = ""

    def _clear_octomap(self, segment_name):
        """Clear the MoveIt octomap immediately before a contact-descent segment.

        The octomap updates at 2Hz and is cumulative, so even with the
        pickup_box_pointcloud_filter stripping the box, residual box-surface
        voxels (handle ridges, OBB-edge points, 2.5cm voxel quantization) get
        re-inserted between PlanPick's clear and the attach descent several
        seconds later. Those stale voxels block the cup from reaching the box
        top (Cartesian stops short, OMPL goal is GOAL_IN_COLLISION vs <octomap>).
        Clearing here removes them; the box is still a collision object via
        current_pickup_box (+ ACM for the panel), and the panel is already
        directly above the box so the cleared region contains no other obstacle
        the descent would hit. Best-effort: a failure logs but does not abort.
        """
        try:
            rospy.wait_for_service("/clear_octomap", timeout=1.0)
            self._clear_octomap_srv()
            rospy.loginfo("Cleared MoveIt octomap before %s descent", segment_name)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn("clear_octomap before %s failed: %s", segment_name, exc)

    def _is_pickup_touch_segment(self, segment):
        """True for segments that need the panel allowed to touch the box
        (the post-pre_grasp descent/lift: approach/attach/retreat). pre_grasp
        and all non-pick segments keep the touch ENFORCED so the panel cannot
        clip the box while repositioning to directly above it."""
        return segment.name in self._pickup_touch_segments



# Log level must be chosen before init_node, so it cannot come from a private
# param; log_level_utils reads the LUGGAGE_LOG_LEVEL environment variable.
import os as _os
import sys as _sys
import rospkg as _rospkg
_DESC = _os.path.join(
    _rospkg.RosPack().get_path("luggage_description"), "scripts")
if _DESC not in _sys.path:
    _sys.path.insert(0, _DESC)
from log_level_utils import resolve_log_level  # noqa: E402

def main():
    rospy.init_node("motion_planner", log_level=resolve_log_level())
    planner = MotionPlanner()
    rospy.Service("~go_to_robot_pose", GoToRobotPose, planner.handle_go_to_pose)
    rospy.Service("~go_to_joint_values", GoToJointValues, planner.handle_go_to_joint_values)
    rospy.Service("~aim_camera_at_container", AimCameraAtContainer, planner._aim_service_wrapper)
    rospy.Service("~plan_motion", PlanMotion, planner.handle_plan_motion)
    rospy.Service(
        "~validate_motion_sequence",
        ValidateMotionSequence,
        planner.handle_validate_motion_sequence,
    )
    rospy.loginfo(
        "motion_planner ready (go_to_robot_pose, go_to_joint_values, "
        "aim_camera_at_container, plan_motion, validate_motion_sequence)"
    )
    rospy.spin()


if __name__ == "__main__":
    main()
