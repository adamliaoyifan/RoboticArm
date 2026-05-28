#!/usr/bin/env python3
"""Plan and execute motions via MoveIt (observe reset + container aim)."""

import os
import sys
import threading

import actionlib
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
    PositionConstraint,
    RobotState,
)
from shape_msgs.msg import SolidPrimitive

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
from container_config_utils import (  # noqa: E402
    default_config_path,
    load_container_config,
    opening_target_point,
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
        self._tf_wait_timeout = float(rospy.get_param("~tf_wait_timeout", 5.0))
        self._container_config_path = rospy.get_param("~container_config", default_config_path())
        self._optical_frame = rospy.get_param("~optical_frame", OPTICAL_FRAME)
        self._link6_frame = rospy.get_param("~link6_frame", LINK6_FRAME)
        self._base_frame = rospy.get_param("~base_frame", BASE_FRAME)
        self._group = None
        self._move_group_ready = False
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

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
        """Return opening center [x,y,z] in elfin_base_link (TF first, then yaml)."""
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
            config = load_container_config(self._container_config_path)
            xyz = opening_target_point(config)
            rospy.logwarn(
                "Using opening xyz from %s (no TF %s -> %s)",
                self._container_config_path,
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

    def _execute_plan(self, group, plan):
        result = {"done": False, "ok": False}

        def _run():
            result["ok"] = bool(group.execute(plan, wait=True))
            result["done"] = True

        thread = threading.Thread(target=_run)
        thread.daemon = True
        thread.start()
        thread.join(self._execute_timeout)
        if not result["done"]:
            try:
                group.stop()
            except MoveItCommanderException:
                pass
            return False, "Execution timed out after %.0fs (is Gazebo + elfin_arm_controller running?)" % self._execute_timeout
        if not result["ok"]:
            return False, "Execution rejected by move_group"
        return True, "Reached target"

    def _move_to_joint_target(self, joint_names, target_values, tolerance, label, path_constraints=None):
        try:
            group = self._ensure_move_group()
        except MoveItCommanderException as exc:
            return False, False, "MoveIt not ready: %s" % exc

        current_values = self._ordered_joint_values(group, joint_names)
        if self._within_tolerance(current_values, target_values, tolerance):
            rospy.loginfo("Already at target: %s", label)
            return True, True, "Already at target: %s" % label

        if path_constraints is not None:
            group.set_path_constraints(path_constraints)

        group.set_joint_value_target(dict(zip(joint_names, target_values)))
        rospy.loginfo("Planning move to %s ...", label)
        success, plan, _planning_time, error_code = group.plan()
        if path_constraints is not None:
            group.clear_path_constraints()

        if not success or not plan.joint_trajectory.points:
            return False, False, "Planning failed for %s (error %s)" % (label, error_code.val)

        exec_ok, exec_message = self._execute_plan(group, plan)
        if not exec_ok:
            return False, False, exec_message

        rospy.loginfo("Reached %s", label)
        return True, False, "Reached %s" % label

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

    def handle_plan_motion(self, req):
        if req.segment.type == "aim_camera":
            aim_req = AimCameraAtContainerRequest()
            aim_req.container_frame = "container_opening_frame"
            aim_req.link6_xy_tolerance = 0.03
            aim_req.link6_z_tolerance = 0.15
            aim_req.execute = True
            resp = self.handle_aim_camera_at_container(aim_req)
            return PlanMotionResponse(success=resp.success, message=resp.message)

        rospy.logwarn(
            "PlanMotion stub — segment=%s type=%s",
            req.segment.name,
            req.segment.type,
        )
        return PlanMotionResponse(success=True, message="stub")


def main():
    rospy.init_node("motion_planner")
    planner = MotionPlanner()
    rospy.Service("~go_to_robot_pose", GoToRobotPose, planner.handle_go_to_pose)
    rospy.Service("~go_to_joint_values", GoToJointValues, planner.handle_go_to_joint_values)
    rospy.Service("~aim_camera_at_container", AimCameraAtContainer, planner._aim_service_wrapper)
    rospy.Service("~plan_motion", PlanMotion, planner.handle_plan_motion)
    rospy.loginfo(
        "motion_planner ready (go_to_robot_pose, go_to_joint_values, aim_camera_at_container, plan_motion)"
    )
    rospy.spin()


if __name__ == "__main__":
    main()
