#!/usr/bin/env python3
"""Pose-tune-only workspace reachability / collision preview (no scene_manager)."""

from __future__ import division

import json
import math
import os
import sys
import time

import moveit_commander
import rospy
import rospkg
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from moveit_commander import PlanningSceneInterface
from moveit_msgs.msg import PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK, GetStateValidity, GetStateValidityRequest
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA, String
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

DESC_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_description"), "scripts")
PLAN_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_planning"), "scripts")
for path in (DESC_SCRIPTS, PLAN_SCRIPTS):
    if path not in sys.path:
        sys.path.insert(0, path)

from scene_tf_config_utils import (  # noqa: E402
    container_in_base_link,
    container_inner_floor_z,
    container_usable_dimensions,
    container_opening_in_base_link,
    container_opening_target_point,
)
from exploration_config_utils import (  # noqa: E402
    default_exploration_path,
    exploration_joint_names,
    load_exploration_config,
    view_planning_constraints,
)
from container_aim_utils import look_at_quaternion  # noqa: E402
from pose_tune_draft_utils import (  # noqa: E402
    DRAFT_PARAM,
    JOINT_CHECK_PARAM,
    get_draft,
    init_draft_from_production,
)
from scene_mesh_utils import (  # noqa: E402
    container_collision_mesh_path,
    container_model_name,
    require_existing_mesh,
)


class PoseTuneWorkspaceAnalyzer:
    def __init__(self):
        moveit_commander.roscpp_initialize([])
        self._base_frame = rospy.get_param("~base_frame", "elfin_base_link")
        self._group = rospy.get_param("~move_group", "elfin_arm")
        self._ik_link = rospy.get_param("~ik_link", "camera_depth_optical_frame")
        self._placement_link = rospy.get_param("~placement_link", "elfin_link6")
        self._exploration_config = rospy.get_param("~exploration_config", default_exploration_path())
        self._ik_service = rospy.get_param("~ik_service", "/compute_ik")
        self._validity_service = rospy.get_param("~validity_service", "/check_state_validity")
        self._ik_timeout = float(rospy.get_param("~ik_timeout", 0.15))
        self._view_step = float(rospy.get_param("~view_grid_step", 0.20))
        self._view_y_span = float(rospy.get_param("~view_y_span", 0.6))
        self._view_z_span = float(rospy.get_param("~view_z_span", 0.4))
        self._enable_view = bool(rospy.get_param("~enable_view_reachability", True))
        self._enable_placement = bool(rospy.get_param("~enable_placement_reachability", False))
        self._wait_timeout = float(rospy.get_param("~moveit_wait_timeout", 120.0))
        self._joint_state_wait_timeout = float(rospy.get_param("~joint_state_wait_timeout", 15.0))
        self._container_collision_mesh = rospy.get_param("~container_collision_mesh", "")

        self._scene = PlanningSceneInterface(synchronous=True)
        self._markers_pub = rospy.Publisher(
            "/luggage/pose_tune/workspace_markers", MarkerArray, queue_size=1, latch=True
        )
        self._summary_pub = rospy.Publisher(
            "/luggage/pose_tune/workspace_summary", String, queue_size=1, latch=True
        )
        self._last_pose_status = "unknown"

        if not rospy.has_param(DRAFT_PARAM):
            init_draft_from_production(rospy)

        self._ik = None
        self._validity = None
        self._joint_names = exploration_joint_names(load_exploration_config(self._exploration_config))
        self._current_joints = {}
        self._joint_sub = rospy.Subscriber("/joint_states", JointState, self._on_joint_states, queue_size=1)

        rospy.Service("~analyze", Trigger, self._handle_analyze)
        rospy.Service("~check_pose", Trigger, self._handle_check_pose)
        rospy.Service("~validate_presets", Trigger, self._handle_validate_presets)
        rospy.loginfo("pose_tune_workspace_analyzer ready (preview-only, no scene_manager)")

    def _ensure_moveit_services(self):
        if self._ik is None:
            try:
                rospy.wait_for_service(self._ik_service, timeout=self._wait_timeout)
                self._ik = rospy.ServiceProxy(self._ik_service, GetPositionIK)
            except rospy.ROSException:
                rospy.logwarn("IK service %s unavailable", self._ik_service)
        if self._validity is None:
            try:
                rospy.wait_for_service(self._validity_service, timeout=5.0)
                self._validity = rospy.ServiceProxy(self._validity_service, GetStateValidity)
            except rospy.ROSException:
                rospy.logwarn("Validity service %s unavailable", self._validity_service)

    def _on_joint_states(self, msg):
        if not msg.name or not msg.position:
            return
        for name, value in zip(msg.name, msg.position):
            self._current_joints[name] = float(value)

    def _current_seed_values(self):
        if not all(name in self._current_joints for name in self._joint_names):
            return None
        return [self._current_joints[name] for name in self._joint_names]

    def _wait_for_joint_state(self, timeout=None):
        timeout = self._joint_state_wait_timeout if timeout is None else timeout
        deadline = time.time() + timeout
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            values = self._current_seed_values()
            if values is not None:
                return values
            if time.time() >= deadline:
                missing = [name for name in self._joint_names if name not in self._current_joints]
                rospy.logwarn(
                    "No complete /joint_states seed for pose_tune preview; missing: %s",
                    ", ".join(missing),
                )
                return None
            rate.sleep()

    def _quat_msg(self, qdict):
        q = Quaternion()
        q.x = qdict["x"]
        q.y = qdict["y"]
        q.z = qdict["z"]
        q.w = qdict["w"]
        return q

    @staticmethod
    def _rpy_to_quaternion(rpy):
        roll, pitch, yaw = rpy
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        return Quaternion(
            x=sr * cp * cy - cr * sp * sy,
            y=cr * sp * cy + sr * cp * sy,
            z=cr * cp * sy - sr * sp * cy,
            w=cr * cp * cy + sr * sp * sy,
        )

    def _preview_object_ids(self, draft):
        prefix = "pose_tune_preview_"
        return [
            prefix + container_model_name(draft),
            prefix + "container_floor",
            prefix + "container_ceiling",
            prefix + "container_back_wall",
            prefix + "container_left_wall",
            prefix + "container_right_wall",
            prefix + "container_front_left",
            prefix + "container_front_right",
            prefix + "container_front_bottom",
            prefix + "container_front_top",
            prefix + "robot_pedestal",
        ]

    def _container_mesh_path(self, draft):
        if self._container_collision_mesh:
            return require_existing_mesh(self._container_collision_mesh)
        return require_existing_mesh(container_collision_mesh_path(draft))

    def _sync_preview_scene(self, draft):
        try:
            known = set(self._scene.get_known_object_names())
        except Exception:
            known = set()
        for obj_id in self._preview_object_ids(draft):
            if obj_id in known:
                self._scene.remove_world_object(obj_id)
        base_xyz, base_rpy = container_in_base_link(draft)
        pose = PoseStamped()
        pose.header.frame_id = self._base_frame
        pose.pose.position = Point(x=base_xyz[0], y=base_xyz[1], z=base_xyz[2])
        pose.pose.orientation = self._rpy_to_quaternion(base_rpy)
        obj_id = "pose_tune_preview_" + container_model_name(draft)
        self._scene.add_mesh(
            obj_id,
            pose,
            filename=self._container_mesh_path(draft),
            size=(1.0, 1.0, 1.0),
        )
        rospy.sleep(0.3)

    def _robot_state_from_joints(self, joint_names, values):
        state = RobotState()
        state.joint_state = JointState(name=list(joint_names), position=[float(v) for v in values])
        return state

    def _check_joints_valid(self, joint_names, values):
        self._ensure_moveit_services()
        if self._validity is None:
            return None
        req = GetStateValidityRequest()
        req.robot_state = self._robot_state_from_joints(joint_names, values)
        req.group_name = self._group
        try:
            resp = self._validity(req)
            return bool(resp.valid)
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(5.0, "check_state_validity failed: %s", exc)
            return None

    def _check_view_ik(self, camera_xyz, look_at, seed_values=None):
        self._ensure_moveit_services()
        if self._ik is None:
            return None
        if seed_values is None:
            seed_values = self._current_seed_values()
        if seed_values is None or len(seed_values) != len(self._joint_names):
            return None
        q = look_at_quaternion(camera_xyz, look_at)
        req = PositionIKRequest()
        req.group_name = self._group
        req.ik_link_name = self._ik_link
        req.avoid_collisions = True
        req.timeout = rospy.Duration(self._ik_timeout)
        req.pose_stamped = PoseStamped()
        req.pose_stamped.header.frame_id = self._base_frame
        req.pose_stamped.pose = Pose(
            position=Point(x=camera_xyz[0], y=camera_xyz[1], z=camera_xyz[2]),
            orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
        )
        req.robot_state = self._robot_state_from_joints(self._joint_names, seed_values)
        try:
            resp = self._ik(req)
            return resp.error_code.val == resp.error_code.SUCCESS
        except rospy.ServiceException:
            return False

    def _check_placement_ik(self, xyz, yaw=0.0):
        self._ensure_moveit_services()
        if self._ik is None:
            return None
        seed_values = self._current_seed_values()
        if seed_values is None:
            return None
        c = math.cos(yaw * 0.5)
        s = math.sin(yaw * 0.5)
        req = PositionIKRequest()
        req.group_name = self._group
        req.ik_link_name = self._placement_link
        req.avoid_collisions = True
        req.timeout = rospy.Duration(self._ik_timeout)
        req.pose_stamped = PoseStamped()
        req.pose_stamped.header.frame_id = self._base_frame
        req.pose_stamped.pose = Pose(
            position=Point(x=xyz[0], y=xyz[1], z=xyz[2]),
            orientation=Quaternion(x=c, y=s, z=0.0, w=0.0),
        )
        req.robot_state = self._robot_state_from_joints(self._joint_names, seed_values)
        try:
            resp = self._ik(req)
            return resp.error_code.val == resp.error_code.SUCCESS
        except rospy.ServiceException:
            return False

    def _view_grid_samples(self, draft, constraints):
        opening_xyz = container_opening_target_point(draft)
        inner_l, inner_w, inner_h = container_usable_dimensions(draft)
        look_depth = inner_w * 0.3
        look_down = 0.15
        look_at = [opening_xyz[0], opening_xyz[1] - look_depth, opening_xyz[2] - look_down]
        standoff = 0.10
        height_off = 0.25
        center_cam = [
            opening_xyz[0],
            opening_xyz[1] + standoff,
            opening_xyz[2] + height_off,
        ]
        samples = []
        y_steps = max(1, int(self._view_y_span / self._view_step))
        z_steps = max(1, int(self._view_z_span / self._view_step))
        for yi in range(-y_steps, y_steps + 1):
            for zi in range(-z_steps, z_steps + 1):
                cam = [
                    center_cam[0],
                    center_cam[1] + yi * self._view_step,
                    center_cam[2] + zi * self._view_step,
                ]
                if cam[2] > constraints.get("camera_z_max", 1.45) + 1e-6:
                    samples.append((cam, look_at, "height"))
                else:
                    samples.append((cam, look_at, "sample"))
        return samples

    def _placement_grid_samples(self, draft):
        opening_xyz, _ = container_opening_in_base_link(draft)
        inner_l, inner_w, _inner_h = container_usable_dimensions(draft)
        container_xyz, _ = container_in_base_link(draft)
        floor_z = container_xyz[2] + container_inner_floor_z(draft)
        samples = []
        step = self._view_step
        xs = frange(-inner_l * 0.35, inner_l * 0.35, step)
        ys = frange(opening_xyz[1] - inner_w * 0.75, opening_xyz[1] - inner_w * 0.15, step)
        z = floor_z + 0.05
        for x in xs:
            for y in ys:
                samples.append([x, y, z])
        return samples

    def _publish_markers(self, points):
        markers = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)
        stamp = rospy.Time.now()
        for idx, item in enumerate(points):
            m = Marker()
            m.header.frame_id = self._base_frame
            m.header.stamp = stamp
            m.ns = "workspace"
            m.id = idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = Point(item["xyz"][0], item["xyz"][1], item["xyz"][2])
            m.scale.x = m.scale.y = m.scale.z = 0.06
            status = item["status"]
            if status == "ok":
                m.color = ColorRGBA(0.1, 0.9, 0.2, 0.85)
            elif status == "collision":
                m.color = ColorRGBA(0.95, 0.15, 0.1, 0.85)
            elif status == "height":
                m.color = ColorRGBA(1.0, 0.6, 0.0, 0.75)
            else:
                m.color = ColorRGBA(0.6, 0.6, 0.6, 0.65)
            markers.markers.append(m)
        self._markers_pub.publish(markers)

    def _publish_summary(self, summary):
        self._summary_pub.publish(String(data=json.dumps(summary)))

    def analyze(self):
        seed_values = self._wait_for_joint_state()
        if seed_values is None:
            summary = {
                "reachable_pct": 0.0,
                "reachable": 0,
                "checked": 0,
                "height_violations": 0,
                "ik_fail": 0,
                "collision": 0,
                "last_pose_status": "waiting_for_joint_states",
            }
            self._publish_markers([])
            self._publish_summary(summary)
            return summary
        draft = get_draft(rospy)
        self._sync_preview_scene(draft)
        exploration = load_exploration_config(self._exploration_config)
        constraints = view_planning_constraints(exploration)
        marker_points = []
        reachable = 0
        collision = 0
        height_viol = 0
        ik_fail = 0
        checked = 0

        if self._enable_view:
            for cam, look_at, kind in self._view_grid_samples(draft, constraints):
                if kind == "height":
                    height_viol += 1
                    marker_points.append({"xyz": cam, "status": "height"})
                    continue
                result = self._check_view_ik(cam, look_at, seed_values=seed_values)
                checked += 1
                if result is None:
                    marker_points.append({"xyz": cam, "status": "unknown"})
                elif result:
                    reachable += 1
                    marker_points.append({"xyz": cam, "status": "ok"})
                else:
                    ik_fail += 1
                    marker_points.append({"xyz": cam, "status": "fail"})

        if self._enable_placement:
            for xyz in self._placement_grid_samples(draft):
                result = self._check_placement_ik(xyz)
                checked += 1
                if result:
                    reachable += 1
                    marker_points.append({"xyz": xyz, "status": "ok"})
                else:
                    ik_fail += 1
                    marker_points.append({"xyz": xyz, "status": "fail"})

        total = max(1, checked + height_viol)
        summary = {
            "reachable_pct": round(100.0 * reachable / total, 1),
            "reachable": reachable,
            "checked": checked,
            "height_violations": height_viol,
            "ik_fail": ik_fail,
            "collision": collision,
            "last_pose_status": self._last_pose_status,
        }
        self._publish_markers(marker_points)
        self._publish_summary(summary)
        return summary

    def check_pose_from_param(self):
        if not rospy.has_param(JOINT_CHECK_PARAM):
            return None, "no joint values set"
        data = rospy.get_param(JOINT_CHECK_PARAM)
        names = data.get("names", self._joint_names)
        values = data.get("values", [])
        if len(values) != len(names):
            return None, "joint count mismatch"
        draft = get_draft(rospy)
        self._sync_preview_scene(draft)
        valid = self._check_joints_valid(names, values)
        if valid is None:
            self._last_pose_status = "unknown"
            return None, "validity service unavailable"
        if valid:
            self._last_pose_status = "ok"
            return True, "OK"
        self._last_pose_status = "collision"
        return False, "COLLISION"

    def validate_presets(self):
        return "No fixed exploration presets; use smart workspace analysis."

    def _handle_analyze(self, _req):
        try:
            summary = self.analyze()
            return TriggerResponse(
                success=True,
                message="reachable %.1f%% (%d/%d)"
                % (summary["reachable_pct"], summary["reachable"], summary["checked"]),
            )
        except Exception as exc:
            rospy.logerr("analyze failed: %s", exc)
            return TriggerResponse(success=False, message=str(exc))

    def _handle_check_pose(self, _req):
        try:
            valid, msg = self.check_pose_from_param()
            if valid is None:
                return TriggerResponse(success=False, message=msg)
            return TriggerResponse(success=bool(valid), message=msg)
        except Exception as exc:
            return TriggerResponse(success=False, message=str(exc))

    def _handle_validate_presets(self, _req):
        try:
            report = self.validate_presets()
            return TriggerResponse(success=True, message=report)
        except Exception as exc:
            return TriggerResponse(success=False, message=str(exc))


def frange(start, stop, step):
    values = []
    x = start
    while x <= stop + 1e-9:
        values.append(x)
        x += step
    return values


def main():
    rospy.init_node("pose_tune_workspace_analyzer")
    PoseTuneWorkspaceAnalyzer()
    rospy.spin()


if __name__ == "__main__":
    main()
