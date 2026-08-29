#!/usr/bin/env python3
"""Sync static container collision objects into MoveIt planning scene."""

import math
import os
import sys

import rospy
import rospkg
import tf2_ros
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from moveit_commander import PlanningSceneInterface
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest
from std_msgs.msg import ColorRGBA
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from visualization_msgs.msg import Marker

try:  # registers PoseStamped <-> tf2 conversion for the TF sanity check
    import tf2_geometry_msgs  # noqa: F401
except ImportError:
    pass

from luggage_msgs.srv import SyncStaticScene, SyncStaticSceneResponse
from luggage_msgs.srv import SyncPickupBox, SyncPickupBoxResponse
from luggage_msgs.srv import AddPlacedBox, AddPlacedBoxResponse
from luggage_msgs.srv import RemovePlacedBox, RemovePlacedBoxResponse
from luggage_msgs.srv import GetCurrentBox

DESC_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_description"), "scripts")
if DESC_SCRIPTS not in sys.path:
    sys.path.insert(0, DESC_SCRIPTS)

from scene_tf_config_utils import (  # noqa: E402
    base_in_world,
    container_in_base_link,
    load_scene_tf_config,
    pedestal_center_in_base_link,
    pedestal_config,
    pedestal_dimensions,
    pedestal_enabled,
    resolve_scene_tf_config_path,
    _compose,
    _invert_transform,
)
from scene_mesh_utils import (  # noqa: E402
    container_collision_mesh_path,
    container_model_name,
    require_existing_mesh,
)


class SceneManager:
    def __init__(self):
        self._scene_tf_config = rospy.get_param(
            "~scene_tf_config",
            rospy.get_param("/luggage/scene_tf_config", resolve_scene_tf_config_path()),
        )
        self._base_frame = rospy.get_param("~base_frame", "elfin_base_link")
        self._container_collision_mesh = rospy.get_param("~container_collision_mesh", "")
        self._scene = None
        self._placed_ids = []
        self._pickup_object_id = rospy.get_param("~pickup_object_id", "current_pickup_box")
        self._pickup_box_service = rospy.get_param(
            "~pickup_box_service", "/pickup_box_spawner/get_current_box"
        )
        # Allowed-collision: permit the suction panel (and any configured touch
        # links) to contact the pickup box so the attach descent reaches the box
        # top. PlanningSceneInterface has no ACM setter in Noetic, so we publish
        # a PlanningScene diff (fetch -> merge -> republish) in
        # _apply_pickup_touch().
        #
        # SEGMENT-SCOPED: touch starts ENFORCED (panel collides with box) so
        # pre_grasp reaches directly above the box without clipping it. The
        # motion planner calls ~set_pickup_touch(True) only for the post-pre_grasp
        # segments (approach/attach/retreat) and (False) again after retreat, so
        # the panel may touch the box ONLY while descending onto / lifting off it.
        # Other links always collide with the box (ACM relaxes only the panel).
        self._pickup_touch_links = [
            s.strip()
            for s in rospy.get_param("~pickup_touch_links", "suction_panel").split(",")
            if s.strip()
        ]
        self._pickup_touch_enabled = False
        self._place_support_touch_enabled = False
        # None until the first ACM update, so the initial state is reported once.
        self._pickup_acm_allowed = None
        self._pickup_attached = False
        self._attach_link = rospy.get_param("~pickup_attach_link", "suction_panel")
        self._planning_scene_pub = rospy.Publisher(
            "/planning_scene", PlanningScene, queue_size=1, latch=True
        )
        self._get_planning_scene = rospy.ServiceProxy(
            "/get_planning_scene", GetPlanningScene
        )
        # Diagnostics: cross-check the box pose vs the live world->base TF and
        # publish a wireframe of the collision object so RViz can confirm it
        # overlaps the real Gazebo box at a glance.
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._box_debug_pub = rospy.Publisher(
            "/luggage/debug/pickup_box_scene", Marker, queue_size=1, latch=True
        )

    def _ensure_scene(self):
        if self._scene is None:
            moveit_commander.roscpp_initialize([])
            self._scene = PlanningSceneInterface(synchronous=True)
        return self._scene

    def _remove_container_objects(self, scene):
        for name in (
            "airport_container",
            "airport_container_real",
        ):
            scene.remove_world_object(name)

    def _container_mesh_path(self, scene_config):
        if self._container_collision_mesh:
            return require_existing_mesh(self._container_collision_mesh)
        return require_existing_mesh(container_collision_mesh_path(scene_config))

    def _add_container_mesh(self, scene, scene_config, base_xyz, base_rpy):
        mesh_path = self._container_mesh_path(scene_config)
        pose = PoseStamped()
        pose.header.frame_id = self._base_frame
        pose.pose.position = Point(x=base_xyz[0], y=base_xyz[1], z=base_xyz[2])
        pose.pose.orientation = self._rpy_to_quaternion(base_rpy)
        obj_id = container_model_name(scene_config)
        scene.add_mesh(obj_id, pose, filename=mesh_path, size=(1.0, 1.0, 1.0))
        rospy.loginfo(
            "Added %s mesh collision from %s at %s",
            obj_id,
            mesh_path,
            [round(v, 3) for v in base_xyz],
        )
        return obj_id

    def _remove_pickup_objects(self, scene):
        scene.remove_world_object(self._pickup_object_id)
        scene.remove_world_object("pickup_box")

    def _world_pose_to_base_pose(self, world_pose, scene_config):
        """Convert a Gazebo/world pose into the MoveIt base frame.

        Current scene configs keep the robot base yaw-only, but this uses the
        shared scene_tf math so pedestal translations/rotations stay aligned.
        """
        world_t = [
            world_pose.position.x,
            world_pose.position.y,
            world_pose.position.z,
        ]
        world_r = self._quaternion_to_rpy(world_pose.orientation)
        base_t, base_r = base_in_world(scene_config)
        base_inv_t, base_inv_r = _invert_transform(base_t, base_r)
        base_xyz, base_pose_rpy = _compose(base_inv_t, base_inv_r, world_t, world_r)
        pose = PoseStamped()
        pose.header.frame_id = self._base_frame
        pose.pose.position = Point(x=base_xyz[0], y=base_xyz[1], z=base_xyz[2])
        pose.pose.orientation = self._rpy_to_quaternion(base_pose_rpy)
        return pose

    def _add_pickup_box(self, scene, scene_config, box, source):
        self._remove_pickup_objects(scene)
        self._pickup_attached = False
        pose = self._world_pose_to_base_pose(box.pose, scene_config)
        scene.add_box(
            self._pickup_object_id,
            pose,
            size=(box.width, box.depth, box.height),
        )
        self._pickup_touch_enabled = False
        self._apply_pickup_touch()
        self._place_support_touch_enabled = False
        self._apply_place_support_touch()
        self._publish_pickup_box_marker(pose, box)
        self._log_box_pose_sanity(box, pose)
        rospy.loginfo(
            "Added current pickup box collision %s from %s (%s) "
            "size=%.2fx%.2fx%.2f at %s",
            self._pickup_object_id, box.id, source,
            box.width, box.depth, box.height,
            [
                round(pose.pose.position.x, 3),
                round(pose.pose.position.y, 3),
                round(pose.pose.position.z, 3),
            ],
        )
        return self._pickup_object_id

    def _add_current_pickup_box(self, scene, scene_config):
        try:
            rospy.wait_for_service(self._pickup_box_service, timeout=0.5)
            get_current = rospy.ServiceProxy(self._pickup_box_service, GetCurrentBox)
            resp = get_current()
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn("Current pickup box unavailable for scene sync: %s", exc)
            return None

        if not resp.success:
            return None
        return self._add_pickup_box(
            scene, scene_config, resp.box, "spawner_gt")

    # ── Allowed-collision-matrix (ACM) for suction_panel <-> pickup box ────

    def handle_attach_pickup_box(self, _req):
        """Attach the current pickup collision object to the suction panel."""
        try:
            scene = self._ensure_scene()
            if self._pickup_attached:
                return TriggerResponse(
                    success=True,
                    message="pickup box already attached to %s" % self._attach_link,
                )
            scene.attach_object(
                self._pickup_object_id,
                link=self._attach_link,
                touch_links=self._pickup_touch_links,
            )
            self._pickup_attached = True
            self._pickup_touch_enabled = True
            self._apply_pickup_touch(allowed=True)
            rospy.loginfo(
                "Attached %s to %s (touch_links=%s)",
                self._pickup_object_id,
                self._attach_link,
                ",".join(self._pickup_touch_links),
            )
            return TriggerResponse(
                success=True,
                message="attached %s to %s"
                % (self._pickup_object_id, self._attach_link),
            )
        except Exception as exc:
            rospy.logerr("SceneManager.attach_pickup_box failed: %s", exc)
            return TriggerResponse(success=False, message=str(exc))

    def handle_detach_pickup_box(self, _req):
        """Detach and remove the current pickup collision object."""
        try:
            scene = self._ensure_scene()
            if self._pickup_attached:
                scene.remove_attached_object(self._pickup_object_id)
                self._pickup_attached = False
            self._remove_pickup_objects(scene)
            self._pickup_touch_enabled = False
            self._apply_pickup_touch(allowed=False)
            self._place_support_touch_enabled = False
            self._apply_place_support_touch()
            rospy.loginfo("Detached and removed %s", self._pickup_object_id)
            return TriggerResponse(
                success=True,
                message="detached %s" % self._pickup_object_id,
            )
        except Exception as exc:
            rospy.logerr("SceneManager.detach_pickup_box failed: %s", exc)
            return TriggerResponse(success=False, message=str(exc))

    def handle_set_pickup_touch(self, req):
        """Toggle suction_panel<->current_pickup_box contact (called by the
        motion planner per pick segment)."""
        self._pickup_touch_enabled = bool(req.data)
        self._apply_pickup_touch()
        return SetBoolResponse(
            success=True,
            message="pickup touch %s"
            % ("ALLOWED" if self._pickup_touch_enabled else "enforced (collides)"),
        )

    def handle_set_place_support_touch(self, req):
        """Allow the carried box to touch committed support boxes on descend."""
        self._place_support_touch_enabled = bool(req.data)
        self._apply_place_support_touch()
        return SetBoolResponse(
            success=True,
            message="place support touch %s"
            % ("ALLOWED" if self._place_support_touch_enabled else "enforced"),
        )

    def _apply_place_support_touch(self):
        if not self._placed_ids:
            return
        acm = self._fetch_acm()
        if acm is None:
            rospy.logwarn("ACM: cannot update place support touch")
            return
        for placed_id in self._placed_ids:
            self._set_acm_pair(
                acm, self._pickup_object_id, placed_id,
                allowed=self._place_support_touch_enabled)
        ps = PlanningScene()
        ps.is_diff = True
        ps.name = ""
        ps.allowed_collision_matrix = acm
        self._planning_scene_pub.publish(ps)
        rospy.loginfo(
            "ACM: %s <-> %d placed supports = %s",
            self._pickup_object_id,
            len(self._placed_ids),
            "ALLOWED" if self._place_support_touch_enabled else "enforced",
        )

    def _apply_pickup_touch(self, allowed=None):
        """Set the suction_panel<->current_pickup_box ACM entry to `allowed`.

        When allowed is None, uses self._pickup_touch_enabled. The motion planner
        toggles this per pick segment: enforced (False) for pre_grasp so the
        panel reaches directly above the box without clipping it; relaxed
        (True) for approach/attach/retreat so the panel can descend to / lift
        off contact. Other links always collide with the box (the ACM relaxes
        only the panel).

        PlanningSceneInterface has no ACM setter in Noetic, so we fetch the
        current AllowedCollisionMatrix, set the symmetric touch-link<->box
        entry, and republish the full merged matrix as a PlanningScene diff
        (safe under both merge- and replace-style diff application). When
        enabling, a short verify poll re-reads the scene so the descent can rely
        on the entry being live before it runs.
        """
        if not self._pickup_touch_links:
            return
        if allowed is None:
            allowed = self._pickup_touch_enabled
        object_id = self._pickup_object_id
        try:
            rospy.wait_for_service("/get_planning_scene", timeout=2.0)
        except rospy.ROSException:
            rospy.logwarn("ACM: /get_planning_scene unavailable; touch unchanged")
            return
        acm = self._fetch_acm()
        if acm is None:
            rospy.logwarn("ACM: planning scene fetch failed; touch unchanged")
            return
        for link in self._pickup_touch_links:
            self._set_acm_pair(acm, object_id, link, allowed=allowed)
        ps = PlanningScene()
        ps.is_diff = True
        ps.name = ""
        ps.allowed_collision_matrix = acm
        self._planning_scene_pub.publish(ps)
        links = ",".join(self._pickup_touch_links)
        # Only report when the permission actually flips. Logging every call
        # produced a dozen identical lines per run with no state change in them;
        # a failed verify still warns because that one is not routine.
        changed = self._pickup_acm_allowed != allowed
        self._pickup_acm_allowed = allowed
        if allowed:
            if self._verify_acm_pair(object_id, self._pickup_touch_links, timeout=1.0):
                if changed:
                    rospy.logdebug("ACM: %s <-> %s = ALLOWED", object_id, links)
            else:
                rospy.logwarn(
                    "ACM: %s <-> %s = ALLOWED (verify timed out)", object_id, links
                )
        elif changed:
            rospy.logdebug(
                "ACM: %s <-> %s = enforced (panel collides)", object_id, links)

    def _fetch_acm(self):
        try:
            req = GetPlanningSceneRequest()
            req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            resp = self._get_planning_scene(req)
            return resp.scene.allowed_collision_matrix
        except rospy.ServiceException as exc:
            rospy.logwarn("ACM fetch failed: %s", exc)
            return None

    @staticmethod
    def _acm_index(acm, name):
        """Return the row index for `name`, appending a symmetric row/column
        (all False) if it is not already present."""
        if name in acm.entry_names:
            return acm.entry_names.index(name)
        idx = len(acm.entry_names)
        acm.entry_names.append(name)
        for entry in acm.entry_values:
            entry.enabled.append(False)
        acm.entry_values.append(AllowedCollisionEntry(enabled=[False] * (idx + 1)))
        return idx

    def _set_acm_pair(self, acm, a, b, allowed):
        ia = self._acm_index(acm, a)
        ib = self._acm_index(acm, b)
        width = len(acm.entry_names)
        for entry in acm.entry_values:
            while len(entry.enabled) < width:
                entry.enabled.append(False)
        acm.entry_values[ia].enabled[ib] = bool(allowed)
        acm.entry_values[ib].enabled[ia] = bool(allowed)

    @staticmethod
    def _acm_pair_allowed(acm, a, b):
        if a not in acm.entry_names or b not in acm.entry_names:
            return False
        ia = acm.entry_names.index(a)
        ib = acm.entry_names.index(b)
        try:
            return bool(acm.entry_values[ia].enabled[ib])
        except (IndexError, AttributeError):
            return False

    def _verify_acm_pair(self, object_id, links, timeout=2.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while rospy.Time.now() < deadline and not rospy.is_shutdown():
            acm = self._fetch_acm()
            if acm is not None and all(
                self._acm_pair_allowed(acm, object_id, link) for link in links
            ):
                return True
            rospy.sleep(0.1)
        return False

    # ── Pickup-box diagnostics ───────────────────────────────────────────

    def _publish_pickup_box_marker(self, pose_stamped, box):
        """Publish a wireframe of the pickup-box collision object (base frame)
        so RViz can confirm it overlaps the real Gazebo box at a glance."""
        w, d, h = float(box.width), float(box.depth), float(box.height)
        hx, hy, hz = w * 0.5, d * 0.5, h * 0.5
        corners = [
            (-hx, -hy, -hz), (hx, -hy, -hz), (hx, hy, -hz), (-hx, hy, -hz),
            (-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz),
        ]
        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        ]
        points = []
        for a, b in edges:
            ca, cb = corners[a], corners[b]
            points.append(Point(x=ca[0], y=ca[1], z=ca[2]))
            points.append(Point(x=cb[0], y=cb[1], z=cb[2]))
        m = Marker()
        m.header.frame_id = pose_stamped.header.frame_id
        m.header.stamp = rospy.Time.now()
        m.ns = "pickup_box_scene"
        m.id = 0
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.pose.position = Point(
            x=pose_stamped.pose.position.x,
            y=pose_stamped.pose.position.y,
            z=pose_stamped.pose.position.z,
        )
        m.pose.orientation = pose_stamped.pose.orientation
        m.scale.x = 0.03  # line width
        m.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.9)
        m.points = points
        m.lifetime = rospy.Duration(0)
        self._box_debug_pub.publish(m)

    def _log_box_pose_sanity(self, box, base_pose_stamped):
        """Cross-check the scene_tf-derived box pose (base frame) against the
        live world->elfin_base_link TF. Agreement <1 cm means the collision
        object matches the real Gazebo box; a mismatch points at a scene_tf
        config issue (not this code)."""
        world_pose = PoseStamped()
        world_pose.header.frame_id = "world"
        world_pose.header.stamp = rospy.Time(0)
        world_pose.pose = box.pose
        try:
            tf_base_pose = self._tf_buffer.transform(
                world_pose, self._base_frame, rospy.Duration(0.5)
            )
        except Exception as exc:
            rospy.logdebug("pickup box TF sanity skipped: %s", exc)
            return
        dp = base_pose_stamped.pose.position
        tp = tf_base_pose.pose.position
        dist = math.sqrt(
            (dp.x - tp.x) ** 2 + (dp.y - tp.y) ** 2 + (dp.z - tp.z) ** 2
        )
        rospy.loginfo(
            "pickup box pose base: scene_tf=%s tf=%s delta=%.4fm %s",
            [round(dp.x, 3), round(dp.y, 3), round(dp.z, 3)],
            [round(tp.x, 3), round(tp.y, 3), round(tp.z, 3)],
            dist,
            "OK" if dist < 0.01 else "MISMATCH-check scene_tf vs live TF",
        )

    def _resolve_scene_config_path(self):
        """Re-resolve the scene_tf path on each call.

        The layout atlas evaluator writes a per-slice effective scene_tf YAML and
        points scene_manager's PRIVATE ``~scene_tf_config`` param at it before
        calling ``sync_static_scene`` (it deliberately does NOT touch the global
        ``/luggage/scene_tf_config`` shared by other nodes). ``resolve_scene_tf_config_path``
        checks ``~scene_tf_config`` first, so re-resolving here makes collision-aware
        atlas slices actually place the container mesh at the shifted pose. Falls back
        to the cached path when no live param is set.
        """
        resolved = resolve_scene_tf_config_path()
        if resolved:
            return resolved
        return self._scene_tf_config

    def sync_pickup(self, _req):
        try:
            scene = self._ensure_scene()
            scene_config = load_scene_tf_config(self._resolve_scene_config_path())
            pickup_id = self._add_current_pickup_box(scene, scene_config)
            if pickup_id:
                return SyncStaticSceneResponse(
                    success=True,
                    message="pickup scene synced (%s)" % pickup_id,
                )
            return SyncStaticSceneResponse(
                success=True,
                message="no current pickup box to sync",
            )
        except Exception as exc:
            rospy.logerr("SceneManager.sync_pickup failed: %s", exc)
            return SyncStaticSceneResponse(success=False, message=str(exc))

    def sync_detected_pickup(self, req):
        """Sync pickup collision from the perception-owned pose."""
        try:
            box = req.box
            if (
                    box.width <= 0.0
                    or box.depth <= 0.0
                    or box.height <= 0.0):
                return SyncPickupBoxResponse(
                    success=False, message="detected box dimensions missing")
            scene = self._ensure_scene()
            scene_config = load_scene_tf_config(
                self._resolve_scene_config_path())
            pickup_id = self._add_pickup_box(
                scene, scene_config, box, "strict_rgbd")
            return SyncPickupBoxResponse(
                success=bool(pickup_id),
                message="synced detected pickup box %s" % pickup_id,
            )
        except Exception as exc:
            rospy.logerr("SceneManager.sync_detected_pickup failed: %s", exc)
            return SyncPickupBoxResponse(success=False, message=str(exc))

    def sync_static(self, _req):
        try:
            scene = self._ensure_scene()
            scene_config = load_scene_tf_config(self._resolve_scene_config_path())
            base_xyz, base_rpy = container_in_base_link(scene_config)

            self._remove_container_objects(scene)
            scene.remove_world_object(container_model_name(scene_config))
            added_container = self._add_container_mesh(
                scene, scene_config, base_xyz, base_rpy
            )

            scene.remove_world_object("robot_pedestal")
            if pedestal_enabled(scene_config):
                ped = pedestal_config(scene_config)
                ped_length, ped_width, ped_height = pedestal_dimensions(scene_config)
                ped_center = pedestal_center_in_base_link(scene_config)
                ped_pose = PoseStamped()
                ped_pose.header.frame_id = self._base_frame
                ped_pose.pose.position = Point(
                    x=ped_center[0],
                    y=ped_center[1],
                    z=ped_center[2],
                )
                ped_pose.pose.orientation = self._rpy_to_quaternion(ped["rotation_rpy"])
                scene.add_box(
                    "robot_pedestal",
                    ped_pose,
                    size=(ped_length, ped_width, ped_height),
                )
                rospy.loginfo(
                    "Added robot_pedestal collision box %.2fx%.2fx%.2f at %s",
                    ped_length,
                    ped_width,
                    ped_height,
                    [round(v, 3) for v in ped_center],
                )

            return SyncStaticSceneResponse(
                success=True,
                message="static scene synced (%s)" % added_container,
            )
        except Exception as exc:
            rospy.logerr("SceneManager.sync_static failed: %s", exc)
            return SyncStaticSceneResponse(success=False, message=str(exc))

    def add_placed(self, req):
        try:
            scene = self._ensure_scene()
            obj_id = self._placed_object_id(req.slot)
            pose = PoseStamped()
            pose.header.frame_id = self._base_frame
            pose.pose = req.slot.place_pose
            scene.add_box(
                obj_id,
                pose,
                size=(req.slot.width, req.slot.depth, req.slot.height),
            )
            self._placed_ids.append(obj_id)
            return AddPlacedBoxResponse(success=True, message="added %s" % obj_id)
        except Exception as exc:
            rospy.logerr("SceneManager.add_placed failed: %s", exc)
            return AddPlacedBoxResponse(success=False, message=str(exc))

    @staticmethod
    def _placed_object_id(slot):
        return "placed_%d_%d_%d" % (slot.layer, slot.row, slot.col)

    def remove_placed(self, req):
        try:
            obj_id = self._placed_object_id(req.slot)
            scene = self._ensure_scene()
            scene.remove_world_object(obj_id)
            self._placed_ids = [
                existing for existing in self._placed_ids
                if existing != obj_id]
            return RemovePlacedBoxResponse(
                success=True, message="removed %s" % obj_id)
        except Exception as exc:
            rospy.logerr("SceneManager.remove_placed failed: %s", exc)
            return RemovePlacedBoxResponse(success=False, message=str(exc))

    @staticmethod
    def _rpy_to_quaternion(rpy):
        import math

        roll, pitch, yaw = rpy
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

    @staticmethod
    def _quaternion_to_rpy(q):
        import math

        # Standard quaternion -> roll/pitch/yaw conversion.
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return [roll, pitch, yaw]



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
    rospy.init_node("scene_manager", log_level=resolve_log_level())
    mgr = SceneManager()
    rospy.Service("~sync_static_scene", SyncStaticScene, mgr.sync_static)
    rospy.Service("~sync_pickup_box", SyncStaticScene, mgr.sync_pickup)
    rospy.Service(
        "~sync_detected_pickup_box", SyncPickupBox,
        mgr.sync_detected_pickup)
    rospy.Service("~add_placed_box", AddPlacedBox, mgr.add_placed)
    rospy.Service("~remove_placed_box", RemovePlacedBox, mgr.remove_placed)
    rospy.Service("~set_pickup_touch", SetBool, mgr.handle_set_pickup_touch)
    rospy.Service(
        "~set_place_support_touch", SetBool,
        mgr.handle_set_place_support_touch)
    rospy.Service("~attach_pickup_box", Trigger, mgr.handle_attach_pickup_box)
    rospy.Service("~detach_pickup_box", Trigger, mgr.handle_detach_pickup_box)
    rospy.loginfo("scene_manager ready")
    rospy.spin()


if __name__ == "__main__":
    main()
