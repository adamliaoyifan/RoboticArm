#!/usr/bin/env python3
"""Sync static container / pedestal collision objects into MoveIt.

Humble port of the ROS 1 scene_manager. Pickup-box attach/detach stays on
PlanningSceneClient inside vacuum_controller; this node only owns:

- container collision mesh
- pedestal AABB
- placed-box collision objects
- pickup_box <-> (container + placed_*) ACM for descend contact
"""

from __future__ import division

import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import SetBool

from luggage_description.scene_mesh_utils import (
    container_collision_mesh_path,
    container_model_name,
    require_existing_mesh,
)
from luggage_description.scene_tf_config_utils import (
    load_scene_tf_config,
    origin_in_world,
    pedestal_collision_pose_in_world,
    pedestal_dimensions,
    pedestal_enabled,
    resolve_scene_tf_config_path,
)
from luggage_description.scene_tf_publisher import rpy_to_quaternion
from luggage_msgs.srv import AddPlacedBox, RemovePlacedBox, SyncStaticScene
from luggage_planning.planning_scene_client import (
    BOX_OBJECT_ID,
    PlanningSceneClient,
    load_stl_mesh_msg,
)


def placed_object_id(slot):
    return "placed_%d_%d_%d" % (int(slot.layer), int(slot.row), int(slot.col))


class SceneManagerNode(Node):

    def __init__(self):
        super().__init__("scene_manager")
        group = ReentrantCallbackGroup()

        self.declare_parameter("scene_tf_config", "")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("pickup_object_id", BOX_OBJECT_ID)
        self.declare_parameter("container_collision_mesh", "")
        self.declare_parameter("mesh_max_faces", 0)
        self.declare_parameter("auto_sync", True)

        self._world_frame = str(self.get_parameter("world_frame").value)
        self._pickup_id = str(self.get_parameter("pickup_object_id").value)
        self._placed_ids = []
        self._container_id = ""
        self._place_support_touch = False
        self._synced = False
        self._sync_lock = threading.Lock()

        self._scene = PlanningSceneClient(self, callback_group=group)
        self.create_service(
            SyncStaticScene, "~/sync_static_scene",
            self._handle_sync_static, callback_group=group)
        self.create_service(
            AddPlacedBox, "~/add_placed_box",
            self._handle_add_placed, callback_group=group)
        self.create_service(
            RemovePlacedBox, "~/remove_placed_box",
            self._handle_remove_placed, callback_group=group)
        self.create_service(
            SetBool, "~/set_place_support_touch",
            self._handle_place_support, callback_group=group)

        if bool(self.get_parameter("auto_sync").value):
            self._sync_timer = self.create_timer(
                1.0, self._auto_sync_tick, callback_group=group)
        else:
            self._sync_timer = None
        self.get_logger().info("scene_manager ready")

    def _config(self):
        path = str(self.get_parameter("scene_tf_config").value or "")
        return load_scene_tf_config(resolve_scene_tf_config_path(path or None))

    def _auto_sync_tick(self):
        if self._synced:
            return
        if not self._scene.wait_ready(timeout_sec=0.2):
            return
        ok, message = self._sync_static()
        if ok:
            self._synced = True
            if self._sync_timer is not None:
                self._sync_timer.cancel()
            self.get_logger().info("auto sync_static_scene: %s" % message)
        else:
            self.get_logger().warn(
                "auto sync_static_scene waiting: %s" % message)

    def _handle_sync_static(self, _request, response):
        ok, message = self._sync_static()
        response.success = ok
        response.message = message
        if ok:
            self._synced = True
        return response

    def _sync_static(self):
        with self._sync_lock:
            try:
                config = self._config()
            except Exception as exc:  # noqa: BLE001
                return False, "scene_tf load failed: %s" % exc
            mesh_override = str(
                self.get_parameter("container_collision_mesh").value or "")
            try:
                mesh_path = require_existing_mesh(
                    mesh_override or container_collision_mesh_path(config))
            except Exception as exc:  # noqa: BLE001
                return False, "container mesh: %s" % exc
            obj_id = container_model_name(config)
            xyz, rpy = origin_in_world(config)
            quat = rpy_to_quaternion(rpy)
            try:
                mesh_msg = load_stl_mesh_msg(
                    mesh_path,
                    max_faces=int(self.get_parameter("mesh_max_faces").value))
            except Exception as exc:  # noqa: BLE001
                return False, "stl load failed: %s" % exc
            self._scene.remove_object(obj_id)
            ok, message = self._scene.add_collision_mesh(
                obj_id, xyz, quat, mesh_msg, frame_id=self._world_frame)
            if not ok:
                return False, "container mesh apply: %s" % message
            self._container_id = obj_id

            self._scene.remove_object("robot_pedestal")
            if pedestal_enabled(config):
                center, ped_rpy = pedestal_collision_pose_in_world(config)
                length, width, height = pedestal_dimensions(config)
                ok, message = self._scene.add_collision_box(
                    "robot_pedestal", center, rpy_to_quaternion(ped_rpy),
                    (length, width, height), frame_id=self._world_frame)
                if not ok:
                    return False, "pedestal apply: %s" % message
            return True, "static scene synced (%s, %d triangles)" % (
                obj_id, len(mesh_msg.triangles))

    def _handle_add_placed(self, request, response):
        slot = request.slot
        obj_id = placed_object_id(slot)
        xyz = [
            slot.place_pose.position.x,
            slot.place_pose.position.y,
            slot.place_pose.position.z,
        ]
        quat = [
            slot.place_pose.orientation.x,
            slot.place_pose.orientation.y,
            slot.place_pose.orientation.z,
            slot.place_pose.orientation.w,
        ]
        size = [slot.width, slot.depth, slot.height]
        ok, message = self._scene.add_collision_box(
            obj_id, xyz, quat, size, frame_id=self._world_frame)
        if ok and obj_id not in self._placed_ids:
            self._placed_ids.append(obj_id)
        response.success = ok
        response.message = message if not ok else "added %s" % obj_id
        return response

    def _handle_remove_placed(self, request, response):
        obj_id = placed_object_id(request.slot)
        ok, message = self._scene.remove_object(obj_id)
        self._placed_ids = [
            existing for existing in self._placed_ids if existing != obj_id]
        response.success = ok
        response.message = message if not ok else "removed %s" % obj_id
        return response

    def _support_pairs(self):
        pairs = []
        if self._container_id:
            pairs.append((self._pickup_id, self._container_id))
        for placed_id in self._placed_ids:
            pairs.append((self._pickup_id, placed_id))
        return pairs

    def _handle_place_support(self, request, response):
        allowed = bool(request.data)
        self._place_support_touch = allowed
        pairs = self._support_pairs()
        if not pairs:
            response.success = True
            response.message = "place support touch %s (no pairs)" % (
                "ALLOWED" if allowed else "enforced")
            return response
        ok, message = self._scene.set_acm_pairs(pairs, allowed, verify=True)
        response.success = ok
        response.message = (
            "place support touch %s (%s)"
            % ("ALLOWED" if allowed else "enforced", message))
        return response


def main(argv=None):
    rclpy.init(args=argv)
    node = SceneManagerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
