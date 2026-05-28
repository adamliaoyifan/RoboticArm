#!/usr/bin/env python3
"""Sync static container collision objects into MoveIt planning scene."""

import os
import sys

import rospy
import rospkg
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from moveit_commander import PlanningSceneInterface

from luggage_msgs.srv import SyncStaticScene, SyncStaticSceneResponse
from luggage_msgs.srv import AddPlacedBox, AddPlacedBoxResponse

DESC_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_description"), "scripts")
if DESC_SCRIPTS not in sys.path:
    sys.path.insert(0, DESC_SCRIPTS)

from container_config_utils import (  # noqa: E402
    container_in_base_link,
    default_config_path,
    load_container_config,
    outer_box_center_in_container,
    outer_dimensions,
)


class SceneManager:
    def __init__(self):
        self._container_config = rospy.get_param("~container_config", default_config_path())
        self._base_frame = rospy.get_param("~base_frame", "elfin_base_link")
        self._scene = None
        self._placed_ids = []

    def _ensure_scene(self):
        if self._scene is None:
            moveit_commander.roscpp_initialize([])
            self._scene = PlanningSceneInterface(synchronous=True)
        return self._scene

    def sync_static(self, _req):
        try:
            scene = self._ensure_scene()
            config = load_container_config(self._container_config)
            length, width, height = outer_dimensions(config)
            center_local = outer_box_center_in_container(config)
            base_xyz, base_rpy = container_in_base_link(config)

            rot = self._rpy_to_quaternion(base_rpy)
            local_center = [
                base_xyz[0] + self._rotate_vec(rot, center_local)[0],
                base_xyz[1] + self._rotate_vec(rot, center_local)[1],
                base_xyz[2] + self._rotate_vec(rot, center_local)[2],
            ]

            pose = PoseStamped()
            pose.header.frame_id = self._base_frame
            pose.pose.position = Point(x=local_center[0], y=local_center[1], z=local_center[2])
            pose.pose.orientation = rot

            scene.remove_world_object("airport_container")
            scene.add_box("airport_container", pose, size=(length, width, height))
            rospy.loginfo(
                "Added airport_container collision box %.2fx%.2fx%.2f at %s",
                length,
                width,
                height,
                [round(v, 3) for v in local_center],
            )
            return SyncStaticSceneResponse(success=True, message="container synced")
        except Exception as exc:
            rospy.logerr("SceneManager.sync_static failed: %s", exc)
            return SyncStaticSceneResponse(success=False, message=str(exc))

    def add_placed(self, req):
        try:
            scene = self._ensure_scene()
            obj_id = "placed_%d_%d_%d" % (req.slot.layer, req.slot.row, req.slot.col)
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
    def _rotate_vec(q, vec):
        import math

        # Quaternion rotate v by q.
        x, y, z = vec
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        ix = qw * x + qy * z - qz * y
        iy = qw * y + qz * x - qx * z
        iz = qw * z + qx * y - qy * x
        iw = -qx * x - qy * y - qz * z
        rx = ix * qw + iw * -qx + iy * -qz - iz * -qy
        ry = iy * qw + iw * -qy + iz * -qx - ix * -qz
        rz = iz * qw + iw * -qz + ix * -qy - iy * -qx
        return [rx, ry, rz]


def main():
    rospy.init_node("scene_manager")
    mgr = SceneManager()
    rospy.Service("~sync_static_scene", SyncStaticScene, mgr.sync_static)
    rospy.Service("~add_placed_box", AddPlacedBox, mgr.add_placed)
    rospy.loginfo("scene_manager ready")
    rospy.spin()


if __name__ == "__main__":
    main()
