#!/usr/bin/env python3
"""Runtime spawner for the single fixed-source pickup box."""

from __future__ import division

import os
import random
import sys

import rospy
import rospkg
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion

from luggage_msgs.msg import DetectedLuggage
from luggage_msgs.srv import (
    ClearCurrentBox,
    ClearCurrentBoxResponse,
    GetCurrentBox,
    GetCurrentBoxResponse,
    SpawnNextBox,
    SpawnNextBoxResponse,
)

DESC_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_description"), "scripts")
if DESC_SCRIPTS not in sys.path:
    sys.path.insert(0, DESC_SCRIPTS)

from container_config_utils import (  # noqa: E402
    box_catalog_entries,
    default_box_catalog_path,
    default_config_path,
    load_box_catalog,
    load_container_config,
    pickup_source_in_world,
)


class PickupBoxSpawner:
    def __init__(self):
        self._container_config_path = rospy.get_param("~container_config", default_config_path())
        self._box_catalog_path = rospy.get_param("~box_catalog_config", default_box_catalog_path())
        self._model_prefix = rospy.get_param("~model_prefix", "pickup_box")
        self._world_frame = rospy.get_param("~world_frame", "world")
        self._seed = rospy.get_param("~random_seed", None)
        self._rng = random.Random(self._seed)
        self._current_box = None
        self._current_model = None
        self._sequence = 0

        container_config = load_container_config(self._container_config_path)
        self._catalog_config = load_box_catalog(self._box_catalog_path, container_config)
        self._source_xyz, self._source_rpy = pickup_source_in_world(
            container_config, self._catalog_config
        )
        self._entries = box_catalog_entries(self._catalog_config)
        if not self._entries:
            raise rospy.ROSException("box catalog is empty")

        gazebo_pkg = rospkg.RosPack().get_path("luggage_gazebo")
        self._model_paths = {
            entry["model"]: os.path.join(gazebo_pkg, "models", entry["model"], "model.sdf")
            for entry in self._entries
        }

        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        rospy.wait_for_service("/gazebo/delete_model")
        self._spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self._delete = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

    def _choose_entry(self):
        total = sum(max(0.0, entry["weight"]) for entry in self._entries)
        if total <= 0.0:
            return self._rng.choice(self._entries)
        pick = self._rng.uniform(0.0, total)
        running = 0.0
        for entry in self._entries:
            running += max(0.0, entry["weight"])
            if pick <= running:
                return entry
        return self._entries[-1]

    @staticmethod
    def _quaternion_from_rpy(roll, pitch, yaw):
        import math

        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        return Quaternion(
            w=cr * cp * cy + sr * sp * sy,
            x=sr * cp * cy - cr * sp * sy,
            y=cr * sp * cy + sr * cp * sy,
            z=cr * cp * sy - sr * sp * cy,
        )

    def _entry_pose(self, entry):
        yaw = self._source_rpy[2]
        allowed = entry.get("allowed_yaws") or [0.0]
        yaw += self._rng.choice(allowed)
        size = entry["size"]
        return Pose(
            position=Point(
                x=self._source_xyz[0],
                y=self._source_xyz[1],
                z=self._source_xyz[2] + size[2] * 0.5,
            ),
            orientation=self._quaternion_from_rpy(self._source_rpy[0], self._source_rpy[1], yaw),
        )

    @staticmethod
    def _box_to_param(box):
        return {
            "id": box.id,
            "width": box.width,
            "depth": box.depth,
            "height": box.height,
            "pose": {
                "position": {
                    "x": box.pose.position.x,
                    "y": box.pose.position.y,
                    "z": box.pose.position.z,
                },
                "orientation": {
                    "x": box.pose.orientation.x,
                    "y": box.pose.orientation.y,
                    "z": box.pose.orientation.z,
                    "w": box.pose.orientation.w,
                },
            },
        }

    def _read_sdf(self, model):
        path = self._model_paths.get(model)
        if not path or not os.path.exists(path):
            raise IOError("missing SDF for model %s" % model)
        with open(path, "r") as handle:
            return handle.read()

    def handle_clear(self, _req):
        if not self._current_model:
            rospy.delete_param("/luggage/current_box") if rospy.has_param("/luggage/current_box") else None
            self._current_box = None
            return ClearCurrentBoxResponse(success=True, message="no current pickup box")
        resp = self._delete(self._current_model)
        if not resp.success:
            return ClearCurrentBoxResponse(success=False, message=resp.status_message)
        rospy.loginfo("Deleted current pickup box '%s'", self._current_model)
        self._current_model = None
        self._current_box = None
        if rospy.has_param("/luggage/current_box"):
            rospy.delete_param("/luggage/current_box")
        return ClearCurrentBoxResponse(success=True, message="cleared current pickup box")

    def handle_get_current(self, _req):
        if self._current_box is None:
            return GetCurrentBoxResponse(
                box=DetectedLuggage(), success=False, message="no current pickup box"
            )
        return GetCurrentBoxResponse(
            box=self._current_box, success=True, message="current pickup box"
        )

    def handle_spawn_next(self, _req):
        clear = self.handle_clear(None)
        if not clear.success:
            return SpawnNextBoxResponse(box=DetectedLuggage(), success=False, message=clear.message)

        entry = self._choose_entry()
        self._sequence += 1
        model_name = "%s_%04d_%s" % (self._model_prefix, self._sequence, entry["id"])
        pose = self._entry_pose(entry)
        try:
            resp = self._spawn(
                model_name,
                self._read_sdf(entry["model"]),
                "",
                pose,
                self._world_frame,
            )
        except Exception as exc:
            return SpawnNextBoxResponse(box=DetectedLuggage(), success=False, message=str(exc))

        if not resp.success:
            return SpawnNextBoxResponse(
                box=DetectedLuggage(), success=False, message=resp.status_message
            )

        size = entry["size"]
        box = DetectedLuggage(
            id=model_name,
            pose=pose,
            width=size[0],
            depth=size[1],
            height=size[2],
        )
        self._current_model = model_name
        self._current_box = box
        rospy.set_param("/luggage/current_box", self._box_to_param(box))
        rospy.loginfo(
            "Spawned %s at pickup source size=%.2fx%.2fx%.2f",
            model_name,
            box.width,
            box.depth,
            box.height,
        )
        return SpawnNextBoxResponse(box=box, success=True, message="spawned %s" % model_name)


def main():
    rospy.init_node("pickup_box_spawner")
    spawner = PickupBoxSpawner()
    rospy.Service("~spawn_next_box", SpawnNextBox, spawner.handle_spawn_next)
    rospy.Service("~clear_current_box", ClearCurrentBox, spawner.handle_clear)
    rospy.Service("~get_current_box", GetCurrentBox, spawner.handle_get_current)
    rospy.loginfo("pickup_box_spawner ready")
    rospy.spin()


if __name__ == "__main__":
    main()
