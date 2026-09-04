#!/usr/bin/env python3
"""Draft scene_tf helpers for pose_tune preview (isolated from production)."""

from __future__ import division

import copy
import os

import rospkg
import yaml

DRAFT_PARAM = "/luggage/pose_tune/scene_tf_draft"
JOINT_CHECK_PARAM = "/luggage/pose_tune/joint_values_for_check"


def default_scene_tf_path():
    return os.path.join(
        rospkg.RosPack().get_path("luggage_description"),
        "config",
        "scene_tf.yaml.example",
    )


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def save_yaml(path, data):
    with open(path, "w", encoding="utf-8") as handle:
        yaml.dump(data, handle, default_flow_style=False, sort_keys=False)


def resolve_production_scene_tf_path(rospy_module=None):
    if rospy_module is not None:
        try:
            return rospy_module.get_param("/luggage/scene_tf_config", default_scene_tf_path())
        except Exception:
            pass
    return default_scene_tf_path()


def init_draft_from_production(rospy_module):
    """Copy production yaml into draft param without modifying production."""
    path = resolve_production_scene_tf_path(rospy_module)
    draft = load_yaml(path)
    rospy_module.set_param(DRAFT_PARAM, draft)
    return copy.deepcopy(draft)


def get_draft(rospy_module):
    if not rospy_module.has_param(DRAFT_PARAM):
        return init_draft_from_production(rospy_module)
    return copy.deepcopy(rospy_module.get_param(DRAFT_PARAM))


def set_draft(rospy_module, draft):
    rospy_module.set_param(DRAFT_PARAM, draft)


def find_container_transform(draft):
    for item in draft.get("static_transforms", []):
        if item.get("child") == "container_link":
            return item
    return None


def ensure_container_transform(draft):
    item = find_container_transform(draft)
    if item is None:
        item = {
            "parent": draft.get("world_frame", "world"),
            "child": "container_link",
            "translation": [1.5, -0.8, 0.0],
            "rotation_rpy": [0.0, 0.0, 0.0],
        }
        draft.setdefault("static_transforms", []).append(item)
    item.setdefault("translation", [0.0, 0.0, 0.0])
    item.setdefault("rotation_rpy", [0.0, 0.0, 0.0])
    return item


def ensure_robot_rotation(draft):
    robot = draft.setdefault("robot", {})
    robot.setdefault("base_frame", "elfin_base_link")
    rpy = robot.setdefault("rotation_rpy", [0.0, 0.0, 0.0])
    while len(rpy) < 3:
        rpy.append(0.0)
    return rpy
