#!/usr/bin/env python3
"""Publish scene-derived parameters that other nodes would otherwise hard-code.

The task cloud filter needs the container OBB and the opening corridor in
base_link. Those used to be transcribed into active_loading.launch as literal
numbers, which had two problems: the copy drifts whenever scene_tf changes, and
roslaunch types `value="[0.0, -1.5, 0.145]"` as a *string*, so the C++ node's
`nh_.param(name, std::vector<double>, default)` silently fell back to its
defaults -- a 1 m cube at the robot origin rather than the real container.

Deriving them here from the single scene_tf source fixes both: the values track
the scene, and they are set as real numeric lists.
"""

import os
import sys

import rospy
import rospkg

_SCRIPTS = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from scene_tf_config_utils import (  # noqa: E402
    container_in_base_link,
    container_opening_dimensions,
    container_opening_normal_in_base_link,
    container_opening_target_point,
    container_usable_center_in_base_link,
    container_usable_dimensions,
    load_scene_tf_config,
    resolve_scene_tf_config_path,
)


def publish_task_roi_params(scene_config, namespace):
    """Set the task-ROI geometry for ``namespace`` and return what was set."""
    _base_xyz, base_rpy = container_in_base_link(scene_config)
    aperture_width, aperture_height = container_opening_dimensions(scene_config)
    values = {
        "container_center": [
            float(v) for v in
            container_usable_center_in_base_link(scene_config)],
        "container_dims": [
            float(v) for v in container_usable_dimensions(scene_config)],
        "container_yaw": float(base_rpy[2]),
        "opening_center": [
            float(v) for v in container_opening_target_point(scene_config)],
        "opening_normal": [
            float(v) for v in
            container_opening_normal_in_base_link(scene_config)],
        "aperture_width": float(aperture_width),
        "aperture_height": float(aperture_height),
    }
    for key, value in values.items():
        rospy.set_param("%s/%s" % (namespace.rstrip("/"), key), value)
    return values


def main():
    rospy.init_node("description_params", log_level=resolve_log_level())
    scene_path = rospy.get_param(
        "~scene_tf_config",
        rospy.get_param(
            "/luggage/scene_tf_config", resolve_scene_tf_config_path()))
    scene_config = load_scene_tf_config(scene_path)
    namespace = rospy.get_param("~task_roi_namespace", "/task_cloud_filter")
    values = publish_task_roi_params(scene_config, namespace)
    rospy.logwarn(
        "task ROI derived from %s -> %s: center=%s dims=%s yaw=%.4f "
        "opening=%s aperture=%.3fx%.3f",
        os.path.basename(scene_path), namespace,
        [round(v, 4) for v in values["container_center"]],
        [round(v, 4) for v in values["container_dims"]],
        values["container_yaw"],
        [round(v, 4) for v in values["opening_center"]],
        values["aperture_width"], values["aperture_height"],
    )
    rospy.spin()


# Log level must be chosen before init_node, so it cannot come from a private
# param; log_level_utils reads the LUGGAGE_LOG_LEVEL environment variable.
from log_level_utils import resolve_log_level  # noqa: E402

if __name__ == "__main__":
    main()
