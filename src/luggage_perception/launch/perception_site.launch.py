"""实机代码 (site / real-cell). Not part of the Gazebo simulation stack.

Orin (or any perception host) owns Mid-360, D555, preprocessor, YOLO-World,
semantic filter, and DetectLuggage. Topic names are unchanged so the
laptop planning graph can subscribe the same names over DDS:

  /camera/d555/color/image_raw/compressed
  /camera/d555/color/camera_info
  /camera/d555/aligned_depth_to_color/image_raw/compressed
  /camera/d555/aligned_depth_to_color/camera_info
  /livox/lidar
  /livox/imu
  /luggage/preprocessed/camera/color/image
  /luggage/preprocessed/camera/color/camera_info
  /luggage/preprocessed/camera/depth/image
  /luggage/preprocessed/camera/depth/camera_info
  /luggage/preprocessed/status
  /luggage/semantic/mask
  /luggage/semantic/overlay
  /luggage/semantic/instance_mask
  /luggage/semantic/yolo_detections
  /luggage/semantic/cargo_points
  /luggage/semantic/obstacle_points
  /luggage_detector/detect_luggage

  ros2 launch luggage_perception perception_site.launch.py
  # or: src/luggage_perception/scripts/perception_site.sh

Do not start this graph on the same machine as hardware_pick.sh with
start_d555/start_perception left true (duplicate D555 / YOLO).
Do not start jazzy_real / the CPS executor node here (one CPS owner).
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _bool_text(value):
    return str(value).strip().lower() in ("true", "1", "yes", "on")


def _first_existing(paths):
    for path in paths:
        if path and os.path.isfile(path):
            return path
    return paths[-1] if paths else ""


def _default_mid360():
    env = os.environ.get("MID360_CONFIG", "")
    candidates = [env]
    for prefix in os.environ.get("COLCON_PREFIX_PATH", "").split(os.pathsep):
        if not prefix:
            continue
        ws = os.path.abspath(os.path.join(prefix, ".."))
        candidates.append(os.path.join(ws, "config", "MID360s_config.json"))
        candidates.append(
            os.path.join(ws, "deployment_ws", "config", "MID360s_config.json")
        )
    try:
        share = get_package_share_directory("luggage_description")
        candidates.append(os.path.join(share, "config", "MID360s_config.json"))
        candidates.append(
            os.path.join(share, "config", "MID360s_config.json.example")
        )
    except Exception:
        pass
    home = os.path.expanduser("~")
    candidates.append(os.path.join(home, "ros2_ws", "config", "MID360s_config.json"))
    return _first_existing(candidates)


def _default_scene_tf(desc_share):
    env = os.environ.get("SCENE_TF_CONFIG", "")
    site = os.path.join(desc_share, "config", "scene_tf.yaml")
    example = os.path.join(desc_share, "config", "scene_tf.yaml.example")
    return _first_existing([env, site, example])


def _optional_share(package_name):
    try:
        return get_package_share_directory(package_name)
    except Exception:
        return ""


def _start_d555(context, *args, **kwargs):
    if not _bool_text(LaunchConfiguration("start_d555").perform(context)):
        return [LogInfo(msg="[perception_site] D555 off (start_d555:=false)")]
    exec_share = _optional_share("elfin_trajectory_executor")
    if not exec_share:
        raise RuntimeError(
            "start_d555:=true needs elfin_trajectory_executor "
            "(d555_rgbd.launch.py + d555_host_stamp). Copy "
            "deployment_ws/src/elfin_trajectory_executor into this workspace "
            "and colcon build it. Do not run trajectory_executor on Orin."
        )
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(exec_share, "launch", "d555_rgbd.launch.py")
            ),
            launch_arguments={
                "color_profile": LaunchConfiguration("color_profile").perform(context),
                "depth_profile": LaunchConfiguration("depth_profile").perform(context),
            }.items(),
        )
    ]


def _start_mid360(context, *args, **kwargs):
    if not _bool_text(LaunchConfiguration("start_mid360").perform(context)):
        return [LogInfo(msg="[perception_site] Mid-360 off (start_mid360:=false)")]
    desc_share = get_package_share_directory("luggage_description")
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(desc_share, "launch", "mid360.launch.py")
            ),
            launch_arguments={
                "user_config_path": LaunchConfiguration("user_config_path").perform(context),
                "frame_id": "livox_frame",
                "xfer_format": "0",
            }.items(),
        )
    ]


def generate_launch_description():
    desc_share = get_package_share_directory("luggage_description")
    perc_share = get_package_share_directory("luggage_perception")
    default_scene = _default_scene_tf(desc_share)
    live_pp = os.path.join(perc_share, "config", "preprocessor_d555_live.yaml")
    semantic = os.path.join(perc_share, "config", "semantic_segmenter.yaml")
    site_pp = os.path.join(perc_share, "config", "preprocessor_d555_site.yaml")
    start_perception = IfCondition(LaunchConfiguration("start_perception"))

    return LaunchDescription(
        [
            DeclareLaunchArgument("scene_tf_config", default_value=default_scene),
            DeclareLaunchArgument(
                "user_config_path",
                default_value=_default_mid360(),
                description="Livox JSON. Copy site IPs; keep the path on one line.",
            ),
            DeclareLaunchArgument(
                "start_perception",
                default_value="true",
                description=(
                    "Preprocessor, YOLO-World segmenter, semantic point "
                    "filter, luggage detector (same topics as hardware_pick)."
                ),
            ),
            DeclareLaunchArgument("start_d555", default_value="true"),
            DeclareLaunchArgument("start_mid360", default_value="true"),
            DeclareLaunchArgument(
                "preprocessor_config",
                default_value=site_pp,
                description=(
                    "Site preprocessor B is preprocessor_d555_site.yaml. "
                    "Profile A is preprocessor_config:=%s. Do not use "
                    "preprocessor_d555_replay.yaml on live cameras." % live_pp
                ),
            ),
            DeclareLaunchArgument("publish_overlay", default_value="true"),
            DeclareLaunchArgument("semantic_device", default_value="cuda"),
            DeclareLaunchArgument(
                "max_rate_hz",
                default_value="5.0",
                description="YOLO cap. Orin cuda uses 5; laptop CPU often 2.",
            ),
            DeclareLaunchArgument(
                "color_profile",
                default_value="640,360,15",
                description="D555 RGB profile. Do not use 896x504@30.",
            ),
            DeclareLaunchArgument(
                "depth_profile",
                default_value="640,360,15",
                description="D555 depth profile. Match colour.",
            ),
            LogInfo(
                msg=(
                    "[perception_site] Mid-360 + D555 + YOLO on this host. "
                    "Same topic names as hardware_pick. Laptop subscribes "
                    "/camera/d555/... /livox/lidar /luggage/preprocessed/... "
                    "/luggage/semantic/yolo_detections /luggage/semantic/overlay "
                    "and /luggage_detector/detect_luggage. No CPS here."
                )
            ),
            OpaqueFunction(function=_start_d555),
            OpaqueFunction(function=_start_mid360),
            Node(
                package="luggage_perception",
                executable="sensor_preprocessor_node.py",
                name="sensor_preprocessor",
                output="screen",
                parameters=[
                    os.path.join(perc_share, "config", "sensor_preprocessor.yaml"),
                    LaunchConfiguration("preprocessor_config"),
                    {"use_sim_time": False},
                ],
                condition=start_perception,
            ),
            Node(
                package="luggage_perception",
                executable="semantic_segmenter_node.py",
                name="semantic_segmenter",
                output="screen",
                parameters=[
                    semantic,
                    {
                        "use_sim_time": False,
                        "device": LaunchConfiguration("semantic_device"),
                        "self_body_camera_frame": "d555_color_optical_frame",
                        "require_backend": "yolo_world",
                        "workspace_accept_enabled": False,
                        "publish_overlay": ParameterValue(
                            LaunchConfiguration("publish_overlay"),
                            value_type=bool,
                        ),
                        "max_rate_hz": ParameterValue(
                            LaunchConfiguration("max_rate_hz"),
                            value_type=float,
                        ),
                    },
                ],
                condition=start_perception,
            ),
            Node(
                package="luggage_perception",
                executable="semantic_point_filter_node.py",
                name="semantic_point_filter",
                output="screen",
                parameters=[semantic, {"use_sim_time": False}],
                condition=start_perception,
            ),
            Node(
                package="luggage_perception",
                executable="luggage_detector_node.py",
                name="luggage_detector",
                output="screen",
                parameters=[{
                    "scene_tf_config": LaunchConfiguration("scene_tf_config"),
                    "use_semantic": True,
                    "use_sim_time": False,
                    "depth_topic": "/luggage/preprocessed/camera/depth/image",
                    "cloud_max_age_sec": 30.0,
                    "estimate_retry_count": 6,
                    "estimate_retry_period_sec": 2.0,
                    "support_mode": "auto",
                    "platform_z": "",
                    "suitcase_update_timeout_sec": 0.0,
                    "crop_to_workspace": False,
                }],
                condition=start_perception,
            ),
        ]
    )
