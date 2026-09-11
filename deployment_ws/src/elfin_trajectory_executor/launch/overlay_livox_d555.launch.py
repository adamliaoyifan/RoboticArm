"""Overlay Mid-360S and D555 clouds in RViz on the real cell.

Fixed Frame is elfin_base_link: CPS /joint_states drive the arm, RSP
publishes world→elfin_base_link→flange→livox/camera TF, and
scene_assets draws the container. Clouds stay in their sensor frames;
RViz looks up TF into the Fixed Frame.

Stop other D555 wrappers first (one realsense2_camera_node per camera).
Same ROS_DOMAIN_ID everywhere. Do not set ROS_LOCALHOST_ONLY.
enp0s31f6 MTU 9000. Livox JSON host_ip (192.168.1.5) must be on that NIC.

  source /opt/ros/jazzy/setup.bash
  source /home/adamliao/work/RoboticArm/deployment_ws/livox_ws/env.sh
  source /home/adamliao/work/RoboticArm/elfin_humble_ws/install/setup.bash
  source /home/adamliao/work/RoboticArm/deployment_ws/install/setup.bash
  unset ROS_LOCALHOST_ONLY
  export ROS_DOMAIN_ID=7
  ros2 launch elfin_trajectory_executor overlay_livox_d555.launch.py
"""

from __future__ import annotations

import json
import os
import subprocess

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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _share(package_name: str, hint: str) -> str:
    try:
        return get_package_share_directory(package_name)
    except Exception as exc:
        raise RuntimeError(
            "%s is not on AMENT_PREFIX_PATH. %s (%s)"
            % (package_name, hint, exc)
        ) from exc


def _first_existing(paths):
    for path in paths:
        if path and os.path.isfile(path):
            return path
    return paths[0] if paths else ""


def _default_mid360():
    env = os.environ.get("MID360_CONFIG", "")
    candidates = [env]
    for prefix in os.environ.get("COLCON_PREFIX_PATH", "").split(os.pathsep):
        if not prefix:
            continue
        ws = os.path.abspath(os.path.join(prefix, ".."))
        candidates.append(os.path.join(ws, "config", "MID360s_config.json"))
    try:
        share = get_package_share_directory("luggage_description")
        candidates.append(os.path.join(share, "config", "MID360s_config.json.example"))
    except Exception:
        pass
    candidates.append(
        "/home/adamliao/work/RoboticArm/deployment_ws/config/MID360s_config.json"
    )
    return _first_existing(candidates)


def _default_scene_tf():
    env = os.environ.get("SCENE_TF_CONFIG", "")
    candidates = [env]
    try:
        share = get_package_share_directory("luggage_description")
        candidates.append(os.path.join(share, "config", "scene_tf.yaml"))
        candidates.append(os.path.join(share, "config", "scene_tf.yaml.example"))
    except Exception:
        pass
    candidates.append(
        "/home/adamliao/work/RoboticArm/elfin_humble_ws/src/"
        "luggage_description/config/scene_tf.yaml"
    )
    return _first_existing(candidates)


def _truthy(value: str) -> bool:
    return value.strip().lower() in ("true", "1", "yes", "on")


def _iface_ipv4():
    try:
        out = subprocess.check_output(
            ["ip", "-4", "-o", "addr", "show"], text=True
        )
    except (OSError, subprocess.CalledProcessError):
        return []
    addrs = []
    for line in out.splitlines():
        parts = line.split()
        if "inet" in parts:
            idx = parts.index("inet")
            if idx + 1 < len(parts):
                addrs.append(parts[idx + 1].split("/", 1)[0])
    return addrs


def _livox_host_ip(config_path: str) -> str:
    try:
        with open(config_path, encoding="utf-8") as handle:
            data = json.load(handle)
        hosts = data.get("Mid360s", {}).get("host_net_info", [])
        if hosts:
            return str(hosts[0].get("host_ip", "")).strip()
    except (OSError, json.JSONDecodeError, TypeError):
        return ""
    return ""


def _warn_livox_host(context, *args, **kwargs):
    config = LaunchConfiguration("user_config_path").perform(context)
    host = _livox_host_ip(config)
    if not host:
        return [
            LogInfo(
                msg="[overlay] Livox JSON has no host_ip: %s" % config
            )
        ]
    if host in _iface_ipv4():
        return [
            LogInfo(msg="[overlay] Livox host_ip %s is on this PC" % host)
        ]
    return [
        LogInfo(
            msg=(
                "[overlay] Livox host_ip %s is not on this PC. "
                "Add it to enp0s31f6 (keep MTU 9000) or /livox/lidar "
                "will stay empty. JSON=%s" % (host, config)
            )
        )
    ]


def _cps(context, *args, **kwargs):
    if not _truthy(LaunchConfiguration("start_cps").perform(context)):
        return [
            LogInfo(
                msg=(
                    "[overlay] CPS off. Relative clouds still work in "
                    "Fixed Frame eef_mount_adapter."
                )
            )
        ]
    ip = LaunchConfiguration("robot_ip").perform(context)
    port = LaunchConfiguration("robot_port").perform(context)
    rate = LaunchConfiguration("rate_hz").perform(context)
    return [
        LogInfo(
            msg="[overlay] CPS monitor %s:%s (no servo enable)" % (ip, port)
        ),
        Node(
            package="elfin_trajectory_executor",
            executable="cps_telemetry",
            name="cps_telemetry",
            output="screen",
            parameters=[
                {
                    "use_sim_time": False,
                    "robot_ip": ip,
                    "robot_port": int(port),
                    "rate_hz": float(rate),
                }
            ],
        ),
    ]


def generate_launch_description():
    luggage = _share(
        "luggage_description",
        "source elfin_humble_ws/install/setup.bash after livox_ws/env.sh",
    )
    executor = _share(
        "elfin_trajectory_executor",
        "source deployment_ws/install/setup.bash",
    )
    scene_launch = os.path.join(luggage, "launch", "scene_hardware.launch.py")
    mid360_launch = os.path.join(luggage, "launch", "mid360.launch.py")
    d555_launch = os.path.join(executor, "launch", "d555_calib.launch.py")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "user_config_path",
                default_value=_default_mid360(),
                description="Livox JSON. Keep this path on one line.",
            ),
            DeclareLaunchArgument(
                "scene_tf_config",
                default_value=_default_scene_tf(),
                description="Measured scene_tf.yaml.",
            ),
            DeclareLaunchArgument(
                "serial_no",
                default_value="419222302385",
            ),
            DeclareLaunchArgument("camera_namespace", default_value="camera"),
            DeclareLaunchArgument("camera_name", default_value="d555"),
            DeclareLaunchArgument(
                "robot_ip",
                default_value=os.environ.get("ROBOT_IP", "192.168.0.10"),
            ),
            DeclareLaunchArgument(
                "robot_port",
                default_value=os.environ.get("ROBOT_PORT", "10003"),
            ),
            DeclareLaunchArgument("rate_hz", default_value="50.0"),
            DeclareLaunchArgument("start_scene", default_value="true"),
            DeclareLaunchArgument("start_cps", default_value="true"),
            DeclareLaunchArgument("start_mid360", default_value="true"),
            DeclareLaunchArgument("start_d555", default_value="true"),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description=(
                    "This overlay's RViz. Do not name it use_rviz: "
                    "scene_hardware shares that arg and we pass false."
                ),
            ),
            LogInfo(
                msg=(
                    "[overlay] Arm from CPS /joint_states, container mesh "
                    "from /luggage/debug/scene_assets, clouds via TF into "
                    "elfin_base_link. Livox /livox/lidar + D555 "
                    "/camera/d555/depth/color/points. One D555 wrapper."
                )
            ),
            OpaqueFunction(function=_warn_livox_host),
            OpaqueFunction(function=_cps),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(scene_launch),
                launch_arguments={
                    "scene_tf_config": LaunchConfiguration("scene_tf_config"),
                    "use_rviz": "false",
                    "use_sim_time": "false",
                }.items(),
                condition=IfCondition(LaunchConfiguration("start_scene")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mid360_launch),
                launch_arguments={
                    "user_config_path": LaunchConfiguration("user_config_path"),
                    "frame_id": "livox_frame",
                    "xfer_format": "0",
                }.items(),
                condition=IfCondition(LaunchConfiguration("start_mid360")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(d555_launch),
                launch_arguments={
                    "serial_no": LaunchConfiguration("serial_no"),
                    "camera_namespace": LaunchConfiguration("camera_namespace"),
                    "camera_name": LaunchConfiguration("camera_name"),
                    "pointcloud_enable": "true",
                    "align_depth_enable": "true",
                }.items(),
                condition=IfCondition(LaunchConfiguration("start_d555")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="overlay_rviz",
                output="screen",
                arguments=[
                    "-d",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("elfin_description"),
                            "rviz",
                            "overlay_livox_d555.rviz",
                        ]
                    ),
                ],
                condition=IfCondition(LaunchConfiguration("start_rviz")),
            ),
        ]
    )
