"""Site recording graph: CPS joints, Mid-360, D555 RGBD, optional ros2 bag.

Source order (one shell, Jazzy only, typically ROS_DOMAIN_ID=7):

  source /opt/ros/jazzy/setup.bash
  source /home/adamliao/work/RoboticArm/deployment_ws/livox_ws/env.sh
  source /home/adamliao/work/RoboticArm/elfin_humble_ws/install/setup.bash
  source /home/adamliao/work/RoboticArm/deployment_ws/install/setup.bash

Then:

  ros2 launch elfin_trajectory_executor record_site.launch.py

Do not start jazzy_real.launch.py at the same time (CPS is one TCP client).
Do not start scene.launch.py (zero joints) with this file.

What this starts
----------------
* cps_telemetry — HRIF_ReadActACS / ReadActJointVel / ReadActPos
  /joint_states, /elfin/tcp_pose, /elfin/cps_telemetry
* scene_hardware — robot_state_publisher + container TF (needs /joint_states)
* Mid-360 — /livox/lidar (PointCloud2), /livox/imu
* D555 — color, depth, aligned depth, RGBD, colored points, IMU if present
* ros2 bag (after bag_delay_s) matching the regex below

Bag topics (regex)
------------------
  /joint_states
  /livox/lidar /livox/imu
  /elfin/*
  /camera/d555/*
  /realsense/*

Scene TF is off by default (start_scene:=false) so /tf and /robot_description
are not recorded. Pass start_scene:=true only if you want URDF/world frames.

Host networking (before launch)
-------------------------------
  sudo ip link set enp0s31f6 mtu 9000
  sudo ip addr add 192.168.1.5/24 dev enp0s31f6   # if missing
  sudo ip addr add 192.168.11.70/24 dev enp0s31f6 # D555 LAN, if missing
  pkill -f livox_ros_driver2_node                 # leftover bind

To record motion while the executor owns CPS:

  ros2 launch elfin_trajectory_executor record_site.launch.py \\
    start_cps:=false start_executor:=true
"""

from __future__ import annotations

import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

BAG_REGEX = (
    r"(/joint_states"
    r"|/livox/lidar|/livox/imu"
    r"|/elfin/|/camera/d555/|/realsense/)"
)
BAG_EXCLUDE = r"(/parameter_events|/diagnostics|/rosout)"


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


def _cps_or_executor(context, *args, **kwargs):
    start_cps = _truthy(LaunchConfiguration("start_cps").perform(context))
    start_executor = _truthy(LaunchConfiguration("start_executor").perform(context))
    ip = LaunchConfiguration("robot_ip").perform(context)
    port = LaunchConfiguration("robot_port").perform(context)
    rate = LaunchConfiguration("rate_hz").perform(context)
    if start_executor:
        pkg = get_package_share_directory("elfin_trajectory_executor")
        params = os.path.join(pkg, "config", "executor.yaml")
        return [
            LogInfo(
                msg=(
                    "[record_site] jazzy_real executor owns CPS at %s:%s; "
                    "cps_telemetry is off"
                    % (ip, port)
                )
            ),
            Node(
                package="elfin_trajectory_executor",
                executable="trajectory_executor",
                name="trajectory_executor",
                output="screen",
                parameters=[
                    params,
                    {
                        "mode": "real",
                        "use_sim_time": False,
                        "robot_ip": ip,
                        "robot_port": int(port),
                        "default_velocity_deg": 10.0,
                        "max_velocity_deg": 20.0,
                    },
                ],
            ),
        ]
    if not start_cps:
        return [
            LogInfo(
                msg="[record_site] CPS off. Provide /joint_states from another node."
            )
        ]
    return [
        LogInfo(msg="[record_site] CPS monitor %s:%s (no servo enable)" % (ip, port)),
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


def _bag(context, *args, **kwargs):
    if not _truthy(LaunchConfiguration("record").perform(context)):
        return [LogInfo(msg="[record_site] record:=false — no ros2 bag")]
    pkg = get_package_share_directory("elfin_trajectory_executor")
    qos = LaunchConfiguration("bag_qos").perform(context) or os.path.join(
        pkg, "config", "bag_qos_overrides.yaml"
    )
    bag_dir = os.path.expanduser(LaunchConfiguration("bag_dir").perform(context))
    os.makedirs(bag_dir, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    out = os.path.join(bag_dir, "record_site_%s" % stamp)
    delay = float(LaunchConfiguration("bag_delay_s").perform(context))
    cmd = [
        "ros2",
        "bag",
        "record",
        "-o",
        out,
        "--qos-profile-overrides-path",
        qos,
        "--regex",
        BAG_REGEX,
        "--exclude-regex",
        BAG_EXCLUDE,
        "--disable-keyboard-controls",
        "--max-cache-size",
        str(200 * 1024 * 1024),
    ]
    return [
        LogInfo(msg="[record_site] bag in %.1fs -> %s" % (delay, out)),
        TimerAction(
            period=delay,
            actions=[
                ExecuteProcess(cmd=cmd, output="screen", name="ros2_bag_record"),
            ],
        ),
    ]


def generate_launch_description():
    pkg = get_package_share_directory("elfin_trajectory_executor")
    default_qos = os.path.join(pkg, "config", "bag_qos_overrides.yaml")
    d555_launch = os.path.join(pkg, "launch", "d555_rgbd.launch.py")

    scene_share = _share(
        "luggage_description",
        "source /home/adamliao/work/RoboticArm/elfin_humble_ws/install/setup.bash "
        "after livox_ws/env.sh",
    )
    scene_launch = os.path.join(scene_share, "launch", "scene_hardware.launch.py")
    mid360_launch = os.path.join(scene_share, "launch", "mid360.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_ip",
                default_value=os.environ.get("ROBOT_IP", "192.168.0.10"),
            ),
            DeclareLaunchArgument(
                "robot_port",
                default_value=os.environ.get("ROBOT_PORT", "10003"),
            ),
            DeclareLaunchArgument("rate_hz", default_value="50.0"),
            DeclareLaunchArgument(
                "start_cps",
                default_value="true",
                description="Read-only CPS telemetry (no electrify).",
            ),
            DeclareLaunchArgument(
                "start_executor",
                default_value="false",
                description="If true, start jazzy_real executor instead of telemetry.",
            ),
            DeclareLaunchArgument(
                "start_scene",
                default_value="false",
                description="scene_tf / robot_state_publisher. Off for raw sensor bags.",
            ),
            DeclareLaunchArgument("start_mid360", default_value="true"),
            DeclareLaunchArgument("start_d555", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("record", default_value="true"),
            DeclareLaunchArgument(
                "bag_dir",
                default_value=os.path.expanduser("~/robotarm_bags"),
            ),
            DeclareLaunchArgument("bag_delay_s", default_value="5.0"),
            DeclareLaunchArgument("bag_qos", default_value=default_qos),
            DeclareLaunchArgument(
                "scene_tf_config",
                default_value=_default_scene_tf(),
            ),
            DeclareLaunchArgument(
                "user_config_path",
                default_value=_default_mid360(),
                description="Livox JSON. Keep this path on one line.",
            ),
            DeclareLaunchArgument("serial_no", default_value="419222302385"),
            DeclareLaunchArgument("camera_namespace", default_value="camera"),
            DeclareLaunchArgument("camera_name", default_value="d555"),
            LogInfo(
                msg=(
                    "[record_site] No Gazebo. CPS + Mid-360 /livox/lidar,/livox/imu "
                    "+ D555 RGBD + bag regex for those topics. use_sim_time=false."
                )
            ),
            OpaqueFunction(function=_cps_or_executor),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(scene_launch),
                launch_arguments={
                    "scene_tf_config": LaunchConfiguration("scene_tf_config"),
                    "use_rviz": LaunchConfiguration("use_rviz"),
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
                }.items(),
                condition=IfCondition(LaunchConfiguration("start_d555")),
            ),
            OpaqueFunction(function=_bag),
        ]
    )
