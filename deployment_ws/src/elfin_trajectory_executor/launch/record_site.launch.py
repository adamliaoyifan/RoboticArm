"""Site recording graph: two replay-capable bag modes.

  record_mode:=pendant  teach pendant, CPS monitor only (default)
  record_mode:=real     jazzy_real executor owns the arm (FJT)

Source (one shell, Jazzy only):

  source /opt/ros/jazzy/setup.bash
  source /home/adamliao/work/RoboticArm/deployment_ws/livox_ws/env.sh
  source /home/adamliao/work/RoboticArm/elfin_humble_ws/install/setup.bash
  source /home/adamliao/work/RoboticArm/deployment_ws/install/setup.bash
  export ROS_DOMAIN_ID=7
  export PYTHONPATH=/home/adamliao/work/RoboticArm/third_party/huayan_python_sdk:${PYTHONPATH}
  export LD_LIBRARY_PATH=/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH}

Ctrl+C the launch to stop. Do not start scene.launch.py (zero joints).
Do not run pendant and real at once (one CPS TCP client).

Replay contract: docs/status/bag_recording_replay.md
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

# Raw sensors + actual joints + TF. Enough to replay perception (remap D555
# names, rebuild camera points with depth_image_proc). No /luggage/* outputs.

def _pendant_regex(compress: bool) -> str:
    if compress:
        images = (
            r"|/camera/d555/color/image_raw/compressed$"
            r"|/camera/d555/aligned_depth_to_color/image_raw/compressed$"
        )
    else:
        images = (
            r"|/camera/d555/color/image_raw$"
            r"|/camera/d555/aligned_depth_to_color/image_raw$"
        )
    return (
        r"(/joint_states$"
        r"|/livox/lidar$|/livox/imu$"
        r"|/elfin/"
        r"|/vacuum/"
        r"|/tf$|/tf_static$"
        + images
        + r"|/camera/d555/color/camera_info$"
        r"|/camera/d555/aligned_depth_to_color/camera_info$"
        r"|/camera/d555/imu$"
        r"|/camera/d555/gyro/"
        r"|/camera/d555/accel/"
        r"|/camera/d555/motion/"
        r"|/clock_sync/)"
    )


BAG_REGEX_PENDANT = _pendant_regex(True)
BAG_REGEX_REAL = (
    BAG_REGEX_PENDANT[:-1]
    + r"|/trajectory_executor/"
    + r"|/elfin_arm_controller/follow_joint_trajectory"
    + r"|/motion_planner/)"
)
BAG_REGEX_FULL = (
    r"(/joint_states"
    r"|/livox/lidar|/livox/imu"
    r"|/elfin/|/camera/d555/|/realsense/|/vacuum/"
    r"|/clock_sync/"
    r"|/tf$|/tf_static$"
    r"|/trajectory_executor/"
    r"|/elfin_arm_controller/follow_joint_trajectory"
    r"|/motion_planner/)"
)
# Keep algorithm outputs out of the bag so replay can run a new stack.
BAG_EXCLUDE = r"(/parameter_events|/diagnostics|/rosout|/luggage/|_hw$)"


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


def _record_mode(context) -> str:
    return str(LaunchConfiguration("record_mode").perform(context)).strip().lower()


def _want_executor(context) -> bool:
    mode = _record_mode(context)
    if mode == "real":
        return True
    if mode == "pendant":
        return False
    return _truthy(LaunchConfiguration("start_executor").perform(context))


def _want_cps(context) -> bool:
    if _want_executor(context):
        return False
    mode = _record_mode(context)
    if mode == "pendant":
        return True
    return _truthy(LaunchConfiguration("start_cps").perform(context))


def _want_scene(context) -> bool:
    if _record_mode(context) in ("pendant", "real"):
        return True
    return _truthy(LaunchConfiguration("start_scene").perform(context))


def _scene(context, *args, **kwargs):
    if not _want_scene(context):
        return [LogInfo(msg="[record_site] scene_hardware off")]
    scene_share = _share(
        "luggage_description",
        "source /home/adamliao/work/RoboticArm/elfin_humble_ws/install/setup.bash "
        "after livox_ws/env.sh",
    )
    scene_launch = os.path.join(scene_share, "launch", "scene_hardware.launch.py")
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(scene_launch),
            launch_arguments={
                "scene_tf_config": LaunchConfiguration("scene_tf_config"),
                "use_rviz": LaunchConfiguration("use_rviz"),
                "use_sim_time": "false",
            }.items(),
        )
    ]


def _cps_or_executor(context, *args, **kwargs):
    ip = LaunchConfiguration("robot_ip").perform(context)
    port = LaunchConfiguration("robot_port").perform(context)
    rate = LaunchConfiguration("rate_hz").perform(context)
    if _want_executor(context):
        pkg = get_package_share_directory("elfin_trajectory_executor")
        params = os.path.join(pkg, "config", "executor.yaml")
        return [
            LogInfo(
                msg=(
                    "[record_site] REAL mode: jazzy_real executor owns CPS at %s:%s"
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
    if not _want_cps(context):
        return [
            LogInfo(
                msg="[record_site] CPS off. Provide /joint_states from another node."
            )
        ]
    return [
        LogInfo(
            msg="[record_site] PENDANT mode: CPS monitor %s:%s (no servo enable)"
            % (ip, port)
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


def _bag(context, *args, **kwargs):
    if not _truthy(LaunchConfiguration("record").perform(context)):
        return [LogInfo(msg="[record_site] record:=false — no ros2 bag")]
    pkg = get_package_share_directory("elfin_trajectory_executor")
    qos = LaunchConfiguration("bag_qos").perform(context) or os.path.join(
        pkg, "config", "bag_qos_overrides.yaml"
    )
    bag_path = str(LaunchConfiguration("bag_path").perform(context)).strip()
    mode = _record_mode(context)
    if mode not in ("pendant", "real"):
        mode = "real" if _want_executor(context) else "pendant"
    if bag_path:
        out = os.path.abspath(os.path.expanduser(bag_path))
        parent = os.path.dirname(out)
        if parent:
            os.makedirs(parent, exist_ok=True)
        if os.path.exists(out):
            raise RuntimeError("bag_path already exists: %s" % out)
    else:
        bag_dir = os.path.abspath(
            os.path.expanduser(LaunchConfiguration("bag_dir").perform(context))
        )
        os.makedirs(bag_dir, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        out = os.path.join(bag_dir, "record_site_%s_%s" % (mode, stamp))
    delay = float(LaunchConfiguration("bag_delay_s").perform(context))
    bag_full = _truthy(LaunchConfiguration("bag_full").perform(context))
    compress = _truthy(LaunchConfiguration("compress_rgbd").perform(context))
    pendant = _pendant_regex(compress)
    if bag_full:
        regex = BAG_REGEX_FULL
        kind = "full"
    elif _want_executor(context):
        regex = (
            pendant[:-1]
            + r"|/trajectory_executor/"
            + r"|/elfin_arm_controller/follow_joint_trajectory"
            + r"|/motion_planner/)"
        )
        kind = "real"
    else:
        regex = pendant
        kind = "pendant"
    cmd = [
        "ros2",
        "bag",
        "record",
        "-o",
        out,
        "--qos-profile-overrides-path",
        qos,
        "--regex",
        regex,
        "--exclude-regex",
        BAG_EXCLUDE,
        "--disable-keyboard-controls",
        "--max-cache-size",
        str(64 * 1024 * 1024),
    ]
    return [
        LogInfo(msg="[record_site] %s bag in %.1fs -> %s" % (kind, delay, out)),
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
                "record_mode",
                default_value="pendant",
                description="pendant (teach + CPS monitor) or real (jazzy_real FJT).",
            ),
            DeclareLaunchArgument(
                "start_cps",
                default_value="true",
                description="Read-only CPS telemetry. Ignored when record_mode is set.",
            ),
            DeclareLaunchArgument(
                "start_executor",
                default_value="false",
                description="Used only if record_mode is not pendant/real.",
            ),
            DeclareLaunchArgument(
                "start_scene",
                default_value="false",
                description="Used only if record_mode is not pendant/real.",
            ),
            DeclareLaunchArgument("start_mid360", default_value="true"),
            DeclareLaunchArgument("start_d555", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("record", default_value="true"),
            DeclareLaunchArgument(
                "bag_full",
                default_value="false",
                description="If true, record all D555/realsense topics (huge).",
            ),
            DeclareLaunchArgument("pointcloud_enable", default_value="false"),
            DeclareLaunchArgument("enable_rgbd", default_value="false"),
            DeclareLaunchArgument("enable_infra", default_value="false"),
            DeclareLaunchArgument("enable_imu", default_value="true"),
            DeclareLaunchArgument("enable_color1", default_value="false"),
            DeclareLaunchArgument(
                "compress_rgbd",
                default_value="true",
                description="JPEG color + PNG depth in the bag (not raw Image).",
            ),
            DeclareLaunchArgument("jpeg_quality", default_value="80"),
            DeclareLaunchArgument(
                "publish_raw",
                default_value="false",
                description="Also publish uncompressed D555 Image (more ROS traffic).",
            ),
            DeclareLaunchArgument(
                "xfer_format",
                default_value="1",
                description="Livox 0=PointCloud2, 1=CustomMsg (smaller ROS/bag).",
            ),
            DeclareLaunchArgument(
                "bag_dir",
                default_value=os.path.expanduser("~/robotarm_bags"),
            ),
            DeclareLaunchArgument(
                "bag_path",
                default_value="",
                description="Exact ros2 bag -o path. Empty = bag_dir/record_site_<mode>_<stamp>.",
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
                    "[record_site] No Gazebo. Clock master = host ROS system time. "
                    "CPS + Mid-360 CustomMsg + D555 JPEG/PNG (d555_host_stamp). "
                    "use_sim_time=false."
                )
            ),
            OpaqueFunction(function=_cps_or_executor),
            OpaqueFunction(function=_scene),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mid360_launch),
                launch_arguments={
                    "user_config_path": LaunchConfiguration("user_config_path"),
                    "frame_id": "livox_frame",
                    "xfer_format": LaunchConfiguration("xfer_format"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("start_mid360")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(d555_launch),
                launch_arguments={
                    "serial_no": LaunchConfiguration("serial_no"),
                    "camera_namespace": LaunchConfiguration("camera_namespace"),
                    "camera_name": LaunchConfiguration("camera_name"),
                    "pointcloud_enable": LaunchConfiguration("pointcloud_enable"),
                    "enable_rgbd": LaunchConfiguration("enable_rgbd"),
                    "enable_infra": LaunchConfiguration("enable_infra"),
                    "enable_imu": LaunchConfiguration("enable_imu"),
                    "enable_color1": LaunchConfiguration("enable_color1"),
                    "compress": LaunchConfiguration("compress_rgbd"),
                    "jpeg_quality": LaunchConfiguration("jpeg_quality"),
                    "publish_raw": LaunchConfiguration("publish_raw"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("start_d555")),
            ),
            OpaqueFunction(function=_bag),
        ]
    )
