"""D555 color-only bring-up for ChArUco / hand-eye capture.

The production launch (d555_rgbd.launch.py) remaps images to *_hw and waits
for a color+aligned_depth pair in d555_host_stamp. On this PoE D555 that pair
never appears (color_hw=0), so canonical /camera/d555/color/image_raw stays
silent. Hand-eye only needs a still RGB frame + CameraInfo, not aligned depth
or host-clock restamp.

This launch:
  - streams Color+Depth 640x360@15 (depth stays on so D555 DDS emits RGB)
  - color_qos DEFAULT = RELIABLE+VOLATILE so `ros2 topic echo` / `hz` work.
    SENSOR_DATA (BEST_EFFORT) cannot serve echo/hz; the wrapper then sees
    zero matching subscribers and skips publishFrame entirely.
  - does not remap image_raw
  - does not start d555_host_stamp
  - does not lock image_transport plugins

Stop d555_rgbd.launch.py before starting this. One wrapper per D555.

  ros2 launch elfin_trajectory_executor d555_calib.launch.py
  ros2 launch elfin_trajectory_executor d555_calib.launch.py pointcloud_enable:=true align_depth_enable:=true
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

APT_LIBREALSENSE = "/lib/x86_64-linux-gnu"


def _truthy(value: str) -> bool:
    return value.strip().lower() in ("true", "1", "yes", "on")


def _ld_library_path():
    cur = os.environ.get("LD_LIBRARY_PATH", "")
    parts = [p for p in cur.split(os.pathsep) if p]
    drop = {"/opt/ros/jazzy/lib/x86_64-linux-gnu"}
    parts = [p for p in parts if p not in drop]
    if APT_LIBREALSENSE in parts:
        parts.remove(APT_LIBREALSENSE)
    parts.insert(0, APT_LIBREALSENSE)
    return os.pathsep.join(parts)


def _node(context, *args, **kwargs):
    ns = LaunchConfiguration("camera_namespace").perform(context)
    name = LaunchConfiguration("camera_name").perform(context)
    serial = LaunchConfiguration("serial_no").perform(context)
    pointcloud = _truthy(LaunchConfiguration("pointcloud_enable").perform(context))
    align_depth = _truthy(LaunchConfiguration("align_depth_enable").perform(context))
    ld_path = _ld_library_path()
    driver_params = {
        "use_sim_time": False,
        "camera_name": name,
        "serial_no": serial,
        "enable_color": True,
        "enable_color1": False,
        # D555 DDS often emits no RGB frames if depth is off. Keep depth
        # running. Alignment / native cloud stay off unless overlay asks.
        "enable_depth": True,
        "enable_sync": False,
        "enable_rgbd": False,
        "align_depth.enable": align_depth,
        "pointcloud.enable": pointcloud,
        "pointcloud.allow_no_texture_points": True,
        "pointcloud.ordered_pc": False,
        "pointcloud_qos": "DEFAULT",
        "enable_infra": False,
        "enable_infra1": False,
        "enable_infra2": False,
        "enable_gyro": False,
        "enable_accel": False,
        "enable_motion": False,
        "enable_safety": False,
        "enable_labeled_point_cloud": False,
        "enable_occupancy": False,
        "publish_tf": True,
        "tf_publish_rate": 0.0,
        "diagnostics_period": 0.0,
        # DEFAULT = RELIABLE + VOLATILE. Serves ros2 topic echo/hz
        # (Reliable) and RViz Image (Best Effort). Do not use SENSOR_DATA:
        # a Best-Effort publisher cannot send to echo, and the wrapper
        # skips frames when subscription_count is 0.
        # Do not use SYSTEM_DEFAULT: FastDDS turns it into TRANSIENT_LOCAL
        # and large images never leave the writer.
        "color_qos": "DEFAULT",
        "color_info_qos": "DEFAULT",
        "depth_qos": "DEFAULT",
        "depth_info_qos": "DEFAULT",
        "rgb_camera.color_format": "RGB8",
        "rgb_camera.color_profile": LaunchConfiguration("color_profile").perform(
            context
        ),
        "depth_module.depth_profile": LaunchConfiguration("depth_profile").perform(
            context
        ),
    }
    return [
        LogInfo(
            msg=(
                "[d555_calib] namespace=/%s/%s serial=%s color+depth "
                "qos=DEFAULT (echo/hz) align=%s cloud=%s. "
                "Topics /%s/%s/color/image_raw /%s/%s/depth/color/points"
                % (
                    ns,
                    name,
                    serial or "(first DDS device)",
                    align_depth,
                    pointcloud,
                    ns,
                    name,
                    ns,
                    name,
                )
            )
        ),
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            namespace=ns,
            name=name,
            output="screen",
            emulate_tty=True,
            additional_env={"LD_LIBRARY_PATH": ld_path},
            prefix="env LD_LIBRARY_PATH=" + ld_path,
            parameters=[driver_params],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_namespace", default_value="camera"),
            DeclareLaunchArgument("camera_name", default_value="d555"),
            DeclareLaunchArgument(
                "serial_no",
                default_value="419222302385",
                description="D555 PoE serial. Empty = first DDS device.",
            ),
            DeclareLaunchArgument(
                "color_profile",
                default_value="640,360,15",
                description="rgb_camera.color_profile. 896x504@30 drops DDS.",
            ),
            DeclareLaunchArgument(
                "depth_profile",
                default_value="640,360,15",
                description="Keep depth on so D555 DDS actually emits RGB.",
            ),
            DeclareLaunchArgument(
                "pointcloud_enable",
                default_value="false",
                description=(
                    "Publish /camera/d555/depth/color/points. Off for "
                    "ChArUco; on for Livox overlay."
                ),
            ),
            DeclareLaunchArgument(
                "align_depth_enable",
                default_value="false",
                description=(
                    "Colour-align depth. Needed for an RGB cloud. Off for "
                    "hand-eye stills."
                ),
            ),
            OpaqueFunction(function=_node),
        ]
    )
