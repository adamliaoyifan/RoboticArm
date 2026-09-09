"""D555 PoE RGBD + point cloud via apt librealsense (DDS).

Intel's ROS wrapper is built against a librealsense without DDS. The apt
library in /lib/x86_64-linux-gnu enumerates the D555. This launch prepends
that path for this node only so Livox's SDK path is left alone.

Standalone:
  ros2 launch elfin_trajectory_executor d555_rgbd.launch.py

Expect:
  /camera/d555/color/image_raw
  /camera/d555/depth/image_rect_raw
  /camera/d555/aligned_depth_to_color/image_raw
  /camera/d555/rgbd
  /camera/d555/depth/color/points
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

APT_LIBREALSENSE = "/lib/x86_64-linux-gnu"


def _ld_library_path():
    cur = os.environ.get("LD_LIBRARY_PATH", "")
    parts = [p for p in cur.split(os.pathsep) if p]
    if APT_LIBREALSENSE not in parts:
        parts.insert(0, APT_LIBREALSENSE)
    return os.pathsep.join(parts)


def _node(context, *args, **kwargs):
    ns = LaunchConfiguration("camera_namespace").perform(context)
    name = LaunchConfiguration("camera_name").perform(context)
    serial = LaunchConfiguration("serial_no").perform(context)
    return [
        LogInfo(
            msg=(
                "[d555] RGBD+pointcloud namespace=/%s/%s serial=%s "
                "LD_LIBRARY_PATH starts with %s"
                % (ns, name, serial or "(first DDS device)", APT_LIBREALSENSE)
            )
        ),
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            namespace=ns,
            name=name,
            output="screen",
            emulate_tty=True,
            additional_env={"LD_LIBRARY_PATH": _ld_library_path()},
            parameters=[
                {
                    "use_sim_time": False,
                    "camera_name": name,
                    "serial_no": serial,
                    "enable_color": True,
                    "enable_depth": True,
                    "enable_sync": True,
                    "enable_rgbd": True,
                    "align_depth.enable": True,
                    "pointcloud.enable": True,
                    "enable_gyro": True,
                    "enable_accel": True,
                    "enable_motion": True,
                    "unite_imu_method": 2,
                    "publish_tf": True,
                    "rgb_camera.color_format": "RGB8",
                }
            ],
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
            OpaqueFunction(function=_node),
        ]
    )
