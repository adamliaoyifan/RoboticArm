"""Start livox_ros_driver2 for Mid-360S. No Gazebo.

Build first: deployment_ws/scripts/setup_livox_driver.sh
then source deployment_ws/livox_ws/install/setup.bash.

Does not include the vendor launch files (they hard-code JSON and
xfer_format=custom). This launch starts the driver node with PointCloud2
and a site JSON.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _driver_node(context, *args, **kwargs):
    try:
        get_package_share_directory("livox_ros_driver2")
    except Exception as exc:
        raise RuntimeError(
            "livox_ros_driver2 is not on AMENT_PREFIX_PATH. "
            "Run deployment_ws/scripts/setup_livox_driver.sh (%s)" % exc
        ) from exc

    config = LaunchConfiguration("user_config_path").perform(context)
    frame_id = LaunchConfiguration("frame_id").perform(context)
    xfer = int(LaunchConfiguration("xfer_format").perform(context))
    return [
        LogInfo(
            msg="[mid360] PointCloud2=%s frame=%s config=%s"
            % (xfer == 0, frame_id, config)
        ),
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_lidar_publisher",
            output="screen",
            parameters=[
                {
                    "xfer_format": xfer,
                    "multi_topic": 0,
                    "data_src": 0,
                    "publish_freq": 10.0,
                    "output_data_type": 0,
                    "frame_id": frame_id,
                    "user_config_path": config,
                    "cmdline_input_bd_code": "livox0000000001",
                }
            ],
        ),
    ]


def generate_launch_description():
    desc_share = get_package_share_directory("luggage_description")
    default_cfg = os.path.join(desc_share, "config", "MID360s_config.json.example")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "user_config_path",
                default_value=default_cfg,
                description="Livox JSON. Copy, set sensor/host IP, then pass the path.",
            ),
            DeclareLaunchArgument("frame_id", default_value="livox_frame",
                                 description="PointCloud2 / IMU header frame."),
            DeclareLaunchArgument(
                "xfer_format",
                default_value="0",
                description="0 = PointCloud2 (PointXYZRTLT). 1 = Livox CustomMsg.",
            ),
            LogInfo(msg="[mid360] No Gazebo. Expect /livox/lidar and /livox/imu."),
            OpaqueFunction(function=_driver_node),
        ]
    )
