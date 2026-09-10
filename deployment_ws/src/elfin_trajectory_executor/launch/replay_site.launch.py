"""Replay compressed site bags onto the canonical Image + PointCloud2 names.

  ros2 launch elfin_trajectory_executor replay_site.launch.py bag:=/path/to/bag

Plays with --clock. Decompresses D555 JPEG/PNG. Converts Livox CustomMsg
(recorded as /livox/lidar) onto /livox/lidar PointCloud2 via a remap.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _play(context, *args, **kwargs):
    bag = LaunchConfiguration("bag").perform(context)
    qos = LaunchConfiguration("qos").perform(context)
    if not bag:
        raise RuntimeError("pass bag:=/path/to/rosbag")
    cmd = [
        "ros2",
        "bag",
        "play",
        bag,
        "--clock",
        "--remap",
        "/livox/lidar:=/livox/lidar_custom",
    ]
    if qos:
        cmd.extend(["--qos-profile-overrides-path", qos])
    return [ExecuteProcess(cmd=cmd, output="screen", name="ros2_bag_play")]


def generate_launch_description():
    from ament_index_python.packages import get_package_share_directory
    import os

    pkg = get_package_share_directory("elfin_trajectory_executor")
    default_qos = os.path.join(pkg, "config", "bag_qos_overrides.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("bag", default_value=""),
            DeclareLaunchArgument("qos", default_value=default_qos),
            OpaqueFunction(function=_play),
            Node(
                package="elfin_trajectory_executor",
                executable="d555_decompress",
                name="d555_decompress",
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="elfin_trajectory_executor",
                executable="livox_custom_to_cloud",
                name="livox_custom_to_cloud",
                parameters=[
                    {
                        "use_sim_time": True,
                        "input_topic": "/livox/lidar_custom",
                        "output_topic": "/livox/lidar",
                    }
                ],
            ),
        ]
    )
