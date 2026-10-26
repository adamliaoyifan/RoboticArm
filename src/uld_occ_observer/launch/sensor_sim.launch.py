"""Sensor-only ROS graph. Gazebo itself is owned by the eval driver PID group."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    default_config = str(Path(get_package_share_directory("uld_occ_observer")) / "config/observer.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("geometry_file"),
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("optical_frame", default_value="d555_color_optical_frame"),
        DeclareLaunchArgument("cloud_topic", default_value="/uld_occ_observer/optical_points"),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="observer_clock_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            output="screen",
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="observer_depth_bridge",
            arguments=[
                "/observer_depth@sensor_msgs/msg/Image[gz.msgs.Image",
                "/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            ],
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
        Node(
            package="uld_occ_observer",
            executable="observer_node",
            output="screen",
            parameters=[{
                "geometry_file": ParameterValue(LaunchConfiguration("geometry_file"), value_type=str),
                "config_file": ParameterValue(LaunchConfiguration("config_file"), value_type=str),
                "input_mode": "subscription",
                "optical_frame": ParameterValue(LaunchConfiguration("optical_frame"), value_type=str),
                "cloud_topic": ParameterValue(LaunchConfiguration("cloud_topic"), value_type=str),
                "use_sim_time": True,
                "sensor_id": "d555",
            }],
        ),
    ])
