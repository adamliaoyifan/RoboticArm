"""Private snapshot joint connector; explicit model/profile supplied by caller.

The parameter file must carry the exact robot_description and SRDF bytes shared
with the preparation node and its captured scenes. It can include OMPL planner
configuration. This starts no controller, move_group or live scene updater.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model_parameters", description=(
            "ROS parameter file binding the current robot model and OMPL configuration")),
        Node(package="luggage_planning", executable="snapshot_joint_path_node",
             name="snapshot_joint_path", output="screen",
             parameters=[LaunchConfiguration("model_parameters")]),
    ])
