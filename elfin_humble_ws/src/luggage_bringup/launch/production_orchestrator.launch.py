"""Launch the production orchestrator without manufacturing operator consent."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="luggage_bringup",
                executable="orchestrator_node.py",
                name="orchestrator",
                output="screen",
            )
        ]
    )
