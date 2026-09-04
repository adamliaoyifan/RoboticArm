"""Scene static TF plus robot_state_publisher for S20 + suction + D435."""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_scene = PathJoinSubstitution(
        [
            FindPackageShare("luggage_description"),
            "config",
            "scene_tf.yaml.example",
        ]
    )
    scene_tf_config = LaunchConfiguration("scene_tf_config")
    use_rviz = LaunchConfiguration("use_rviz")

    helper = os.path.join(
        get_package_prefix("luggage_description"),
        "lib",
        "luggage_description",
        "xacro_robot_with_scene_base",
    )
    xacro_path = os.path.join(
        get_package_share_directory("luggage_description"),
        "urdf",
        "elfin_s20_with_camera.urdf.xacro",
    )
    robot_description = {
        "robot_description": ParameterValue(
            Command([helper, " ", xacro_path, " ", scene_tf_config]),
            value_type=str,
        )
    }

    publisher = Node(
        package="luggage_description",
        executable="container_tf_publisher",
        name="container_tf_publisher",
        output="screen",
        parameters=[
            {
                "scene_tf_config": scene_tf_config,
                "republish_period": 0.0,
            }
        ],
    )
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    # Humble RSP only publishes revolute-joint TF after /joint_states.
    # Zeros are enough to check suction/camera frames; no ros2_control.
    jsp = Node(
        package="luggage_description",
        executable="zero_joint_state_publisher",
        name="zero_joint_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("elfin_description"), "rviz", "view_robot.rviz"]
            ),
        ],
        parameters=[robot_description],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("scene_tf_config", default_value=default_scene),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            publisher,
            rsp,
            jsp,
            rviz,
        ]
    )
