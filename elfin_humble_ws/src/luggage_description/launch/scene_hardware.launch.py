"""Scene TF + robot_state_publisher for the real arm.

Uses executor /joint_states. Does not start Gazebo or zero_joint_state_publisher.
Pass a measured scene_tf.yaml; the share example is simulation geometry.
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Simulation numbers. Gate 3 requires a measured file via scene_tf_config:=
    default_scene = PathJoinSubstitution(
        [
            FindPackageShare("luggage_description"),
            "config",
            "scene_tf.yaml.example",
        ]
    )
    scene_tf_config = LaunchConfiguration("scene_tf_config")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

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
                "use_sim_time": use_sim_time,
            }
        ],
    )
    scene_assets = Node(
        package="luggage_description",
        executable="scene_assets_node",
        name="scene_assets",
        output="screen",
        parameters=[
            {
                "scene_tf_config": scene_tf_config,
                "use_sim_time": use_sim_time,
            }
        ],
    )
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("elfin_description"), "rviz", "view_arm_livox.rviz"]
            ),
        ],
        parameters=[robot_description],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scene_tf_config",
                default_value=default_scene,
                description=(
                    "Measured scene_tf.yaml. Default is the simulation example; "
                    "do not treat those numbers as site geometry."
                ),
            ),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="True when replaying a bag with --clock.",
            ),
            LogInfo(
                msg=(
                    "[scene_hardware] No Gazebo, no zero joints. "
                    "Expect /joint_states from cps_telemetry or jazzy_real. "
                    "Publishes world→elfin_base_link (URDF) + container TF + "
                    "/luggage/debug/scene_assets. "
                    "Replace scene_tf.yaml with measured site values."
                )
            ),
            publisher,
            scene_assets,
            rsp,
            rviz,
        ]
    )
