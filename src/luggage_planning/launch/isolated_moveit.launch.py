"""MoveIt 2 plan-only stack with no Gazebo and no trajectory execution.

Intended for site-bag pick replay on an isolated ROS_DOMAIN_ID (never 7).
``allow_trajectory_execution`` is false so this cannot drive the real arm
even if DDS isolation failed.
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def _semantic_description():
    path = os.path.join(
        get_package_share_directory("luggage_description"),
        "config",
        "S20_with_camera.srdf",
    )
    with open(path, "r", encoding="utf-8") as handle:
        xml = handle.read()
    xml = xml.replace('name="elfin_s20_with_camera"', 'name="S20"', 1)
    return {"robot_description_semantic": xml}


def _launch(context, *args, **kwargs):
    scene_tf_config = LaunchConfiguration("scene_tf_config")
    use_rviz = LaunchConfiguration("use_rviz")
    use_gz_sim = LaunchConfiguration("use_gz_sim").perform(context)
    hardware_plugin = LaunchConfiguration("hardware_plugin").perform(context)
    measure_distance = LaunchConfiguration("measure_distance").perform(context)
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
    xacro_command = [helper, " ", xacro_path, " ", scene_tf_config]
    if hardware_plugin:
        xacro_command.extend([" ", "hardware_plugin:=" + hardware_plugin])
    if use_gz_sim.strip().lower() in ("1", "true", "yes"):
        xacro_command.extend([" ", "use_gz_sim:=true"])
    extra = LaunchConfiguration("xacro_extra").perform(context)
    for item in extra.split(","):
        item = item.strip()
        if item:
            xacro_command.extend([" ", item])
    robot_description = {
        "robot_description": ParameterValue(
            Command(xacro_command),
            value_type=str,
        )
    }
    measure = measure_distance.strip().lower() in ("1", "true", "yes")
    moveit_config = (
        MoveItConfigsBuilder("S20", package_name="elfin_moveit_config")
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": False}],
    )
    publisher = Node(
        package="luggage_description",
        executable="container_tf_publisher",
        name="container_tf_publisher",
        output="screen",
        parameters=[{"scene_tf_config": scene_tf_config,
                     "republish_period": 0.0}],
    )
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            robot_description,
            _semantic_description(),
            {
                "publish_robot_description_semantic": True,
                "allow_trajectory_execution": False,
                "use_sim_time": False,
            },
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(use_rviz),
        output="screen",
        parameters=[robot_description],
    )
    # Same URDF/SRDF move_group plans with. Plan-only certification
    # calls /validate_route_v2; without this node a planned route dies
    # as CERT_SERVICE_UNAVAILABLE.
    validator = Node(
        package="luggage_planning",
        executable="validate_route_node",
        name="validate_route_v2",
        output="screen",
        parameters=[robot_description, _semantic_description(), {
            "measure_distance": measure,
        }],
    )
    return [publisher, rsp, move_group, validator, rviz]


def generate_launch_description():
    from luggage_description.launch_profile import with_profile
    default_scene = PathJoinSubstitution(
        [
            FindPackageShare("luggage_description"),
            "config",
            "scene_tf_deploy_update.yaml",
        ]
    )
    return with_profile(LaunchDescription(
        [
            DeclareLaunchArgument(
                "scene_tf_config",
                default_value=default_scene,
                description="scene_tf.yaml; default is the simulation example",
            ),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument(
                "use_gz_sim", default_value="false",
                description="Expand the Gazebo URDF flavour.",
            ),
            DeclareLaunchArgument(
                "hardware_plugin", default_value="",
                description="xacro hardware_plugin override; empty keeps mock.",
            ),
            DeclareLaunchArgument(
                "measure_distance", default_value="true",
                description="GLOBAL distanceRobot at sample 0 of each leg.",
            ),
            DeclareLaunchArgument(
                "xacro_extra", default_value="",
                description="Comma-separated extra xacro key:=value mappings.",
            ),
            SetEnvironmentVariable(name="ROS_LOCALHOST_ONLY", value="1"),
            LogInfo(msg=(
                "[isolated_moveit] No Gazebo, no trajectory execution. "
                "Do not use ROS_DOMAIN_ID=7 (live sim).")),
            OpaqueFunction(function=_launch),
        ]
    ))
