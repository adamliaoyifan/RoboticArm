"""Offline deploy-asset sampling stack (ST09-PD). No Gazebo, no hardware.

robot_state_publisher (hardware-flavour robot_description, the exact
bytes hardware_pick expands) + a static /joint_states publisher +
MoveIt 2 move_group (planning only, no execution) + scene_manager
(container collision world). The atlas builder, the FK certifier and
the route-graph builder run against this stack. Nothing here executes
a trajectory or touches a controller.
"""

from __future__ import annotations

import os

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _robot_description(scene_tf_config):
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
    return {
        "robot_description": ParameterValue(
            Command([helper, " ", xacro_path, " ", scene_tf_config]),
            value_type=str,
        )
    }


def _semantic_srdf():
    path = os.path.join(
        get_package_share_directory("luggage_description"),
        "config",
        "S20_with_camera.srdf",
    )
    with open(path, "r", encoding="utf-8") as handle:
        xml = handle.read()
    xml = xml.replace('name="elfin_s20_with_camera"', 'name="S20"', 1)
    return {"robot_description_semantic": xml}


def generate_launch_description():
    desc_share = get_package_share_directory("luggage_description")
    default_scene = os.path.join(desc_share, "config", "scene_tf_deploy.yaml")
    xacro_path = os.path.join(
        desc_share, "urdf", "elfin_s20_with_camera.urdf.xacro")

    from moveit_configs_utils import MoveItConfigsBuilder
    moveit_config = (
        MoveItConfigsBuilder("S20", package_name="elfin_moveit_config")
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    moveit_config.robot_description = _robot_description(
        LaunchConfiguration("scene_tf_config"))
    params = dict(moveit_config.to_dict())
    # NOTE: no Jazzy OMPL overlay here. This stack samples on the Humble
    # host, whose move_group expects scalar plugin keys; the real Jazzy
    # cell applies ompl_planning.jazzy.yaml inside hardware_pick.

    return LaunchDescription([
        DeclareLaunchArgument(
            "scene_tf_config",
            default_value=default_scene,
            description="Deploy scene the assets are sampled against.",
        ),
        LogInfo(msg=(
            "[deploy_asset_stack] No Gazebo, no hardware. Planning-scene "
            "only: RSP + static joint_states + move_group + scene_manager."
        )),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                _robot_description(LaunchConfiguration("scene_tf_config")),
                {"use_sim_time": False},
            ],
        ),
        Node(
            package="luggage_planning",
            executable="static_joint_state_publisher.py",
            name="static_joint_state_publisher",
            output="screen",
            arguments=[
                "--xacro", xacro_path,
                "--scene-tf", LaunchConfiguration("scene_tf_config"),
            ],
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            name="move_group",
            output="screen",
            parameters=[
                params,
                _semantic_srdf(),
                {
                    "publish_robot_description_semantic": True,
                    "allow_trajectory_execution": False,
                    "use_sim_time": False,
                },
            ],
        ),
        Node(
            package="luggage_planning",
            executable="scene_manager_node.py",
            name="scene_manager",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "scene_tf_config": LaunchConfiguration("scene_tf_config"),
                "world_frame": "world",
                "auto_sync": True,
            }],
        ),
    ])
