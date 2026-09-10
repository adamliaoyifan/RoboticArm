"""Real-cell pick graph: D555 + CPS executor + detect + MoveIt + vacuum.

No Gazebo. Person on e-stop. One CPS client (jazzy_real).

  ros2 launch luggage_planning hardware_pick.launch.py

Then in another terminal:

  ros2 run luggage_planning hardware_pick_driver.py
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _bool_text(value):
    return str(value).strip().lower() in ("true", "1", "yes", "on")


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


def _move_group(context, *args, **kwargs):
    if not _bool_text(LaunchConfiguration("use_moveit").perform(context)):
        return []
    try:
        from moveit_configs_utils import MoveItConfigsBuilder
    except ImportError:
        return [
            LogInfo(msg=(
                "[hardware_pick] MoveIt is not installed. Skipping move_group. "
                "Detect-only still works. For plan/pick: sudo apt install "
                "ros-jazzy-moveit-msgs ros-jazzy-moveit-configs-utils "
                "ros-jazzy-moveit-ros-move-group ros-jazzy-moveit-planners-ompl "
                "ros-jazzy-moveit-kinematics ros-jazzy-moveit-simple-controller-manager"
            ))
        ]
    scene = LaunchConfiguration("scene_tf_config").perform(context)
    robot_description = _robot_description(scene)
    moveit_config = (
        MoveItConfigsBuilder("S20", package_name="elfin_moveit_config")
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    moveit_config.robot_description = robot_description
    return [
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            name="move_group",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                _semantic_srdf(),
                {
                    "publish_robot_description_semantic": True,
                    "allow_trajectory_execution": True,
                    "use_sim_time": False,
                },
            ],
        )
    ]


def generate_launch_description():
    desc_share = get_package_share_directory("luggage_description")
    perc_share = get_package_share_directory("luggage_perception")
    exec_share = get_package_share_directory("elfin_trajectory_executor")
    default_scene = os.path.join(desc_share, "config", "scene_tf.yaml")
    if not os.path.isfile(default_scene):
        default_scene = os.path.join(desc_share, "config", "scene_tf.yaml.example")
    poses = os.path.join(desc_share, "config", "robot_poses.yaml.example")
    live_pp = os.path.join(perc_share, "config", "preprocessor_d555_live.yaml")
    semantic = os.path.join(perc_share, "config", "semantic_segmenter.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("scene_tf_config", default_value=default_scene),
            DeclareLaunchArgument("robot_poses_config", default_value=poses),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("use_moveit", default_value="true"),
            DeclareLaunchArgument("start_d555", default_value="true"),
            DeclareLaunchArgument(
                "start_aligned_depth_cloud",
                default_value="false",
                description="Legacy raw-depth PointCloud2 node. Off: preprocessor unprojects compressed depth.",
            ),
            DeclareLaunchArgument("start_executor", default_value="true"),
            DeclareLaunchArgument("semantic_device", default_value="cuda"),
            DeclareLaunchArgument("robot_ip", default_value="192.168.0.10"),
            DeclareLaunchArgument("robot_port", default_value="10003"),
            DeclareLaunchArgument(
                "color_profile",
                default_value="640,360,15",
                description="D555 RGB profile. Do not use 896x504@30.",
            ),
            DeclareLaunchArgument(
                "depth_profile",
                default_value="640,360,15",
                description="D555 depth profile. Match colour.",
            ),
            LogInfo(
                msg=(
                    "[hardware_pick] No Gazebo. e-stop person required. "
                    "CPS via jazzy_real only. D555 colour+aligned depth. "
                    "Ctrl+C does not BlackOut."
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(exec_share, "launch", "jazzy_real.launch.py")
                ),
                launch_arguments={
                    "robot_ip": LaunchConfiguration("robot_ip"),
                    "robot_port": LaunchConfiguration("robot_port"),
                    "default_velocity_deg": "10.0",
                    "max_velocity_deg": "20.0",
                }.items(),
                condition=IfCondition(LaunchConfiguration("start_executor")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(desc_share, "launch", "scene_hardware.launch.py")
                ),
                launch_arguments={
                    "scene_tf_config": LaunchConfiguration("scene_tf_config"),
                    "use_rviz": LaunchConfiguration("use_rviz"),
                    "use_sim_time": "false",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(exec_share, "launch", "d555_rgbd.launch.py")
                ),
                launch_arguments={
                    "color_profile": LaunchConfiguration("color_profile"),
                    "depth_profile": LaunchConfiguration("depth_profile"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("start_d555")),
            ),
            Node(
                package="luggage_perception",
                executable="aligned_depth_cloud_node.py",
                name="aligned_depth_cloud",
                output="screen",
                parameters=[{"use_sim_time": False}],
                condition=IfCondition(
                    LaunchConfiguration("start_aligned_depth_cloud")
                ),
            ),
            Node(
                package="luggage_perception",
                executable="sensor_preprocessor_node.py",
                name="sensor_preprocessor",
                output="screen",
                parameters=[
                    os.path.join(perc_share, "config", "sensor_preprocessor.yaml"),
                    live_pp,
                    {"use_sim_time": False},
                ],
            ),
            Node(
                package="luggage_perception",
                executable="semantic_segmenter_node.py",
                name="semantic_segmenter",
                output="screen",
                parameters=[
                    semantic,
                    {
                        "use_sim_time": False,
                        "device": LaunchConfiguration("semantic_device"),
                        "self_body_camera_frame": "d555_color_optical_frame",
                        "require_backend": "yolo_world",
                        # D555 is 15 Hz; CPU YOLO cannot keep that queue.
                        # Unbounded processing leaves DetectionFrame stamps
                        # older than cloud_max_age (DETECT_STALE_CLOUD).
                        "max_rate_hz": 2.0,
                    },
                ],
            ),
            Node(
                package="luggage_perception",
                executable="semantic_point_filter_node.py",
                name="semantic_point_filter",
                output="screen",
                parameters=[semantic, {"use_sim_time": False}],
            ),
            Node(
                package="luggage_perception",
                executable="luggage_detector_node.py",
                name="luggage_detector",
                output="screen",
                parameters=[{
                    "scene_tf_config": LaunchConfiguration("scene_tf_config"),
                    "use_semantic": True,
                    "use_sim_time": False,
                    "depth_topic": "/luggage/preprocessed/camera/depth/points",
                    "cloud_max_age_sec": 8.0,
                    "estimate_retry_count": 4,
                    "estimate_retry_period_sec": 0.25,
                    "support_mode": "auto",
                    "platform_z": "",
                    "suitcase_update_timeout_sec": 0.0,
                }],
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
                    "pickup_object_id": "pickup_box",
                    "auto_sync": True,
                }],
            ),
            Node(
                package="luggage_planning",
                executable="waypoint_generator_node.py",
                name="waypoint_generator",
                output="screen",
                parameters=[{
                    "use_sim_time": False,
                    "scene_tf_config": LaunchConfiguration("scene_tf_config"),
                    "world_frame": "world",
                    "place_slot_frame": "elfin_base_link",
                    "use_perception_approach": False,
                }],
            ),
            Node(
                package="luggage_planning",
                executable="motion_planner_node.py",
                name="motion_planner",
                output="screen",
                parameters=[{
                    "use_sim_time": False,
                    "robot_poses_config": LaunchConfiguration("robot_poses_config"),
                    "named_pose_duration": 8.0,
                    "named_pose_max_vel": 0.25,
                    "fjt_action": "/elfin_arm_controller/follow_joint_trajectory",
                }],
            ),
            Node(
                package="luggage_planning",
                executable="vacuum_controller_node.py",
                name="vacuum_controller",
                output="screen",
                parameters=[{
                    "use_sim_time": False,
                    "backend": "hardware",
                    "seal_timeout_sec": 8.0,
                    "follow_rate_hz": 10.0,
                }],
            ),
            OpaqueFunction(function=_move_group),
        ]
    )
