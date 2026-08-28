"""Minimal camera calibration-check simulation.

World = ground plane (infinite, z=0) + sun + the robot on its pedestal.
NO pickup platform, container or boxes: the camera looks straight down at
mostly the known ground plane, so the reconstruction is testable against
exact geometry (ground z=0, pedestal top z=0.86, camera height ~1.7 m).

Camera bridges are split into two processes: a parameter_bridge holding
more than one gz Image topic intermittently fails to create the second
Image bridge (see docs/status/m1_m2_issues_and_fixes.md).
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from luggage_description.scene_tf_config_utils import (
    load_scene_tf_config,
    gazebo_pedestal_spawn_pose,
)
from luggage_description.xacro_robot_with_scene_base import expand_and_patch

WORLD_NAME = "airport_loading"


def _resource_path() -> str:
    entries = []
    for pkg in ("elfin_description", "luggage_gazebo", "luggage_description"):
        entries.append(os.path.join(get_package_prefix(pkg), "share"))
    entries.append(
        os.path.join(get_package_share_directory("luggage_gazebo"), "models")
    )
    return ":".join(entries)


def _robot_state_publisher(scene_tf_config: str):
    helper = os.path.join(
        get_package_prefix("luggage_description"),
        "lib", "luggage_description", "xacro_robot_with_scene_base",
    )
    xacro_path = os.path.join(
        get_package_share_directory("luggage_description"),
        "urdf", "elfin_s20_with_camera.urdf.xacro",
    )
    robot_description = {
        "robot_description": ParameterValue(
            Command([
                helper, " ", xacro_path, " ", scene_tf_config, " ",
                "hardware_plugin:=gz_ros2_control/GazeboSimSystem", " ",
                "use_gz_sim:=true",
            ]),
            value_type=str,
        )
    }
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )


def _launch_setup(context):
    scene_tf_config = LaunchConfiguration("scene_tf_config").perform(context)

    world_path = os.path.join(
        get_package_share_directory("luggage_gazebo"), "worlds", "airport_loading.sdf"
    )
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_args": "-s -r -v 3 " + world_path,
            "on_exit_shutdown": "true",
        }.items(),
    )

    clock_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        name="clock_bridge", output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )
    image_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        name="image_bridge", output="screen",
        arguments=[
            "/d435/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/d435/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        remappings=[
            ("/d435/image", "/camera/color/image_raw"),
            ("/d435/camera_info", "/camera/depth/camera_info"),
        ],
    )
    depth_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        name="depth_bridge", output="screen",
        arguments=["/d435/depth_image@sensor_msgs/msg/Image[gz.msgs.Image"],
        remappings=[("/d435/depth_image", "/camera/depth/image_meters")],
    )
    depth_republisher = Node(
        package="luggage_gazebo",
        executable="depth_image_republisher.py",
        name="depth_image_republisher",
        output="screen",
    )

    # Welded robot URDF: world_base carries the scene pose; spawn at origin.
    xacro_path = os.path.join(
        get_package_share_directory("luggage_description"),
        "urdf", "elfin_s20_with_camera.urdf.xacro",
    )
    urdf_xml, _pose = expand_and_patch(
        xacro_path,
        config_path=scene_tf_config,
        xacro_args=[
            "fixed_world:=true",
            "use_gz_sim:=true",
            "hardware_plugin:=gz_ros2_control/GazeboSimSystem",
        ],
    )
    urdf_file = os.path.join(os.environ.get("TMPDIR", "/tmp"), "elfin_s20_calib.urdf")
    with open(urdf_file, "w", encoding="utf-8") as handle:
        handle.write(urdf_xml)

    robot_spawner = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=[
            "-world", WORLD_NAME, "-file", urdf_file, "-name", "S20",
            "-x", "0", "-y", "0", "-z", "0",
            "-R", "0", "-P", "0", "-Y", "0",
        ],
    )

    # pedestal (visual/collision reference; the arm is welded via world_base)
    config = load_scene_tf_config(scene_tf_config)
    pedestal = gazebo_pedestal_spawn_pose(config)
    pedestal_spawner = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=[
            "-world", WORLD_NAME,
            "-file", os.path.join(
                get_package_share_directory("luggage_gazebo"),
                "models", "robot_pedestal", "model.sdf"),
            "-name", "robot_pedestal",
            "-x", "%.6f" % float(pedestal["x"]),
            "-y", "%.6f" % float(pedestal["y"]),
            "-z", "%.6f" % float(pedestal["z"]),
            "-R", "%.6f" % float(pedestal["R"]),
            "-P", "%.6f" % float(pedestal["P"]),
            "-Y", "%.6f" % float(pedestal["Y"]),
        ],
    )

    jsb = Node(
        package="controller_manager", executable="spawner", output="screen",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    arm = Node(
        package="controller_manager", executable="spawner", output="screen",
        arguments=["elfin_arm_controller", "--controller-manager", "/controller_manager"],
    )

    return [
        gz_sim,
        clock_bridge,
        image_bridge,
        depth_bridge,
        depth_republisher,
        _robot_state_publisher(scene_tf_config),
        pedestal_spawner,
        RegisterEventHandler(OnProcessExit(target_action=pedestal_spawner, on_exit=[robot_spawner])),
        jsb,
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "scene_tf_config",
            default_value=os.path.join(
                get_package_share_directory("luggage_description"),
                "config", "scene_tf.yaml.example"),
        ),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", _resource_path()),
        SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", _resource_path()),
        OpaqueFunction(function=_launch_setup),
    ])
