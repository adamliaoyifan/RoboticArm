from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hardware_plugin = LaunchConfiguration("hardware_plugin")
    use_rviz = LaunchConfiguration("use_rviz")

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    PathJoinSubstitution(
                        [FindPackageShare("elfin_description"), "urdf", "S20.urdf.xacro"]
                    ),
                    " hardware_plugin:=",
                    hardware_plugin,
                ]
            ),
            value_type=str,
        )
    }
    controllers = PathJoinSubstitution(
        [FindPackageShare("elfin_control"), "config", "elfin_controllers.yaml"]
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers],
        remappings=[("~/robot_description", "/robot_description")],
        output="screen",
    )
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        output="screen",
    )
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "elfin_arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        output="screen",
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
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hardware_plugin",
                default_value="mock_components/GenericSystem",
            ),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            rsp,
            control_node,
            jsb_spawner,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=jsb_spawner,
                    on_exit=[arm_spawner],
                )
            ),
            rviz,
        ]
    )
