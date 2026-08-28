from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hardware_plugin",
                default_value="mock_components/GenericSystem",
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description],
                output="screen",
            ),
            Node(
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
            ),
        ]
    )
