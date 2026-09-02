"""Jazzy real-robot execution graph. No Gazebo, no MoveIt, no mock joints.

The only motion server is the Huayan TCP executor. Planning / test clients
send FollowJointTrajectory and must wait for the action result before the
next step.

Do not combine this launch with:
  - luggage_gazebo/sim_world.launch.py
  - zero_joint_state_publisher
  - elfin_ros_control / EtherCAT
  - a Humble overlay sourced in the same shell
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_ACTION_NAME = "/elfin_arm_controller/follow_joint_trajectory"


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("elfin_trajectory_executor")
    params_file = os.path.join(pkg_share, "config", "executor.yaml")

    robot_ip = LaunchConfiguration("robot_ip")
    robot_port = LaunchConfiguration("robot_port")
    default_vel = LaunchConfiguration("default_velocity_deg")
    max_vel = LaunchConfiguration("max_velocity_deg")
    action_name = LaunchConfiguration("action_name")

    executor_node = Node(
        package="elfin_trajectory_executor",
        executable="trajectory_executor",
        name="trajectory_executor",
        output="screen",
        parameters=[
            params_file,
            {
                "mode": "real",
                "use_sim_time": False,
                "robot_ip": robot_ip,
                "robot_port": robot_port,
                "default_velocity_deg": default_vel,
                "max_velocity_deg": max_vel,
                "action_name": action_name,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_ip",
                default_value=os.environ.get("ROBOT_IP", "192.168.0.10"),
                description="Huayan controller TCP address (or $ROBOT_IP).",
            ),
            DeclareLaunchArgument(
                "robot_port",
                default_value=os.environ.get("ROBOT_PORT", "10003"),
                description="Huayan CPS TCP port.",
            ),
            DeclareLaunchArgument(
                "default_velocity_deg",
                default_value="10.0",
                description="Fallback joint speed (deg/s). First bring-up uses 10.",
            ),
            DeclareLaunchArgument(
                "max_velocity_deg",
                default_value="20.0",
                description="Hard clamp on waypoint speed (deg/s).",
            ),
            DeclareLaunchArgument(
                "action_name",
                default_value=DEFAULT_ACTION_NAME,
                description=(
                    "FollowJointTrajectory action. Must match the planner "
                    "(/elfin_arm_controller/follow_joint_trajectory)."
                ),
            ),
            LogInfo(
                msg=[
                    "[jazzy_real] No Gazebo. Executor action=",
                    action_name,
                    " ip=",
                    robot_ip,
                    ":",
                    robot_port,
                    ". Next step only after FJT SUCCEEDED.",
                ]
            ),
            executor_node,
        ]
    )
