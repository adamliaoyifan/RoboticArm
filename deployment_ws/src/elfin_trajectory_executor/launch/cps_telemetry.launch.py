"""CPS monitor only: joint angles, velocity, acceleration, TCP. No servo enable.

Publishes CPS ReadActACS onto /joint_states (rad, rad/s, effort=ampere) and
/elfin/joint_kinematics (includes rad/s^2). Do not combine with
jazzy_real.launch.py (one TCP client to the box).

  ros2 launch elfin_trajectory_executor cps_telemetry.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    robot_port = LaunchConfiguration("robot_port")
    rate_hz = LaunchConfiguration("rate_hz")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_ip",
                default_value=os.environ.get("ROBOT_IP", "192.168.0.10"),
            ),
            DeclareLaunchArgument(
                "robot_port",
                default_value=os.environ.get("ROBOT_PORT", "10003"),
            ),
            DeclareLaunchArgument("rate_hz", default_value="50.0"),
            LogInfo(
                msg=[
                    "[cps_telemetry] monitor ",
                    robot_ip,
                    ":",
                    robot_port,
                    " (no electrify / enable)",
                ]
            ),
            Node(
                package="elfin_trajectory_executor",
                executable="cps_telemetry",
                name="cps_telemetry",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": False,
                        "robot_ip": robot_ip,
                        "robot_port": robot_port,
                        "rate_hz": rate_hz,
                    }
                ],
            ),
        ]
    )
