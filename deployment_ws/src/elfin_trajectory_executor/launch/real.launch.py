"""
real.launch.py
~~~~~~~~~~~~~~
Launch the trajectory executor in real-hardware mode.

The node connects to the HuayanRobot controller via TCP/IP on startup,
runs the full enable sequence, and then waits for FollowJointTrajectory
action goals.

Usage (on Ubuntu 20.04 host or inside Docker):
    ros2 launch elfin_trajectory_executor real.launch.py

Override controller address:
    ros2 launch elfin_trajectory_executor real.launch.py \
        robot_ip:=192.168.1.50 robot_port:=10003

Note: ensure the CPS SDK (.so) is importable (set PYTHONPATH or use Docker).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('elfin_trajectory_executor')
    params_file = os.path.join(pkg_share, 'config', 'executor.yaml')

    # ----------------------------------------------------------------
    # Declare overridable arguments
    # ----------------------------------------------------------------
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.0.10',
        description='IP address of the HuayanRobot controller.',
    )
    robot_port_arg = DeclareLaunchArgument(
        'robot_port',
        default_value='10003',
        description='TCP port of the HuayanRobot controller.',
    )
    default_vel_arg = DeclareLaunchArgument(
        'default_velocity_deg',
        default_value='30.0',
        description='Default joint velocity (°/s) when trajectory lacks time hints.',
    )
    max_vel_arg = DeclareLaunchArgument(
        'max_velocity_deg',
        default_value='60.0',
        description='Maximum joint velocity (°/s).',
    )

    # ----------------------------------------------------------------
    # Trajectory executor node (real mode)
    # ----------------------------------------------------------------
    executor_node = Node(
        package='elfin_trajectory_executor',
        executable='trajectory_executor',
        name='trajectory_executor',
        output='screen',
        parameters=[
            params_file,
            {
                'mode': 'real',
                'robot_ip': LaunchConfiguration('robot_ip'),
                'robot_port': LaunchConfiguration('robot_port'),
                'default_velocity_deg': LaunchConfiguration('default_velocity_deg'),
                'max_velocity_deg': LaunchConfiguration('max_velocity_deg'),
            },
        ],
    )

    return LaunchDescription([
        robot_ip_arg,
        robot_port_arg,
        default_vel_arg,
        max_vel_arg,
        LogInfo(msg='[real.launch] Starting trajectory executor in REAL mode.'),
        executor_node,
    ])
