"""
sim.launch.py
~~~~~~~~~~~~~
Launch the trajectory executor in pure-software simulation mode together
with robot_state_publisher so that RViz2 on any Humble machine can
visualise live joint motion.

Usage:
    ros2 launch elfin_trajectory_executor sim.launch.py

Optional overrides:
    ros2 launch elfin_trajectory_executor sim.launch.py \
        urdf_file:=/path/to/S20.urdf
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('elfin_trajectory_executor')
    params_file = os.path.join(pkg_share, 'config', 'executor.yaml')

    # ----------------------------------------------------------------
    # Declare arguments
    # ----------------------------------------------------------------
    urdf_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value='',
        description=(
            'Absolute path to the Elfin S20 URDF/xacro file for '
            'robot_state_publisher.  Leave empty to skip RSP.'
        ),
    )

    # ----------------------------------------------------------------
    # Trajectory executor node (sim mode)
    # ----------------------------------------------------------------
    executor_node = Node(
        package='elfin_trajectory_executor',
        executable='trajectory_executor',
        name='trajectory_executor',
        output='screen',
        parameters=[
            params_file,
            {'mode': 'sim'},       # force simulation regardless of yaml
        ],
    )

    # ----------------------------------------------------------------
    # robot_state_publisher (optional — requires a URDF)
    # ----------------------------------------------------------------
    # Uncomment and set urdf_file:= to enable RSP for RViz TF.
    #
    # rsp_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[{
    #         'robot_description': Command([
    #             FindExecutable(name='xacro'), ' ',
    #             LaunchConfiguration('urdf_file'),
    #         ])
    #     }],
    # )

    return LaunchDescription([
        urdf_arg,
        LogInfo(msg='[sim.launch] Starting trajectory executor in SIM mode.'),
        executor_node,
        # rsp_node,   # uncomment when URDF is available
    ])
