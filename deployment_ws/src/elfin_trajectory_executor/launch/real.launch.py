"""
real.launch.py
~~~~~~~~~~~~~~
Thin wrapper around jazzy_real.launch.py. Same graph: Huayan TCP executor,
no Gazebo. Kept so existing commands keep working.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("elfin_trajectory_executor")
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, "launch", "jazzy_real.launch.py")
                )
            )
        ]
    )
