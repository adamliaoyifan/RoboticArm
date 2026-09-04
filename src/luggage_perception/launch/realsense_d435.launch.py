"""Real D435 driver. No Gazebo.

Requires ros-jazzy-realsense2-camera. Aligns depth to color and uses the
optical frame names expected by the preprocessor.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    align = LaunchConfiguration("align_depth")
    serial = LaunchConfiguration("serial_no")

    rs_launch = PathJoinSubstitution(
        [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("align_depth", default_value="true"),
            DeclareLaunchArgument(
                "serial_no",
                default_value="",
                description="Empty = first D435 the driver finds.",
            ),
            LogInfo(msg="[realsense] No Gazebo. Expect /camera/color and /camera/depth."),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rs_launch),
                launch_arguments={
                    "align_depth.enable": align,
                    "pointcloud.enable": "true",
                    "enable_sync": "true",
                    "serial_no": serial,
                    # Empty namespace -> /camera/color/image_raw (preprocessor names).
                    "camera_namespace": "",
                    "camera_name": "camera",
                }.items(),
            ),
        ]
    )
