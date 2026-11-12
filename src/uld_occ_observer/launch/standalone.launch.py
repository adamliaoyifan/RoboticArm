"""Independent observer only: no mapper, planning, Gazebo or robot control."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    default_config = str(Path(get_package_share_directory('uld_occ_observer')) / 'config/observer.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('geometry_file'),
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('input_mode', default_value='service'),
        DeclareLaunchArgument('optical_frame', default_value=''),
        DeclareLaunchArgument('cloud_topic', default_value='optical_points'),
        DeclareLaunchArgument('lidar_cloud_topic', default_value=''),
        DeclareLaunchArgument('lidar_frame', default_value='livox_frame'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='uld_occ_observer', executable='observer_node', output='screen', parameters=[{
            key: ParameterValue(LaunchConfiguration(key), value_type=str)
            for key in ('geometry_file','config_file','input_mode','optical_frame','cloud_topic','lidar_cloud_topic','lidar_frame')
        }, {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}]),
    ])
