"""Launch only the scene static-TF publisher."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_scene = PathJoinSubstitution(
        [
            FindPackageShare("luggage_description"),
            "config",
            "scene_tf.yaml.example",
        ]
    )
    scene_tf_config = LaunchConfiguration("scene_tf_config")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scene_tf_config",
                default_value=default_scene,
            ),
            Node(
                package="luggage_description",
                executable="container_tf_publisher",
                name="container_tf_publisher",
                output="screen",
                parameters=[
                    {
                        "scene_tf_config": scene_tf_config,
                        "republish_period": 0.0,
                    }
                ],
            ),
        ]
    )
