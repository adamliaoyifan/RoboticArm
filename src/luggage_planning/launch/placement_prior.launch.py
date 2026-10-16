"""Read-only prior sidecar; OCC and MoveIt remain externally owned.

No robot execution, perception trigger, map reset or scene update is started.
The frame revision must match the running production planning stack.
"""
import os
import json
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    from luggage_description.scene_tf_config_utils import load_scene_tf_config, origin_in_world
    from luggage_description.scene_tf_publisher import rpy_to_quaternion
    from luggage_description.xacro_robot_with_scene_base import expand_and_patch
    revision = LaunchConfiguration('frame_scene_revision').perform(context)
    scene_path = LaunchConfiguration('scene_tf_config').perform(context)
    if not revision:
        raise ValueError('placement_prior requires the running frame_scene_revision')
    share = get_package_share_directory('luggage_description')
    xml, _ = expand_and_patch(os.path.join(share, 'urdf', 'elfin_s20_with_camera.urdf.xacro'),
                              config_path=scene_path, xacro_args=[])
    with open(os.path.join(share, 'config', 'S20_with_camera.srdf')) as stream:
        semantic = stream.read().replace('name="elfin_s20_with_camera"', 'name="S20"', 1)
    translation, rpy = origin_in_world(load_scene_tf_config(scene_path))
    model = {'robot_description': xml, 'robot_description_semantic': semantic}
    return [
        Node(package='luggage_planning', executable='placement_tool_check_node',
             name='placement_tool_check', output='screen', parameters=[model]),
        Node(package='luggage_planning', executable='place_prior_node.py', name='place_prior',
             output='screen', parameters=[model, {
                 'frame_scene_revision': revision,
                 'container_translation': list(map(float, translation)),
                 'container_quaternion_xyzw': list(map(float, rpy_to_quaternion(rpy))),
                 'collision_margin_m': float(LaunchConfiguration('collision_margin_m').perform(context)),
                 'map_parameter_service': LaunchConfiguration('map_parameter_service').perform(context),
                 'prefetch_enabled': LaunchConfiguration('prefetch_enabled').perform(context).lower()=='true',
                 'prefetch_box_wdh': [float(v) for v in json.loads(LaunchConfiguration('prefetch_box_wdh').perform(context))],
                 'prefetch_grasps_flat': [float(v) for v in json.loads(LaunchConfiguration('prefetch_grasps_flat').perform(context))],
             }]),
    ]


def generate_launch_description():
    share = get_package_share_directory('luggage_description')
    return LaunchDescription([
        DeclareLaunchArgument('scene_tf_config', default_value=os.path.join(share, 'config', 'scene_tf.yaml')),
        DeclareLaunchArgument('frame_scene_revision', default_value=''),
        DeclareLaunchArgument('collision_margin_m', default_value='0.05'),
        DeclareLaunchArgument('map_parameter_service', default_value='/cargo_volume_mapper/get_parameters'),
        DeclareLaunchArgument('prefetch_enabled', default_value='false'),
        DeclareLaunchArgument('prefetch_box_wdh', default_value='[0,0,0]'),
        DeclareLaunchArgument('prefetch_grasps_flat', default_value='[0,0,0,0,0,0,0]'),
        OpaqueFunction(function=_nodes),
    ])
