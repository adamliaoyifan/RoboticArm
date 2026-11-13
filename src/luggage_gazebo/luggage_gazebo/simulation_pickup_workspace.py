"""Bind simulated pickup acceptance to the selected static platform geometry.

The platform is known scene geometry, never a source of cargo dimensions or
occupancy. The segmenter's existing radial test and configured margin remain
unchanged. Hardware launches keep their own measured-input contract.
"""
import math

from launch_ros.actions import Node
from luggage_description.scene_tf_config_utils import (
    load_scene_tf_config,
    pickup_platform_config,
    pickup_platform_top_in_world,
)


def pickup_workspace_parameters(scene):
    platform = pickup_platform_config(scene)
    if not platform['enabled']:
        return {'workspace_accept_enabled': False}
    # The existing predicate intersects a world-horizontal plane. Do not
    # silently project a tilted platform onto a different acceptance region.
    roll, pitch, _yaw = platform['rotation_rpy']
    if abs(math.sin(roll)) > 1e-9 or abs(math.sin(pitch)) > 1e-9 \
            or math.cos(roll) * math.cos(pitch) < 0:
        raise ValueError('SIM_PICKUP_WORKSPACE_REQUIRES_HORIZONTAL_PLATFORM')
    position, _rotation = pickup_platform_top_in_world(scene)
    return {
        'workspace_accept_enabled': True,
        'workspace_center_xy': position[:2],
        'workspace_half_extents': [value / 2. for value in platform['size'][:2]],
        'workspace_plane_z': position[2],
        'workspace_world_frame': str(scene.get('world_frame', 'world')),
    }


def simulation_segmenter_node(*, scene_config, semantic_config, model_path,
                              require_backend, environment, condition):
    """Construct the actual segmenter Node with scene overrides after YAML."""
    workspace = pickup_workspace_parameters(load_scene_tf_config(scene_config))
    return Node(
        package='luggage_perception', executable='semantic_segmenter_node.py',
        name='semantic_segmenter', output='screen',
        additional_env=environment, condition=condition,
        parameters=[semantic_config, {
            'use_sim_time': True, 'require_backend': require_backend,
            'model_name': model_path, **workspace,
        }],
    )
