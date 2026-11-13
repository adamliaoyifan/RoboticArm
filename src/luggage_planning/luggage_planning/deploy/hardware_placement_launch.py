"""Placement service providers for the real-cell launch (no execution clients).

The caller supplies the same frame identity used by the pick chain. The mapper
is the sole OCC owner; the prior sidecar is a separate read-only consumer.
"""
from launch_ros.actions import Node
from luggage_mapping.configuration import MapOwnerConfiguration


def placement_nodes(*, frame, scene, atlas, poses, owner,
                    observer_input_mode='service', observer_cloud_topic='',
                    observer_optical_frame=''):
    if type(owner) is not MapOwnerConfiguration:
        raise ValueError('Explicit map-owner configuration required')
    resolution = owner.resolution_m
    common = {"use_sim_time": False, "scene_tf_config": scene, **frame}
    if owner.backend == 'observer':
        if observer_input_mode not in ('service', 'subscription'):
            raise ValueError('observer_input_mode must be service or subscription')
        if observer_input_mode == 'subscription' and not (observer_cloud_topic and observer_optical_frame):
            raise ValueError('Optical cloud topic and frame are required for O subscription')
        if not owner.geometry_file or not owner.observer_config_file:
            raise ValueError('O owner requires explicit frozen geometry/configuration')
        producer = Node(package='uld_occ_observer', executable='observer_node',
            name='uld_occ_observer', output='screen', parameters=[{
                'use_sim_time': False, 'geometry_file': owner.geometry_file,
                'config_file': owner.observer_config_file,
                'input_mode': observer_input_mode, 'cloud_topic': observer_cloud_topic,
                'optical_frame': observer_optical_frame}])
        consumer = dict(volume_evidence_source='observer', validity_backend='observer_o',
            validity_topic='/uld_occ_observer/validity',
            validity_status_service='/uld_occ_observer/get_validity',
            volume_observer_geometry_file=owner.geometry_file,
            volume_observer_config_file=owner.observer_config_file)
    elif owner.backend == 'grid_g':
        producer = Node(package="luggage_perception", executable="cargo_volume_mapper_node.py",
             name="cargo_volume_mapper", output="screen", parameters=[{
                 **common, "base_frame": "container_link", "resolution": resolution,
                 "cargo_cloud_topic": "/luggage/semantic/cargo_points_untracked",
                 "cloud_buffer_size": 12, "freshness_window_sec": 0.20,
                 "require_confirmed_map": True,
             }])
        consumer = dict(volume_evidence_source='grid_g', validity_backend='grid_g',
            validity_topic='/cargo_map/validity',
            validity_status_service='/cargo_map/get_validity', volume_grid_g_cells=True)
    else:
        raise ValueError('Unknown map owner backend')
    return [producer,
        Node(package="luggage_packing", executable="placement_planner_node.py",
             name="placement_planner", output="screen", parameters=[{
                 **common, "atlas_runtime_config": atlas, "floor_prior_resolution": resolution,
                 **consumer,
             }]),
        Node(package="luggage_planning", executable="trajectory_selector_node.py",
             name="trajectory_selector", output="screen", parameters=[{
                 **common, "atlas_runtime_config": atlas,
                 "robot_poses_config": poses, "corridor_planner": "graph",
             }]),
    ]
