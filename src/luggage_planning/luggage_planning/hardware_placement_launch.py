"""Placement service providers for the real-cell launch (no execution clients).

The caller supplies the same frame identity used by the pick chain. The mapper
is the sole OCC owner; the prior sidecar is a separate read-only consumer.
"""
from launch_ros.actions import Node


def placement_nodes(*, frame, scene, atlas, poses):
    common = {"use_sim_time": False, "scene_tf_config": scene, **frame}
    return [
        Node(package="luggage_perception", executable="cargo_volume_mapper_node.py",
             name="cargo_volume_mapper", output="screen", parameters=[{
                 **common, "base_frame": "container_link",
                 "cargo_cloud_topic": "/luggage/semantic/cargo_points_untracked",
                 "cloud_buffer_size": 12, "freshness_window_sec": 0.20,
                 "require_confirmed_map": True,
             }]),
        Node(package="luggage_packing", executable="placement_planner_node.py",
             name="placement_planner", output="screen", parameters=[{
                 **common, "atlas_runtime_config": atlas,
             }]),
        Node(package="luggage_planning", executable="trajectory_selector_node.py",
             name="trajectory_selector", output="screen", parameters=[{
                 **common, "atlas_runtime_config": atlas,
                 "robot_poses_config": poses, "corridor_planner": "graph",
             }]),
    ]
