#!/bin/bash
# Kill every ST09-I stack-related process on this host (probe stacks,
# conftest stacks, validators). Run from a FILE so pkill -f never
# matches the invoking shell's own command line.
for pat in \
    "graph_launch.py" \
    "cargo_volume_mapper_node" \
    "trajectory_selector_node" \
    "motion_planner_node" \
    "motion_occ_driver" \
    "motion_executor" \
    "move_group" \
    "ros2_control_node" \
    "controller_spawner" \
    "semantic_point_filter_node" \
    "container_tf_publisher" \
    "robot_state_publisher" \
    "validate_route_node" \
    "placement_planner_node" \
    "waypoint_generator" \
    "vacuum_controller" \
    "scene_manager_node" \
    "ign gazebo" \
    "gz sim" \
    "parameter_bridge" \
    "ros_gz_sim" \
    "pickup_box_spawner_node" \
    "sensor_preprocessor_node" \
    "luggage_detector_node" \
    "semantic_segmenter_node" \
    "depth_image_republisher" \
    "camera_info_alias" \
    "static_joint_state_publisher" \
    "m3_three_box_driver"; do
    pkill -9 -f "$pat" 2>/dev/null
done
sleep 2
exit 0
