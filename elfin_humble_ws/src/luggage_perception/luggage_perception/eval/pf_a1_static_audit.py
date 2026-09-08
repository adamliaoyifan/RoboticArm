#!/usr/bin/env python3
"""PF-A1 static privileged-input audit. No ROS.

Scans Humble online geometry nodes for GetCurrentBox / spawned-geometry
clients, and classifies remaining current_box / scene_tf / vacuum uses.
Does not edit PF-R3/PF-R4 implementation files.
"""

from __future__ import division

import os
import re

GETCURRENTBOX_RE = re.compile(r"\bGetCurrentBox\b")
GETCURRENTBOX_CODE_RE = re.compile(
    r"(import[^\n]*GetCurrentBox)|"
    r"(create_client\s*\(\s*GetCurrentBox)|"
    r"(ServiceProxy\s*\([^)]*GetCurrentBox)"
)
BOX_FROM_PAYLOAD_RE = re.compile(r"\bbox_from_current_box_payload\b")
PARSE_EPOCH_RE = re.compile(r"\bparse_current_box_payload\b")

# Online Humble nodes that estimate or consume pickup geometry.
ONLINE_GEOMETRY_NODES = (
    "src/luggage_perception/scripts/luggage_detector_node.py",
    "src/luggage_perception/scripts/semantic_segmenter_node.py",
    "src/luggage_perception/scripts/semantic_point_filter_node.py",
    "src/luggage_perception/scripts/sensor_preprocessor_node.py",
    "src/luggage_perception/scripts/cargo_volume_mapper_node.py",
    "src/luggage_planning/scripts/waypoint_generator_node.py",
    "src/luggage_packing/scripts/placement_planner_node.py",
)

# Sim attach backend: privileged box pose/size is expected here, not in
# the height estimator. Classified separately; does not fail PF-A1 height.
SIM_BACKEND_NODES = (
    "src/luggage_planning/scripts/vacuum_controller_node.py",
)

EVAL_PROVIDERS = (
    "src/luggage_gazebo/scripts/pickup_box_spawner_node.py",
    "scripts/platform_free_height_gate4_eval.py",
    "src/luggage_gazebo/scripts/pack_eval_driver.py",
    "src/luggage_gazebo/scripts/place_smoke_driver.py",
    "src/luggage_gazebo/scripts/pick_retreat_eval_driver.py",
    "src/luggage_perception/luggage_perception/eval/gate4_scoring.py",
    "src/luggage_perception/luggage_perception/eval/detection_gate_sampling.py",
    "src/luggage_perception/luggage_perception/eval/detection_accuracy.py",
)

ROS1_NOT_HUMBLE_ONLINE = (
    "src/luggage_bringup/scripts/orchestrator_node.py",
    "src/luggage_perception/scripts/ros1_reference/luggage_detector_node.py",
    "src/luggage_planning/scripts/ros1_reference/scene_manager_node.py",
    "src/luggage_packing/scripts/bin_packer_node.py",
    "src/luggage_planning/scripts/vacuum_simulator_node.py",
)

CLASS_SENSOR = "sensor_observation"
CLASS_CONFIG = "calibrated_deployment_config"
CLASS_TASK = "backend_neutral_task_lifecycle"
CLASS_PRIVILEGED = "privileged_simulation_truth"
CLASS_EVAL = "eval_only_reference"
CLASS_SIM_BACKEND = "sim_backend_only"


def _read(root, rel):
    path = os.path.join(root, rel)
    with open(path, "r") as handle:
        return handle.read()


def _line_hits(text, pattern):
    hits = []
    for index, line in enumerate(text.splitlines(), 1):
        if pattern.search(line):
            hits.append(index)
    return hits


def _online_getcurrentbox_lines(text):
    """Imports and service clients, not docstring mentions."""
    hits = []
    in_srv_import = False
    for index, line in enumerate(text.splitlines(), 1):
        stripped = line.strip()
        if stripped.startswith("#"):
            continue
        if "luggage_msgs.srv" in line and "import" in line:
            in_srv_import = "(" in line and ")" not in line
            if "GetCurrentBox" in line:
                hits.append(index)
        elif in_srv_import:
            if "GetCurrentBox" in line:
                hits.append(index)
            if ")" in stripped:
                in_srv_import = False
        elif GETCURRENTBOX_CODE_RE.search(line):
            hits.append(index)
    return hits


def audit_tree(root):
    """Return inventory + pass/fail for the height/geometry online path."""
    findings = []
    blockers = []

    for rel in ONLINE_GEOMETRY_NODES:
        text = _read(root, rel)
        gc_lines = _online_getcurrentbox_lines(text)
        box_geom = _line_hits(text, BOX_FROM_PAYLOAD_RE)
        epoch = _line_hits(text, PARSE_EPOCH_RE)
        if gc_lines:
            blockers.append({
                "id": "online_getcurrentbox",
                "file": rel,
                "lines": gc_lines,
                "detail": "GetCurrentBox import or client in online geometry node",
            })
        if box_geom:
            blockers.append({
                "id": "online_spawned_box_geometry",
                "file": rel,
                "lines": box_geom,
                "detail": "spawned pose/size parser in online geometry node",
            })
        findings.append({
            "file": rel,
            "layer": "online_geometry",
            "getcurrentbox_lines": gc_lines,
            "spawned_geometry_parser_lines": box_geom,
            "epoch_parser_lines": epoch,
        })

    vacuum_text = _read(root, SIM_BACKEND_NODES[0])
    vacuum_geom = _line_hits(vacuum_text, BOX_FROM_PAYLOAD_RE)
    vacuum_gc = _line_hits(vacuum_text, GETCURRENTBOX_RE)

    eval_hits = []
    for rel in EVAL_PROVIDERS:
        if not os.path.isfile(os.path.join(root, rel)):
            continue
        text = _read(root, rel)
        eval_hits.append({
            "file": rel,
            "getcurrentbox_lines": _line_hits(text, GETCURRENTBOX_RE),
        })

    ros1_hits = []
    for rel in ROS1_NOT_HUMBLE_ONLINE:
        if not os.path.isfile(os.path.join(root, rel)):
            continue
        text = _read(root, rel)
        ros1_hits.append({
            "file": rel,
            "getcurrentbox_lines": _line_hits(text, GETCURRENTBOX_RE),
        })

    inventory = _inventory_table(vacuum_geom, vacuum_gc)

    height_pass = not blockers
    return {
        "height_geometry_pass": height_pass,
        "blockers": blockers,
        "online_geometry_nodes": findings,
        "sim_backend": {
            "file": SIM_BACKEND_NODES[0],
            "spawned_geometry_parser_lines": vacuum_geom,
            "getcurrentbox_lines": vacuum_gc,
            "class": CLASS_SIM_BACKEND,
            "hardware_provider": (
                "none for pose/size/mass; hardware attach is future GPIO/"
                "pressure (stub backend). Does not feed detector/waypoint/"
                "placement geometry."),
        },
        "eval_providers": eval_hits,
        "ros1_not_humble_online": ros1_hits,
        "inventory": inventory,
        "task_state_hardware_provider": {
            "contract": (
                "std_msgs/String JSON on /luggage/current_box with id and "
                "generation only"),
            "perception_use": "epoch reset; pose/size/mass ignored",
            "hardware": (
                "topic may be omitted (generation stays 0, tracker still "
                "measures) or published by a task orchestrator without "
                "spawned geometry"),
            "credible": True,
        },
    }


def _inventory_table(vacuum_geom_lines, vacuum_gc_lines):
    """Source-to-consumer table required by PF-A1."""
    return [
        {
            "input": "/luggage/preprocessed/camera/* and depth/points",
            "class": CLASS_SENSOR,
            "sim_provider": "Gazebo rgbd_camera via ros_gz_bridge + preprocessor",
            "hardware_provider": "D435 (or equivalent) driver + preprocessor",
            "consumers": "semantic_segmenter, semantic_point_filter, luggage_detector",
            "feeds_online_geometry": True,
            "privileged": False,
        },
        {
            "input": "/joint_states",
            "class": CLASS_SENSOR,
            "sim_provider": "gz_ros2_control / observe_pose_hold",
            "hardware_provider": "elfin hardware joint state",
            "consumers": "sensor_preprocessor motion gate",
            "feeds_online_geometry": False,
            "privileged": False,
        },
        {
            "input": "/tf and /tf_static at acquisition stamp",
            "class": CLASS_SENSOR,
            "sim_provider": "robot_state_publisher + scene_tf_publisher",
            "hardware_provider": "same publishers with measured scene_tf.yaml",
            "consumers": "detector, semantic_point_filter (stamped lookup)",
            "feeds_online_geometry": True,
            "privileged": False,
            "residual": (
                "semantic_segmenter self-body mask still uses latest TF "
                "(rclpy.time.Time()); PF-R3 cargo/filter path is stamped"),
        },
        {
            "input": "scene_tf pickup_source XY / container dims",
            "class": CLASS_CONFIG,
            "sim_provider": "scene_tf.yaml.example",
            "hardware_provider": "measured scene_tf.yaml via scene_hardware.launch",
            "consumers": "detector workspace XY, placement/scene_manager container",
            "feeds_online_geometry": False,
            "privileged": False,
            "note": "pickup_source.z is not passed into the height pipeline",
        },
        {
            "input": "platform_z node parameter",
            "class": CLASS_CONFIG,
            "sim_provider": "omitted in sim_world.launch (support_mode=auto)",
            "hardware_provider": "optional measured prior; configured mode only",
            "consumers": "PlatformFreeDetector compose_box_geometry",
            "feeds_online_geometry": False,
            "privileged": False,
        },
        {
            "input": "/luggage/current_box id+generation",
            "class": CLASS_TASK,
            "sim_provider": "pickup_box_spawner_node",
            "hardware_provider": (
                "omit topic (generation=0) or orchestrator JSON without pose"),
            "consumers": "detector, semantic_segmenter, semantic_point_filter",
            "feeds_online_geometry": False,
            "privileged": False,
        },
        {
            "input": "/luggage/current_box pose/size/mass",
            "class": CLASS_PRIVILEGED,
            "sim_provider": "pickup_box_spawner_node JSON",
            "hardware_provider": "none (must not be required)",
            "consumers": (
                "vacuum_controller VacuumGate/SimVacuumBackend only; "
                "perception parsers ignore these fields"),
            "feeds_online_geometry": False,
            "privileged": True,
            "severity": "hardware_attach_followup",
            "lines": vacuum_geom_lines,
        },
        {
            "input": "GetCurrentBox / spawned model pose",
            "class": CLASS_EVAL,
            "sim_provider": "pickup_box_spawner_node service + gz spawn",
            "hardware_provider": "n/a (eval-only)",
            "consumers": "gate4_eval, pack/place/pick_retreat eval drivers",
            "feeds_online_geometry": False,
            "privileged": True,
            "getcurrentbox_in_vacuum": vacuum_gc_lines,
        },
        {
            "input": "box catalog sizes",
            "class": CLASS_CONFIG,
            "sim_provider": "box catalog YAML",
            "hardware_provider": "same catalog as deployment prior",
            "consumers": "compose_box_geometry HEIGHT_SOURCE_CATALOG_PRIOR",
            "feeds_online_geometry": False,
            "privileged": False,
            "note": "height_valid stays false for catalog prior",
        },
        {
            "input": "Gazebo /world/*/set_pose",
            "class": CLASS_SIM_BACKEND,
            "sim_provider": "ros_gz_interfaces SetEntityPose",
            "hardware_provider": "none; stub/hardware vacuum backend",
            "consumers": "vacuum_controller SimVacuumBackend",
            "feeds_online_geometry": False,
            "privileged": True,
            "severity": "sim_physics_follow",
        },
    ]


def write_report(report, path):
    import json
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, "w") as handle:
        json.dump(report, handle, indent=2, sort_keys=True)
        handle.write("\n")
    return path
