#!/usr/bin/env python3
"""Load exploration.yaml and normalize exploration planner settings."""

from __future__ import division

import os

import yaml

from luggage_description._share import description_config_path


def default_exploration_path():
    return description_config_path("exploration.yaml.example")


def load_exploration_config(path=None):
    path = path or default_exploration_path()
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def exploration_joint_names(config):
    return list(config.get("joint_names", [
        "elfin_joint1", "elfin_joint2", "elfin_joint3",
        "elfin_joint4", "elfin_joint5", "elfin_joint6",
    ]))


def view_planning_constraints(config):
    vp = config.get("view_planning", {})
    return {
        "camera_z_max": float(vp.get("camera_z_max", 1.45)),
        "wrist_z_max": float(vp.get("wrist_z_max", 1.55)),
        "coverage_radius": float(vp.get("coverage_radius", 0.9)),
        "alignment_min": float(vp.get("alignment_min", 0.2)),
    }
def smart_explore_config(config):
    """Return the smart_explore config block with defaults applied.

    Structure:
        {
            "enabled": bool,
            "arm_reach": float,
            "phase0": {num_views, arc_radius, height_above_opening, max_tilt_deg,
                       min_improvement, stagnation_limit, min_inside_fov},
            "phase1": {lateral_steps, height_steps, standoff_values,
                       max_tilt_deg, look_depth_ratio},
            "ik": {timeout, attempts, avoid_collisions},
            "termination": {unknown_threshold, max_views},
        }
    """
    se = config.get("smart_explore", {}) or {}
    phase0 = se.get("phase0", {}) or {}
    phase1 = se.get("phase1", {}) or {}
    ik = se.get("ik", {}) or {}
    term = se.get("termination", {}) or {}
    return {
        "enabled": bool(se.get("enabled", True)),
        "arm_reach": float(se.get("arm_reach", 1.6)),
        "phase0": {
            "num_views": int(phase0.get("num_views", 3)),
            "arc_radius": float(phase0.get("arc_radius", 0.30)),
            "height_above_opening": float(phase0.get("height_above_opening", 0.60)),
            "max_tilt_deg": float(phase0.get("max_tilt_deg", 45.0)),
            # Marginal-gain early stop: skip remaining phase0 views once
            # improvement stagnates (never before at least one view is used).
            "min_improvement": float(phase0.get("min_improvement", 0.01)),
            "stagnation_limit": int(phase0.get("stagnation_limit", 1)),
            # FOV early stop: skip an about-to-run phase0 candidate (and the
            # rest) once too little of its field of view actually falls
            # inside the container. Defaults to 0.0 (never skips -- any
            # ratio is >= 0.0) until calibrated against real
            # inside_container_fov_ratio values from a live run; see
            # smart_explore_termination.phase0_low_fov.
            "min_inside_fov": float(phase0.get("min_inside_fov", 0.0)),
        },
        "phase1": {
            "lateral_steps": int(phase1.get("lateral_steps", 3)),
            "height_steps": int(phase1.get("height_steps", 3)),
            "standoff_values": [
                float(v) for v in phase1.get("standoff_values", [0.05, 0.15, 0.30])
            ],
            "max_tilt_deg": float(phase1.get("max_tilt_deg", 60.0)),
            "look_depth_ratio": float(phase1.get("look_depth_ratio", 0.3)),
            # When true, phase1 uses the camera-down interior geometry and the
            # planner overrides each candidate's orientation with the TF-derived
            # suction-down camera quaternion (see downward_constraint_utils).
            "camera_down_mode": bool(phase1.get("camera_down_mode", False)),
        },
        "ik": {
            "timeout": float(ik.get("timeout", 0.2)),
            "attempts": int(ik.get("attempts", 3)),
            "avoid_collisions": bool(ik.get("avoid_collisions", True)),
        },
        "termination": {
            "unknown_threshold": float(term.get("unknown_threshold", 0.15)),
            "max_views": int(term.get("max_views", 12)),
        },
    }


def downward_constraints_config(config):
    """Return the downward_constraints config block with defaults applied.

    Governs strict downward planning. ``primary_constraint`` selects the single
    rigid-frame path constraint (normally suction); both camera and suction are
    still checked by per-point FK when ``validate_trajectory`` is true.

    Structure:
        {
            "enabled": bool,
            "camera_max_tilt_deg": float,
            "suction_max_tilt_deg": float,
            "free_yaw": bool,
            "strict": bool,
            "validate_trajectory": bool,
            "candidate_min_separation_m": float,
            "primary_constraint": str,
            "align_before_probe": bool,
        }
    """
    dc = config.get("downward_constraints", {}) or {}
    return {
        "enabled": bool(dc.get("enabled", False)),
        "camera_max_tilt_deg": float(dc.get("camera_max_tilt_deg", 15.0)),
        "suction_max_tilt_deg": float(dc.get("suction_max_tilt_deg", 5.0)),
        "free_yaw": bool(dc.get("free_yaw", True)),
        "strict": bool(dc.get("strict", False)),
        "validate_trajectory": bool(dc.get("validate_trajectory", True)),
        "candidate_min_separation_m": float(dc.get("candidate_min_separation_m", 0.08)),
        "primary_constraint": str(dc.get("primary_constraint", "suction")),
        "align_before_probe": bool(dc.get("align_before_probe", True)),
    }


def interior_probe_config(config):
    """Return geometry, IK and termination settings for interior_probe mode."""
    probe = config.get("interior_probe", {}) or {}
    ik = probe.get("ik", {}) or {}
    term = probe.get("termination", {}) or {}
    return {
        "enabled": bool(probe.get("enabled", True)),
        "lateral_steps": int(probe.get("lateral_steps", 3)),
        "depth_steps": int(probe.get("depth_steps", 3)),
        "camera_z": (
            None if probe.get("camera_z", None) is None
            else float(probe.get("camera_z"))
        ),
        "depth_min_from_opening": float(
            probe.get("depth_min_from_opening", 0.20)
        ),
        "depth_max_ratio": float(probe.get("depth_max_ratio", 0.75)),
        "wall_clearance": float(probe.get("wall_clearance", 0.15)),
        "opening_clearance": float(probe.get("opening_clearance", 0.20)),
        "aperture_margin": float(probe.get("aperture_margin", 0.12)),
        "look_down": float(probe.get("look_down", 0.80)),
        "coverage_radius": float(probe.get("coverage_radius", 0.65)),
        "min_improvement": float(term.get("min_improvement", 0.005)),
        "stagnation_limit": int(term.get("stagnation_limit", 2)),
        "ik": {
            "timeout": float(ik.get("timeout", 0.5)),
            "attempts": int(ik.get("attempts", 5)),
            "avoid_collisions": bool(ik.get("avoid_collisions", True)),
        },
        "termination": {
            "unknown_threshold": float(term.get("unknown_threshold", 0.15)),
            "max_views": int(term.get("max_views", 9)),
        },
    }
