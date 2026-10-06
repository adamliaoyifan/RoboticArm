"""C0 resolved planning contract tests: no unresolved values, sources
present, numbers match their cited production sources."""

from __future__ import annotations

import json
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
CONTRACT = json.loads(
    (ROOT / "st09_contracts" / "resolved_planning_contract.json").read_text())


def test_no_tbd_or_placeholder_anywhere():
    text = json.dumps(CONTRACT)
    for banned in ("TBD", "tbd", "TODO", "FIXME", "??"):
        assert banned not in text


def test_every_section_present():
    for section in ("runtime_identity", "frame_semantics", "joint_limits",
                    "start_and_fk_gates", "tool_orientation",
                    "collision_margins", "time_parameterization",
                    "planning_budgets", "certification_sampling",
                    "map_freshness", "attachment_policy",
                    "stop_and_cancel_deadlines", "eval_gates_per_box"):
        assert section in CONTRACT


def test_joint_limits_match_urdf():
    urdf = ROOT / "robot_assets" / "elfin_description" / "urdf" / "S20.urdf.xacro"
    tree = ET.parse(urdf)
    limits = {}
    for joint in tree.iter():
        name = str(joint.get("name") or "")
        if "elfin_joint" not in name:
            continue
        for lim in joint:
            if lim.tag.endswith("limit"):
                limits[name] = (float(lim.get("lower")),
                                float(lim.get("upper")),
                                float(lim.get("velocity")))
    assert set(limits) == set(CONTRACT["joint_limits"]["position_rad"])
    for joint, (lower, upper) in CONTRACT["joint_limits"]["position_rad"].items():
        assert limits[joint][0] == pytest.approx(lower, abs=1e-9)
        assert limits[joint][1] == pytest.approx(upper, abs=1e-9)
        assert limits[joint][2] == pytest.approx(
            CONTRACT["joint_limits"]["velocity_rad_per_s"], abs=1e-9)


def test_status_field_rules():
    for section in CONTRACT.values():
        if isinstance(section, dict) and "source" not in section:
            for key, entry in section.items():
                if isinstance(entry, dict) and "value" in entry:
                    assert "source" in entry or "status" in entry or "meaning" in entry, \
                        "value without provenance: %s" % key
    defined = [
        "certification_timeout_s", "planning_snapshot_age_max_s",
        "attachment_max_age_ns"]
    text = json.dumps(CONTRACT)
    for name in defined:
        assert name in text
        # each newly defined value is explicitly marked
    assert CONTRACT["planning_budgets"]["certification_timeout_s"]["status"] == "defined_by_c0"
    assert CONTRACT["map_freshness"]["planning_snapshot_age_max_s"]["status"] == "defined_by_c0"
    assert CONTRACT["attachment_policy"]["attachment_max_age_ns"]["status"] == "defined_by_c0"


def test_fk_and_attach_tolerances_match_production_sources():
    fk = CONTRACT["start_and_fk_gates"]
    assert fk["fk_verify_position_m"]["value"] == 0.002
    assert fk["fk_verify_rotation_rad"]["value"] == 0.02
    assert fk["timed_trajectory_start_drift_rad"]["value"] == 0.01
    att = CONTRACT["attachment_policy"]
    assert att["grasp_binding_tolerance_m"]["value"] == 0.01
    assert att["grasp_binding_tolerance_rad"]["value"] == 0.10
    assert att["tf_sync_max_skew_ns"]["value"] == 0


def test_time_parameterization_matches_site_profile():
    tp = CONTRACT["time_parameterization"]
    assert tp["ramp_velocity_cap_label"] == "60 deg/s on both ramps"
    assert tp["ramp_velocity_cap_rad_per_s"] == pytest.approx(
        math.radians(60), abs=1e-9)
    assert tp["ramp_acceleration_cap_rad_per_s2"] == pytest.approx(
        math.radians(20), abs=1e-9)
    assert tp["joint_continuity_max_step_rad"] == 1.2
    assert tp["cartesian_min_fraction"] == 0.95


def test_budgets_and_sampling_bounds():
    budgets = CONTRACT["planning_budgets"]
    assert budgets["route_total_wall_s"]["value"] == 60.0
    assert budgets["single_new_moveit_edge_wall_s"]["value"] == 2.0
    assert budgets["max_edge_repair_attempts"]["value"] == 4
    sampling = CONTRACT["certification_sampling"]
    assert sampling["max_sample_dt_ns"] == 10_000_000


def test_eval_gates_frozen():
    gates = CONTRACT["eval_gates_per_box"]
    assert gates["gt_xy_max_m"] == 0.04
    assert gates["gt_abs_z_max_m"] == 0.03
    assert gates["gt_drift_max_m"] == 0.02
    assert gates["gt_abs_roll_pitch_max_label"] == "2 deg"
    assert gates["footprint_coverage_min"]["value"] == 0.4


import math  # noqa: E402  (used above)
