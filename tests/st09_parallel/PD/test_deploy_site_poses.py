"""Deploy site-pose schema and observation policy (ST09-PD deliverable 5).

Cases: the valid synthetic measured fixture, stale capture, wrong scene,
wrong joint order, simulation provenance, and the committed unmeasured
file (every entry valid:false, current the only executable fallback).
Only the synthetic measured fixture yields executable joints.
"""

from __future__ import division

import copy

import pytest
import yaml

from luggage_planning.deploy import site_pose_schema as sps
from luggage_planning.deploy.named_robot_poses import (
    default_observe_pose_name,
    load_poses_config,
    plan_goto_joints,
)

SCENE_REVISION = "98313fb8b30e51521a4c45a829ff6d5d"


def base_record():
    return {
        "schema": sps.SITE_POSE_SCHEMA,
        "identity": {
            "scene_revision": SCENE_REVISION,
            "joint_names": list(sps.JOINTS),
        },
        "poses": {kind: unmeasured() for kind in sps.SITE_POSE_KINDS},
        "defaults": {"observe_pose": "current"},
    }


def unmeasured():
    return {
        "valid": False,
        "calibration_status": "unmeasured",
        "values": [],
        "acquisition_stamp_ns": 0,
        "scene_revision_at_capture": "",
        "provenance": {"source": "unmeasured"},
    }


def measured(kind="pickup", scene=SCENE_REVISION):
    return {
        "valid": True,
        "calibration_status": "measured",
        "values": [0.10, -1.20, 1.40, 0.00, 1.50, 0.20],
        "acquisition_stamp_ns": 1_759_000_000_000_000_000,
        "scene_revision_at_capture": scene,
        "provenance": {"source": "pendant_dump", "operator": "site"},
    }


def validate(data, **kwargs):
    return sps.validate_site_poses(
        data, expected_scene_revision=SCENE_REVISION, **kwargs)


def test_committed_deploy_file_is_wellformed_but_unmeasured():
    from conftest import _REPO_ROOT

    path = (_REPO_ROOT / "src" / "luggage_description" / "config" /
            "robot_poses.deploy.yaml")
    data = sps.load_site_poses(str(path))
    report = validate(data)
    assert report["ok"], report["issues"]
    assert report["executable_fallback"] == "current"
    for kind in sps.SITE_POSE_KINDS:
        assert report["poses"][kind]["valid"] is False
        assert report["poses"][kind]["outcome"] == sps.NOT_EXECUTABLE
        assert sps.executable_site_pose(report, kind) is None
    # named_robot_poses compatibility: current means stay (no FJT).
    config = load_poses_config(str(path))
    assert default_observe_pose_name(config) == "current"
    assert plan_goto_joints("current", config, [0.0] * 6) is None


def test_synthetic_measured_fixture_executes():
    data = base_record()
    data["poses"]["pickup"] = measured()
    report = validate(data)
    assert report["ok"], report["issues"]
    assert report["poses"]["pickup"]["valid"] is True
    assert sps.executable_site_pose(report, "pickup") == \
        measured()["values"]
    # Unmeasured siblings stay non-executable on the same file.
    assert sps.executable_site_pose(report, "return") is None


def test_stale_capture_not_executable():
    data = base_record()
    stale = measured(scene="0" * 32)
    data["poses"]["return"] = stale
    report = validate(data)
    assert report["ok"] is False
    entry = report["poses"]["return"]
    assert entry["valid"] is False
    assert entry["outcome"] == sps.NOT_EXECUTABLE
    assert any("stale" in text for text in entry["issues"])
    assert sps.executable_site_pose(report, "return") is None


def test_wrong_scene_rejected():
    data = base_record()
    data["identity"]["scene_revision"] = "b" * 32
    report = validate(data)
    assert report["ok"] is False
    assert any("wrong scene" in text for text in report["issues"])
    # Nothing from a wrong-scene file may execute.
    data["poses"]["pickup"] = measured()
    report = validate(data)
    assert sps.executable_site_pose(report, "pickup") is None


def test_wrong_joint_order_rejected():
    data = base_record()
    joints = list(sps.JOINTS)
    joints[0], joints[1] = joints[1], joints[0]
    data["identity"]["joint_names"] = joints
    report = validate(data)
    assert report["ok"] is False
    assert any("joint order" in text for text in report["issues"])


def test_sim_provenance_rejected():
    data = base_record()
    sim = measured()
    sim["provenance"] = {
        "source": "gazebo", "note": "copied from robot_poses.yaml.example"}
    data["poses"]["portal_observation"] = sim
    report = validate(data)
    entry = report["poses"]["portal_observation"]
    assert entry["valid"] is False
    assert entry["outcome"] == sps.NOT_EVALUATED
    assert any("simulation provenance" in text for text in entry["issues"])
    # The file itself is a defect: sim provenance is not a deploy file.
    assert any("simulation provenance" in text for text in report["issues"])


def test_valid_flag_false_never_executes_even_with_values():
    data = base_record()
    lying = measured()
    lying["valid"] = False  # values present but not validated
    data["poses"]["second_region_observation"] = lying
    report = validate(data)
    assert report["ok"], report["issues"]
    assert report["poses"]["second_region_observation"]["valid"] is False
    assert sps.executable_site_pose(
        report, "second_region_observation") is None


def test_declared_model_pin_must_match():
    data = base_record()
    data["identity"]["urdf_sha256"] = "4" * 64
    report = validate(data, expected_urdf_sha256="5" * 64)
    assert any("urdf_sha256 mismatch" in text for text in report["issues"])
    # A matching pin stays silent.
    report = validate(data, expected_urdf_sha256="4" * 64)
    assert report["ok"], report["issues"]


def test_missing_entry_and_unknown_kind():
    data = base_record()
    del data["poses"]["pickup"]
    data["poses"]["hover"] = unmeasured()
    report = validate(data)
    assert report["poses"]["pickup"]["outcome"] == sps.NOT_EVALUATED
    assert any("unknown pose kind" in text for text in report["issues"])


# -- [ST09-I S5.7-6] SIM site-pose schema (GoToRobotPoseV2) --------------

def sim_record(scene=SCENE_REVISION, valid=True):
    def pose(values):
        return {
            "valid": bool(valid),
            "calibration_status": "synthetic",
            "values": list(values),
            "acquisition_stamp_ns": 1_759_000_000_000_000_000,
            "scene_revision_at_capture": scene,
            "provenance": {"source": "synthetic_fixture"},
        }

    return {
        "schema": sps.SIM_SITE_POSE_SCHEMA,
        "identity": {
            "scene_revision": scene,
            "joint_names": list(sps.JOINTS),
            "urdf_sha256": "",
            "srdf_sha256": "",
        },
        "poses": {
            "pickup_observe_sim": pose([1.8806, -1.7736, -1.0491, 4.4439,
                                        1.7721, 1.8473]),
            "portal_observe_sim": pose([0.7412, -1.8043, -1.0461, -1.8668,
                                        1.5749, -2.4370]),
        },
        "defaults": {"observe_pose": "pickup_observe_sim"},
    }


def test_sim_schema_accepts_synthetic_calibration():
    report = validate(sim_record(), schema=sps.SIM_SITE_POSE_SCHEMA)
    assert report["ok"], report["issues"]
    values = sps.executable_site_pose(report, "pickup_observe_sim")
    assert values is not None and len(values) == 6


def test_deploy_schema_still_rejects_synthetic_calibration():
    """Deploy strictness is unchanged: the same synthetic document under
    the DEPLOY schema (or without the schema switch) is refused."""
    report = validate(sim_record())
    assert not report["ok"]
    assert any("synthetic" in text or "schema is" in text
               for text in report["issues"])
    report = validate(sim_record(), schema=sps.SITE_POSE_SCHEMA)
    assert not report["ok"]


def test_sim_file_defaults_observe_pose_allowed_deploy_stays_current():
    """A SIM file names its observe pose; the deploy schema still
    demands current until every entry is measured."""
    report = validate(sim_record(), schema=sps.SIM_SITE_POSE_SCHEMA)
    assert report["ok"], report["issues"]
    data = base_record()
    data["defaults"]["observe_pose"] = "pickup"
    report = validate(data)
    assert any("defaults.observe_pose must stay" in text
               for text in report["issues"])


def test_sim_invalid_entry_refuses():
    """valid:false entries are the honest unmeasured state: the file
    stays well-formed, the pose is simply NOT executable."""
    report = validate(sim_record(valid=False),
                      schema=sps.SIM_SITE_POSE_SCHEMA)
    assert report["poses"]["pickup_observe_sim"]["valid"] is False
    assert sps.executable_site_pose(report, "pickup_observe_sim") is None


def test_checked_in_sim_deploy_yaml_validates_against_deploy_scene():
    """The committed sim_deploy.yaml is a well-formed SIM document whose
    identity pins md5(scene_tf_deploy.yaml) — the scene the S5 gazebo
    runner launches."""
    import hashlib
    import os

    root = os.path.normpath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
    yaml_path = os.path.join(
        root, "src", "luggage_description", "config",
        "robot_poses.sim_deploy.yaml")
    scene_path = os.path.join(
        root, "src", "luggage_description", "config", "scene_tf_deploy.yaml")
    data = yaml.safe_load(open(yaml_path, encoding="utf-8"))
    scene_md5 = hashlib.md5(
        open(scene_path, "rb").read()).hexdigest()
    assert data["schema"] == sps.SIM_SITE_POSE_SCHEMA
    # Sim poses pin md5(scene_tf_deploy.yaml). That file is the gazebo
    # scene, not the hardware cell hash in SCENE_REVISION.
    report = sps.validate_site_poses(
        data, expected_scene_revision=scene_md5,
        schema=sps.SIM_SITE_POSE_SCHEMA)
    assert report["ok"], report["issues"]
    assert data["identity"]["scene_revision"] == scene_md5
    # the named-residual discipline: urdf declared but EMPTY
    assert data["identity"]["urdf_sha256"] == ""
    assert data["identity"]["srdf_sha256"]
