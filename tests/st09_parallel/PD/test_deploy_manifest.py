"""Versioned deploy identity manifest (ST09-PD deliverable 2).

Build/validate round-trip on a synthetic cell, then one named rejection
per tampered section hash, plus the structural guarantees (joint order,
group/tool link, semantics, atlas entry count).
"""

from __future__ import division

import copy

import pytest
import yaml

from luggage_description.deploy_manifest import (
    DEPLOY_MANIFEST_SCHEMA,
    build_deploy_manifest,
    manifest_identity,
    semantic_srdf_text,
    validate_deploy_manifest,
)
from luggage_description.frame_contracts import ContractError

from conftest import MANIFEST_PATH, assets_present, minimal_arm_urdf

SCENE_REL = "config/scene_tf_deploy.yaml"
URDF_REL = "data/reachability_atlas_deploy/robot_description.xml"
SRDF_REL = "config/S20_with_camera.srdf"
NPZ_REL = "data/reachability_atlas_deploy/s20_container_collision_aware.npz"
META_REL = "data/reachability_atlas_deploy/s20_container_collision_aware.yaml"
GRAPH_REL = "data/reachability_atlas_deploy/graphs/s20_deploy_empty.json"


@pytest.fixture
def cell(tmp_path):
    desc = tmp_path / "luggage_description"
    plan = tmp_path / "luggage_planning"
    data = plan / "data" / "reachability_atlas_deploy" / "graphs"
    data.mkdir(parents=True)
    (desc / "config").mkdir(parents=True)

    scene = desc / "config" / "scene_tf_deploy.yaml"
    scene.write_text(
        "world_frame: world\ncontainer:\n  inner_size: [1.0, 0.8, 0.8]\n"
        "  inner:\n    chamfer:\n"
        "      side: positive_y\n      floor_y: 0.0\n"
        "      wall_y: 0.4\n      wall_z: 1.2\n"
        "    floor_z: 0.5\n    ceiling_z: 1.3\n  opening_side: negative_x\n",
        encoding="utf-8")
    (desc / "config" / "S20_with_camera.srdf").write_text(
        '<robot name="elfin_s20_with_camera"><group name="elfin_arm"/>'
        '</robot>\n', encoding="utf-8")
    urdf = plan / "data" / "reachability_atlas_deploy" / \
        "robot_description.xml"
    urdf.write_text(minimal_arm_urdf(), encoding="utf-8")
    (plan / "data" / "reachability_atlas_deploy" /
     "s20_container_collision_aware.npz").write_bytes(b"npz")
    (plan / "data" / "reachability_atlas_deploy" /
     "s20_container_collision_aware.yaml").write_bytes(b"meta")
    (data / "s20_deploy_empty.json").write_text("{}", encoding="utf-8")

    def resolve(package, rel):
        root = {"luggage_description": desc,
                "luggage_planning": plan}[package]
        import os
        return os.path.join(str(root), str(rel))

    entries = [{
        "payload": None,
        "basename": "s20_container_collision_aware",
        "npz_path": str(plan / NPZ_REL),
        "npz_rel": NPZ_REL,
        "meta_path": str(plan / META_REL),
        "meta_rel": META_REL,
        "graph": {
            "graph_id": "s20_deploy_empty",
            "path": str(plan / GRAPH_REL),
            "rel": GRAPH_REL,
            "digest": "ab" * 32,
        },
    }]
    manifest = build_deploy_manifest(
        scene=(str(scene), SCENE_REL),
        urdf=(str(urdf), URDF_REL),
        srdf=(str(desc / "config" / "S20_with_camera.srdf"), SRDF_REL),
        atlas_entries=entries,
        resolve=resolve,
        provenance={"command": "pytest", "environment": "synthetic"},
        payload_envelopes=[])
    return {
        "desc": desc, "plan": plan, "resolve": resolve, "manifest": manifest,
        "scene": scene, "urdf": urdf,
    }


def test_build_and_validate_roundtrip(cell):
    again = validate_deploy_manifest(cell["manifest"], cell["resolve"])
    assert again == cell["manifest"]
    assert cell["manifest"]["schema"] == DEPLOY_MANIFEST_SCHEMA
    # Tool identity transform = FK(q=0) over the fixture chain:
    # t = Ry(pi/2) @ (0,0,0.75) + (0.1,0.2,0.3) = (0.85, 0.2, 0.3), and
    # R = Ry(pi/2) @ Rz(pi/2) is the 120-degree rotation about (1,1,1):
    # quaternion (0.5, 0.5, 0.5, 0.5).
    transform = cell["manifest"]["planning"]["tool_transform_identity"]
    assert transform["translation"] == pytest.approx(
        [0.85, 0.2, 0.3], abs=1e-6)
    assert transform["quaternion_xyzw"] == pytest.approx(
        [0.5, 0.5, 0.5, 0.5], abs=1e-6)
    # Semantic SRDF text is the final robot_description_semantic string.
    srdf_text = (cell["desc"] / "config" /
                 "S20_with_camera.srdf").read_text("utf-8")
    assert 'name="S20"' in semantic_srdf_text(srdf_text)


def test_scene_hash_tamper(cell):
    broken = copy.deepcopy(cell["manifest"])
    broken["scene"]["sha256"] = "0" * 64
    with pytest.raises(ContractError) as caught:
        validate_deploy_manifest(broken, cell["resolve"])
    assert caught.value.code == "HASH_MISMATCH"
    assert caught.value.field == "scene.sha256"


def test_urdf_byte_tamper(cell):
    broken = copy.deepcopy(cell["manifest"])
    broken["urdf"]["md5"] = "0" * 32
    with pytest.raises(ContractError) as caught:
        validate_deploy_manifest(broken, cell["resolve"])
    assert caught.value.field == "urdf.md5"


def test_srdf_hash_tamper(cell):
    broken = copy.deepcopy(cell["manifest"])
    broken["srdf"]["sha256"] = "1" * 64
    with pytest.raises(ContractError) as caught:
        validate_deploy_manifest(broken, cell["resolve"])
    assert caught.value.field == "srdf.sha256"


def test_atlas_npz_tamper(cell):
    broken = copy.deepcopy(cell["manifest"])
    broken["atlas"][0]["npz"]["sha256"] = "2" * 64
    with pytest.raises(ContractError) as caught:
        validate_deploy_manifest(broken, cell["resolve"])
    assert caught.value.field.endswith(".npz.sha256")


def test_graph_hash_tamper(cell):
    broken = copy.deepcopy(cell["manifest"])
    broken["atlas"][0]["graph"]["sha256"] = "3" * 64
    with pytest.raises(ContractError) as caught:
        validate_deploy_manifest(broken, cell["resolve"])
    assert caught.value.field.endswith(".graph.sha256")


def test_missing_file_rejected(cell):
    broken = copy.deepcopy(cell["manifest"])
    broken["atlas"][0]["graph"]["path"] = "data/reachability_atlas/" \
                                         "graphs/ghost.json"
    with pytest.raises(ContractError) as caught:
        validate_deploy_manifest(broken, cell["resolve"])
    assert caught.value.code == "MANIFEST_FILE_MISSING"


def test_joint_order_and_identity_pins(cell):
    broken = copy.deepcopy(cell["manifest"])
    broken["urdf"]["joint_names"] = ["elfin_joint1"] * 6
    with pytest.raises(ContractError) as caught:
        validate_deploy_manifest(broken, cell["resolve"])
    assert caught.value.code == "JOINT_ORDER_MISMATCH"

    broken = copy.deepcopy(cell["manifest"])
    broken["planning"]["tool_link"] = "tool0"
    with pytest.raises(ContractError) as caught:
        validate_deploy_manifest(broken, cell["resolve"])
    assert caught.value.code == "INVALID_MANIFEST"

    broken = copy.deepcopy(cell["manifest"])
    broken["semantics"]["orientation_semantics"] = "pi_folded"
    with pytest.raises(ContractError) as caught:
        validate_deploy_manifest(broken, cell["resolve"])
    assert caught.value.code == "MANIFEST_SEMANTICS_MISMATCH"


def test_atlas_entry_count(cell):
    broken = copy.deepcopy(cell["manifest"])
    broken["payload_envelopes"] = [[0.55, 0.40, 0.25]]
    with pytest.raises(ContractError) as caught:
        validate_deploy_manifest(broken, cell["resolve"])
    assert caught.value.code == "ATLAS_ENTRY_COUNT_MISMATCH"


def test_real_manifest_if_present():
    import os

    if not (assets_present() and os.path.isfile(MANIFEST_PATH)):
        pytest.skip("deploy manifest not generated yet")
    manifest = yaml.safe_load(MANIFEST_PATH.read_text("utf-8"))
    from luggage_planning.deploy_profile import default_resolver
    validated = validate_deploy_manifest(manifest, default_resolver())
    assert len(validated["atlas"]) == 4
    assert validated["urdf"]["joint_names"][0] == "elfin_joint1"
    assert manifest_identity(validated)
