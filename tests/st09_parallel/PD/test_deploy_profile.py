"""Fail-closed deploy profile resolution (ST09-PD deliverable 1).

Four behavioural cases on a synthetic deploy tree -- default deploy
success, explicit deploy success, sim asset rejection, missing asset
rejection -- plus hash-mismatch, ambiguity and non-strict rejections,
and the same default/explicit pair against the real generated assets
when the campaign has run.
"""

from __future__ import division

import os
import shutil

import pytest
import yaml

from luggage_description.frame_contracts import ContractError
from conftest import minimal_arm_urdf
from luggage_planning import deploy_profile as dp


def write_deploy_tree(root, *, scene_bytes=None, payload_meta=None,
                      urdf_bytes=minimal_arm_urdf().encode("utf-8"), tamper=None):
    """Synthetic deploy cell: scene, atlas config, atlas pair, URDF,
    manifest -- all cross-consistent by construction."""
    import hashlib

    desc = root / "luggage_description"
    plan = root / "luggage_planning"
    (desc / "config").mkdir(parents=True)
    (plan / "config").mkdir(parents=True)
    (plan / "data" / "reachability_atlas_deploy" / "graphs").mkdir(parents=True)

    scene_bytes = scene_bytes if scene_bytes is not None else (
        b"# deploy scene fixture\nworld_frame: world\n")
    (desc / "config" / "scene_tf_deploy.yaml").write_bytes(scene_bytes)
    (desc / "config" / "scene_tf.yaml").write_bytes(b"# sim scene\n")
    (desc / "config" / "scene_tf.yaml.example").write_bytes(
        b"# sim scene (must be rejected on hardware)\n")

    scene_md5 = hashlib.md5(scene_bytes).hexdigest()
    urdf_md5 = hashlib.md5(urdf_bytes).hexdigest()

    (desc / "config" / "S20_with_camera.srdf").write_bytes(b"<robot/>")
    (plan / "config" / "atlas_runtime.yaml").write_text(
        "enabled: true\nhash_policy: strict\natlas_dir: \"\"\n",
        encoding="utf-8")
    cfg_text = {
        "enabled": True,
        "atlas_dir": "reachability_atlas_deploy",
        "basename": "s20_container_collision_aware",
        "atlas_file": "",
        "hash_policy": "strict",
        "query_offset": [0.0, 0.0, 0.0],
        "payload": {"policy": "auto", "explicit_size": [0.0, 0.0, 0.0]},
        "placement": {"hard_reject": True, "w_reachability": 0.3,
                      "dump_only": False},
        "motion": {"enabled": True, "seed_set": "transit", "max_seeds": 3,
                   "mode": "miss-retry", "fk_verify_tol_m": 0.25,
                   "submit_mode": "joint_goal"},
    }
    (plan / "config" / "atlas_runtime_deploy.yaml").write_text(
        yaml.safe_dump(cfg_text), encoding="utf-8")

    data = plan / "data" / "reachability_atlas_deploy"
    (data / "robot_description.xml").write_bytes(urdf_bytes)
    (data / "s20_container_collision_aware.npz").write_bytes(b"npz-bytes")
    meta = {
        "dependencies": {
            "robot_model": "elfin_s20_with_camera",
            "scene_tf_hash": scene_md5,
            "urdf_hash": urdf_md5,
            "ik_link": "suction_contact_frame",
            "ik_group": "elfin_arm",
        },
        "container": {
            "inner_dimensions": [1.0, 0.8, 0.6],
            "floor_z": 0.5,
            "ceiling_z": 1.3,
            "opening_side": "negative_x",
        },
        "grid": {"origin": [0.0, 0.0, 0.5], "size": [1, 1, 1, 4],
                 "resolution_xyz": 0.15},
        "payload": payload_meta or {"enabled": False, "shape": None,
                                    "size": None, "attach_link":
                                    "suction_contact_frame",
                                    "offset": None, "touch_links": None},
    }
    (data / "s20_container_collision_aware.yaml").write_text(
        yaml.safe_dump(meta), encoding="utf-8")
    (data / "graphs" / "s20_deploy_empty.json").write_text(
        '{"schema": "st09.route-graph/1"}', encoding="utf-8")

    if tamper:
        tamper(root, desc, plan, data)
    return root


def synthetic_manifest(root):
    """Build a real manifest for the synthetic tree via the builder."""
    from luggage_description.deploy_manifest import build_deploy_manifest

    desc = root / "luggage_description"
    plan = root / "luggage_planning"
    data = plan / "data" / "reachability_atlas_deploy"
    return build_deploy_manifest(
        scene=(str(desc / "config" / "scene_tf_deploy.yaml"),
               "config/scene_tf_deploy.yaml"),
        urdf=(str(data / "robot_description.xml"),
              "data/reachability_atlas_deploy/robot_description.xml"),
        srdf=(str(desc / "config" / "S20_with_camera.srdf"),
              "config/S20_with_camera.srdf"),
        atlas_entries=[{
            "payload": None,
            "basename": "s20_container_collision_aware",
            "npz_path": str(data / "s20_container_collision_aware.npz"),
            "npz_rel": "data/reachability_atlas_deploy/"
                       "s20_container_collision_aware.npz",
            "meta_path": str(data / "s20_container_collision_aware.yaml"),
            "meta_rel": "data/reachability_atlas_deploy/"
                        "s20_container_collision_aware.yaml",
            "graph": {"graph_id": "s20_deploy_empty",
                      "path": str(data / "graphs" / "s20_deploy_empty.json"),
                      "rel": "data/reachability_atlas_deploy/graphs/"
                             "s20_deploy_empty.json",
                      "digest": "0" * 64},
        }],
        resolve=dp.default_resolver(shares={
            "luggage_description": str(desc),
            "luggage_planning": str(plan)}),
        provenance={"command": "pytest", "environment": "synthetic"},
        payload_envelopes=[])


def write_manifest(root, manifest):
    desc = root / "luggage_description"
    (desc / "config" / "deploy_asset_manifest.yaml").write_text(
        yaml.safe_dump(manifest), encoding="utf-8")


@pytest.fixture
def deploy_tree(tmp_path):
    root = write_deploy_tree(tmp_path / "cell")
    write_manifest(root, synthetic_manifest(root))
    return root


def resolver_for(root):
    return dp.default_resolver(shares={
        "luggage_description": str(root / "luggage_description"),
        "luggage_planning": str(root / "luggage_planning")})


def test_default_resolves_deploy_assets(deploy_tree):
    record = dp.resolve_deploy_profile(resolver=resolver_for(deploy_tree))
    assert record["scene_tf_path"].endswith("scene_tf_deploy.yaml")
    assert record["atlas_runtime_config"].endswith(
        "atlas_runtime_deploy.yaml")
    assert record["atlas_npz"].endswith("s20_container_collision_aware.npz")
    assert record["manifest_path"].endswith("deploy_asset_manifest.yaml")


def test_explicit_deploy_path_overrides(deploy_tree):
    root = deploy_tree
    scene_copy = root / "scene_tf_deploy_copy.yaml"
    shutil.copyfile(root / "luggage_description" / "config" /
                    "scene_tf_deploy.yaml", scene_copy)
    # The explicit copy is a different file: re-freeze identity around it.
    import hashlib
    scene_bytes = scene_copy.read_bytes()
    data = root / "luggage_planning" / "data" / "reachability_atlas_deploy"
    meta = yaml.safe_load(
        (data / "s20_container_collision_aware.yaml").read_text("utf-8"))
    meta["dependencies"]["scene_tf_hash"] = hashlib.md5(scene_bytes).hexdigest()
    (data / "s20_container_collision_aware.yaml").write_text(
        yaml.safe_dump(meta), encoding="utf-8")
    write_manifest(root, synthetic_manifest(root))

    record = dp.resolve_deploy_profile(
        scene_arg=str(scene_copy), resolver=resolver_for(root))
    assert record["scene_tf_path"] == str(scene_copy)


def test_sim_scene_rejected_explicitly(deploy_tree):
    sim_scene = deploy_tree / "luggage_description" / "config" / \
        "scene_tf.yaml.example"
    with pytest.raises(ContractError) as caught:
        dp.resolve_deploy_profile(scene_arg=str(sim_scene),
                                  resolver=resolver_for(deploy_tree))
    assert caught.value.code == "SIM_SCENE_ON_HARDWARE"


def test_missing_scene_rejected(tmp_path):
    root = write_deploy_tree(tmp_path / "cell")
    os.remove(root / "luggage_description" / "config" / "scene_tf_deploy.yaml")
    with pytest.raises(ContractError) as caught:
        dp.resolve_deploy_profile(resolver=resolver_for(root))
    assert caught.value.code == "DEPLOY_ASSET_MISSING"


def test_missing_atlas_pair_rejected(tmp_path):
    root = write_deploy_tree(tmp_path / "cell")
    os.remove(root / "luggage_planning" / "data" /
              "reachability_atlas_deploy" /
              "s20_container_collision_aware.npz")
    with pytest.raises(ContractError) as caught:
        dp.resolve_deploy_profile(resolver=resolver_for(root),
                                  check_manifest=False)
    assert caught.value.code == "ATLAS_FILES_NOT_FOUND"


def test_sim_atlas_config_rejected(deploy_tree):
    sim_cfg = deploy_tree / "luggage_planning" / "config" / \
        "atlas_runtime.yaml"
    with pytest.raises(ContractError) as caught:
        dp.resolve_deploy_profile(atlas_config_arg=str(sim_cfg),
                                  resolver=resolver_for(deploy_tree))
    assert caught.value.code == "SIM_ATLAS_ON_HARDWARE"


def test_non_strict_deploy_config_rejected(tmp_path):
    root = write_deploy_tree(tmp_path / "cell")

    def make_warn(_root, desc, plan, data):
        cfg = yaml.safe_load(
            (plan / "config" / "atlas_runtime_deploy.yaml").read_text("utf-8"))
        cfg["hash_policy"] = "warn"
        (plan / "config" / "atlas_runtime_deploy.yaml").write_text(
            yaml.safe_dump(cfg), encoding="utf-8")

    root = write_deploy_tree(tmp_path / "warn", tamper=make_warn)
    with pytest.raises(ContractError) as caught:
        dp.resolve_deploy_profile(resolver=resolver_for(root))
    assert caught.value.code == "DEPLOY_ATLAS_NOT_STRICT"


def test_hash_mismatch_rejected(deploy_tree):
    scene = deploy_tree / "luggage_description" / "config" / \
        "scene_tf_deploy.yaml"
    scene.write_bytes(scene.read_bytes() + b"# tuned\n")
    with pytest.raises(ContractError) as caught:
        dp.resolve_deploy_profile(resolver=resolver_for(deploy_tree))
    assert caught.value.code in ("ATLAS_IDENTITY_MISMATCH",
                                 "HASH_MISMATCH")


def test_ambiguous_assets_rejected(tmp_path):
    """Production drift: the source tree and the install prefix hold
    different bytes under one deploy asset name."""
    one = tmp_path / "one" / "scene_tf_deploy.yaml"
    other = tmp_path / "two" / "scene_tf_deploy.yaml"
    one.parent.mkdir()
    other.parent.mkdir()
    one.write_bytes(b"# frozen deploy scene\n")
    other.write_bytes(b"# drifted install copy\n")
    with pytest.raises(ContractError) as caught:
        dp.reject_ambiguous(str(one), [str(other)], "scene_tf_config")
    assert caught.value.code == "AMBIGUOUS_DEPLOY_ASSET"
    # Identical bytes (or the same file) stay unambiguous.
    other.write_bytes(b"# frozen deploy scene\n")
    dp.reject_ambiguous(str(one), [str(other)], "scene_tf_config")
    dp.reject_ambiguous(str(one), [str(one)], "scene_tf_config")


def test_manifest_tamper_rejected(deploy_tree):
    # One byte in the frozen atlas npz: named rejection, not a fallback.
    npz = deploy_tree / "luggage_planning" / "data" / \
        "reachability_atlas_deploy" / "s20_container_collision_aware.npz"
    npz.write_bytes(npz.read_bytes() + b"x")
    with pytest.raises(ContractError) as caught:
        dp.resolve_deploy_profile(resolver=resolver_for(deploy_tree))
    assert caught.value.code == "HASH_MISMATCH"


def test_detect_only_skips_generated_assets(tmp_path):
    root = write_deploy_tree(tmp_path / "cell")
    os.remove(root / "luggage_planning" / "data" /
              "reachability_atlas_deploy" / "s20_container_collision_aware.npz")
    record = dp.resolve_deploy_profile(
        resolver=resolver_for(root), check_atlas=False)
    assert record["scene_tf_path"].endswith("scene_tf_deploy.yaml")
    assert "atlas_npz" not in record


def test_real_tree_default_and_explicit():
    """The committed deploy cell itself: default and explicit resolution."""
    from conftest import assets_present

    if not assets_present():
        pytest.skip("deploy assets not generated yet")
    record = dp.resolve_deploy_profile()
    assert record["scene_tf_path"].endswith("scene_tf_deploy.yaml")
    explicit = dp.resolve_deploy_profile(
        scene_arg=record["scene_tf_path"],
        atlas_config_arg=record["atlas_runtime_config"],
        manifest_arg=record["manifest_path"])
    assert explicit["scene_tf_hash"] == record["scene_tf_hash"]
    assert explicit["urdf_hash"] == record["urdf_hash"]
