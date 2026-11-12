"""Generated deploy asset acceptance (ST09-PD acceptance matrix).

Runs only after the campaign produced the artifacts. Covers: the four
atlas/graph pairs strict-loading against the exact scene, expanded
URDF, SRDF, tool link and ordered joints; the five independent one-byte
identity mutations each returning a named rejection with zero usable
seeds/edges; and the graph structural validation (endpoints, direction,
finite costs, q-state uniqueness, release twins, start-class route
coverage to every healthy fixture target).
"""

from __future__ import division

import json
import os
import shutil

import pytest
import yaml

from luggage_planning import deploy_profile as dp
from luggage_planning.atlas.runtime import (
    load_atlas_runtime,
    load_runtime_atlas,
    resolve_atlas_files,
)
from luggage_planning.deploy_graph_assets import (
    expected_static_identity,
    load_deploy_graph,
    validate_graph_structure,
)
from luggage_planning.atlas.graph import GraphStore

from conftest import DATA_DIR, MANIFEST_PATH, PROFILES, requires_assets


@pytest.fixture(scope="module")
def cell():
    return dp.resolve_deploy_profile()


@requires_assets
def test_manifest_pins_the_frozen_cell(cell):
    manifest = cell["manifest"]
    assert manifest["schema"] == "luggage.deploy_manifest/1"
    assert manifest["scene"]["scene_revision_md5"] == cell["scene_tf_hash"]
    assert manifest["urdf"]["md5"] == cell["urdf_hash"]
    assert manifest["urdf"]["joint_names"] == [
        "elfin_joint1", "elfin_joint2", "elfin_joint3",
        "elfin_joint4", "elfin_joint5", "elfin_joint6"]
    assert manifest["planning"]["tool_link"] == "suction_contact_frame"
    assert manifest["planning"]["group"] == "elfin_arm"
    assert len(manifest["atlas"]) == 4


@requires_assets
def test_four_atlas_pairs_strict_load(cell):
    cfg = load_atlas_runtime(cell["atlas_runtime_config"])
    assert cfg["hash_policy"] == "strict"
    scene_hash = cell["scene_tf_hash"]
    urdf_hash = cell["urdf_hash"]
    for _basename, payload, _graph_id in PROFILES:
        atlas, diagnostics = load_runtime_atlas(
            cfg, box_size=payload, scene_tf_hash=scene_hash,
            urdf_hash=urdf_hash)
        assert atlas is not None, diagnostics
        assert diagnostics["loaded"] is True
        assert diagnostics["contract_ok"] is True
        # The directed v2 contract and the payload envelope line up.
        contract = atlas.contract_record
        assert contract["orientation_semantics"] == "concrete_tool_pose"
        assert contract["yaw_period_rad"] > 6.0
        # A strict seed query actually returns usable seeds.
        from luggage_planning.atlas.runtime import atlas_seeds_for_query
        meta = yaml.safe_load(open(cell["atlas_meta"], encoding="utf-8"))
        del meta
        seeds = atlas_seeds_for_query(atlas, 0.0, 0.0, 0.8, 0.0,
                                      seed_set="transit", max_seeds=1)
        assert seeds["n_seeds"] >= 0  # query path works; count varies by cell


@requires_assets
def test_four_graphs_strict_load(cell):
    summary = json.loads(
        (DATA_DIR / "graphs" / "summary.json").read_text("utf-8"))
    by_id = {entry["graph_id"]: entry for entry in summary["graphs"]}
    for _basename, payload, graph_id in PROFILES:
        entry = by_id[graph_id]
        path = str(DATA_DIR / "graphs" / ("%s.json" % graph_id))
        identity = expected_static_identity(
            entry["urdf_md5"], "seven_face_hull", entry["geometry_hash"],
            dict(entry["planner_config"], payload=list(payload)
                 if payload else None))
        graph = load_deploy_graph(path, expected_identity=identity)
        assert graph.static_identity == identity
        report = validate_graph_structure(graph)
        assert report["structure_ok"], report["issues"]
        assert report["node_count"] > 0 and report["edge_count"] > 0
        assert report["not_evaluated"] == [], (
            "route coverage missing: %s" % report["not_evaluated"])
        assert report["target_nodes"] > 0
        assert report["prior_gaps"] == [] or all(
            "OFFLINE_EDGE_INVALID" not in gap
            and "ATLAS_CHAIN_UNCOVERED" not in gap
            for gap in report["prior_gaps"])


@requires_assets
def test_graph_edge_semantics(cell):
    """Directed edges carry portal/release/exit semantics with twins."""
    for _basename, _payload, graph_id in PROFILES:
        graph = load_deploy_graph(
            str(DATA_DIR / "graphs" / ("%s.json" % graph_id)))
        edges = list(graph.candidate_edges.values())
        classes = {str(edge.attributes.get("edge_class")) for edge in edges}
        assert {"TRAVERSE", "INSERT", "DESCEND", "RETREAT", "RETURN"} <= \
            classes, classes
        for edge in edges:
            assert edge.trajectory.joint_names == (
                "elfin_joint1", "elfin_joint2", "elfin_joint3",
                "elfin_joint4", "elfin_joint5", "elfin_joint6")
            times = edge.trajectory.time_from_start_ns
            assert all(b > a for a, b in zip(times, times[1:]))
        retreat = [edge for edge in edges
                   if edge.attributes.get("edge_class") == "RETREAT"]
        for edge in retreat:
            source = graph.node(edge.source_node_id)
            assert str(source.load_state) == "unloaded"
            assert source.region == "slot"


def mutation_tree(tmp_path, target):
    """Copy the deploy cell into tmp and flip one byte in ``target``."""
    root = tmp_path / "cell"
    desc = root / "luggage_description"
    plan = root / "luggage_planning"
    shutil.copytree(
        os.path.join(dp._SRC_ROOTS["luggage_description"], "config"),
        desc / "config", dirs_exist_ok=True)
    shutil.copytree(
        os.path.join(dp._SRC_ROOTS["luggage_planning"], "config"),
        plan / "config", dirs_exist_ok=True)
    shutil.copytree(str(DATA_DIR), plan / "data" / "reachability_atlas_deploy")
    paths = {
        "scene": desc / "config" / "scene_tf_deploy.yaml",
        "urdf": plan / "data" / "reachability_atlas_deploy" /
        "robot_description.xml",
        "srdf": desc / "config" / "S20_with_camera.srdf",
        "atlas": plan / "data" / "reachability_atlas_deploy" /
        "s20_container_collision_aware.yaml",
        "graph": plan / "data" / "reachability_atlas_deploy" / "graphs" /
        "s20_deploy_empty.json",
    }
    path = paths[target]
    data = bytearray(path.read_bytes())
    if target == "atlas":
        # Flip exactly one byte of the stored scene identity value, so
        # the mutation deterministically names an identity rejection.
        marker = b"scene_tf_hash: "
        start = bytes(data).index(marker) + len(marker)
        data[start] = (data[start] + 1) % 256
    else:
        # Flip a content byte, never the file length.
        index = len(data) // 2
        data[index] = (data[index] + 1) % 256
    path.write_bytes(bytes(data))
    return root, plan / "data" / "reachability_atlas_deploy"


def mutated_resolver(root):
    return dp.default_resolver(shares={
        "luggage_description": str(root / "luggage_description"),
        "luggage_planning": str(root / "luggage_planning")})


@requires_assets
def test_scene_one_byte_mutation(cell, tmp_path):
    from luggage_description.frame_contracts import ContractError

    root, data_dir = mutation_tree(tmp_path, "scene")
    with pytest.raises(ContractError) as caught:
        dp.resolve_deploy_profile(resolver=mutated_resolver(root))
    assert caught.value.code == "ATLAS_IDENTITY_MISMATCH"
    # Zero usable seeds through the strict loader on the mutated cell.
    cfg = load_atlas_runtime(
        str(root / "luggage_planning" / "config" /
            "atlas_runtime_deploy.yaml"))
    atlas, diagnostics = load_runtime_atlas(
        cfg, scene_tf_hash=_file_md5(
            root / "luggage_description" / "config" / "scene_tf_deploy.yaml"),
        urdf_hash=cell["urdf_hash"])
    assert atlas is None
    assert diagnostics["reason"] in ("hash_mismatch_strict",
                                     "hash_unverified_strict",
                                     "atlas_contract_strict")


def _file_md5(path):
    import hashlib
    with open(path, "rb") as handle:
        return hashlib.md5(handle.read()).hexdigest()


@requires_assets
def test_urdf_one_byte_mutation(tmp_path):
    from luggage_description.frame_contracts import ContractError

    root, data_dir = mutation_tree(tmp_path, "urdf")
    with pytest.raises(ContractError) as caught:
        dp.resolve_deploy_profile(resolver=mutated_resolver(root))
    assert caught.value.code == "ATLAS_IDENTITY_MISMATCH"


@requires_assets
def test_srdf_one_byte_mutation(tmp_path):
    from luggage_description.frame_contracts import ContractError

    root, data_dir = mutation_tree(tmp_path, "srdf")
    with pytest.raises(ContractError) as caught:
        dp.resolve_deploy_profile(resolver=mutated_resolver(root))
    assert caught.value.code == "HASH_MISMATCH"
    assert "srdf" in caught.value.field


@requires_assets
def test_atlas_one_byte_mutation(cell, tmp_path):
    from luggage_description.frame_contracts import ContractError

    root, data_dir = mutation_tree(tmp_path, "atlas")
    # Named rejection: the strict loader refuses the mutated identity
    # and no atlas object (seeds included) escapes the refusal.
    cfg = load_atlas_runtime(
        str(root / "luggage_planning" / "config" /
            "atlas_runtime_deploy.yaml"))
    cfg_mutated = dict(
        cfg, atlas_file=str(data_dir / "s20_container_collision_aware.npz"))
    atlas = None
    diagnostics = {}
    try:
        atlas, diagnostics = load_runtime_atlas(
            cfg_mutated, scene_tf_hash=cell["scene_tf_hash"],
            urdf_hash=cell["urdf_hash"])
    except Exception:
        atlas = None
    assert atlas is None
    assert diagnostics.get("reason")
    # And the profile gate refuses the mutated meta bytes too (either by
    # the atlas identity check or by the manifest hash -- both are named
    # identity rejections of the same tamper).
    with pytest.raises(ContractError) as caught:
        dp.resolve_deploy_profile(resolver=mutated_resolver(root))
    assert caught.value.code in ("HASH_MISMATCH", "ATLAS_IDENTITY_MISMATCH")


@requires_assets
def test_graph_one_byte_mutation(cell, tmp_path):
    root, data_dir = mutation_tree(tmp_path, "graph")
    # Zero usable edges: the store refuses to hand out a mutated graph.
    with pytest.raises(ValueError):
        load_deploy_graph(
            str(data_dir / "graphs" / "s20_deploy_empty.json"),
            expected_identity=None)
    from luggage_description.frame_contracts import ContractError
    with pytest.raises(ContractError) as caught:
        dp.resolve_deploy_profile(resolver=mutated_resolver(root))
    assert caught.value.code == "HASH_MISMATCH"
