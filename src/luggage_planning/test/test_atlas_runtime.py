"""Offline tests for luggage_planning.atlas_runtime."""

from __future__ import division

import math
import os
import sys

import pytest

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(
    0, os.path.normpath(os.path.join(TEST_DIR, "..")))

from luggage_planning import atlas_runtime as ar  # noqa: E402

ATLAS_DATA = os.path.normpath(os.path.join(
    TEST_DIR, "..", "data", "reachability_atlas"))
REAL_ATLAS = os.path.join(
    ATLAS_DATA, "s20_container_collision_aware_payload_0.55x0.40x0.25.npz")

# The 2026-09-22_1540 failing slot in container_link coordinates. Pins are
# against the ST06 directed 4-bin rebuild (2026-09-29): the legacy base-yaw-0
# branch is now named by the DIRECTED container yaw pi/2 (bin index 1, scene
# container yaw -pi/2), and the rebuild reproduces the 2026-09-24
# adaptive-lift seeds there exactly. Container yaw 0 / pi are new branches.
SLOT_XYZ = (-0.47, -0.735, 0.7771664127331008)
SLOT_CELL = [1, 1, 1, 1]
SEED_CELL = [2, 2, 2, 1]
SEED_CELL_CENTER = (-0.37, -0.61, 0.875)
SEED_YAW = math.pi / 2.0
KNOWN_SEED = [0.936, -1.025, 1.268, -1.819, -1.567, -0.635]
# Nearest seeded neighbour of the slot point (one z-step up, same column).
NEAREST_SEED_CELL = [1, 1, 2, 1]
NEAREST_SEED = [0.858, -1.175, 1.504, -1.904, -1.567, -0.713]


@pytest.fixture()
def data_cfg():
    # Query tests read the on-disk grid. Identity is covered separately;
    # the certification default is strict and would refuse a hash-less load.
    return ar._deep_merge(ar.DEFAULT_ATLAS_RUNTIME, {
        "atlas_dir": ATLAS_DATA,
        "hash_policy": "warn",
    })


def test_defaults_and_deep_merge(tmp_path):
    assert ar.load_atlas_runtime().__getitem__("placement")["hard_reject"]
    override = tmp_path / "atlas_runtime.yaml"
    override.write_text(
        "placement: {w_reachability: 0.0}\nmotion: {max_seeds: 1}\n")
    cfg = ar.load_atlas_runtime(str(override))
    assert cfg["placement"]["w_reachability"] == 0.0
    # siblings survive the override
    assert cfg["placement"]["hard_reject"] is True
    assert cfg["motion"]["max_seeds"] == 1
    assert cfg["motion"]["mode"] == "miss-retry"


def test_missing_explicit_path_uses_defaults(tmp_path):
    cfg = ar.load_atlas_runtime(str(tmp_path / "nope.yaml"))
    assert cfg == ar._deep_merge(ar.DEFAULT_ATLAS_RUNTIME, {})


def test_resolve_atlas_files_payload_policy(data_cfg):
    # enveloping: 0.72x0.46x0.30 must pick the 0.80 atlas
    npz, meta = ar.resolve_atlas_files(data_cfg, [0.72, 0.46, 0.30])
    assert npz.endswith("_payload_0.80x0.50x0.32.npz")
    assert meta.endswith(".yaml")
    # exact match wins
    npz, _ = ar.resolve_atlas_files(data_cfg, [0.55, 0.40, 0.25])
    assert npz.endswith("_payload_0.55x0.40x0.25.npz")
    # no box -> empty-load atlas
    npz, _ = ar.resolve_atlas_files(data_cfg, None)
    assert npz.endswith("s20_container_collision_aware.npz")
    # explicit policy pins the size
    explicit = ar._deep_merge(data_cfg, {
        "payload": {"policy": "explicit",
                    "explicit_size": [0.55, 0.40, 0.25]}})
    npz, _ = ar.resolve_atlas_files(explicit, box_size=None)
    assert npz.endswith("_payload_0.55x0.40x0.25.npz")


DEPLOY_RUNTIME = os.path.normpath(os.path.join(
    TEST_DIR, "..", "config", "atlas_runtime_deploy.yaml"))
DEPLOY_SCENE = os.path.normpath(os.path.join(
    TEST_DIR, "..", "..", "luggage_description", "config",
    "scene_tf_deploy.yaml"))
DEPLOY_ATLAS_DIR = ar.resolve_atlas_dir("reachability_atlas_deploy")
DEPLOY_EMPTY = os.path.join(
    DEPLOY_ATLAS_DIR, "s20_container_collision_aware.npz")
DEPLOY_PAYLOADS = (
    [0.55, 0.40, 0.25],
    [0.70, 0.45, 0.28],
    [0.80, 0.50, 0.32],
)


def test_relative_atlas_dir_is_under_package_data():
    resolved = ar.resolve_atlas_dir("reachability_atlas_deploy")
    assert os.path.isabs(resolved)
    assert resolved.endswith(
        os.path.join("data", "reachability_atlas_deploy"))
    assert ar.resolve_atlas_dir("") == ar.default_atlas_dir()
    assert ar.resolve_atlas_dir("reachability_atlas") == ar.default_atlas_dir()
    assert ar.resolve_atlas_dir("/tmp/atlases") == "/tmp/atlases"


def test_default_runtime_stays_on_sim_atlas():
    cfg = ar.load_atlas_runtime()
    npz, _meta = ar.resolve_atlas_files(cfg, None)
    assert npz.endswith(os.path.join(
        "reachability_atlas", "s20_container_collision_aware.npz"))
    assert "reachability_atlas_deploy" not in npz
    assert cfg["hash_policy"] == "strict"


def test_deploy_runtime_is_fail_closed():
    # ST09-PD: the deploy cell refuses an unverified/mismatched atlas
    # exactly like the simulation certification cell. warn is forbidden.
    cfg = ar.load_atlas_runtime(DEPLOY_RUNTIME)
    assert cfg["hash_policy"] == "strict"


SIM_SCENE = os.path.normpath(os.path.join(
    TEST_DIR, "..", "..", "luggage_description", "config",
    "scene_tf.yaml.example"))


def test_scene_file_selects_matching_atlas_cell():
    sim = ar.load_atlas_runtime_for_scene(SIM_SCENE)
    npz, _meta = ar.resolve_atlas_files(sim, None)
    assert npz.endswith(os.path.join(
        "reachability_atlas", "s20_container_collision_aware.npz"))
    assert "reachability_atlas_deploy" not in npz
    deploy = ar.load_atlas_runtime_for_scene(DEPLOY_SCENE)
    npz, _meta = ar.resolve_atlas_files(deploy, None)
    assert npz.startswith(DEPLOY_ATLAS_DIR + os.sep)
    forced = ar.load_atlas_runtime_for_scene(
        DEPLOY_SCENE, explicit_path=ar.atlas_runtime_config_path())
    npz, _meta = ar.resolve_atlas_files(forced, None)
    assert "reachability_atlas_deploy" not in npz


@pytest.mark.skipif(
    not os.path.isfile(DEPLOY_EMPTY), reason="deploy atlas not built")
def test_deploy_runtime_selects_scene_matched_atlases():
    cfg = ar.load_atlas_runtime(DEPLOY_RUNTIME)
    assert cfg["atlas_dir"] == "reachability_atlas_deploy"
    assert cfg["motion"]["seed_set"] == "transit"
    scene_hash = ar.compute_scene_tf_hash(DEPLOY_SCENE)
    assert scene_hash
    import yaml
    for size in (None,) + DEPLOY_PAYLOADS:
        npz, meta_path = ar.resolve_atlas_files(cfg, size)
        assert npz.startswith(DEPLOY_ATLAS_DIR + os.sep)
        if size is None:
            assert npz.endswith("s20_container_collision_aware.npz")
        else:
            assert ("_payload_%.2fx%.2fx%.2f.npz" % tuple(size)) in npz
        with open(meta_path, "r", encoding="utf-8") as handle:
            meta = yaml.safe_load(handle)
        assert meta["dependencies"]["scene_tf_hash"] == scene_hash
        assert meta["mode"] == "collision_aware"


def test_resolve_atlas_file_override_wins(data_cfg, tmp_path):
    override = ar._deep_merge(data_cfg, {"atlas_file": REAL_ATLAS})
    npz, _ = ar.resolve_atlas_files(override, [0.72, 0.46, 0.30])
    assert npz == REAL_ATLAS
    broken = ar._deep_merge(data_cfg, {
        "atlas_file": str(tmp_path / "missing.npz")})
    npz, _ = ar.resolve_atlas_files(broken, [0.72, 0.46, 0.30])
    assert npz.endswith("_payload_0.80x0.50x0.32.npz")  # fell back


@pytest.mark.skipif(not os.path.isfile(REAL_ATLAS), reason="atlas data")
def test_load_runtime_atlas_hash_policies(data_cfg):
    wrong_hash = "deadbeef" * 4
    atlas, diag = ar.load_runtime_atlas(
        data_cfg, box_size=[0.55, 0.40, 0.25], scene_tf_hash=wrong_hash)
    # warn keeps the atlas loaded but records the mismatch
    assert atlas is not None and diag["loaded"]
    assert not diag["scene_tf_ok"]
    assert "scene_tf" in diag["hash_message"]

    strict = ar._deep_merge(data_cfg, {"hash_policy": "strict"})
    atlas, diag = ar.load_runtime_atlas(
        strict, box_size=[0.55, 0.40, 0.25], scene_tf_hash=wrong_hash)
    assert atlas is None and not diag["loaded"]
    assert diag["reason"] == "hash_mismatch_strict"

    off = ar._deep_merge(data_cfg, {"hash_policy": "off"})
    atlas, diag = ar.load_runtime_atlas(
        off, box_size=[0.55, 0.40, 0.25], scene_tf_hash=wrong_hash)
    assert atlas is not None and diag["scene_tf_ok"]


@pytest.mark.skipif(not os.path.isfile(REAL_ATLAS), reason="atlas data")
def test_strict_requires_both_live_hashes(data_cfg):
    import yaml
    strict = ar._deep_merge(data_cfg, {"hash_policy": "strict"})
    _npz, meta_path = ar.resolve_atlas_files(strict, [0.55, 0.40, 0.25])
    with open(meta_path, "r", encoding="utf-8") as handle:
        deps = (yaml.safe_load(handle).get("dependencies") or {})
    scene = deps["scene_tf_hash"]
    urdf = deps["urdf_hash"]
    atlas, diag = ar.load_runtime_atlas(
        strict, box_size=[0.55, 0.40, 0.25],
        scene_tf_hash=scene, urdf_hash=urdf)
    assert atlas is not None and diag["loaded"]
    assert diag["reason"] == "ok"

    atlas, diag = ar.load_runtime_atlas(
        strict, box_size=[0.55, 0.40, 0.25],
        scene_tf_hash=scene, urdf_hash="")
    assert atlas is None and not diag["loaded"]
    assert diag["reason"] == "hash_unverified_strict"
    assert diag["scene_tf_ok"] and not diag["urdf_ok"]

    atlas, diag = ar.load_runtime_atlas(
        strict, box_size=[0.55, 0.40, 0.25],
        scene_tf_hash=scene, urdf_hash="deadbeef" * 4)
    assert atlas is None
    assert diag["reason"] == "hash_mismatch_strict"
    assert not diag["urdf_ok"]


def test_certification_block_reason_is_fail_closed():
    cfg = {
        "enabled": True,
        "hash_policy": "strict",
        "placement": {"dump_only": False, "hard_reject": True},
    }
    assert ar.certification_block_reason(
        cfg, {"loaded": False, "reason": "hash_unverified_strict"},
    ) == ar.ATLAS_IDENTITY_UNVERIFIED
    assert ar.certification_block_reason(cfg, {"loaded": True}) == ""
    warn = dict(cfg, hash_policy="warn")
    assert ar.certification_block_reason(warn, {"loaded": False}) == ""
    dump_only = ar._deep_merge(cfg, {"placement": {"dump_only": True}})
    assert ar.certification_block_reason(dump_only, {"loaded": False}) == ""
    assert ar.ATLAS_IDENTITY_UNVERIFIED == "ATLAS_IDENTITY_UNVERIFIED"


@pytest.mark.skipif(not os.path.isfile(REAL_ATLAS), reason="atlas data")
def test_atlas_seeds_for_query_known_cell(data_cfg):
    atlas, diag = ar.load_runtime_atlas(data_cfg, box_size=[0.55, 0.40, 0.25])
    assert diag["loaded"]
    # Exact slot point: containment cell UNREACHABLE with NaN seeds, the
    # neighbour fallback supplies three seed families from [1,1,2,1].
    out = ar.atlas_seeds_for_query(
        atlas, *SLOT_XYZ, yaw=SEED_YAW, seed_set="transit", max_seeds=3)
    assert out["cell"] == SLOT_CELL
    assert out["status"] == 1                      # UNREACHABLE at exact point
    assert out["status_name"] == "unreachable"
    assert out["seed_cell"] == NEAREST_SEED_CELL
    assert out["seed_cell_offset"] == [0, 0, 1]
    assert out["n_seeds"] >= 1
    assert out["seeds"][0] == pytest.approx(NEAREST_SEED, abs=2e-3)
    for seed in out["seeds"]:
        assert len(seed) == 6
        assert all(v == v for v in seed)           # no NaN
        assert any(abs(v) > 1e-9 for v in seed)    # no zero-padded rows


@pytest.mark.skipif(not os.path.isfile(REAL_ATLAS), reason="atlas data")
def test_atlas_seeds_direct_cell_no_fallback(data_cfg):
    atlas, _ = ar.load_runtime_atlas(data_cfg, box_size=[0.55, 0.40, 0.25])
    out = ar.atlas_seeds_for_query(
        atlas, *SEED_CELL_CENTER, yaw=SEED_YAW, seed_set="transit", max_seeds=3)
    assert out["cell"] == SEED_CELL
    assert out["status"] == 3                      # REACHABLE
    assert out["seed_cell_offset"] == [0, 0, 0]    # no neighbour fallback
    assert out["seeds"][0] == pytest.approx(KNOWN_SEED, abs=2e-3)


@pytest.mark.skipif(not os.path.isfile(REAL_ATLAS), reason="atlas data")
def test_atlas_seeds_directed_bins_and_offset(data_cfg):
    atlas, _ = ar.load_runtime_atlas(data_cfg, box_size=[0.55, 0.40, 0.25])
    base = ar.atlas_seeds_for_query(atlas, *SEED_CELL_CENTER, yaw=SEED_YAW)
    # ST06: directed 2pi bins -- yaw and yaw+pi name DIFFERENT concrete
    # branches (the legacy pi fold is gone; 0 and pi stay distinct).
    twin = ar.atlas_seeds_for_query(
        atlas, *SEED_CELL_CENTER, yaw=SEED_YAW + math.pi)
    assert twin["cell"] != base["cell"]
    assert twin["cell"][3] == 3                    # pi/2 + pi -> 3*pi/2 bin
    assert twin["seeds"][0] != pytest.approx(base["seeds"][0], abs=1e-6)
    neg = ar.atlas_seeds_for_query(atlas, *SLOT_XYZ, yaw=-math.pi / 2.0)
    assert neg["cell"][3] == 3                     # -pi/2 -> 3*pi/2 bin
    # a large offset moves the query to a different cell
    shifted = ar.atlas_seeds_for_query(
        atlas, *SEED_CELL_CENTER, yaw=SEED_YAW, query_offset=(0.3, 0.0, 0.0))
    assert shifted["cell"] != base["cell"]


def test_atlas_seeds_none_atlas():
    out = ar.atlas_seeds_for_query(None, 0.0, 0.0, 0.0, 0.0)
    assert out["n_seeds"] == 0 and out["seeds"] == []
    assert out["status_name"] == "no_atlas"


def _scene_copy():
    return {
        "pedestal": {
            "enabled": True,
            "size": [1.05, 0.74, 0.86],
            "translation": [0.0, 0.0, 0.0],
            "rotation_rpy": [0.0, 0.0, 1.5708],
        },
        "robot": {"base_frame": "elfin_base_link", "rotation_rpy": [0.0, 0.0, 0.0]},
        "container": {
            "inner": {
                "length": 1.49, "width": 1.97, "floor_z": 0.53,
                "ceiling_z": 2.01, "height": 2.01,
                "chamfer": {"side": "positive_y", "floor_y": 0.55, "wall_z": 0.90},
            },
            "opening": {
                "side": "negative_x", "width": 1.33, "height": 1.41,
                "aperture": {"corners": [[-0.745, -0.928, 0.597]]},
            },
        },
        "static_transforms": [{
            "parent": "world", "child": "container_link",
            "translation": [-1.5, 0.6, 0.0],
            "rotation_rpy": [0.0, 0.0, 3.1416],
        }],
        "gazebo": {"container_model": "airport_container_real"},
        "pickup_platform": {"enabled": False, "size": [1.0, 1.0, 0.86]},
        "pickup_source": {"parent": "world", "translation": [1.0, 0.0, 0.0]},
    }


def test_reachability_hash_ignores_pickup_and_catches_container_motion(tmp_path):
    import yaml
    original = _scene_copy()
    moved_pickup = _scene_copy()
    moved_pickup["pickup_platform"] = {
        "enabled": True, "size": [1.0, 1.0, 0.66]}
    moved_pickup["pickup_source"] = {
        "parent": "pickup_platform_top", "translation": [0.0, 0.0, 0.0]}
    moved_container = _scene_copy()
    moved_container["static_transforms"][0]["translation"] = [-1.2, 0.6, 0.0]
    paths = {}
    for name, doc in (
            ("base", original), ("pickup", moved_pickup),
            ("container", moved_container)):
        path = tmp_path / ("%s.yaml" % name)
        path.write_text(yaml.safe_dump(doc), encoding="utf-8")
        paths[name] = str(path)
    base = ar.compute_reachability_scene_hash(paths["base"])
    assert base
    assert ar.compute_reachability_scene_hash(paths["pickup"]) == base
    assert ar.compute_reachability_scene_hash(paths["container"]) != base
    assert ar.compute_scene_tf_hash(paths["base"]) != ar.compute_scene_tf_hash(
        paths["pickup"])
    ok, message, scene_ok, _urdf_ok = ar.identity_verdict(
        {"reachability_scene_hash": base, "scene_tf_hash": "old-file",
         "urdf_hash": "urdf"},
        scene_tf_hash="new-file", urdf_hash="urdf", policy="strict",
        reachability_scene_hash=base)
    assert ok and scene_ok
    assert "diagnostic" in message
    bad, _message, scene_bad, _urdf = ar.identity_verdict(
        {"reachability_scene_hash": base, "scene_tf_hash": "old-file",
         "urdf_hash": "urdf"},
        scene_tf_hash="new-file", urdf_hash="urdf", policy="strict",
        reachability_scene_hash="not-the-same-hash")
    assert not bad and not scene_bad


def test_compute_scene_tf_hash(tmp_path):
    path = tmp_path / "scene_tf.yaml"
    path.write_bytes(b"hello atlas")
    import hashlib
    assert ar.compute_scene_tf_hash(str(path)) == hashlib.md5(b"hello atlas").hexdigest()
    assert ar.compute_scene_tf_hash(str(tmp_path / "missing.yaml")) == ""


def test_compute_urdf_hash_matches_builder():
    import hashlib
    text = "<robot name=\"elfin\"/>"
    assert ar.compute_urdf_hash(text) == hashlib.md5(text.encode()).hexdigest()
    assert ar.compute_urdf_hash("") == ""
    assert ar.compute_urdf_hash(None) == ""


class _StubParam(object):
    def __init__(self, value):
        self.value = value


class _StubNode(object):
    def __init__(self, values):
        self._values = values

    def get_parameter(self, name):
        return _StubParam(self._values[name])


def test_apply_ros_overrides():
    cfg = ar._deep_merge(ar.DEFAULT_ATLAS_RUNTIME, {})
    node = _StubNode({
        "atlas_enabled": False,
        "atlas_w_reachability": 0.0,
        "atlas_query_offset": [0.1, 0.0, 0.0],
        "atlas_submit_mode": "joint_goal",
    })
    ar.apply_ros_overrides(node, cfg)
    assert cfg["enabled"] is False
    assert cfg["placement"]["w_reachability"] == 0.0
    assert cfg["query_offset"] == [0.1, 0.0, 0.0]
    assert cfg["motion"]["submit_mode"] == "joint_goal"
    # untouched keys keep yaml defaults
    assert cfg["motion"]["max_seeds"] == 3


def test_query_semantics_defaults_in_config():
    cfg = ar._deep_merge(ar.DEFAULT_ATLAS_RUNTIME, {})
    sem = cfg["query_semantics"]
    assert sem["enabled"] and sem["floor_clamp"]
    assert sem["neighbor_promote"] and sem["strict_interior_reject"]
    # ROS override path reaches the block
    node = _StubNode({"atlas_qsem_floor_clamp": False})
    ar.apply_ros_overrides(node, cfg)
    assert cfg["query_semantics"]["floor_clamp"] is False


def test_apply_query_semantics_gating(data_cfg):
    assert ar.apply_query_semantics(None, data_cfg) is None
    atlas, _ = ar.load_runtime_atlas(data_cfg, box_size=[0.55, 0.40, 0.25])
    off = ar._deep_merge(data_cfg, {"query_semantics": {"enabled": False}})
    assert ar.apply_query_semantics(atlas, off) is atlas  # passthrough
    wrapped = ar.apply_query_semantics(atlas, data_cfg)
    assert isinstance(wrapped, ar.QuerySemantics)
    # pass-through members keep duck-typing alive
    assert wrapped.yaw_bins == atlas.yaw_bins
    assert wrapped.resolution == atlas.resolution
    assert wrapped.meta is atlas.meta


@pytest.mark.skipif(not os.path.isfile(REAL_ATLAS), reason="atlas data")
def test_query_semantics_floor_reanchor(data_cfg):
    """The smoke1 slot contact: raw iz1 false negative -> semantic iz2.

    iz1's sampled centre 0.725 puts a 0.25 m payload 55 mm into the floor
    slab, so the whole cell reads UNREACHABLE while the real 0.78 pose is
    collision-free (the box was placed there in sim). The semantics layer
    re-anchors the query to the first cell whose centre clears the floor.
    """
    atlas, _ = ar.load_runtime_atlas(data_cfg, box_size=[0.55, 0.40, 0.25])
    sem = ar.QuerySemantics(atlas, floor_z=0.53, box_height=0.25)
    raw = atlas.query(-0.495, -0.735, 0.7772, SEED_YAW)
    got = sem.query(-0.495, -0.735, 0.7772, SEED_YAW)
    assert int(raw.status) == 1 and tuple(raw.indices) == (1, 1, 1, 1)
    assert int(got.status) == 3                      # REACHABLE after re-anchor
    assert tuple(got.indices) == (1, 1, 2, 1)
    assert not got.hard_reject_safe
    # a mid-height feasible query is untouched (no snapshot-to-centre drift)
    mid_raw = atlas.query(-0.37, -0.61, 1.175, SEED_YAW)
    mid_sem = sem.query(-0.37, -0.61, 1.175, SEED_YAW)
    assert int(mid_raw.status) == int(mid_sem.status)
    assert tuple(mid_raw.indices) == tuple(mid_sem.indices)
    # without a box height there is nothing to re-anchor against (promote
    # disabled so the boundary rule cannot mask the raw verdict here)
    no_h = ar.QuerySemantics(
        atlas, floor_z=0.53, box_height=None,
        opts={"neighbor_promote": False, "strict_interior_reject": False})
    assert int(no_h.query(-0.495, -0.735, 0.7772, SEED_YAW).status) == 1


def _cells_with_reachable_neighbors(atlas):
    """Grid cells (status<=1) that have a {2,3} cell in their 26-neighbourhood."""
    import numpy as np
    grid = atlas.status
    reach_any_yaw = (grid >= 2).any(axis=3)
    pad = np.pad(reach_any_yaw, 1, constant_values=False)
    import numpy.lib.stride_tricks as tricks
    near = tricks.sliding_window_view(pad, (3, 3, 3)).any(axis=(-1, -2, -3))
    return [tuple(int(v) for v in cell)
            for cell in np.argwhere((grid <= 1) & near[..., None])]


@pytest.mark.skipif(not os.path.isfile(REAL_ATLAS), reason="atlas data")
def test_query_semantics_boundary_never_rejected(data_cfg):
    """Design guarantee: a point near reachable cells is never hard-rejected.

    Every non-reachable cell that has a reachable 26-neighbour must come
    back UNKNOWN (promoted) and never hard_reject_safe, whatever its own
    confidence says -- "between two reachable points" cannot be rejected.
    """
    atlas, _ = ar.load_runtime_atlas(data_cfg, box_size=[0.55, 0.40, 0.25])
    sem = ar.QuerySemantics(atlas)                    # no floor: pure verdict fix
    origin = atlas.meta["grid"]["origin"]
    res = float(atlas.resolution)
    cells = _cells_with_reachable_neighbors(atlas)
    assert len(cells) > 300                           # the atlas is boundary-rich
    checked = 0
    for ix, iy, iz, iyaw in cells:
        center = (origin[0] + (ix + 0.5) * res,
                  origin[1] + (iy + 0.5) * res,
                  origin[2] + (iz + 0.5) * res)
        yaw = (atlas.yaw_bins or [0.0, 1.5707963])[iyaw]
        raw = atlas.query(*center, yaw)
        got = sem.query(*center, yaw)
        assert int(got.status) == 0
        assert got.reason == "boundary_reachable_neighbor"
        assert not got.hard_reject_safe
        # promotion only demotes; it must not upgrade anything
        if int(raw.status) >= 2:
            assert int(got.status) == int(raw.status)
        checked += 1
        if checked >= 500:                            # cap runtime, seeded by the scan
            break


@pytest.mark.skipif(not os.path.isfile(REAL_ATLAS), reason="atlas data")
def test_query_semantics_strict_interior_blocks_reject(data_cfg):
    """With promotion off, the strict-interior rule alone blocks rejection."""
    atlas, _ = ar.load_runtime_atlas(data_cfg, box_size=[0.55, 0.40, 0.25])
    strict_only = ar.QuerySemantics(
        atlas, opts={"neighbor_promote": False, "strict_interior_reject": True})
    origin = atlas.meta["grid"]["origin"]
    res = float(atlas.resolution)
    for ix, iy, iz, iyaw in _cells_with_reachable_neighbors(atlas)[:200]:
        raw = atlas.query(
            origin[0] + (ix + 0.5) * res, origin[1] + (iy + 0.5) * res,
            origin[2] + (iz + 0.5) * res,
            (atlas.yaw_bins or [0.0, 1.5707963])[iyaw])
        got = strict_only.query(
            origin[0] + (ix + 0.5) * res, origin[1] + (iy + 0.5) * res,
            origin[2] + (iz + 0.5) * res,
            (atlas.yaw_bins or [0.0, 1.5707963])[iyaw])
        # status untouched (promotion off), rejection suppressed
        assert int(got.status) == int(raw.status)
        assert not got.hard_reject_safe
