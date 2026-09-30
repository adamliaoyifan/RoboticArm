"""ST06 frame-migration atlas suite: F14 directed concrete-tool branches.

Covers the atlas target construction, the v2 artifact contract, directed
query semantics (no pi fold), strict-policy v1 refusal, the both-grasp
placement rule, the F16 seed-side yaw mapping, and the offline FK
certification gate. Runs after building/sourcing the isolated install; no
ROS nodes, no TF, no simulation. FRAME_MIGRATION_CASE_LOG selects an
append-only JSONL evidence file.
"""

from __future__ import division

import copy
import hashlib
import itertools
import json
import math
import os
import runpy
import sys
import tempfile
import time
import traceback
import unittest
from contextlib import contextmanager
from pathlib import Path

import numpy as np
import yaml

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.normpath(os.path.join(TEST_DIR, ".."))
for _extra in (
        PKG_DIR,
        os.path.normpath(os.path.join(PKG_DIR, "..", "luggage_description")),
        os.path.normpath(os.path.join(PKG_DIR, "..", "luggage_packing"))):
    if _extra not in sys.path:
        sys.path.insert(0, _extra)

from luggage_description.frame_contracts import (  # noqa: E402
    ContractError, validate_atlas,
)
from luggage_planning.atlas_frame_contracts import (  # noqa: E402
    DIRECTED_YAW_BINS,
    atlas_contract_record,
    container_yaw_offset,
    geometry_hash,
    payload_hash,
    snap_yaw_directed,
    tool_target_in_base,
    wrapped_yaw_error,
)
from luggage_planning.reachability_atlas import (  # noqa: E402
    MARGINAL,
    REACHABLE,
    UNREACHABLE,
    UNKNOWN,
    ReachabilityAtlas,
)
from luggage_planning import atlas_runtime as ar  # noqa: E402
from luggage_packing.placement_reachability import (  # noqa: E402
    annotate_with_atlas,
    make_atlas_annotate,
)

SCENE = "scene-calibration-17"
WDH = (0.80, 0.50, 0.32)
CONTAINER_IN_BASE = {
    "sim": ((0.02, -1.51, -0.86), (0.0, 0.0, -math.pi / 2.0)),
}
TWO_PI = 2.0 * math.pi


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return repr(value)
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_safe(v) for v in value]
    return value


@contextmanager
def case(case_id, inputs):
    row = dict(case_id=case_id, input=copy.deepcopy(inputs), status="pass",
               observations={})
    started = time.monotonic()
    try:
        yield row["observations"]
    except BaseException:
        row.update(status="fail", traceback=traceback.format_exc())
        raise
    finally:
        row["elapsed_seconds"] = time.monotonic() - started
        output = os.environ.get("FRAME_MIGRATION_CASE_LOG")
        if output:
            with Path(output).open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(json_safe(row), allow_nan=False,
                                        sort_keys=True) + "\n")
                handle.flush()
                os.fsync(handle.fileno())
        print("CASE", case_id, row["status"], flush=True)


# ---------------------------------------------------------------------------
# Independent oracle (scipy), deliberately not the implementation's math.
# ---------------------------------------------------------------------------

def _oracle_matrix_from_quat(q):
    from scipy.spatial.transform import Rotation as Rot
    return Rot.from_quat(q).as_matrix()


def _oracle_quat_from_matrix(R):
    from scipy.spatial.transform import Rotation as Rot
    return Rot.from_matrix(R).as_quat()


def oracle_tool_target(x, y, z, yaw_container, container_xyz, container_rpy):
    """Independent base_link target for the advertised container sample."""
    from scipy.spatial.transform import Rotation as Rot
    R_cb = Rot.from_euler("xyz", container_rpy).as_matrix()
    pos = np.array(container_xyz) + R_cb @ np.array([x, y, z])
    R_tool_down = _oracle_matrix_from_quat((1.0, 0.0, 0.0, 0.0))
    R_target = R_cb @ Rot.from_euler("z", float(yaw_container)).as_matrix() \
        @ R_tool_down
    return pos, _oracle_quat_from_matrix(R_target)


def quat_distance_sign_equivalent(qa, qb):
    dot = abs(float(np.dot(np.array(qa), np.array(qb))))
    return min(
        float(np.max(np.abs(np.array(qa) - np.array(qb)))),
        float(np.max(np.abs(np.array(qa) + np.array(qb)))))


class QueryResultStub:
    """Minimal result for the atlas duck type (status/hard_reject_safe)."""

    def __init__(self, status, hard_reject_safe, seeds=()):
        self.status = status
        self.hard_reject_safe = hard_reject_safe
        self.contact_seeds = list(seeds)
        self.transit_seeds = []
        self.reason = "stub"


class TestTargetConstruction(unittest.TestCase):
    """64-case analytic matrix: advertised container target -> base."""

    def test_matrix_64_target_cases(self):
        yaws = (0.0, math.pi / 2.0, math.pi, math.atan2(4.0, 3.0))
        translations = ((0.0, 0.0, 0.0), (1.5, -0.4, 0.2))
        floors = (0.53, 0.71)
        supports = (0.0, 0.30)
        grasp_offsets = ((0.0, 0.0, 0.16), (0.10, -0.05, 0.16))
        width, depth, height = WDH
        cases = 0
        factors = itertools.product(
            yaws, translations, floors, supports, grasp_offsets)
        for cyaw, (tx, ty, tz), floor, support, (ox, oy, oz) in factors:
            case_id = "target/%s" % cases
            box_yaw = cyaw  # box axes aligned with the container yaw factor
            # Slot centre: container box centre z = floor + support + h/2.
            center = (0.11, -0.07, floor + support + height / 2.0)
            # TCP (suction contact) through the grasp offset in BOX axes.
            c, s = math.cos(box_yaw), math.sin(box_yaw)
            contact = (
                center[0] + c * ox + s * oy,
                center[1] - s * ox + c * oy,
                center[2] + oz,
            )
            container_xyz = (tx, ty, tz - 0.53)  # keep the sample in-grid
            with case(case_id, dict(
                    container_yaw=cyaw, translation=(tx, ty, tz),
                    floor=floor, support=support, grasp_offset=(ox, oy, oz),
                    contact_container=contact)) as obs:
                pos, quat = tool_target_in_base(
                    contact[0], contact[1], contact[2], box_yaw,
                    container_xyz, (0.0, 0.0, cyaw))
                o_pos, o_quat = oracle_tool_target(
                    contact[0], contact[1], contact[2], box_yaw,
                    container_xyz, (0.0, 0.0, cyaw))
                self.assertLessEqual(
                    np.linalg.norm(pos - o_pos), 1e-9,
                    "position drift at %s" % case_id)
                self.assertLessEqual(
                    quat_distance_sign_equivalent(quat, o_quat), 1e-9,
                    "orientation drift at %s" % case_id)
                obs["pos_base"] = list(pos)
                obs["quat_base"] = list(quat)
            cases += 1
        self.assertEqual(cases, 64)

    def test_directed_yaw_bins_are_distinct_branches(self):
        with case("bins/distinct-0-pi", dict(bins=list(DIRECTED_YAW_BINS))):
            self.assertEqual(len(set(DIRECTED_YAW_BINS)), 4)
            for bin_yaw in DIRECTED_YAW_BINS:
                self.assertTrue(0.0 <= bin_yaw < TWO_PI)
            bin0, err0 = snap_yaw_directed(0.0, DIRECTED_YAW_BINS)
            bin_pi, err_pi = snap_yaw_directed(math.pi, DIRECTED_YAW_BINS)
            self.assertEqual(bin0, 0.0)
            self.assertEqual(bin_pi, math.pi)
            self.assertGreaterEqual(wrapped_yaw_error(0.0, math.pi),
                                    math.pi - 1e-9)

    def test_q_neg_q_share_rotation_yaw_pi_distinct(self):
        with case("bins/sign-equivalence", {}):
            # tool yaw 0 vs pi name DIFFERENT base orientations.
            _, q0 = tool_target_in_base(
                0.1, 0.2, 0.7, 0.0, *CONTAINER_IN_BASE["sim"])
            _, qpi = tool_target_in_base(
                0.1, 0.2, 0.7, math.pi, *CONTAINER_IN_BASE["sim"])
            self.assertGreater(
                quat_distance_sign_equivalent(q0, qpi), 1e-6)
            # q and -q are the same rotation.
            self.assertAlmostEqual(
                quat_distance_sign_equivalent(q0, tuple(-v for v in q0)),
                0.0, delta=1e-12)

    def test_tilted_container_rejected(self):
        with case("target/tilt-rejected",
                  dict(roll=0.2)) as obs:
            with self.assertRaises(ValueError):
                tool_target_in_base(
                    0.1, 0.2, 0.7, 0.0, (0.0, -1.5, -0.86),
                    (0.2, 0.0, -math.pi / 2.0))
            obs["admission"] = "2.5D gate rejects roll=0.2"


class TestContractRecord(unittest.TestCase):
    """The artifact contract block vs validate_atlas."""

    BASE = {
        "yaw_bins": list(DIRECTED_YAW_BINS),
        "scene_tf_hash": "ab" * 16,
        "urdf_hash": "cd" * 16,
        "geometry": (
            [1.49, 1.97, 2.01], 0.53, 2.01, "negative_x"),
        "payload": {
            "enabled": True, "shape": "box", "size": [0.80, 0.50, 0.32],
            "attach_link": "suction_contact_frame",
            "offset": [0.0, 0.0, 0.16],
            "touch_links": ["suction_panel"],
        },
    }

    def record(self, **overrides):
        base = copy.deepcopy(self.BASE)
        geom = base.pop("geometry")
        payload = base.pop("payload")
        base.update(overrides)
        record = atlas_contract_record(
            base["yaw_bins"], base["scene_tf_hash"], base["urdf_hash"],
            geometry_hash(*geom), payload_hash(payload))
        record["_geom"] = geom
        record["_payload"] = payload
        return record

    def test_valid_record_passes_validate_atlas(self):
        with case("contract/valid", {}) as obs:
            record = self.record()
            validate_atlas(record, {
                "scene_revision": record["scene_revision"],
                "geometry_hash": record["geometry_hash"],
                "urdf_hash": record["urdf_hash"],
                "payload_hash": record["payload_hash"],
            })
            obs["fields"] = sorted(
                k for k in record if not k.startswith("_"))

    def _negative(self, case_id, mutate, code, field):
        with case(case_id, dict(code=code)):
            record = self.record()
            # The consumer's expected identity stays the ORIGINAL one.
            expected = {
                "scene_revision": record["scene_revision"],
                "geometry_hash": record["geometry_hash"],
                "urdf_hash": record["urdf_hash"],
                "payload_hash": record["payload_hash"],
            }
            mutate(record)
            with self.assertRaises(ContractError) as ctx:
                validate_atlas(record, expected)
            self.assertEqual(ctx.exception.code, code)
            if field is not None:
                self.assertEqual(ctx.exception.field, field)

    def test_negatives_exact_codes(self):
        self._negative(
            "contract/wrong-orientation-frame",
            lambda r: r.__setitem__("orientation_frame", "elfin_base_link"),
            "ATLAS_SEMANTICS_MISMATCH", "orientation_frame")
        self._negative(
            "contract/pi-fold-rejected",
            lambda r: r.__setitem__("yaw_period_rad", math.pi),
            "ATLAS_SEMANTICS_MISMATCH", "yaw_period_rad")
        self._negative(
            "contract/stale-payload",
            lambda r: r.__setitem__("payload_hash", "ef" * 16),
            "ATLAS_IDENTITY_MISMATCH", "payload_hash")
        self._negative(
            "contract/stale-scene",
            lambda r: r.__setitem__("scene_revision", "ff" * 16),
            "ATLAS_IDENTITY_MISMATCH", "scene_revision")
        self._negative(
            "contract/missing-urdf-identity",
            lambda r: r.pop("urdf_hash"),
            "ATLAS_IDENTITY_MISMATCH", "urdf_hash")
        self._negative(
            "contract/duplicate-bins-rejected",
            lambda r: r.__setitem__(
                "yaw_bins_rad", [0.0, 0.0, math.pi / 2.0, math.pi]),
            "ATLAS_SEMANTICS_MISMATCH", "yaw_bins_rad")


def synthetic_atlas(contract=True, directed=True, status_map=None,
                    seed_rows=(((0.936, -1.025, 1.268, -1.819, -1.567,
                                 -0.635),),)):
    """Small in-memory atlas with optional v2 contract block."""
    nx = ny = nz = 3
    bins = list(DIRECTED_YAW_BINS) if directed else [0.0, math.pi / 2.0]
    nyaw = len(bins)
    shape = (nx, ny, nz, nyaw)
    status = np.full(shape, UNKNOWN, dtype=np.uint8)
    contact = np.full(shape + (1, 6), np.nan, dtype=np.float64)
    transit = np.full_like(contact, np.nan)
    if status_map:
        for cell, value in status_map.items():
            status[cell] = value
            if value >= MARGINAL:
                row = list(seed_rows[0][0])
                contact[cell + (0,)] = row
                transit[cell + (0,)] = row
    meta = {
        "grid": {
            "frame": "container_link",
            "resolution_xyz": 0.15,
            "origin": [-0.225, -0.225, 0.5],
            "size": [nx, ny, nz],
            "yaw_bins": bins,
            "transit_clearance": 0.3,
        },
        "query": {"yaw_tolerance": 0.10},
        "dependencies": {
            "robot_model": "elfin_s20_with_camera",
            "scene_tf_hash": "ab" * 16,
            "urdf_hash": "cd" * 16,
        },
        "container": {
            "inner_dimensions": [1.49, 1.97, 2.01],
            "floor_z": 0.53,
            "ceiling_z": 2.01,
            "base_xyz": list(CONTAINER_IN_BASE["sim"][0]),
            "base_rpy": list(CONTAINER_IN_BASE["sim"][1]),
            "opening_side": "negative_x",
        },
        "payload": {
            "enabled": False, "shape": None, "size": None,
            "attach_link": "suction_contact_frame", "offset": None,
            "touch_links": None,
        },
    }
    if contract:
        meta["contract"] = atlas_contract_record(
            bins, meta["dependencies"]["scene_tf_hash"],
            meta["dependencies"]["urdf_hash"],
            geometry_hash(
                meta["container"]["inner_dimensions"],
                meta["container"]["floor_z"],
                meta["container"]["ceiling_z"],
                meta["container"]["opening_side"]),
            payload_hash(meta["payload"]))
        meta["atlas_version"] = "3.0"
        meta["schema_version"] = 3
    data = {
        "status": status,
        "contact_seeds": contact,
        "transit_seeds": transit,
        "solution_count": (status >= MARGINAL).astype(np.uint8),
        "neighbor_confidence": np.full(shape, 0.95, dtype=np.float32),
        "opening_connected": status >= MARGINAL,
    }
    return ReachabilityAtlas.from_builder(meta=meta, **data)


class TestDirectedQuerySemantics(unittest.TestCase):

    def test_v2_queries_both_directed_grasps(self):
        # Bin 0 confidently unreachable interior; bin pi reachable. The
        # directed artifact must keep the candidate (pi twin carries it).
        atlas = synthetic_atlas(status_map={
            (1, 1, 1, 0): UNREACHABLE,
            (1, 1, 1, 2): REACHABLE,
        })
        with case("query/directed-both-grasps", {}) as obs:
            best, all_hard = atlas.query_pi_equivalents(0.0, 0.0, 0.725, 0.0)
            self.assertFalse(all_hard)
            self.assertEqual(best.status, REACHABLE)
            obs["status"] = int(best.status)

    def test_v2_hard_reject_needs_both_branches(self):
        atlas = synthetic_atlas(status_map={
            (1, 1, 1, 0): UNREACHABLE,
            (1, 1, 1, 2): UNREACHABLE,
        })
        with case("query/directed-hard-reject", {}) as obs:
            best, all_hard = atlas.query_pi_equivalents(0.0, 0.0, 0.725, 0.0)
            self.assertTrue(all_hard)
            self.assertEqual(best.status, UNREACHABLE)
            obs["all_hard"] = all_hard

    def test_v1_keeps_single_folded_query(self):
        atlas = synthetic_atlas(
            contract=False, directed=False,
            status_map={(1, 1, 1, 0): UNREACHABLE})
        with case("query/v1-fold-unchanged", {}) as obs:
            self.assertIsNone(atlas.contract_record)
            best, all_hard = atlas.query_pi_equivalents(0.0, 0.0, 0.725, 0.0)
            self.assertEqual(all_hard, best.hard_reject_safe)
            # The v1 fold maps yaw pi onto bin 0.
            folded, _ = ar.snap_yaw_to_bin(atlas, math.pi)
            self.assertEqual(folded, 0.0)
            obs["folded"] = folded

    def test_snap_for_atlas_dispatch(self):
        v2 = synthetic_atlas()
        v1 = synthetic_atlas(contract=False, directed=False)
        with case("snap/dispatch", {}):
            bin_v2, _ = ar.snap_yaw_for_atlas(v2, math.pi)
            self.assertEqual(bin_v2, math.pi)
            bin_v1, _ = ar.snap_yaw_for_atlas(v1, math.pi)
            self.assertEqual(bin_v1, 0.0)
            self.assertEqual(
                len(ar.directed_yaw_candidates(v2, 0.3)), 2)
            self.assertEqual(
                len(ar.directed_yaw_candidates(v1, 0.3)), 1)

    def test_unknown_stays_unknown(self):
        atlas = synthetic_atlas(status_map={})
        with case("query/unknown", {}):
            best, all_hard = atlas.query_pi_equivalents(0.0, 0.0, 0.725, 0.0)
            self.assertEqual(best.status, UNKNOWN)
            self.assertFalse(all_hard)


class TestRuntimeStrictGate(unittest.TestCase):

    def _write_artifact(self, directory, atlas, basename="toy"):
        npz = os.path.join(directory, basename + ".npz")
        meta_path = os.path.join(directory, basename + ".yaml")
        atlas.save(npz, meta_path)
        return npz, meta_path

    def test_strict_refuses_v1_semantics_with_zero_seeds(self):
        atlas = synthetic_atlas(contract=False, directed=False)
        with tempfile.TemporaryDirectory() as tmp:
            npz, meta_path = self._write_artifact(tmp, atlas)
            cfg = {"enabled": True, "hash_policy": "strict",
                   "atlas_file": npz, "basename": "unused"}
            with case("strict/v1-refused", dict(npz=npz)) as obs:
                loaded, diag = ar.load_runtime_atlas(
                    cfg, scene_tf_hash="ab" * 16, urdf_hash="cd" * 16)
                self.assertIsNone(loaded)
                self.assertEqual(diag["reason"], "atlas_semantics_v1_strict")
                self.assertEqual(
                    ar.certification_block_reason(cfg, diag),
                    ar.ATLAS_SEMANTICS_MISMATCH)
                # Zero seeds escape: the refused atlas never reaches the
                # seed query.
                seeds = ar.atlas_seeds_for_query(
                    loaded, 0.0, 0.0, 0.725, 0.0)
                self.assertEqual(seeds["n_seeds"], 0)
                obs["reason"] = diag["reason"]

    def test_strict_accepts_self_consistent_contract(self):
        atlas = synthetic_atlas(status_map={(1, 1, 1, 1): REACHABLE})
        with tempfile.TemporaryDirectory() as tmp:
            npz, _ = self._write_artifact(tmp, atlas)
            cfg = {"enabled": True, "hash_policy": "strict",
                   "atlas_file": npz, "basename": "unused"}
            with case("strict/v2-accepted", {}) as obs:
                loaded, diag = ar.load_runtime_atlas(
                    cfg, scene_tf_hash="ab" * 16, urdf_hash="cd" * 16)
                self.assertIsNotNone(loaded)
                self.assertTrue(diag["loaded"])
                self.assertEqual(diag["reason"], "ok")
                obs["contract"] = bool(loaded.contract_record)

    def test_strict_refuses_relabelled_contract(self):
        atlas = synthetic_atlas(status_map={(1, 1, 1, 1): REACHABLE})
        # Relabel: keep the stamped contract, change the payload block.
        atlas.meta["payload"]["enabled"] = True
        atlas.meta["payload"]["size"] = [0.55, 0.40, 0.25]
        with tempfile.TemporaryDirectory() as tmp:
            npz, _ = self._write_artifact(tmp, atlas)
            cfg = {"enabled": True, "hash_policy": "strict",
                   "atlas_file": npz, "basename": "unused"}
            with case("strict/relabel-refused", {}) as obs:
                loaded, diag = ar.load_runtime_atlas(
                    cfg, scene_tf_hash="ab" * 16, urdf_hash="cd" * 16)
                self.assertIsNone(loaded)
                self.assertEqual(diag["reason"], "atlas_contract_strict")
                self.assertFalse(diag["loaded"])
                obs["reason"] = diag["reason"]

    def test_warn_keeps_v1_with_downgrade(self):
        atlas = synthetic_atlas(contract=False, directed=False)
        with tempfile.TemporaryDirectory() as tmp:
            npz, _ = self._write_artifact(tmp, atlas)
            cfg = {"enabled": True, "hash_policy": "warn",
                   "atlas_file": npz, "basename": "unused"}
            with case("warn/v1-downgrade", {}):
                loaded, diag = ar.load_runtime_atlas(cfg)
                self.assertIsNotNone(loaded)
                self.assertTrue(diag["loaded"])
                self.assertIsNone(loaded.contract_record)


class TestPlacementHookBothGrasps(unittest.TestCase):

    CANDIDATE = {
        "center_container": [0.0, 0.0, 0.565],
        "size": [WDH[0], WDH[1], WDH[2]],
        "box_yaw": 0.0,
        "score": 1.0,
    }

    def test_one_live_branch_survives(self):
        atlas = synthetic_atlas(status_map={
            (1, 1, 1, 0): UNREACHABLE,
            (1, 1, 1, 2): REACHABLE,
        })
        hook = make_atlas_annotate(
            atlas, floor_z=0.53, inner_h=2.01, hard_reject=True)
        with case("hook/one-branch-live", {}) as obs:
            rows, report = hook([copy.deepcopy(self.CANDIDATE)])
            self.assertEqual(report["n_hard_rejected"], 0)
            self.assertEqual(rows[0]["atlas_status"], REACHABLE)
            self.assertTrue(rows[0].get("feasible", True))
            obs["bins_queried"] = rows[0]["atlas_yaw_bins_queried"]

    def test_both_branches_dead_rejects(self):
        atlas = synthetic_atlas(status_map={
            (1, 1, 1, 0): UNREACHABLE,
            (1, 1, 1, 2): UNREACHABLE,
        })
        hook = make_atlas_annotate(
            atlas, floor_z=0.53, inner_h=2.01, hard_reject=True)
        with case("hook/both-branches-dead", {}):
            # The hook marks rejects in place (kept in the list so the
            # reject histogram sees them) -- the original contract.
            rows, report = hook([copy.deepcopy(self.CANDIDATE)])
            self.assertEqual(report["n_hard_rejected"], 1)
            self.assertFalse(rows[0]["feasible"])
            self.assertEqual(rows[0]["reason"], "atlas_unreachable")

    def test_stub_atlas_keeps_legacy_single_query(self):
        class Stub:
            yaw_bins = [0.0, math.pi / 2.0]

            def query(self, x, y, z, yaw):
                self.queried_yaw = yaw
                return QueryResultStub(UNREACHABLE, True)

        stub = Stub()
        legacy = dict(self.CANDIDATE)
        # annotate_with_atlas serves the replay-row shape (center_local).
        legacy["center_local"] = [0.0, 0.0, 0.0]
        legacy["peak"] = 0.0
        with case("hook/stub-legacy", {}):
            kept, rejected = annotate_with_atlas([legacy], stub, 0.53)
            self.assertEqual(len(kept), 0)
            self.assertEqual(rejected, 1)
            self.assertEqual(stub.queried_yaw, 0.0)


class TestSeedSideYaw(unittest.TestCase):
    """F16 seed side: the node maps the world yaw into container axes."""

    NODE = os.path.join(PKG_DIR, "scripts", "motion_planner_node.py")

    @unittest.skipUnless(os.path.isfile(NODE), "node script not present")
    def test_container_tool_yaw_subtracts_scene_yaw(self):
        module = runpy.run_path(self.NODE)
        # origin_in_world reads static_transforms (world -> container_link).
        scene = {"static_transforms": [
            {"parent": "world", "child": "container_link",
             "translation": [0.4, -0.2, 0.0],
             "rotation_rpy": [0.0, 0.0, math.pi / 4.0]}]}
        namespace = type("NodeStub", (), {})()
        namespace._scene = scene
        # Tool-down family q=(cos(y/2), sin(y/2), 0, 0) at world yaw y.
        world_yaw = math.pi / 4.0 + math.pi / 6.0
        half = world_yaw / 2.0
        quat = type("Quat", (), {
            "x": math.cos(half), "y": math.sin(half),
            "z": 0.0, "w": 0.0})()
        with case("seed/container-yaw", dict(world_yaw=world_yaw)):
            got = module["MotionPlannerNode"]._container_tool_yaw(
                namespace, quat)
            self.assertAlmostEqual(got, math.pi / 6.0, delta=1e-9)


class TestFKCertification(unittest.TestCase):
    """The offline FK gate on a real-model synthetic artifact."""

    @classmethod
    def setUpClass(cls):
        from luggage_planning.offline_ik import load_urdf, refine_to_pose
        cls.urdf = load_urdf()
        path = cls.urdf.path_to("suction_contact_frame")
        cls.tcp_path = path[path.index("elfin_joint1"):]
        # Known-reachable slot class (2026-09-24 adaptive-lift evidence):
        # container centre (-0.37, -0.61, 0.875) at base yaw 0 == container
        # bin pi/2 for the sim container (base_rpy z = -pi/2).
        cls.cell = (-0.37, -0.61, 0.875)
        seed = [0.936, -1.025, 1.268, -1.819, -1.567, -0.635]
        pos, quat = tool_target_in_base(
            cls.cell[0], cls.cell[1], cls.cell[2], math.pi / 2.0,
            *CONTAINER_IN_BASE["sim"])
        joints, _info = refine_to_pose(
            cls.urdf, cls.tcp_path, pos, quat, seed)
        if joints is None:
            raise unittest.SkipTest("refine did not converge on this host")
        cls.contact_joints = list(joints)
        t_pos, t_quat = tool_target_in_base(
            cls.cell[0], cls.cell[1], cls.cell[2] + 0.30, math.pi / 2.0,
            *CONTAINER_IN_BASE["sim"])
        tjoints, _tinfo = refine_to_pose(
            cls.urdf, cls.tcp_path, t_pos, t_quat, joints)
        cls.transit_joints = list(tjoints) if tjoints is not None else None

    def _artifact(self, tmp, contact_row, transit_row, lift=0.30,
                  urdf_hash="cd" * 16):
        nx = ny = nz = 3
        bins = list(DIRECTED_YAW_BINS)
        shape = (nx, ny, nz, len(bins))
        status = np.full(shape, UNKNOWN, dtype=np.uint8)
        contact = np.full(shape + (1, 6), np.nan, dtype=np.float64)
        transit = np.full_like(contact, np.nan)
        transit_lift = np.full(shape + (1,), np.nan, dtype=np.float32)
        cell = (1, 1, 1, 1)  # bin index 1 == container pi/2
        status[cell] = REACHABLE
        contact[cell + (0,)] = contact_row
        transit[cell + (0,)] = transit_row
        transit_lift[cell + (0,)] = lift
        payload = {"enabled": False, "shape": None, "size": None,
                   "attach_link": "suction_contact_frame", "offset": None,
                   "touch_links": None}
        meta = {
            "grid": {"frame": "container_link",
                     "resolution_xyz": 0.15,
                     "origin": [-0.595, -0.835, 0.65],
                     "size": [nx, ny, nz], "yaw_bins": bins,
                     "transit_clearance": 0.3},
            "query": {"yaw_tolerance": 0.10},
            "dependencies": {"robot_model": "elfin_s20_with_camera",
                             "scene_tf_hash": "ab" * 16,
                             "urdf_hash": urdf_hash},
            "container": {
                "inner_dimensions": [1.49, 1.97, 2.01], "floor_z": 0.53,
                "ceiling_z": 2.01,
                "base_xyz": list(CONTAINER_IN_BASE["sim"][0]),
                "base_rpy": list(CONTAINER_IN_BASE["sim"][1]),
                "opening_side": "negative_x"},
            "payload": payload,
            "contract": atlas_contract_record(
                bins, "ab" * 16, urdf_hash,
                geometry_hash([1.49, 1.97, 2.01], 0.53, 2.01, "negative_x"),
                payload_hash(payload)),
            "atlas_version": "3.0", "schema_version": 3,
        }
        npz = os.path.join(tmp, "fk.npz")
        meta_path = os.path.join(tmp, "fk.yaml")
        np.savez_compressed(
            npz, status=status, contact_seeds=contact,
            transit_seeds=transit, transit_lift=transit_lift,
            solution_count=(status >= MARGINAL).astype(np.uint8),
            neighbor_confidence=np.full(shape, 0.95, dtype=np.float32),
            opening_connected=status >= MARGINAL)
        with open(meta_path, "w", encoding="utf-8") as handle:
            yaml.safe_dump(meta, handle)
        return npz, meta_path

    def test_certify_pass_and_breach(self):
        script = os.path.join(
            PKG_DIR, "..", "..", "scripts", "certify_atlas_seeds.py")
        module = runpy.run_path(script)
        certify = module["certify"]
        with tempfile.TemporaryDirectory() as tmp:
            capture = os.path.join(tmp, "robot_description.xml")
            description = "<robot name='elfin_s20_with_camera'/>"
            with open(capture, "w", encoding="utf-8") as handle:
                handle.write(description)
            urdf_hash = hashlib.md5(description.encode()).hexdigest()
            transit = self.transit_joints or self.contact_joints
            lift = 0.30 if self.transit_joints is not None else 0.0
            npz, meta_path = self._artifact(
                tmp, self.contact_joints, transit, lift=lift,
                urdf_hash=urdf_hash)
            with case("fk/certify-pass", {}) as obs:
                result = certify(npz, meta_path, urdf_file=capture)
                self.assertEqual(result["verdict"], "not_evaluated",
                                 "single-bin coverage is not_evaluated")
                self.assertEqual(result["counts"]["contact_rows"], 1)
                self.assertEqual(
                    result["counts"]["transit_vertical_rows"], 1)
                self.assertFalse(result["breaches"])
                obs["counts"] = result["counts"]

            # Perturb one joint: the FK gate must fail the seed.
            bad = list(self.contact_joints)
            bad[0] += 0.05
            npz_bad, meta_bad = self._artifact(
                tmp, bad, transit, lift=lift, urdf_hash=urdf_hash)
            with case("fk/certify-breach", {}):
                result = certify(npz_bad, meta_bad, urdf_file=capture)
                # A breach dominates the coverage verdict.
                self.assertEqual(result["verdict"], "fail")
                self.assertTrue(result["breaches"])
                breach = result["breaches"][0]
                self.assertEqual(breach["kind"], "contact")
                self.assertGreater(breach["pos_err_m"], 0.002)

    def test_certify_refuses_without_capture(self):
        script = os.path.join(
            PKG_DIR, "..", "..", "scripts", "certify_atlas_seeds.py")
        certify = runpy.run_path(script)["certify"]
        with tempfile.TemporaryDirectory() as tmp:
            npz, meta_path = self._artifact(
                tmp, self.contact_joints, self.contact_joints)
            with case("fk/no-capture", {}):
                result = certify(npz, meta_path)
                self.assertEqual(result["verdict"], "not_evaluated")
                self.assertIsNone(result["urdf_hash_ok"])

    def test_certify_refuses_v1(self):
        script = os.path.join(
            PKG_DIR, "..", "..", "scripts", "certify_atlas_seeds.py")
        certify = runpy.run_path(script)["certify"]
        atlas = synthetic_atlas(contract=False, directed=False)
        with tempfile.TemporaryDirectory() as tmp:
            npz = os.path.join(tmp, "v1.npz")
            meta_path = os.path.join(tmp, "v1.yaml")
            atlas.save(npz, meta_path)
            with case("fk/v1-refused", {}):
                result = certify(npz, meta_path)
                self.assertEqual(result["verdict"], "not_evaluated")


class TestTaskGridIsolation(unittest.TestCase):

    def test_task_grid_writes_v1_marker(self):
        from luggage_planning.reachable_task_grid import (
            V1_SEMANTICS, load_task_grid, save_task_grid,
        )
        with tempfile.TemporaryDirectory() as tmp:
            npz = os.path.join(tmp, "grid.npz")
            yaml_path = os.path.join(tmp, "grid.yaml")
            status = np.zeros((2, 2, 2, 2), dtype=np.uint8)
            seeds = np.zeros((2, 2, 2, 2, 1, 6))
            save_task_grid(npz, yaml_path, status, seeds, seeds, {"x": 1})
            with case("taskgrid/v1-marker", {}):
                with open(yaml_path) as handle:
                    meta = yaml.safe_load(handle)
                self.assertEqual(meta["semantics"], V1_SEMANTICS)
                loaded = load_task_grid(npz, yaml_path)
                self.assertEqual(loaded["meta"]["semantics"],
                                 V1_SEMANTICS)


class TestRebuiltArtifacts(unittest.TestCase):
    """The four committed profiles: v2 contract, 4 bins, per-bin coverage.

    These assertions are against the REBUILT artifacts on this branch; they
    are the runtime-side acceptance for the campaign. Before the campaign
    lands they fail on the still-v1 committed files -- that red is the
    work-in-progress marker, not a regression.
    """

    DATA = os.path.join(PKG_DIR, "data", "reachability_atlas")
    PROFILES = (
        "s20_container_collision_aware",
        "s20_container_collision_aware_payload_0.55x0.40x0.25",
        "s20_container_collision_aware_payload_0.70x0.45x0.28",
        "s20_container_collision_aware_payload_0.80x0.50x0.32",
    )

    def test_all_profiles_directed_v2_with_coverage(self):
        from luggage_planning.atlas_frame_contracts import DIRECTED_YAW_BINS
        for profile in self.PROFILES:
            npz = os.path.join(self.DATA, profile + ".npz")
            meta_path = os.path.join(self.DATA, profile + ".yaml")
            with case("artifact/%s" % profile, dict(profile=profile)):
                meta = yaml.safe_load(open(meta_path))
                contract = meta.get("contract")
                self.assertIsInstance(contract, dict, profile)
                self.assertEqual(contract["yaw_period_rad"], TWO_PI)
                self.assertEqual(
                    [round(b, 9) for b in contract["yaw_bins_rad"]],
                    [round(b, 9) for b in DIRECTED_YAW_BINS])
                validate_atlas(contract, {
                    "scene_revision": contract["scene_revision"],
                    "geometry_hash": contract["geometry_hash"],
                    "urdf_hash": contract["urdf_hash"],
                    "payload_hash": contract["payload_hash"]})
                # Per directed bin: at least one sampled reachable seed.
                data = np.load(npz)
                status = np.asarray(data["status"])
                seeds = np.asarray(data["contact_seeds"])
                for iyaw in range(status.shape[3]):
                    slice_status = status[..., iyaw]
                    slice_seeds = seeds[:, :, :, iyaw]
                    seeded_reachable = np.logical_and(
                        slice_status >= MARGINAL,
                        np.isfinite(slice_seeds).any(axis=(-1, -2)))
                    self.assertTrue(
                        seeded_reachable.any(),
                        "%s bin %d carries no sampled reachable seed"
                        % (profile, iyaw))


if __name__ == "__main__":
    unittest.main()
