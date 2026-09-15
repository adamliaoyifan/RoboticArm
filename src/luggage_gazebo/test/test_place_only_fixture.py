#!/usr/bin/env python3
"""Unit tests for the POS-1 place-only fixture library.

Covers the plan's required focused tests: fixture construction, perfect
descriptor isolation, no-pick/no-detect guard logic, commit
ordering/idempotency, occupancy diff scoring, P3/P4 failure classification,
and dump completeness. Pure Python; the container hull matches the
calibrated ``scene_tf.yaml.example`` container section.
"""

from __future__ import division

import copy
import json
import math
import os
import tempfile

import pytest

from luggage_gazebo import place_only_fixture as fx

SCENE_CONFIG = {
    "container": {
        "inner": {
            "length": 1.49,
            "width": 1.97,
            "floor_z": 0.53,
            "ceiling_z": 2.01,
            "chamfer": {"side": "positive_y", "floor_y": 0.55,
                        "wall_z": 0.90},
        },
    },
}


@pytest.fixture(scope="module")
def ctx():
    hull = fx.hull_from_scene_config(SCENE_CONFIG)
    return fx.hull_context(hull, hull.floor_z)


def _surface(resolution=0.05, height_value=0.0):
    inner_l, inner_w, inner_h = 1.49, 1.97, 1.48
    nx = int(round(inner_l / resolution))
    ny = int(round(inner_w / resolution))
    return {
        "resolution": resolution,
        "nx": nx, "ny": ny,
        "inner_size": [inner_l, inner_w, inner_h],
        "floor_z": 0.0,
        "map_revision": 0,
        "height": [[height_value] * ny for _ in range(nx)],
        "state": [["unknown"] * ny for _ in range(nx)],
    }


# ---------------------------------------------------------------------------
# Case matrix + fixture construction
# ---------------------------------------------------------------------------

def test_case_matrix_order_and_expectations():
    cases = fx.case_matrix()
    assert [c.case_id for c in cases] == ["P0", "P1", "P2", "P3", "P4"]
    by_id = {c.case_id: c for c in cases}
    assert [by_id[i].cargo_id for i in ("P0", "P1", "P2", "P3", "P4")] == [
        "carryon", "standard", "large", "carryon", "standard"]
    assert [by_id[i].initial for i in ("P0", "P1", "P2", "P3", "P4")] == [
        "empty", "carry", "carry", "saturated", "obstacle"]
    assert [by_id[i].expect_place for i in cases_ids()] == [
        True, True, True, False, True]
    assert by_id["P3"].fail_closed_codes == (
        fx.REASON_BIN_FULL, fx.REASON_NO_CANDIDATE)
    assert by_id["P4"].expect_committed_after == 1


def cases_ids():
    return ["P0", "P1", "P2", "P3", "P4"]


def test_saturated_fixture_covers_floor_and_blocks_capacity(ctx):
    boxes = fx.saturated_fixture_boxes(
        ctx, fx.CATALOG_SIZES["carryon"])
    assert len(boxes) == 2
    aabbs = [box.aabb() for box in boxes]
    # Slabs are hull-legal and adjacent (no gap along x).
    assert aabbs[0][3] == pytest.approx(aabbs[1][0], abs=1e-6)
    for box in boxes:
        assert ctx["contains_floor_box"](box.center, box.size, box.yaw)
    # Independent capacity test: no carryon footprint fits anywhere.
    fits, first = fx.geometric_capacity(
        ctx, fx.CATALOG_SIZES["carryon"], aabbs)
    assert fits is False
    assert first is None


def test_obstacle_fixture_leaves_tempting_slot_and_blocks_sweep(ctx):
    boxes = fx.obstacle_fixture_boxes(ctx, fx.CATALOG_SIZES["standard"])
    roles = {box.role: box for box in boxes}
    assert set(roles) == {"path_obstacle", "front_low_box"}
    wall = roles["path_obstacle"]
    low = roles["front_low_box"]
    assert wall.in_map is False and low.in_map is True
    for box in boxes:
        assert ctx["contains_floor_box"](box.center, box.size, box.yaw)
    # Standard still fits geometrically behind the blind wall (map-blind).
    fits, first = fx.geometric_capacity(
        ctx, fx.CATALOG_SIZES["standard"], [low.aabb()])
    assert fits is True
    # A deep slot beyond the wall is swept-path blocked at the map-derived
    # traverse height (the corridor raise cannot see the blind wall).
    behind = [wall.aabb()[3] + 0.25, wall.center[1], 0.14]
    traverse_z = fx.corridor_traverse_z(
        ctx, behind, fx.CATALOG_SIZES["standard"], [low])
    blocked, hits = fx.swept_path_blocked(
        ctx, behind, fx.CATALOG_SIZES["standard"], 0.0,
        [box.aabb() for box in boxes], traverse_contact_z=traverse_z)
    assert blocked is True and hits
    assert hits[0]["stage"] == "traverse"
    # A slot stacked on the low front box is sweep-free (corridor raise
    # comes from the committed map, which knows the low box).
    stacked = [low.center[0], low.center[1], low.size[2] + 0.14]
    traverse_z2 = fx.corridor_traverse_z(
        ctx, stacked, fx.CATALOG_SIZES["standard"], [low])
    blocked2, hits2 = fx.swept_path_blocked(
        ctx, stacked, fx.CATALOG_SIZES["standard"], 0.0,
        [box.aabb() for box in boxes], traverse_contact_z=traverse_z2)
    assert blocked2 is False and not hits2


def test_enumeration_is_deterministic(ctx):
    aabbs = [box.aabb() for box in fx.obstacle_fixture_boxes(
        ctx, fx.CATALOG_SIZES["standard"])]
    first = fx.enumerate_footprints(ctx, fx.CATALOG_SIZES["standard"], aabbs)
    second = fx.enumerate_footprints(ctx, fx.CATALOG_SIZES["standard"], aabbs)
    assert first == second
    assert any(c["feasible"] for c in first)


def test_hull_clearance_rejects_tight_candidates(ctx):
    # Floor-level usable Y span is [-0.985, 0.55] (chamfer on +Y).
    wide = (1.00, 1.50, 0.25)
    # Centered on the usable span: inside.
    assert ctx["contains_floor_box"]((0.0, -0.235, 0.125), wide, 0.0) is True
    # Shifted toward the chamfered +Y side: outside the seven-face hull.
    assert ctx["contains_floor_box"]((0.0, 0.0, 0.125), wide, 0.0) is False


def test_lateral_hull_margin_allows_floor_contact_but_not_wall_contact(ctx):
    size = fx.CATALOG_SIZES["carryon"]
    center = (-0.465, -0.50, 0.125)  # 5 mm off the -X wall, on the floor
    # Strict containment passes (inside the hull at all).
    assert ctx["contains_floor_box"](center, size, 0.0) is True
    # Isotropic kernel margin would fail it for touching the FLOOR; the
    # lateral check only fails it for being <10 mm from the -X wall.
    assert ctx["contains_floor_box_lateral"](center, size, 0.0,
                                             margin=0.010) is False
    deep = (-0.455, -0.50, 0.125)    # 15 mm off the -X wall
    assert ctx["contains_floor_box_lateral"](deep, size, 0.0,
                                             margin=0.010) is True
    # Enumeration honors the configured lateral margin.
    ctx_strict = fx.hull_context(ctx["hull"], ctx["floor_z"])
    ctx_strict["lateral_margin"] = 0.010
    aabbs = [box.aabb() for box in fx.saturated_fixture_boxes(
        ctx_strict, size)]
    fits, _first = fx.geometric_capacity(ctx_strict, size, aabbs)
    assert fits is False


# ---------------------------------------------------------------------------
# Perfect descriptor isolation
# ---------------------------------------------------------------------------

def test_perfect_descriptor_declared_only():
    fields = fx.perfect_descriptor_fields(
        "carryon", fx.CATALOG_SIZES["carryon"], (1.2, -0.4, 0.10), 0.25)
    assert fields["input_source"] == fx.INPUT_SOURCE
    assert fields["size_wdh"] == [0.55, 0.40, 0.25]
    assert fields["height_valid"] is True
    assert fields["height_source"] == 2
    assert fields["top_surface_valid"] is True
    assert fields["top_surface_z"] == pytest.approx(0.10 + 0.125)
    assert fields["yaw"] == 0.25 and fields["yaw_valid"] is True
    # Isolation: the descriptor never carries sim/GT provenance beyond the
    # declared numbers; a different declared size produces a different
    # descriptor with no shared mutable state.
    other = fx.perfect_descriptor_fields(
        "large", fx.CATALOG_SIZES["large"], (1.2, -0.4, 0.10), 0.25)
    assert other["size_wdh"] == [0.80, 0.50, 0.32]
    assert fields["size_wdh"] == [0.55, 0.40, 0.25]


def test_case_manifest_records_input_source():
    case = fx.case_matrix()[0]
    manifest = case.manifest(fx.CATALOG_SIZES[case.cargo_id])
    assert manifest["input_source"] == fx.INPUT_SOURCE
    assert manifest["cargo_size_wdh"] == [0.55, 0.40, 0.25]
    assert manifest["case_id"] == "P0"


# ---------------------------------------------------------------------------
# Isolation guards (no-pick / no-detect)
# ---------------------------------------------------------------------------

def test_unexpected_segment_names_guard():
    allowed = ["transit", "traverse", "insert", "descend", "retreat",
               "stage", "setup_pre_over_box", "setup_attach"]
    assert fx.unexpected_segment_names(allowed) == []
    polluted = allowed + ["pre_grasp", "approach", "attach", "pick_retreat"]
    assert fx.unexpected_segment_names(polluted) == [
        "approach", "attach", "pick_retreat", "pre_grasp"]


def test_pick_phase_names_are_not_place_segments():
    for name in ("pre_grasp", "approach", "attach", "pick_retreat"):
        assert name not in fx.PLACE_SEGMENT_NAMES
        assert name not in fx.SETUP_SEGMENT_NAMES


# ---------------------------------------------------------------------------
# Commit ordering + idempotency
# ---------------------------------------------------------------------------

def test_commit_order_valid_and_invalid():
    ok = [("release", 10.0), ("retreat", 10.5), ("verify", 11.0),
          ("commit", 11.5)]
    assert fx.validate_commit_order(ok)[0] is True
    # Equal timestamps are tolerated (probe chains inside one sim tick).
    assert fx.validate_commit_order([
        ("release", 10.0), ("retreat", 10.0), ("verify", 10.0),
        ("commit", 10.0)])[0] is True
    assert fx.validate_commit_order([
        ("release", 10.0), ("retreat", 10.5), ("verify", 11.0),
        ("commit", 10.6)])[0] is False
    assert fx.validate_commit_order([
        ("release", 10.0), ("retreat", 10.5), ("commit", 11.0)])[0] is False
    assert fx.validate_commit_order([
        ("release", 10.0), ("retreat", 10.5), ("verify", 11.0),
        ("commit", 11.5), ("commit", 12.0)])[0] is False


def test_idempotency_check():
    base = {"map_revision": 3, "committed_box_count": 1}
    same = {"map_revision": 3, "committed_box_count": 1}
    assert fx.idempotency_check(base, same, "d1", "d1")["ok"] is True
    bumped = {"map_revision": 4, "committed_box_count": 2}
    verdict = fx.idempotency_check(base, bumped, "d1", "d2")
    assert verdict["ok"] is False
    assert set(verdict["diffs"]) == {
        "map_revision", "committed_box_count", "digest"}


# ---------------------------------------------------------------------------
# Occupancy diff scoring
# ---------------------------------------------------------------------------

def test_occupancy_diff_scores_commit_footprint():
    pre = _surface()
    post = _surface()
    center = (0.10, -0.30, 0.125)   # carryon center, floor-relative
    size = fx.CATALOG_SIZES["carryon"]
    res = post["resolution"]
    for ix, iy in fx.gt_footprint_cells(post, center, size, 0.0):
        post["height"][ix][iy] = 0.25
        post["state"][ix][iy] = "occupied"
    diff = fx.occupancy_diff(pre, post, center, size, 0.0)
    assert diff["iou"] == pytest.approx(1.0)
    assert diff["iou_ok"] is True
    assert diff["outside_footprint_changes"] == 0
    assert diff["height_within_one_cell"] is True
    assert diff["max_height_error_m"] <= res + 1e-9


def test_occupancy_diff_flags_outside_mutation_and_bad_height():
    pre = _surface()
    post = _surface()
    center = (0.0, 0.0, 0.125)
    size = (0.30, 0.30, 0.25)
    footprint = sorted(fx.gt_footprint_cells(post, center, size, 0.0))
    assert footprint
    inside_a, inside_b = footprint[0], footprint[len(footprint) // 2]
    post["height"][0][0] = 9.9              # far outside any footprint
    post["height"][inside_a[0]][inside_a[1]] = 0.25
    post["height"][inside_b[0]][inside_b[1]] = 0.40  # wrong height
    diff = fx.occupancy_diff(pre, post, center, size, 0.0)
    assert diff["outside_footprint_changes"] >= 1
    assert diff["height_within_one_cell"] is False


def test_surface_digest_changes_only_with_content():
    a = _surface()
    b = _surface()
    assert fx.surface_digest(a) == fx.surface_digest(b)
    b["height"][0][0] = 0.25
    assert fx.surface_digest(a) != fx.surface_digest(b)
    c = _surface()
    c["map_revision"] = 1
    assert fx.surface_digest(a) != fx.surface_digest(c)


def test_gt_footprint_rotates_with_yaw():
    surface = _surface()
    cells0 = fx.gt_footprint_cells(surface, (0.0, 0.0, 0.125),
                                   (0.60, 0.30, 0.25), 0.0)
    cells90 = fx.gt_footprint_cells(surface, (0.0, 0.0, 0.125),
                                    (0.60, 0.30, 0.25), math.pi / 2.0)
    assert cells0 and cells90
    # Rotation is area-preserving on the axis-aligned grid.
    assert len(cells0) == pytest.approx(len(cells90), rel=0.15)
    assert cells0 != cells90


# ---------------------------------------------------------------------------
# P3/P4 failure classification
# ---------------------------------------------------------------------------

def test_bin_full_accepted_only_with_capacity_proof():
    message = "BIN_FULL no_candidate: overlap=210 insufficient_clearance=40"
    verdict = fx.classify_placement_failure(message, False, {"overlap": 210})
    assert verdict["ok"] is True
    assert verdict["code"] == fx.REASON_BIN_FULL
    assert verdict["capacity_confirmed"] is True


def test_candidate_exhaustion_not_relabelled_bin_full():
    message = "BIN_FULL no_candidate: outside_aperture=210"
    verdict = fx.classify_placement_failure(message, True)
    assert verdict["ok"] is False
    assert verdict["code"] == "PLACE_CANDIDATE_EXHAUSTED"
    assert verdict["bin_full_accepted"] is False


def test_unexpected_failure_shape_rejected():
    verdict = fx.classify_placement_failure("DETECT_FULL_GEOMETRY_REQUIRED",
                                            False)
    assert verdict["ok"] is False


# ---------------------------------------------------------------------------
# Dump completeness + replay determinism
# ---------------------------------------------------------------------------

def test_dump_artifacts_complete(tmp_path):
    case_dir = tmp_path / "P0_ok"
    case_dir.mkdir()
    verdict = fx.dump_artifacts_complete(str(case_dir))
    assert verdict["capture_complete"] is False
    assert set(fx.CASE_REQUIRED_ARTIFACTS) <= set(verdict["missing"])
    for name in fx.CASE_REQUIRED_ARTIFACTS:
        (case_dir / name).write_text("{}\n")
    verdict = fx.dump_artifacts_complete(str(case_dir))
    assert verdict["capture_complete"] is True
    assert verdict["replay_possible"] is True
    (case_dir / "t1_trace.jsonl").write_text("")
    verdict = fx.dump_artifacts_complete(str(case_dir))
    assert verdict["capture_complete"] is False
    assert verdict["empty"] == ["t1_trace.jsonl"]


def test_replay_candidates_deterministic(ctx):
    surface = _surface()
    aabbs = [box.aabb() for box in fx.saturated_fixture_boxes(
        ctx, fx.CATALOG_SIZES["carryon"])]
    first = fx.replay_candidates(surface, fx.CATALOG_SIZES["carryon"],
                                 aabbs, ctx)
    second = fx.replay_candidates(surface, fx.CATALOG_SIZES["carryon"],
                                  aabbs, ctx)
    assert first == second
    assert all(entry["reason"] in ("overlap", "insufficient_clearance",
                                   "outside_hull", "ok")
               for entry in first.values())
    assert not any(entry["feasible"] for entry in first.values())


# ---------------------------------------------------------------------------
# Independent capacity test cross-checks
# ---------------------------------------------------------------------------

def test_capacity_positive_control_empty_container(ctx):
    fits, first = fx.geometric_capacity(
        ctx, fx.CATALOG_SIZES["carryon"], [])
    assert fits is True
    assert first["reason"] == "ok"
    assert ctx["contains_floor_box"](
        first["center"], first["footprint"] + (0.25,), first["yaw"])


def test_support_touching_neighbours_are_not_overlap():
    # P0/P1 dry-run geometry: carryon top edge exactly at the standard's
    # bottom edge. Face contact must not read as footprint overlap.
    carryon = fx.FixtureBox((-0.42, -0.685, 0.125), (0.55, 0.40, 0.25))
    touching = (-0.345, -0.26, 0.14)
    assert fx.box_overlaps_aabb(
        touching, (0.70, 0.45, 0.28), 0.0, carryon.aabb()) is False
    intruding = (-0.345, -0.265, 0.14)   # 5 mm into the carryon's Y span
    assert fx.box_overlaps_aabb(
        intruding, (0.70, 0.45, 0.28), 0.0, carryon.aabb()) is True


def test_capacity_matches_matrix_manifest_sizes(ctx):
    for cargo in ("carryon", "standard", "large"):
        fits, _ = fx.geometric_capacity(
            ctx, fx.CATALOG_SIZES[cargo], [])
        assert fits is True, cargo


def test_swept_path_hull_check(ctx):
    # A carry whose height pokes above the ceiling must fail the hull sweep.
    size = fx.CATALOG_SIZES["carryon"]
    assert ctx["contains_floor_sweep"](
        (-0.4, -0.3, 1.42), (0.4, -0.3, 1.42), size, 0.0) is False
    assert ctx["contains_floor_sweep"](
        (-0.4, -0.3, 0.6), (0.4, -0.3, 0.6), size, 0.0) is True


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
