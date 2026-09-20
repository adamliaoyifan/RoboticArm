#!/usr/bin/env python3
"""Placement failure taxonomy: BIN_FULL is a capacity claim, nothing else.

Capacity gates are overlap, top clearance and hull containment. Aperture,
insertion corridor, unobserved support and insufficient support ratio are
policy/observation gates and must never read as a full container
(docs/architecture/container_geometry.md).
"""

from luggage_packing.placement_solver import (
    CODE_BIN_FULL,
    CODE_BOX_EXCEEDS_CONTAINER,
    CODE_INVALID_BOX_SIZE,
    CODE_PLACE_CANDIDATE_EXHAUSTED,
    placement_constraint_verdict,
    solve_placement,
)

BOX = [0.2, 0.2, 0.2]
PARAMS = {"top_n": 100, "keep_rejected": 100}


def surface(height=0.0, state="free", inner_h=0.5, nx=6, ny=4,
            resolution=0.1):
    return {
        "map_revision": 1,
        "resolution": resolution,
        "nx": nx,
        "ny": ny,
        "inner_size": [nx * resolution, ny * resolution, inner_h],
        "floor_z": 0.0,
        "center_base": [0.0, 0.0, inner_h * 0.5],
        "yaw": 0.0,
        "height": [[height] * ny for _ in range(nx)],
        "state": [[state] * ny for _ in range(nx)],
        "confidence": [["sensor"] * ny for _ in range(nx)],
    }


def validator(placed_aabbs=None, aperture_y=None, inner_size=None,
              smallest_size=None, inner_h=0.5):
    def _validate(candidate):
        return placement_constraint_verdict(
            candidate, inner_h, placed_aabbs=placed_aabbs,
            aperture_y=aperture_y, inner_size=inner_size,
            smallest_size=smallest_size)
    return _validate


def test_invalid_size_is_not_a_capacity_claim():
    result = solve_placement(surface(), [0.2, 0.0, 0.2], params=PARAMS)
    assert result["reason_code"] == CODE_INVALID_BOX_SIZE
    assert result["capacity_feasible_count"] == 0


def test_box_larger_than_container_is_not_a_full_bin():
    result = solve_placement(surface(), [0.7, 0.2, 0.2], allowed_yaws=[0.0],
                             params=PARAMS)
    assert result["reason_code"] == CODE_BOX_EXCEEDS_CONTAINER
    assert result["candidates_enumerated"] == 0


def test_no_headroom_anywhere_is_bin_full():
    # Cargo already stacked to the ceiling: every candidate loses top
    # clearance, which is a capacity gate.
    result = solve_placement(
        surface(height=0.45, state="occupied"), BOX, allowed_yaws=[0.0],
        params=PARAMS)
    assert result["reason_code"] == CODE_BIN_FULL
    assert result["capacity_feasible_count"] == 0
    assert result["reject_histogram"]["insufficient_clearance"] > 0
    assert result["message"].startswith("BIN_FULL no_candidate")


def test_overlap_everywhere_is_bin_full():
    floor = [(-0.3, -0.2, 0.0, 0.3, 0.2, 0.45)]
    result = solve_placement(
        surface(), BOX, allowed_yaws=[0.0], params=PARAMS,
        candidate_validator=validator(placed_aabbs=floor))
    assert result["reason_code"] == CODE_BIN_FULL
    assert result["capacity_feasible_count"] == 0


def test_aperture_only_rejection_is_candidate_exhaustion():
    result = solve_placement(
        surface(), BOX, allowed_yaws=[0.0], params=PARAMS,
        candidate_validator=validator(aperture_y=(-0.001, 0.001)))
    assert result["reason_code"] == CODE_PLACE_CANDIDATE_EXHAUSTED
    assert result["reject_histogram"]["outside_aperture"] > 0
    assert result["capacity_feasible_count"] > 0


def test_corridor_only_rejection_is_candidate_exhaustion():
    # Full-width wall across the opening: slots behind it still fit, they are
    # merely unreachable, so the container is not full.
    wall = [(-0.3, -0.2, 0.0, -0.2, 0.2, 0.5)]
    result = solve_placement(
        surface(), BOX, allowed_yaws=[0.0], params=PARAMS,
        candidate_validator=validator(
            placed_aabbs=wall, inner_size=[0.6, 0.4, 0.5],
            smallest_size=BOX))
    assert result["reason_code"] == CODE_PLACE_CANDIDATE_EXHAUSTED
    assert result["reject_histogram"]["corridor_blocked"] > 0


def test_unobserved_support_is_candidate_exhaustion():
    # Something unseen appears to hold the box at peak > floor. Refusing to
    # trust it is an observation gate, not proof the container is full.
    result = solve_placement(
        surface(height=0.3, state="unknown", inner_h=1.0), BOX,
        allowed_yaws=[0.0], params=PARAMS,
        candidate_validator=validator(inner_h=1.0))
    assert result["reason_code"] == CODE_PLACE_CANDIDATE_EXHAUSTED
    assert result["reject_histogram"]["unknown_above_floor"] > 0
    assert result["capacity_feasible_count"] > 0


def test_insufficient_support_is_candidate_exhaustion():
    # Ridged surface: every footprint straddles a step, so no window has
    # enough contact area. The container still has room above the ridges.
    uneven = surface(state="occupied", inner_h=1.0)
    for ix in range(uneven["nx"]):
        for iy in range(uneven["ny"]):
            uneven["height"][ix][iy] = 0.3 if iy % 2 else 0.0
    result = solve_placement(
        uneven, BOX, allowed_yaws=[0.0],
        params=dict(PARAMS, min_support_ratio=0.99),
        candidate_validator=validator(inner_h=1.0))
    assert result["reason_code"] == CODE_PLACE_CANDIDATE_EXHAUSTED
    assert result["reject_histogram"]["insufficient_support"] > 0


def test_capacity_count_is_taken_before_retention_truncation():
    wall = [(-0.3, -0.2, 0.0, -0.2, 0.2, 0.5)]
    argv = dict(
        surface_map=surface(), box_size=BOX, allowed_yaws=[0.0],
        candidate_validator=validator(
            placed_aabbs=wall, inner_size=[0.6, 0.4, 0.5],
            smallest_size=BOX))
    full = solve_placement(params=PARAMS, **argv)
    clipped = solve_placement(
        params={"top_n": 1, "keep_rejected": 1}, **argv)
    assert clipped["reason_code"] == full["reason_code"]
    assert clipped["capacity_feasible_count"] == \
        full["capacity_feasible_count"]
    assert len(clipped["candidates"]) < clipped["candidates_enumerated"]


def test_capacity_verdict_is_independent_of_gate_order():
    # Outside the aperture and overlapping a placed box: the reported reason
    # is the aperture (report order), but capacity is still False.
    candidate = {
        "center_local": [0.0, 0.0, -0.15],
        "footprint": [0.2, 0.2],
        "size": BOX,
    }
    reason, capacity_ok = placement_constraint_verdict(
        candidate, 0.5, placed_aabbs=[(-0.3, -0.2, 0.0, 0.3, 0.2, 0.45)],
        aperture_y=(-0.001, 0.001))
    assert reason == "outside_aperture"
    assert capacity_ok is False
