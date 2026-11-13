"""ST09-A acceptance: exact geometry — centers and eight corners against an
independent scipy expectation at 1e-9, footprint classes, theta+pi gating
and q/-q, pi/-pi representation equivalence (never rejected)."""

import math
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(__file__))

from helpers import BOX_WDH, PARAMS_FULL, chamfered_surface

from luggage_description.container_geometry import oriented_box_corners
from luggage_packing.placement_solver import (
    SnapshotCandidateGenerator,
    generate_candidates,
    solve_placement,
)

TOLERANCE = 1e-9


def _scipy_corners(center, size, theta):
    half = np.asarray(size, dtype=float) * 0.5
    offsets = np.array([
        [dx, dy, dz]
        for dx in (-half[0], half[0])
        for dy in (-half[1], half[1])
        for dz in (-half[2], half[2])
    ])
    rotation = Rotation.from_euler("z", theta)
    return (rotation.apply(offsets) + np.asarray(center, dtype=float)).tolist()


class TestExactGeometry(object):
    def test_kernel_corners_match_scipy(self):
        surface = chamfered_surface()
        result = solve_placement(
            surface, list(BOX_WDH), allowed_yaws=[0.0, math.pi / 2.0],
            params=PARAMS_FULL)
        checked = 0
        for candidate in result["candidates"]:
            center = candidate["center_container"]
            # The kernel stores the already-rotated footprint dims at yaw 0;
            # the independent expectation rotates the ORIGINAL box size by
            # the heading, which also cross-checks the footprint class.
            kernel = sorted(map(tuple, oriented_box_corners(
                center, list(candidate["footprint"]) + [candidate["size"][2]],
                yaw=0.0)))
            expect = sorted(map(tuple, _scipy_corners(
                center, list(candidate["size"]),
                candidate["heading_theta"] % math.pi)))
            assert len(kernel) == len(expect) == 8
            for got, want in zip(kernel, expect):
                assert max(abs(a - b) for a, b in zip(got, want)) <= TOLERANCE
            checked += 1
            if checked >= 40:
                break
        assert checked == 40

    def test_center_matches_independent_window_arithmetic(self):
        surface = chamfered_surface()
        resolution = surface["resolution"]
        half_l = 0.5 * surface["inner_size"][0]
        half_w = 0.5 * surface["inner_size"][1]
        floor_z = 0.53
        candidates = generate_candidates(
            surface, list(BOX_WDH), allowed_yaws=[0.0], params=PARAMS_FULL)
        for candidate in candidates[:40]:
            expect_x = (
                -half_l + (candidate["ix0"] + candidate["cells_x"] * 0.5)
                * resolution)
            expect_y = (
                -half_w + (candidate["iy0"] + candidate["cells_y"] * 0.5)
                * resolution)
            expect_z = floor_z + 0.0 + BOX_WDH[2] * 0.5
            got = candidate["center_container"]
            assert abs(got[0] - expect_x) <= TOLERANCE
            assert abs(got[1] - expect_y) <= TOLERANCE
            assert abs(got[2] - expect_z) <= TOLERANCE

    def test_footprint_classes_map_dimensions(self):
        surface = chamfered_surface()
        candidates = generate_candidates(
            surface, list(BOX_WDH), allowed_yaws=[0.0, math.pi / 2.0],
            params=PARAMS_FULL)
        classes = {c["footprint_class"] for c in candidates}
        assert classes == {0, 1}
        for candidate in candidates:
            length, width = candidate["footprint"]
            if candidate["footprint_class"] == 0:
                assert abs(length - BOX_WDH[0]) <= TOLERANCE
                assert abs(width - BOX_WDH[1]) <= TOLERANCE
                assert abs(candidate["heading_theta"] % math.pi) <= TOLERANCE
            else:
                assert abs(length - BOX_WDH[1]) <= TOLERANCE
                assert abs(width - BOX_WDH[0]) <= TOLERANCE
                assert abs(
                    (candidate["heading_theta"] % math.pi) - math.pi / 2.0
                ) <= TOLERANCE


class TestHeadingExpansion(object):
    def test_no_symmetry_single_heading(self):
        surface = chamfered_surface()
        candidates = generate_candidates(
            surface, list(BOX_WDH), allowed_yaws=[0.0],
            params=dict(PARAMS_FULL, payload_symmetry="none"))
        ids = [c["candidate_id"] for c in candidates]
        assert ids, "healthy positive case must produce candidates"
        assert all(not i.endswith("_h1") for i in ids)

    def test_pi_symmetry_expands_both_headings(self):
        surface = chamfered_surface()
        candidates = generate_candidates(
            surface, list(BOX_WDH), allowed_yaws=[0.0],
            params=dict(PARAMS_FULL, payload_symmetry="pi"))
        by_window = {}
        for candidate in candidates:
            key = (candidate["ix0"], candidate["iy0"], candidate["footprint_class"])
            by_window.setdefault(key, []).append(candidate)
        populated = [rows for rows in by_window.values() if rows]
        assert populated
        for rows in populated[:20]:
            headings = sorted(r["heading_theta"] for r in rows)
            assert any(
                abs(h - math.pi) <= 1e-9 for h in headings), (
                "theta+pi must be emitted under declared pi symmetry")
            # same footprint, distinct physical rotations, distinct ids
            assert len({r["candidate_id"] for r in rows}) == len(rows)

    def test_pi_minus_pi_yaws_are_equivalent(self):
        surface = chamfered_surface()
        base = generate_candidates(
            surface, list(BOX_WDH), allowed_yaws=[math.pi],
            params=PARAMS_FULL)
        mirror = generate_candidates(
            surface, list(BOX_WDH), allowed_yaws=[-math.pi],
            params=PARAMS_FULL)
        key = lambda rows: sorted(
            (round(c["center_container"][0], 6), round(c["center_container"][1], 6),
             c["footprint_class"], round(c["score"], 6), c["candidate_id"])
            for c in rows)
        assert key(base) == key(mirror), (
            "pi and -pi are the same rotation and must not diverge")


class TestAdapterQuaternionEquivalence(object):
    def test_adapter_quaternion_reproduces_corners_and_q_neg_q(self):
        surface = chamfered_surface()
        payload = _payload()
        generator = SnapshotCandidateGenerator(
            payload=payload,
            geometry_descriptor=surface["geometry_descriptor"],
            allowed_yaws=[0.0, math.pi / 2.0],
            params=PARAMS_FULL,
        )
        from luggage_planning.planning_contract.fixtures import make_snapshot
        candidates, _requirement = generator.generate_candidates(
            make_snapshot(map_revision=1), payload.digest())
        assert candidates
        from luggage_planning.planning_contract.fixtures import BOX_WDH as FIXTURE_BOX
        for candidate in candidates[:40]:
            quaternion = candidate.quaternion_xyzw_container
            # q and -q are the same rotation: both must reconstruct the
            # same corner cloud as rotating the ORIGINAL box dims by the
            # heading (the footprint class must not be applied twice).
            negated = tuple(-v for v in quaternion)
            size = list(FIXTURE_BOX)
            for rotation_quaternion in (quaternion, negated):
                matrix = Rotation.from_quat(rotation_quaternion).as_matrix()
                half = np.asarray(size) * 0.5
                offsets = np.array([
                    [dx, dy, dz]
                    for dx in (-half[0], half[0])
                    for dy in (-half[1], half[1])
                    for dz in (-half[2], half[2])
                ])
                rotated = offsets @ matrix.T + np.asarray(
                    candidate.position_container)
                expect = np.asarray(_scipy_corners(
                    candidate.position_container, size,
                    candidate.heading_theta))
                rotated = np.sort(np.asarray(rotated), axis=0)
                expect_sorted = np.sort(expect, axis=0)
                assert np.max(np.abs(rotated - expect_sorted)) <= 1e-9


def _payload():
    from luggage_planning.planning_contract.fixtures import make_attachment, make_payload
    return make_payload(make_attachment("centered"))
