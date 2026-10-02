"""ST09-A acceptance: the 24 saved slanted-wall candidates are all rejected.

Replay inputs are the frozen rehearsal-52 revision-2 surface and the saved
24 candidate poses (checkpoint evidence, copied into ``fixtures/`` by A0).
The saved poses sit on the HULL-frame window grid (centres on +/-0.745 /
+/-0.985); the solver enumerates the MAP-frame grid (``inner_size``
1.50 x 2.00 — the 3 cm overshoot). Both must be caught by the exact kernel:
each saved pose directly, and every solver-feasible candidate re-checked.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from helpers import PARAMS_FULL, hull_from_descriptor, load_rev2_surface, load_saved24

from luggage_description.container_geometry import (
    contains_oriented_box_through_aperture,
    oriented_box_corners,
    y_max_at_z,
)
from luggage_packing.placement_solver import solve_placement


def _footprint(box_wdh, yaw):
    box_l, box_w, box_h = box_wdh
    rotated = abs((yaw % math.pi) - math.pi / 2.0) < abs((yaw % math.pi))
    return (box_w, box_l, box_h) if rotated else (box_l, box_w, box_h)


class TestSaved24Rejected(object):
    def test_every_saved_pose_fails_the_exact_kernel(self):
        saved = load_saved24()
        hull = hull_from_descriptor()
        assert len(saved["candidates"]) == 24
        for row in saved["candidates"]:
            size = _footprint(saved["box_m"], row["yaw"])
            legal = contains_oriented_box_through_aperture(
                hull, (row["x"], row["y"], row["z"]), size, yaw=0.0)
            assert not legal, (
                "saved candidate (%.3f, %.3f, yaw %.2f) must violate the "
                "seven-face hull" % (row["x"], row["y"], row["yaw"]))
            assert row["gate_pass"] is False

    def test_saved_violation_numbers_reproduce_independently(self):
        """Bottom-corner chamfer violation recomputed from the descriptor.

        The saved evidence records 444.6 mm (yaw 0) / 463.1 mm (yaw 90) as
        the binding bottom-corner embedding into the chamfer; recompute it
        from the kernel here rather than trusting the record.
        """
        saved = load_saved24()
        hull = hull_from_descriptor()
        for row in saved["candidates"]:
            size = _footprint(saved["box_m"], row["yaw"])
            corners = oriented_box_corners(
                (row["x"], row["y"], row["z"]), size, yaw=0.0)
            bottom = [c for c in corners if c[2] < row["z"] - 1e-9]
            violation = max(
                c[1] - y_max_at_z(hull, c[2]) for c in bottom)
            assert abs(violation * 1000.0 - row["chamfer_bottom_mm"]) < 0.2, (
                "chamfer bottom violation drift at (%.3f, %.3f): %.3f mm vs "
                "saved %.1f mm"
                % (row["x"], row["y"], violation * 1000.0,
                   row["chamfer_bottom_mm"]))

    def test_saved_bounds_match_recorded_extremes(self):
        saved = load_saved24()
        assert saved["violation_mm"]["chamfer_bottom_corner"]["min"] == 444.6
        assert saved["violation_mm"]["chamfer_bottom_corner"]["max"] == 463.1
        assert saved["gate_pass_count"] == 0


class TestSolverReplayOnRev2(object):
    def test_solver_produces_zero_false_legal_and_rejects_the_rim(self):
        surface = load_rev2_surface()
        saved = load_saved24()
        hull = hull_from_descriptor()
        result = solve_placement(
            surface, saved["box_m"], allowed_yaws=[0.0, math.pi / 2.0],
            params=dict(PARAMS_FULL, keep_rejected=1200, top_n=1200))
        # Every candidate the solver calls feasible must pass the exact
        # kernel — zero false-legal slots on the starved map.
        feasible = [c for c in result["candidates"] if c["feasible"]]
        assert feasible, "interior floor-prior legal slots must exist"
        for candidate in feasible:
            center = tuple(candidate["center_container"])
            size = tuple(candidate["footprint"]) + (
                candidate["size"][2],)
            assert contains_oriented_box_through_aperture(
                hull, center, size, yaw=0.0), (
                "false-legal candidate %s at %r"
                % (candidate["candidate_id"], center))
        # The rim band (the free row y in [0.965, 1.015] the 24 saved
        # candidates anchored on) is entirely hull-rejected. Map-frame
        # window centres top out at y=0.775, so identify rim windows by
        # footprint coverage of the top grid rows (iy0 + cells_y == ny).
        assert result["reject_histogram"].get("outside_hull", 0) >= 24
        rim = [
            c for c in result["candidates"]
            if c["iy0"] + c["cells_y"] >= surface["ny"]
        ]
        assert len(rim) >= 24, "rim windows must be enumerated and rejected"
        assert all(not c["feasible"] for c in rim)
        assert all(c["reason"] == "outside_hull" for c in rim)
        assert all(c["verdict"] == "hull_violation" for c in rim)

    def test_selected_is_legal_deep_floor_prior_with_requirement(self):
        """Post-fix behaviour on the starved map: the rim dies, a legal deep
        floor-prior slot wins (floor-first preserved, deep-first applied),
        and the still-unknown footprint/corridor cells are emitted as an
        ObservationRequirement for the bounded observation stage."""
        surface = load_rev2_surface()
        saved = load_saved24()
        result = solve_placement(
            surface, saved["box_m"], allowed_yaws=[0.0, math.pi / 2.0],
            params=PARAMS_FULL)
        assert result["success"] is True
        selected = result["selected"]
        assert selected["verdict"] == "legal"
        assert selected["reason"] == "ok"
        # floor-first: the floor layer wins over any stacked support
        assert selected["support_layer_z"] == 0.0
        # deep-first: deepest legal floor slot, not the shallowest score win
        assert selected["depth_m"] > 0.9
        # unobserved interior is floor_prior, never faked as observed free
        assert selected["support_source"] == "floor_prior"
        assert selected["confidence_ratio"] == 0.0
        requirement = result["observation_requirement"]
        assert requirement is not None
        assert requirement["missing_cells"], (
            "the selected footprint/corridor is unobserved")
        assert all(
            surface["state"][ix][iy] == "unknown"
            for ix, iy in requirement["missing_cells"])

    def test_feasible_centers_respect_hull_valid_y_limits(self):
        """Independent expectation: at this box height the hull admits
        centre y <= 0.345 (yaw 0) / 0.227 (yaw 90)."""
        surface = load_rev2_surface()
        saved = load_saved24()
        result = solve_placement(
            surface, saved["box_m"], allowed_yaws=[0.0, math.pi / 2.0],
            params=PARAMS_FULL)
        limits = saved["hull_valid_center_limits"]
        for candidate in result["candidates"]:
            if not candidate["feasible"]:
                continue
            y = candidate["center_container"][1]
            if candidate["footprint_class"] == 0:
                assert y <= limits["yaw0"] + 1e-6
            else:
                assert y <= limits["yaw90"] + 1e-6
