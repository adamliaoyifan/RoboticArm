#!/usr/bin/env python3
"""A0 occupancy sweep, variants, and selector (launch-free)."""

import os
import unittest

from luggage_description.container_geometry import (
    cuboid_from_inner_size,
    normalize_descriptor,
)
from luggage_planning.occupancy_place_paths import (
    DEFAULT_WEIGHTS,
    OccupancyMismatch,
    OccupancySnapshot,
    PAYLOAD_SOURCE_DEFAULT,
    PAYLOAD_SOURCE_MEASURED,
    PLACE_PATH_INFEASIBLE,
    exempt_footprint_locals,
    generate_path_variants,
    load_selector_weights,
    occupancy_collision_boxes,
    required_carry_suction_z,
    resolve_payload_wdh,
    select_trajectory,
    suction_ceiling_z,
    sweep_polyline,
)


YAML_PATH = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "config", "place_path_selector.yaml"))


def _surface(nx=20, ny=10, res=0.05, inner=(1.0, 0.5, 1.0),
             center=(0.0, 0.0, 0.5), geom=None, rev=3, height=0.0,
             state="free", hull=None):
    if hull is None:
        hull = cuboid_from_inner_size(inner, floor_z=0.0)
    desc = hull.descriptor()
    return {
        "geometry_hash": desc["geometry_hash"] if geom is None else geom,
        "geometry_descriptor": desc,
        "map_revision": rev,
        "resolution": res,
        "nx": nx,
        "ny": ny,
        "inner_size": list(inner),
        "floor_z": 0.0,
        "center_base": list(center),
        "yaw": 0.0,
        "height": [[height] * ny for _ in range(nx)],
        "state": [[state] * ny for _ in range(nx)],
        "confidence": [["none"] * ny for _ in range(nx)],
    }


def _occupy_column(surface, ix, height=0.40):
    ny = surface["ny"]
    for iy in range(ny):
        surface["state"][ix][iy] = "occupied"
        surface["height"][ix][iy] = height
        surface["confidence"][ix][iy] = "sensor"


class TestOccupancyPlacePaths(unittest.TestCase):
    def test_yaml_weights_pinned(self):
        weights = load_selector_weights(YAML_PATH)
        self.assertAlmostEqual(weights["w_placement"], 1.0)
        self.assertAlmostEqual(weights["w_safety"], 1.5)
        self.assertAlmostEqual(weights["w_efficiency"], 0.5)
        self.assertEqual(weights["w_placement"], DEFAULT_WEIGHTS["w_placement"])
        self.assertAlmostEqual(weights["carry_margin_m"], 0.05)
        self.assertAlmostEqual(weights["collision_pad_m"], 0.02)
        self.assertAlmostEqual(weights["arm_overhead_m"], 0.20)

    def test_default_path_resolves_in_src_tree(self):
        # symlink-install / test runs: module file sits next to ../config.
        from luggage_planning.occupancy_place_paths import (
            selector_weights_path)
        path = selector_weights_path()
        self.assertIsNotNone(path)
        self.assertTrue(os.path.isfile(path))
        self.assertEqual(os.path.basename(path),
                         "place_path_selector.yaml")

    def test_default_path_falls_back_to_installed_share(self):
        # copy-install: module lives in site-packages, the src-relative
        # lookup misses, ament share carries the yaml.
        import shutil
        import tempfile
        import ament_index_python.packages as pkgs
        from luggage_planning import occupancy_place_paths as occ
        here = os.path.dirname(os.path.abspath(__file__))
        fake_module = os.path.join(tempfile.gettempdir(),
                                   "site_packages", "luggage_planning",
                                   "occupancy_place_paths.py")
        share = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, share, True)
        os.makedirs(os.path.join(share, "config"))
        with open(os.path.join(share, "config",
                               "place_path_selector.yaml"), "w",
                  encoding="utf-8") as handle:
            handle.write("inflate_m: 0.07\n")
        original = pkgs.get_package_share_directory
        pkgs.get_package_share_directory = (
            lambda package: self.assertEqual(package, "luggage_planning")
            or share)
        self.addCleanup(setattr, pkgs, "get_package_share_directory",
                        original)
        path = occ.selector_weights_path(module_file=fake_module)
        self.assertIsNotNone(path)
        self.assertIn(share, path)
        # The share yaml actually loads (not silently the defaults).
        weights = occ.load_selector_weights(path)
        self.assertAlmostEqual(weights["inflate_m"], 0.07)

    def test_unresolvable_defaults_warn_on_stderr(self):
        import contextlib
        import io
        from luggage_planning import occupancy_place_paths as occ
        original = occ.selector_weights_path
        occ.selector_weights_path = lambda: None
        self.addCleanup(setattr, occ, "selector_weights_path", original)
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr):
            weights = occ.load_selector_weights()
        self.assertEqual(weights, DEFAULT_WEIGHTS)
        self.assertIn("place_path_selector.yaml not found",
                      stderr.getvalue())
        self.assertIn("DEFAULT_WEIGHTS", stderr.getvalue())

    def test_hash_mismatch_fails_closed(self):
        surface = _surface()
        with self.assertRaises(OccupancyMismatch) as ctx:
            OccupancySnapshot.from_surface_2d(
                surface, expected_hash="other", inflate_m=0.0)
        self.assertIn(PLACE_PATH_INFEASIBLE, str(ctx.exception))

    def test_revision_mismatch_fails_closed(self):
        surface = _surface(rev=3)
        with self.assertRaises(OccupancyMismatch) as ctx:
            OccupancySnapshot.from_surface_2d(
                surface, expected_revision=9, inflate_m=0.0)
        self.assertIn(PLACE_PATH_INFEASIBLE, str(ctx.exception))

    def test_raised_clears_occupied_direct(self):
        surface = _surface()
        _occupy_column(surface, 10, height=0.40)
        snap = OccupancySnapshot.from_surface_2d(surface, inflate_m=0.0)
        portal = (-0.35, 0.0, 0.50)
        target = (0.35, 0.0, 0.50)
        payload = (0.08, 0.08, 0.15)
        direct = sweep_polyline(snap, [portal, target], payload, arm_radius=0.0)
        self.assertTrue(direct.collides, direct)
        raised = [(portal[0], portal[1], 0.65), (target[0], target[1], 0.65)]
        high = sweep_polyline(snap, raised, payload, arm_radius=0.0)
        self.assertFalse(high.collides, high)

    def test_all_infeasible_not_bin_full(self):
        surface = _surface(state="occupied", height=0.9)
        snap = OccupancySnapshot.from_surface_2d(surface, inflate_m=0.0)
        portal = (-0.35, 0.0, 0.50)
        target = (0.35, 0.0, 0.50)
        variants = generate_path_variants(
            portal, target, snapshot=snap, inner_h=1.0, max_candidates=4,
            payload_height=0.15)
        rows = []
        for variant in variants:
            sweep = sweep_polyline(
                snap, variant.waypoints, (0.08, 0.08, 0.15), arm_radius=0.0)
            rows.append({
                "method": variant.method,
                "waypoints": variant.waypoints,
                "feasible": not sweep.collides,
                "min_clearance": sweep.min_clearance,
                "slot_score": 1.0,
                "cartesian_fraction": 1.0,
            })
        winner, rows, reason = select_trajectory(rows)
        self.assertIsNone(winner)
        self.assertEqual(reason, PLACE_PATH_INFEASIBLE)
        self.assertNotEqual(reason, "BIN_FULL")

    def test_selector_prefers_clearance(self):
        long_clear = {
            "method": "offset",
            "waypoints": [(0.0, 0.0, 1.0), (0.4, 0.3, 1.0), (0.8, 0.0, 1.0)],
            "feasible": True,
            "min_clearance": 0.10,
            "slot_score": 1.0,
            "cartesian_fraction": 1.0,
        }
        tight = {
            "method": "direct",
            "waypoints": [(0.0, 0.0, 1.0), (0.8, 0.0, 1.0)],
            "feasible": True,
            "min_clearance": 0.01,
            "slot_score": 1.0,
            "cartesian_fraction": 1.0,
        }
        winner, rows, reason = select_trajectory(
            [tight, long_clear], DEFAULT_WEIGHTS)
        self.assertEqual(reason, "")
        self.assertEqual(winner["method"], "offset")
        self.assertIsNotNone(winner["C_dim"])
        self.assertIsNotNone(winner["C_safe"])
        self.assertIsNotNone(winner["C_eff"])
        self.assertTrue(any(row.get("selected") for row in rows))

    def test_collision_boxes_merged(self):
        surface = _surface()
        _occupy_column(surface, 10, height=0.40)
        snap = OccupancySnapshot.from_surface_2d(surface, inflate_m=0.0)
        boxes = occupancy_collision_boxes(snap, max_objects=200)
        self.assertGreaterEqual(len(boxes), 1)
        self.assertTrue(all(b["id"].startswith("cargo_occ_") for b in boxes))

    def test_variants_at_least_two(self):
        surface = _surface()
        snap = OccupancySnapshot.from_surface_2d(surface, inflate_m=0.0)
        variants = generate_path_variants(
            (-0.3, 0.0, 0.5), (0.3, 0.0, 0.5), snapshot=snap, max_candidates=4)
        self.assertGreaterEqual(len(variants), 2)
        methods = [v.method for v in variants]
        self.assertIn("direct", methods)

    def test_clear_top_follows_neighbor_top_not_fixed_delta(self):
        surface = _surface(inner=(1.0, 0.5, 2.0), center=(0.0, 0.0, 1.0))
        _occupy_column(surface, 10, height=0.25)
        snap = OccupancySnapshot.from_surface_2d(surface, inflate_m=0.0)
        payload = (0.20, 0.20, 0.28)
        contact = 0.40
        variants = generate_path_variants(
            (-0.35, 0.0, 0.45), (0.35, 0.0, contact), snapshot=snap,
            max_candidates=4, payload_height=payload[2], payload_wdh=payload,
            arm_radius=0.0)
        by_name = {v.method: v for v in variants}
        self.assertIn("clear_top", by_name)
        dest_z = by_name["clear_top"].waypoints[-1][2]
        need = required_carry_suction_z(0.25, payload[2])
        self.assertAlmostEqual(dest_z, need, places=2)
        self.assertGreater(dest_z, contact + 0.10 + 1e-6)
        bottom = dest_z - payload[2]
        self.assertGreaterEqual(bottom - 0.25, DEFAULT_WEIGHTS["carry_margin_m"] - 1e-6)

    def test_ceiling_clamps_carry_below_arm_stack(self):
        surface = _surface()
        _occupy_column(surface, 10, height=0.70)
        snap = OccupancySnapshot.from_surface_2d(surface, inflate_m=0.0)
        payload = (0.08, 0.08, 0.25)
        variants = generate_path_variants(
            (-0.35, 0.0, 0.50), (0.35, 0.0, 0.50), snapshot=snap,
            max_candidates=4, payload_height=payload[2], payload_wdh=payload,
            arm_radius=0.0)
        cap = suction_ceiling_z(snap)
        self.assertLess(cap, required_carry_suction_z(0.70, payload[2]))
        for variant in variants:
            for wp in variant.waypoints:
                self.assertLessEqual(wp[2], cap + 1e-6, variant.method)

    def test_sweep_pad_rejects_tangent_gap(self):
        surface = _surface()
        _occupy_column(surface, 10, height=0.40)
        snap = OccupancySnapshot.from_surface_2d(surface, inflate_m=0.0)
        payload = (0.08, 0.08, 0.15)
        # 2.8 mm above the column, same class as trial_01 vs placed_0_0_0.
        tangent = [(0.0, 0.0, 0.40 + 0.15 + 0.0028),
                   (0.05, 0.0, 0.40 + 0.15 + 0.0028)]
        tight = sweep_polyline(
            snap, tangent, payload, arm_radius=0.0, collision_pad=0.02)
        self.assertTrue(tight.collides, tight)
        cleared = [(0.0, 0.0, 0.40 + 0.15 + 0.05),
                   (0.05, 0.0, 0.40 + 0.15 + 0.05)]
        ok = sweep_polyline(
            snap, cleared, payload, arm_radius=0.0, collision_pad=0.02)
        self.assertFalse(ok.collides, ok)

    def test_missing_descriptor_fails_closed(self):
        surface = _surface()
        del surface["geometry_descriptor"]
        with self.assertRaises(OccupancyMismatch) as ctx:
            OccupancySnapshot.from_surface_2d(surface, inflate_m=0.0)
        self.assertIn(PLACE_PATH_INFEASIBLE, str(ctx.exception))

    def test_chamfer_payload_is_outside_hull(self):
        hull = normalize_descriptor({
            "frame_id": "container_link",
            "length": 1.0,
            "width": 1.0,
            "floor_z": 0.0,
            "ceiling_z": 1.0,
            "chamfer": {
                "side": "positive_y",
                "floor_y": 0.10,
                "wall_y": 0.50,
                "wall_z": 0.40,
            },
        })
        surface = _surface(nx=20, ny=20, inner=(1.0, 1.0, 1.0), hull=hull)
        snap = OccupancySnapshot.from_surface_2d(surface, inflate_m=0.0)
        payload = (0.10, 0.10, 0.10)
        path = [(0.0, 0.35, 0.15), (0.10, 0.35, 0.15)]
        result = sweep_polyline(snap, path, payload, arm_radius=0.0)
        self.assertTrue(result.collides, result)
        self.assertEqual(result.reason, "outside_hull")
        self.assertTrue(snap.cell_inside_hull(snap.nx // 2, 0))
        self.assertFalse(snap.cell_inside_hull(snap.nx // 2, snap.ny - 1))


class TestResolvePayloadWdh(unittest.TestCase):

    def test_measured_wins(self):
        wdh, source = resolve_payload_wdh(
            {"width": 0.72, "depth": 0.50, "height": 0.31},
            [0.55, 0.40, 0.25])
        self.assertEqual(wdh, [0.72, 0.50, 0.31])
        self.assertEqual(source, PAYLOAD_SOURCE_MEASURED)

    def test_fallback_when_unmeasured(self):
        for measured in (None, {}, {"width": 0.7},
                         {"width": 0.0, "depth": 0.4, "height": 0.2}):
            wdh, source = resolve_payload_wdh(measured, [0.55, 0.40, 0.25])
            self.assertEqual(wdh, [0.55, 0.40, 0.25])
            self.assertEqual(source, PAYLOAD_SOURCE_DEFAULT)

    def test_defaults_are_coerced_to_float(self):
        wdh, _source = resolve_payload_wdh(
            None, [1, 2, 3])
        self.assertEqual(wdh, [1.0, 2.0, 3.0])
        self.assertTrue(all(isinstance(v, float) for v in wdh))


class TestLandingExemption(unittest.TestCase):
    """Landing-footprint exemption: all-segment occupancy without
    false-blocking the slot the segment is inserting into."""

    PAYLOAD = (0.30, 0.20, 0.25)

    def _snapshot(self, surface):
        return OccupancySnapshot.from_surface_2d(surface, inflate_m=0.05)

    def _anchor(self, snapshot, xyz, margin_m=0.0):
        return exempt_footprint_locals(
            snapshot, [tuple(xyz)], self.PAYLOAD,
            arm_radius=0.08, inflate_m=0.05, margin_m=margin_m)

    def test_descend_onto_support_passes_with_exemption(self):
        # Slot column occupied (support top 0.40); payload descends onto it.
        surface = _surface()
        _occupy_column(surface, 10, height=0.40)
        snapshot = self._snapshot(surface)
        target = snapshot.cell_center_map(10, 5, 0.45 - snapshot.center_base[2])
        points = [(target[0], target[1], 0.80), tuple(target)]
        blocked = sweep_polyline(snapshot, points, self.PAYLOAD)
        self.assertTrue(blocked.collides)
        exempt = self._anchor(snapshot, target)
        self.assertEqual(len(exempt), 1)
        freed = sweep_polyline(snapshot, points, self.PAYLOAD,
                               exempt=exempt)
        self.assertFalse(freed.collides)

    def test_lateral_scrape_still_collides_with_exemption(self):
        # Exemption covers the landing corridor (payload + arm_radius +
        # inflate around the anchor); a low path over an occupied column
        # OUTSIDE that corridor must still collide. The corridor is wide
        # by design: those cells are swept only through the fat
        # payload+arm proxy, which the landing deliberately overrides.
        surface = _surface()
        _occupy_column(surface, 10, height=0.40)
        _occupy_column(surface, 2, height=0.40)
        snapshot = self._snapshot(surface)
        slot = snapshot.cell_center_map(10, 5, 0.45 - snapshot.center_base[2])
        exempt = self._anchor(snapshot, slot)
        other = snapshot.cell_center_map(2, 5, 0.60 - snapshot.center_base[2])
        points = [(other[0], other[1], 0.60)]
        hit = sweep_polyline(snapshot, points, self.PAYLOAD, exempt=exempt)
        self.assertTrue(hit.collides)

    def test_exempt_cells_drop_boxes_far_cells_kept(self):
        surface = _surface()
        _occupy_column(surface, 10, height=0.40)
        _occupy_column(surface, 2, height=0.40)
        snapshot = self._snapshot(surface)
        self.assertTrue(occupancy_collision_boxes(snapshot))
        anchor = snapshot.cell_center_map(10, 5, 0.45 - snapshot.center_base[2])
        exempt = self._anchor(snapshot, anchor)
        boxes = occupancy_collision_boxes(snapshot, exempt=exempt)
        self.assertTrue(boxes)
        half_w = 0.5 * self.PAYLOAD[0]
        for box in boxes:
            self.assertFalse(box["id"].startswith("cargo_occ_10_"),
                             "slot column leaked a box: %s" % box["id"])
            # A remaining box may poke inflate into the corridor edge, but
            # never reach the payload footprint itself.
            bx, _by, _bz = box["xyz"]
            sx = 0.5 * box["size"][0]
            gap = abs(bx - anchor[0]) - sx - half_w
            self.assertGreaterEqual(gap, -1e-6,
                                    "box %s reaches the payload footprint"
                                    % box["id"])
        # Without the far column the exempt slot yields no box at all.
        surface_far = _surface()
        _occupy_column(surface_far, 10, height=0.40)
        snapshot_far = self._snapshot(surface_far)
        self.assertEqual(
            occupancy_collision_boxes(snapshot_far, exempt=exempt), [])

    def test_unknown_full_height_column_exempted(self):
        # Unknown-above-floor becomes a full-height obstacle; the landing
        # exemption must clear the slot's own column for insertion.
        surface = _surface(state="unknown", height=0.30)
        snapshot = self._snapshot(surface)
        self.assertTrue(snapshot.obstacle[10][5])
        anchor = snapshot.cell_center_map(10, 5, 0.45 - snapshot.center_base[2])
        points = [(anchor[0], anchor[1], 0.80), tuple(anchor)]
        self.assertTrue(sweep_polyline(
            snapshot, points, self.PAYLOAD).collides)
        exempt = self._anchor(snapshot, anchor)
        self.assertFalse(sweep_polyline(
            snapshot, points, self.PAYLOAD, exempt=exempt).collides)

    def test_outside_hull_waypoints_anchor_nothing(self):
        surface = _surface()
        snapshot = self._snapshot(surface)
        # Staging/portal points (outside the closed hull) and points high
        # above the ceiling anchor no exemption: carry keeps today's scope.
        self.assertEqual(self._anchor(snapshot, [1.50, 0.0, 0.60]), [])
        self.assertEqual(self._anchor(snapshot, [0.0, 0.0, 5.00]), [])
        self.assertEqual(
            exempt_footprint_locals(
                snapshot, [(1.5, 0.0, 0.6), (0.0, 0.0, 5.0)], self.PAYLOAD),
            [])
        self.assertEqual(
            exempt_footprint_locals(snapshot, [(0.0, 0.0, 0.6)], (0, 0, 0)),
            [])

    def test_margin_widens_the_exemption(self):
        surface = _surface()
        _occupy_column(surface, 10, height=0.40)
        snapshot = self._snapshot(surface)
        anchor = snapshot.cell_center_map(10, 5, 0.45 - snapshot.center_base[2])
        tight = self._anchor(snapshot, anchor, margin_m=0.0)[0]
        wide = self._anchor(snapshot, anchor, margin_m=0.20)[0]
        self.assertGreater(wide[2] - wide[0], tight[2] - tight[0])
        self.assertGreater(wide[3] - wide[1], tight[3] - tight[1])

    def test_duplicate_waypoints_dedup(self):
        surface = _surface()
        snapshot = self._snapshot(surface)
        anchor = snapshot.cell_center_map(10, 5, 0.45 - snapshot.center_base[2])
        exempt = exempt_footprint_locals(
            snapshot, [tuple(anchor), tuple(anchor), tuple(anchor)],
            self.PAYLOAD, arm_radius=0.08, inflate_m=0.05)
        self.assertEqual(len(exempt), 1)


if __name__ == "__main__":
    unittest.main()
