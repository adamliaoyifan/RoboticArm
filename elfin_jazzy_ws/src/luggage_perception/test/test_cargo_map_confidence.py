#!/usr/bin/env python3
"""Unit tests for cargo map confidence/source tracking (no roscore required)."""

import os
import sys
import unittest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

from luggage_perception.cargo_volume_mapper import (
    CargoVolumeMapper,
    SOURCE_GEOMETRY,
    SOURCE_NONE,
    SOURCE_SENSOR,
)


class TestSourceTracking(unittest.TestCase):
    def _make_mapper(self):
        return CargoVolumeMapper(
            inner_size=[2.0, 2.0, 2.0],
            center_base=[0.0, 0.0, 1.0],
            yaw=0.0,
            resolution=0.5,
        )

    def test_integrate_points_marks_sensor(self):
        m = self._make_mapper()
        m.integrate_points([(0.0, 0.0, 1.0)], origin=(0.0, 0.0, 3.0))
        sm = m.surface_map_2d()
        found_sensor = False
        for ix in range(sm["nx"]):
            for iy in range(sm["ny"]):
                if sm["state"][ix][iy] == "occupied":
                    self.assertEqual(sm["confidence"][ix][iy], "sensor")
                    found_sensor = True
        self.assertTrue(found_sensor, "Expected at least one sensor-occupied cell")

    def test_mark_placed_box_marks_geometry(self):
        m = self._make_mapper()
        m.mark_placed_box([0.0, 0.0, 1.0], [0.5, 0.5, 0.5])
        sm = m.surface_map_2d()
        found_geometry = False
        for ix in range(sm["nx"]):
            for iy in range(sm["ny"]):
                if sm["state"][ix][iy] == "occupied":
                    self.assertEqual(sm["confidence"][ix][iy], "geometry")
                    found_geometry = True
        self.assertTrue(found_geometry, "Expected at least one geometry-occupied cell")

    def test_reset_preserve_placed_replays_geometry(self):
        m = self._make_mapper()
        m.mark_placed_box([0.0, 0.0, 1.0], [0.5, 0.5, 0.5])
        before = m.stats()["map_revision"]
        m.reset(preserve_placed=True)
        self.assertGreater(m.stats()["map_revision"], before)
        self.assertEqual(m.stats()["committed_box_count"], 1)
        sm = m.surface_map_2d()
        self.assertTrue(any(
            sm["confidence"][ix][iy] == "geometry"
            for ix in range(sm["nx"])
            for iy in range(sm["ny"])
        ))

    def test_unmark_placed_box_rebuilds_remaining_geometry(self):
        m = self._make_mapper()
        first = ([0.0, 0.0, 0.5], [0.5, 0.5, 0.5])
        second = ([0.6, 0.0, 0.5], [0.5, 0.5, 0.5])
        m.mark_placed_box(*first)
        m.mark_placed_box(*second)
        self.assertTrue(m.unmark_placed_box(*first))
        self.assertEqual(m.stats()["committed_box_count"], 1)
        self.assertFalse(m.unmark_placed_box(*first))

    def test_sensor_overrides_geometry_source(self):
        m = self._make_mapper()
        m.mark_placed_box([0.0, 0.0, 1.0], [0.5, 0.5, 0.5])
        m.integrate_points([(0.0, 0.0, 1.5)], origin=(0.0, 0.0, 3.0))
        sm = m.surface_map_2d()
        for ix in range(sm["nx"]):
            for iy in range(sm["ny"]):
                if sm["state"][ix][iy] == "occupied":
                    self.assertIn(sm["confidence"][ix][iy], ("sensor", "geometry"))

    def test_unknown_cells_have_none_confidence(self):
        m = self._make_mapper()
        sm = m.surface_map_2d()
        for ix in range(sm["nx"]):
            for iy in range(sm["ny"]):
                self.assertEqual(sm["state"][ix][iy], "unknown")
                self.assertEqual(sm["confidence"][ix][iy], "none")

    def test_free_cells_have_sensor_confidence(self):
        m = CargoVolumeMapper(
            inner_size=[1.0, 1.0, 1.0],
            center_base=[0.0, 0.0, 0.5],
            yaw=0.0,
            resolution=0.25,
        )
        origin = (0.0, 0.0, 3.0)
        hit_points = [(0.0, 0.0, -0.1)]
        for _ in range(5):
            m.integrate_points(hit_points, origin=origin)
        m.mark_free_world(0.0, 0.0, 0.25)
        m.mark_free_world(0.0, 0.0, 0.50)
        m.mark_free_world(0.0, 0.0, 0.75)
        m.mark_free_world(0.0, 0.0, 1.0)
        sm = m.surface_map_2d()
        found_free = False
        for ix in range(sm["nx"]):
            for iy in range(sm["ny"]):
                if sm["state"][ix][iy] == "free":
                    self.assertEqual(sm["confidence"][ix][iy], "sensor")
                    found_free = True
        self.assertTrue(found_free, "Expected some free cells from explicit marking")

    def test_reset_clears_source(self):
        m = self._make_mapper()
        m.integrate_points([(0.0, 0.0, 1.0)], origin=(0.0, 0.0, 3.0))
        m.reset()
        sm = m.surface_map_2d()
        for ix in range(sm["nx"]):
            for iy in range(sm["ny"]):
                self.assertEqual(sm["confidence"][ix][iy], "none")

    def test_map_revision_advances_on_integration_and_reset(self):
        m = self._make_mapper()
        initial = m.stats()["map_revision"]
        m.integrate_points(
            [(0.0, 0.0, 1.0)], origin=(0.0, 0.0, 3.0))
        integrated = m.stats()["map_revision"]
        self.assertGreater(integrated, initial)
        m.reset()
        self.assertGreater(m.stats()["map_revision"], integrated)

    def test_unknown_corridor_is_not_treated_as_safe_free_space(self):
        m = self._make_mapper()
        confidence = m.corridor_free_confidence(
            [-0.5, 0.0, 1.0], [0.5, 0.0, 1.0], radius=0.0)
        self.assertEqual(confidence, 0.0)

    def test_occupied_corridor_is_rejected(self):
        m = self._make_mapper()
        for _ in range(5):
            m.integrate_points(
                [(0.0, 0.0, 1.0)], origin=(-0.5, 0.0, 1.0))
        confidence = m.corridor_free_confidence(
            [-0.5, 0.0, 1.0], [0.5, 0.0, 1.0], radius=0.0)
        self.assertEqual(confidence, 0.0)


class TestPlacementSolverConfidence(unittest.TestCase):
    def test_confidence_in_surface_map_accepted_by_solver(self):
        from luggage_packing.placement_solver import generate_candidates, best_candidate

        sm = {
            "resolution": 0.1,
            "nx": 20,
            "ny": 16,
            "inner_size": [2.0, 1.6, 1.5],
            "center_base": [2.0, 0.0, 0.75],
            "yaw": 0.0,
            "height": [[0.0] * 16 for _ in range(20)],
            "state": [["free"] * 16 for _ in range(20)],
            "clearance": [[1.5] * 16 for _ in range(20)],
            "known_ratio": [[1.0] * 16 for _ in range(20)],
            "confidence": [["sensor"] * 16 for _ in range(20)],
        }
        cands = generate_candidates(sm, [0.7, 0.45, 0.28], allowed_yaws=[0.0])
        top = best_candidate(cands)
        self.assertIsNotNone(top)
        self.assertTrue(top["feasible"])
        self.assertGreater(top["confidence_ratio"], 0.9)

    def test_geometry_only_gets_lower_score(self):
        from luggage_packing.placement_solver import generate_candidates, best_candidate

        sm_sensor = {
            "resolution": 0.1,
            "nx": 20,
            "ny": 16,
            "inner_size": [2.0, 1.6, 1.5],
            "center_base": [2.0, 0.0, 0.75],
            "yaw": 0.0,
            "height": [[0.0] * 16 for _ in range(20)],
            "state": [["free"] * 16 for _ in range(20)],
            "clearance": [[1.5] * 16 for _ in range(20)],
            "known_ratio": [[1.0] * 16 for _ in range(20)],
            "confidence": [["sensor"] * 16 for _ in range(20)],
        }
        sm_geometry = dict(sm_sensor)
        sm_geometry["confidence"] = [["geometry"] * 16 for _ in range(20)]

        cands_sensor = generate_candidates(sm_sensor, [0.7, 0.45, 0.28], allowed_yaws=[0.0])
        cands_geom = generate_candidates(sm_geometry, [0.7, 0.45, 0.28], allowed_yaws=[0.0])
        top_sensor = best_candidate(cands_sensor)
        top_geom = best_candidate(cands_geom)
        self.assertIsNotNone(top_sensor)
        self.assertIsNotNone(top_geom)
        self.assertGreater(top_sensor["score"], top_geom["score"])

    def test_no_confidence_field_backward_compatible(self):
        from luggage_packing.placement_solver import generate_candidates, best_candidate

        sm = {
            "resolution": 0.1,
            "nx": 20,
            "ny": 16,
            "inner_size": [2.0, 1.6, 1.5],
            "center_base": [2.0, 0.0, 0.75],
            "yaw": 0.0,
            "height": [[0.0] * 16 for _ in range(20)],
            "state": [["free"] * 16 for _ in range(20)],
            "clearance": [[1.5] * 16 for _ in range(20)],
            "known_ratio": [[1.0] * 16 for _ in range(20)],
        }
        cands = generate_candidates(sm, [0.7, 0.45, 0.28], allowed_yaws=[0.0])
        top = best_candidate(cands)
        self.assertIsNotNone(top)
        self.assertTrue(top["feasible"])


if __name__ == "__main__":
    unittest.main()
