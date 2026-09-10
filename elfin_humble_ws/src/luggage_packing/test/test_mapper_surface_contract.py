#!/usr/bin/env python3
"""Planner floor-prior dict × generate_candidates: slab slot z = floor + h/2.

Must match cargo_volume_mapper.surface_map_2d: floor-relative height=0 and
center_base = usable-volume center in container_link.
"""

import unittest

from luggage_perception.cargo_surface_schema import (
    SurfaceMapValidationError,
    validate_surface_map,
)
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_packing.placement_solver import generate_candidates


class TestFloorPriorMapperContract(unittest.TestCase):
    FLOOR_Z = 0.53
    INNER = (1.49, 1.97, 1.48)
    BOX = [0.55, 0.40, 0.25]

    @staticmethod
    def _geometry():
        return {
            "schema_version": 1,
            "frame_id": "container_link",
            "length": 1.49,
            "width": 1.97,
            "floor_z": 0.53,
            "ceiling_z": 2.01,
            "chamfer": {
                "side": "positive_y",
                "floor_y": 0.55,
                "wall_y": 0.985,
                "wall_z": 0.90,
            },
        }

    def _prior(self):
        inner_l, inner_w, inner_h = self.INNER
        res = 0.05
        nx = max(1, int(round(inner_l / res)))
        ny = max(1, int(round(inner_w / res)))
        return {
            "resolution": res,
            "nx": nx, "ny": ny,
            "inner_size": [inner_l, inner_w, inner_h],
            "floor_z": 0.0,
            "center_base": [0.0, 0.0, self.FLOOR_Z + 0.5 * inner_h],
            "yaw": 0.0,
            "height": [[0.0] * ny for _ in range(nx)],
            "state": [["unknown"] * ny for _ in range(nx)],
        }

    def test_empty_prior_slot_z_is_slab_plus_half_height(self):
        feasible = [
            cand for cand in generate_candidates(
                self._prior(), self.BOX, allowed_yaws=[0.0])
            if cand["feasible"]
        ]
        self.assertGreater(len(feasible), 0)
        expected_z = self.FLOOR_Z + 0.5 * self.BOX[2]
        for cand in feasible:
            self.assertEqual(cand["support_source"], "floor_prior")
            self.assertAlmostEqual(
                cand["center_base"][2], expected_z, delta=0.05)

    def test_weighted_mapper_surface_validates_without_placement_wiring(self):
        mapper = CargoVolumeMapper(
            inner_size=self.INNER,
            center_base=[0.0, 0.0, self.FLOOR_Z + 0.5 * self.INNER[2]],
            yaw=0.0,
            resolution=0.19,
            geometry_descriptor=self._geometry(),
        )
        surface = mapper.surface_map_2d()
        normalized = validate_surface_map(
            surface, expected_geometry_hash=mapper.geometry_hash)
        self.assertEqual(normalized["geometry_hash"], mapper.geometry_hash)

    def test_surface_geometry_hash_mismatch_fails_closed(self):
        mapper = CargoVolumeMapper(
            inner_size=self.INNER,
            center_base=[0.0, 0.0, self.FLOOR_Z + 0.5 * self.INNER[2]],
            yaw=0.0,
            resolution=0.23,
            geometry_descriptor=self._geometry(),
        )
        with self.assertRaises(SurfaceMapValidationError) as raised:
            validate_surface_map(
                mapper.surface_map_2d(), expected_geometry_hash="stale")
        self.assertEqual(raised.exception.reason, "GEOMETRY_HASH_MISMATCH")


if __name__ == "__main__":
    unittest.main()
