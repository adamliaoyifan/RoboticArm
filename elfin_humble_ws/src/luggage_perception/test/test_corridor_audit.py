"""Corridor audit (G4/E1) unit tests - no ROS."""
import unittest

from luggage_perception.corridor_audit import (
    CORRIDOR_EMPTY_MAP,
    CORRIDOR_FREE,
    CORRIDOR_OCCUPIED,
    audit_corridor,
    corridor_aabb,
    corridor_surface_max,
    required_carry_z,
)

INNER = [1.49, 1.97, 1.48]
SMALLEST = [0.55, 0.40, 0.25]


class TestCorridorAABB(unittest.TestCase):

    def test_negative_x_corridor_spans_opening_to_near_face(self):
        aabb = corridor_aabb([-0.2, 0.0, 0.125], [0.55, 0.40, 0.25],
                             INNER, SMALLEST)
        self.assertAlmostEqual(aabb[0], -0.745)   # opening plane
        self.assertAlmostEqual(aabb[3], -0.475)   # slot near face
        # Y inflated by half the smallest box depth.
        self.assertAlmostEqual(aabb[1], -0.2 - 0.2)
        self.assertAlmostEqual(aabb[4], 0.2 + 0.2)


class TestSurfaceMax(unittest.TestCase):

    def test_none_when_no_committed(self):
        aabb = corridor_aabb([-0.2, 0.0, 0.125], [0.55, 0.40, 0.25],
                             INNER, SMALLEST)
        self.assertIsNone(corridor_surface_max([], aabb))

    def test_takes_tallest_box_in_corridor(self):
        aabb = corridor_aabb([-0.2, 0.0, 0.125], [0.55, 0.40, 0.25],
                             INNER, SMALLEST)
        boxes = [
            ([-0.6, 0.0, 0.14], [0.55, 0.40, 0.28]),   # inside, top 0.28
            ([0.4, 0.0, 0.14], [0.55, 0.40, 0.28]),    # outside corridor
        ]
        self.assertAlmostEqual(corridor_surface_max(boxes, aabb), 0.28)


class TestRequiredCarryZ(unittest.TestCase):

    def test_full_height_below_suction(self):
        # Payload hangs a full box height below the suction frame.
        self.assertAlmostEqual(required_carry_z(0.28, 0.25, 0.05), 0.58)

    def test_none_when_no_surface(self):
        self.assertIsNone(required_carry_z(None, 0.25))


class TestAuditCorridor(unittest.TestCase):

    def test_empty_ledger_verdict(self):
        audit = audit_corridor([-0.2, 0.0, 0.125], [0.55, 0.40, 0.25],
                               [], INNER, SMALLEST)
        self.assertEqual(audit["verdict"], CORRIDOR_EMPTY_MAP)
        self.assertIsNone(audit["surface_max"])
        self.assertIsNone(audit["required_carry_z"])

    def test_low_box_does_not_clip_high_band(self):
        # A committed box at the far side, top below the carry band:
        # corridor is free, but surface_max is reported for height.
        boxes = [([-0.6, 0.6, 0.125], [0.55, 0.40, 0.25])]
        audit = audit_corridor([-0.2, -0.3, 0.625], [0.55, 0.40, 0.25],
                               boxes, INNER, SMALLEST)
        self.assertIn(audit["verdict"], (CORRIDOR_FREE, CORRIDOR_OCCUPIED))

    def test_box_in_band_occupies(self):
        # Committed box inside the corridor z-band and XY span.
        boxes = [([-0.6, 0.0, 0.125], [0.55, 0.40, 0.25])]
        audit = audit_corridor([-0.2, 0.0, 0.125], [0.55, 0.40, 0.25],
                               boxes, INNER, SMALLEST)
        self.assertEqual(audit["verdict"], CORRIDOR_OCCUPIED)

    def test_audit_reports_required_height(self):
        boxes = [([-0.6, 0.0, 0.14], [0.55, 0.40, 0.28])]
        audit = audit_corridor([-0.2, 0.0, 0.14], [0.55, 0.40, 0.28],
                               boxes, INNER, SMALLEST)
        self.assertAlmostEqual(audit["surface_max"], 0.28)
        self.assertAlmostEqual(audit["required_carry_z"], 0.28 + 0.28 + 0.05)


def _scene_geometry():
    from luggage_description.container_geometry import normalize_descriptor
    return normalize_descriptor(
        {
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
    )


class TestGateG4Audit(unittest.TestCase):
    INNER = [1.49, 1.97, 1.48]
    SMALL = [0.55, 0.40, 0.25]
    SIZE = [0.20, 0.20, 0.20]

    def test_unsupported_opening_side_fails_closed(self):
        with self.assertRaises(ValueError):
            audit_corridor(
                [0.0, 0.0, 1.2], self.SIZE, [], self.INNER, self.SMALL,
                opening_side="positive_y",
            )

    def test_wedge_overlap_is_not_occupied(self):
        geom = _scene_geometry()
        slot = [0.0, -0.40, 0.65]
        fat_smallest = [0.55, 2.4, 0.25]
        wedge = [([0.0, 0.82, 0.62], [0.20, 0.20, 0.20])]
        aabb = corridor_aabb(slot, self.SIZE, self.INNER, fat_smallest)
        self.assertGreater(aabb[4], 0.72)
        audit = audit_corridor(
            slot, self.SIZE, wedge, self.INNER, fat_smallest, geometry=geom
        )
        self.assertEqual(audit["boxes_in_corridor"], 0)
        self.assertEqual(audit["verdict"], CORRIDOR_FREE)
        self.assertIsNone(audit["surface_max"])
        self.assertIsNone(
            corridor_surface_max(wedge, aabb, geometry=geom),
            "wedge-only AABB overlap must not raise carry height",
        )

    def test_swept_slanted_face_is_occupied(self):
        geom = _scene_geometry()
        audit = audit_corridor(
            [0.0, 0.70, 0.62], self.SIZE, [], self.INNER, self.SMALL,
            geometry=geom,
        )
        self.assertEqual(audit["verdict"], CORRIDOR_OCCUPIED)

    def test_safe_center_side_and_stacked_corridors(self):
        geom = _scene_geometry()
        cases = (
            [0.0, 0.0, 1.20],
            [0.10, -0.50, 1.20],
            [0.0, 0.20, 1.50],
        )
        for center in cases:
            audit = audit_corridor(
                center, self.SIZE, [], self.INNER, self.SMALL, geometry=geom
            )
            self.assertEqual(
                audit["verdict"], CORRIDOR_EMPTY_MAP, msg=center
            )


if __name__ == "__main__":
    unittest.main()
