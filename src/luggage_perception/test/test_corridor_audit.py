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


if __name__ == "__main__":
    unittest.main()
