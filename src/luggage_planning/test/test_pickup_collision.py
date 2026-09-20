#!/usr/bin/env python3
"""Pickup collision AABB must follow the E0 measured-height contract."""

import unittest
from types import SimpleNamespace

from luggage_planning.pick_authorization import (
    DETECT_FULL_GEOMETRY_REQUIRED,
    HEIGHT_SOURCE_CONFIGURED_SUPPORT,
    HEIGHT_SOURCE_MEASURED_SUPPORT,
    HEIGHT_SOURCE_UNAVAILABLE,
)
from luggage_planning.pickup_collision import pickup_collision_aabb


def _box(**fields):
    pose = SimpleNamespace(
        position=SimpleNamespace(x=-1.0, y=0.0, z=fields.get("pose_z", 1.107)),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    top = SimpleNamespace(
        position=SimpleNamespace(x=-1.0, y=0.0, z=fields.get("top_z", 1.107)),
        orientation=pose.orientation,
    )
    return SimpleNamespace(
        pose=pose,
        top_surface_pose=top,
        width=0.50,
        depth=0.35,
        height=fields.get("height", 0.25),
        height_valid=fields.get("height_valid", False),
        height_source=fields.get(
            "height_source", HEIGHT_SOURCE_UNAVAILABLE),
        top_surface_valid=fields.get("top_surface_valid", True),
    )


class TestPickupCollisionAabb(unittest.TestCase):

    def test_measured_height_uses_pose_as_centre(self):
        xyz, _quat, size = pickup_collision_aabb(_box(
            height_valid=True, height_source=HEIGHT_SOURCE_MEASURED_SUPPORT,
            pose_z=0.98, height=0.25, top_z=1.107))
        self.assertAlmostEqual(xyz[2], 0.98)
        self.assertAlmostEqual(size[2], 0.25)

    def test_invalid_height_raises(self):
        with self.assertRaises(ValueError) as ctx:
            pickup_collision_aabb(_box(
                height_valid=False, pose_z=1.107, height=0.25, top_z=1.107))
        self.assertIn(DETECT_FULL_GEOMETRY_REQUIRED, str(ctx.exception))

    def test_configured_height_raises(self):
        with self.assertRaises(ValueError) as ctx:
            pickup_collision_aabb(_box(
                height_valid=True,
                height_source=HEIGHT_SOURCE_CONFIGURED_SUPPORT,
                pose_z=0.98, height=0.25, top_z=1.107))
        self.assertIn(DETECT_FULL_GEOMETRY_REQUIRED, str(ctx.exception))


if __name__ == "__main__":
    unittest.main()
