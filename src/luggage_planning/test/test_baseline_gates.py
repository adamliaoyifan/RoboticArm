"""B1 authorization and B3 occupancy collision for the baseline gate."""

import unittest

from luggage_description.container_geometry import cuboid_from_inner_size
from luggage_planning.occupancy_place_paths import (
    OccupancySnapshot,
    occupancy_collision_boxes,
)
from luggage_planning.pick_authorization import (
    AUTHORIZE,
    HEIGHT_SOURCE_MEASURED_SUPPORT,
    PickAuthorizationPolicy,
    pick_authorized,
)


class _Box(object):
    def __init__(self, **kwargs):
        self.top_surface_valid = True
        self.height_valid = True
        self.height_source = HEIGHT_SOURCE_MEASURED_SUPPORT
        self.pca_source = "measure"
        for key, value in kwargs.items():
            setattr(self, key, value)


def _surface(state="free", height=0.0):
    inner = (1.0, 0.5, 1.0)
    hull = cuboid_from_inner_size(inner, floor_z=0.0)
    desc = hull.descriptor()
    nx, ny = 8, 4
    return {
        "geometry_hash": desc["geometry_hash"],
        "geometry_descriptor": desc,
        "map_revision": 1,
        "resolution": 0.1,
        "nx": nx,
        "ny": ny,
        "inner_size": list(inner),
        "floor_z": 0.0,
        "center_base": [0.0, 0.0, 0.5],
        "yaw": 0.0,
        "height": [[height] * ny for _ in range(nx)],
        "state": [[state] * ny for _ in range(nx)],
        "confidence": [["sensor"] * ny for _ in range(nx)],
    }


class TestAuthorization(unittest.TestCase):
    def test_measured_input_authorizes_twice(self):
        policy = PickAuthorizationPolicy()
        box = _Box()
        first = policy.evaluate(True, box, 0.0, 1)
        second = policy.evaluate(True, box, 0.1, 1)
        self.assertEqual(first.action, AUTHORIZE)
        self.assertEqual(second.action, first.action)
        self.assertEqual(second.reason, first.reason)
        self.assertTrue(pick_authorized(box))

    def test_missing_detection_never_authorizes(self):
        decision = PickAuthorizationPolicy().evaluate(False, None, 0.0, 1)
        self.assertNotEqual(decision.action, AUTHORIZE)
        self.assertFalse(pick_authorized(None))

    def test_hold_only_never_authorizes(self):
        box = _Box(pca_source="hold_track")
        decision = PickAuthorizationPolicy().evaluate(True, box, 0.0, 1)
        self.assertNotEqual(decision.action, AUTHORIZE)
        self.assertFalse(pick_authorized(box))

    def test_invalid_height_never_authorizes(self):
        box = _Box(height_valid=False, height_source=0)
        decision = PickAuthorizationPolicy().evaluate(True, box, 0.0, 1)
        self.assertNotEqual(decision.action, AUTHORIZE)
        self.assertFalse(pick_authorized(box))


class TestOccupancyObstacle(unittest.TestCase):
    def test_insert_then_remove_changes_collision_boxes(self):
        empty = OccupancySnapshot.from_surface_2d(_surface(), inflate_m=0.0)
        self.assertEqual(occupancy_collision_boxes(empty), [])
        occupied = _surface()
        occupied["state"][2][1] = "occupied"
        occupied["height"][2][1] = 0.4
        blocked = OccupancySnapshot.from_surface_2d(occupied, inflate_m=0.0)
        boxes = occupancy_collision_boxes(blocked)
        self.assertGreater(len(boxes), 0)
        cleared = OccupancySnapshot.from_surface_2d(_surface(), inflate_m=0.0)
        self.assertEqual(occupancy_collision_boxes(cleared), [])
