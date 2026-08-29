#!/usr/bin/env python3
import math
import os
import sys
import unittest


from luggage_perception.known_scene_point_filter import (  # noqa: E402
    KnownScenePointFilter,
    OrientedBox,
)


class TestKnownScenePointFilter(unittest.TestCase):
    def test_container_shell_removed_but_inner_kept(self):
        filt = KnownScenePointFilter(
            container_outer=OrientedBox(
                "outer", [0.0, 0.0, 1.0], [2.0, 2.0, 2.0]
            ),
            container_inner=OrientedBox(
                "inner", [0.0, 0.0, 1.0], [1.8, 1.8, 1.8]
            ),
            padding=0.0,
            filter_ground=False,
        )
        points = [(0.0, 0.0, 1.0), (0.96, 0.0, 1.0), (2.0, 0.0, 1.0)]
        kept, indices = filt.filter_points(points)
        self.assertEqual(kept, [points[0], points[2]])
        self.assertEqual(indices, [0, 2])
        self.assertEqual(
            filt.last_stats["dropped_by_class"]["container_shell"], 1
        )

    def test_solid_box_and_ground_removed(self):
        filt = KnownScenePointFilter(
            solid_boxes=[
                OrientedBox("pedestal", [0.0, 0.0, -0.5], [1.0, 1.0, 1.0])
            ],
            base_in_world=([0.0, 0.0, 1.0], [0.0, 0.0, 0.0]),
            padding=0.02,
            filter_ground=True,
        )
        self.assertEqual(filt.classify((0.0, 0.0, -0.5)), "pedestal")
        self.assertEqual(filt.classify((2.0, 0.0, -1.0)), "ground")
        self.assertIsNone(filt.classify((2.0, 0.0, 0.0)))

    def test_rotated_box_and_padding(self):
        box = OrientedBox(
            "platform", [0.0, 0.0, 0.0], [2.0, 0.2, 0.2],
            [0.0, 0.0, math.pi * 0.5],
        )
        self.assertTrue(box.contains((0.0, 0.95, 0.0)))
        self.assertFalse(box.contains((0.15, 0.0, 0.0)))
        self.assertTrue(box.contains((0.15, 0.0, 0.0), padding=0.06))


if __name__ == "__main__":
    unittest.main()
