#!/usr/bin/env python3
"""Unit tests for configurable pick/place waypoint generation."""
import math
import os
import re
import unittest

from luggage_planning.pose import Point, Pose, Quaternion
from luggage_planning.waypoint_generator import (
    DEFAULT_PICK_CLEARANCES,
    DEFAULT_PLACE_CLEARANCE_Z,
    build_sequence,
    insertion_clearance,
    pick_tool_yaw,
    segment_names_for_phase,
    staging_offset,
    _perception_clearances,
)


class Box:
    def __init__(self):
        self.pose = Pose(
            position=Point(x=1.0, y=2.0, z=0.25),
            orientation=Quaternion(w=1.0),
        )
        self.height = 0.50


class Slot:
    def __init__(self):
        self.place_pose = Pose(
            position=Point(x=3.0, y=4.0, z=0.20),
            orientation=Quaternion(w=1.0),
        )


class TestWaypointGenerator(unittest.TestCase):
    def test_default_pick_clearances_match_the_launched_values(self):
        """Module defaults and launch args must agree.

        They used to differ (approach 0.10 here vs 0.25 in the launch file), so
        the effective clearance depended on how the node was started. The
        module now carries the launched values as the single source.
        """
        segs = build_sequence(Box(), Slot(), "pick")
        self.assertEqual(
            [s.name for s in segs],
            ["pre_grasp", "approach", "attach", "pick_retreat"])
        self.assertTrue(segs[-1].keep_tool_down)
        # top_z = 0.25 + 0.50*0.5 = 0.50
        self.assertAlmostEqual(segs[0].target_pose.position.z, 0.80)  # +0.30
        self.assertAlmostEqual(segs[1].target_pose.position.z, 0.75)  # +0.25
        self.assertAlmostEqual(segs[2].target_pose.position.z, 0.50)  # +0.00 (contact)
        self.assertAlmostEqual(segs[3].target_pose.position.z, 0.85)  # +0.35

    def test_launch_defaults_match_module_defaults(self):
        """Guards against the three-way default drift returning."""
        launch = os.path.abspath(os.path.join(
            os.path.dirname(__file__), "..", "..",
            "luggage_bringup", "launch", "active_loading.launch"))
        if not os.path.isfile(launch):
            self.skipTest("bringup launch not present")
        text = open(launch).read()

        def launch_default(name):
            match = re.search(
                r'<arg name="%s" default="([-0-9.]+)"' % name, text)
            self.assertIsNotNone(match, "%s arg missing" % name)
            return float(match.group(1))

        self.assertAlmostEqual(
            launch_default("place_clearance_z"), DEFAULT_PLACE_CLEARANCE_Z)
        for arg, key in (
                ("pick_pre_grasp_clearance", "pre_grasp"),
                ("pick_approach_clearance", "approach"),
                ("pick_attach_clearance", "attach"),
                ("pick_retreat_clearance", "pick_retreat")):
            self.assertAlmostEqual(
                launch_default(arg), DEFAULT_PICK_CLEARANCES[key],
                msg="%s drifted from the module default" % arg)

    def test_insertion_depth_scales_with_box_height(self):
        """A fixed 0.10 m insert offset meant a different fraction of every box.

        With continuously sized boxes the offset has to follow the box, bounded
        by the transit clearance and a floor that keeps the cup clear.
        """
        short = insertion_clearance(0.25, place_clearance_z=0.15)
        tall = insertion_clearance(0.32, place_clearance_z=0.15)
        self.assertGreater(tall, short)
        self.assertGreaterEqual(short, 0.06)
        self.assertLessEqual(tall, 0.15)

    def test_insertion_depth_never_exceeds_transit_clearance(self):
        self.assertLessEqual(insertion_clearance(1.0, 0.15), 0.15)

    def test_insertion_depth_has_a_floor_for_flat_boxes(self):
        self.assertAlmostEqual(insertion_clearance(0.01, 0.15), 0.06)

    def test_pick_clearances_are_configurable_for_contact_diagnostics(self):
        segs = build_sequence(
            Box(),
            Slot(),
            "pick",
            pick_clearances={"pre_grasp": 0.40, "approach": 0.18, "attach": 0.14, "pick_retreat": 0.50},
        )
        self.assertAlmostEqual(segs[0].target_pose.position.z, 0.90)
        self.assertAlmostEqual(segs[1].target_pose.position.z, 0.68)
        self.assertAlmostEqual(segs[2].target_pose.position.z, 0.64)
        self.assertAlmostEqual(segs[3].target_pose.position.z, 1.00)

    def test_place_clearance_is_configurable(self):
        segs = build_sequence(Box(), Slot(), "place", place_clearance_z=0.42)
        self.assertEqual(segs[0].name, "transit")
        # slot center 0.20 + half box height 0.25 + clearance 0.42
        self.assertAlmostEqual(segs[0].target_pose.position.z, 0.87)
        self.assertAlmostEqual(segs[3].target_pose.position.z, 0.45)
        self.assertEqual(
            [segment.type for segment in segs],
            ["pose_target", "cartesian", "cartesian", "cartesian", "cartesian"],
        )
        self.assertTrue(all(segment.keep_tool_down for segment in segs))
        self.assertFalse(
            segs[0].lock_wrist,
            "tool-down is the safety constraint; redundant wrist-lock can "
            "make valid transit goals unsampleable",
        )
        self.assertFalse(
            segs[0].keep_camera_down,
            "tool-down already fixes the rigid camera mount; duplicating the "
            "camera guard can disconnect the constrained transit sampler",
        )

    def test_place_opening_portal_precedes_horizontal_traverse(self):
        info = {
            "point": [0.0, -0.75, 0.44],
            "normal": [0.0, 1.0, 0.0],
            "outward_clearance": 0.15,
        }
        segs = build_sequence(
            Box(), Slot(), "place", opening_info=info)
        self.assertEqual(
            [segment.name for segment in segs],
            ["stage", "transit", "traverse", "insert", "descend", "retreat"],
        )
        self.assertAlmostEqual(
            segs[0].target_pose.position.y, 0.05, places=6)
        self.assertAlmostEqual(
            segs[1].target_pose.position.y, -0.60, places=6)
        self.assertAlmostEqual(
            segs[2].target_pose.position.y,
            Slot().place_pose.position.y,
            places=6,
        )

    def test_staging_offset_is_three_axis(self):
        world_offset = staging_offset([-1.0, 0.0, 0.0], 0.65)
        base_offset = staging_offset([0.0, 1.0, 0.0], 0.65)
        self.assertAlmostEqual(
            math.sqrt(sum(v * v for v in world_offset)), 0.65, places=6)
        self.assertAlmostEqual(
            math.sqrt(sum(v * v for v in base_offset)), 0.65, places=6)
        self.assertAlmostEqual(world_offset[0], -0.65, places=6)
        self.assertAlmostEqual(base_offset[1], 0.65, places=6)

    def test_place_staging_along_world_negative_x_is_not_degenerate(self):
        info = {
            "point": [0.755, -0.27, 1.30],
            "normal": [-1.0, 0.0, 0.0],
            "outward_clearance": 0.15,
            "stage_outward_clearance": 0.65,
        }
        segs = build_sequence(Box(), Slot(), "place", opening_info=info)
        names = [segment.name for segment in segs]
        self.assertIn("stage", names)
        stage = next(s for s in segs if s.name == "stage")
        transit = next(s for s in segs if s.name == "transit")
        dx = stage.target_pose.position.x - transit.target_pose.position.x
        dy = stage.target_pose.position.y - transit.target_pose.position.y
        self.assertAlmostEqual(dx, -0.65, places=5)
        self.assertAlmostEqual(dy, 0.0, places=5)
        self.assertGreater(
            math.hypot(dx, dy), 0.1,
            "staging collapsed onto the portal (Y-only extrusion bug)")

    def test_pick_and_place_lift_off_names_are_distinct(self):
        """Pick lift-off is pick_retreat; place lift-off stays retreat."""
        self.assertEqual(
            segment_names_for_phase("pick"),
            ["pre_grasp", "approach", "attach", "pick_retreat"],
        )
        pick_names = [s.name for s in build_sequence(Box(), Slot(), "pick")]
        self.assertEqual(pick_names, segment_names_for_phase("pick"))
        self.assertNotIn("retreat", pick_names)
        self.assertIn("pick_retreat", DEFAULT_PICK_CLEARANCES)
        self.assertNotIn("retreat", DEFAULT_PICK_CLEARANCES)

        place_names = [s.name for s in build_sequence(Box(), Slot(), "place")]
        self.assertEqual(place_names, segment_names_for_phase("place"))
        self.assertEqual(place_names[-1], "retreat")
        self.assertNotEqual(pick_names[-1], place_names[-1])

    def test_pick_tool_yaw_keeps_detection_when_valid(self):
        self.assertAlmostEqual(pick_tool_yaw(1.2, True, fallback_yaw=0.3), 1.2)

    def test_pick_tool_yaw_uses_fallback_when_invalid(self):
        self.assertAlmostEqual(pick_tool_yaw(1.2, False, fallback_yaw=0.3), 0.3)

    def test_invalid_detection_yaw_does_not_steer_pick(self):
        box = Box()
        box.yaw_valid = False
        c, s = math.cos(0.3 * 0.5), math.sin(0.3 * 0.5)
        segs = build_sequence(box, Slot(), "pick", fallback_yaw=0.3)
        for seg in segs:
            q = seg.target_pose.orientation
            self.assertAlmostEqual(q.x, c)
            self.assertAlmostEqual(q.y, s)
            self.assertAlmostEqual(q.z, 0.0)
            self.assertAlmostEqual(q.w, 0.0)

    def test_valid_detection_yaw_steers_pick(self):
        yaw = math.pi / 4.0
        box = Box()
        box.pose.orientation = Quaternion(
            x=0.0, y=0.0,
            z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5),
        )
        box.yaw_valid = True
        segs = build_sequence(box, Slot(), "pick", fallback_yaw=0.0)
        c, s = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        q = segs[0].target_pose.orientation
        self.assertAlmostEqual(q.x, c)
        self.assertAlmostEqual(q.y, s)


class TestPerceptionApproachClearance(unittest.TestCase):
    def test_clearance_computation(self):
        info = {"box_top_z": 1.0, "suction_z": 1.5}
        c = _perception_clearances(info, None)
        self.assertAlmostEqual(c["pre_grasp"], 0.5 * 0.6, places=4)
        self.assertAlmostEqual(c["approach"], 0.5 * 0.3, places=4)
        self.assertAlmostEqual(c["attach"], 0.0)
        self.assertAlmostEqual(c["pick_retreat"], 0.35)

        info2 = {"box_top_z": 1.0, "suction_z": 1.1}
        c2 = _perception_clearances(info2, None)
        self.assertAlmostEqual(c2["pre_grasp"], 0.20)
        self.assertAlmostEqual(c2["approach"], 0.08)


class TestCorridorClearance(unittest.TestCase):
    """G1: traverse/extract heights must clear the tallest corridor neighbor.

    The payload hangs a full box height below the suction frame (box top =
    suction frame during tool-down carry), so the required suction height is
    surface_max + box_height + margin (docs/plans/corridor_constraints.md).
    """

    def _segments(self, corridor_surface_max=None, box_height=0.30,
                  slot_z=0.655, margin=0.05):
        box = Box()
        slot = Slot()
        slot.place_pose.position.z = slot_z
        slot.height = box_height
        return build_sequence(
            box, slot, "place",
            opening_info={
                "point": [0.755, 0.0, 1.30],
                "normal": [-1.0, 0.0, 0.0],
                "outward_clearance": 0.15,
                "min_height_above_opening": 0.35,
                "stage_outward_clearance": 0.65,
            },
            corridor_surface_max=corridor_surface_max,
            corridor_margin=margin,
        )

    def test_none_corridor_keeps_single_slot_behavior(self):
        segs_none = self._segments(corridor_surface_max=None)
        segs_default = self._segments()
        for a, b in zip(segs_none, segs_default):
            self.assertAlmostEqual(a.target_pose.position.z,
                                   b.target_pose.position.z, places=9)

    def test_traverse_retreat_raised_above_tall_neighbor(self):
        box_height = 0.30
        slot_z = 0.655
        contact_z = slot_z + box_height * 0.5  # 0.805
        surface_max = 1.30  # a neighbor 0.5 m taller than our slot top
        margin = 0.05
        segs = self._segments(corridor_surface_max=surface_max,
                              box_height=box_height, slot_z=slot_z,
                              margin=margin)
        by_name = {s.name: s for s in segs}
        required = surface_max + box_height + margin  # 1.65
        self.assertGreaterEqual(
            by_name["traverse"].target_pose.position.z, required)
        self.assertGreaterEqual(
            by_name["retreat"].target_pose.position.z, required)
        # Payload bottom (suction - box_height) clears by the margin.
        self.assertGreaterEqual(
            by_name["retreat"].target_pose.position.z - box_height
            - surface_max, margin - 1e-9)

    def test_low_neighbor_keeps_default_clearance(self):
        # Neighbor below the slot top: no raise needed beyond default 0.15.
        segs = self._segments(corridor_surface_max=0.60)
        by_name = {s.name: s for s in segs}
        self.assertAlmostEqual(
            by_name["traverse"].target_pose.position.z, 0.805 + 0.15,
            places=6)

    def test_insert_descend_unchanged_by_corridor(self):
        # Slot-column segments stay at their own heights regardless of the
        # corridor: insert/descend only descend inside the slot column.
        segs_a = self._segments(corridor_surface_max=None)
        segs_b = self._segments(corridor_surface_max=1.30)
        for name in ("insert", "descend"):
            za = [s for s in segs_a if s.name == name][0].target_pose.position.z
            zb = [s for s in segs_b if s.name == name][0].target_pose.position.z
            self.assertAlmostEqual(za, zb, places=9, msg=name)

    def test_corridor_clearance_unit(self):
        from luggage_planning.waypoint_generator import corridor_clearance
        # None -> passthrough.
        self.assertEqual(corridor_clearance(None, 0.3, 0.8, 0.15), 0.15)
        # Required below default -> default.
        self.assertEqual(
            corridor_clearance(0.60, 0.30, 0.805, 0.15), 0.15)
        # Required above default -> surface + box + margin - contact_z.
        self.assertAlmostEqual(
            corridor_clearance(1.30, 0.30, 0.805, 0.15),
            1.30 + 0.30 + 0.05 - 0.805, places=9)


if __name__ == "__main__":
    unittest.main()
