#!/usr/bin/env python3
"""Unit tests for configurable pick/place waypoint generation."""
import math
import os
import re
import unittest

from luggage_planning.geometry.pose import Point, Pose, Quaternion
from luggage_planning.place.waypoint_generator import (
    DEFAULT_PICK_CLEARANCES,
    DEFAULT_PLACE_CLEARANCE_Z,
    PLACE_RELEASE_GAP_M,
    build_sequence,
    corridor_clearance,
    insertion_clearance,
    nearest_box_yaw,
    pick_tool_yaw,
    segment_names_for_phase,
    staging_offset,
    tool_down_yaw,
    _perception_clearances,
    _tool_down_quaternion,
)


class Box:
    def __init__(self):
        self.pose = Pose(
            position=Point(x=1.0, y=2.0, z=0.25),
            orientation=Quaternion(w=1.0),
        )
        self.height = 0.50
        # Platform-free contract: fixture boxes carry measured geometry.
        self.height_valid = True
        self.top_surface_valid = False


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

    def test_staging_offset_is_three_axis(self):
        world_offset = staging_offset([-1.0, 0.0, 0.0], 0.65)
        base_offset = staging_offset([0.0, 1.0, 0.0], 0.65)
        self.assertAlmostEqual(
            math.sqrt(sum(v * v for v in world_offset)), 0.65, places=6)
        self.assertAlmostEqual(
            math.sqrt(sum(v * v for v in base_offset)), 0.65, places=6)
        self.assertAlmostEqual(world_offset[0], -0.65, places=6)
        self.assertAlmostEqual(base_offset[1], 0.65, places=6)

    def test_pick_tool_yaw_keeps_detection_when_valid(self):
        self.assertAlmostEqual(pick_tool_yaw(1.2, True, fallback_yaw=0.3), 1.2)

    def test_pick_top_z_prefers_top_surface_pose(self):
        box = Box()
        box.top_surface_valid = True
        box.top_surface_pose = Pose(position=Point(x=1.0, y=2.0, z=0.71))
        # Measured top (0.71) beats the legacy center+height/2 (0.50).
        segs = build_sequence(box, Slot(), "pick")
        self.assertAlmostEqual(
            segs[0].target_pose.position.z, 0.71 + DEFAULT_PICK_CLEARANCES["pre_grasp"])

    def test_pick_top_z_rejects_prior_only_height(self):
        box = Box()
        box.height_valid = False
        with self.assertRaises(ValueError) as ctx:
            build_sequence(box, Slot(), "pick")
        self.assertIn("DETECT_FULL_GEOMETRY_REQUIRED", str(ctx.exception))

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


def _exit_quat(measured_x, measured_y):
    """What `_exit_to_portal` builds for a wrist holding this quaternion."""
    yaw = nearest_box_yaw(tool_down_yaw(measured_x, measured_y), 0.0)
    return math.cos(0.5 * yaw), math.sin(0.5 * yaw)


class TestExitPortalYaw(unittest.TestCase):
    """The portal hop must keep the wrist the place left the arm in.

    Portal yaw is free (`keep_tool_down` allows ±π about Z), so asking for
    yaw 0 from a wrist at yaw ±π is the same heading commanded as a 180°
    wrist change. 1838 answered that with a 6 rad joint-1 unwind.
    """

    def test_wrist_at_plus_pi_keeps_plus_pi(self):
        x, y = _exit_quat(0.0, 1.0)
        self.assertAlmostEqual(x, 0.0, places=3)
        self.assertAlmostEqual(y, 1.0, places=3)

    def test_wrist_at_minus_pi_keeps_minus_pi(self):
        """[0,-1,0,0] must not be answered with [0,+1,0,0]."""
        x, y = _exit_quat(0.0, -1.0)
        self.assertAlmostEqual(x, 0.0, places=3)
        self.assertAlmostEqual(y, -1.0, places=3)

    def test_measured_1838_retreat_quaternion(self):
        """The exact orientation the arm held when the old exit flipped it."""
        x, y = _exit_quat(-1.6e-16, 1.0)
        self.assertAlmostEqual(x, 0.0, places=3)
        self.assertAlmostEqual(y, 1.0, places=3)

    def test_wrist_near_zero_still_asks_for_zero(self):
        x, y = _exit_quat(1.0, 0.0)
        self.assertAlmostEqual(x, 1.0, places=3)
        self.assertAlmostEqual(y, 0.0, places=3)
        x, y = _exit_quat(math.cos(0.05), math.sin(0.05))
        self.assertAlmostEqual(x, 1.0, places=3)
        self.assertAlmostEqual(y, 0.0, places=3)

    def test_yaw_is_the_inverse_of_the_quaternion_builder(self):
        for yaw in (0.0, 0.3, -0.4, math.pi, -math.pi, 2.0):
            quat = _tool_down_quaternion(yaw)
            self.assertAlmostEqual(tool_down_yaw(quat.x, quat.y), yaw,
                                   places=9)


class TestNearestBoxYaw(unittest.TestCase):
    def test_pi_slot_stays_with_pi_carry(self):
        self.assertAlmostEqual(nearest_box_yaw(math.pi, 0.0), math.pi)
        self.assertAlmostEqual(nearest_box_yaw(0.0, math.pi), 0.0)
        self.assertAlmostEqual(nearest_box_yaw(-math.pi, 0.0), -math.pi)

    def test_same_heading_is_unchanged(self):
        self.assertAlmostEqual(nearest_box_yaw(0.3, 0.3), 0.3)
        self.assertAlmostEqual(nearest_box_yaw(-0.4, -0.4), -0.4)


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

    def test_insertion_clearance_is_capped_by_corridor_clearance(self):
        # [ST09-I] pure pin: insertion starts below the traverse height
        # and above the floor for flat boxes.
        self.assertLessEqual(
            insertion_clearance(0.30, corridor_clearance(
                None, 0.30, 0.80, 0.15)), 0.15)
        self.assertGreaterEqual(insertion_clearance(0.05, 0.15), 0.06)

    def test_none_corridor_keeps_default_clearance(self):
        # [ST09-I] pure pin: the corridor helper is the route anchor
        # generator now; None corridor keeps the fixed clearance.
        self.assertEqual(
            corridor_clearance(None, 0.30, 0.80, 0.15), 0.15)

    def test_tall_neighbor_raises_clearance(self):
        # required suction z = surface_max + box_height + margin.
        self.assertAlmostEqual(
            corridor_clearance(1.30, 0.30, 0.80, 0.15, margin=0.05),
            1.30 + 0.30 + 0.05 - 0.80)
        # and never below the fixed clearance.
        self.assertEqual(
            corridor_clearance(0.50, 0.30, 0.80, 0.15, margin=0.05),
            0.15)

    def test_corridor_clearance_unit(self):
        from luggage_planning.place.waypoint_generator import corridor_clearance
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
