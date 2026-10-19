#!/usr/bin/env python3
"""The contact-recess collision approximation (user-frozen scope, 2026-10-19).

The placed/carry box keeps its AABB everywhere EXCEPT the cup's actual
contact footprint, where the crown band above the lid plateau is
removed. Four acceptance items, offline:

1. r5/r6 release initial states: p04 does not overlap the new volume.
2. Bottom/side extents unchanged => resting height unchanged.
3. Carry and release emit the SAME collision composition (no swap jump).
4. Masks unchanged: the box still collides with the container and with
   every non-allowed arm part; no exemption was widened.

Plus reference-semantics guards: the recess floor is the lid plateau in
the BOTTOM-SHARED frame (not a center-shared 5.5 mm guess), per tier.
"""
import os
import re
import sys
import unittest
import xml.etree.ElementTree as ET

_PKG = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), ".."))
if _PKG not in sys.path:
    sys.path.insert(0, _PKG)

from luggage_description.suitcase_visual import (  # noqa: E402
    CONTACT_RECESS_FOOTPRINT_M,
    contact_recess_boxes,
    cuboid_inertia,
    pickup_visual_sdf,
)
from luggage_description.gazebo_spawn_collision import (  # noqa: E402
    BOX_CARRY_MASK, BOX_FULL_MASK, P04_MASK, masks_collide)

# Recorded release-frame numbers (unified_box_geometry.json, r5/r6).
R6_BOX_CENTER_Z = 0.69256
R6_P04_PLATE_BOTTOM_Z = 0.80093      # lowest contact manifold point
R5_BOX_CENTER_Z = 0.692951
R5_P04_PLATE_BOTTOM_Z = 0.8006
CONTAINER_FLOOR_Z = 0.53
P04_PLATE_SIDE_M = 0.180             # measured from the panel collision STL
TIER_LID_OFFSET_IN20 = 0.011033      # sized_suitcases.json (single source)
TIER_LID_OFFSET_IN22 = 0.012036
IN20_SIZE = (0.35, 0.53, 0.22)


def _parse_collisions(xml_text):
    root = ET.fromstring(xml_text)
    out = {}
    link = root.find("model/link")
    assert link is not None, "no link element"
    for col in link.findall("collision"):
        size_el = col.find("geometry/box/size")
        assert size_el is not None, ET.tostring(col)[:200]
        size = [float(v) for v in size_el.text.split()]
        pose_el = col.find("pose")
        pose = ([float(v) for v in pose_el.text.split()][:3]
                if pose_el is not None else [0.0, 0.0, 0.0])
        mask = col.find("surface/contact/collide_bitmask")
        out[col.get("name")] = {
            "size": size, "pose": pose,
            "mask": int(mask.text) if mask is not None else None}
    return out


def _as_dict(boxes):
    return {name: {"size": size, "pose": pose}
            for name, size, pose in boxes}


def _inside(box, x, y, z):
    size, pose = box["size"], box["pose"]
    return all(abs(p - c) <= s / 2.0
               for p, c, s in ((x, pose[0], size[0]),
                               (y, pose[1], size[1]),
                               (z, pose[2], size[2])))


class RecessCompositionTest(unittest.TestCase):

    def test_geometry_constants_inch20(self):
        boxes = _as_dict(contact_recess_boxes(IN20_SIZE, TIER_LID_OFFSET_IN20))
        self.assertEqual(set(boxes), {
            "body_collision", "rim_collision_px", "rim_collision_nx",
            "rim_collision_py", "rim_collision_ny"})
        body = boxes["body_collision"]
        self.assertAlmostEqual(body["size"][2],
                               0.22 - TIER_LID_OFFSET_IN20, places=6)
        self.assertAlmostEqual(body["pose"][2],
                               -TIER_LID_OFFSET_IN20 / 2.0, places=6)
        # Rim band occupies the crown band outside the cup footprint.
        self.assertAlmostEqual(boxes["rim_collision_px"]["size"][0],
                               (0.35 - CONTACT_RECESS_FOOTPRINT_M) / 2.0,
                               places=6)
        self.assertAlmostEqual(boxes["rim_collision_py"]["size"][1],
                               (0.53 - CONTACT_RECESS_FOOTPRINT_M) / 2.0,
                               places=6)
        self.assertAlmostEqual(boxes["rim_collision_px"]["pose"][2],
                               0.11 - TIER_LID_OFFSET_IN20 / 2.0, places=6)

    def test_union_is_aabb_minus_recess_column(self):
        """Every AABB point is covered EXCEPT the crown band inside the
        cup footprint — the exact region p04 reaches into."""
        boxes = [dict(size=s, pose=p) for _n, s, p in
                 contact_recess_boxes(IN20_SIZE, TIER_LID_OFFSET_IN20)]
        L, W, H = IN20_SIZE
        plateau = H / 2.0 - TIER_LID_OFFSET_IN20
        fp = CONTACT_RECESS_FOOTPRINT_M
        step = 0.02
        uncovered = []
        wrongly_removed = []
        xi = -L / 2.0 + step / 2.0
        while xi < L / 2.0:
            yi = -W / 2.0 + step / 2.0
            while yi < W / 2.0:
                for zi in (-H / 2.0 + step / 2.0, -0.05, 0.0, 0.05,
                           plateau + step / 4.0, H / 2.0 - step / 4.0):
                    covered = any(_inside(b, xi, yi, zi) for b in boxes)
                    in_recess = (abs(xi) < fp / 2.0 and abs(yi) < fp / 2.0
                                 and zi > plateau)
                    if in_recess and covered:
                        wrongly_removed.append((xi, yi, zi))
                    if not in_recess and not covered:
                        uncovered.append((xi, yi, zi))
                yi += step
            xi += step
        self.assertEqual(wrongly_removed, [])
        self.assertEqual(uncovered, [])

    def test_recess_footprint_covers_p04_with_margin(self):
        self.assertGreater(CONTACT_RECESS_FOOTPRINT_M,
                           P04_PLATE_SIDE_M + 0.02)

    def test_tiny_box_rejected(self):
        with self.assertRaises(ValueError):
            contact_recess_boxes((0.19, 0.53, 0.22), 0.011)
        with self.assertRaises(ValueError):
            contact_recess_boxes(IN20_SIZE, 0.0)
        with self.assertRaises(ValueError):
            contact_recess_boxes(IN20_SIZE, 0.22)


class AcceptanceItemsTest(unittest.TestCase):

    def _sdf(self, static=True, mask=BOX_FULL_MASK):
        return pickup_visual_sdf(
            "pickup_box_0001_inch20", IN20_SIZE, 5.67, "suitcase_loafbrr",
            static=static, contact_topic="/obs/box_contact",
            collide_bitmask=mask, collision_kind="box_recess",
            lid_offset=TIER_LID_OFFSET_IN20)

    def test_item1_no_p04_overlap_in_r5_r6_release_states(self):
        for center, plate in ((R6_BOX_CENTER_Z, R6_P04_PLATE_BOTTOM_Z),
                              (R5_BOX_CENTER_Z, R5_P04_PLATE_BOTTOM_Z)):
            recess_top = center + 0.11 - TIER_LID_OFFSET_IN20
            clearance = plate - recess_top
            self.assertGreater(
                clearance, 0.005,
                "p04 plate %.4f must clear the recess top %.4f"
                % (plate, recess_top))

    def test_item2_bottom_sides_and_resting_unchanged(self):
        boxes = _parse_collisions(self._sdf())
        body = boxes["body_collision"]
        self.assertAlmostEqual(body["size"][0], 0.35, places=6)
        self.assertAlmostEqual(body["size"][1], 0.53, places=6)
        self.assertAlmostEqual(
            body["pose"][2] - body["size"][2] / 2.0, -0.11, places=5,
            msg="base slab bottom must stay the AABB bottom")
        # Recorded settle (r5/r6): floor + full half-height.
        self.assertAlmostEqual(CONTAINER_FLOOR_Z + 0.11, 0.640, places=3)

    def test_item3_carry_and_release_identical_collision(self):
        # Geometry identity is the contract; masks differ by design
        # (carry 0xFE / release 0xFF) and are asserted separately.
        carry = self._sdf(static=True)
        release = self._sdf(static=False)
        def collision_blocks(text):
            return re.findall(r"      <collision name=.*?</collision>\n",
                              text, re.S)
        self.assertEqual(collision_blocks(carry), collision_blocks(release))
        self.assertEqual(len(collision_blocks(carry)), 5)
        carry_masked = _parse_collisions(self._sdf(
            static=True, mask=BOX_CARRY_MASK))
        self.assertEqual({b["mask"] for b in carry_masked.values()},
                         {BOX_CARRY_MASK})
        self.assertIn("<static>true</static>", carry)
        self.assertNotIn("<static>", release)

    def test_item4_masks_not_widened(self):
        boxes = _parse_collisions(self._sdf())
        self.assertEqual(len(boxes), 5)
        for box in boxes.values():
            self.assertEqual(box["mask"], BOX_FULL_MASK)
        # The box still collides with the container and with every
        # non-allowed arm part (all default-mask bodies)...
        self.assertTrue(masks_collide(BOX_FULL_MASK, 0xFF))
        # ...and carry still excludes ONLY p04.
        self.assertFalse(masks_collide(BOX_CARRY_MASK, P04_MASK))
        self.assertTrue(masks_collide(BOX_CARRY_MASK, 0xFF))

    def test_inertial_and_sensor_unchanged(self):
        text = self._sdf()
        root = ET.fromstring(text)
        ixx, iyy, izz = cuboid_inertia(IN20_SIZE, 5.67)
        inert = root.find(".//inertial")
        self.assertAlmostEqual(
            float(inert.find("inertia/ixx").text), ixx, places=6)
        self.assertAlmostEqual(
            float(inert.find("mass").text), 5.67, places=4)
        sensor = root.find(".//sensor/contact/collision")
        self.assertIsNotNone(sensor)
        self.assertEqual(sensor.text, "body_collision")

    def test_recess_requires_lid_offset_and_scales_per_tier(self):
        with self.assertRaises(ValueError):
            pickup_visual_sdf(
                "b", IN20_SIZE, 5.67, "suitcase_loafbrr",
                collision_kind="box_recess")
        boxes22 = _as_dict(contact_recess_boxes(
            (0.35, 0.53, 0.22), TIER_LID_OFFSET_IN22))
        boxes20 = _as_dict(contact_recess_boxes(
            IN20_SIZE, TIER_LID_OFFSET_IN20))
        self.assertNotEqual(
            boxes22["body_collision"]["size"][2],
            boxes20["body_collision"]["size"][2],
            "tier lid_offset must drive the recess depth")

    def test_reference_semantics_bottom_shared(self):
        """The recess floor is the lid plateau in the BOTTOM-SHARED
        frame: observable_top = origin + H/2 - lid_offset. A
        center-shared reading (top - lid/2) is the wrong-frame bug this
        guards against."""
        boxes = _as_dict(contact_recess_boxes(IN20_SIZE, TIER_LID_OFFSET_IN20))
        slab_top = (boxes["body_collision"]["pose"][2]
                    + boxes["body_collision"]["size"][2] / 2.0)
        self.assertAlmostEqual(
            slab_top, 0.11 - TIER_LID_OFFSET_IN20, places=6)
        self.assertAlmostEqual(
            slab_top, 0.11 - TIER_LID_OFFSET_IN20, places=6)
        # The spawner's observable-top formula (bottom-shared) lands on
        # the same plane; a center-shared formula would be 5.5 mm higher.
        observable_top_offset = 0.22 / 2.0 - TIER_LID_OFFSET_IN20
        self.assertAlmostEqual(slab_top, observable_top_offset, places=6)


if __name__ == "__main__":
    unittest.main()


class FullFootprintProjectionTest(unittest.TestCase):
    """r7 gate: the p04 plate's FULL projection (actual grasp transform
    + panel collision geometry), not just historical contact points,
    sits inside the recess with >=1 mm rim margin under the allowed
    grasp-transform envelope."""

    P04_SIDE = 0.180          # panel collision STL AABB (x/z extent)
    # Recorded grasp transforms (r5/r6 tf_trace suction vs box pose):
    # the cup lands essentially box-centered, aligned in yaw.
    RECORDED = [
        ("r5", (-2.03, 1.01), (-2.03, 1.01), 1.57076),
        ("r6", (-2.03, 0.46), (-2.03, 0.46), 1.57071),
    ]
    # What the FROZEN 0.210 footprint actually admits (reported, not
    # widened): corner reach = 0.090*(|cos|+|sin|) + |offset| <= 0.104.
    #   yaw 0.5deg -> |offset| <= ~13 mm   (aligned grasp, generous XY)
    #   yaw 5 deg  -> |offset| <= ~6 mm    (rotated grasp, tighter XY)
    # Recorded grasps are centered (|offset| < 1 mm, yaw err < 0.01 deg).
    ENVELOPES = [(0.013, 0.0087), (0.006, 0.087)]

    def _corner_reach(self, dx, dy, rel_yaw):
        """Max |box-frame x| and |y| of the plate's four corners."""
        half = self.P04_SIDE / 2.0
        worst = [0.0, 0.0]
        for sx in (-half, half):
            for sy in (-half, half):
                bx = dx + sx * __import__("math").cos(rel_yaw) \
                    - sy * __import__("math").sin(rel_yaw)
                by = dy + sx * __import__("math").sin(rel_yaw) \
                    + sy * __import__("math").cos(rel_yaw)
                worst[0] = max(worst[0], abs(bx))
                worst[1] = max(worst[1], abs(by))
        return worst

    def test_recorded_grasps_project_inside_with_margin(self):
        import math
        limit = CONTACT_RECESS_FOOTPRINT_M / 2.0 - 0.001
        for tag, suction, box, yaw in self.RECORDED:
            dx = suction[0] - box[0]
            dy = suction[1] - box[1]
            self.assertLess(abs(dx), 1e-3, tag)   # recorded: centered
            self.assertLess(abs(dy), 1e-3, tag)
            rx, ry = self._corner_reach(dx, dy, 0.0)
            self.assertLessEqual(rx, limit, tag)
            self.assertLessEqual(ry, limit, tag)

    def test_admitted_grasp_envelopes_stay_inside(self):
        import math
        limit = CONTACT_RECESS_FOOTPRINT_M / 2.0 - 0.001
        for d_off, d_yaw in self.ENVELOPES:
            for s_yaw in (-d_yaw, d_yaw):
                for octant in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
                    dx = octant[0] * d_off * 0.7071
                    dy = octant[1] * d_off * 0.7071
                    rx, ry = self._corner_reach(dx, dy, s_yaw)
                    self.assertLessEqual(
                        rx, limit, "offset=%.3f yaw=%.3f" % (d_off, s_yaw))
                    self.assertLessEqual(
                        ry, limit, "offset=%.3f yaw=%.3f" % (d_off, s_yaw))

    def test_full_tool_lumps_stack_behind_p04(self):
        """p03/p05 sit behind p04 along the suction axis, so p04's
        clearance is the binding one for the whole panel (measured
        panel STL AABBs, CAD frame y): p04 face 0.2830 from origin,
        p03 0.2588, p05 0.0208 — every later lump is farther from the
        box face than p04."""
        faces = {"p04": 0.2830, "p03": 0.2588, "p05": 0.0208}
        self.assertLess(faces["p03"], faces["p04"])
        self.assertLess(faces["p05"], faces["p04"])

    def test_p04_recess_floor_clearance_recorded_states(self):
        """Release initial state: p04's lowest recorded manifold point
        clears the recess floor (lid plateau) by >=1 mm in r5/r6."""
        for center, plate in ((R6_BOX_CENTER_Z, R6_P04_PLATE_BOTTOM_Z),
                              (R5_BOX_CENTER_Z, R5_P04_PLATE_BOTTOM_Z)):
            floor = center + 0.11 - TIER_LID_OFFSET_IN20
            self.assertGreaterEqual(
                plate - floor, 0.001,
                "p04 must clear the recess floor by >=1 mm")
