#!/usr/bin/env python3
"""Unit tests for downward_constraint_utils (no roscore required).

Covers the pure geometry: tilt-from-down, vector alignment, the
suction-down orientation derivation, the mount feasibility check, and the
per-point trajectory tilt validation. The TF wrapper
(``compute_downward_orientations``) is exercised via its pure helper
``downward_orientations_from_matrix`` with a known camera<->suction rotation.
"""

from __future__ import division

import math
import os
import sys
import unittest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

import luggage_planning.downward_constraint_utils as d  # noqa: E402


class TestDownwardMath(unittest.TestCase):
    def test_tilt_from_down(self):
        self.assertAlmostEqual(d.tilt_from_down((0.0, 0.0, -1.0)), 0.0, places=4)
        self.assertAlmostEqual(d.tilt_from_down((1.0, 0.0, 0.0)), 90.0, places=4)
        self.assertAlmostEqual(d.tilt_from_down((0.0, 0.0, 1.0)), 180.0, places=4)

    def test_angle_deg_between(self):
        self.assertAlmostEqual(
            d.angle_deg_between((0, 0, 1), (0, 0, -1)), 180.0, places=3)
        self.assertAlmostEqual(
            d.angle_deg_between((1, 0, 0), (0, 1, 0)), 90.0, places=3)

    def test_quaternion_align_vectors_identity(self):
        q = d.quaternion_align_vectors((0, 0, 1), (0, 0, 1))
        self.assertAlmostEqual(q[0], 0.0, places=6)
        self.assertAlmostEqual(q[1], 0.0, places=6)
        self.assertAlmostEqual(q[2], 0.0, places=6)
        self.assertAlmostEqual(q[3], 1.0, places=6)

    def test_quaternion_align_z_to_down(self):
        # Align +Z to -Z: a 180 deg rotation; w = cos(pi/2) = 0, |axis| = 1.
        q = d.quaternion_align_vectors((0, 0, 1), (0, 0, -1))
        self.assertAlmostEqual(q[3], 0.0, places=6)
        self.assertAlmostEqual(
            math.sqrt(q[0] ** 2 + q[1] ** 2 + q[2] ** 2), 1.0, places=6)

    def test_downward_orientations_inter_axis(self):
        # camera->suction rotation about Y by 12.25 deg: suction +Z in the
        # camera frame is (sin a, 0, cos a), 12.25 deg off the camera +Z.
        alpha = math.radians(12.25)
        ca, sa = math.cos(alpha), math.sin(alpha)
        rotation = [
            [ca, 0.0, sa],
            [0.0, 1.0, 0.0],
            [-sa, 0.0, ca],
        ]
        res = d.downward_orientations_from_matrix(rotation)
        self.assertAlmostEqual(res["inter_axis_deg"], 12.25, places=1)

        # suction_down_quat aligns suction +Z with base -Z: a 180 deg rotation
        # (q and -q represent the same rotation, so check sign-agnostic).
        sq = res["suction_down_quat"]
        self.assertAlmostEqual(sq[1], 0.0, places=5)
        self.assertAlmostEqual(sq[2], 0.0, places=5)
        self.assertAlmostEqual(abs(sq[0]), 1.0, places=5)
        self.assertAlmostEqual(sq[3], 0.0, places=5)

        # camera_down_quat: the camera +Z in base ends up inter_axis off -Z.
        cdq = res["camera_down_quat"]
        cam_z_in_base = d._rotate_vector(d._quaternion_to_matrix(cdq), (0, 0, 1))
        self.assertAlmostEqual(
            d.tilt_from_down(cam_z_in_base), 12.25, places=1)

    def test_feasibility_check(self):
        ok, _ = d.feasibility_check(12.25, 15.0, 5.0)
        self.assertTrue(ok)
        # Camera budget smaller than the inter-axis tilt is infeasible: the
        # suction-down orientation must tilt the camera by the inter-axis angle.
        ok, msg = d.feasibility_check(20.0, 15.0, 5.0)
        self.assertFalse(ok)
        self.assertIn("camera_max_tilt", msg)

    def test_link_z_tilt_deg(self):
        # Identity orientation: link +Z == base +Z -> points up -> 180 deg.
        self.assertAlmostEqual(d.link_z_tilt_deg((0, 0, 0, 1)), 180.0, places=4)
        # 180 deg about X: link +Z -> base -Z -> 0 deg.
        self.assertAlmostEqual(d.link_z_tilt_deg((1, 0, 0, 0)), 0.0, places=4)

    def test_validate_downward_tilts_ok(self):
        cams = [12.0, 12.5, 13.0]
        sucs = [0.0, 1.0, 2.0]
        ok, mc, ms, wi = d.validate_downward_tilts(cams, sucs, 15.0, 5.0)
        self.assertTrue(ok)
        self.assertAlmostEqual(mc, 13.0, places=6)
        self.assertAlmostEqual(ms, 2.0, places=6)
        self.assertEqual(wi, -1)

    def test_validate_downward_tilts_violation(self):
        cams = [12.0, 16.0, 13.0]  # 16 exceeds the 15 deg camera budget
        sucs = [0.0, 1.0, 2.0]
        ok, mc, ms, wi = d.validate_downward_tilts(cams, sucs, 15.0, 5.0)
        self.assertFalse(ok)
        self.assertEqual(wi, 1)
        self.assertAlmostEqual(mc, 16.0, places=6)


class TestTiltAzimuth(unittest.TestCase):
    """optical_axis_from_quat / yaw_about_down / align_tilt_azimuth.

    These back the phase0-camera-yaw fix: quaternion_align_vectors leaves an
    arbitrary azimuth for the suction-down camera's residual tilt, which in
    the field pointed at the robot's own pedestal instead of the container
    interior. Re-yawing about DOWN_AXIS (the rotation axis itself) must
    change only that azimuth, never the achieved tilt-from-down of either the
    camera axis or the suction normal -- otherwise it could silently violate
    the camera/suction tilt budgets ``feasibility_check`` already validated.
    """

    # The incident's observed opening-view quaternion: ~12.28 deg tilt whose
    # horizontal projection points toward +Y (the robot base), not into the
    # container (which is toward -Y in that scene).
    INCIDENT_QUAT = (-0.992, 0.067, 0.000, 0.107)

    def test_optical_axis_from_quat_identity(self):
        axis = d.optical_axis_from_quat((0.0, 0.0, 0.0, 1.0))
        self.assertAlmostEqual(axis[0], 0.0, places=6)
        self.assertAlmostEqual(axis[1], 0.0, places=6)
        self.assertAlmostEqual(axis[2], 1.0, places=6)

    def test_yaw_about_down_preserves_tilt_from_down(self):
        axis_before = d.optical_axis_from_quat(self.INCIDENT_QUAT)
        tilt_before = d.tilt_from_down(axis_before)
        rotated = d.yaw_about_down(self.INCIDENT_QUAT, math.radians(73.0))
        axis_after = d.optical_axis_from_quat(rotated)
        self.assertAlmostEqual(
            d.tilt_from_down(axis_after), tilt_before, places=4)

    def test_yaw_about_down_rotates_azimuth_by_the_given_angle(self):
        axis_before = d.optical_axis_from_quat(self.INCIDENT_QUAT)
        az_before = math.degrees(math.atan2(axis_before[1], axis_before[0]))
        rotated = d.yaw_about_down(self.INCIDENT_QUAT, math.radians(90.0))
        axis_after = d.optical_axis_from_quat(rotated)
        az_after = math.degrees(math.atan2(axis_after[1], axis_after[0]))
        delta = (az_after - az_before + 180.0) % 360.0 - 180.0
        self.assertAlmostEqual(delta, 90.0, places=2)

    def test_align_tilt_azimuth_points_toward_target_direction(self):
        before = d.optical_axis_from_quat(self.INCIDENT_QUAT)
        self.assertGreater(before[1], 0.0)  # currently points at +Y (base)

        corrected = d.align_tilt_azimuth(self.INCIDENT_QUAT, (0.0, -1.0))
        after = d.optical_axis_from_quat(corrected)
        # Tilt-from-down is unchanged...
        self.assertAlmostEqual(
            d.tilt_from_down(after), d.tilt_from_down(before), places=4)
        # ...but the horizontal projection now points into the container.
        self.assertLess(after[1], 0.0)
        self.assertAlmostEqual(after[0], 0.0, places=3)

    def test_align_tilt_azimuth_keeps_suction_on_down_axis(self):
        # Realistic 12.25 deg mount: confirm the suction normal is still
        # exactly on base -Z after re-aiming the camera's azimuth, so
        # align_tilt_azimuth never needs to re-run feasibility_check.
        alpha = math.radians(12.25)
        ca, sa = math.cos(alpha), math.sin(alpha)
        rotation = [[ca, 0.0, sa], [0.0, 1.0, 0.0], [-sa, 0.0, ca]]
        res = d.downward_orientations_from_matrix(rotation)
        corrected = d.align_tilt_azimuth(res["camera_down_quat"], (1.0, 1.0))

        suction_in_base = d._rotate_vector(
            d._quaternion_to_matrix(corrected), res["suction_normal_in_cam"])
        self.assertAlmostEqual(
            d.tilt_from_down(suction_in_base), 0.0, places=3)

        cam_axis = d.optical_axis_from_quat(corrected)
        self.assertAlmostEqual(
            d.tilt_from_down(cam_axis), res["inter_axis_deg"], places=1)

    def test_align_tilt_azimuth_degenerate_camera_returns_unchanged(self):
        # A camera pointing exactly straight down has zero horizontal
        # projection to align -- nothing to do.
        straight_down = d.quaternion_align_vectors(
            (0.0, 0.0, 1.0), (0.0, 0.0, -1.0))
        result = d.align_tilt_azimuth(straight_down, (1.0, 0.0))
        self.assertEqual(result, straight_down)

    def test_align_tilt_azimuth_degenerate_target_returns_unchanged(self):
        result = d.align_tilt_azimuth(self.INCIDENT_QUAT, (0.0, 0.0))
        self.assertEqual(result, self.INCIDENT_QUAT)


if __name__ == "__main__":
    unittest.main()
