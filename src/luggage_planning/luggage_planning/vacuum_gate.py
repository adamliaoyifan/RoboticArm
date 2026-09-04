#!/usr/bin/env python3
"""Vacuum attach legality gate (no ROS, no rospy).

Pure geometry/physics checks deciding whether a VacuumCommand(enable) may
bind the box to the suction panel. Reuses the existing algorithm modules:
``vacuum_attach_utils.top_face_contact_ok`` (panel over the lid),
``vacuum_retention.retention_metrics`` (payload margin),
``downward_constraint_utils.link_z_tilt_deg`` (panel tilt).
"""

from __future__ import division

from luggage_planning.downward_constraint_utils import link_z_tilt_deg
from luggage_planning.vacuum_attach_utils import top_face_contact_ok
from luggage_planning.vacuum_retention import retention_metrics

# Failure-code prefixes (shared contract with the service message).
VACUUM_NO_BOX = "VACUUM_NO_BOX"
VACUUM_NOT_IN_CONTACT = "VACUUM_NOT_IN_CONTACT"
VACUUM_TILT_EXCEEDED = "VACUUM_TILT_EXCEEDED"
VACUUM_RETENTION_MARGIN = "VACUUM_RETENTION_MARGIN"

# Static accel assumption for the retention gate while the arm holds still
# at attach (the ROS 1 simulator used the same constant).
ATTACH_LINEAR_ACCEL_MPS2 = 2.0
ATTACH_ANGULAR_ACCEL_RADPS2 = 1.0


class VacuumGateResult(object):
    __slots__ = ("ok", "reason", "contact_distance", "retention_margin",
                 "tilt_deg")

    def __init__(self, ok, reason, contact_distance=float("nan"),
                 retention_margin=float("nan"), tilt_deg=float("nan")):
        self.ok = bool(ok)
        self.reason = str(reason)
        self.contact_distance = float(contact_distance)
        self.retention_margin = float(retention_margin)
        self.tilt_deg = float(tilt_deg)


class VacuumGate(object):
    """Decides whether attach is legal for the current panel/box state."""

    def __init__(self,
                 pressure_kpa=70.0,
                 effective_area_m2=0.012,
                 seal_efficiency=0.80,
                 friction_coefficient=0.60,
                 minimum_retention_margin=2.0,
                 max_suction_tilt_deg=5.0,
                 contact_margin=0.05,
                 contact_xy_margin=0.05,
                 contact_gap_min=-0.01):
        self.pressure_kpa = float(pressure_kpa)
        self.effective_area_m2 = float(effective_area_m2)
        self.seal_efficiency = float(seal_efficiency)
        self.friction_coefficient = float(friction_coefficient)
        self.minimum_retention_margin = float(minimum_retention_margin)
        self.max_suction_tilt_deg = float(max_suction_tilt_deg)
        self.contact_margin = float(contact_margin)
        self.contact_xy_margin = float(contact_xy_margin)
        self.contact_gap_min = float(contact_gap_min)

    def evaluate(self, panel_xyz, panel_orientation_xyzw,
                 box_xyz, box_size, box_mass_kg, payload_radius_m):
        """Check all attach conditions.

        ``panel_xyz`` / ``panel_orientation_xyzw``: suction panel (or
        contact frame) pose in world; ``box_*``: the tracked pickup box.
        Returns a :class:`VacuumGateResult`.
        """
        if box_xyz is None or box_size is None:
            return VacuumGateResult(False, VACUUM_NO_BOX)

        ok_contact, gap = top_face_contact_ok(
            panel_xyz, box_xyz, box_size,
            xy_margin=self.contact_xy_margin,
            gap_min=self.contact_gap_min,
            gap_max=self.contact_margin)
        tilt_deg = link_z_tilt_deg(panel_orientation_xyzw)

        if not ok_contact:
            return VacuumGateResult(
                False, VACUUM_NOT_IN_CONTACT, contact_distance=gap,
                tilt_deg=tilt_deg)
        if tilt_deg > self.max_suction_tilt_deg:
            return VacuumGateResult(
                False, VACUUM_TILT_EXCEEDED, contact_distance=gap,
                tilt_deg=tilt_deg)

        metrics = retention_metrics(
            max(0.0, float(box_mass_kg or 0.0)),
            tilt_deg,
            self.pressure_kpa,
            self.effective_area_m2,
            self.seal_efficiency,
            self.friction_coefficient,
            ATTACH_LINEAR_ACCEL_MPS2,
            ATTACH_ANGULAR_ACCEL_RADPS2,
            float(payload_radius_m),
        )
        if metrics["margin"] < self.minimum_retention_margin:
            return VacuumGateResult(
                False, VACUUM_RETENTION_MARGIN, contact_distance=gap,
                retention_margin=metrics["margin"], tilt_deg=tilt_deg)

        return VacuumGateResult(
            True, "ok", contact_distance=gap,
            retention_margin=metrics["margin"], tilt_deg=tilt_deg)
