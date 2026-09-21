#!/usr/bin/env python3
"""Multi-placement occupancy-aware place-path planner (ROS-free).

Takes K scored slots, expands carry-path variants per slot, sweeps occupancy,
optionally probes IK/cartesian fraction, and returns the lowest-cost
``(slot, path)`` pair. Nodes supply ``probe_fn`` and frame conversion;
this module must not import ``rclpy`` or ``moveit_msgs``.
"""

from __future__ import division

from luggage_planning.occupancy_place_paths import (
    DEFAULT_WEIGHTS,
    OccupancySnapshot,
    PLACE_PATH_INFEASIBLE,
    generate_path_variants,
    load_selector_weights,
    select_trajectory,
    sweep_polyline,
)


CARRY_SEGMENT_NAMES = ("transit", "traverse")


class PlacePathPlanner(object):
    """Expand (slot, path) candidates and select by weighted cost."""

    def __init__(self, weights=None, inflate_m=None, arm_radius=None,
                 max_candidates_per_slot=None, cartesian_min_fraction=None):
        cfg = dict(DEFAULT_WEIGHTS)
        if weights:
            cfg.update(weights)
        self.weights = cfg
        self.inflate_m = float(
            inflate_m if inflate_m is not None else cfg["inflate_m"])
        self.arm_radius = float(
            arm_radius if arm_radius is not None else cfg["arm_radius_m"])
        self.max_candidates_per_slot = int(
            max_candidates_per_slot if max_candidates_per_slot is not None
            else cfg["max_candidates_per_slot"])
        self.cartesian_min_fraction = float(
            cartesian_min_fraction if cartesian_min_fraction is not None
            else cfg["cartesian_min_fraction"])

    @classmethod
    def from_yaml(cls, path=None):
        return cls(weights=load_selector_weights(path))

    def plan(self, surface, slots, portal, payload_wdh,
             expected_hash=None, expected_revision=None,
             opening_tangent=None, probe_fn=None, yaw=0.0):
        """Return ``(winner, rows, reason, snapshot)``.

        ``slots`` is a list of dicts with at least ``target`` (XYZ above-slot
        in map frame), ``score``, and optional ``index``. ``portal`` is map
        frame XYZ. ``probe_fn(row)`` may set ``ik_ok`` and
        ``cartesian_fraction``; it may also set ``feasible=False``.
        """
        snapshot = OccupancySnapshot.from_surface_2d(
            surface, expected_hash=expected_hash,
            expected_revision=expected_revision, inflate_m=self.inflate_m)
        inner_h = float(snapshot.inner_size[2])
        payload_h = float(payload_wdh[2])
        rows = []
        for slot in slots:
            index = int(slot.get("index", len(rows)))
            target = tuple(float(v) for v in slot["target"])
            score = float(slot.get("score", 0.0))
            slot_yaw = float(slot.get("yaw", yaw))
            variants = generate_path_variants(
                portal, target, snapshot=snapshot,
                opening_tangent=opening_tangent, inner_h=inner_h,
                max_candidates=self.max_candidates_per_slot,
                payload_height=payload_h, payload_wdh=payload_wdh,
                yaw=slot_yaw, arm_radius=self.arm_radius,
                carry_margin=self.weights.get("carry_margin_m"),
                collision_pad=self.weights.get("collision_pad_m"),
                arm_overhead=self.weights.get("arm_overhead_m"))
            if len(variants) < 2:
                extra = generate_path_variants(
                    portal, target, snapshot=snapshot,
                    opening_tangent=opening_tangent, inner_h=inner_h,
                    max_candidates=8, payload_height=payload_h,
                    payload_wdh=payload_wdh, yaw=slot_yaw,
                    arm_radius=self.arm_radius,
                    carry_margin=self.weights.get("carry_margin_m"),
                    collision_pad=self.weights.get("collision_pad_m"),
                    arm_overhead=self.weights.get("arm_overhead_m"))
                for variant in extra:
                    if not variants or variant.method != variants[0].method:
                        variants.append(variant)
                    if len(variants) >= 2:
                        break
            for variant in variants:
                sweep = sweep_polyline(
                    snapshot, variant.waypoints, payload_wdh,
                    yaw=slot_yaw, arm_radius=self.arm_radius,
                    collision_pad=self.weights.get("collision_pad_m"))
                row = {
                    "slot_index": index,
                    "slot_score": score,
                    "method": variant.method,
                    "waypoints": list(variant.waypoints),
                    "feasible": (not sweep.collides),
                    "reason": sweep.reason or (
                        "ok" if not sweep.collides else "occupancy_collision"),
                    "min_clearance": sweep.min_clearance,
                    "first_hit_cell": sweep.first_hit_cell,
                    "cartesian_fraction": None,
                    "ik_ok": None,
                    "yaw": slot_yaw,
                }
                if sweep.collides:
                    row["feasible"] = False
                    row["reason"] = sweep.reason or "occupancy_collision"
                elif probe_fn is not None:
                    probe_fn(row)
                    if row.get("ik_ok") is False:
                        row["feasible"] = False
                        row["reason"] = row.get("reason") or "ik_failed"
                    frac = row.get("cartesian_fraction")
                    if (frac is not None
                            and float(frac) < self.cartesian_min_fraction
                            and not row.get("ompl_ok")):
                        row["feasible"] = False
                        row["reason"] = row.get("reason") or (
                            "cartesian_fraction_%.3f" % float(frac))
                rows.append(row)
        winner, rows, reason = select_trajectory(rows, self.weights)
        return winner, rows, reason, snapshot


def apply_variant_to_segment(segment, waypoints, frame_convert=None):
    """Write intermediate waypoints onto a MotionSegment-like object.

    Last waypoint becomes ``target_pose`` position (orientation kept).
    ``frame_convert(xyz) -> xyz`` maps the waypoints (e.g. cargo-map
    frame) into the segment's planning frame (e.g. world) before they are
    written; the selected carry variant stays in the map frame and each
    consumer converts with its own scene transform.
    """
    if not waypoints:
        return segment
    convert = frame_convert or (lambda xyz: xyz)
    intermediates = [convert(xyz) for xyz in waypoints[:-1]]
    dest = convert(waypoints[-1])
    orientation = segment.target_pose.orientation
    template = segment.target_pose

    def _pose_at(xyz):
        pose = type(template)()
        if hasattr(pose, "position") and hasattr(pose.position, "x"):
            pose.position.x = float(xyz[0])
            pose.position.y = float(xyz[1])
            pose.position.z = float(xyz[2])
            pose.orientation = orientation
            return pose
        from luggage_planning.pose import Point, Pose
        return Pose(
            position=Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2])),
            orientation=orientation)

    # Dataclass Pose() default-constructs nested Point; ROS Pose does too.
    try:
        segment.waypoints = [_pose_at(xyz) for xyz in intermediates]
        segment.target_pose = _pose_at(dest)
    except TypeError:
        from luggage_planning.pose import Point, Pose
        def _dc(xyz):
            return Pose(
                position=Point(x=float(xyz[0]), y=float(xyz[1]),
                               z=float(xyz[2])),
                orientation=orientation)
        segment.waypoints = [_dc(xyz) for xyz in intermediates]
        segment.target_pose = _dc(dest)
    return segment
