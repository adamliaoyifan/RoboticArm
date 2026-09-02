#!/usr/bin/env python3
"""ROS node: rule-based placement candidate generator on the 2.5D surface map.

Consumes ``/luggage/cargo_map/surface_2d`` (from cargo_volume_mapper) and the
current box (``/luggage/current_box`` or service request), produces scored
candidate center poses, publishes them as a param + RViz markers, and exposes
a ``ComputePlacement`` service that returns the best feasible candidate as a
``SlotSpec`` (drop-in for the legacy bin_packer interface).
"""

from __future__ import division

import math
import os
import sys

import rospy
import rospkg
from geometry_msgs.msg import Pose, Point, Quaternion
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import MarkerArray

from luggage_msgs.msg import DetectedLuggage, SlotSpec
from luggage_msgs.srv import ComputePlacement, ComputePlacementResponse

PKG_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_packing"), "scripts")
DESC_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_description"), "scripts")
PLAN_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_planning"), "scripts")
for _path in (PKG_SCRIPTS, DESC_SCRIPTS, PLAN_SCRIPTS):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from placement_solver import generate_candidates, best_candidate  # noqa: E402
from free_space_model import FreeSpaceModel  # noqa: E402
from ems import EMS  # noqa: E402
from geometry_profile import GeometryProfile  # noqa: E402
from placement_markers import build_candidate_markers  # noqa: E402
from placement_reachability import (  # noqa: E402
    annotate_with_atlas,
    resolve_atlas_path,
)
from placement_scoring import (  # noqa: E402
    DEFAULT_W_FLOOR_FIRST,
    score_candidates,
)
from scene_tf_config_utils import (  # noqa: E402
    container_in_base_link,
    container_interior_center_in_base_link,
    container_inner_dimensions,
    container_inner_floor_z,
    container_opening_aperture_corners_in_container,
    load_scene_tf_config,
    resolve_scene_tf_config_path,
)

try:
    from reachability_atlas import ReachabilityAtlas
except ImportError:  # numpy/yaml present in all supported envs; guard anyway
    ReachabilityAtlas = None

ATLAS_DATA_DIR = os.path.join(
    rospkg.RosPack().get_path("luggage_planning"), "data", "reachability_atlas")


class PlacementPlannerNode:
    BOX_SOURCE_POLICIES = ("request_only", "request_or_param", "param_only")

    def __init__(self):
        self._base_frame = rospy.get_param("~base_frame", "elfin_base_link")
        self._box_source_policy = self._load_box_source_policy()
        self._allowed_yaws = [
            float(v)
            for v in rospy.get_param(
                "~allowed_yaws",
                [0.0, math.pi / 2.0, math.pi, -math.pi / 2.0])
        ]
        self._params = {
            "clearance_margin": float(rospy.get_param("~clearance_margin", 0.03)),
            "boundary_margin": float(rospy.get_param("~boundary_margin", 0.05)),
            "min_support_ratio": float(rospy.get_param("~min_support_ratio", 0.6)),
            "support_tol": float(rospy.get_param("~support_tol", 0.05)),
            "stride_cells": int(rospy.get_param("~stride_cells", 1)),
            "top_n": int(rospy.get_param("~top_n", 8)),
        }
        self._surface_param = rospy.get_param(
            "~surface_param", "/luggage/cargo_map/surface_2d"
        )
        # Event-driven by default. A periodic timer can overwrite the
        # motion-filtered candidate list with stale raw candidates.
        self._publish_rate = float(rospy.get_param("~publish_rate", 0.0))
        self._use_free_space_model = bool(rospy.get_param(
            "~use_free_space_model", True))
        # Reachability atlas: cheap per-candidate prior + conservative hard
        # reject. MoveIt IK in placement_motion_filter remains authoritative.
        self._use_atlas = bool(rospy.get_param(
            "~use_reachability_atlas", True))
        self._atlas_dir = rospy.get_param("~atlas_dir", ATLAS_DATA_DIR)
        self._atlas_basename = rospy.get_param(
            "~atlas_basename", "s20_container_collision_aware")
        self._atlas_cache = {}
        # Lateral placement bounds come from the opening aperture (geometry),
        # depth is left to the reachability atlas. The hand-tuned near-ROI
        # rectangle it replaces cut usable floor area to 38% of the container
        # regardless of what the arm could actually reach; it stays available
        # as an explicit debugging override only.
        self._near_roi_enabled = bool(rospy.get_param(
            "~near_roi/enabled", False))
        self._near_roi_x_min = float(rospy.get_param(
            "~near_roi/x_min", -1.0e9))
        self._near_roi_x_max = float(rospy.get_param(
            "~near_roi/x_max", 1.0e9))
        self._near_roi_y_min = float(rospy.get_param(
            "~near_roi/y_min", -1.0e9))
        self._near_roi_y_max = float(rospy.get_param(
            "~near_roi/y_max", 1.0e9))
        self._aperture_gate_enabled = bool(rospy.get_param(
            "~aperture_gate/enabled", True))
        self._aperture_margin = float(rospy.get_param(
            "~aperture_gate/margin", 0.0))
        # Scoring weights. floor_first offsets the structural stack bonus that
        # proxy_score carries via observation_confidence and support_quality:
        # an unobserved floor column scores 0 on both, a committed box top
        # scores 1 on both, so without this term the ranking prefers stacking
        # over an empty floor.
        self._w_floor_first = float(rospy.get_param(
            "~score/w_floor_first", DEFAULT_W_FLOOR_FIRST))

        # Floor-prior fallback (design §4.2.2): when surface_2d is unavailable
        # (empty container / perception not yet integrated), generate candidates
        # on the a-priori floor using container geometry from scene_tf instead
        # of failing. The floor's existence is geometric prior, not perception.
        self._scene_config = load_scene_tf_config(
            rospy.get_param(
                "~scene_tf_config",
                rospy.get_param("/luggage/scene_tf_config", resolve_scene_tf_config_path()),
            )
        )
        self._floor_prior_resolution = float(
            rospy.get_param("~floor_prior_resolution", 0.05)
        )
        self._floor_prior_surface = None  # lazily built; fields match surface_map_2d
        self._aperture_bounds = self._compute_aperture_bounds()
        self._opening_side = str(
            self._scene_config.get("container", {})
            .get("opening", {})
            .get("side", "negative_x"))
        self._smallest_box_size = self._load_smallest_box_size()

        self._markers_pub = rospy.Publisher(
            "/luggage/placement/candidate_markers", MarkerArray, queue_size=1, latch=True
        )

        rospy.Service("~plan_placements", Trigger, self.handle_plan)
        rospy.Service("~compute_placement", ComputePlacement, self.handle_compute)

        if self._publish_rate > 0.0:
            rospy.Timer(rospy.Duration(1.0 / self._publish_rate), self._on_timer)

    def _load_box_source_policy(self):
        """Which box-size source this node is allowed to use."""
        policy = str(rospy.get_param(
            "~box_source_policy", "request_or_param")).strip().lower()
        if policy not in self.BOX_SOURCE_POLICIES:
            raise ValueError(
                "invalid ~box_source_policy %r, expected one of %s"
                % (policy, ", ".join(self.BOX_SOURCE_POLICIES)))
        return policy

    def _box_size_from_request(self, box):
        box_size = [float(box.width), float(box.depth), float(box.height)]
        if min(box_size) <= 0.0:
            return None
        return box_size

    def _select_box_size(self, request_box=None):
        """Resolve the current box size according to the launch-time policy."""
        if self._box_source_policy != "param_only" and request_box is not None:
            box_size = self._box_size_from_request(request_box)
            if box_size is not None:
                return box_size, "request"
        if self._box_source_policy != "request_only":
            box_size = self._box_size_from_param()
            if box_size is not None and min(box_size) > 0.0:
                return box_size, "param"
        return None, self._box_source_policy

    def _compute_aperture_bounds(self):
        """Lateral (container-local Y) span the box can pass through.

        The opening is narrower than the interior and laterally offset, so a
        placement outside the aperture shadow cannot be inserted at all. This
        replaces the hand-copied near_roi y constants with the aperture
        geometry they were transcribed from.
        """
        try:
            corners = container_opening_aperture_corners_in_container(
                self._scene_config, self._aperture_margin)
        except (KeyError, TypeError, ValueError):
            corners = None
        if not corners:
            return None
        ys = [float(corner[1]) for corner in corners]
        return min(ys), max(ys)

    def _load_smallest_box_size(self):
        """Smallest catalog box, used as the insertion-corridor probe size."""
        configured = rospy.get_param("~smallest_box_size", None)
        if configured:
            return [float(v) for v in configured]
        try:
            from box_catalog_utils import (
                box_catalog_path_from_scene,
                box_size_range,
                load_box_catalog,
            )
            catalog = load_box_catalog(
                box_catalog_path_from_scene(self._scene_config))
            # Lower bound of the sampling envelope, not the smallest catalog
            # entry: with continuous sizes the smallest box that may still
            # arrive is the envelope minimum, and the insertion corridor has to
            # stay passable for it.
            return [low for low, _high in box_size_range(catalog)]
        except Exception as exc:  # noqa: BLE001 - fall back to a safe default
            rospy.logwarn(
                "box catalog unavailable for corridor probe size (%s)", exc)
        return [0.55, 0.40, 0.25]

    def _load_surface_map(self):
        sm = rospy.get_param(self._surface_param, None)
        if not sm or "height" not in sm:
            return None
        return sm

    def _box_size_from_param(self):
        data = rospy.get_param("/luggage/current_box", {})
        if not data:
            return None
        return [
            float(data.get("width", 0.0)),
            float(data.get("depth", 0.0)),
            float(data.get("height", 0.0)),
        ]

    def _floor_prior_surface_map(self):
        """Synthetic surface_map_2d representing the bare a-priori floor.

        Used when no sensor heightmap is available yet (empty container / no
        integrated depth). All columns are ``state="unknown"`` with
        ``height=floor_z``; the solver's floor-prior gate (§4.2.2) allows
        these at ``peak≈floor_z`` and marks them
        ``support_source=floor_prior``.
        Fields match ``cargo_volume_mapper.surface_map_2d()`` so downstream
        consumers (markers, motion filter) are unchanged.
        """
        if self._floor_prior_surface is None:
            inner_l, inner_w, inner_h = container_inner_dimensions(self._scene_config)
            floor_z = container_inner_floor_z(self._scene_config)
            base_xyz, base_rpy = container_in_base_link(self._scene_config)
            res = self._floor_prior_resolution
            nx = max(1, int(round(inner_l / res)))
            ny = max(1, int(round(inner_w / res)))
            self._floor_prior_surface = {
                "resolution": res,
                "nx": nx,
                "ny": ny,
                "inner_size": [inner_l, inner_w, inner_h],
                "floor_z": floor_z,
                "center_base": list(
                    container_interior_center_in_base_link(self._scene_config)
                ),
                "yaw": float(base_rpy[2]),
                "height": [[floor_z] * ny for _ in range(nx)],
                "state": [["unknown"] * ny for _ in range(nx)],
                "confidence": [["none"] * ny for _ in range(nx)],
            }
        return self._floor_prior_surface

    def _atlas_for_box(self, box_size):
        """Payload-matched atlas for ``box_size``; ``(None, 0.0)`` when absent.

        Returns ``(atlas, floor_z)`` where ``floor_z`` is the atlas's own
        container floor elevation, so the contact-frame conversion always uses
        the geometry the grid was sampled with.
        """
        if not self._use_atlas or ReachabilityAtlas is None:
            return None, 0.0
        key = tuple(round(float(v), 3) for v in box_size)
        if key in self._atlas_cache:
            return self._atlas_cache[key]
        paths = resolve_atlas_path(
            self._atlas_dir, self._atlas_basename, box_size)
        entry = (None, 0.0)
        if paths is None:
            rospy.logwarn_throttle(
                60.0,
                "no reachability atlas under %s (basename=%s); "
                "placement runs without an atlas prior",
                self._atlas_dir, self._atlas_basename)
        else:
            npz_path, meta_path = paths
            try:
                atlas = ReachabilityAtlas.load(npz_path, meta_path)
                floor_z = float(
                    atlas.meta.get("container", {}).get("floor_z", 0.0))
                entry = (atlas, floor_z)
                rospy.loginfo(
                    "placement atlas loaded: %s (floor_z=%.3f)",
                    os.path.basename(npz_path), floor_z)
            except Exception as exc:  # noqa: BLE001 - degrade, never block
                rospy.logwarn(
                    "reachability atlas %s unusable (%s); "
                    "placement runs without an atlas prior",
                    os.path.basename(npz_path), exc)
        self._atlas_cache[key] = entry
        return entry

    @staticmethod
    def _base_to_map_local(center, map_center, yaw):
        dx = center[0] - map_center[0]
        dy = center[1] - map_center[1]
        return [
            math.cos(-yaw) * dx - math.sin(-yaw) * dy,
            math.sin(-yaw) * dx + math.cos(-yaw) * dy,
            center[2] - map_center[2],
        ]

    @staticmethod
    def _usable_inner_size(surface):
        """Floor-relative usable extents (EMS / proxy_score coordinate frame)."""
        inner = [float(v) for v in surface["inner_size"]]
        return [inner[0], inner[1], inner[2] - float(surface.get("floor_z", 0.0))]

    @staticmethod
    def _candidate_floor_aabb(candidate):
        """Candidate AABB in floor-relative coordinates, for EMS bookkeeping."""
        lx, ly = candidate["center_local"][0], candidate["center_local"][1]
        box_l, box_w, box_h = candidate["size"]
        peak = float(candidate.get("peak", 0.0))
        return (lx - box_l * 0.5, ly - box_w * 0.5, peak,
                lx + box_l * 0.5, ly + box_w * 0.5, peak + box_h)

    def _fsm_candidates(self, surface, box_size):
        usable_inner = self._usable_inner_size(surface)
        geometry = GeometryProfile.from_scene_config(
            self._scene_config, usable_inner,
            floor_z=float(surface.get("floor_z", 0.0)))
        model = FreeSpaceModel(
            geometry,
            surface["center_base"],
            yaw=float(surface["yaw"]),
            resolution=float(surface["resolution"]),
            clearance_margin=self._params["clearance_margin"],
            boundary_margin=self._params["boundary_margin"],
            floor_prior=True,
        )
        ems = EMS(geometry, min_useful_edge=min(self._smallest_box_size))
        for item in rospy.get_param(
                "/luggage/container_inspection/placed_boxes", []):
            position = item.get("place_pose", {}).get("position", {})
            center = [
                float(position.get("x", 0.0)),
                float(position.get("y", 0.0)),
                float(position.get("z", 0.0)),
            ]
            local = self._base_to_map_local(
                center, surface["center_base"], float(surface["yaw"]))
            size = [
                float(item.get("width", 0.0)),
                float(item.get("depth", 0.0)),
                float(item.get("height", 0.0)),
            ]
            if min(size) > 0.0:
                orientation = item.get(
                    "place_pose", {}).get("orientation", {})
                qz = float(orientation.get("z", 0.0))
                qw = float(orientation.get("w", 1.0))
                world_yaw = 2.0 * math.atan2(qz, qw)
                relative_yaw = (
                    world_yaw - float(surface["yaw"])) % math.pi
                if abs(relative_yaw - math.pi * 0.5) < math.pi * 0.25:
                    size[0], size[1] = size[1], size[0]
                placed_box = model.add_placed_box(local, size)
                ems.place((
                    placed_box["x0"], placed_box["y0"], placed_box["z0"],
                    placed_box["x1"], placed_box["y1"], placed_box["z1"]))
        # Merge sensor observations after exact committed geometry. Geometry
        # confidence must not quantize a known box top upward by one voxel.
        model.merge_surface_2d(surface)
        candidates = model.candidates(
            box_size,
            allowed_yaws=self._allowed_yaws,
            top_n=max(int(self._params["top_n"]) * 8, 50),
        )
        candidates = self._apply_aperture_gate(candidates)
        candidates = self._apply_near_roi(candidates)
        atlas, atlas_floor_z = self._atlas_for_box(box_size)
        candidates, atlas_rejected = annotate_with_atlas(
            candidates, atlas, atlas_floor_z)
        score_candidates(
            candidates, model, ems, geometry, self._smallest_box_size,
            opening_side=self._opening_side,
            w_floor_first=self._w_floor_first)
        return candidates, atlas_rejected

    def _apply_aperture_gate(self, candidates):
        """Drop placements whose footprint lies outside the opening shadow."""
        if not self._aperture_gate_enabled or self._aperture_bounds is None:
            return candidates
        y_min, y_max = self._aperture_bounds
        kept = []
        for cand in candidates:
            cy = cand["center_local"][1]
            footprint = cand.get("footprint", cand["size"][:2])
            half_y = footprint[1] * 0.5
            if cy - half_y >= y_min - 1e-9 and cy + half_y <= y_max + 1e-9:
                kept.append(cand)
            else:
                cand["feasible"] = False
                cand["reason"] = "outside_aperture"
        return kept

    def _apply_near_roi(self, candidates):
        if not self._near_roi_enabled:
            return candidates
        kept = []
        for cand in candidates:
            cx, cy = cand["center_local"][:2]
            footprint = cand.get("footprint", cand["size"][:2])
            hx, hy = footprint[0] * 0.5, footprint[1] * 0.5
            if (
                    cx - hx >= self._near_roi_x_min
                    and cx + hx <= self._near_roi_x_max
                    and cy - hy >= self._near_roi_y_min
                    and cy + hy <= self._near_roi_y_max):
                kept.append(cand)
            else:
                cand["feasible"] = False
                cand["reason"] = "outside_near_roi"
        return kept

    @staticmethod
    def _annotate_candidate_session(candidates, surface):
        revision = int(surface.get("map_revision", -1))
        placed_count = len(rospy.get_param(
            "/luggage/container_inspection/placed_boxes", []))
        for index, candidate in enumerate(candidates):
            candidate["candidate_id"] = "m%d_p%d_c%d" % (
                revision, placed_count, index)
            candidate["map_revision"] = revision
            candidate["placed_count"] = placed_count
        return revision, placed_count

    def _compute(self, box_size):
        surface = self._load_surface_map()
        source = "surface_2d"
        if surface is None:
            # Floor-prior fallback (§4.2.2): no heightmap yet. The floor's
            # existence is geometric prior, so generate candidates on the bare
            # floor instead of returning None.
            surface = self._floor_prior_surface_map()
            source = "floor_prior"
        if not box_size or min(box_size) <= 0.0:
            return None, "current box dimensions missing"
        atlas_rejected = 0
        if self._use_free_space_model:
            # The FSM path applies the aperture/ROI gates itself so the atlas
            # prior only has to be evaluated for geometrically valid poses.
            candidates, atlas_rejected = self._fsm_candidates(
                surface, box_size)
        else:
            candidates = generate_candidates(
                surface, box_size, allowed_yaws=self._allowed_yaws,
                params=self._params)
            candidates = self._apply_near_roi(candidates)
        revision, placed_count = self._annotate_candidate_session(
            candidates, surface)
        candidates = candidates[:int(self._params["top_n"])]
        rospy.set_param("/luggage/placement/candidates", candidates)
        best = best_candidate(candidates)
        rospy.set_param(
            "/luggage/placement/best", best if best is not None else {}
        )
        self._publish_markers(candidates)
        feasible = sum(1 for c in candidates if c["feasible"])
        floor_level = sum(
            1 for c in candidates
            if c.get("feasible") and float(c.get("peak", 0.0)) <= 1e-3)
        return candidates, (
            "%d candidates (%d feasible, %d on floor, source=%s, "
            "map_revision=%d, placed=%d, atlas_rejected=%d)" % (
                len(candidates), feasible, floor_level, source, revision,
                placed_count, atlas_rejected))

    def _publish_markers(self, candidates):
        stamp = rospy.Time.now()
        self._markers_pub.publish(
            build_candidate_markers(candidates, self._base_frame, stamp)
        )

    def _on_timer(self, _event):
        box_size, _source = self._select_box_size()
        if box_size is None:
            return
        self._compute(box_size)

    def handle_plan(self, _req):
        box_size, source = self._select_box_size()
        if box_size is None:
            return TriggerResponse(
                success=False,
                message="box dimensions unavailable (box_source_policy=%s)"
                % source)
        candidates, message = self._compute(box_size)
        return TriggerResponse(success=candidates is not None, message=message)

    @staticmethod
    def _yaw_quaternion(yaw):
        return Quaternion(z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))

    def _candidate_to_slot(self, cand, index):
        center = cand["center_base"]
        local = cand["center_local"]
        layer = int(round((local[2]) * 1000.0))
        return SlotSpec(
            layer=layer,
            row=int(round((local[1] + 10.0) * 1000.0)),
            col=int(round(local[0] * 1000.0)) + index,
            width=cand["size"][0],
            depth=cand["size"][1],
            height=cand["size"][2],
            place_pose=Pose(
                position=Point(x=center[0], y=center[1], z=center[2]),
                orientation=self._yaw_quaternion(cand["yaw"]),
            ),
        )

    def handle_compute(self, req):
        box_size, source = self._select_box_size(req.box)
        if box_size is None:
            return ComputePlacementResponse(
                slot=SlotSpec(),
                success=False,
                message="box dimensions unavailable (box_source_policy=%s)"
                % source)
        candidates, message = self._compute(box_size)
        if candidates is None:
            return ComputePlacementResponse(slot=SlotSpec(), success=False, message=message)
        best = best_candidate(candidates)
        if best is None:
            return ComputePlacementResponse(
                slot=SlotSpec(),
                success=False,
                message="no feasible placement (%s)" % message,
            )
        slot = self._candidate_to_slot(best, 0)
        return ComputePlacementResponse(
            slot=slot,
            success=True,
            message="placement score=%.2f clearance=%.2f (%s)"
            % (best["score"], best["clearance_top"], message),
        )



# Log level must be chosen before init_node, so it cannot come from a private
# param; log_level_utils reads the LUGGAGE_LOG_LEVEL environment variable.
import os as _os
import sys as _sys
import rospkg as _rospkg
_DESC = _os.path.join(
    _rospkg.RosPack().get_path("luggage_description"), "scripts")
if _DESC not in _sys.path:
    _sys.path.insert(0, _DESC)
from log_level_utils import resolve_log_level  # noqa: E402

def main():
    rospy.init_node("placement_planner", log_level=resolve_log_level())
    PlacementPlannerNode()
    rospy.loginfo("placement_planner ready")
    rospy.spin()


if __name__ == "__main__":
    main()
