#!/usr/bin/env python3
"""ComputePlacement service shell (ROS 2 Humble port, Todo 5 slice C).

Thin node around the pure ``placement_solver.generate_candidates`` plus the
aperture gate (G2) and corridor check from the corridor constraint doc.
No TF lookups inside the solver path; container-local math only.

Subscribes ``/luggage/cargo_map/surface_2d`` (JSON from
cargo_volume_mapper_node) with a floor-prior fallback when no map has been
published yet (empty container): the floor's existence is geometric prior.

Service ``/placement_planner/compute_placement`` (luggage_msgs/ComputePlacement):
  request  box (DetectedLuggage), placed (SlotSpec[] in elfin_base_link)
  response slot (SlotSpec in elfin_base_link), success, message
           message carries the reject histogram when no candidate survives
           (BIN_FULL semantics), e.g. "BIN_FULL no_candidate: overlap=10
           outside_aperture=6 corridor_blocked=2"

Latched ``/placement_planner/last_result`` (std_msgs/String JSON) lists every
candidate (feasible + rejected) for pack-eval dumps.

G2 aperture gate: a candidate footprint outside the opening-aperture Y
shadow gets ``reason=outside_aperture``. The 7-face hull gate rejects boxes
whose AABB corners leave the chamfered inner volume (``outside_hull``).
Corridor check uses floor-relative container AABBs
(insertion_corridor.corridor_blocked, single-box wall).
"""

from __future__ import division

import json
import math

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from geometry_msgs.msg import Pose as PoseMsg
from geometry_msgs.msg import Quaternion
from luggage_msgs.msg import SlotSpec
from luggage_msgs.srv import ComputePlacement

from luggage_description.box_catalog_utils import (
    box_catalog_path_from_scene,
    box_size_range,
    load_box_catalog,
)
from luggage_description.scene_tf_config_utils import (
    _local_point_to_base_link,
    _point_in_container_link,
    container_inner_ceiling_z,
    container_inner_dimensions,
    container_inner_floor_z,
    container_opening_aperture_corners_in_container,
    load_scene_tf_config,
    origin_in_world,
    point_inside_container_inner_hull_container,
    resolve_scene_tf_config_path,
    yaw_base_link_to_world,
    yaw_world_to_base_link,
)
from luggage_packing.insertion_corridor import corridor_blocked
from luggage_packing.placement_solver import generate_candidates

# Reject reasons surfaced in the response message (A5 histogram inputs).
REASON_OVERLAP = "overlap"
REASON_OUTSIDE_APERTURE = "outside_aperture"
REASON_OUTSIDE_HULL = "outside_hull"
REASON_CORRIDOR_BLOCKED = "corridor_blocked"


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def jsonable_candidate(candidate):
    """Round numeric fields so last_result dumps stay small and stable."""
    out = {}
    for key, value in candidate.items():
        if key in ("center_base", "center_local", "center_world",
                   "center_base_link", "footprint", "size"):
            out[key] = [round(float(v), 4) for v in value]
        elif isinstance(value, float):
            out[key] = round(value, 4)
        else:
            out[key] = value
    return out


class PlacementPlannerNode(Node):

    def __init__(self):
        super().__init__("placement_planner")
        group = ReentrantCallbackGroup()

        self.declare_parameter("scene_tf_config", "")
        self.declare_parameter("allowed_yaws", [0.0, 1.5707963, 3.14159265,
                                                -1.5707963])
        self.declare_parameter("clearance_margin", 0.03)
        self.declare_parameter("min_support_ratio", 0.6)
        self.declare_parameter("top_n", 2000)
        self.declare_parameter("aperture_margin", 0.0)
        self.declare_parameter("floor_prior_resolution", 0.05)

        config_path = str(self.get_parameter("scene_tf_config").value)
        if not config_path:
            config_path = resolve_scene_tf_config_path()
        self._scene = load_scene_tf_config(config_path)

        self._allowed_yaws = [
            float(v) for v in self.get_parameter("allowed_yaws").value]
        self._params = {
            "clearance_margin": float(
                self.get_parameter("clearance_margin").value),
            "min_support_ratio": float(
                self.get_parameter("min_support_ratio").value),
            "top_n": int(self.get_parameter("top_n").value),
            # Enough of the sliding window that corridor/aperture gates
            # see deep slots, not only the 8 highest scores.
            "keep_rejected": 200,
        }
        self._aperture_margin = float(
            self.get_parameter("aperture_margin").value)

        self._inner_size = self._inner_dimensions()
        self._floor_z = container_inner_floor_z(self._scene)
        self._aperture_y = self._aperture_bounds()
        self._smallest_box = self._smallest_box_size()
        self._surface = None

        map_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, "/luggage/cargo_map/surface_2d",
            self._on_surface, map_qos, callback_group=group)
        self._last_pub = self.create_publisher(
            String, "/placement_planner/last_result", map_qos)

        self.create_service(
            ComputePlacement, "/placement_planner/compute_placement",
            self._handle, callback_group=group)

        self.get_logger().info(
            "placement_planner ready (aperture_y=%s, smallest=%s)"
            % (self._aperture_y, self._smallest_box))

    # ------------------------------------------------------------------
    # Scene-derived bounds

    def _inner_dimensions(self):
        inner = container_inner_dimensions(self._scene)
        floor_z = container_inner_floor_z(self._scene)
        ceiling_z = container_inner_ceiling_z(self._scene)
        return [float(inner[0]), float(inner[1]), float(ceiling_z - floor_z)]

    def _aperture_bounds(self):
        """Lateral (container-local Y) span the box can pass through."""
        try:
            corners = container_opening_aperture_corners_in_container(
                self._scene, self._aperture_margin)
        except (KeyError, TypeError, ValueError):
            return None
        if not corners:
            return None
        ys = [float(corner[1]) for corner in corners]
        return min(ys), max(ys)

    def _smallest_box_size(self):
        try:
            catalog = load_box_catalog(
                box_catalog_path_from_scene(self._scene))
            return [low for low, _high in box_size_range(catalog)]
        except Exception as exc:  # noqa: BLE001 - safe default below
            self.get_logger().warning(
                "box catalog unavailable for corridor probe (%s)" % exc)
        return [0.55, 0.40, 0.25]

    def _container_to_world(self, xyz):
        origin, rpy = origin_in_world(self._scene)
        yaw = float(rpy[2])
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        x, y, z = [float(v) for v in xyz]
        return [
            origin[0] + cos_y * x - sin_y * y,
            origin[1] + sin_y * x + cos_y * y,
            origin[2] + z,
        ]

    def _annotate_frames(self, candidate):
        """Add world / base_link centers. center_base is container_link."""
        container_xyz = candidate.get("center_base") or candidate.get(
            "center_local") or [0.0, 0.0, 0.0]
        world_xyz = self._container_to_world(container_xyz)
        base_xyz = _local_point_to_base_link(container_xyz, self._scene)
        world_yaw = float(candidate.get("yaw", 0.0))
        candidate["center_world"] = world_xyz
        candidate["center_base_link"] = base_xyz
        candidate["yaw_world"] = world_yaw
        candidate["yaw_base_link"] = yaw_world_to_base_link(
            self._scene, world_yaw)
        return candidate

    def _slot_from_candidate(self, candidate, size):
        """SlotSpec pose is elfin_base_link (waypoint_generator default)."""
        annotated = self._annotate_frames(dict(candidate))
        slot = SlotSpec()
        slot.layer = 0
        slot.row = 0
        slot.col = 0
        base = annotated["center_base_link"]
        slot.place_pose = PoseMsg()
        slot.place_pose.position.x = float(base[0])
        slot.place_pose.position.y = float(base[1])
        slot.place_pose.position.z = float(base[2])
        yaw = float(annotated["yaw_base_link"])
        slot.place_pose.orientation = Quaternion(
            x=0.0, y=0.0, z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))
        slot.width = float(size[0])
        slot.depth = float(size[1])
        slot.height = float(size[2])
        return slot, annotated

    # ------------------------------------------------------------------
    # Surface map in / floor prior

    def _on_surface(self, msg):
        try:
            surface = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(surface, dict) and "height" in surface:
            self._surface = surface

    def _floor_prior_surface(self):
        """Match cargo_volume_mapper.surface_map_2d: floor-relative height.

        height=0, center_base = usable-volume center in container_link.
        Empty-container floor slot z = floor_z + box_h/2.
        """
        res = float(self.get_parameter("floor_prior_resolution").value)
        inner_l, inner_w, inner_h = self._inner_size
        nx = max(1, int(round(inner_l / res)))
        ny = max(1, int(round(inner_w / res)))
        return {
            "resolution": res,
            "nx": nx, "ny": ny,
            "inner_size": [inner_l, inner_w, inner_h],
            "floor_z": 0.0,
            "center_base": [0.0, 0.0, self._floor_z + 0.5 * inner_h],
            "yaw": 0.0,
            "height": [[0.0] * ny for _ in range(nx)],
            "state": [["unknown"] * ny for _ in range(nx)],
            "clearance": [[inner_h] * ny for _ in range(nx)],
            "known_ratio": [[0.0] * ny for _ in range(nx)],
            "confidence": [["none"] * ny for _ in range(nx)],
        }

    def _active_surface(self):
        return self._surface or self._floor_prior_surface()

    # ------------------------------------------------------------------
    # Feasibility gates (G2 + corridor)

    def _footprint_y_span(self, candidate):
        footprint = candidate.get("footprint") or [0.0, 0.0]
        local = candidate.get("center_local") or [0.0, 0.0, 0.0]
        half_w = float(footprint[1]) * 0.5
        return float(local[1]) - half_w, float(local[1]) + half_w

    def _aperture_reject(self, candidate):
        """True when the footprint cannot pass through the opening at all."""
        if self._aperture_y is None:
            return False
        lo, hi = self._footprint_y_span(candidate)
        a_lo, a_hi = self._aperture_y
        return lo < a_lo - 1e-6 or hi > a_hi + 1e-6

    def _hull_reject(self, candidate):
        """True when any AABB corner sits in the cut-off +Y triangle."""
        x0, y0, z0, x1, y1, z1 = self._candidate_aabb(candidate)
        floor_z = self._floor_z
        for x in (x0, x1):
            for y in (y0, y1):
                for z in (z0, z1):
                    if not point_inside_container_inner_hull_container(
                            [x, y, z + floor_z], self._scene):
                        return True
        return False

    def _placed_aabbs(self, placed_slots):
        """Floor-relative container AABBs from elfin_base_link SlotSpecs."""
        aabbs = []
        inner_h = self._inner_size[2]
        for slot in placed_slots or []:
            pose = slot.place_pose
            local = _point_in_container_link(
                [pose.position.x, pose.position.y, pose.position.z],
                self._scene)
            yaw_world = yaw_base_link_to_world(
                self._scene, yaw_from_quaternion(pose.orientation))
            w, d = float(slot.width), float(slot.depth)
            rotated = abs(abs(yaw_world) - math.pi / 2.0) < math.radians(10.0)
            footprint_l = d if rotated else w
            footprint_w = w if rotated else d
            h = float(slot.height)
            z_floor = local[2] - self._floor_z
            aabbs.append((
                local[0] - footprint_l * 0.5,
                local[1] - footprint_w * 0.5,
                max(0.0, z_floor - h * 0.5),
                local[0] + footprint_l * 0.5,
                local[1] + footprint_w * 0.5,
                min(inner_h, z_floor + h * 0.5),
            ))
        return aabbs

    def _candidate_aabb(self, candidate):
        """Floor-relative AABB matching insertion_corridor.corridor_blocked."""
        local = candidate.get("center_local") or [0.0, 0.0, 0.0]
        footprint = candidate.get("footprint") or candidate["size"][:2]
        h = float(candidate["size"][2])
        half_h = self._inner_size[2] * 0.5
        z_floor_c = float(local[2]) + half_h
        fl, fw = float(footprint[0]), float(footprint[1])
        return (
            float(local[0]) - fl * 0.5,
            float(local[1]) - fw * 0.5,
            z_floor_c - h * 0.5,
            float(local[0]) + fl * 0.5,
            float(local[1]) + fw * 0.5,
            z_floor_c + h * 0.5,
        )

    def _corridor_reject(self, candidate, placed_slots):
        """corridor_blocked: the opening corridor to this candidate is walled."""
        inner_l, inner_w, inner_h = self._inner_size
        return corridor_blocked(
            self._candidate_aabb(candidate),
            self._placed_aabbs(placed_slots),
            [inner_l, inner_w, inner_h], list(self._smallest_box))

    def _publish_last(self, payload):
        pub = dict(payload)
        cands = list(pub.get("candidates") or [])
        if len(cands) > 24:
            pub["candidates"] = cands[:24]
            pub["candidates_truncated"] = True
        self._last_pub.publish(String(data=json.dumps(
            pub, sort_keys=True, default=str)))

    # ------------------------------------------------------------------
    # Service

    def _handle(self, request, response):
        box = request.box
        size = [max(0.0, float(box.width)),
                max(0.0, float(box.depth)),
                max(0.0, float(box.height))]
        surface = self._active_surface()
        candidates = generate_candidates(
            surface, size,
            allowed_yaws=self._allowed_yaws,
            params=self._params)

        histogram = {}
        feasible = []
        for candidate in candidates:
            if not candidate.get("feasible", False):
                reason = candidate.get("reason", REASON_OVERLAP)
                histogram[reason] = histogram.get(reason, 0) + 1
                continue
            if self._aperture_reject(candidate):
                candidate["feasible"] = False
                candidate["reason"] = REASON_OUTSIDE_APERTURE
                histogram[REASON_OUTSIDE_APERTURE] = \
                    histogram.get(REASON_OUTSIDE_APERTURE, 0) + 1
                continue
            if self._hull_reject(candidate):
                candidate["feasible"] = False
                candidate["reason"] = REASON_OUTSIDE_HULL
                histogram[REASON_OUTSIDE_HULL] = \
                    histogram.get(REASON_OUTSIDE_HULL, 0) + 1
                continue
            if self._corridor_reject(candidate, request.placed):
                candidate["feasible"] = False
                candidate["reason"] = REASON_CORRIDOR_BLOCKED
                histogram[REASON_CORRIDOR_BLOCKED] = \
                    histogram.get(REASON_CORRIDOR_BLOCKED, 0) + 1
                continue
            feasible.append(candidate)

        dump = {
            "success": bool(feasible),
            "size_wdh": size,
            "n_candidates_total": len(candidates),
            "n_feasible": len(feasible),
            "reject_histogram": histogram,
            "map_revision": surface.get("map_revision"),
            "floor_prior": self._surface is None,
            "candidates": [jsonable_candidate(self._annotate_frames(dict(c)))
                           for c in candidates],
        }

        if not feasible:
            response.slot = SlotSpec()
            response.success = False
            parts = " ".join("%s=%d" % (k, v)
                             for k, v in sorted(histogram.items()))
            response.message = "BIN_FULL no_candidate%s%s" % (
                ": " if parts else "", parts)
            dump["message"] = response.message
            self._publish_last(dump)
            return response

        best = max(feasible, key=lambda c: c.get("score", 0.0))
        slot, annotated = self._slot_from_candidate(best, size)
        response.slot = slot
        response.success = True
        response.message = "slot score=%.3f feasible=%d rejected=%s" % (
            best.get("score", 0.0), len(feasible),
            json.dumps(histogram, sort_keys=True) or "{}")
        dump["message"] = response.message
        dump["selected"] = jsonable_candidate(annotated)
        dump["pose_base_link"] = annotated["center_base_link"]
        dump["pose_world"] = annotated["center_world"]
        self._publish_last(dump)
        return response


def main(argv=None):
    rclpy.init(args=argv)
    node = PlacementPlannerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
