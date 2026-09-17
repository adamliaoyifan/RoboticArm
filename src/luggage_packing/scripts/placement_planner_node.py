#!/usr/bin/env python3
"""ComputePlacement service shell (ROS 2 Humble port, Todo 5 slice C).

Thin node around the pure ``placement_solver.generate_candidates`` plus the
aperture gate (G2) and corridor check from the corridor constraint doc.
No TF lookups inside the solver path; container-local math only.

Subscribes ``/luggage/cargo_map/surface_2d`` (JSON from
cargo_volume_mapper_node) with a floor-prior fallback when no map has been
published yet (empty container): the floor's existence is geometric prior.
A map whose ``geometry_hash`` is missing or differs from this node's kernel
hull is rejected outright; the planner never reads a foreign map's
``inner_size`` (docs/architecture/container_geometry.md).

Service ``/placement_planner/compute_placement`` (luggage_msgs/ComputePlacement):
  request  box (DetectedLuggage), placed (SlotSpec[] in elfin_base_link),
           geometry_hash (optional pin on the container hull)
  response slot (SlotSpec in elfin_base_link), success, message,
           geometry_hash (hull the answer was computed against), reason_code
           message carries the reject histogram when no candidate survives,
           e.g. "PLACE_CANDIDATE_EXHAUSTED no_candidate: overlap=10
           outside_aperture=6 corridor_blocked=2". Only reason_code=BIN_FULL
           claims the container is physically full.

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
    resolve_scene_tf_config_path,
    yaw_base_link_to_world,
    yaw_world_to_base_link,
)
from luggage_packing.placement_solver import (
    CODE_CARGO_MAP_GEOMETRY_MISMATCH,
    CODE_DETECT_FULL_GEOMETRY_REQUIRED,
    placement_constraint_verdict,
    solve_placement,
)
from luggage_description.container_geometry import (
    descriptor_from_scene_config,
    y_max_at_z,
)


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
        # Inward clearance kept between the candidate box and the seven-face
        # hull. Default 0 preserves the historical flush-wall behavior; eval
        # profiles configure 0.01 (plan acceptance: >=10 mm).
        self.declare_parameter("hull_margin", 0.0)
        self.declare_parameter("floor_prior_resolution", 0.05)
        # Cap on candidates serialized into the latched last_result dump
        # (eval drivers retry over the retained list; 24 is the historical
        # visualization-sized default).
        self.declare_parameter("last_result_max_candidates", 24)

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
        self._hull = descriptor_from_scene_config(self._scene)
        self._geometry_hash = str(self._hull.geometry_hash)
        self._rejected_maps = 0

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
            "placement_planner ready (aperture_y=%s, smallest=%s, "
            "geometry_hash=%s)"
            % (self._aperture_y, self._smallest_box, self._geometry_hash))

    def _hull_geometry(self):
        """Authoritative kernel hull (cached; config is immutable here)."""
        return self._hull

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
        if not isinstance(surface, dict) or "height" not in surface:
            return
        incoming = str(surface.get("geometry_hash") or "")
        if incoming != self._geometry_hash:
            # Fail closed: a map built for another hull must never be read
            # for its inner_size or heights.
            self._rejected_maps += 1
            self.get_logger().error(
                "rejected cargo map: geometry_hash %s != planner %s "
                "(rejected=%d)"
                % (incoming or "<missing>", self._geometry_hash,
                   self._rejected_maps))
            return
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
            "geometry_hash": self._geometry_hash,
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

    def _constraint_verdict(self, candidate, placed_aabbs):
        """Return ``(reason, capacity_ok)`` for one candidate."""
        hull_margin = float(self.get_parameter("hull_margin").value)
        hull = self._hull_geometry()

        def hull_contains_floor_relative(point):
            # Lateral clearance only: a floor-resting box touches the floor
            # by design, so the margin applies to the x walls, the -y wall,
            # the chamfer plane, and the ceiling — never the floor.
            x, y, z_rel = point
            z = z_rel + self._floor_z
            if not (float(hull.floor_z) - 1e-9 <= z
                    <= float(hull.ceiling_z) - hull_margin):
                return False
            if not (-hull.half_x + hull_margin <= x <= hull.half_x - hull_margin):
                return False
            if y < -hull.half_y + hull_margin:
                return False
            return y <= y_max_at_z(hull, z, margin=hull_margin) + 1e-9

        return placement_constraint_verdict(
            candidate,
            self._inner_size[2],
            placed_aabbs=placed_aabbs,
            aperture_y=self._aperture_y,
            hull_contains=hull_contains_floor_relative,
            inner_size=self._inner_size,
            smallest_size=self._smallest_box,
        )

    def _publish_last(self, payload):
        pub = dict(payload)
        cands = list(pub.get("candidates") or [])
        cap = int(self.get_parameter("last_result_max_candidates").value)
        if len(cands) > cap:
            pub["candidates"] = cands[:cap]
            pub["candidates_truncated"] = True
        self._last_pub.publish(String(data=json.dumps(
            pub, sort_keys=True, default=str)))

    # ------------------------------------------------------------------
    # Service

    def _handle(self, request, response):
        box = request.box
        response.geometry_hash = self._geometry_hash
        # E0/E4 contract: packing needs measured full geometry. A top-only
        # detection (or a catalog prior with height_valid=false) must not
        # become a collision box inside the container.
        if not bool(getattr(box, "height_valid", False)):
            response.success = False
            response.reason_code = CODE_DETECT_FULL_GEOMETRY_REQUIRED
            response.message = (
                "DETECT_FULL_GEOMETRY_REQUIRED: ComputePlacement needs "
                "height_valid=true (measured support or configured mode); "
                "got height_source=%d" % int(getattr(box, "height_source", 0)))
            return response
        requested_hash = str(getattr(request, "geometry_hash", "") or "")
        if requested_hash and requested_hash != self._geometry_hash:
            response.success = False
            response.reason_code = CODE_CARGO_MAP_GEOMETRY_MISMATCH
            response.message = (
                "%s: request pinned %s, planner hull is %s"
                % (CODE_CARGO_MAP_GEOMETRY_MISMATCH, requested_hash,
                   self._geometry_hash))
            return response
        if self._surface is None and self._rejected_maps:
            # A live mapper published a hull we refused. Answering from the
            # floor prior here would be the silent fallback GEO-8 forbids.
            response.success = False
            response.reason_code = CODE_CARGO_MAP_GEOMETRY_MISMATCH
            response.message = (
                "%s: %d cargo map(s) rejected, planner hull is %s"
                % (CODE_CARGO_MAP_GEOMETRY_MISMATCH, self._rejected_maps,
                   self._geometry_hash))
            self._publish_last({
                "success": False,
                "size_wdh": [max(0.0, float(box.width)),
                             max(0.0, float(box.depth)),
                             max(0.0, float(box.height))],
                "n_candidates_total": 0,
                "n_feasible": 0,
                "reject_histogram": {},
                "reason_code": response.reason_code,
                "geometry_hash": self._geometry_hash,
                "rejected_maps": self._rejected_maps,
                "floor_prior": False,
                "candidates": [],
                "message": response.message,
            })
            return response
        size = [max(0.0, float(box.width)),
                max(0.0, float(box.depth)),
                max(0.0, float(box.height))]
        surface = self._active_surface()
        placed_aabbs = self._placed_aabbs(request.placed)
        result = solve_placement(
            surface, size,
            allowed_yaws=self._allowed_yaws,
            params=self._params,
            candidate_validator=lambda candidate: self._constraint_verdict(
                candidate, placed_aabbs),
        )
        candidates = result["candidates"]
        histogram = result["reject_histogram"]
        feasible = [candidate for candidate in candidates
                    if candidate.get("feasible", False)]

        dump = {
            "success": bool(feasible),
            "size_wdh": size,
            "n_candidates_total": len(candidates),
            "n_feasible": len(feasible),
            "n_capacity_feasible": result["capacity_feasible_count"],
            "reject_histogram": histogram,
            "reason_code": result["reason_code"],
            "geometry_hash": self._geometry_hash,
            "map_geometry_hash": str(surface.get("geometry_hash") or ""),
            "rejected_maps": self._rejected_maps,
            "map_revision": surface.get("map_revision"),
            "floor_prior": self._surface is None,
            "candidates": [jsonable_candidate(self._annotate_frames(dict(c)))
                           for c in candidates],
        }

        if not result["success"]:
            response.slot = SlotSpec()
            response.success = False
            response.reason_code = result["reason_code"]
            response.message = result["message"]
            dump["message"] = response.message
            self._publish_last(dump)
            return response

        best = result["selected"]
        slot, annotated = self._slot_from_candidate(best, size)
        response.slot = slot
        response.success = True
        response.reason_code = ""
        response.message = result["message"]
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
