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
           geometry_hash (optional pin on the container hull),
           max_candidates (0 = node default)
  response slot (SlotSpec in elfin_base_link), success, message,
           geometry_hash (hull the answer was computed against), reason_code,
           candidates / candidate_scores / candidate_reasons (feasible
           prefix; candidates[0] matches slot on success).
           message carries the reject histogram when no candidate survives,
           e.g. "PLACE_CANDIDATE_EXHAUSTED no_candidate: overlap=10
           outside_aperture=6 corridor_blocked=2". Only reason_code=BIN_FULL
           claims the container is physically full.

Latched ``/placement_planner/last_result`` (std_msgs/String JSON) lists every
candidate (feasible + rejected) for pack-eval dumps.

G2 aperture gate: a candidate footprint outside the opening-aperture Y
shadow gets ``reason=outside_aperture``. The 7-face hull gate rejects boxes
whose AABB corners leave the chamfered inner volume (``outside_hull``).
Corridor check uses a hull-clipped insertion corridor
(insertion_corridor.corridor_blocked, single-box wall spanning hull Y).
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
from luggage_packing.placement_reachability import make_atlas_annotate
from luggage_packing.placement_admission import (
    BACKEND_GRID_G,
    BACKEND_OBSERVER_O,
    AdmissionGate,
    EpochFormatError,
    MapObservation,
    RobotStateSample,
    ValidityObservation,
    normalize_epoch)
from luggage_packing.placement_solver import (
    CODE_CARGO_MAP_GEOMETRY_MISMATCH,
    CODE_DETECT_FULL_GEOMETRY_REQUIRED,
    REASON_OVERLAP,
    REASON_SAME_LAYER_CLEARANCE,
    REASON_VOLUME_UNVERIFIED,
    SAME_LAYER_CLEARANCE_M,
    make_volume_verifier,
    placement_constraint_verdict,
    candidate_prefix_note,
    ranked_feasible_candidates,
    solve_placement,
    volume_evidence_from_pages,
)
from luggage_description.container_geometry import (
    descriptor_from_scene_config,
    y_max_at_z,
)
from luggage_description.frame_contracts import ContractError
from luggage_description.frame_diagnostics import BoundaryContext
from luggage_description.ros_frame_bindings.poses import (
    pose_from_message, pose_to_message,
)
from luggage_description.ros_frame_bindings.surfaces import (
    surface_from_message,
)
from luggage_msgs.msg import SurfaceContractV2
from luggage_msgs.srv import ComputePlacementV2

#: Correlation identity the cargo mapper stamps on every v2 surface
#: publication (one per map revision). Mirrors the mapper-side producer
#: constant; the cross-side regression test pins the two equal, and a
#: mismatch fails closed here.
SURFACE_V2_REQUEST_TEMPLATE = "map-rev-%d"

#: A ComputePlacementV2 request whose declared map revision does not match
#: the admitted surface is stale: it was planned against a different map.
CODE_STALE_MAP_REVISION = "STALE_MAP_REVISION"
#: [B2] The request-carried placed ledger disagrees with the mapper's
#: committed ledger (count or geometry). The driver's local copy diverged
#: from the single source — the witness_02 divergence class, now caught
#: at the planner boundary instead of planning against a phantom.
CODE_PLACED_LEDGER_MISMATCH = "PLACED_LEDGER_MISMATCH"
#: [A6, user ruling 2026-11-09] A box landed in the committed ledger
#: between this request's compute and its commit: the selected candidate
#: no longer clears the LATEST ledger, so the plan is not delivered.
#: The caller re-requests and re-plans against the moved world.
CODE_PLACED_LEDGER_MOVED = "PLACED_LEDGER_MOVED"


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

    def __init__(self, **node_options):
        super().__init__("placement_planner", **node_options)
        group = ReentrantCallbackGroup()

        self.declare_parameter("scene_tf_config", "")
        self.declare_parameter("allowed_yaws", [0.0, 1.5707963, 3.14159265,
                                                -1.5707963])
        self.declare_parameter("clearance_margin", 0.03)
        self.declare_parameter(
            "same_layer_clearance_m", SAME_LAYER_CLEARANCE_M)
        self.declare_parameter("min_support_ratio", 0.6)
        self.declare_parameter("top_n", 2000)
        self.declare_parameter("aperture_margin", 0.0)
        # Wall margin for placement legality. Default -1.0 = AUTO: one map
        # cell (the admitted surface's resolution) — the map's conservative
        # boundary column reaches up to one cell inside the exact wall
        # face, so an exact-face margin can still overlap map-occupied
        # ring cells (K01 run04/07 5 mm anchoring miss). An explicit
        # value (0.0 = exact face) overrides. Default changed 2026-11-08.
        self.declare_parameter("hull_margin", -1.0)
        from luggage_description.cargo_grid_parameters import declare_resolution
        declare_resolution(self, "floor_prior_resolution")
        # Cap on candidates serialized into the latched last_result dump
        # (eval drivers retry over the retained list; 24 is the historical
        # visualization-sized default).
        self.declare_parameter("last_result_max_candidates", 24)
        # G3 structural-first chain: off keeps the byte-identical
        # exhaustive path; on serves session batches (see
        # _structural_result) with the same hard gates.
        self.declare_parameter("structural_search", False)
        self.declare_parameter("structural_budget_s", 20.0)
        self.declare_parameter("max_candidates", 5)
        # Reachability atlas wiring (shared yaml; every key overridable).
        # luggage_planning is imported lazily: declaring the exec_depend
        # would close a colcon build cycle (see package.xml NOTE).
        self.declare_parameter("atlas_runtime_config", "")
        try:
            from luggage_planning.atlas_runtime import (
                PARAM_SPEC,
                apply_query_semantics,
                apply_ros_overrides,
                compute_reachability_scene_hash,
                compute_scene_tf_hash,
                load_atlas_runtime,
                load_atlas_runtime_for_scene,
                load_runtime_atlas,
            )
            from luggage_planning.urdf_identity import UrdfIdentity
        except ImportError as exc:
            PARAM_SPEC = ()
            load_atlas_runtime = None
            apply_query_semantics = None
            load_atlas_runtime_for_scene = None
            UrdfIdentity = None
            compute_scene_tf_hash = lambda path: ""  # noqa: E731
            compute_reachability_scene_hash = lambda path: ""  # noqa: E731
            self.get_logger().warning(
                "atlas runtime unavailable (%s); placement runs no_atlas",
                exc)
        config_path = str(self.get_parameter("scene_tf_config").value)
        if not config_path:
            config_path = resolve_scene_tf_config_path()
        if load_atlas_runtime_for_scene is not None:
            atlas_cfg = load_atlas_runtime_for_scene(
                config_path,
                str(self.get_parameter("atlas_runtime_config").value) or None)
            for ros_name, _path, _cast in PARAM_SPEC:
                default = atlas_cfg
                for key in _path:
                    default = default[key]
                self.declare_parameter(ros_name, default)
            apply_ros_overrides(self, atlas_cfg)
        else:
            atlas_cfg = {"enabled": False, "placement": {}, "motion": {},
                         "query_offset": None}
        self._atlas_cfg = atlas_cfg
        # The lazy imports above are locals of __init__; methods must go
        # through these handles (a direct global lookup NameErrors -- caught
        # once by test_placement_geometry_identity_live).
        self._load_runtime_atlas = (
            load_runtime_atlas if load_atlas_runtime is not None else None)
        self._apply_query_semantics = apply_query_semantics

        self._scene = load_scene_tf_config(config_path)
        self._scene_tf_hash = compute_scene_tf_hash(config_path)
        self._reachability_scene_hash = compute_reachability_scene_hash(
            config_path)

        self._allowed_yaws = [
            float(v) for v in self.get_parameter("allowed_yaws").value]
        self._params = {
            "clearance_margin": float(
                self.get_parameter("clearance_margin").value),
            "same_layer_clearance_m": float(
                self.get_parameter("same_layer_clearance_m").value),
            "min_support_ratio": float(
                self.get_parameter("min_support_ratio").value),
            "top_n": int(self.get_parameter("top_n").value),
            # Enough of the sliding window that corridor/aperture gates
            # see deep slots, not only the 8 highest scores.
            "keep_rejected": 200,
        }
        # Forward the wall margin into the solver params: the -1.0 auto
        # default resolves per-surface INSIDE the solver (it owns the
        # map resolution); an explicit value passes through unchanged.
        _hull_margin_param = float(self.get_parameter("hull_margin").value)
        self._params["hull_margin"] = (
            None if _hull_margin_param < 0.0 else _hull_margin_param)
        self._aperture_margin = float(
            self.get_parameter("aperture_margin").value)

        self._inner_size = self._inner_dimensions()
        self._floor_z = container_inner_floor_z(self._scene)
        self._aperture_y = self._aperture_bounds()
        self._surface = None
        self._hull = descriptor_from_scene_config(self._scene)
        self._geometry_hash = str(self._hull.geometry_hash)
        self._rejected_maps = 0
        # geometry-identity rejections (foreign-hull maps): the identity
        # contract's own counter, distinct from lifecycle admission
        # rejections (a first publication legitimately races the
        # validity binding and is finally rejected)
        self._geometry_rejected_maps = 0
        # Highest map revision admitted through either surface transport
        # (None = no map admitted yet). The v2 subscription enforces strict
        # progression against it; ComputePlacementV2 checks requests against
        # the active surface's revision.
        self._admitted_map_revision = None
        # OCToMap reset validity (plan OCTOMAP_RESET_VALIDITY_NEXT_20261103
        # §3): the latched observer validity stream drives the admission
        # gate. The floor prior serves only BEFORE the first map admission;
        # after an invalidation the gate stays closed until a fresh VALID
        # map and a post-transition robot-state sample both exist.
        self._admission = AdmissionGate()
        self._ever_admitted_map = False
        self._validity_seen = False
        self.declare_parameter("robot_state_max_age_ms", 500)
        self.declare_parameter("validity_gate_enforce", True)
        self._validity_gate_enforce = bool(
            self.get_parameter("validity_gate_enforce").value)
        # [G1] Common-consumer identity wiring: which validity stream this
        # deployment consumes (observer_o: /uld_occ_observer/validity,
        # grid_g: /cargo_map/validity) and the producer's status service
        # used ONLY for controlled confirmation of an unfamiliar identity
        # (a delayed old stream and a restarted producer are otherwise
        # indistinguishable; I2 forbids guessing).
        self.declare_parameter(
            "validity_topic", "/uld_occ_observer/validity")
        self.declare_parameter("validity_backend", "observer_o")
        self.declare_parameter(
            "validity_status_service",
            "/uld_occ_observer/get_validity")
        self._validity_backend = str(
            self.get_parameter("validity_backend").value or "observer_o")
        self._validity_topic = str(
            self.get_parameter("validity_topic").value)
        self._validity_status_service = str(
            self.get_parameter("validity_status_service").value or "")
        if not self._validity_gate_enforce:
            # I1: disabling the validity gate must not be a silent test
            # success path — a common deployment refuses to start.
            raise RuntimeError(
                "validity_gate_enforce=false is not a legal common-"
                "consumer deployment: the admission gate is mandatory")
        self._robot_max_age_ns = int(max(0, int(
            self.get_parameter("robot_state_max_age_ms").value)) * 1_000_000)
        # [Volume authorization + single-lease compute, user ruling
        # 2026-11-08] volume_evidence_source "" (default) keeps the
        # legacy topic-surface path with no volume gate. "observer" =
        # SINGLE-LEASE compute: every request pulls BOTH the surface AND
        # the volume evidence from ONE fresh observer lease (the latest
        # map) — plan and verify against the same revision by
        # construction; the two-reader identity-drift class is
        # structurally impossible and there is no mixed mode.
        self.declare_parameter("volume_evidence_source", "")
        self.declare_parameter("volume_observer_prefix",
                               "/uld_occ_observer")
        self.declare_parameter("volume_max_age_ms", 1000)
        self.declare_parameter("volume_observer_resolution", 0.02)
        self.declare_parameter("volume_service_timeout_s", 2.0)
        # [G v2 production wiring, 2026-11-09] "grid_g" = the G mapper's
        # own typed query services: ONE snapshot token supplies BOTH the
        # mapper's surface projection (get_surface_patch_v2 FULL) and the
        # paged cell evidence (query_cells_v2 CELLS) — the same
        # single-snapshot shape as the observer lease, with the token's
        # identity (epoch/revision) in place of the lease stamp. No
        # topic-surface fallback: any failure fails the request closed.
        self.declare_parameter("volume_grid_g_prefix", "/cargo_map")
        # Full-container typed cells dump for the volume gate. OFF by
        # default: the G service's token budget (five wall seconds,
        # 4096 cells/page hard cap) cannot serve a 4.3M-cell 1 cm grid
        # read — measured live 2026-11-09 (SNAPSHOT_EXPIRED mid
        # paging). The paging assembly below stays unit-pinned for when
        # the G service grows a budget-fitting evidence export; until
        # then grid_g computes on the snapshot surface with verifier
        # None (the chain-accepted G semantics).
        self.declare_parameter("volume_grid_g_cells", False)
        self._volume_clients = None
        self._grid_g_clients = None
        # [G1] Controlled confirmation driver: when the gate parks an
        # unfamiliar identity, this timer asks the producer's REAL status
        # service for its current identity (never a stream re-read).
        self._confirm_in_flight = False
        self._validity_status_client = None
        self._dropped_zero_stamp_robot_samples = 0
        self._confirmed_restarts = 0
        # Atlas handles keyed by requested box size (payload selection is
        # per-request; see atlas_runtime.resolve_atlas_files).
        self._atlas_cache = {}
        self._urdf_identity = None
        if UrdfIdentity is not None and self._atlas_cfg.get("enabled"):
            self._urdf_identity = UrdfIdentity(self)
            self._urdf_identity.start()
        if self._atlas_cfg.get("enabled"):
            self.get_logger().info(
                "atlas runtime gate on (dir=%s, w=%s, hard_reject=%s)"
                % (self._atlas_cfg.get("atlas_dir") or "reachability_atlas",
                   self._atlas_cfg["placement"].get("w_reachability"),
                   self._atlas_cfg["placement"].get("hard_reject")))

        map_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # ST05 surface mode: one map transport per deployment. With an
        # explicit scene revision the planner subscribes ONLY the v2
        # contract topic (a single admitted-revision stream); the v1 JSON
        # topic stays for v1 deployments. A v2-mode planner on a mapper
        # that does not publish v2 is a wiring gap: no map is admitted,
        # the floor prior serves, and requests fail closed on the
        # revision gate — never silently fed by the v1 topic. The v1
        # ComputePlacement service keeps answering in v2 mode from the
        # same admitted map (shared normalization, response unchanged).
        self.declare_parameter("frame_scene_revision", "")
        self._frame_scene_revision = str(
            self.get_parameter("frame_scene_revision").value or "")
        self._v2_surface_mode = bool(self._frame_scene_revision)
        if not self._v2_surface_mode:
            self.create_subscription(
                String, "/luggage/cargo_map/surface_2d",
                self._on_surface, map_qos, callback_group=group)
        else:
            self.create_subscription(
                SurfaceContractV2, "/luggage/cargo_map/surface_v2",
                self._on_surface_v2, map_qos, callback_group=group)
            # [B2] The mapper's committed ledger is the single placed
            # source: the request-carried copy is validated against it,
            # and the solver input is derived from it, never from the
            # driver's local bookkeeping.
            self.create_subscription(
                String, "/luggage/cargo_map/committed",
                self._on_committed, map_qos, callback_group=group)
            self.create_service(
                ComputePlacementV2, "/placement_planner/compute_placement_v2",
                self._handle_v2, callback_group=group)
        self._last_pub = self.create_publisher(
            String, "/placement_planner/last_result", map_qos)
        # Reset-contract inputs: latched validity (replayable via the
        # transient-local QoS) and the robot joint state whose freshness
        # gates recovery. The validity arrival is re-stamped in this node's
        # clock so robot-sample comparisons stay in one time domain.
        self.create_subscription(
            String, self._validity_topic,
            self._on_validity, map_qos, callback_group=group)
        from sensor_msgs.msg import JointState
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10,
            callback_group=group)
        if self._validity_status_service:
            # Both producers expose the same field names on their status
            # services (observer GetValidity / grid GetCargoValidity).
            if self._validity_backend == "grid_g":
                from luggage_msgs.srv import GetCargoValidity as _StatusSrv
            else:
                from uld_occ_observer.srv import GetValidity as _StatusSrv
            self._validity_status_client = self.create_client(
                _StatusSrv, self._validity_status_service)
        self._confirm_timer = self.create_timer(
            0.5, self._drive_pending_confirmation,
            callback_group=group)

        self.create_service(
            ComputePlacement, "/placement_planner/compute_placement",
            self._handle, callback_group=group)

        # ST09 F21: when frame_runtime is on, this node publishes its
        # measured identity, observes peers, and admits placement requests
        # only against a fully ready v2 graph. Off = legacy byte-identical.
        self.declare_parameter("frame_runtime", False)
        self.declare_parameter("frame_identity_json", "")
        self.declare_parameter("frame_peers", "")
        self._runtime = None
        if bool(self.get_parameter("frame_runtime").value):
            import sys as _sys
            from luggage_description.ros_frame_bindings.runtime import (
                node_runtime_from_params)
            # The frame contract needs a concrete float at startup (before
            # any surface is admitted): the -1.0 auto default resolves to
            # the exact face here — the placement margin itself is applied
            # per-surface in the solver and _constraint_verdict.
            _frame_hull_margin = float(
                self.get_parameter("hull_margin").value)
            self._runtime = node_runtime_from_params(
                self,
                resolved_config={
                    "scene_tf_config": str(
                        self.get_parameter("scene_tf_config").value or ""),
                    "frame_scene_revision": self._frame_scene_revision,
                    "hull_margin": max(0.0, _frame_hull_margin),
                    "atlas_runtime_config": str(
                        self.get_parameter("atlas_runtime_config").value or ""),
                },
                modules=[(_sys.modules["__main__"], "module", "application")],
                package_names=["luggage_packing"])

        self.get_logger().info(
            "placement_planner ready (aperture_y=%s, corridor=request_box, "
            "geometry_hash=%s)"
            % (self._aperture_y, self._geometry_hash))

        # [G1/V2] Behavior-test park points (param-gated, off default):
        # a listed point publishes its arrival on
        # /placement_planner/test_stage and blocks until released on
        # /placement_planner/test_release (30s safety timeout = missed
        # barrier). Parking the SERVICE handler is safe for reset races:
        # validity arrives on a separate reentrant group, so the gate
        # advances while the computation is parked.
        self.declare_parameter("behavior_test_hooks", "")
        self._hook_points = {
            name.strip() for name in str(
                self.get_parameter("behavior_test_hooks").value).split(",")
            if name.strip()}
        self._hook_pub = self.create_publisher(
            String, "/placement_planner/test_stage", 10)
        self._hook_release = {}
        import threading as _threading
        self._hook_lock = _threading.Lock()

        def _on_release(msg):
            with self._hook_lock:
                for name, event in list(self._hook_release.items()):
                    if not msg.data or msg.data == name:
                        event.set()

        self._hook_sub = self.create_subscription(
            String, "/placement_planner/test_release", _on_release, 10)

    def behavior_hook(self, name, detail=""):
        """Park at a named point when enabled (V2 race barriers)."""
        if name not in self._hook_points:
            return
        import threading as _threading
        with self._hook_lock:
            event = _threading.Event()
            self._hook_release[name] = event
        message = String()
        message.data = "%s:%s" % (name, detail)
        self._hook_pub.publish(message)
        released = event.wait(timeout=30.0)
        with self._hook_lock:
            self._hook_release.pop(name, None)
        if not released:
            self.get_logger().warning(
                "behavior hook %s timed out (missed barrier)" % name)

    def _hull_geometry(self):
        """Authoritative kernel hull (cached; config is immutable here)."""
        return self._hull

    # ------------------------------------------------------------------
    # Scene-derived bounds

    def _inner_dimensions(self):
        """[FRAME MAP] inner_size[2] is the interior HEIGHT
        (ceiling_z - floor_z = 1.48), and every z in the solver chain is
        FLOOR-RELATIVE container z (0 at the inner floor). This is a
        DIFFERENT convention from the oracle pair, whose
        hull.inner_size[2] is read as the container-ABSOLUTE ceiling
        (2.01) — see v1_states._k01_hull and AGENTS.md "ULD frames"."""
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

    def _validity_admit(self):
        """Admission check BEFORE planning (§3).

        Returns ``(reason, admission)``: ``reason`` is None when planning
        may proceed, ``admission`` is the immutable token captured BEFORE
        computing. The matching commit must present exactly this token —
        re-reading the board's current generation at commit is the gen1
        defect (review 2026-11-03): after reset+recovery it admitted the
        old computation. [G1] The gate is MANDATORY in a common
        deployment: before the first validity publication it fails closed
        (VALIDITY_GATE_UNBOUND) instead of serving the floor prior.
        """
        if not self._validity_seen:
            return ("VALIDITY_GATE_UNBOUND: no validity publication "
                    "observed on %s" % self._validity_topic), None
        decision = self._admission.admit_planning(
            now_mono_ns=int(self.get_clock().now().nanoseconds),
            robot_max_age_ns=self._robot_max_age_ns)
        if decision.admitted:
            return None, decision.admission_token
        return ("VALIDITY_GATE_CLOSED: %s (state=%s epoch=%s gen=%s "
                "execution_active=%s)"
                % (decision.reason, decision.validity.state,
                   decision.validity.epoch, decision.validity.generation,
                   decision.execution_active)), None

    def _validity_commit(self, planning_admission):
        """Commit check BEFORE delivering a successful plan (§3): the
        second epoch/generation look against the admission captured before
        computing. Returns None to commit, else the rejection reason."""
        token = planning_admission
        decision = self._admission.commit_plan(
            token.generation if token is not None else None,
            admission_token=token,
            now_mono_ns=int(self.get_clock().now().nanoseconds),
            robot_max_age_ns=self._robot_max_age_ns)
        if decision.admitted:
            return None
        return ("VALIDITY_COMMIT_REJECTED: %s (state=%s epoch=%s gen=%s "
                "planned_gen=%s execution_active=%s)"
                % (decision.reason, decision.validity.state,
                   decision.validity.epoch, decision.validity.generation,
                   token.generation if token is not None else None,
                   decision.execution_active))

    def _on_validity(self, msg):
        """Consume the producer's latched validity publication (§2/§3).

        The arrival time replaces the producer's monotonic transition stamp
        so the robot-sample freshness comparison stays in one clock domain;
        arrival is strictly after the transition, which keeps the recovery
        rule conservative (a sample must post-date the transition, and a
        sample newer than arrival certainly does). The producer's original
        stamp is preserved on the observation (judge evidence). A malformed
        payload or an epoch illegal for this deployment's backend is
        counted and dropped — never served, never fatal.
        """
        try:
            payload = json.loads(msg.data)
        except ValueError:
            self._rejected_validity_messages = \
                getattr(self, "_rejected_validity_messages", 0) + 1
            return
        try:
            observation = ValidityObservation.from_json(
                payload, backend=self._validity_backend)
        except EpochFormatError as error:
            self._rejected_validity_messages = \
                getattr(self, "_rejected_validity_messages", 0) + 1
            self.get_logger().error(
                "rejected validity payload (epoch format: %s)" % error)
            return
        self._validity_seen = True
        observation = ValidityObservation(
            state=observation.state, epoch=observation.epoch,
            generation=observation.generation, reason=observation.reason,
            reset_id=observation.reset_id,
            transition_mono_ns=self.get_clock().now().nanoseconds,
            map_revision=observation.map_revision,
            observed_revision=observation.observed_revision,
            geometry_hash=observation.geometry_hash,
            config_digest=observation.config_digest,
            clock_domain=observation.clock_domain,
            backend=observation.backend,
            producer_transition_mono_ns=(
                observation.producer_transition_mono_ns))
        self._admission.observe_validity(observation)
        if observation.state != "VALID" and self._ever_admitted_map:
            # Invalidation observed: placement stops. The admitted map and
            # its candidates die here; the gate reopens only through a fresh
            # VALID map + post-transition robot state.
            if self._surface is not None:
                self._surface = None
            self._admitted_map_revision = None
            self.get_logger().warning(
                "validity %s epoch %s gen %s: admitted map invalidated"
                % (observation.state, observation.epoch,
                   observation.generation))

    def _drive_pending_confirmation(self):
        """Resolve parked unfamiliar identities via the producer's status.

        Runs only when the gate parked something AND the real status
        service answers; a failed/unavailable query keeps the gate closed
        with the parking intact (named, never a guess).
        """
        if (self._confirm_in_flight or not self._validity_status_client
                or not self._admission.pending_confirmation):
            return
        self._confirm_in_flight = True
        try:
            if not self._validity_status_client.wait_for_service(
                    timeout_sec=0.1):
                return
            future = self._validity_status_client.call_async(
                self._validity_status_client.srv_type.Request())
            import time as _time
            deadline = _time.monotonic() + 1.0
            while not future.done() and _time.monotonic() < deadline:
                _time.sleep(0.02)
            if not future.done():
                return
            response = future.result()
            observation = ValidityObservation(
                state=str(response.state),
                epoch=(response.epoch if self._validity_backend == "grid_g"
                       else int(response.epoch)),
                generation=int(response.generation),
                reason=str(response.reason or ""),
                reset_id=str(response.reset_id or ""),
                transition_mono_ns=self.get_clock().now().nanoseconds,
                map_revision=int(response.map_revision),
                observed_revision=int(response.observed_revision),
                geometry_hash=str(response.geometry_hash or ""),
                config_digest=str(response.config_digest or ""),
                clock_domain=str(response.clock_domain or ""),
                backend=self._validity_backend)
            decision = self._admission.confirm_producer(observation)
            if decision.admitted:
                self._confirmed_restarts += 1
                self.get_logger().warning(
                    "confirmed producer identity via %s: epoch=%s gen=%s"
                    % (self._validity_status_service,
                       observation.epoch, observation.generation))
        finally:
            self._confirm_in_flight = False

    def _on_joint_state(self, msg):
        stamp = int(msg.header.stamp.sec) * 1_000_000_000 \
            + int(msg.header.stamp.nanosec)
        if not stamp:
            # [G1/I2] A zero stamp is not "now": backfilling would fabricate
            # freshness. The sample is dropped and counted.
            self._dropped_zero_stamp_robot_samples += 1
            return
        self._admission.observe_robot_state(RobotStateSample(
            stamp, "joint_states", {"name": list(msg.name)}))

    def _compute_inputs(self):
        """(surface, volume_verifier) for one compute request.

        volume_evidence_source "" (default): the admitted topic surface,
        no volume gate (legacy path). "observer": SINGLE-LEASE compute
        (user ruling 2026-11-08 — no mixed two-reader mode): one fresh
        observer lease supplies BOTH the surface (ReadSurface ->
        conservative bridge mapping over this planner's own hull) and
        the volume evidence (paged ReadCells), so the solver plans and
        the verifier verifies against the same revision by construction
        and every compute takes the observer's latest map. Failures
        raise; the caller fails the request closed — never a silent
        fallback to the topic surface."""
        source = str(
            self.get_parameter("volume_evidence_source").value or "")
        if not source:
            surface = self._active_surface()
            # resolve the auto wall margin ONCE per compute (the active
            # surface view may be rebuilt on access — doing this inside
            # _constraint_verdict cost one rebuild PER CANDIDATE and ate
            # the E4 500 ms robot window; K01/E4 run09 regression)
            self._constraint_margin = (
                float(surface.get("resolution") or 0.0)
                if float(self.get_parameter("hull_margin").value) < 0.0
                else float(self.get_parameter("hull_margin").value))
            return surface, None
        if source == "grid_g":
            return self._grid_g_snapshot_compute()
        if source != "observer":
            raise ValueError(
                "unknown volume_evidence_source %r" % source)
        return self._observer_snapshot_compute()

    def _observer_snapshot_compute(self):
        """One observer lease -> (surface, volume verifier).

        The lease's ValidityStamp must read VALID at the epoch this
        planner's validity stream is bound to (reset contract), and the
        lease-built surface enters the SAME admission gate as topic
        surfaces. Transport, binding, admission, or completeness
        failures raise RuntimeError (fail closed)."""
        import time as _time
        from uld_occ_observer.srv import (GetSnapshot, ReadCells,
                                          ReadSurface, ReleaseSnapshot)
        try:
            import occ_surface_bridge as _bridge
        except ImportError as error:
            # Deployment pairing: the observer package ships the bridge
            # module beside its scripts (not a compiled library); the
            # deployment must put it on PYTHONPATH. Fail closed, named.
            raise RuntimeError(
                "occ_surface_bridge not importable (the observer "
                "package must ship it on PYTHONPATH): %s" % error)

        if self._volume_clients is None:
            prefix = str(
                self.get_parameter("volume_observer_prefix").value)
            self._volume_clients = {
                name: self.create_client(srv, "%s/%s" % (prefix, name))
                for name, srv in (
                    ("get_snapshot", GetSnapshot),
                    ("read_surface", ReadSurface),
                    ("read_cells", ReadCells),
                    ("release_snapshot", ReleaseSnapshot))}

        timeout_s = float(
            self.get_parameter("volume_service_timeout_s").value)

        def call(client, request):
            future = client.call_async(request)
            deadline = _time.monotonic() + max(timeout_s, 10.0)
            while not future.done():
                if _time.monotonic() > deadline:
                    raise RuntimeError(
                        "service timeout: %s" % client.srv_name)
                _time.sleep(0.02)
            return future.result()

        for client in self._volume_clients.values():
            if not client.wait_for_service(timeout_sec=timeout_s):
                raise RuntimeError(
                    "service missing: %s" % client.srv_name)
        lease = call(
            self._volume_clients["get_snapshot"],
            GetSnapshot.Request(max_age_ms=int(
                self.get_parameter("volume_max_age_ms").value)))
        if str(lease.status) != "OK":
            raise RuntimeError("get_snapshot %s" % lease.status)
        # Reset-contract binding ON THE LEASE: the observer's validity
        # board must be VALID, at the epoch this planner is bound to,
        # when the snapshot was taken (the stamp is from the response's
        # own commit check).
        stamp = lease.validity
        if str(stamp.state) != "VALID":
            raise RuntimeError(
                "observer validity %s (%s)" % (stamp.state, stamp.reason))
        current_epoch = str(
            getattr(self._admission.validity, "epoch", "") or "")
        if current_epoch and str(stamp.epoch) != current_epoch:
            raise RuntimeError(
                "lease epoch %s != admitted epoch %s"
                % (stamp.epoch, current_epoch))
        identity = {
            "geometry_hash": str(lease.identity.geometry_hash),
            "map_revision": int(lease.identity.map_revision),
            "epoch": int(lease.identity.epoch),
            "observed_revision": int(lease.identity.observed_revision),
            "last_integrated_stamp_ns":
                int(lease.identity.last_integrated_stamp_ns),
            "clock_domain": str(lease.identity.clock_domain),
            "validity_generation": int(stamp.generation),
        }
        resolution = float(
            self.get_parameter("volume_observer_resolution").value)
        try:
            columns = []
            cursor, done = 0, False
            while not done:
                page = call(
                    self._volume_clients["read_surface"],
                    ReadSurface.Request(handle=lease.handle, cursor=cursor,
                                        limit=65536))
                if str(page.status) != "OK":
                    raise RuntimeError(
                        "read_surface %s" % page.status)
                for i in range(len(page.x)):
                    columns.append({
                        "x": page.x[i], "y": page.y[i],
                        "active_count": page.active_count[i],
                        "known_count": page.known_count[i],
                        "has_occupied": page.has_occupied[i],
                        "top_z": page.top_z[i]})
                cursor, done = page.next_cursor, page.done
            geometry = dict(self._hull.descriptor())
            geometry["_resolution"] = resolution
            surface = _bridge.build_surface_map(columns, geometry, identity)
            if "error" in surface:
                raise RuntimeError(
                    "surface build: %s" % surface["error"])
            # the lease surface passes the SAME admission gate as topic
            # surfaces: epoch / validity generation / map progression
            if not self._admit_surface_map(surface, "observer_lease"):
                raise RuntimeError(
                    "lease surface rejected by admission gate")
            nx, ny = int(surface["nx"]), int(surface["ny"])
            inner_h = float(surface["inner_size"][2])
            nz = int(math.ceil(inner_h / resolution - 1e-9))
            pages = []
            cursor, done = 0, False
            while not done:
                page = call(
                    self._volume_clients["read_cells"],
                    ReadCells.Request(handle=lease.handle,
                                      begin=[0, 0, 0], end=[nx, ny, nz],
                                      cursor=cursor, limit=65536))
                if str(page.status) != "OK":
                    raise RuntimeError("read_cells %s" % page.status)
                pages.append({
                    "status": str(page.status),
                    "states": page.states,
                    "identity": {
                        "geometry_hash": str(page.identity.geometry_hash),
                        "map_revision": int(page.identity.map_revision),
                        "epoch": int(page.identity.epoch),
                        "observed_revision":
                            int(page.identity.observed_revision),
                    }})
                cursor, done = int(page.next_cursor), bool(page.done)
                if len(pages) > 64:
                    raise RuntimeError("read_cells paging did not finish")
        finally:
            self._volume_clients["release_snapshot"].call_async(
                ReleaseSnapshot.Request(handle=lease.handle))
        evidence = volume_evidence_from_pages(
            pages, resolution=resolution, nx=nx, ny=ny, nz=nz,
            origin_xy=[-nx * resolution * 0.5, -ny * resolution * 0.5],
            floor_z=float(geometry["floor_z"]), identity=identity)
        if "error" in evidence:
            raise RuntimeError(
                "cells evidence: %s" % evidence["error"])
        self.get_logger().info(
            "observer lease epoch=%s rev=%d gen=%d -> %d columns, "
            "%d cells pages" % (identity["epoch"], identity["map_revision"],
                                identity["validity_generation"],
                                len(columns), len(pages)))
        return surface, make_volume_verifier(evidence)

    def _grid_g_snapshot_compute(self):
        """One /cargo_map snapshot token -> (surface, volume verifier).

        grid_g single-snapshot compute (G v2 production wiring): the G
        mapper's typed query services share one snapshot-token
        mechanism, so ONE ``use_current`` acquire supplies BOTH the
        mapper's own surface projection (get_surface_patch_v2 FULL)
        and the paged cell evidence (query_cells_v2 CELLS) — the same
        single-snapshot shape as the observer lease, with the token's
        identity (epoch/revision) in place of the lease stamp. Surface
        and cells are bound to ONE revision by construction; there is
        no topic-surface fallback. Transport, binding, identity, or
        admission failures raise RuntimeError (fail closed).

        Cells (volume evidence) only run with volume_grid_g_cells:=true:
        the typed dump cannot fit the service's five-second token at
        1 cm (see the parameter note), so the default returns
        verifier=None — the volume authorization gate stays
        observer-wired until the G service offers a budget-fitting
        evidence export.
        """
        import time as _time
        from luggage_msgs.srv import (GetCargoSurfacePatchV2,
                                      QueryCargoCellsV2)

        if self._validity_backend != "grid_g":
            raise RuntimeError(
                "volume_evidence_source=grid_g requires "
                "validity_backend=grid_g (got %r)"
                % self._validity_backend)
        if self._grid_g_clients is None:
            prefix = str(
                self.get_parameter("volume_grid_g_prefix").value)
            from luggage_msgs.srv import QueryCargoCellsDenseV2
            self._grid_g_clients = {
                name: self.create_client(srv, "%s/%s" % (prefix, name))
                for name, srv in (
                    ("query_cells_dense_v2", QueryCargoCellsDenseV2),
                    ("get_surface_patch_v2", GetCargoSurfacePatchV2))}

        timeout_s = float(
            self.get_parameter("volume_service_timeout_s").value)

        def call(client, request):
            future = client.call_async(request)
            deadline = _time.monotonic() + max(timeout_s, 10.0)
            while not future.done():
                if _time.monotonic() > deadline:
                    raise RuntimeError(
                        "service timeout: %s" % client.srv_name)
                _time.sleep(0.02)
            return future.result()

        cells_mode = bool(
            self.get_parameter("volume_grid_g_cells").value)
        # only the clients the CONFIGURED mode needs must be present:
        # surface-only deployments run against mappers predating the
        # dense export (their typed cells path cannot serve a full-grid
        # read anyway) — requiring the dense service there would break
        # the surface-only mode that needs nothing beyond the patch.
        needed = ("get_surface_patch_v2",) if not cells_mode else (
            "get_surface_patch_v2", "query_cells_dense_v2")
        for name in needed:
            client = self._grid_g_clients[name]
            if not client.wait_for_service(timeout_sec=timeout_s):
                raise RuntimeError(
                    "service missing: %s" % client.srv_name)

        # 1) acquire a FRESH snapshot and pull the mapper's own surface
        #    projection from it in one call; the returned token keeps
        #    the session open for the cell pages below.
        request = GetCargoSurfacePatchV2.Request()
        request.mode = GetCargoSurfacePatchV2.Request.FULL
        request.query.request_id = "planner-surface-%d" % (
            _time.monotonic_ns(),)
        request.query.frame_id = "container_link"
        request.query.use_current = True
        request.query.release_after_query = not cells_mode
        patch = call(self._grid_g_clients["get_surface_patch_v2"], request)
        meta = patch.meta
        if str(meta.status) != "OK":
            raise RuntimeError("surface_patch %s (%s)"
                               % (meta.status, meta.reason))
        token = str(meta.snapshot_token)
        if cells_mode and not token:
            raise RuntimeError(
                "surface_patch returned no snapshot token")
        ident = meta.identity
        # Same-snapshot epoch binding (reset contract): the snapshot
        # must belong to the epoch this planner's validity stream is
        # bound to — the grid_g analog of the observer lease stamp.
        # The validity intake (from_json) canonicalizes epochs; the
        # snapshot identity arrives raw, so normalize before comparing
        # (a dashed UUID and its 32-hex form are one epoch).
        try:
            snapshot_epoch = normalize_epoch(str(ident.epoch),
                                             self._validity_backend)
        except EpochFormatError as error:
            raise RuntimeError("snapshot epoch format: %s" % error)
        current_epoch = str(
            getattr(self._admission.validity, "epoch", "") or "")
        if current_epoch and snapshot_epoch != current_epoch:
            raise RuntimeError(
                "snapshot epoch %s != admitted epoch %s"
                % (snapshot_epoch, current_epoch))
        if str(meta.grid.geometry_hash) != self._geometry_hash:
            # GEO-8: never consume a map built for another hull, and
            # never answer from the floor prior instead.
            raise RuntimeError(
                "CARGO_MAP_GEOMETRY_MISMATCH: snapshot hull %s != "
                "planner hull %s" % (meta.grid.geometry_hash,
                                     self._geometry_hash))

        # 2) rebuild the mapper's surface record from the patch arrays.
        #    The patch raveled layout is x-fastest (i = x + shape_x*y);
        #    the record layout is rows of columns (row = x, len = ny) —
        #    the same shape surface_from_message produces for the topic
        #    contract, so the planner's own view conversion applies.
        state_names = {0: "unknown", 1: "free", 2: "occupied"}
        source_names = {0: "unknown", 1: "sensor", 2: "verified_geometry"}
        nx, ny = int(patch.shape_xy[0]), int(patch.shape_xy[1])
        if nx <= 0 or ny <= 0 or len(patch.surface_z_container) != nx * ny:
            raise RuntimeError("surface patch shape mismatch")
        z_rows, s_rows, c_rows = [], [], []
        for ix in range(nx):
            z_row, s_row, c_row = [], [], []
            for iy in range(ny):
                i = ix + nx * iy
                if bool(patch.has_surface_z[i]):
                    z_row.append(float(patch.surface_z_container[i]))
                    s_row.append(state_names.get(
                        int(patch.state[i]), "unknown"))
                    c_row.append(source_names.get(
                        int(patch.contract_confidence[i]), "unknown"))
                else:
                    z_row.append(None)
                    s_row.append("unknown")
                    c_row.append("none")
            z_rows.append(z_row)
            s_rows.append(s_row)
            c_rows.append(c_row)
        view = self._surface_view_from_record({
            "geometry_hash": str(meta.grid.geometry_hash),
            "floor_z_container": float(patch.floor_z_container),
            "resolution_m": float(meta.grid.resolution),
            "surface_z_container": z_rows,
            "state": s_rows,
            "confidence": c_rows,
            "map_revision": int(ident.revision)})
        # Opportunistic cross-check against an admitted topic surface of
        # the SAME revision: a transposed x/y rebuild would show up as
        # swapped grid dims here.
        topic = self._surface
        if (topic is not None
                and int(topic.get("map_revision", -1) or -1)
                == int(ident.revision)
                and (int(topic.get("nx", 0) or 0),
                     int(topic.get("ny", 0) or 0)) != (nx, ny)):
            raise RuntimeError(
                "snapshot surface %dx%d != admitted topic surface %dx%d"
                % (nx, ny, topic.get("nx"), topic.get("ny")))
        # The snapshot surface enters the SAME admission gate as topic
        # surfaces (epoch / generation / revision binding). Generation
        # is the planner's current validity generation — the snapshot
        # epoch binding above ties the two.
        if not self._admit_surface_map({
                "epoch": snapshot_epoch,
                "validity_generation": int(getattr(
                    self._admission.validity, "generation", 0) or 0),
                "map_revision": int(ident.revision),
                "geometry_hash": str(meta.grid.geometry_hash)},
                "grid_g_snapshot"):
            raise RuntimeError(
                "snapshot surface rejected by admission gate")
        if not cells_mode:
            self.get_logger().info(
                "grid_g snapshot epoch=%s rev=%d -> surface %dx%d "
                "(cells off: volume gate not wired for this backend)"
                % (snapshot_epoch, int(ident.revision), nx, ny))
            return view, None

        # 3) pull the SAME snapshot's DENSE state plane (dense export,
        #    unlock ruling 2026-11-10): one uint8 per cell, x-fastest —
        #    byte-for-byte the evidence layout. The typed per-cell path
        #    (~28 us/cell) could not serve a 4.3M-cell 1 cm read inside
        #    the service's five-second token window; the dense plane is
        #    ~66 pages of 64 KiB. Anchored on the snapshot's OWN grid
        #    origin, so cell z and the verifier's container-z candidates
        #    stay in one frame by construction.
        resolution = float(meta.grid.resolution)
        grid_shape = [int(v) for v in meta.grid.shape]
        if grid_shape[0] != nx or grid_shape[1] != ny:
            raise RuntimeError(
                "snapshot cell grid %s != surface grid %dx%d"
                % (grid_shape, nx, ny))
        inner_h = float(view["inner_size"][2])
        nz = int(math.ceil(inner_h / resolution - 1e-9))
        if grid_shape[2] < nz:
            raise RuntimeError(
                "snapshot grid z %d < planner volume nz %d"
                % (grid_shape[2], nz))

        def _identity_fields(message_identity):
            return {field: str(getattr(message_identity, field))
                    for field in ("epoch", "revision", "content_digest",
                                  "voxel_digest", "index_schema_version")}

        expected = _identity_fields(ident)
        from luggage_msgs.srv import QueryCargoCellsDenseV2
        states = bytearray()          # x-fastest plane bytes accumulate here
        cursor, pages = "", 0
        try:
            while True:
                request = QueryCargoCellsDenseV2.Request()
                request.query.request_id = "planner-dense-%d" % (
                    _time.monotonic_ns(),)
                request.query.frame_id = "container_link"
                request.query.use_current = pages == 0
                request.query.snapshot_token = "" if pages == 0 else token
                request.query.cursor = cursor
                request.query.limit = 65536
                if pages:
                    for field in ("epoch", "revision", "content_digest",
                                  "voxel_digest", "index_schema_version"):
                        setattr(request.query.expected_identity, field,
                                getattr(ident, field))
                page = call(
                    self._grid_g_clients["query_cells_dense_v2"], request)
                pages += 1
                if str(page.meta.status) != "OK":
                    raise RuntimeError("query_cells_dense %s (%s)"
                                       % (page.meta.status,
                                          page.meta.reason))
                if _identity_fields(page.meta.identity) != expected:
                    raise RuntimeError(
                        "dense plane identity != the surface snapshot")
                states += page.states
                if bool(page.meta.complete) or not str(
                        page.meta.next_cursor):
                    break
                cursor = str(page.meta.next_cursor)
                if pages > 128:
                    raise RuntimeError(
                        "query_cells_dense paging did not finish")
        finally:
            # Best-effort explicit release (a one-byte page with
            # release_after_query): if this call is lost the server
            # expires the token within its five-second window anyway.
            release = QueryCargoCellsDenseV2.Request()
            release.query.request_id = "planner-release-%d" % (
                _time.monotonic_ns(),)
            release.query.frame_id = "container_link"
            release.query.use_current = False
            release.query.snapshot_token = token
            release.query.release_after_query = True
            release.query.limit = 1
            for field in ("epoch", "revision", "content_digest",
                          "voxel_digest", "index_schema_version"):
                setattr(release.query.expected_identity, field,
                        getattr(ident, field))
            self._grid_g_clients["query_cells_dense_v2"].call_async(release)

        if len(states) < nx * ny * nz:
            raise RuntimeError(
                "dense plane short: %d of %d cells"
                % (len(states), nx * ny * nz))
        # iz is the SLOWEST axis of the x-fastest plane, so the first
        # nx*ny*nz bytes are exactly the planner's nz layers.
        states = bytes(states[:nx * ny * nz])
        identity = {
            "geometry_hash": str(meta.grid.geometry_hash),
            "map_revision": int(ident.revision),
            "epoch": str(ident.epoch)}
        evidence = volume_evidence_from_pages(
            [{"status": "OK", "states": states, "identity": identity}],
            resolution=resolution, nx=nx, ny=ny, nz=nz,
            origin_xy=[float(meta.grid.origin.x),
                       float(meta.grid.origin.y)],
            floor_z=float(meta.grid.origin.z),
            identity=identity)
        if "error" in evidence:
            raise RuntimeError(
                "cells evidence: %s" % evidence["error"])
        self.get_logger().info(
            "grid_g snapshot epoch=%s rev=%d -> surface %dx%d, %d dense "
            "pages (%d bytes)" % (ident.epoch, int(ident.revision),
                                  nx, ny, pages, len(states)))
        return view, make_volume_verifier(evidence)

    def _evidence_unavailable_code(self):
        """Named fail-closed code for the configured evidence source."""
        if str(self.get_parameter("volume_evidence_source").value
               or "") == "grid_g":
            return "GRID_G_SNAPSHOT_UNAVAILABLE"
        return "OBSERVER_SNAPSHOT_UNAVAILABLE"

    def _admit_surface_map(self, surface, source):
        """Admission-gate a freshly delivered surface map (v1 or v2).

        Returns True when the map is bound to the current VALID epoch and
        generation; a mismatched map is counted and never stored. Epochs
        are normalized per this deployment's backend (observer uint64 ->
        decimal string, grid UUID verbatim); a malformed epoch rejects the
        map instead of defaulting to 0.
        """
        try:
            epoch = normalize_epoch(surface.get("epoch"),
                                    self._validity_backend)
        except EpochFormatError:
            self._rejected_maps += 1
            self.get_logger().error(
                "rejected %s cargo map: epoch format invalid for %s "
                "(epoch=%r, rejected=%d)"
                % (source, self._validity_backend, surface.get("epoch"),
                   self._rejected_maps))
            return False
        decision = self._admission.observe_map(MapObservation(
            epoch=epoch,
            generation=int(surface.get("validity_generation", 0) or 0),
            map_revision=int(surface.get("map_revision", 0) or 0),
            geometry_hash=str(surface.get("geometry_hash") or ""),
            backend=self._validity_backend,
            payload=surface))
        if not decision.admitted:
            self._rejected_maps += 1
            self.get_logger().error(
                "rejected %s cargo map by admission gate: %s "
                "(epoch=%s gen=%s, rejected=%d)"
                % (source, decision.reason,
                   surface.get("epoch"), surface.get("validity_generation"),
                   self._rejected_maps))
            return False
        self._ever_admitted_map = True
        return True

    def _on_surface(self, msg):
        if getattr(self, "_v2_surface_mode", False):
            # v2 mode subscribes only the contract topic; a v1 JSON
            # message delivered here anyway is not this deployment's map
            # stream. Consuming it could regress the admitted v2 map.
            self._rejected_maps += 1
            self.get_logger().error(
                "ignored v1 cargo map in v2 surface mode (rejected=%d)"
                % self._rejected_maps)
            return
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
            self._geometry_rejected_maps += 1
            self.get_logger().error(
                "rejected cargo map: geometry_hash %s != planner %s "
                "(rejected=%d)"
                % (incoming or "<missing>", self._geometry_hash,
                   self._rejected_maps))
            return
        if not self._admit_surface_map(surface, "v1"):
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
            "geometry_descriptor": dict(self._hull.descriptor()),
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
    # ST05 v2 surface ingest + ComputePlacementV2 (F07/F08 producer)

    def _surface_context(self, request_id, map_revision):
        return BoundaryContext(
            "F06", "cargo_volume_mapper", "placement_planner",
            str(request_id), self._frame_scene_revision, "container_link",
            "", "scene", geometry_hash=self._geometry_hash,
            map_revision=int(map_revision))

    def _on_surface_v2(self, msg):
        """Ingest the v2 surface record beside the v1 JSON topic.

        Subscriber admission policy (checked before the strict adapter):
        correlation identity — the mapper stamps every publication with
        ``map-rev-<revision>``, so a message whose request_id disagrees
        with its claimed revision is not the mapper's map — and strict
        revision progression: a revision older than or equal to the
        admitted one is a stale or reordered duplicate, never re-ingested.
        Scene and hull binding stay inside ``surface_from_message`` with
        consumer-owned identities (this node's configured scene revision
        and hull hash, never values copied from the message); the decoded
        record's hull is re-checked below on top of the strict validation.
        """
        claimed_revision = int(getattr(msg, "map_revision", -1))
        expected_request = SURFACE_V2_REQUEST_TEMPLATE % claimed_revision
        if str(msg.identity.request_id or "") != expected_request:
            self._rejected_maps += 1
            self.get_logger().error(
                "rejected v2 cargo map: correlation %r != expected %r "
                "(revision %d, rejected=%d)"
                % (msg.identity.request_id, expected_request,
                   claimed_revision, self._rejected_maps))
            return
        admitted = self._admitted_map_revision
        if admitted is not None and claimed_revision <= admitted:
            self._rejected_maps += 1
            self.get_logger().error(
                "rejected v2 cargo map: stale/reordered revision %d <= "
                "admitted %d (rejected=%d)"
                % (claimed_revision, admitted, self._rejected_maps))
            return
        try:
            record = surface_from_message(
                msg, self._surface_context(expected_request, claimed_revision))
        except ContractError as error:
            self._rejected_maps += 1
            if "GEOMETRY" in str(error.code):
                # the contract layer's hull rejection (e.g.
                # GEOMETRY_IDENTITY_MISMATCH on the geometry_hash field)
                # is the SAME fact as the post-decode hash check below —
                # it must trip the foreign-hull fail-closed gate too
                self._geometry_rejected_maps += 1
            self.get_logger().error(
                "rejected v2 cargo map (%s %s, rejected=%d)"
                % (error.code, error.field, self._rejected_maps))
            return
        if str(record.get("geometry_hash") or "") != self._geometry_hash:
            self._rejected_maps += 1
            self._geometry_rejected_maps += 1
            self.get_logger().error(
                "rejected v2 cargo map: geometry_hash %s != planner %s"
                % (record.get("geometry_hash"), self._geometry_hash))
            return
        if not self._admit_surface_map({
                "epoch": str(getattr(msg, "epoch", "") or ""),
                "validity_generation": int(
                    getattr(msg, "validity_generation", 0) or 0),
                "map_revision": claimed_revision,
                "geometry_hash": str(record.get("geometry_hash") or "")},
                "v2"):
            return
        self._surface = self._surface_view_from_record(record)
        self._admitted_map_revision = claimed_revision

    def _surface_view_from_record(self, record):
        """Normalize one v2 surface record into the solver's internal view.

        The solver's internal representation stays floor-relative (support
        quantities are derived locally); the v2 container-origin datum is
        converted once, here, with the planner's declared inner size.
        """
        inner_l, inner_w, inner_h = self._inner_size
        floor = float(record["floor_z_container"])
        resolution = float(record["resolution_m"])
        heights, states, confidence = [], [], []
        inverse = {"sensor": "sensor", "verified_geometry": "geometry"}
        for row_z, row_s, row_c in zip(record["surface_z_container"],
                                       record["state"],
                                       record["confidence"]):
            h_row, s_row, c_row = [], [], []
            for top, state, source in zip(row_z, row_s, row_c):
                if top is None:
                    h_row.append(0.0)
                    s_row.append("unknown")
                    c_row.append("none")
                else:
                    h_row.append(float(top) - floor)
                    s_row.append(str(state))
                    c_row.append(inverse.get(str(source), "sensor"))
            heights.append(h_row)
            states.append(s_row)
            confidence.append(c_row)
        return {
            "geometry_hash": str(record["geometry_hash"]),
            "geometry_descriptor": dict(self._hull.descriptor()),
            "resolution": resolution,
            "nx": len(heights), "ny": len(heights[0]) if heights else 0,
            "inner_size": [inner_l, inner_w, inner_h],
            "floor_z": 0.0,
            "center_base": [0.0, 0.0, floor + 0.5 * inner_h],
            "yaw": 0.0,
            # The admitted revision travels with the view: ComputePlacementV2
            # compares requests against the ACTIVE surface's revision — a
            # conversion that dropped it would read 0 and stale-reject every
            # request planned against the very map this node just admitted.
            "map_revision": int(record["map_revision"]),
            "height": heights,
            "state": states,
            "confidence": confidence,
        }

    def _slot_record(self, candidate, request_id):
        """Slot-profile record (container box center) for one candidate."""
        center = candidate["center_container"]
        yaw = float(candidate["box_yaw"])
        return {
            "contract_version": 2,
            "frame_id": "container_link",
            "length_unit": "m",
            "stamp_ns": 0,
            "time_basis": "scene",
            "scene_revision": self._frame_scene_revision,
            "request_id": str(request_id),
            "kind": "pose",
            "reference": "box_center",
            "orientation_semantics": "box_axis_pi",
            "position": [float(center[0]), float(center[1]),
                         float(center[2])],
            "quaternion_xyzw": [0.0, 0.0,
                                math.sin(yaw * 0.5), math.cos(yaw * 0.5)],
        }

    def _slot_message(self, candidate, request_id):
        record = self._slot_record(candidate, request_id)
        context = BoundaryContext(
            "F07", "placement_planner", "v2_client", str(request_id),
            self._frame_scene_revision, "container_link", "box_center",
            "scene", geometry_hash=self._geometry_hash, map_revision=None)
        return pose_to_message(record, "slot", context), record

    def _placed_aabbs_v2(self, placed_messages, sizes_flat):
        """Floor-relative AABBs from v2 placed slot records."""
        aabbs = []
        inner_h = self._inner_size[2]
        sizes = [sizes_flat[i:i + 3] for i in range(0, len(sizes_flat), 3)]
        for index, message in enumerate(placed_messages or []):
            context = BoundaryContext(
                "F07", "v2_client", "placement_planner",
                str(message.identity.request_id or "placed-%d" % index),
                self._frame_scene_revision, "container_link", "box_center",
                "scene", geometry_hash=self._geometry_hash,
                map_revision=None)
            try:
                record = pose_from_message(message, "slot", context)
            except ContractError as error:
                raise ValueError("placed[%d] rejected (%s %s)"
                                 % (index, error.code, error.field))
            w, d, h = (float(v) for v in sizes[index]) if index < len(sizes) \
                else (0.0, 0.0, 0.0)
            if w <= 0.0 or d <= 0.0 or h <= 0.0:
                raise ValueError("placed[%d] needs positive wdh" % index)
            yaw = 2.0 * math.atan2(record["quaternion_xyzw"][2],
                                    record["quaternion_xyzw"][3])
            rotated = abs(abs(yaw) - math.pi / 2.0) < math.radians(10.0)
            footprint_l = d if rotated else w
            footprint_w = w if rotated else d
            z_rel = float(record["position"][2]) - self._floor_z
            x, y = float(record["position"][0]), float(record["position"][1])
            aabbs.append((
                x - footprint_l * 0.5, y - footprint_w * 0.5,
                max(0.0, z_rel - h * 0.5),
                x + footprint_l * 0.5, y + footprint_w * 0.5,
                min(inner_h, z_rel + h * 0.5),
            ))
        return aabbs

    def _on_committed(self, message):
        """Latch the mapper's committed placed ledger (single source)."""
        try:
            payload = json.loads(message.data)
            boxes = list(payload.get("boxes") or [])
            self._committed = {
                "boxes": boxes,
                "map_revision": int(payload.get("map_revision") or 0),
                "geometry_hash": str(payload.get("geometry_hash") or ""),
                "epoch": str(payload.get("epoch") or ""),
                "content_digest": str(payload.get("content_digest") or ""),
            }
        except (ValueError, TypeError):  # malformed publication: keep last
            self.get_logger().warning(
                "committed ledger publication unparsable; keeping previous")

    def _ledger_aabbs(self, committed):
        """Floor-relative AABBs from the mapper's committed boxes."""
        aabbs = []
        inner_h = self._inner_size[2]
        for box in committed["boxes"]:
            center = [float(v) for v in box["center"]]
            w, d, h = (float(v) for v in box["size"])
            yaw = float(box.get("yaw") or 0.0)
            rotated = abs(abs(yaw) - math.pi / 2.0) < math.radians(10.0)
            footprint_l = d if rotated else w
            footprint_w = w if rotated else d
            z_rel = center[2] - self._floor_z
            aabbs.append((
                center[0] - footprint_l * 0.5, center[1] - footprint_w * 0.5,
                max(0.0, z_rel - h * 0.5),
                center[0] + footprint_l * 0.5, center[1] + footprint_w * 0.5,
                min(inner_h, z_rel + h * 0.5),
            ))
        return aabbs

    def _placed_matches_ledger(self, placed_messages, sizes_flat, committed,
                                tolerance_m=0.10):
        """Subset agreement between the request-carried placed copy and
        the mapper's committed ledger.

        An EMPTY or PARTIAL request copy is "the driver defers to the
        ledger" (the single-source direction; the pre-B2 second compute
        inside the select path never carried placed and the map's
        columns are the planning truth). What refuses is a copy that
        CLAIMS a box the ledger does not hold (the witness_02 phantom
        class) or disagrees geometrically.
        """
        sizes = [sizes_flat[i:i + 3] for i in range(0, len(sizes_flat), 3)]
        if not placed_messages:
            return True, ""
        if len(placed_messages) > len(committed["boxes"]):
            return False, "count %d > ledger %d" % (
                len(placed_messages), len(committed["boxes"]))
        ledger = [([float(v) for v in box["center"]],
                   [float(v) for v in box["size"]])
                  for box in committed["boxes"]]
        for index, message in enumerate(placed_messages):
            context = BoundaryContext(
                "F07", "v2_client", "placement_planner",
                str(getattr(message.identity, "request_id", "")
                    or "placed-%d" % index),
                self._frame_scene_revision, "container_link", "box_center",
                "scene", geometry_hash=self._geometry_hash,
                map_revision=None)
            try:
                record = pose_from_message(message, "slot", context)
            except ContractError:
                return False, "placed[%d] undecodable" % index
            position = [float(v) for v in record["position"]]
            size = ([float(v) for v in sizes[index]]
                    if index < len(sizes) else [0.0, 0.0, 0.0])
            best = None
            for center, box_size in ledger:
                if any(abs(a - b) > 0.05 for a, b in zip(size, box_size)):
                    continue
                delta = math.sqrt(sum(
                    (a - b) ** 2 for a, b in zip(position, center)))
                if best is None or delta < best:
                    best = delta
            if best is None or best > tolerance_m:
                return False, "placed[%d] %.3f m from any ledger box" % (
                    index, best if best is not None else -1.0)
        return True, ""

    def _handle_v2(self, request, response):
        """ComputePlacementV2: container box-center slot semantics."""
        response.geometry_hash = self._geometry_hash
        self._v2_request_seq = getattr(self, "_v2_request_seq", 0) + 1
        request_id = "place-v2-%d" % self._v2_request_seq
        if getattr(self, "_runtime", None) is not None:
            from luggage_description.frame_contracts import ContractError
            try:
                admission = self._runtime.admit_request(
                    request_id, endpoint="/placement_planner/compute_placement_v2")
            except ContractError as error:
                response.success = False
                response.reason_code = error.code
                response.message = "%s: %s" % (error.code, error.field)
                return response
        # Reset-contract admission check BEFORE planning (§3), shared with
        # the v1 handler: gate closed -> no new computations. The admission
        # token travels with this request to its commit check.
        # Single-lease inputs BEFORE the admission gate (user ruling
        # 2026-11-08): in observer mode the surface can ONLY come from
        # the fresh lease (the topic surface cannot carry
        # validity_generation — ReadSurface's identity has no such
        # field — so topic maps fail the generation binding). The lease
        # surface carries the stamp's generation, passes the SAME
        # admission gate here, and the gate/commit checks below still
        # bind the solve to the token captured after this point.
        volume_verifier = None
        try:
            surface, volume_verifier = self._compute_inputs()
        except Exception as error:  # noqa: BLE001 - fail closed
            code = self._evidence_unavailable_code()
            if "CARGO_MAP_GEOMETRY_MISMATCH" in str(error):
                # GEO-8 attribution survives the snapshot path too.
                code = "CARGO_MAP_GEOMETRY_MISMATCH"
            response.success = False
            response.reason_code = code
            response.message = "%s: %s" % (code, error)
            return response
        gate, planning_admission = self._validity_admit()
        if gate is not None:
            response.success = False
            response.reason_code = "VALIDITY_GATE_CLOSED"
            response.message = gate
            return response
        self.behavior_hook("after_admit", "token=%s" % (
            planning_admission.seq if planning_admission is not None
            else "?"))
        self.behavior_hook("before_solve", "")
        if not bool(request.height_measured):
            response.success = False
            response.reason_code = CODE_DETECT_FULL_GEOMETRY_REQUIRED
            response.message = ("DETECT_FULL_GEOMETRY_REQUIRED: V2 needs "
                                "declared measured geometry")
            return response
        requested_hash = str(request.geometry_hash or "")
        if requested_hash and requested_hash != self._geometry_hash:
            response.success = False
            response.reason_code = CODE_CARGO_MAP_GEOMETRY_MISMATCH
            response.message = ("%s: request pinned %s, planner hull is %s"
                                % (CODE_CARGO_MAP_GEOMETRY_MISMATCH,
                                   requested_hash, self._geometry_hash))
            return response
        # Map-identity gate before any solver call: a request planned
        # against a different map revision is stale, whatever the solver
        # would find on the current one. The floor prior carries revision
        # 0; a first-box request against no admitted map matches 0.
        # Single-lease mode (volume_evidence_source=observer): the
        # request's map IS the fresh lease snapshot (surface + volume
        # evidence from one revision, fail-closed, no mixed mode).
        active_revision = int(surface.get("map_revision") or 0)
        requested_revision = int(request.expected_map_revision or 0)
        if requested_revision != active_revision:
            response.success = False
            response.reason_code = CODE_STALE_MAP_REVISION
            response.resulting_map_revision = active_revision
            response.message = (
                "%s: request planned against map revision %d, planner map "
                "is revision %d" % (CODE_STALE_MAP_REVISION,
                                    requested_revision, active_revision))
            return response
        size = [max(0.0, float(v)) for v in request.box_wdh]
        if any(v <= 0.0 for v in size):
            response.success = False
            response.reason_code = "INVALID_BOX_SIZE"
            response.message = "box_wdh must be positive"
            return response
        try:
            # [B2 single-source] Once the mapper's committed ledger is
            # latched, the request-carried placed copy is VALIDATED against
            # it and the solver input comes from the ledger — never from
            # the driver's local bookkeeping. Before the first committed
            # publication arrives, the request copy is the only source
            # (startup fail-open, unchanged legacy behavior).
            committed = getattr(self, "_committed", None)
            if committed is not None:
                ok, detail = self._placed_matches_ledger(
                    list(request.placed), list(request.placed_wdh_flat),
                    committed)
                if not ok:
                    response.success = False
                    response.reason_code = CODE_PLACED_LEDGER_MISMATCH
                    response.message = (
                        "PLACED_LEDGER_MISMATCH: %s (mapper rev %d)"
                        % (detail, committed["map_revision"]))
                    return response
                placed_aabbs = self._ledger_aabbs(committed)
            else:
                placed_aabbs = self._placed_aabbs_v2(
                    list(request.placed), list(request.placed_wdh_flat))
        except ValueError as error:
            response.success = False
            response.reason_code = "PLACED_SLOT_REJECTED"
            response.message = str(error)
            return response
        result = solve_placement(
            surface, size,
            allowed_yaws=self._allowed_yaws,
            params=self._params,
            candidate_validator=lambda candidate: self._constraint_verdict(
                candidate, placed_aabbs, size),
            atlas_annotate=self._atlas_hook(size),
            volume_verifier=volume_verifier)
        response.resulting_map_revision = int(
            surface.get("map_revision") or 0)
        if not result["success"]:
            response.success = False
            response.reason_code = result["reason_code"]
            response.message = result["message"]
            return response
        best = result.get("preferred_candidate", result["selected"])
        if getattr(self, "_runtime", None) is not None:
            # Effect boundary: the solver ran asynchronously under the
            # admission; recheck before committing the slot to the caller.
            from luggage_description.frame_contracts import ContractError
            try:
                self._runtime.authorize_effect(admission, request_id)
            except ContractError as error:
                response.success = False
                response.reason_code = error.code
                response.message = "%s: %s" % (error.code, error.field)
                return response
        # Reset-contract commit check (§3) before the slot is committed:
        # against the admission captured before computing, never a re-read
        # of the board's current generation.
        self.behavior_hook("before_commit", "")
        commit = self._validity_commit(planning_admission)
        if commit is not None:
            response.success = False
            response.reason_code = "VALIDITY_COMMIT_REJECTED"
            response.message = commit
            return response
        moved = self._placed_ledger_recheck(best, size)
        if moved is not None:
            response.success = False
            response.reason_code = CODE_PLACED_LEDGER_MOVED
            response.message = (
                "%s: a box landed in the committed ledger during planning "
                "(%s); re-request against the moved map" % (
                    CODE_PLACED_LEDGER_MOVED, moved))
            return response
        self._admission.record_execution()
        slot_message, _record = self._slot_message(best, request_id)
        response.slot = slot_message
        response.success = True
        response.reason_code = ""
        want = int(request.max_candidates or 0)
        if want <= 0:
            want = int(self.get_parameter("max_candidates").value)
        topk = ranked_feasible_candidates(result, want)
        n_feasible = sum(
            1 for candidate in (result.get("candidates") or [])
            if candidate.get("feasible"))
        response.message = "%s %s" % (
            result["message"], candidate_prefix_note(topk, n_feasible))
        response.candidates = []
        response.candidate_scores = []
        response.candidate_reasons = []
        for candidate in topk:
            message, _record = self._slot_message(candidate, request_id)
            response.candidates.append(message)
            response.candidate_scores.append(float(candidate.get("score", 0.0)))
            response.candidate_reasons.append(str(candidate.get("reason", "")))
        return response

    def _atlas_for(self, size):
        """Cached (atlas, diagnostics) for one requested box size."""
        if not self._atlas_cfg.get("enabled"):
            return None, {"loaded": False, "reason": "disabled"}
        if self._load_runtime_atlas is None:
            return None, {"loaded": False, "reason": "unavailable"}
        if self._urdf_identity is not None:
            self._urdf_identity.poll()
        urdf_hash = (
            self._urdf_identity.hash
            if self._urdf_identity is not None and self._urdf_identity.ready
            else "")
        key = (tuple(round(float(v), 4) for v in size),
               self._reachability_scene_hash, self._scene_tf_hash, urdf_hash)
        if key not in self._atlas_cache:
            self._atlas_cache[key] = self._load_runtime_atlas(
                self._atlas_cfg, box_size=list(key[0]),
                scene_tf_hash=self._scene_tf_hash,
                urdf_hash=urdf_hash,
                reachability_scene_hash=self._reachability_scene_hash)
        return self._atlas_cache[key]

    def _atlas_hook(self, size):
        """solve_placement hook for this request, or None when disabled."""
        if not self._atlas_cfg.get("enabled"):
            return None
        atlas, _diag = self._atlas_for(size)
        # Query-layer semantics (floor re-anchoring, boundary promotion):
        # the hook queries the exact contact point, so it is the side that
        # inherits the iz1 floor false-negative without this wrap.
        if self._apply_query_semantics is not None:
            atlas = self._apply_query_semantics(
                atlas, self._atlas_cfg, floor_z=self._floor_z,
                box_height=float(size[2]))
        placement = self._atlas_cfg.get("placement") or {}
        # A neutral no_atlas hook still annotates candidates for the dump.
        hook = make_atlas_annotate(
            atlas,
            self._floor_z,
            self._inner_size[2],
            score_weight=float(placement.get("w_reachability", 0.3)),
            hard_reject=bool(placement.get("hard_reject", True)),
            dump_only=bool(placement.get("dump_only", False)),
            query_offset=self._atlas_cfg.get("query_offset"),
        )
        return hook

    # ------------------------------------------------------------------
    # Feasibility gates (G2 + corridor)

    def _placed_aabbs(self, placed_slots):
        """Floor-relative container AABBs from elfin_base_link SlotSpecs.

        [FRAME MAP] input poses are elfin_base_link (the v1 wire
        contract); each converts through the FROZEN scene transform into
        container_link, then z becomes floor-relative (container z -
        scene inner floor 0.53). Output AABBs are exactly the frame the
        solver compares candidates in."""
        
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

    def _placed_ledger_recheck(self, best, size):
        """[A6] Re-verify the SELECTED candidate against the LATEST
        latched committed ledger at commit time.

        The compute-time clearance ran against a ledger snapshot; a box
        that landed in the window between compute and commit must void
        the plan (user ruling 2026-11-09: re-run on the latest ledger).
        Returns None when the plan still clears (or no v2 committed
        latch exists — v1 deployments have nothing newer than the
        request copy), else the collision reason."""
        committed = getattr(self, "_committed", None)
        if committed is None or not best:
            return None
        reason, _capacity = self._constraint_verdict(
            best, self._ledger_aabbs(committed), size)
        if reason in (REASON_OVERLAP, REASON_SAME_LAYER_CLEARANCE):
            return reason
        return None

    def _constraint_verdict(self, candidate, placed_aabbs, smallest_size):
        """Return ``(reason, capacity_ok)`` for one candidate."""
        hull_margin = float(self.get_parameter("hull_margin").value)
        if hull_margin < 0.0:
            # auto: one map cell — resolved ONCE per compute in
            # _compute_inputs (self._constraint_margin); fall back exact
            hull_margin = float(
                getattr(self, "_constraint_margin", 0.0) or 0.0)
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
            smallest_size=smallest_size,
            hull=hull,
            floor_z=self._floor_z,
            same_layer_clearance_m=float(
                self._params["same_layer_clearance_m"]),
            support_tol=float(self._params.get("support_tol", 0.05)),
        )

    def _structural_result(self, surface, size, placed_aabbs):
        """Structural-first chain (G3): session batch + atlas + snug rank.

        Same hard-gate semantics as the exhaustive chain (the session
        reuses the solver's own window evaluation, plus this node's
        constraint validator for placed boxes / aperture / corridor);
        ordering is ``structural_snug_depth_v1``. Route-failure
        continuation is the caller's: ask the session ``next_batch`` --
        here the first batch is served and returned with diagnostics.
        """
        import time as _time
        from luggage_packing.placement_search_session import (
            PlacementSearchSession)
        from luggage_packing.placement_snug_ranking import snug_rank

        stages = {}
        t0 = _time.perf_counter()
        session = PlacementSearchSession(
            surface, size, self._allowed_yaws, params=self._params,
            budget_s=float(self.get_parameter("structural_budget_s").value),
            candidate_validator=lambda candidate: self._constraint_verdict(
                candidate, placed_aabbs, size))
        batch = session.start()
        stages["search_core_s"] = round(_time.perf_counter() - t0, 4)
        candidates = list(batch["candidates"])

        t_atlas = _time.perf_counter()
        atlas_report = None
        hook = self._atlas_hook(size)
        if hook is not None:
            candidates, atlas_report = hook(candidates)
        stages["atlas_s"] = round(_time.perf_counter() - t_atlas, 4)

        feasible = [c for c in candidates if c.get("feasible")]
        success = bool(feasible)
        return {
            "success": success,
            "candidates": candidates,
            "selected": feasible[0] if feasible else None,
            "preferred_candidate": feasible[0] if feasible else None,
            "reason_code": "" if success else "PLACE_CANDIDATE_EXHAUSTED",
            "message": ("structural first batch: %d served (%s)"
                        % (batch["served_total"], batch["source"])),
            "capacity_feasible_count": len(feasible),
            "reject_histogram": {},
            "atlas": atlas_report,
            "structural": {
                "batch_source": batch["source"],
                "batch_index": batch["batch_index"],
                "coverage_complete": batch["coverage_complete"],
                "budget_exhausted": batch["budget_exhausted"],
                "terminal_reason": batch["terminal_reason"],
                "proposals": batch["proposals"],
                "setup_s": batch["setup_s"],
                "stages": stages,
                "session_identity": session.identity(),
            },
        }

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
        if getattr(self, "_runtime", None) is not None:
            # v1 cannot bypass profile admission: readiness still holds here.
            from luggage_description.frame_contracts import ContractError
            try:
                self._runtime.authorize_legacy_effect()
            except ContractError as error:
                response.success = False
                response.reason_code = error.code
                response.message = "%s: %s" % (error.code, error.field)
                return response
        # Reset-contract admission check BEFORE planning (§3): the gate is
        # closed until a fresh VALID map and a post-transition robot-state
        # sample both exist; a delayed old response never re-opens it. The
        # admission token travels with this request to its commit check.
        # Single-lease inputs BEFORE the admission gate (user ruling
        # 2026-11-08): in observer mode the surface can ONLY come from
        # the fresh lease (the topic surface cannot carry
        # validity_generation — ReadSurface's identity has no such
        # field — so topic maps fail the generation binding). The lease
        # surface carries the stamp's generation, passes the SAME
        # admission gate here, and the gate/commit checks below still
        # bind the solve to the token captured after this point.
        volume_verifier = None
        try:
            surface, volume_verifier = self._compute_inputs()
        except Exception as error:  # noqa: BLE001 - fail closed
            code = self._evidence_unavailable_code()
            if "CARGO_MAP_GEOMETRY_MISMATCH" in str(error):
                # GEO-8 attribution survives the snapshot path too.
                code = "CARGO_MAP_GEOMETRY_MISMATCH"
            response.success = False
            response.reason_code = code
            response.message = "%s: %s" % (code, error)
            return response
        if (self._surface is None and self._geometry_rejected_maps
                and str(self.get_parameter(
                    "volume_evidence_source").value or "") != "observer"):
            # A live mapper published a hull we refused. Answering from the
            # floor prior here would be the silent fallback GEO-8 forbids.
            # ONLY geometry rejections trip this (run15 K04/R10: a producer
            # boot/reset legitimately emits validity_building maps and
            # stale revisions — those rejections must answer in the
            # validity vocabulary, never as a foreign-hull refusal).
            # Single-lease mode (observer): the request's map is the FRESH
            # LEASE surface (self._surface tracks only topic maps), which
            # carries its own fail-closed chain — the admission gate above
            # bound it; a rejected topic map is a representational gap
            # (ReadSurface identity cannot carry validity_generation),
            # not a foreign-hull fallback.
            response.success = False
            response.reason_code = CODE_CARGO_MAP_GEOMETRY_MISMATCH
            response.message = (
                "%s: %d cargo map(s) rejected, planner hull is %s"
                % (CODE_CARGO_MAP_GEOMETRY_MISMATCH,
                   self._geometry_rejected_maps,
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
        gate, planning_admission = self._validity_admit()
        if gate is not None:
            response.success = False
            response.reason_code = "VALIDITY_GATE_CLOSED"
            response.message = gate
            self._publish_last({
                "success": False,
                "reason_code": "VALIDITY_GATE_CLOSED",
                "message": gate,
                "geometry_hash": self._geometry_hash,
                "candidates": [],
                "execution_active": bool(
                    self._admission.executing_generations),
            })
            return response
        size = [max(0.0, float(box.width)),
                max(0.0, float(box.depth)),
                max(0.0, float(box.height))]
        atlas, atlas_diag = self._atlas_for(size)
        # 2026-10-15: the identity block reason is gone with the full-hash
        # verification; only a semantics refusal (v1 artifact) still fails
        # the request by name.
        block = str((atlas_diag or {}).get("semantics") or "")
        if block:
            response.success = False
            response.reason_code = block
            response.message = "%s: %s" % (
                block,
                atlas_diag.get("hash_message") or atlas_diag.get("reason")
                or "atlas identity unverified")
            self._publish_last({
                "success": False,
                "size_wdh": size,
                "n_candidates_total": 0,
                "n_feasible": 0,
                "reject_histogram": {},
                "reason_code": response.reason_code,
                "geometry_hash": self._geometry_hash,
                "floor_prior": False,
                "candidates": [],
                "message": response.message,
                "atlas": {
                    "enabled": True,
                    "diagnostics": atlas_diag,
                },
            })
            return response
        placed_aabbs = self._placed_aabbs(request.placed)
        atlas, atlas_diag = self._atlas_for(size)
        if bool(self.get_parameter("structural_search").value):
            result = self._structural_result(
                surface, size, placed_aabbs)
        else:
            result = solve_placement(
                surface, size,
                allowed_yaws=self._allowed_yaws,
                params=self._params,
                candidate_validator=lambda candidate: self._constraint_verdict(
                    candidate, placed_aabbs, size),
                atlas_annotate=self._atlas_hook(size),
                volume_verifier=volume_verifier,
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
            # ST09-A S1 4.3: what a bounded observation stage (ST09-D) must
            # still see — unknown cells in the selected footprint and entry
            # corridor, or of the best hypothetical legal window when no
            # candidate survived (OBSERVATION_REQUIRED).
            "observation_requirement": result.get("observation_requirement"),
            "hypothetical_legal_windows": result.get(
                "hypothetical_legal_windows", []),
            "atlas": {
                "enabled": bool(self._atlas_cfg.get("enabled")),
                "diagnostics": atlas_diag,
                "report": result.get("atlas"),
            },
            "structural": result.get("structural"),
            "candidates": [jsonable_candidate(self._annotate_frames(dict(c)))
                           for c in candidates],
        }

        if not result["success"]:
            response.slot = SlotSpec()
            response.success = False
            response.reason_code = result["reason_code"]
            response.message = result["message"]
            response.candidates = []
            response.candidate_scores = []
            response.candidate_reasons = []
            dump["message"] = response.message
            self._publish_last(dump)
            return response
        self.behavior_hook("after_admit", "token=%s" % (
            planning_admission.seq if planning_admission is not None
            else "?"))
        self.behavior_hook("before_solve", "")
        # E0/E4 contract: packing needs measured full geometry. A top-only
        # detection must not become a collision box inside the container.
        # Catalog size is spawn-only and is never a substitute for height.
        source = int(getattr(box, "height_source", 0) or 0)
        if (not bool(getattr(box, "height_valid", False))
                or source != 1):
            response.success = False
            response.reason_code = CODE_DETECT_FULL_GEOMETRY_REQUIRED
            response.message = (
                "DETECT_FULL_GEOMETRY_REQUIRED: ComputePlacement needs "
                "height_source=MEASURED_SUPPORT; got height_valid=%s "
                "height_source=%d" % (
                    bool(getattr(box, "height_valid", False)), source))
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


        best = result.get("preferred_candidate", result["selected"])
        # Reset-contract commit check (§3): the second epoch/generation look
        # before a plan becomes a valid placement result. A reset that won
        # while planning discards the candidate here — checked against the
        # admission captured before computing, never the board's current
        # generation (gen1 defect, review 2026-11-03).
        self.behavior_hook("before_commit", "")
        commit = self._validity_commit(planning_admission)
        if commit is not None:
            response.success = False
            response.reason_code = "VALIDITY_COMMIT_REJECTED"
            response.message = commit
            dump["success"] = False
            dump["reason_code"] = response.reason_code
            dump["message"] = commit
            dump["candidates"] = []
            dump["execution_active"] = bool(
                self._admission.executing_generations)
            self._publish_last(dump)
            return response
        moved = self._placed_ledger_recheck(best, size)
        if moved is not None:
            response.success = False
            response.reason_code = CODE_PLACED_LEDGER_MOVED
            response.message = (
                "%s: a box landed in the committed ledger during planning "
                "(%s); re-request against the moved map" % (
                    CODE_PLACED_LEDGER_MOVED, moved))
            dump["success"] = False
            dump["reason_code"] = response.reason_code
            dump["message"] = response.message
            dump["candidates"] = []
            self._publish_last(dump)
            return response
        slot, annotated = self._slot_from_candidate(best, size)
        # The plan crossed the commit check: record its generation so a later
        # invalidation reports execution_active instead of silent cancel.
        self._admission.record_execution()
        response.slot = slot
        response.success = True
        response.reason_code = ""
        response.message = result["message"]
        want = int(getattr(request, "max_candidates", 0) or 0)
        if want <= 0:
            want = int(self.get_parameter("max_candidates").value)
        topk = ranked_feasible_candidates(result, want)
        n_feasible = sum(
            1 for candidate in (result.get("candidates") or [])
            if candidate.get("feasible"))
        response.message = "%s %s" % (
            result["message"], candidate_prefix_note(topk, n_feasible))
        response.candidates = []
        response.candidate_scores = []
        response.candidate_reasons = []
        for candidate in topk:
            slot_i, _ann = self._slot_from_candidate(candidate, size)
            response.candidates.append(slot_i)
            response.candidate_scores.append(float(candidate.get("score", 0.0)))
            response.candidate_reasons.append(str(candidate.get("reason") or "ok"))
        dump["message"] = response.message
        dump["validity"] = {
            "state": self._admission.validity.state,
            "epoch": self._admission.validity.epoch,
            "generation": self._admission.validity.generation,
        }
        dump["execution_active"] = bool(
            self._admission.executing_generations)
        # Packing preference only. The certified slot is written by
        # SelectPlacePathV2 after a complete route succeeds.
        dump["preferred_candidate"] = jsonable_candidate(annotated)
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
