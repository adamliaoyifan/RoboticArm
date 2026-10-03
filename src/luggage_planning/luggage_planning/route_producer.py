"""ST09-I: the single admitted route producer behind SelectPlacePathV2.

Composition only — this module owns no planning, certification, or
measurement policy of its own:

* ``PlanningSnapshot`` from the admitted surface contract (D:
  ``planning_snapshot_from_surface``).
* ``AttachmentRecord`` frozen by the measured provider (D:
  ``MeasuredAttachmentProvider``) from the admitted same-stamp measured
  request fields (``register_attach_from_request``).
* The directed q-state route (B: ``GraphRoutePlanner`` with the MoveIt
  edge builder) under the frozen budgets.
* Authoritative certification (C: ``TrajectoryCertifier`` fed the real
  MoveIt validity adapter from ``moveit_state_validity``) with the
  frozen fail-closed certification timeout, then pre-send map
  revalidation (C: ``MapRevalidator``).

The output is one complete ``joint_path`` leg record per certified route
edge (``leg_records``), plus the per-candidate evaluation rows. There is
no fallback planner here: a candidate that cannot produce a certified
route under budget is recorded with its named reason and the next
candidate is tried; exhaustion is ``PLANNING_BUDGET_EXHAUSTED`` /
``NO_FEASIBLE_ROUTE``, never ``BIN_FULL``.

Region anchors reuse the frozen waypoint clearances
(``waypoint_generator``) as *graph node generators only* — they seed
portal/approach/insert/exit nodes for the edge builder and never become
execution specifications. Production never imports
``st09_contracts.fixtures`` and never constructs a
``CertificationResult`` here; certification comes from C.
"""

from __future__ import division

import dataclasses
import math
import time
from typing import List, Mapping, Optional, Sequence, Tuple

from st09_contracts._base import ContractViolation
from st09_contracts.attachment import UNLOADED_ATTACHMENT_DIGEST
from st09_contracts.graph import LoadState, PlannedRoute
from st09_contracts.planning import GeometryVerdict, PlacementCandidate

from luggage_planning.grid_path_search import (
    DEFAULT_ENTRY_HOPS,
    ROUTE_TOTAL_WALL_S,
    SINGLE_NEW_EDGE_WALL_S,
    RoutePlanningError,
    hops_from_table,
)
#: [ST09-I S5.7-4] ObservationRouteV2 guards.
OBSERVATION_TARGET_LIMIT = 2
CAMERA_LINK = "camera_depth_optical_frame"

_CAMERA_TCP_CACHE = {}


def camera_to_tcp_transform(urdf):
    """Rigid camera->TCP transform from the (world-patched) offline URDF.

    [ST09-I S5.7-4] The camera->TCP conversion is RouteProducer work.
    Computed at two distinct configurations and compared -- a camera
    bracket that moved independently of the arm fails closed (None).
    """
    if urdf is None:
        return None
    cached = _CAMERA_TCP_CACHE.get(id(urdf))
    if cached is not None:
        return cached
    import numpy as np

    camera_path = urdf.path_to(CAMERA_LINK)
    tcp_path = list(getattr(urdf, "tcp_path", None)
                    or ("elfin_joint1", "elfin_joint2", "elfin_joint3",
                        "elfin_joint4", "elfin_joint5", "elfin_joint6",
                        "suction_contact_frame"))
    if not camera_path or not tcp_path:
        return None

    def _at(joints):
        t_base_camera = urdf.tcp(joints, camera_path)
        t_base_tcp = urdf.tcp(joints, tcp_path)
        return np.linalg.inv(t_base_camera) @ t_base_tcp

    first = _at([0.0] * 6)
    second = _at([0.4, -0.3, 0.2, -0.1, 0.5, 0.25])
    if not np.allclose(first, second, atol=1e-9):
        return None
    from scipy.spatial.transform import Rotation as _Rot

    transform = (
        tuple(float(v) for v in first[:3, 3]),
        tuple(float(v) for v in _Rot.from_matrix(first[:3, :3]).as_quat()))
    _CAMERA_TCP_CACHE[id(urdf)] = transform
    return transform


from luggage_planning.waypoint_generator import (
    DEFAULT_CORRIDOR_MARGIN,
    DEFAULT_PLACE_CLEARANCE_Z,
    corridor_clearance,
    insertion_clearance,
)
from luggage_description.frame_transforms import rotate_vector

CANONICAL_JOINTS = tuple("elfin_joint%d" % i for i in range(1, 7))

#: Route edge classes -> the canonical leg names the place cycle's
#: state/release hooks expect (descend ends the pre-release chain;
#: retreat/return run after release).
LEG_NAMES = {
    "PORTAL_TRANSIT": "transit",
    "TRAVERSE": "traverse",
    "INSERT": "insert",
    "DESCEND": "descend",
    "RETREAT": "retreat",
    "RETURN": "return",
}

#: Frozen route wall budget for one select request, nanoseconds.
ROUTE_TOTAL_WALL_NS = int(ROUTE_TOTAL_WALL_S * 1_000_000_000)

#: Frozen certifier wall budget (resolved_planning_contract.json
#: planning_budgets.certification_timeout_s, defined_by_c0).
CERTIFICATION_TIMEOUT_S = 2.0

#: Budget share reserved for the certification + revalidation tail so a
#: route planned at t=9.9s cannot enter certification without time left.
CERTIFICATION_RESERVE_S = CERTIFICATION_TIMEOUT_S + 0.5

#: Extra lift of the post-release exit anchor above the traverse height
#: (anchor seeding detail only — the certified retreat edge decides the
#: real motion; declared here, not a frozen contract value).
EXIT_EXTRA_LIFT_M = 0.10

#: Declared release gap ([ST09-I] release policy, 2026-10-02): the
#: certified descend ends with the payload bottom this far ABOVE the
#: support surface; release is vacuum-off rigid free fall (Gazebo
#: physics; the mock chain records the analytic landed pose). The
#: certified trajectory is therefore contact-free end to end, and the
#: gap absorbs perception height error up to itself — the payload can
#: never be pushed through the support. Landed quality is measured
#: post-release (err_xy, perceived-vs-actual z), never excused in
#: certification. Declared and recorded in evidence, not tuned silently.
RELEASE_DROP_M = 0.05

#: [ST09-I P3 ordering contract] EXECUTABLE-FIRST. Inside each atlas
#: bucket (seeded -> reachable -> unknown) candidates are ordered by
#: distance-to-opening ASCENDING (projected on the opening's inward
#: normal) — an explicit geometric key, independent of placement's sort.
#: Audit basis (2026-10-03, two-box mock): with placement's deep-first
#: order primary, each deep seeded candidate consumed its whole
#: per-candidate slice (~4 s) failing goal refinement at the reach
#: limit, and the frozen 10 s route budget expired after 2-3 attempts —
#: deep-first enumeration cannot reach a certifiable slot under this
#: contract. COST: near-first selection can occupy opening-adjacent
#: cells; the THREE-BOX CAPACITY ACCEPTANCE at M3 (S6) must verify the
#: carryon -> standard -> carryon sequence still fits with this
#: ordering. If capacity fails, the contract decision is the user's:
#: raise the frozen budget or restore deep-first with pre-validated
#: graphs.
EXECUTABLE_FIRST_ORDER = True

#: IK seed prior for the tool-down branch when the atlas is unavailable
#: and the live start configuration is degenerate (mock zero state).
#: A seed is never a verdict: the bounded MoveIt supplementation and the
#: certifier decide reachability.
TOOL_DOWN_SEED_Q = (0.936, -1.025, 1.268, -1.819, -1.567, -0.635)


def revalidation_policy_from_contract(contract: Mapping):
    """RevalidationPolicy with the FROZEN snapshot freshness window.

    C's 2 s dataclass default is a development stand-in; the resolved
    contract's ``map_freshness.planning_snapshot_age_max_s`` (10 s,
    defined_by_c0) is the authority at route admission.
    """
    from luggage_planning.map_revalidation import RevalidationPolicy

    freshness = (contract.get("map_freshness") or {}).get(
        "planning_snapshot_age_max_s") or {}
    age_s = float(freshness.get("value", 10.0)
                  if isinstance(freshness, Mapping) else freshness)
    return RevalidationPolicy(max_snapshot_age_ns=int(age_s * 1e9))


def certification_policy_from_contract(contract: Mapping):
    """CertificationPolicy with the FROZEN joint bounds.

    C's dataclass defaults are a narrower development stand-in; the
    resolved contract's ``joint_limits.position_rad`` is the authority
    (j2 [-3.3158, 0.1745] etc.), so production builds the policy from
    the packaged contract document.
    """
    from luggage_planning.trajectory_certifier import CertificationPolicy

    limits = (contract.get("joint_limits") or {}).get("position_rad") or {}
    lower, upper = [], []
    for index in range(1, 7):
        bounds = limits.get("elfin_joint%d" % index)
        if not bounds or len(bounds) != 2:
            raise ValueError(
                "resolved contract missing elfin_joint%d position_rad"
                % index)
        lower.append(float(bounds[0]))
        upper.append(float(bounds[1]))
    return CertificationPolicy(
        joint_names=tuple("elfin_joint%d" % i for i in range(1, 7)),
        joint_lower=tuple(lower), joint_upper=tuple(upper))


def _named_reason(exc):
    """``ContractViolation`` messages are ``REASON: detail``; keep the name."""
    text = str(exc)
    return text.split(":", 1)[0].strip() if text else "ATTACHMENT_MISSING"


def surface_map_from_contract(surface: Mapping, request: Mapping) -> dict:
    """Bridge the admitted v2 surface record to the D snapshot input.

    ``planning_snapshot_from_surface`` consumes the mapper's
    ``surface_map_2d`` dict; the select request carries the same grid as
    a ``SurfaceContractV2`` record plus the request-level
    geometry/revision the contract pins. Identity comes from the
    request: the latched surface correlates as its own map revision, not
    as this select request.
    """
    state = surface["state"]
    rows = len(state)
    columns = len(state[0]) if rows else 0
    return {
        "frame": str(surface.get("frame_id") or "container_local"),
        "resolution": float(surface["resolution_m"]),
        "origin_local": [float(v) for v in surface["origin_xy_container"]],
        "nx": rows,
        "ny": columns,
        "state": [list(row) for row in state],
        "acquisition_stamp_ns": int(surface.get("stamp_ns") or 0),
        "commit_revision": int(surface.get("commit_revision")
                               or request["map_revision"]),
        "geometry_hash": str(request["geometry_hash"]),
        "map_revision": int(request["map_revision"]),
        "inner_size": [float(v) for v in request["inner_size"]],
    }


def _yaw(quaternion_xyzw) -> float:
    qx, qy, qz, qw = (float(v) for v in quaternion_xyzw)
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


def candidate_from_slot(slot: Mapping, index: int) -> PlacementCandidate:
    """Minimal frozen-candidate view of one admitted slot record.

    The ordering was frozen by A when the slot list was produced — the
    producer keeps request order. This view carries identity + geometry
    so the route planner can synthesize targets; ``depth_m`` is filled
    by the producer when the opening geometry is known.
    """
    return PlacementCandidate(
        candidate_id=str(slot.get("request_id") or ("slot-%d" % index)),
        footprint_class=0,
        heading_theta=_yaw(slot["quaternion_xyzw"]),
        position_container=tuple(float(v) for v in slot["position"]),
        quaternion_xyzw_container=tuple(
            float(v) for v in slot["quaternion_xyzw"]),
        support_layer_z=0.0,
        depth_m=0.0,
        verdict=GeometryVerdict.LEGAL,
        score=0.0,
        detail={},
    )


@dataclasses.dataclass(frozen=True)
class RegionAnchorPlan(object):
    """World-frame TCP anchor poses seeding the route graph regions."""

    portal: Tuple[float, float, float]
    slot_approach: Tuple[float, float, float]
    slot_upper: Tuple[float, float, float]
    slot: Tuple[float, float, float]
    exit: Tuple[float, float, float]
    depth_m: float
    #: Open-air carrying anchor above the pedestal line (None when the
    #: scene declares no transit clearance and the single-hop transit
    #: stays). Seeded above the portal so the payload clears the
    #: pedestal on the approach and only descends past its footprint.
    staging: "Tuple[float, float, float] | None" = None


#: Payload-bottom margin kept above the transit clearance (pedestal top)
#: when seeding the staging anchor.
STAGING_MARGIN_M = 0.10

#: Staging pullback along the opening's outward normal: the anchor sits
#: in FREE space off the container face, so the open-air sweep never
#: threads the aperture and only the final portal descent enters it.
STAGING_PULLBACK_M = 0.35

#: Bounded route rebuilds after a CERT_COLLISION verdict (fresh OMPL
#: sampling inside the frozen budget; the certifier is still the only
#: authority — a rebuild that still collides is still a refusal).
CERT_COLLISION_REBUILDS = 2


def anchor_plan(contact_world: Sequence[float], container_quaternion,
                box_height_m: float, *, corridor_surface_max=None,
                place_clearance_z=DEFAULT_PLACE_CLEARANCE_Z,
                opening=None, inward_normal=None,
                corridor_margin=DEFAULT_CORRIDOR_MARGIN,
                transit_clearance_z=None) -> RegionAnchorPlan:
    """Anchor geometry for one candidate, world frame.

    ``contact_world`` is the candidate's world TCP contact pose
    (pure-synthesis target); the plan derives the vertical ladder
    (slot -> slot_upper -> slot_approach) and the post-release exit from
    the frozen clearances, plus the insertion depth along the opening
    inward normal when the scene provides one. Without opening data the
    portal collapses onto the approach pose, which keeps the region
    topology intact for tight scenes.
    """
    container_quaternion = tuple(float(v) for v in container_quaternion)

    def lift(length):
        delta = rotate_vector(container_quaternion, (0.0, 0.0, float(length)))
        return (contact_world[0] + delta[0], contact_world[1] + delta[1],
                contact_world[2] + delta[2])

    height = max(0.0, float(box_height_m))
    clearance = corridor_clearance(
        corridor_surface_max, height, float(contact_world[2]),
        float(place_clearance_z), margin=corridor_margin)
    insert_len = insertion_clearance(height, clearance)
    slot = lift(0.0)
    slot_upper = lift(insert_len)
    slot_approach = lift(clearance)
    exit_pose = lift(clearance + EXIT_EXTRA_LIFT_M)
    depth = 0.0
    portal = slot_approach
    if opening is not None and inward_normal is not None:
        point = [float(v) for v in opening]
        normal = [float(v) for v in inward_normal]
        # depth of the contact along the inward normal from the opening
        depth = sum((slot[i] - point[i]) * normal[i] for i in range(3))
        offset = sum(
            (slot_approach[i] - point[i]) * normal[i] for i in range(3))
        portal = tuple(
            slot_approach[i] + normal[i] * (0.15 - offset)
            for i in range(3))
    staging = None
    if transit_clearance_z is not None and opening is not None \
            and inward_normal is not None:
        # The staging anchor sits on the OPENING CENTERLINE, pulled back
        # along the outward normal into free space and lifted so the
        # payload bottom clears the transit floor (pedestal top). Aiming
        # the open-air sweep at the aperture center keeps the wrist rig
        # and the carried payload inside the aperture's clearance on the
        # final descent (a staging on the slot's own y line let OMPL
        # wander off-axis and the certified payload grazed the container
        # face corner); it also never drags the payload across the
        # pedestal — a one-shot home->portal joint plan had to rediscover
        # all of that in 6-DOF and kept failing validation live.
        staging_z = max(
            float(portal[2]),
            float(transit_clearance_z) + float(height) + STAGING_MARGIN_M)
        if staging_z > float(portal[2]) + 0.05:
            outward = [-float(v) for v in inward_normal]
            staging = tuple(
                float(opening[i]) + outward[i] * STAGING_PULLBACK_M
                for i in range(3))
            staging = (staging[0], staging[1], staging_z)
    return RegionAnchorPlan(
        portal=portal, slot_approach=slot_approach, slot_upper=slot_upper,
        slot=slot, exit=exit_pose, depth_m=float(depth), staging=staging)


class RouteProducer(object):
    """Compose snapshot + attachment + route + certification for select.

    All collaborators are injected: ``graph`` and ``edge_builder`` (B),
    ``certifier`` + ``validity_adapter_factory`` + ``revalidator`` (C),
    ``attachment_provider`` (D), ``fk`` / ``refine`` / ``urdf`` (offline
    IK). Deterministic given the injected clock.
    """

    def __init__(self, *, graph, edge_builder, certifier,
                 validity_adapter_factory, attachment_provider,
                 revalidator=None, latest_snapshot_fn=None,
                 fk, refine=None, urdf=None,
                 tcp_path=("elfin_joint1", "elfin_joint2", "elfin_joint3",
                           "elfin_joint4", "elfin_joint5", "elfin_joint6",
                           "suction_contact_frame"),
                 cert_policy=None, budget_ns=ROUTE_TOTAL_WALL_NS,
                 clock_ns=None, corridor_surface_max=None,
                 opening_info=None,
                 transit_clearance_z=None, log=None,
                 atlas_prior=None):
        from luggage_planning.trajectory_certifier import CertificationPolicy

        self.graph = graph
        self.edge_builder = edge_builder
        self.certifier = certifier
        self.validity_adapter_factory = validity_adapter_factory
        self.attachment_provider = attachment_provider
        self.revalidator = revalidator
        #: Fresh-map fetch at admission time (ST09-I release/map
        #: discipline): callable -> latest PlanningSnapshot, or None to
        #: fail the revalidation closed. When unset the planned snapshot
        #: is reused (development wiring; admission then never sees a
        #: map that changed after planning).
        self.latest_snapshot_fn = latest_snapshot_fn
        self.fk = fk
        self.refine = refine
        self.urdf = urdf
        self.tcp_path = tuple(tcp_path)
        self.cert_policy = cert_policy or CertificationPolicy()
        self.budget_ns = int(budget_ns)
        self.corridor_surface_max = corridor_surface_max
        self.opening_info = opening_info
        #: Data-driven reachability prior replacing the removed dev
        #: 1.35 m ball: callable(position_container,
        #: quaternion_xyzw_container) -> {"verdict":
        #: "unreachable"|"reachable"|"unknown", "seeds": [q, ...]} or
        #: None (no atlas loaded). The producer's OWN iteration order
        #: becomes reachable-first, unknown next; "unreachable" (the
        #: atlas SAMPLED the cell and both pi-equivalent branches have
        #: no solution) is dropped with a named row before any
        #: IK/MoveIt budget. Unknown/miss still proceeds — atlas miss
        #: is unknown, never unreachable — and the returned contact
        #: seeds seed the goal IK (the atlas' own sampled solutions).
        #: The placement list's frozen deep-first order is untouched;
        #: MoveIt and certification remain the authority for everything
        #: that runs.
        self.atlas_prior = atlas_prior
        #: World Z the carried payload bottom must stay above during
        #: open-air transit (the pedestal top). None disables the staging
        #: anchor — the entry table then keeps its single-hop transit.
        self.transit_clearance_z = (None if transit_clearance_z is None
                                    else float(transit_clearance_z))
        self._log = log or (lambda message: None)
        self._clock_ns = clock_ns or time.monotonic_ns
        self.last_route = None
        self.last_certification = None
        self.last_revalidation = None
        self.last_budget_snapshot = None
        self.last_rows = None
        #: [ST09-I S5.7] The last admitted selection, for pre-execution
        #: re-verification (certify A, execute A — never a second path).
        self.last_selection = None

    # -- helpers ---------------------------------------------------------

    def _attachment(self, request, now_ns):
        """Latest valid measured record for this request, or a named reason."""
        provider = self.attachment_provider
        expire = getattr(provider, "expire_stale", None)
        if expire is not None:
            expire(str(request["request_id"]), now_ns)
        try:
            return provider.latest_record(str(request["request_id"])), None
        except (ContractViolation, LookupError, ValueError) as exc:
            return None, _named_reason(exc)

    def _seed_candidates(self, start_q):
        seeds = [tuple(float(v) for v in start_q), tuple(TOOL_DOWN_SEED_Q)]
        return list(dict.fromkeys(seeds))

    def _refine_to(self, position, orientation, seed_q,
                   prefer_tool_down=False):
        if self.refine is None:
            return None, {"reason": "no_refine"}
        seeds = self._seed_candidates(seed_q)
        if prefer_tool_down and len(seeds) > 1:
            seeds = [seeds[1], seeds[0]]
        info = {}
        for seed in seeds:
            try:
                q, info = self.refine(tuple(position), tuple(orientation),
                                      list(seed))
                if q is not None:
                    return tuple(q), info
            except Exception:  # noqa: BLE001 - anchor seeding is best effort
                info = {"reason": "ik_unavailable"}
        return None, info or {"reason": "ik_unavailable"}

    def _box_target(self, candidate, attachment, request):
        from luggage_planning.place_targets import box_target_from_candidate

        transform = request["container_transform"]
        return box_target_from_candidate(
            candidate, attachment,
            stamp_ns=attachment.acquisition_stamp_ns,
            t_frame_container=transform["translation"],
            q_frame_container=transform["quaternion_xyzw"])

    def _prepare_candidate(self, base_candidate, attachment, request,
                           start_q, tcp_orientation, tcp_position):
        """Depth-annotate the candidate and seed the region ladder.

        Anchor nodes are graph NODES ONLY: their poses seed the portal /
        approach / insert / exit regions the bounded search routes
        through; the certified edges decide the actual motion. The
        anchor orientation is the candidate's synthesized TCP target
        orientation (``T_world_box * inverse(T_tcp_box)`` — tool-down
        for a suction grasp), so carry edges keep one transport
        orientation.
        """
        box_target = self._box_target(base_candidate, attachment, request)
        orientation = tuple(float(v) for v in tcp_orientation)
        # The ladder hangs off the synthesized TCP contact (the first
        # expanded target), NOT the box centre: the box centre would sit
        # every anchor a half-payload low and drive the carried box into
        # the floor on the insert/descend edges.
        plan = anchor_plan(
            tuple(float(v) for v in tcp_position),
            request["container_transform"]["quaternion_xyzw"],
            float(request["box_wdh"][2]),
            corridor_surface_max=self.corridor_surface_max,
            opening=(self.opening_info or {}).get("point"),
            inward_normal=self._inward_normal(),
            transit_clearance_z=self.transit_clearance_z)
        candidate = dataclasses.replace(base_candidate, depth_m=plan.depth_m)

        graph = self.graph
        seed_q = tuple(float(v) for v in start_q)
        graph.register_node(seed_q, self.fk(seed_q)[0], orientation,
                            "home", LoadState.CARRYING, "start")
        graph.register_node(seed_q, self.fk(seed_q)[0], orientation,
                            "home", LoadState.UNLOADED, "start")
        seeded = []
        has_opening = (self.opening_info or {}).get("point") is not None
        anchor_rows = [
            ("portal", plan.portal, LoadState.CARRYING),
            ("slot_approach", plan.slot_approach, LoadState.CARRYING),
            ("slot_upper", plan.slot_upper, LoadState.CARRYING),
            ("exit", plan.exit, LoadState.UNLOADED),
        ]
        if plan.staging is not None:
            anchor_rows.insert(0, ("staging", plan.staging,
                                   LoadState.CARRYING))
        if not has_opening:
            # The collapsed portal shares the approach pose; registering
            # it would steal the approach node's (load, q) key.
            anchor_rows = [row for row in anchor_rows
                           if row[0] != "portal"]
        for region, position, load_state in anchor_rows:
            # Register BOTH refined branches (tool-down prior first, then
            # the live-start branch): a single anchor q carries no
            # collision awareness, and the per-hop search needs the
            # alternative when one branch's interpolated edge collides.
            for seed in self._seed_candidates(seed_q):
                q, _info = self._refine_to(position, orientation, seed)
                if q is not None:
                    graph.register_node(q, position, orientation, region,
                                        load_state, "anchor")
            seeded.append(region)
        return candidate, box_target, plan, seeded

    def _inward_normal(self):
        info = self.opening_info or {}
        normal = info.get("normal")
        if not normal:
            return None
        # The scene normal points outward; the route travels inward.
        return [-float(v) for v in normal]

    # -- main entry ------------------------------------------------------

    def plan_place_route(self, request: Mapping) -> dict:
        """Plan, certify, and revalidate the carried route for one select."""
        from luggage_perception.cargo_view_integration import (
            planning_snapshot_from_surface,
        )
        from luggage_planning.reachable_graph import GraphRoutePlanner

        started_ns = self._clock_ns()
        rows: List[dict] = []
        snapshot = planning_snapshot_from_surface(
            surface_map_from_contract(request["surface"], request),
            scene_revision=str(request["scene_revision"]),
            request_id=str(request["request_id"]))
        attachment, attachment_reason = self._attachment(
            request, self._clock_ns())
        if attachment is None:
            return self._fail(attachment_reason, rows)

        start_q = tuple(float(v) for v in request["start_joints"])
        last_reason = "NO_FEASIBLE_ROUTE"
        purge = getattr(self.graph, "clear_request_scoped", None)
        if callable(purge):
            purge()
        # [ST09-I P3] atlas-informed iteration order: the producer tries
        # atlas-reachable candidates first, unknown next, and drops only
        # SAMPLED-unreachable cells with a named row (placement's frozen
        # deep-first response order is a packing policy, not a planning
        # one — without this, unreachable-deep rows burn the whole
        # route budget before a reachable slot is ever examined).
        ordered_slots = []
        if self.atlas_prior is not None:
            seeded_order, reachable_order, unknown_order = [], [], []
            for index, slot in enumerate(request["slots"]):
                prior_candidate = candidate_from_slot(slot, index)
                # The prior is queried at the REAL TCP targets (the
                # measured attachment composed in by _expand_targets),
                # never at the raw slot box pose.
                from luggage_planning.place_targets import (
                    TargetSynthesisError,
                )

                try:
                    prior_targets = self._expand_targets(
                        prior_candidate, attachment, request)
                except (ContractViolation, TargetSynthesisError):
                    prior_targets = []
                    rows.append({
                        "slot_index": index,
                        "candidate_id": prior_candidate.candidate_id,
                        "depth_m": 0.0,
                        "reason": "TARGET_SYNTHESIS_EMPTY",
                    })
                    continue
                if not prior_targets:
                    rows.append({
                        "slot_index": index,
                        "candidate_id": prior_candidate.candidate_id,
                        "depth_m": 0.0,
                        "reason": "TARGET_SYNTHESIS_EMPTY",
                    })
                    continue
                prior = self.atlas_prior(prior_targets, request) or {}
                if prior.get("verdict") == "unreachable":
                    rows.append({
                        "slot_index": index,
                        "candidate_id": prior_candidate.candidate_id,
                        "depth_m": 0.0,
                        "reason": "ATLAS_UNREACHABLE",
                    })
                    continue
                entry = (index, slot,
                         list(prior.get("seeds") or []))
                if entry[2]:
                    seeded_order.append(entry)
                elif prior.get("verdict") == "reachable":
                    reachable_order.append(entry)
                else:
                    unknown_order.append(entry)
            # [ST09-I P3 ordering contract] The atlas buckets
            # (seeded -> reachable -> unknown) are each ordered by an
            # EXPLICIT geometric key: distance-to-opening ascending
            # (EXECUTABLE_FIRST_ORDER). This is not a reversal of
            # placement's deep-first list — it is an independent key,
            # frozen above with its audit basis and its capacity cost
            # (three-box acceptance at M3). Only SAMPLED-unreachable
            # candidates are dropped (named rows); budget fairness comes
            # from the per-candidate planning slice below.
            inward = self._inward_normal()
            opening_point = (self.opening_info or {}).get("point")

            def _depth_key(entry):
                # depth ascending; ties broken by DESCENDING placement
                # index — the shallower end of placement's deep-first
                # list wins ties, matching the audited passing order
                # (reverse of the deep-first list).
                index, slot, _seeds = entry
                position = slot["position"]
                if not inward or not opening_point:
                    return (-float(index),)
                depth = sum(
                    (float(position[axis]) - float(opening_point[axis]))
                    * float(inward[axis]) for axis in range(3))
                return (depth, -float(index))

            if EXECUTABLE_FIRST_ORDER:
                seeded_order.sort(key=_depth_key)
                reachable_order.sort(key=_depth_key)
                unknown_order.sort(key=_depth_key)
            ordered_slots = (seeded_order + reachable_order
                             + unknown_order)
            self._log("route: atlas order seeded=%d reachable=%d "
                      "unknown=%d dropped=%d "
                      "executable_first=%s" % (
                          len(seeded_order), len(reachable_order),
                          len(unknown_order), len(rows),
                          EXECUTABLE_FIRST_ORDER))
            if not ordered_slots:
                return self._fail("NO_FEASIBLE_ROUTE", rows)
        else:
            ordered_slots = [
                (index, slot, []) for index, slot
                in enumerate(request["slots"])]
        self._log("route: select %s slots=%d hash=%s" % (
            request["request_id"], len(request["slots"]),
            str(request.get("geometry_hash") or "")[:12]))
        for index, slot, atlas_seeds in ordered_slots:
            if index and index % 100 == 0:
                self._log("route: slot %d/%d %.1fs last=%s" % (
                    index, len(request["slots"]),
                    (self._clock_ns() - started_ns) / 1e9, last_reason))
            base_candidate = candidate_from_slot(slot, index)
            elapsed = self._clock_ns() - started_ns
            remaining = self.budget_ns - elapsed
            if remaining <= int(CERTIFICATION_RESERVE_S * 1e9):
                return self._fail("PLANNING_BUDGET_EXHAUSTED", rows)
            # [ST09-I P3 ordering contract] NO per-candidate slice:
            # a measured COMPLETE route build costs ~6-10 s (staging
            # OMPL transit dominates) — most of the frozen 10 s budget —
            # so slicing to guarantee a second attempt starves the only
            # viable one (live: 6 s and 4.6 s slices each killed a build
            # that succeeds at ~7-9 s). Fairness against deep-attempt
            # starvation comes from the EXECUTABLE_FIRST ordering (the
            # first tried candidate is the cheapest, most robust one);
            # if that attempt fails, PLANNING_BUDGET_EXHAUSTED is the
            # honest named outcome. Budget growth or pre-validated
            # persisted graphs are the levers for multi-attempt budgets.
            candidate_budget_ns = int(remaining)
            cert_rebuilds_left = CERT_COLLISION_REBUILDS
            row = {
                "slot_index": index,
                "candidate_id": base_candidate.candidate_id,
                "depth_m": 0.0,
                "reason": "",
            }
            # [ST09-I P3] the dev 1.35 m base-radius prior is REMOVED:
            # reachability filtering belongs to the placement atlas
            # (rank + confident hard-reject) upstream and to IK/MoveIt/
            # certification here — a distance ball around the base both
            # rejected legal deep slots the arm can genuinely reach and
            # passed unreachable ones it cannot.
            try:
                prior_targets = self._expand_targets(
                    base_candidate, attachment, request)
            except ContractViolation as exc:
                row["reason"] = _named_reason(exc)
                rows.append(row)
                last_reason = row["reason"]
                continue
            candidate, _box_target, _plan, _seeded = self._prepare_candidate(
                base_candidate, attachment, request, start_q,
                tcp_orientation=prior_targets[0].quaternion_xyzw,
                tcp_position=prior_targets[0].position)
            self._log("route: anchors %s home %s" % (
                [(n.region, [round(v, 3) for v in n.q])
                 for n in self.graph.nodes.values()
                 if n.region in ("portal", "slot_approach", "slot_upper")],
                [round(v, 3) for v in start_q]))
            row["candidate_id"] = candidate.candidate_id
            row["depth_m"] = candidate.depth_m
            try:
                # Without an opening datum the portal collapses onto the
                # approach anchor, so the portal hop is dropped rather
                # than seeded with a duplicated node.
                has_opening = (self.opening_info or {}).get("point") \
                    is not None
                entry_table = DEFAULT_ENTRY_HOPS if has_opening else tuple(
                    hop for hop in DEFAULT_ENTRY_HOPS
                    if hop[0] != "portal")
                has_staging = bool(
                    self.graph.nodes_by_region("staging"))
                if has_staging:
                    # Staging transit: an OMPL joint sweep to the high
                    # free-space anchor (a single cartesian hop across
                    # the workspace sits in the wrist-singularity family
                    # and stops at fraction 0), then the deterministic
                    # cartesian descent that enters the aperture.
                    entry_table = (
                        ("staging", "PORTAL_TRANSIT"),
                        ("portal", "TRAVERSE"),
                    ) + tuple(hop for hop in entry_table
                              if hop[0] != "portal")
                entry_hops = hops_from_table(entry_table, "carrying")
                planner = GraphRoutePlanner(
                    self.graph, self.edge_builder,
                    home_node_id=self.graph.node_for_q(
                        LoadState.CARRYING, start_q).node_id,
                    entry_hops=entry_hops,
                    t_frame_container=tuple(float(v) for v in
                                            request["container_transform"]
                                            ["translation"]),
                    q_frame_container=tuple(float(v) for v in
                                            request["container_transform"]
                                            ["quaternion_xyzw"]),
                    refine=self.refine, urdf=self.urdf,
                    goal_seeds=self._goal_seed_fn(
                        start_q,
                        branch_q=next(
                            (node.q for node in reversed(
                                self.graph.nodes_by_region("slot_upper"))),
                            None),
                        atlas_seeds=atlas_seeds),
                    tcp_path=self.tcp_path)
                targets = self._expand_targets(
                    candidate, attachment, request)
                if not targets:
                    row["reason"] = "TARGET_SYNTHESIS_EMPTY"
                    rows.append(row)
                    last_reason = row["reason"]
                    continue
                self._log("route: planning candidate %d (%.2fs left)"
                          % (index, remaining / 1e9))
                route = planner.plan_route(
                    snapshot, attachment, candidate, targets,
                    budget_ns=candidate_budget_ns)
                self._log("route: candidate %d planned %s"
                          % (index, route.route_id))
            except RoutePlanningError as exc:
                import json as _json
                row["targets"] = _json.dumps(
                    [[list(t.position), list(t.quaternion_xyzw)]
                     for t in targets], default=float)[:2000]
                row["reason"] = exc.code
                row["detail"] = exc.detail
                rows.append(row)
                last_reason = exc.code
                # [ST09-I] bounded sampler retry: NO_FEASIBLE after an
                # OMPL miss (a staging transit dying at -2 when the next
                # draw resolves) is sampling luck, not geometry. Re-plan
                # the SAME candidate with fresh draws inside the frozen
                # budget; geometry refusals still refuse.
                remaining = self.budget_ns - (self._clock_ns() - started_ns)
                route = None
                if exc.code == "NO_FEASIBLE_ROUTE" \
                        and cert_rebuilds_left > 0 and remaining > 0:
                    cert_rebuilds_left -= 1
                    self._log("route: sampler miss — re-planning "
                              "candidate %d" % index)
                    try:
                        route = planner.plan_route(
                            snapshot, attachment, candidate, targets,
                            budget_ns=int(remaining))
                    except RoutePlanningError:
                        route = None
                if route is None:
                    continue
                row = {
                    "slot_index": index,
                    "candidate_id": base_candidate.candidate_id,
                    "depth_m": 0.0,
                    "reason": "SAMPLER_RETRY",
                }
                self._log("route: candidate %d re-planned %s"
                          % (index, route.route_id))
            except ContractViolation as exc:
                row["reason"] = _named_reason(exc)
                rows.append(row)
                last_reason = row["reason"]
                continue
            except RoutePlanningError as exc:
                row["reason"] = exc.code
                row["detail"] = exc.detail
                dumps = getattr(self.edge_builder, "request_dumps", None)
                if callable(dumps):
                    row["moveit_dumps"] = list(dumps())[-6:]
                failures = getattr(self.edge_builder, "edge_failure_log",
                                   None)
                if callable(failures):
                    row["moveit_failures"] = list(failures())[-8:]
                rows.append(row)
                last_reason = exc.code
                self._log("route: candidate %d failed %s" % (index, exc.code))
                continue
            except Exception as exc:  # noqa: BLE001 - service boundary
                # Edge-builder scene/transport failures are named
                # per-candidate failures, never a crashed service.
                row["reason"] = "EDGE_BUILD_FAILED"
                row["detail"] = {"error": repr(exc)[:300]}
                rows.append(row)
                last_reason = row["reason"]
                self._log("route: candidate %d error %r" % (index, exc))
                continue
            self.last_budget_snapshot = planner.last_budget_snapshot

            row["route_id"] = route.route_id
            while True:
                edges = tuple(
                    self.graph.edges[edge_id] for edge_id in
                    route.entry_edge_ids + route.retreat_edge_ids)
                self._log("route: certifying %d edges" % len(edges))
                certification, certify_failure = self._certify(
                    route, snapshot, attachment, edges, candidate, request)
                self._log("route: certification %s" % (
                    "pass" if certification is not None
                    and certification.passed else "fail"))
                if certification is not None and certification.passed:
                    break
                reason = certify_failure if certification is None \
                    else certification.reason.value
                if certification is not None:
                    row["cert_detail"] = {
                        "min_clearance_m": certification.min_clearance_m,
                        "contacts": [
                            dict(contact) if isinstance(contact, Mapping)
                            else str(contact)
                            for contact in (
                                certification.contact_reports or ())][:6],
                    }
                    self._log("route: cert fail %s clearance=%s" % (
                        certification.reason.value,
                        certification.min_clearance_m))
                # A certified collision after a clean OMPL build is a
                # between-waypoint graze the sampler's own resolution
                # missed (live: the mid360 mount grazing the container
                # corner by 0.45 mm). OMPL is a sampler — re-plan the
                # route with fresh sampling, bounded, inside the frozen
                # budget. The certifier stays the only authority: a
                # rebuild that still collides is still a refusal.
                remaining = self.budget_ns - (self._clock_ns() - started_ns)
                if reason == "CERT_COLLISION" and cert_rebuilds_left > 0 \
                        and remaining > 0:
                    cert_rebuilds_left -= 1
                    self._log("route: cert collision — re-planning "
                              "candidate %d" % index)
                    try:
                        route = planner.plan_route(
                            snapshot, attachment, candidate, targets,
                            budget_ns=int(remaining))
                        row["route_id"] = route.route_id
                        continue
                    except RoutePlanningError:
                        pass
                row["reason"] = reason
                rows.append(row)
                last_reason = row["reason"]
                break
            if row.get("reason"):
                continue
            revalidation = self._revalidate(route, snapshot, edges)
            if revalidation is None:
                row["reason"] = "MAP_REVALIDATION_UNAVAILABLE"
                rows.append(row)
                last_reason = row["reason"]
                continue
            from st09_contracts.certification import MapDecision

            row["decision"] = getattr(revalidation.decision, "value",
                                      str(revalidation.decision))
            if revalidation.decision is not MapDecision.CONTINUE:
                row["reason"] = getattr(
                    revalidation.reason, "value", "MAP_REVISION_MISMATCH")
                rows.append(row)
                last_reason = row["reason"]
                continue
            legs = leg_records(route, edges, request)
            row["reason"] = "OK"
            row["certified"] = True
            rows.append(row)
            self.last_route = route
            self.last_certification = certification
            self.last_revalidation = revalidation
            self.last_rows = rows
            self.last_selection = {
                "path_hash": route.digest(),
                "route": route,
                "edges": edges,
                "snapshot": snapshot,
                "attachment": attachment,
                "candidate": candidate,
                "request": request,
                "certification_id": certification.digest(),
            }
            return {
                "success": True,
                "reason_code": "OK",
                "slot_index": index,
                "selection_id": route.route_id,
                "path_source": "route-graph",
                "path_hash": route.digest(),
                "rows": rows,
                "route": route,
                "edges": edges,
                "legs": legs,
                "certification": certification,
                "revalidation": revalidation,
                "graph_digest": self.graph.digest(),
                "budget_snapshot": self.last_budget_snapshot,
                "elapsed_s": (self._clock_ns() - started_ns) / 1e9,
            }
        return self._fail(last_reason, rows)

    def verify_selected(self, *, route_digest, current_joints,
                        latest_map_revision, scene_revision):
        """[ST09-I S5.7] Pre-execution re-verification of the selected route.

        The probe never generates a second path. Identity gates first
        (digest known, scene revision match, 2*pi-wrapped start-state
        match at the executor's own 0.01 rad gate), map admission via
        the revalidator, then a FRESH full re-certification of the exact
        certified edges against a live scene read. Returns
        ``(ok, reason, detail)``; ``detail["leg_digests"]`` lets the
        caller prove what it holds is bit-for-bit what was certified.
        """
        selection = getattr(self, "last_selection", None)
        if not selection or selection.get("path_hash") != str(route_digest):
            return False, "DIGEST_UNKNOWN", {
                "known": (selection or {}).get("path_hash", "")}
        route = selection["route"]
        edges = selection["edges"]
        snapshot = selection["snapshot"]
        if str(scene_revision) != str(
                getattr(snapshot, "scene_revision", "")):
            return False, "IDENTITY_CHANGED", {
                "scene": str(getattr(snapshot, "scene_revision", ""))}
        current = tuple(float(v) for v in (current_joints or ()))
        first = tuple(edges[0].trajectory.positions[0]) if edges else ()
        if not first or len(current) != len(first):
            return False, "START_STATE_MISMATCH", {
                "certified_start": list(first), "current": list(current)}

        def _wrap(delta):
            return (delta + math.pi) % (2.0 * math.pi) - math.pi

        delta = max(abs(_wrap(c - f)) for c, f in zip(current, first))
        if delta > 0.01:
            return False, "START_STATE_MISMATCH", {
                "delta_rad": round(delta, 6)}
        if type(latest_map_revision) is int \
                and latest_map_revision < int(snapshot.map_revision):
            return False, "MAP_RESTART", {
                "planned": int(snapshot.map_revision),
                "caller_latest": latest_map_revision}
        revalidation = self._revalidate(route, snapshot, edges)
        if revalidation is None:
            return False, "MAP_REVALIDATION_UNAVAILABLE", {}
        from st09_contracts.certification import MapDecision

        if revalidation.decision is not MapDecision.CONTINUE:
            return False, getattr(
                revalidation.reason, "value", "MAP_REVISION_MISMATCH"), {
                "map_revision": getattr(revalidation, "map_revision", 0)}
        certification, failure = self._certify(
            route, snapshot, selection["attachment"], edges,
            selection["candidate"], selection["request"],
            load_state=selection.get("load_state"))
        if certification is None or not certification.passed:
            reason = failure if certification is None \
                else certification.reason.value
            return False, reason, {"recertified": False}
        # [ST09-I S5.7-3b] the FRESH certificate is the live admission
        # the executor will check between legs.
        selection["certification_id"] = certification.digest()
        return True, "OK", {
            "start_state_delta_rad": round(delta, 6),
            "certification_id": certification.digest(),
            "validated_map_revision": int(snapshot.map_revision),
            "leg_digests": [edge.trajectory.digest() for edge in edges],
        }

    def execution_admission_is_live(self, *, route_digest,
                                    certification_id,
                                    admission_map_revision,
                                    scene_revision):
        """[ST09-I S5.7-3b] Is a certified execution admission STILL live?

        Certify A, execute A — AND the certification still holds when A
        executes. The selector owns the verdict; the executor only
        refuses on it. Peer restart loses the selection
        (ADMISSION_UNKNOWN); a superseded certificate or changed scene
        identity refuses; a map that advanced without touching the
        route's edges stays LIVE (new revision recorded); corridor
        impact or staleness revokes. Returns ``(live, reason, detail)``.
        """
        selection = getattr(self, "last_selection", None)
        if not selection or selection.get("path_hash") != str(route_digest):
            return False, "ADMISSION_UNKNOWN", {
                "known": (selection or {}).get("path_hash", "")}
        if str(certification_id) != str(
                selection.get("certification_id", "")):
            return False, "CERTIFICATE_SUPERSEDED", {
                "live": str(selection.get("certification_id", ""))}
        snapshot = selection["snapshot"]
        if str(scene_revision) != str(
                getattr(snapshot, "scene_revision", "")):
            return False, "GENERATION_CHANGED", {
                "scene": str(getattr(snapshot, "scene_revision", ""))}
        if (type(admission_map_revision) is int
                and admission_map_revision < int(snapshot.map_revision)):
            return False, "MAP_RESTART", {
                "planned": int(snapshot.map_revision),
                "admission": admission_map_revision}
        revalidation = self._revalidate(
            selection["route"], snapshot, selection["edges"])
        if revalidation is None:
            return False, "MAP_REVALIDATION_UNAVAILABLE", {}
        from st09_contracts.certification import MapDecision

        if revalidation.decision is not MapDecision.CONTINUE:
            return False, getattr(
                revalidation.reason, "value", "ADMISSION_REVOKED"), {
                "map_revision": getattr(revalidation, "map_revision", 0)}
        return True, "OK", {
            "live_map_revision": int(revalidation.map_revision)}

    def plan_observation_route(self, *, request_id, scene_revision,
                               current_joints, camera_targets,
                               latest_map_revision,
                               purpose="observation"):
        """[ST09-I S5.7-4 §4] Certified motion to observation poses.

        The driver + ObservationPolicy decide WHERE to look (world
        camera poses, at most two per request); this producer owns the
        camera->TCP conversion, the MoveIt edge, map admission, and the
        FULL certification under the explicit LoadState.UNLOADED
        admission (an absent attachment is never silently read as
        unloaded). Targets are tried in the policy's frozen order; the
        first one that builds AND certifies is the answer. Returns a
        dict mirroring plan_place_route's success/refusal shape.
        """
        from st09_contracts.attachment import compose_transform
        from st09_contracts.graph import LoadState, PlannedRoute

        targets = list(camera_targets or [])
        if not targets:
            return self._fail("OBSERVATION_TARGET_MISSING", [])
        if len(targets) > OBSERVATION_TARGET_LIMIT:
            return self._fail("OBSERVATION_TARGET_LIMIT",
                              [{"targets": len(targets)}])
        transform = camera_to_tcp_transform(self.urdf)
        if transform is None:
            return self._fail("OBSERVATION_CAMERA_FRAME_MISSING", [])
        current = tuple(float(v) for v in (current_joints or ()))
        if len(current) != 6:
            return self._fail("START_STATE_MISMATCH",
                              [{"joints": len(current)}])
        # [S5.7 lock] map admission is fail-closed on the LATEST map —
        # never the selection-time snapshot.
        fetch = getattr(self, "latest_snapshot_fn", None)
        snapshot = fetch() if callable(fetch) else None
        if snapshot is None:
            return self._fail("MAP_REVALIDATION_UNAVAILABLE", [])
        if str(scene_revision) != str(
                getattr(snapshot, "scene_revision", "")):
            return self._fail("IDENTITY_CHANGED",
                              [{"scene": str(getattr(
                                  snapshot, "scene_revision", ""))}])
        if type(latest_map_revision) is int:
            if latest_map_revision < int(snapshot.map_revision):
                return self._fail("MAP_RESTART",
                                  [{"planned": int(snapshot.map_revision),
                                    "caller_latest": latest_map_revision}])
            if latest_map_revision > int(snapshot.map_revision):
                return self._fail("MAP_STALE",
                                  [{"snapshot": int(snapshot.map_revision),
                                    "caller_latest": latest_map_revision}])
        request = {"request_id": str(request_id),
                   "scene_revision": str(scene_revision),
                   "start_tcp": {}}
        rows = []
        last_reason = "NO_FEASIBLE_ROUTE"
        reset_scene = getattr(self.edge_builder, "reset_scene", None)
        try:
            for index, camera_pose in enumerate(targets):
                row = {"target_index": index}
                tcp_position, tcp_quaternion = compose_transform(
                    tuple(float(v) for v in camera_pose[0]),
                    tuple(float(v) for v in camera_pose[1]),
                    tuple(float(v) for v in transform[0]),
                    tuple(float(v) for v in transform[1]))
                start_fk = self.fk(list(current))
                start_node = self.graph.register_node(
                    current, tuple(start_fk[0]), tuple(start_fk[1]),
                    "observation_home", LoadState.UNLOADED, "observation")
                q, _info = self._refine_to(
                    tcp_position, tcp_quaternion, list(current))
                if q is None:
                    row["reason"] = "IK_UNAVAILABLE"
                    rows.append(row)
                    last_reason = row["reason"]
                    continue
                goal_node = self.graph.register_node(
                    tuple(q), tuple(tcp_position), tuple(tcp_quaternion),
                    "observation_goal_%d" % index, LoadState.UNLOADED,
                    "observation")
                try:
                    edge = self.edge_builder(
                        start_node, goal_node, "PORTAL_TRANSIT")
                except Exception as error:  # noqa: BLE001 - named row
                    code = getattr(
                        getattr(error, "reason", None), "value", None) \
                        or "EDGE_BUILD_FAILED"
                    row["reason"] = code
                    row["detail"] = repr(error)[:200]
                    rows.append(row)
                    last_reason = code
                    continue
                edges = (edge,)
                route = PlannedRoute(
                    route_id="route-obs-%s-%d" % (str(request_id)[:12],
                                                  index),
                    request_id=str(request_id),
                    snapshot_digest=snapshot.digest(),
                    payload_digest=UNLOADED_ATTACHMENT_DIGEST,
                    attachment_digest=UNLOADED_ATTACHMENT_DIGEST,
                    candidate_id="observation-%s" % str(purpose),
                    entry_edge_ids=(edge.edge_id,),
                    retreat_edge_ids=(),
                    edge_digests=(edge.digest(),),
                    trajectory_digest=edge.trajectory.digest(),
                    planning_revision="observation")
                revalidation = self._revalidate(route, snapshot, edges)
                if revalidation is None:
                    row["reason"] = "MAP_REVALIDATION_UNAVAILABLE"
                    rows.append(row)
                    last_reason = row["reason"]
                    continue
                from st09_contracts.certification import MapDecision

                if revalidation.decision is not MapDecision.CONTINUE:
                    row["reason"] = getattr(
                        revalidation.reason, "value",
                        "MAP_REVISION_MISMATCH")
                    rows.append(row)
                    last_reason = row["reason"]
                    continue
                certification, failure = self._certify(
                    route, snapshot, None, edges, None, None,
                    load_state=LoadState.UNLOADED)
                if certification is None or not certification.passed:
                    row["reason"] = failure if certification is None \
                        else certification.reason.value
                    row["recertified"] = False
                    rows.append(row)
                    last_reason = row["reason"]
                    continue
                legs = leg_records(route, edges, request)
                row["reason"] = "OK"
                row["certified"] = True
                rows.append(row)
                self.last_route = route
                self.last_certification = certification
                self.last_revalidation = revalidation
                self.last_rows = rows
                self.last_selection = {
                    "path_hash": route.digest(),
                    "route": route,
                    "edges": edges,
                    "snapshot": snapshot,
                    "attachment": None,
                    "candidate": None,
                    "request": request,
                    "certification_id": certification.digest(),
                    "load_state": LoadState.UNLOADED,
                    "purpose": str(purpose),
                }
                return {
                    "success": True,
                    "reason_code": "OK",
                    "selection_id": route.route_id,
                    "path_source": "route-graph",
                    "path_hash": route.digest(),
                    "rows": rows,
                    "route": route,
                    "edges": edges,
                    "legs": legs,
                    "certification": certification,
                    "revalidation": revalidation,
                }
        finally:
            if callable(reset_scene):
                reset_scene()
        return self._fail(last_reason, rows)

    def _release_box_target(self, box_target, request):
        """Lift the box target to the release-gap pose along container up.

        The frozen target synthesis is untouched; this declared producer
        lift is what the descend edge actually tracks — payload bottom
        RELEASE_DROP_M above support at descend end, then vacuum-off
        free fall. Both the reach prior and the planner goals consume
        the lifted pose (the arm goes where the release happens).
        """
        import dataclasses

        delta = rotate_vector(
            tuple(float(v) for v in
                  request["container_transform"]["quaternion_xyzw"]),
            (0.0, 0.0, RELEASE_DROP_M))
        return dataclasses.replace(
            box_target,
            position=tuple(box_target.position[i] + delta[i]
                           for i in range(3)))

    def _expand_targets(self, candidate, attachment, request):
        from luggage_planning.place_targets import expand_place_targets

        box_target = self._release_box_target(
            self._box_target(candidate, attachment, request), request)
        return expand_place_targets(box_target, attachment)

    def _goal_seed_fn(self, start_q, branch_q=None, atlas_seeds=None):
        """Goal seeds from the offline refine (B requires injected seeds).

        Atlas contact seeds come first (the atlas' own sampled
        solutions for this cell — the best-informed branch), then the
        slot-upper anchor branch so the descend edge stays on one IK
        branch (a branch-crossing descend blows the frozen
        joint-excursion bound); the tool-down prior and the live-start
        branch follow as alternatives. A seed is never a verdict — the
        bounded supplementation and certifier decide.
        """
        from luggage_planning.place_targets import GoalSeed

        seeds_to_try = [("atlas", tuple(q)) for q in (atlas_seeds or [])]
        seeds_to_try += ([("branch", branch_q)] if branch_q is not None else [])
        seeds_to_try += [("tool_down", TOOL_DOWN_SEED_Q),
                         ("start", start_q)]

        def goal_seeds(target):
            seeds = []
            for label, seed in seeds_to_try:
                q, _info = self._refine_to(
                    target.position, target.quaternion_xyzw, seed)
                if q is not None:
                    seeds.append(GoalSeed(
                        tuple(q), "refine-%s" % label,
                        "route_producer:offline_refine"))
            return seeds
        return goal_seeds

    def edge_scene_states(self, route, edges, snapshot, attachment,
                          candidate, request):
        """Per-edge B ``SceneState`` for certification.

        Entry edges run carrying the measured payload; retreat edges run
        unloaded with the just-released box (at the candidate's world
        rest pose) and the snapshot's occupied cells as obstacles.
        UNKNOWN never becomes free — the boxes are the same coarse
        priors the edge builder planned against.
        """
        from luggage_planning.moveit_edge_adapter import (
            BoxSpec,
            PayloadSpec,
            SceneState,
            scene_boxes_from_snapshot,
        )
        from st09_contracts.attachment import compose_transform

        # [ST09-I S5.7-4] Unloaded-only routes (observation/pick/exit)
        # carry no AttachmentRecord and no placement candidate: no
        # carried payload, no just-released box — the snapshot's placed
        # boxes remain the obstacle set.
        payload = None
        if attachment is not None:
            payload = PayloadSpec(
                dimensions_wdh=tuple(
                    float(v) for v in attachment.dimensions_wdh),
                t_tcp_box_translation=tuple(
                    float(v) for v in attachment.t_tcp_box_translation),
                t_tcp_box_quaternion_xyzw=tuple(
                    float(v) for v in attachment.t_tcp_box_quaternion_xyzw))
        released = None
        container_transform = (request or {}).get("container_transform") \
            if candidate is not None else None
        if container_transform:
            released_center, released_quaternion = compose_transform(
                tuple(float(v) for v in
                      container_transform["translation"]),
                tuple(float(v) for v in
                      container_transform["quaternion_xyzw"]),
                tuple(float(v) for v in candidate.position_container),
                tuple(float(v) for v in
                      candidate.quaternion_xyzw_container))
            released = BoxSpec(
                object_id="route-released-box",
                center=tuple(float(v) for v in released_center),
                size_whd=tuple(
                    float(v) for v in attachment.dimensions_wdh),
                quaternion_xyzw=tuple(
                    float(v) for v in released_quaternion))
        states = {}
        for edge in edges:
            carrying = str(getattr(edge, "load_state", "")
                           or "").lower() == "carrying"
            states[str(edge.edge_id)] = SceneState(
                load_state="carrying" if carrying else "unloaded",
                payload=payload if carrying else None,
                released_box=None if carrying else released,
                placed_boxes=() if carrying else scene_boxes_from_snapshot(
                    snapshot))
        return states

    def _certify(self, route, snapshot, attachment, edges, candidate,
                 request, load_state=None):
        """Authoritative certification under the frozen fail-closed budget.

        Reasons stay DISTINCT: a named ContractViolation (service
        unavailable/timeout/scene/echo mismatch) surfaces its own code,
        an unexpected exception is CERT_SERVICE_EXCEPTION, and only a
        genuine wall-clock overrun is CERTIFICATION_TIMEOUT — never one
        blanket code for every failure ([ST09-I] reason-loss fix).
        Returns ``(result, failure_reason)``.
        """
        import dataclasses

        from st09_contracts._base import ContractViolation
        from luggage_planning.trajectory_certifier import (
            CertificationContext,
        )

        scene_states = self.edge_scene_states(
            route, edges, snapshot, attachment, candidate, request)
        adapter = self.validity_adapter_factory(
            scene_states=scene_states, edges=edges)
        policy = self.cert_policy
        from st09_contracts.graph import LoadState as _LoadState

        context = CertificationContext(
            route=route, snapshot=snapshot, attachment=attachment,
            edges=edges, validity_adapter=adapter,
            now_ns=self._clock_ns(), policy=policy,
            load_state=load_state or _LoadState.CARRYING)
        started_ns = self._clock_ns()
        try:
            result = self.certifier.certify(context)
        except ContractViolation as exc:
            reason = str(exc).split(":", 1)[0].strip() or \
                "CERT_INVALID_EVIDENCE"
            self._log("route: certify refused %s (%s)" % (
                reason, str(exc)[:300]))
            return None, reason
        except Exception as exc:  # noqa: BLE001 - fail closed, but NAMED
            import traceback

            self._log("route: certify exception %r %s" % (
                exc, traceback.format_exc(limit=4).replace("\n", " | ")))
            return None, "CERT_SERVICE_EXCEPTION"
        elapsed = (self._clock_ns() - started_ns) / 1e9
        self._log("route: certify wall %.3fs edges=%d" % (
            elapsed, len(edges)))
        if elapsed > CERTIFICATION_TIMEOUT_S:
            return None, "CERTIFICATION_TIMEOUT"
        return result, None

    def _revalidate(self, route, snapshot, edges):
        if self.revalidator is None:
            return None
        from luggage_planning.map_revalidation import RevalidationInput

        latest = snapshot
        fetch = self.latest_snapshot_fn
        if callable(fetch):
            fetched = fetch()
            if fetched is None:
                return None  # named MAP_REVALIDATION_UNAVAILABLE upstream
            latest = fetched
        return self.revalidator.revalidate(RevalidationInput(
            route=route, planned_snapshot=snapshot, latest_snapshot=latest,
            edges=edges, now_ns=self._clock_ns(), in_flight=False))

    def _fail(self, reason, rows):
        return {
            "success": False,
            "reason_code": str(reason),
            "slot_index": -1,
            "selection_id": "",
            "path_source": "route-graph",
            "path_hash": "",
            "rows": rows,
        }


def _leg_record(name, trajectory, request_id, scene_revision, stamp_ns):
    points = [{
        "positions": [float(v) for v in trajectory.positions[i]],
        "velocities": [float(v) for v in trajectory.velocities[i]],
        "accelerations": [float(v) for v in
                          trajectory.accelerations[i]],
        "effort": [],
        "time_from_start_ns": int(trajectory.time_from_start_ns[i]),
    } for i in range(len(trajectory.positions))]
    return {
        "contract_version": 2,
        "request_id": str(request_id),
        "scene_revision": str(scene_revision),
        "stamp_ns": int(stamp_ns),
        "time_basis": "robot_state",
        "name": str(name),
        "segment_type": "joint_path",
        "target_link": "suction_contact_frame",
        "policy_valid": True,
        "keep_tool_down": True,
        "keep_camera_down": False,
        "lock_wrist": False,
        "allow_ompl_fallback": False,
        "required_cartesian_fraction": 0.0,
        "target_pose": None,
        "waypoints": [],
        "joint_trajectory": {
            "contract_version": 2,
            "kind": "joint_trajectory",
            "request_id": str(request_id),
            "scene_revision": str(scene_revision),
            "stamp_ns": int(stamp_ns),
            "time_basis": "robot_state",
            "angle_unit": "rad",
            "joint_names": list(trajectory.joint_names),
            "points": points,
        },
    }


def leg_records(route: PlannedRoute, edges: Sequence, request: Mapping,
                ) -> List[dict]:
    """One admitted ``joint_path`` record per route edge, in order.

    Names carry the canonical leg names (transit .. descend, then
    retreat/return after release) so the place cycle's state and release
    hooks join the causal chain. Times stay exactly the certified
    per-edge values (each leg is an independent PlanMotionV2 goal
    starting at its own zero).
    """
    stamp_ns = int((request.get("start_tcp") or {}).get("stamp_ns") or 0)
    legs = []
    for edge in edges:
        edge_class = str(edge.attributes.get("edge_class") or "edge")
        name = LEG_NAMES.get(edge_class, edge_class.lower())
        legs.append(_leg_record(name, edge.trajectory,
                                request["request_id"],
                                request["scene_revision"], stamp_ns))
    return legs


#: Pick waypoints -> edge classes for adapter generation. Pre-attach
#: motion carries no payload, so the edges plan unloaded through the
#: same MoveIt builder the carrying route uses (no new generation path).
PICK_LEG_CLASSES = {
    "pre_grasp": "PORTAL_TRANSIT",
    "approach": "TRAVERSE",
    "attach": "DESCEND",
    "pick_retreat": "RETREAT",
}


def pick_leg_records(pick_waypoints, *, graph, edge_builder, refine, fk,
                     start_q, request_id, scene_revision, stamp_ns):
    """Timed joint legs + their MoveIt edges for the pick ladder.

    The waypoint geometry stays the frozen ``build_sequence`` pick
    phases (a candidate generator, never an execution spec); each hop
    becomes a MoveIt-built timed edge. Pre-attach motion has no
    attachment record by definition — its certification is the EXPLICIT
    ``LoadState.UNLOADED`` admission (``certify_unloaded_legs``), so
    the edges are returned alongside the legs for that step
    ([ST09-I S5.7-4] pick-ladder certification).
    """
    from luggage_planning.grid_path_search import RoutePlanningError
    from luggage_planning.reachable_graph import _contracts as _graph_contracts

    _Load = _graph_contracts()[0].LoadState
    current_q = tuple(float(v) for v in start_q)
    position, quaternion = fk(current_q)
    current = graph.register_node(current_q, position, quaternion,
                                  "pick_home", _Load.UNLOADED, "pick")
    legs = []
    edges = []
    for name, target_position, target_quaternion in pick_waypoints:
        q, _info = refine(tuple(target_position),
                          tuple(target_quaternion), list(current_q))
        if q is None:
            raise RoutePlanningError(
                "EDGE_BUILD_FAILED",
                {"leg": name, "reason": "ik_unavailable"})
        node = graph.register_node(tuple(q), tuple(target_position),
                                   tuple(target_quaternion),
                                   "pick_%s" % name, _Load.UNLOADED, "pick")
        try:
            edge = edge_builder(current, node, PICK_LEG_CLASSES[name])
        except RoutePlanningError as error:
            detail = getattr(error, "detail", None) or {}
            if isinstance(detail, dict):
                detail = dict(detail, leg=name)
            raise RoutePlanningError(
                getattr(error, "code", None) or "EDGE_BUILD_FAILED",
                detail)
        legs.append(_leg_record(name, edge.trajectory, request_id,
                                scene_revision, stamp_ns))
        edges.append(edge)
        current, current_q = node, tuple(q)
    return legs, tuple(edges)


def certify_unloaded_legs(*, edges, snapshot, request_id, route_id, purpose,
                          certifier, validity_adapter, policy, now_ns):
    """Certify an explicitly-unloaded leg set (pick/observation/exit).

    [ST09-I S5.7-4] The admission is the EXPLICIT ``LoadState.UNLOADED``
    bound to ``UNLOADED_ATTACHMENT_DIGEST`` — never an absent
    AttachmentRecord silently read as unloaded. The route synthesizes
    the unloaded binding; the certifier runs its full check set
    (limits/timing/sampling/collision/occupancy/clearance) against the
    supplied snapshot and adapter. Returns
    ``(CertificationResult, PlannedRoute)``.
    """
    from st09_contracts._base import digest_of
    from st09_contracts.attachment import UNLOADED_ATTACHMENT_DIGEST
    from st09_contracts.graph import LoadState, PlannedRoute
    from luggage_planning.trajectory_certifier import CertificationContext

    edges = tuple(edges)
    if not edges:
        raise ValueError("certify_unloaded_legs: empty edge set")
    route = PlannedRoute(
        route_id=str(route_id),
        request_id=str(request_id),
        snapshot_digest=snapshot.digest(),
        payload_digest=UNLOADED_ATTACHMENT_DIGEST,
        attachment_digest=UNLOADED_ATTACHMENT_DIGEST,
        candidate_id="unloaded-%s" % str(purpose),
        entry_edge_ids=tuple(edge.edge_id for edge in edges),
        retreat_edge_ids=(),
        edge_digests=tuple(edge.digest() for edge in edges),
        trajectory_digest=digest_of(
            *(edge.trajectory.digest() for edge in edges)),
        planning_revision="unloaded:%s" % str(purpose))
    result = certifier.certify(CertificationContext(
        route=route, snapshot=snapshot, attachment=None, edges=edges,
        validity_adapter=validity_adapter, now_ns=int(now_ns),
        policy=policy, load_state=LoadState.UNLOADED))
    return result, route


def register_attach_from_request(provider, request: Mapping,
                                 *, box_wdh: Sequence[float],
                                 cargo_id: str = "", box_id: str = "",
                                 cycle_id: str = "",
                                 source_method: str =
                                 "v2_grasp_contract_attach_confirmed",
                                 ) -> None:
    """Freeze the attachment record from the admitted measured request.

    The driver measured the box pose and TCP TF at the same stamp during
    attach confirmation and carried their exact relative transform
    through the v2 grasp contract (``T_box_tool`` from
    ``grasp_from_attach``; the same-stamp rule was enforced upstream by
    ``require_measured_pose_agreement``). This re-freezes that measured
    relative transform with the D provider so the route consumes a
    validated record — the provider's world-pose bookkeeping is fed the
    identity TCP pose, which makes the frozen ``T_tcp_box`` exactly the
    inverse of the measured grasp. No GT, planned-slot, or PCA-yaw
    source ever passes through here: the source kind is measured and
    attach-confirmed.
    """
    from st09_contracts.attachment import invert_transform
    from luggage_planning.attachment_provider import (
        AttachmentSourceKind, MeasuredPose,
    )

    grasp = request["grasp"]
    stamp_ns = int(grasp.get("stamp_ns") or 0)
    t_tcp_box = invert_transform(
        tuple(float(v) for v in grasp["position"]),
        tuple(float(v) for v in grasp["quaternion_xyzw"]))
    provider.record_attachment(
        str(request["request_id"]),
        MeasuredPose(frame="box", position=t_tcp_box[0],
                     quaternion_xyzw=t_tcp_box[1], stamp_ns=stamp_ns),
        MeasuredPose(frame="suction_contact_frame",
                     position=(0.0, 0.0, 0.0),
                     quaternion_xyzw=(0.0, 0.0, 0.0, 1.0), stamp_ns=stamp_ns),
        attach_confirmed=True,
        source_kind=AttachmentSourceKind.MEASURED_ATTACH_CONFIRMED,
        source_method=str(source_method),
        dimensions_wdh=[float(v) for v in box_wdh],
        cargo_id=str(cargo_id or request["request_id"]),
        box_id=str(box_id or request["request_id"]),
        cycle_id=str(cycle_id or request["request_id"]),
    )
