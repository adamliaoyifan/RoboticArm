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
#: [ST09-I S5.7-6] Carrying-leg vacuum admission window (s). The
#: /vacuum/state message carries no stamp; the RECEIVE time on the node
#: clock is the freshness basis (the producer's clock_ns is wired to
#: the same node clock, so the age comparison stays in one timebase).
VACUUM_STATE_FRESHNESS_S = 2.0
#: [ST09-I S5.7-6] GoToRobotPoseV2 already-there gate (rad, wrap
#: aware): within it the service returns zero legs and promotes
# nothing — the caller stays put.
NAMED_POSE_ALREADY_THERE_RAD = 0.02
#: [ST09-I S5.7-6] EnsureCycleStartV2 aperture-escape margin (m),
#: re-declared planning-side (mirrors the driver's declared margin) so
#: luggage_planning never imports eval scripts.
APERTURE_ESCAPE_MARGIN_M = 0.10

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
    tcp_path = list(getattr(urdf, "tcp_path", None) or ())
    if not tcp_path:
        # [ST09-I S5.7-6 fix] the offline_ik urdf has no tcp_path
        # attribute and path_to returns the JOINT sequence — the old
        # literal fallback ended in the LINK name and crashed urdf.tcp
        # with KeyError('suction_contact_frame') the first time this
        # ran on the mock stack.
        try:
            tcp_path = list(urdf.path_to("suction_contact_frame"))
        except Exception:  # noqa: BLE001 - defensive default
            tcp_path = ["elfin_joint%d" % index for index in range(1, 7)]
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


def _opening_depth(slot, inward, opening_point):
    """Signed distance from the opening along the inward normal."""
    if not inward or not opening_point:
        return 0.0
    position = slot["position"]
    return sum(
        (float(position[axis]) - float(opening_point[axis]))
        * float(inward[axis]) for axis in range(3))


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
        "map_source": str(surface.get("map_source") or ""),
        "placed_count": int(surface.get("placed_count") or 0),
        "cargo_observed": bool(surface.get("cargo_observed") or False),
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
                 atlas_prior=None, vacuum_state_fn=None,
                 site_poses_fn=None, scene_config=None,
                 scene_revision_binding=""):
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
        #: [ST09-I S5.7-6] Authoritative vacuum feed for the carrying
        #: admission window: callable -> {"attached", "vacuum_on",
        #: "received_ns"} or None (latest_snapshot_fn mode). Keyed on
        #: ``attached`` ONLY — the mock sim backend sets fail_reason
        #: while attached stays True, so fail_reason never gates here.
        self.vacuum_state_fn = vacuum_state_fn
        #: [ST09-I S5.7-6] Site-pose file access for GoToRobotPoseV2
        #: (latest_snapshot_fn mode): callable -> (data_dict, path_str)
        #: or None when no pose file is configured. The producer NEVER
        #: falls back to defaults.observe_pose or current/here — a
        #: named request must name a real, validated site pose.
        self.site_poses_fn = site_poses_fn
        #: [ST09-I S5.7-6] EnsureCycleStartV2 adjudication inputs: the
        #: scene config (world->base_link + container geometry) and the
        #: node's own scene-identity binding the request must match.
        self.scene_config = scene_config
        self.scene_revision_binding = str(scene_revision_binding or "")
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
        #: [ST09-I S5.7-6] Frozen-budget attribution: nanoseconds spent
        #: in certification / map revalidation inside the CURRENT public
        #: call (reset at each entry point; search time is the residue).
        self._stage_ns = {"certify": 0, "revalidate": 0}
        #: [ST09-I S5.7-6] Admissions displaced by a newer selection,
        #: route_digest -> certification_id at demotion time. The LIVE
        #: admission is single-slot on purpose: selecting/verifying
        #: route B makes B the only live admission and IMMEDIATELY
        #: demotes A — A's next leg check then returns the named
        #: CERTIFICATE_SUPERSEDED (the certifier side KNEW A, then
        #: replaced it), never a generic ADMISSION_UNKNOWN. Bounded FIFO.
        self.superseded_admissions = {}

    # -- helpers ---------------------------------------------------------

    def _attachment(self, request, now_ns):
        """Latest valid measured record for this request, or a named reason."""
        record, reason, evidence = admit_attachment_for_route(
            self.attachment_provider, request, now_ns)
        self.last_attachment_evidence = evidence
        return record, reason

    def _fail_attachment(self, reason, rows):
        """Attachment refusal plus the invalidation row the selector dumps."""
        evidence = getattr(self, "last_attachment_evidence", None)
        if evidence:
            row = dict(evidence)
            row["reason"] = str(reason)
            rows = [row] + list(rows)
        return self._fail(reason, rows)

    def _seed_candidates(self, start_q):
        seeds = [tuple(float(v) for v in start_q), tuple(TOOL_DOWN_SEED_Q)]
        return list(dict.fromkeys(seeds))

    def _refine_to(self, position, orientation, seed_q,
                   prefer_tool_down=False):
        if self.refine is None:
            return None, {"reason": "no_refine"}
        requested = tuple(float(v) for v in seed_q)
        seeds = self._seed_candidates(seed_q)
        if prefer_tool_down and len(seeds) > 1:
            seeds = [seeds[1], seeds[0]]
        infos = {}
        for seed in seeds:
            try:
                q, info = self.refine(tuple(position), tuple(orientation),
                                      list(seed))
                if q is not None:
                    return tuple(q), info
            except Exception:  # noqa: BLE001 - anchor seeding is best effort
                info = {"reason": "ik_unavailable"}
            infos[tuple(seed)] = dict(info or {"reason": "ik_unavailable"})
        # Failure evidence is the REQUESTED seed's residual; the tool-down
        # fallback's residual rides beside it, never in its place.
        info = infos.get(requested) or next(iter(infos.values()), {})
        fallback = [value for key, value in infos.items() if key != requested]
        if fallback:
            info = dict(info, fallback=fallback)
        return None, info or {"reason": "ik_unavailable"}

    def _refine_exact(self, position, orientation, seed_q):
        """Refine one seed only. A miss stays a miss.

        DESCEND anchor seeds use this so a tool-down fallback cannot
        replace the hover branch with the opposite heading.
        """
        if self.refine is None:
            return None, {"reason": "no_refine"}
        try:
            q, info = self.refine(tuple(position), tuple(orientation),
                                  list(seed_q))
        except Exception:  # noqa: BLE001 - one seed must not abort the set
            return None, {"reason": "ik_unavailable"}
        info = info if isinstance(info, dict) else {}
        if q is None:
            return None, info or {"reason": "ik_unavailable"}
        return tuple(q), info

    def _atlas_seed_check(self, targets, atlas_seeds):
        """Refine each atlas seed onto each exact TCP target (offline IK).

        ``ok`` when any (seed, target) pair refines within the refine
        tolerance. Uses the raw refine, so no tool-down fallback can mask
        the atlas seed's own residual. Costs milliseconds, not a route build.
        """
        best = None
        attempts = 0
        if self.refine is None:
            return {"ok": True, "attempts": 0, "best_pos_err_m": None}
        for target in targets:
            for seed in atlas_seeds:
                attempts += 1
                try:
                    q, info = self.refine(tuple(target.position),
                                          tuple(target.quaternion_xyzw),
                                          list(seed))
                except Exception:  # noqa: BLE001 - check fails open
                    return {"ok": True, "attempts": attempts,
                            "best_pos_err_m": None}
                if q is not None:
                    return {"ok": True, "attempts": attempts,
                            "best_pos_err_m": 0.0}
                err = (info or {}).get("pos_err_m")
                if isinstance(err, (int, float)):
                    best = err if best is None else min(best, err)
        return {"ok": False, "attempts": attempts, "best_pos_err_m": best}

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
        # Node ids this candidate may route through. The request graph
        # also keeps earlier candidates' anchors; the search ignores them.
        node_ids = set()
        for load_state in (LoadState.CARRYING, LoadState.UNLOADED):
            node_ids.add(graph.register_node(
                seed_q, self.fk(seed_q)[0], orientation, "home",
                load_state, "start").node_id)
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
                    node_ids.add(graph.register_node(
                        q, position, orientation, region, load_state,
                        "anchor").node_id)
            seeded.append(region)
        return candidate, box_target, plan, seeded, node_ids

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
        self._stage_ns = {"certify": 0, "revalidate": 0}
        rows: List[dict] = []
        snapshot = planning_snapshot_from_surface(
            surface_map_from_contract(request["surface"], request),
            scene_revision=str(request["scene_revision"]),
            request_id=str(request["request_id"]))
        attachment, attachment_reason = self._attachment(
            request, self._clock_ns())
        if attachment is None:
            return self._fail_attachment(attachment_reason, rows)

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
            inward = self._inward_normal()
            opening_point = (self.opening_info or {}).get("point")
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
                seeds = list(prior.get("seeds") or [])
                verdict = str(prior.get("verdict") or "unknown")
                entry = (index, slot, seeds, _opening_depth(
                    slot, inward, opening_point), verdict)
                if seeds:
                    seeded_order.append(entry)
                elif verdict == "reachable":
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
            def _depth_key(entry):
                # Opening distance ascending, then more atlas seeds,
                # then the shallower end of placement's list. A deep
                # seeded slot does not jump a nearer seeded slot.
                _index, _slot, seeds, depth, _verdict = entry
                return (float(depth), -len(seeds), -float(_index))

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
                (index, slot, [], 0.0, "no_atlas") for index, slot
                in enumerate(request["slots"])]
        self._log("route: select %s slots=%d hash=%s" % (
            request["request_id"], len(request["slots"]),
            str(request.get("geometry_hash") or "")[:12]))
        tried_indices = []
        for index, slot, atlas_seeds, opening_depth, verdict in ordered_slots:
            if index and index % 100 == 0:
                self._log("route: slot %d/%d %.1fs last=%s" % (
                    index, len(request["slots"]),
                    (self._clock_ns() - started_ns) / 1e9, last_reason))
            base_candidate = candidate_from_slot(slot, index)
            elapsed = self._clock_ns() - started_ns
            remaining = self.budget_ns - elapsed
            if remaining <= int(CERTIFICATION_RESERVE_S * 1e9):
                rows.append(self._attempt_summary(ordered_slots, tried_indices))
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
            tried_indices.append(index)
            row = {
                "slot_index": index,
                "candidate_id": base_candidate.candidate_id,
                "depth_m": float(opening_depth),
                "reachability": str(verdict),
                "seed_count": len(atlas_seeds),
                "_started_ns": self._clock_ns(),
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
                self._close_row(rows, row)
                last_reason = row["reason"]
                continue
            if atlas_seeds:
                seed_check = self._atlas_seed_check(prior_targets, atlas_seeds)
                if not seed_check["ok"]:
                    # A 0.15 m atlas cell can straddle the reach boundary:
                    # REACHABLE for the cell, unreachable at this exact
                    # target. Defer the slot (seedless) behind every other
                    # candidate instead of spending the route budget on it.
                    row.update(reason="ATLAS_SEED_MISS",
                               seed_check=seed_check)
                    self._close_row(rows, row)
                    ordered_slots.append(
                        (index, slot, [], opening_depth, "seed_miss"))
                    continue
            candidate, _box_target, plan, _seeded, scope_ids = \
                self._prepare_candidate(
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
                        anchor_qs=[
                            node.q for node in
                            self.graph.nodes_by_region("slot_upper")
                            if str(node.load_state) == "carrying"
                            and node.node_id in scope_ids],
                        atlas_seeds=atlas_seeds),
                    tcp_path=self.tcp_path,
                    region_poses={
                        region: position for region, position in (
                            ("staging", plan.staging),
                            ("portal", plan.portal),
                            ("slot_approach", plan.slot_approach),
                            ("slot_upper", plan.slot_upper),
                            ("exit", plan.exit))
                        if position is not None},
                    scope_node_ids=scope_ids)
                targets = self._expand_targets(
                    candidate, attachment, request)
                if not targets:
                    row["reason"] = "TARGET_SYNTHESIS_EMPTY"
                    self._close_row(rows, row)
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
                row["ik_refine"] = list(getattr(self, "last_goal_refine", []) or [])
                row["atlas"] = getattr(self, "last_atlas_diag", None)
                self._close_row(rows, row)
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
                self._close_row(rows, row)
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
                self._close_row(rows, row)
                last_reason = exc.code
                self._log("route: candidate %d failed %s" % (index, exc.code))
                continue
            except Exception as exc:  # noqa: BLE001 - service boundary
                # Edge-builder scene/transport failures are named
                # per-candidate failures, never a crashed service.
                row["reason"] = "EDGE_BUILD_FAILED"
                row["detail"] = {"error": repr(exc)[:300]}
                self._close_row(rows, row)
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
                self._close_row(rows, row)
                last_reason = row["reason"]
                break
            if row.get("reason"):
                continue
            revalidation = self._revalidate(route, snapshot, edges)
            if revalidation is None:
                row["reason"] = "MAP_REVALIDATION_UNAVAILABLE"
                self._close_row(rows, row)
                last_reason = row["reason"]
                continue
            from st09_contracts.certification import MapDecision

            row["decision"] = getattr(revalidation.decision, "value",
                                      str(revalidation.decision))
            row["map_admission"] = self._map_admission(revalidation)
            if revalidation.decision is not MapDecision.CONTINUE:
                reason = getattr(
                    revalidation.reason, "value", "MAP_REVISION_MISMATCH")
                row["map_refusal"] = self._map_refusal(
                    snapshot, revalidation, route, edges, request)
                if reason == "MAP_STALE":
                    refreshed = self._revalidate(route, snapshot, edges)
                    if (refreshed is not None
                            and refreshed.decision is MapDecision.CONTINUE):
                        revalidation = refreshed
                        row["map_refresh"] = "fresh_snapshot_accepted"
                        row["map_admission"] = self._map_admission(
                            refreshed)
                    else:
                        row["reason"] = "MAP_STALE_REPEATED"
                        if refreshed is not None:
                            row["map_refusal_after_refresh"] = (
                                self._map_refusal(
                                    snapshot, refreshed, route, edges,
                                    request))
                            row["map_admission"] = self._map_admission(
                                refreshed)
                        self._close_row(rows, row)
                        last_reason = row["reason"]
                        continue
                else:
                    row["reason"] = reason
                    self._close_row(rows, row)
                    last_reason = row["reason"]
                    continue
            legs = leg_records(route, edges, request)
            row["reason"] = "OK"
            row["certified"] = True
            row["map_admission"] = self._map_admission(revalidation)
            self._close_row(rows, row)
            self.last_route = route
            self.last_certification = certification
            self.last_revalidation = revalidation
            self.last_rows = rows
            self._promote_selection({
                "path_hash": route.digest(),
                "route": route,
                "edges": edges,
                "snapshot": snapshot,
                "attachment": attachment,
                "candidate": candidate,
                "request": request,
                "certification_id": certification.digest(),
            })
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
                "stage_timings": self._stage_timings(started_ns),
            }
        rows.append(self._attempt_summary(ordered_slots, tried_indices))
        return self._fail(last_reason, rows)

    def _stage_timings(self, started_ns):
        """[ST09-I S5.7-6] Frozen-budget attribution (ms, rounded).

        Certification and map revalidation are measured directly; the
        graph/edge search share is the residue of the total wall time.
        Recorded for evidence — the frozen budgets themselves never
        move because of these numbers.
        """
        total_ns = self._clock_ns() - started_ns
        stages = getattr(self, "_stage_ns", None) \
            or {"certify": 0, "revalidate": 0}
        return {
            "total_ms": round(total_ns / 1e6, 1),
            "search_ms": round(max(
                0, total_ns - stages["certify"]
                - stages["revalidate"]) / 1e6, 1),
            "certify_ms": round(stages["certify"] / 1e6, 1),
            "revalidate_ms": round(stages["revalidate"] / 1e6, 1),
        }

    def _promote_selection(self, selection):
        """[ST09-I S5.7-6] Promote `selection` to THE live admission.

        The live admission is single-slot: whatever it displaces is
        demoted into ``superseded_admissions`` so its next per-leg
        check gets the NAMED CERTIFICATE_SUPERSEDED (the certifier side
        knew that route, then replaced it) instead of the peer-restart
        ADMISSION_UNKNOWN. Bounded FIFO keeps long sessions flat.
        """
        previous = getattr(self, "last_selection", None)
        if previous and previous.get("path_hash"):
            superseded = getattr(self, "superseded_admissions", None)
            if superseded is not None:
                superseded[str(previous["path_hash"])] = str(
                    previous.get("certification_id", ""))
                while len(superseded) > 16:
                    superseded.pop(next(iter(superseded)))
        # [ST09-I S5.7-6] admission phase: place routes START in the
        # carrying phase (an explicit, evidenced transition is the only
        # way out); attachment-free selections (observation) are born
        # unloaded.
        if "phase" not in selection:
            selection["phase"] = (
                "unloaded" if selection.get("attachment") is None
                else "carrying")
        self.last_selection = selection

    @staticmethod
    def _selection_phase(selection):
        if not selection:
            return "unloaded"
        phase = selection.get("phase")
        if phase:
            return str(phase)
        return "unloaded" if selection.get("attachment") is None \
            else "carrying"

    def _live_attachment_matches(self, selection):
        """Measured attachment binding vs the certified one.

        Returns ``(ok, reason, detail)``: a carrying-phase leg requires
        the measured record to still exist as a VALID record whose
        digest equals the certificate's attachment digest — an
        incarnation change never silently re-authorizes the old
        certificate. The frozen 10 s acquisition->PLANNING-use window
        is deliberately NOT re-applied here: it bounds select/verify
        (where expire_stale already ran); execution minutes later is
        not a fresh acquisition. Payload LOSS mid-route is a live
        vacuum-state fact gated by the DRIVER before each carrying-leg
        submission AND by the admission verdict's own vacuum window
        ([ST09-I S5.7-6] ``_live_vacuum_holds`` runs right after this
        check for carrying claims).
        """
        provider = getattr(self, "attachment_provider", None)
        if provider is None:
            return False, "ATTACHMENT_STATE_UNKNOWN", {
                "provider": "absent"}
        request = selection.get("request") or {}
        request_id = str(request.get("request_id", "") or "")
        certified = selection.get("attachment")
        want_digest = ""
        if certified is not None:
            digest_fn = getattr(certified, "digest", None)
            want_digest = str(digest_fn()) if callable(digest_fn) else ""
        try:
            record = provider.latest_record(request_id)
        except (ContractViolation, LookupError, ValueError) as exc:
            return False, "ATTACHMENT_LOST", {"reason": _named_reason(exc)}
        record_digest_fn = getattr(record, "digest", None)
        record_digest = str(record_digest_fn()) \
            if callable(record_digest_fn) else ""
        if want_digest and record_digest and record_digest != want_digest:
            return False, "ATTACHMENT_INCARNATION_CHANGED", {
                "certified": want_digest, "live": record_digest}
        return True, "", {"attachment_digest": record_digest}

    def _live_vacuum_holds(self):
        """[ST09-I S5.7-6] Authoritative vacuum state vs the carrying leg.

        Closes the driver-check -> controller-commit window: the
        admission verdict reads the authoritative /vacuum/state feed
        (TRANSIENT_LOCAL latched + 30 Hz ticks; both the gazebo and the
        I-suite mock stack run the same vacuum_controller_node). The
        message has no stamp, so the RECEIVE time — same node clock as
        the producer's clock_ns — is the freshness basis. Keyed on
        ``attached`` ONLY: the mock sim backend sets fail_reason while
        attached stays True (gz set_pose unavailable), so fail_reason
        never gates here. Old test stubs without the attribute fail
        CLOSED (getattr -> None -> VACUUM_STATE_UNAVAILABLE), never with
        an AttributeError. Returns ``(ok, reason, detail)``.
        """
        fetch = getattr(self, "vacuum_state_fn", None)
        state = fetch() if callable(fetch) else None
        if not state:
            return False, "VACUUM_STATE_UNAVAILABLE", {"feed": "absent"}
        received_ns = int(state.get("received_ns") or 0)
        age_s = max(0.0, (self._clock_ns() - received_ns) / 1e9)
        if age_s > VACUUM_STATE_FRESHNESS_S:
            return False, "VACUUM_STATE_STALE", {"age_s": round(age_s, 3)}
        if not bool(state.get("attached")):
            return False, "PAYLOAD_LOST", {"attached": False}
        return True, "", {"age_s": round(age_s, 3), "attached": True}

    def ensure_cycle_start_state(self, *, joints, request_scene_revision,
                                 vacuum_state=None):
        """[ST09-I S5.7-6 §2 minimal] Cycle-start adjudication.

        Pure authoritative-state verdict, NO route planning: the
        selector's own latched joints + offline world FK + scene config
        + latched vacuum state classify the suction TCP into the
        container zone taxonomy and report whether the next cycle may
        start from here. ALREADY_READY means the caller may request
        GoToRobotPoseV2; every other outcome is a named RECOVERY stop
        (the certified exit is the place route's own retreat/return
        legs, or a later recovery route — never synthesized here).

        Returns ``(ready, reason, zone, attachment_state, detail)``.
        """
        scene = getattr(self, "scene_config", None)
        attachment = "unknown"
        if vacuum_state:
            attachment = ("carrying" if bool(vacuum_state.get("attached"))
                          else "unloaded")
        vacuum_detail = {
            "attached": bool((vacuum_state or {}).get("attached")),
            "vacuum_on": bool((vacuum_state or {}).get("vacuum_on")),
        }
        if not scene:
            return (False, "SCENE_CONFIG_UNAVAILABLE", "unknown",
                    attachment, {"vacuum": vacuum_detail})
        if str(request_scene_revision or "") != str(
                getattr(self, "scene_revision_binding", "")):
            return (False, "IDENTITY_UNKNOWN", "unknown", attachment, {
                "vacuum": vacuum_detail,
                "request": str(request_scene_revision or "")[:24],
                "binding": str(
                    getattr(self, "scene_revision_binding", ""))[:24]})
        if joints is None or len(tuple(joints)) != 6:
            return (False, "JOINT_STATE_UNAVAILABLE", "unknown",
                    attachment, {"vacuum": vacuum_detail})
        # world-frame FK -> base-link for the zone taxonomy
        from luggage_description.scene_tf_config_utils import (
            container_zone,
            xyz_world_to_base_link,
        )

        try:
            suction_world, _quaternion = self.fk(list(joints))
            base = xyz_world_to_base_link(scene, list(suction_world))
        except Exception as error:  # noqa: BLE001 - adjudication boundary
            return (False, "ZONE_UNKNOWN", "unknown", attachment, {
                "vacuum": vacuum_detail, "fk_error": repr(error)[:200]})
        container = (scene.get("container") or {})
        outer = container.get("outer") or {}
        inner = container.get("inner") or {}
        wall = 0.04
        try:
            if outer.get("length") and inner.get("length"):
                wall = max(0.0, (float(outer["length"])
                                 - float(inner["length"])) * 0.5)
        except (TypeError, ValueError):
            pass
        transform = camera_to_tcp_transform(getattr(self, "urdf", None))
        if transform is None:
            return (False, "CAMERA_FRAME_UNAVAILABLE", "unknown",
                    attachment, {
                        "vacuum": vacuum_detail,
                        "suction_base": [round(float(v), 4) for v in base],
                        "wall_m": round(wall, 4)})
        envelope = float(math.sqrt(sum(
            float(v) * float(v) for v in transform[0])))
        depth = wall + envelope + APERTURE_ESCAPE_MARGIN_M
        zone = container_zone(base, scene, depth)
        detail = {
            "vacuum": vacuum_detail,
            "corridor_depth_m": round(depth, 4),
            "wall_m": round(wall, 4),
            "camera_envelope_m": round(envelope, 4),
            "envelope_source": "urdf",
            "suction_base": [round(float(v), 4) for v in base],
        }
        if attachment == "unknown":
            return False, "ATTACHMENT_STATE_UNKNOWN", zone, attachment, \
                detail
        if attachment == "carrying":
            # The vacuum is held by the backend; the driver must NOT
            # turn it off to force an exit.
            return False, "CARRY_EXIT_UNSUPPORTED", zone, attachment, \
                detail
        if zone == "outside_container":
            return True, "OK", zone, attachment, detail
        if zone in ("inside_container", "opening_corridor"):
            return False, "EXIT_ROUTE_UNAVAILABLE", zone, attachment, \
                detail
        return False, "ZONE_UNKNOWN", zone, attachment, detail

    def transition_admission_phase(self, *, route_digest,
                                   certification_id, target_phase,
                                   detach_confirmed=False,
                                   release_evidence="",
                                   map_revision_after_release=0):
        """[ST09-I S5.7-6] Release-boundary phase transition verdict.

        CARRYING certificate phase -> release side effect (caller) ->
        detach confirmed + evidence identity -> UNLOADED phase active
        -> retreat/return legs execute. Refuses by name when the
        release backend has not confirmed detach, the evidence is
        empty, or the route is not in the carrying phase. The
        certified content never changes — only which phase of it may
        execute.
        """
        selection = getattr(self, "last_selection", None)
        if not selection or selection.get("path_hash") != str(route_digest):
            superseded = getattr(self, "superseded_admissions", None) or {}
            if str(route_digest) in superseded:
                return False, "CERTIFICATE_SUPERSEDED", {
                    "live": (selection or {}).get("path_hash", "")}
            return False, "ADMISSION_UNKNOWN", {
                "known": (selection or {}).get("path_hash", "")}
        if str(certification_id) != str(
                selection.get("certification_id", "")):
            return False, "CERTIFICATE_SUPERSEDED", {
                "live": str(selection.get("certification_id", ""))}
        current = self._selection_phase(selection)
        target = str(target_phase or "").lower()
        if target != "unloaded" or current != "carrying":
            return False, "PHASE_TRANSITION_INVALID", {
                "current": current, "target": target}
        if not detach_confirmed:
            return False, "DETACH_UNCONFIRMED", {
                "release_evidence": str(release_evidence or "")[:200]}
        if not str(release_evidence or "").strip():
            return False, "RELEASE_EVIDENCE_MISSING", {}
        # The release side effect is verified: activate the UNLOADED
        # phase. The released box is already a certified world obstacle
        # in the unloaded edge scene states (edge_scene_states builds
        # it from the candidate rest pose); the transition records the
        # observed post-release map revision when the caller has one.
        selection["phase"] = "unloaded"
        selection["phase_transition"] = {
            "release_evidence": str(release_evidence)[:500],
            "map_revision_after_release": int(
                map_revision_after_release or 0),
            "transition_ns": self._clock_ns(),
        }
        return True, "OK", {
            "phase": "unloaded",
            "map_revision_after_release": int(
                map_revision_after_release or 0)}

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
        started_ns = self._clock_ns()
        self._stage_ns = {"certify": 0, "revalidate": 0}
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
            "leg_load_states": [
                str(getattr(edge, "load_state", "") or "").lower()
                for edge in edges],
            "certify_ms": round(
                (self._stage_ns or {}).get("certify", 0) / 1e6, 1),
            "verify_ms": round(
                (self._clock_ns() - started_ns) / 1e6, 1),
        }

    def execution_admission_is_live(self, *, route_digest,
                                    certification_id,
                                    admission_map_revision,
                                    scene_revision,
                                    leg_digest=None,
                                    leg_load_state=None):
        """[ST09-I S5.7-3b] Is a certified execution admission STILL live?

        Certify A, execute A — AND the certification still holds when A
        executes. The selector owns the verdict; the executor only
        refuses on it. Peer restart loses the selection
        (ADMISSION_UNKNOWN); a superseded certificate or changed scene
        identity refuses; a map that advanced without touching the
        route's edges stays LIVE (new revision recorded); corridor
        impact or staleness revokes. Returns ``(live, reason, detail)``.

        [ST09-I S5.7-6] Per-leg scope: when the executor claims a
        specific leg, the leg must be one of the certified route's
        legs AND its certified load-state index must match the claim.
        One place route covers the carrying legs, the release
        boundary, and the unloaded retreat — each leg executes under
        ITS OWN load-state index, so the vacuum-off attachment digest
        change at release is never misread as an identity change (the
        liveness verdict never consults the live attachment state),
        and a carrying-phase scope can never authorize a post-release
        leg (LOAD_STATE_MISMATCH / LEG_DIGEST_MISMATCH).
        """
        selection = getattr(self, "last_selection", None)
        if not selection or selection.get("path_hash") != str(route_digest):
            superseded = getattr(self, "superseded_admissions", None) or {}
            if str(route_digest) in superseded:
                return False, "CERTIFICATE_SUPERSEDED", {
                    "live": (selection or {}).get("path_hash", ""),
                    "demoted_certification_id": superseded[
                        str(route_digest)]}
            return False, "ADMISSION_UNKNOWN", {
                "known": (selection or {}).get("path_hash", "")}
        if str(certification_id) != str(
                selection.get("certification_id", "")):
            return False, "CERTIFICATE_SUPERSEDED", {
                "live": str(selection.get("certification_id", ""))}
        if leg_digest or leg_load_state:
            certified = [(
                str(edge.trajectory.digest()),
                str(getattr(edge, "load_state", "") or "").lower(),
            ) for edge in selection["edges"]]
            claimed = [state for digest, state in certified
                       if digest == str(leg_digest)] if leg_digest else []
            if leg_digest and not claimed:
                return False, "LEG_DIGEST_MISMATCH", {
                    "claimed_leg": str(leg_digest),
                    "certified_legs": len(certified)}
            if leg_digest and leg_load_state \
                    and str(leg_load_state).lower() != claimed[0]:
                return False, "LOAD_STATE_MISMATCH", {
                    "claimed": str(leg_load_state).lower(),
                    "certified": claimed[0]}
        # [ST09-I S5.7-6] certificate scope AND live state are separate
        # gates: a carrying leg additionally requires the admission to
        # still be in the carrying phase AND a live measured attachment
        # matching the certified digest (payload lost / incarnation
        # changed refuse by name); an unloaded leg requires the release
        # phase transition to have ACTIVATED the unloaded phase — the
        # vacuum-off digest change at the boundary is never misread as
        # an identity change, but a certificate merely SAYING unloaded
        # never authorizes the retreat on its own.
        claimed_state = str(leg_load_state or "").lower()
        if claimed_state:
            phase = self._selection_phase(selection)
            if claimed_state == "carrying":
                if phase != "carrying":
                    return False, "PHASE_STATE_MISMATCH", {
                        "phase": phase}
                live_ok, live_reason, live_detail = \
                    self._live_attachment_matches(selection)
                if not live_ok:
                    return False, live_reason, live_detail
                # [ST09-I S5.7-6] vacuum admission window: the measured
                # attachment binding above proves WHAT is carried; this
                # gate proves it is STILL carried at commit time — the
                # authoritative /vacuum/state must be fresh and attached
                # when the leg claims carrying (payload loss between
                # the driver check and the controller submission refuses
                # by name here).
                vacuum_ok, vacuum_reason, vacuum_detail = \
                    self._live_vacuum_holds()
                if not vacuum_ok:
                    detail = dict(vacuum_detail)
                    detail["vacuum_gate"] = True
                    return False, vacuum_reason, detail
            elif claimed_state == "unloaded" and phase != "unloaded":
                return False, "PHASE_TRANSITION_REQUIRED", {
                    "phase": phase}
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

        started_ns = self._clock_ns()
        self._stage_ns = {"certify": 0, "revalidate": 0}
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
                    self._close_row(rows, row)
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
                    self._close_row(rows, row)
                    last_reason = row["reason"]
                    continue
                from st09_contracts.certification import MapDecision

                if revalidation.decision is not MapDecision.CONTINUE:
                    row["reason"] = getattr(
                        revalidation.reason, "value",
                        "MAP_REVISION_MISMATCH")
                    self._close_row(rows, row)
                    last_reason = row["reason"]
                    continue
                certification, failure = self._certify(
                    route, snapshot, None, edges, None, None,
                    load_state=LoadState.UNLOADED)
                if certification is None or not certification.passed:
                    row["reason"] = failure if certification is None \
                        else certification.reason.value
                    row["recertified"] = False
                    self._close_row(rows, row)
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
                self._promote_selection({
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
                })
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
                    "stage_timings": self._stage_timings(started_ns),
                }
        finally:
            if callable(reset_scene):
                reset_scene()
        return self._fail(last_reason, rows)

    def plan_named_pose_route(self, *, request_id, pose_name,
                              current_joints, scene_revision,
                              latest_map_revision):
        """[ST09-I S5.7-6] Certified motion to a NAMED site pose.

        The caller names an explicit calibrated pose (GoToRobotPoseV2);
        this producer validates the site-pose file against the live
        scene identity, builds ONE MoveIt edge from the live start
        state to the file's joint values (FK fills the goal node's pose
        field — no IK), and returns the COMPLETE certified joint_path
        under the explicit LoadState.UNLOADED admission — the same
        pipeline plan_observation_route uses. No silent
        defaults.observe_pose resolution and no current/here execution:
        those are caller-side decisions, never server-side fallbacks.
        Returns a dict mirroring plan_observation_route's shape.
        """
        from luggage_planning.named_robot_poses import (
            is_current_pose_name)

        started_ns = self._clock_ns()
        self._stage_ns = {"certify": 0, "revalidate": 0}
        fetch_poses = getattr(self, "site_poses_fn", None)
        site = fetch_poses() if callable(fetch_poses) else None
        if site is None:
            return self._fail("POSE_FILE_MISSING", [])
        data, site_path = site
        pose_name = str(pose_name)
        if is_current_pose_name(pose_name):
            return self._fail("CURRENT_POSE_NOT_EXECUTABLE",
                              [{"pose_name": pose_name}])
        identity = data.get("identity") or {}
        file_scene = str(identity.get("scene_revision") or "")
        if file_scene != str(scene_revision):
            return self._fail("SCENE_STALE", [{
                "file": file_scene, "live": str(scene_revision),
                "path": str(site_path)}])
        if list(identity.get("joint_names") or []) != list(CANONICAL_JOINTS):
            return self._fail("JOINT_ORDER_MISMATCH", [{
                "file": list(identity.get("joint_names") or []),
                "expected": list(CANONICAL_JOINTS)}])
        from luggage_planning.site_pose_schema import (
            executable_site_pose,
            validate_site_poses,
        )

        report = validate_site_poses(
            data, expected_scene_revision=str(scene_revision),
            expected_joint_names=list(CANONICAL_JOINTS),
            schema=data.get("schema"))
        values = (executable_site_pose(report, pose_name)
                  if pose_name in (data.get("poses") or {}) else None)
        if (not report.get("ok") or values is None
                or len(values) != len(CANONICAL_JOINTS)):
            return self._fail("POSE_NOT_VALID", [{
                "pose_name": pose_name,
                "issues": report.get("issues") or ["pose not valid"]}])
        current = tuple(float(v) for v in (current_joints or ()))
        if len(current) != 6:
            return self._fail("START_STATE_MISMATCH",
                              [{"joints": len(current)}])

        def _wrap(delta):
            return (delta + math.pi) % (2.0 * math.pi) - math.pi

        delta = max(abs(_wrap(c - t))
                    for c, t in zip(current, values))
        request = {"request_id": str(request_id),
                   "scene_revision": str(scene_revision),
                   "start_tcp": {}}
        if delta <= NAMED_POSE_ALREADY_THERE_RAD:
            # Zero legs, no admission promoted, nothing to submit — the
            # caller records already_there and moves on. Pure
            # joints-vs-file adjudication: no map is read (there is
            # nothing to plan, so nothing to admit against a map).
            return {
                "success": True,
                "reason_code": "OK",
                "already_there": True,
                "path_hash": "",
                "rows": [{"pose_name": pose_name,
                          "reason": "already_there",
                          "delta_rad": round(delta, 6)}],
                "legs": [],
                "edges": (),
                "stage_timings": self._stage_timings(started_ns),
            }
        # [S5.7 lock] map admission is fail-closed on the LATEST map —
        # never the selection-time snapshot (verbatim from
        # plan_observation_route).
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
        reset_scene = getattr(self.edge_builder, "reset_scene", None)
        row = {"pose_name": pose_name}
        rows = []
        last_reason = "EDGE_BUILD_FAILED"
        try:
            start_fk = self.fk(list(current))
            start_node = self.graph.register_node(
                current, tuple(start_fk[0]), tuple(start_fk[1]),
                "named_pose_home", LoadState.UNLOADED, "named_pose")
            # Goal joints are the FILE values directly; FK fills the
            # node's pose field — no IK anywhere on this path.
            goal_fk = self.fk(list(values))
            goal_node = self.graph.register_node(
                tuple(float(v) for v in values),
                tuple(goal_fk[0]), tuple(goal_fk[1]),
                "named_pose_%s" % pose_name, LoadState.UNLOADED,
                "named_pose")
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
                return self._fail(code, rows)
            edges = (edge,)
            route = PlannedRoute(
                route_id="route-named-%s" % str(request_id)[:12],
                request_id=str(request_id),
                snapshot_digest=snapshot.digest(),
                payload_digest=UNLOADED_ATTACHMENT_DIGEST,
                attachment_digest=UNLOADED_ATTACHMENT_DIGEST,
                candidate_id="named-pose-%s" % pose_name,
                entry_edge_ids=(edge.edge_id,),
                retreat_edge_ids=(),
                edge_digests=(edge.digest(),),
                trajectory_digest=edge.trajectory.digest(),
                planning_revision="named_pose")
            revalidation = self._revalidate(route, snapshot, edges)
            if revalidation is None:
                row["reason"] = "MAP_REVALIDATION_UNAVAILABLE"
                rows.append(row)
                return self._fail(row["reason"], rows)
            from st09_contracts.certification import MapDecision

            if revalidation.decision is not MapDecision.CONTINUE:
                row["reason"] = getattr(
                    revalidation.reason, "value", "MAP_REVISION_MISMATCH")
                rows.append(row)
                return self._fail(row["reason"], rows)
            certification, failure = self._certify(
                route, snapshot, None, edges, None, None,
                load_state=LoadState.UNLOADED)
            if certification is None or not certification.passed:
                row["reason"] = failure if certification is None \
                    else certification.reason.value
                row["recertified"] = False
                rows.append(row)
                return self._fail(row["reason"], rows)
            legs = leg_records(route, edges, request)
            row["reason"] = "OK"
            row["certified"] = True
            rows.append(row)
            self.last_route = route
            self.last_certification = certification
            self.last_revalidation = revalidation
            self.last_rows = rows
            self._promote_selection({
                "path_hash": route.digest(),
                "route": route,
                "edges": edges,
                "snapshot": snapshot,
                "attachment": None,
                "candidate": None,
                "request": request,
                "certification_id": certification.digest(),
                "load_state": LoadState.UNLOADED,
                "purpose": "named_pose:%s" % pose_name,
            })
            return {
                "success": True,
                "reason_code": "OK",
                "already_there": False,
                "selection_id": route.route_id,
                "path_source": "route-graph",
                "path_hash": route.digest(),
                "rows": rows,
                "route": route,
                "edges": edges,
                "legs": legs,
                "certification": certification,
                "revalidation": revalidation,
                "stage_timings": self._stage_timings(started_ns),
            }
        finally:
            if callable(reset_scene):
                reset_scene()

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

    def _goal_seed_fn(self, start_q, branch_q=None, atlas_seeds=None,
                      anchor_qs=None):
        """Goal seeds from the offline refine (B requires injected seeds).

        Each slot_upper hover q is refined onto the exact target first,
        and that attempt does not fall back to the tool-down seed. Those
        solutions are the same-heading, same-branch descend goals. Atlas
        seeds, the tool-down prior, and the live start remain as
        alternatives for a branch whose own edge collides. Pair order
        (same heading, then same branch, pi and cross-elbow last) is
        applied by the route search; this function only makes the
        continuous goal exist. A seed is never a verdict.
        """
        from luggage_planning.place_targets import GoalSeed

        anchors = []
        seen = set()
        for q in list(anchor_qs or []):
            key = tuple(float(v) for v in q)
            if key not in seen:
                anchors.append(("anchor", key))
                seen.add(key)
        if branch_q is not None:
            key = tuple(float(v) for v in branch_q)
            if key not in seen:
                anchors.append(("anchor", key))
                seen.add(key)
        alternatives = [("atlas", tuple(float(v) for v in q))
                        for q in (atlas_seeds or [])]
        alternatives += [("tool_down", tuple(float(v) for v in TOOL_DOWN_SEED_Q)),
                         ("start", tuple(float(v) for v in start_q))]
        self.last_goal_refine = []

        def goal_seeds(target):
            seeds = []
            attempts = []
            for label, seed in anchors + alternatives:
                if label == "anchor":
                    q, info = self._refine_exact(
                        target.position, target.quaternion_xyzw, seed)
                else:
                    q, info = self._refine_to(
                        target.position, target.quaternion_xyzw, seed)
                info = info if isinstance(info, dict) else {}
                attempts.append({
                    "seed": label,
                    "ok": q is not None,
                    "pos_err_m": info.get("pos_err_m"),
                    "rot_err_rad": info.get("rot_err_rad"),
                })
                if q is not None:
                    seeds.append(GoalSeed(
                        tuple(q), "refine-%s" % label,
                        "route_producer:offline_refine"))
            self.last_goal_refine.append({
                "target_world": [
                    [float(v) for v in target.position],
                    [float(v) for v in target.quaternion_xyzw]],
                "atlas_seed_count": len(atlas_seeds or []),
                "attempts": attempts,
            })
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
        stages = getattr(self, "_stage_ns", None)
        if stages is not None:
            stages["certify"] += self._clock_ns() - started_ns
        self._log("route: certify wall %.3fs edges=%d" % (
            elapsed, len(edges)))
        if elapsed > CERTIFICATION_TIMEOUT_S:
            return None, "CERTIFICATION_TIMEOUT"
        return result, None

    def _revalidate(self, route, snapshot, edges):
        if self.revalidator is None:
            return None
        import time as _reval_time

        _reval_t0 = _reval_time.monotonic_ns()
        from luggage_planning.map_revalidation import RevalidationInput

        latest = snapshot
        fetch = self.latest_snapshot_fn
        if callable(fetch):
            fetched = fetch()
            if fetched is None:
                return None  # named MAP_REVALIDATION_UNAVAILABLE upstream
            latest = fetched
        self.last_latest_snapshot = latest
        try:
            return self.revalidator.revalidate(RevalidationInput(
                route=route, planned_snapshot=snapshot,
                latest_snapshot=latest, edges=edges,
                now_ns=self._clock_ns(), in_flight=False))
        finally:
            stages = getattr(self, "_stage_ns", None)
            if stages is not None:
                stages["revalidate"] += \
                    _reval_time.monotonic_ns() - _reval_t0

    def _close_row(self, rows, row):
        """Stamp elapsed time, seed source, and recent Cartesian refusals."""
        started = row.pop("_started_ns", None)
        if started is not None:
            row["elapsed_ms"] = round(
                (self._clock_ns() - int(started)) / 1e6, 1)
        if "seed_source" not in row:
            sources = []
            for record in getattr(self, "last_goal_refine", None) or []:
                for attempt in record.get("attempts") or []:
                    if attempt.get("ok"):
                        sources.append(attempt.get("seed"))
            if sources:
                row["seed_source"] = sources
        log = getattr(getattr(self, "edge_builder", None),
                      "edge_failure_log", None)
        if callable(log) and "cartesian" not in row:
            cartesian = [
                item for item in log()
                if item.get("reason") == "fraction_below_min"]
            if cartesian:
                row["cartesian"] = cartesian[-4:]
        rows.append(row)
        return row

    def _map_admission(self, revalidation):
        """Identity the freshness gate actually judged."""
        detail = getattr(revalidation, "detail", None) or {}
        freshness = {}
        if isinstance(detail, Mapping):
            raw = detail.get("freshness") or {}
            if isinstance(raw, Mapping):
                freshness = raw
        latest = getattr(self, "last_latest_snapshot", None)
        source = freshness.get("map_source")
        if source is None:
            source = getattr(latest, "map_source", "") or ""
        placed = freshness.get("placed_count")
        if placed is None:
            placed = getattr(latest, "placed_count", 0) or 0
        stamp = freshness.get("last_acquisition_stamp_ns")
        if stamp is None:
            stamp = getattr(latest, "acquisition_stamp_ns", 0) or 0
        return {
            "map_source": str(source or ""),
            "placed_count": int(placed or 0),
            "last_acquisition_stamp_ns": int(stamp or 0),
            "freshness_policy": str(freshness.get("freshness_policy") or ""),
            "why_allowed": str(freshness.get("why_allowed") or ""),
            "why_refused": str(freshness.get("why_refused") or ""),
        }

    def _map_refusal(self, snapshot, revalidation, route, edges, request):
        """Why a planned route was not admitted, plus the legs already built."""
        latest = getattr(self, "last_latest_snapshot", None)
        now_ns = int(self._clock_ns())
        latest_stamp = int(getattr(latest, "acquisition_stamp_ns", 0) or 0)
        planned_stamp = int(getattr(snapshot, "acquisition_stamp_ns", 0) or 0)
        policy = getattr(getattr(self, "revalidator", None), "policy", None)
        legs = []
        for edge in edges or ():
            trajectory = getattr(edge, "trajectory", None)
            positions = list(getattr(trajectory, "positions", []) or [])
            legs.append({
                "edge_id": str(getattr(edge, "edge_id", "")),
                "name": str((getattr(edge, "attributes", {}) or {}).get(
                    "edge_class") or ""),
                "digest": edge.digest() if hasattr(edge, "digest") else "",
                "n_points": len(positions),
                "joint_names": list(getattr(trajectory, "joint_names", [])
                                    or []),
                "positions": [[float(v) for v in point] for point in positions],
            })
        refusal = {
            "route_id": str(getattr(route, "route_id", "") or ""),
            "planned_map_revision": int(getattr(snapshot, "map_revision", 0)
                                        or 0),
            "planned_acquisition_stamp_ns": planned_stamp,
            "request_map_revision": int((request or {}).get("map_revision")
                                        or 0),
            "latest_map_revision": int(getattr(revalidation, "map_revision", 0)
                                       or 0),
            "latest_acquisition_stamp_ns": latest_stamp,
            "live_map_revision": int(getattr(latest, "map_revision", 0) or 0),
            "now_ns": now_ns,
            # Stamp 0 is "never observed", not time zero. Age is only
            # meaningful once a real acquisition stamp exists.
            "age_ns": (now_ns - latest_stamp) if latest_stamp > 0 else None,
            "stamp_valid": latest_stamp > 0,
            "stale_window_ns": int(getattr(policy, "max_snapshot_age_ns", 0)
                                   or 0),
            "legs": legs,
        }
        refusal.update(self._map_admission(revalidation))
        return refusal

    def _attempt_summary(self, ordered_slots, tried_indices):
        """Whether a shallower seeded slot was still waiting."""
        tried = set(tried_indices or [])
        seeded = [entry for entry in ordered_slots if entry[2]]
        tried_depths = [
            float(entry[3]) for entry in ordered_slots if entry[0] in tried]
        shallowest_tried = min(tried_depths) if tried_depths else None
        untried_shallower = [
            entry[0] for entry in seeded
            if entry[0] not in tried and (
                shallowest_tried is None or float(entry[3]) < shallowest_tried)]
        seeded_depths = [float(entry[3]) for entry in seeded]
        return {
            "reason": "ATTEMPT_SUMMARY",
            "n_ordered": len(ordered_slots),
            "n_seeded": len(seeded),
            "tried_slot_indices": list(tried_indices or []),
            "untried_seeded": sum(
                1 for entry in seeded if entry[0] not in tried),
            "shallowest_seeded_depth_m": (
                min(seeded_depths) if seeded_depths else None),
            "shallowest_tried_depth_m": shallowest_tried,
            "untried_shallower_slot_indices": untried_shallower,
        }

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


def _live_count(provider) -> int:
    counter = getattr(provider, "live_count", None)
    if callable(counter):
        return int(counter())
    records = getattr(provider, "_records", None)
    if not isinstance(records, dict):
        return 0
    from st09_contracts.attachment import AttachmentValidity
    return sum(
        1 for record in records.values()
        if getattr(record, "validity", None) is AttachmentValidity.VALID)


def _record_snapshot(record):
    """Identity of the record admission saw, before expiry replaces it."""
    if record is None or not hasattr(record, "acquisition_stamp_ns"):
        return None
    digest = getattr(record, "digest", None)
    return {
        "record_stamp_ns": int(record.acquisition_stamp_ns),
        "box_id": str(getattr(record, "box_id", "") or ""),
        "attachment_digest": str(digest()) if callable(digest) else "",
        "attachment_incarnation": int(
            getattr(record, "attachment_incarnation", 0) or 0),
    }


def attachment_invalidation_evidence(snapshot, *, now_ns, records_before,
                                     records_after, generation,
                                     invalidation_reason="",
                                     request_box_id="",
                                     request_generation=0):
    """Selector row: why the frozen attach was or was not still live."""
    from st09_contracts.attachment import ATTACHMENT_MAX_AGE_NS
    stamp = None if snapshot is None else int(snapshot["record_stamp_ns"])
    age = None if stamp is None else int(now_ns) - stamp
    return {
        "record_stamp_ns": stamp,
        "now_stamp_ns": int(now_ns),
        "age_ns": age,
        "stale_window_ns": int(ATTACHMENT_MAX_AGE_NS),
        "box_id": "" if snapshot is None else str(snapshot["box_id"]),
        "generation": None if generation is None else int(generation),
        "attachment_digest": (
            "" if snapshot is None else str(snapshot["attachment_digest"])),
        "attachment_incarnation": (
            None if snapshot is None else int(
                snapshot["attachment_incarnation"])),
        "records_before": int(records_before),
        "records_after": int(records_after),
        "invalidation_reason": str(invalidation_reason or ""),
        "request_box_id": str(request_box_id or ""),
        "request_generation": int(request_generation or 0),
    }


def _attachment_identity_reason(record, request, stored_generation):
    """Refuse a live record whose box or generation is not this select."""
    from st09_contracts.attachment import AttachmentValidity
    if getattr(record, "validity", None) is not AttachmentValidity.VALID:
        return None
    if not hasattr(record, "box_id"):
        return None
    wanted_box = str(request.get("box_id") or "")
    if wanted_box and wanted_box != str(record.box_id):
        return "ATTACHMENT_REQUEST_MISMATCH"
    wanted_generation = int(request.get("generation") or 0)
    if not wanted_generation:
        return None
    if stored_generation is None or int(stored_generation) != wanted_generation:
        return "ATTACHMENT_REQUEST_MISMATCH"
    return None


def admit_attachment_for_route(provider, request, now_ns):
    """Admit the frozen attach for one select.

    Returns ``(record, reason, evidence)``. A zero stamp is
    ``STAMP_INVALID`` and is not a live record. A box_id or generation
    that disagrees with the frozen record is
    ``ATTACHMENT_REQUEST_MISMATCH``. Expiry still surfaces as
    ``ATTACHMENT_MISSING``, with the age fields on ``evidence``.
    """
    from st09_contracts.attachment import AttachmentValidity
    from st09_contracts.reasons import Reason

    request_id = str(request["request_id"])
    now_ns = int(now_ns)
    record_for = getattr(provider, "record_for", None)
    prior = record_for(request_id) if callable(record_for) else None
    snapshot = _record_snapshot(prior)
    before = _live_count(provider)
    generation_for = getattr(provider, "generation_for", None)
    generation = (
        generation_for(request_id) if callable(generation_for) else None)
    expire = getattr(provider, "expire_stale", None)
    if callable(expire):
        expire(request_id, now_ns)
    after_record = record_for(request_id) if callable(record_for) else None
    after = _live_count(provider)
    invalidation = ""
    if after_record is not None:
        invalidation = str(
            getattr(after_record, "invalidation_reason", "") or "")
    evidence = attachment_invalidation_evidence(
        snapshot, now_ns=now_ns, records_before=before,
        records_after=after, generation=generation,
        invalidation_reason=invalidation,
        request_box_id=str(request.get("box_id") or ""),
        request_generation=int(request.get("generation") or 0))
    if invalidation == Reason.STAMP_INVALID.value:
        evidence["reason"] = Reason.STAMP_INVALID.value
        return None, Reason.STAMP_INVALID.value, evidence
    if (after_record is not None and getattr(
            after_record, "validity", None) is AttachmentValidity.VALID):
        mismatch = _attachment_identity_reason(
            after_record, request, generation)
        if mismatch:
            evidence["reason"] = mismatch
            evidence["invalidation_reason"] = mismatch
            return None, mismatch, evidence
    try:
        return provider.latest_record(request_id), None, evidence
    except (ContractViolation, LookupError, ValueError) as exc:
        reason = _named_reason(exc)
        evidence["reason"] = reason
        return None, reason, evidence


def register_attach_from_request(provider, request: Mapping,
                                 *, box_wdh: Sequence[float],
                                 cargo_id: str = "", box_id: str = "",
                                 cycle_id: str = "",
                                 generation: Optional[int] = None,
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
    bind = getattr(provider, "bind_generation", None)
    if callable(bind):
        bind(str(request["request_id"]), generation)
