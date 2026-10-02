"""Offline fixtures shared by the ST09 module tasks (C0 freeze).

Everything here is synthetic and ROS-free: no GT of any real scene, no
saved bag data. The fixture exists so D/B can develop against the frozen
contracts without waiting for each other (plan section 3 item 7).
Evidence-grade inputs (the 24 saved slanted-wall candidates, the
rehearsal-52 dumps) live in the checkpoint evidence directories and are
referenced from ``st09_acceptance_manifest.json`` — they are not
duplicated here.

Naming rule: anything that fakes a runtime component carries the
``Fake`` prefix so it can never be mistaken for a real implementation.
"""

from __future__ import annotations

import math
from typing import Mapping, Optional, Sequence, Tuple

from st09_contracts._base import ContractViolation, digest_of
from st09_contracts.attachment import (
    ATTACHMENT_SCHEMA_VERSION,
    AttachmentInvalidation,
    AttachmentRecord,
    AttachmentSourceKind,
    AttachmentValidity,
    BoxTarget,
    compose_transform,
    invert_transform,
    resolve_place_targets,
)
from st09_contracts.certification import (
    CertificationResult,
    CheckDomain,
    EffectCounters,
    EffectTrace,
    MapDecision,
    MapRevalidationDecision,
)
from st09_contracts.graph import (
    GraphEdge,
    GraphNode,
    LoadState,
    PlannedRoute,
    TimedTrajectory,
)
from st09_contracts.planning import (
    OccupancyCellState,
    PayloadIdentity,
    PayloadSymmetry,
    PlacementCandidate,
    GeometryVerdict,
    ObservationRequirement,
    PlanningSnapshot,
)
from st09_contracts.reasons import Reason

# -- container geometry (synthetic; encodes the plan §1 assumptions) ----
# Opening on the container -X side, inward normal +X. Generic code always
# reads INWARD_NORMAL; nothing may hardcode the picture's +Y.
CONTAINER_INNER_HALF = (0.485, 0.985, 0.60)
OPENING_CENTER = (-0.485, 0.0, 0.30)
INWARD_NORMAL = (1.0, 0.0, 0.0)
SURFACE_RESOLUTION_M = 0.05
GRID_I = 20
GRID_J = 12

# rehearsal-52 box-1 measured dimensions (evidence fixture reference).
BOX_WDH = (0.646, 0.409, 0.277)

SIM_SCENE = "sim-scene-20261002"
SIM_CALIBRATION = "sim-urdf-embedded-d435"
DEPLOY_SCENE = "deploy-scene-20261002"
DEPLOY_CALIBRATION = "deploy-handeye-20260928"


def _cells(free: Sequence[Tuple[int, int]] = (),
           occupied: Sequence[Tuple[int, int]] = (),
           default=OccupancyCellState.UNKNOWN) -> Tuple:
    cells_i, cells_j, states = [], [], []
    for i in range(GRID_I):
        for j in range(GRID_J):
            cell = (i, j)
            if cell in set(free):
                state = OccupancyCellState.FREE
            elif cell in set(occupied):
                state = OccupancyCellState.OCCUPIED
            else:
                state = default
            cells_i.append(i)
            cells_j.append(j)
            states.append(state.value)
    return tuple(cells_i), tuple(cells_j), tuple(states)


def make_snapshot(
    map_revision: int,
    observed_free: Sequence[Tuple[int, int]] = (),
    occupied: Sequence[Tuple[int, int]] = (),
    scene_revision: str = SIM_SCENE,
    acquisition_stamp_ns: int = 1_000_000_000,
    commit_revision: int = 1,
) -> PlanningSnapshot:
    """Snapshot with an observed-free window near the opening plus blocks.

    Revision >= 2 models one integrated observation pass: the opening
    half is FREE (observed), everything else stays UNKNOWN, and placed
    boxes are OCCUPIED.
    """
    cells = _build_cells(map_revision, observed_free, occupied)
    cells_i, cells_j, states = cells
    return PlanningSnapshot(
        geometry_id="fixture-container-v1",
        geometry_hash=digest_of(CONTAINER_INNER_HALF, OPENING_CENTER, INWARD_NORMAL),
        map_revision=map_revision,
        acquisition_stamp_ns=acquisition_stamp_ns,
        commit_revision=commit_revision,
        occupancy_digest=digest_of(cells_i, cells_j, states),
        resolution_m=SURFACE_RESOLUTION_M,
        origin_xy=(-0.485, -0.60),
        cells_i=cells_i,
        cells_j=cells_j,
        cell_states=states,
    )


def _build_cells(map_revision, observed_free, occupied):
    if map_revision >= 2:
        free = set(observed_free)
        for i in range(0, GRID_I // 2):
            for j in range(GRID_J):
                free.add((i, j))
        return _cells(free=sorted(free), occupied=occupied)
    return _cells(free=observed_free, occupied=occupied)


def snapshot_pair() -> Tuple[PlanningSnapshot, PlanningSnapshot]:
    """The revision handoff pattern: box 1 integrated between revisions."""
    placed_box_cells = [(14, 5), (14, 6), (15, 5), (15, 6)]
    first = make_snapshot(map_revision=1)
    second = make_snapshot(
        map_revision=2,
        occupied=placed_box_cells,
        acquisition_stamp_ns=2_000_000_000,
        commit_revision=2)
    return first, second


# -- attachment fixtures (the four target classes x sim/deploy) ---------

def _quaternion_z(theta: float) -> Tuple[float, float, float, float]:
    half = 0.5 * theta
    return (0.0, 0.0, math.sin(half), math.cos(half))


def _quaternion_x(theta: float) -> Tuple[float, float, float, float]:
    half = 0.5 * theta
    return (math.sin(half), 0.0, 0.0, math.cos(half))


def make_attachment(
    kind: str,
    *,
    scene_revision: str = SIM_SCENE,
    calibration_id: str = SIM_CALIBRATION,
    symmetry: str = "pi",
    stamp_ns: int = 1_000_000_000,
    request_id: str = "req-fixture-1",
    incarnation: int = 0,
    dimensions_wdh: Tuple[float, float, float] = BOX_WDH,
    source: AttachmentSourceKind = AttachmentSourceKind.MEASURED_ATTACH_CONFIRMED,
    validity: AttachmentValidity = AttachmentValidity.VALID,
    invalidation_reason: Optional[str] = None,
) -> AttachmentRecord:
    """``centered`` / ``eccentric`` / ``relative_yaw`` / ``tilt`` records.

    The stored transform is T_tcp_box (box coords into TCP coords),
    built by inverting the easy-to-author T_box_tcp.
    """
    h = dimensions_wdh[2]
    if kind == "centered":
        t_box_tcp = (0.0, 0.0, 0.5 * h)
        q_box_tcp = _quaternion_z(0.0)
    elif kind == "eccentric":
        t_box_tcp = (0.15, 0.08, 0.5 * h)
        q_box_tcp = _quaternion_z(0.0)
    elif kind == "relative_yaw":
        t_box_tcp = (0.0, 0.0, 0.5 * h)
        q_box_tcp = _quaternion_z(0.4)
    elif kind == "tilt":
        t_box_tcp = (0.0, 0.0, 0.5 * h)
        q_box_tcp = compose_transform(
            (0.0, 0.0, 0.0), _quaternion_z(0.2),
            (0.0, 0.0, 0.0), _quaternion_x(0.03))[1]
    else:
        raise ContractViolation("kind: centered/eccentric/relative_yaw/tilt (%r)" % (kind,))
    t_tcp_box, q_tcp_box = invert_transform(t_box_tcp, q_box_tcp)
    return AttachmentRecord(
        schema_version=ATTACHMENT_SCHEMA_VERSION,
        cargo_id="cargo-fixture-%s" % kind,
        box_id="box-fixture-%s" % kind,
        request_id=request_id,
        cycle_id="cycle-fixture-1",
        attachment_incarnation=incarnation,
        tcp_frame="suction_contact_frame",
        box_frame="box",
        dimensions_wdh=dimensions_wdh,
        symmetry=symmetry,
        t_tcp_box_translation=t_tcp_box,
        t_tcp_box_quaternion_xyzw=q_tcp_box,
        acquisition_stamp_ns=stamp_ns,
        scene_revision=scene_revision,
        calibration_id=calibration_id,
        source=source,
        source_method="fixture:same-stamp-measured" if source.measured else "fixture:reference",
        evidence_digest=digest_of(kind, dimensions_wdh, stamp_ns),
        validity=validity,
        invalidation_reason=invalidation_reason,
    )


def target_fixture_matrix():
    """(kind, scene flavour) pairs covering the 8 frozen target fixtures."""
    out = []
    for kind in ("centered", "eccentric", "relative_yaw", "tilt"):
        for scene, calibration in ((SIM_SCENE, SIM_CALIBRATION),
                                   (DEPLOY_SCENE, DEPLOY_CALIBRATION)):
            out.append((kind, scene, calibration))
    return out


def make_box_target(
    attachment: AttachmentRecord,
    *,
    heading_theta: float = 0.0,
    position=(0.20, -0.10, 0.5 * BOX_WDH[2]),
    stamp_ns: int = 2_000_000_000,
    candidate_id: str = "cand-fixture-1",
) -> BoxTarget:
    """A deep container target: depth along +X from the opening centre."""
    depth = sum((position[i] - OPENING_CENTER[i]) * INWARD_NORMAL[i] for i in range(3))
    assert depth > 0.0
    quat = _quaternion_z(heading_theta)
    return BoxTarget(
        candidate_id=candidate_id,
        request_id=attachment.request_id,
        frame_id="world",
        scene_revision=attachment.scene_revision,
        heading_theta=heading_theta,
        position=tuple(float(v) for v in position),
        quaternion_xyzw=quat,
        stamp_ns=stamp_ns)


def make_payload(attachment: AttachmentRecord) -> PayloadIdentity:
    return PayloadIdentity(
        cargo_id=attachment.cargo_id,
        box_id=attachment.box_id,
        request_id=attachment.request_id,
        cycle_id=attachment.cycle_id,
        dimensions_wdh=attachment.dimensions_wdh,
        symmetry=PayloadSymmetry(attachment.symmetry),
        t_tcp_box_translation=attachment.t_tcp_box_translation,
        t_tcp_box_quaternion_xyzw=attachment.t_tcp_box_quaternion_xyzw,
        attachment_incarnation=attachment.attachment_incarnation,
        acquisition_stamp_ns=attachment.acquisition_stamp_ns)


class FakeAttachmentProvider:
    """Explicit fixture provider for offline development (not a real one).

    D replaces this with the measured provider; the fake exists so the
    protocol, expiry, incarnation and invalidation semantics can be
    exercised without ROS, TF or a vacuum backend.
    """

    def __init__(self, records: Sequence[AttachmentRecord]):
        self._records = {r.request_id: r for r in records}
        self.invalidations: Tuple[AttachmentInvalidation, ...] = ()

    def latest_record(self, request_id: str) -> AttachmentRecord:
        record = self._records.get(request_id)
        if record is None:
            raise ContractViolation("%s: no record for %s" % (Reason.ATTACHMENT_MISSING.value, request_id))
        if record.validity is not AttachmentValidity.VALID:
            raise ContractViolation(
                "%s: %s" % (Reason.ATTACHMENT_MISSING.value, record.invalidation_reason))
        if not record.source.measured:
            raise ContractViolation(
                "%s: source=%s" % (Reason.ATTACHMENT_GT_AS_MEASURED.value, record.source))
        return record

    def invalidate(self, reason: Reason, detail: Mapping) -> AttachmentInvalidation:
        active = None
        for record in self._records.values():
            if record.validity is AttachmentValidity.VALID:
                active = record
        if active is None:
            raise ContractViolation("%s: nothing to invalidate" % Reason.ATTACHMENT_MISSING.value)
        note = AttachmentInvalidation(
            record_digest=active.digest(),
            box_id=active.box_id,
            request_id=active.request_id,
            incarnation=active.attachment_incarnation,
            reason=reason,
            stamp_ns=active.acquisition_stamp_ns + 1,
            detail=dict(detail))
        replaced = make_attachment(
            "centered",
            scene_revision=active.scene_revision,
            calibration_id=active.calibration_id,
            symmetry=active.symmetry,
            stamp_ns=active.acquisition_stamp_ns,
            request_id=active.request_id,
            incarnation=active.attachment_incarnation,
            dimensions_wdh=active.dimensions_wdh,
            source=active.source,
            validity=AttachmentValidity.INVALIDATED,
            invalidation_reason=reason.value)
        self._records[active.request_id] = replaced
        self.invalidations = self.invalidations + (note,)
        return note


# -- minimal graph / route / certification fixtures ----------------------

def make_timed_trajectory() -> TimedTrajectory:
    """Two-joint-move, three-knot rest-to-rest trajectory (6 joints)."""
    zero = (0.0,) * 6
    mid = list(zero)
    mid[0] = 0.20
    end = list(zero)
    end[0] = 0.40
    end[5] = -0.10
    return TimedTrajectory(
        joint_names=tuple("elfin_joint%d" % i for i in range(1, 7)),
        positions=(zero, tuple(mid), tuple(end)),
        velocities=(zero, (0.2,) + (0.0,) * 5, zero),
        accelerations=(zero, zero, zero),
        time_from_start_ns=(0, 500_000_000, 1_000_000_000))


def make_graph_fixture(attachment: AttachmentRecord):
    """Portal / clearance / slot-approach / exit / return small graph."""
    zero = (0.0,) * 6
    approach = list(zero)
    approach[1] = -0.40
    slot = list(approach)
    slot[0] = 0.35
    nodes = (
        GraphNode("n-home", zero, (0.30, 0.0, 0.55), (0, 0, 0, 1), "outside", LoadState.UNLOADED),
        GraphNode("n-portal", tuple(approach), (-0.30, 0.0, 0.45), (0, 0, 0, 1),
                  "portal", LoadState.CARRYING, ik_family="f1"),
        GraphNode("n-slot", tuple(slot), (0.20, 0.0, 0.42), (0, 0, 0, 1),
                  "slot", LoadState.CARRYING, ik_family="f2"),
    )
    trajectory = make_timed_trajectory()
    edges = (
        GraphEdge("e-entry", "n-home", "n-portal", LoadState.CARRYING,
                  "static-fixture-v1", trajectory),
        GraphEdge("e-insert", "n-portal", "n-slot", LoadState.CARRYING,
                  "static-fixture-v1", trajectory),
        GraphEdge("e-retreat", "n-slot", "n-home", LoadState.UNLOADED,
                  "static-fixture-v1", trajectory),
    )
    return nodes, edges


def make_route(attachment: AttachmentRecord, snapshot: PlanningSnapshot,
               candidate_id: str = "cand-fixture-1") -> PlannedRoute:
    return PlannedRoute(
        route_id="route-fixture-1",
        request_id=attachment.request_id,
        snapshot_digest=snapshot.digest(),
        payload_digest=make_payload(attachment).digest(),
        attachment_digest=attachment.digest(),
        candidate_id=candidate_id,
        entry_edge_ids=("e-entry", "e-insert"),
        retreat_edge_ids=("e-retreat",),
        edge_digests=("digest-entry", "digest-insert", "digest-retreat"),
        trajectory_digest=make_timed_trajectory().digest(),
        planning_revision="c0-fixture")


def make_certification(route: PlannedRoute, passed: bool = True) -> CertificationResult:
    return CertificationResult(
        certification_id="cert-fixture-1",
        route_id=route.route_id,
        passed=passed,
        reason=Reason.OK if passed else Reason.CERT_COLLISION,
        checks=(str(CheckDomain.JOINT_LIMITS), str(CheckDomain.JOINT_CONTINUITY),
                str(CheckDomain.FK), str(CheckDomain.TIME_PARAMETERIZATION)),
        validation_revision=1,
        min_clearance_m=0.05,
        fk_position_error_m=0.0005,
        fk_rotation_error_rad=0.004,
        max_joint_velocity_rad=0.47,
        max_joint_acceleration_rad2=0.94,
        sampling_dt_max_ns=10_000_000,
        sampling_motion_bound_m=0.025,
        method_note="fixture: knot sampling bounds reported")


def make_revalidation(decision: MapDecision = MapDecision.CONTINUE) -> MapRevalidationDecision:
    return MapRevalidationDecision(
        decision=decision,
        reason=Reason.OK if decision is MapDecision.CONTINUE else Reason.MAP_REVISION_MISMATCH,
        map_revision=2,
        validation_epoch=1,
        affected_edge_ids=() if decision is MapDecision.CONTINUE else ("e-insert",))


def make_effect_trace(phase: str = "pre_pick_admission",
                      allowed_cancel_hold: bool = False) -> EffectTrace:
    before = EffectCounters(plan_calls=1, execute_calls=0, attach_calls=0)
    after = EffectCounters(plan_calls=1, execute_calls=0, attach_calls=0)
    return EffectTrace(
        trace_id="trace-fixture-1",
        causal_sequence=1,
        request_id="req-fixture-1",
        route_id="route-fixture-1",
        phase=phase,
        stamp_ns=1_000_000_000,
        before=before,
        after=after,
        allowed_cancel_hold=allowed_cancel_hold)


def resolved_targets_for(kind: str, attachment: Optional[AttachmentRecord] = None):
    """resolve_place_targets over the fixture target (all headings)."""
    record = attachment or make_attachment(kind)
    target = make_box_target(record)
    return resolve_place_targets(target, record)
