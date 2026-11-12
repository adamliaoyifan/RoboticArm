"""Bounded complete-operation assembly with independent native planning ports.

The candidate provider owns measured/full-footprint admission and solver slots.
Local branches, middle connections, timing and final whole-chain certification
are separate boundaries. This module has no controller, vacuum or map-write port.
It never converts finite search failure into physical unreachability.
"""
from dataclasses import dataclass, replace
import hashlib
import math
import time

from luggage_planning.joint_operation_contracts import (
    JointKnot, JointOperationPlan, OperationBarrier, OperationIdentity, OperationLeg,
    OPERATION_JOINTS, operation_digest, seal_operation_plan,
)
from luggage_planning.joint_operation_scenes import OperationScenes
from luggage_planning.joint_operation_connection import GeometricConnection
from luggage_planning.placement_prior_contracts import Transform, finite_tuple
from luggage_planning.trajectory_timing import time_parameterize_cartesian
from st09_contracts.attachment_geometry import AttachmentGeometry


def geometric_rows(rows):
    result = tuple(finite_tuple(q, 6, "geometric joint row") for q in rows)
    if not 2 <= len(result) <= 50000:
        raise ValueError("bounded path with at least two joint rows required")
    return result


@dataclass(frozen=True)
class LocalJointBranch:
    branch_id: str
    incoming: tuple
    outgoing: tuple

    def __post_init__(self):
        if not isinstance(self.branch_id, str) or not self.branch_id:
            raise ValueError("local branch identity required")
        for name in ("incoming", "outgoing"):
            object.__setattr__(self, name, geometric_rows(getattr(self, name)))
        if self.incoming[-1] != self.outgoing[0]:
            raise ValueError("local contact winding/branch discontinuity")


@dataclass(frozen=True)
class JointSeedHint:
    """An untrusted IK proposal, retaining its atlas/online branch provenance."""
    seed_id: str
    joints: tuple

    def __post_init__(self):
        if not isinstance(self.seed_id, str) or not self.seed_id.strip():
            raise ValueError('seed provenance required')
        object.__setattr__(self, 'joints', finite_tuple(self.joints, 6, 'entry seed'))


@dataclass(frozen=True)
class OperationCandidate:
    candidate_id: str
    world_from_pick_box: Transform
    world_from_pick_tool: Transform
    world_from_release_box: Transform
    world_from_release_tool: Transform
    world_from_settled_box: Transform
    predicted_attachment: AttachmentGeometry
    scenes: OperationScenes
    # Entry/lift poses are used by the endpoint port and endpoint FK certifier.
    pick_approach: Transform
    pick_lift: Transform
    place_entry: Transform
    place_retreat: Transform
    pick_entry_seeds: tuple = ()
    place_entry_seeds: tuple = ()

    def __post_init__(self):
        if not isinstance(self.candidate_id, str) or not self.candidate_id.strip():
            raise ValueError("candidate identity required")
        if type(self.predicted_attachment) is not AttachmentGeometry or type(self.scenes) is not OperationScenes:
            raise ValueError("predicted geometry and phase snapshots required")
        for name in ("world_from_pick_box", "world_from_pick_tool", "world_from_release_box",
                     "world_from_release_tool", "world_from_settled_box", "pick_approach",
                     "pick_lift", "place_entry", "place_retreat"):
            if type(getattr(self, name)) is not Transform:
                raise ValueError("explicit world Transform required: "+name)
        for name in ('pick_entry_seeds', 'place_entry_seeds'):
            seeds = tuple(getattr(self, name))
            if (len(seeds) > 3 or any(type(seed) is not JointSeedHint for seed in seeds)
                    or len({seed.seed_id for seed in seeds}) != len(seeds)):
                raise ValueError('at most three distinct typed entry seeds required')
            object.__setattr__(self, name, seeds)


@dataclass(frozen=True)
class LocalEndpointEvidence:
    candidate_id: str
    model_sha256: str
    phase_scene_digests: tuple
    attachment_digest: str
    complete: bool
    reason: str
    pick_branches: tuple = ()
    place_branches: tuple = ()

    def __post_init__(self):
        for name in ("phase_scene_digests", "pick_branches", "place_branches"):
            object.__setattr__(self, name, tuple(getattr(self, name)))


@dataclass(frozen=True)
class OperationCertification:
    identity: OperationIdentity
    candidate_id: str
    draft_digest: str
    phase_scene_digests: tuple
    passed: bool
    reason: str
    certificate_id: str = ""


@dataclass(frozen=True)
class PreparationResult:
    status: str
    plan: JointOperationPlan | None
    trace: tuple
    elapsed_s: float


def timed_operation_leg(name, phase, positions, identity, velocity_scale, acceleration_scale):
    rows = geometric_rows(positions)
    timing = time_parameterize_cartesian(rows, velocity_scale, acceleration_scale)
    knots = tuple(JointKnot(q, tuple(v), tuple(a), (), round(t*1e9))
                  for q, v, a, t in zip(rows, timing.velocities_rad,
                                       timing.accelerations_rad, timing.times_s))
    return OperationLeg(name, phase, identity.request_id, identity.scene_revision,
                        identity.robot_state_stamp_ns, OPERATION_JOINTS, knots)


def _connection_rows(connection, start, goal):
    if (type(connection) is not GeometricConnection or connection.success is not True
            or connection.complete is not True):
        return None
    try:
        rows = geometric_rows(connection.positions)
    except (TypeError, ValueError):
        return None
    if rows[0] != start or rows[-1] != goal:
        return None
    if any(max(abs(a-b) for a, b in zip(left, right)) > .01+1e-10
           for left, right in zip(rows, rows[1:])):
        return None
    return rows


class JointOperationPlanner:
    """Prepare from admitted candidates; no execution or predicted-as-actual shim.

    endpoint_port.solve(candidate, request_id=..., ...) returns both complete local branch sets.
    connection_port.connect(snapshot, start, goal, request_id=..., ...) uses the
    exact phase snapshot. certification_port.certify(draft, candidate, ...) checks
    timed controller samples, endpoint FK and all phase scenes before issuing its
    receipt. All ports share a deadline and cancellation; ports retain in-flight
    backend calls so timed-out work cannot accumulate behind later candidates.
    """
    def __init__(self, endpoint_port, connection_port, certification_port, *,
                 velocity_scale, acceleration_scale, certification_reserve_s,
                 max_candidates=8, max_branch_attempts=3,
                 max_branch_attempts_per_candidate=None, clock=time.monotonic):
        for name, value in (("velocity_scale", velocity_scale),
                            ("acceleration_scale", acceleration_scale)):
            if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value <= 1:
                raise ValueError(name+" must be in (0,1]")
        if (not math.isfinite(certification_reserve_s) or certification_reserve_s <= 0
                or type(max_candidates) is not int or not 1 <= max_candidates <= 32
                or type(max_branch_attempts) is not int or not 1 <= max_branch_attempts <= 32):
            raise ValueError("explicit finite budgets and bounded attempt counts required")
        if (max_branch_attempts_per_candidate is not None and
                (type(max_branch_attempts_per_candidate) is not int
                 or not 1 <= max_branch_attempts_per_candidate <= max_branch_attempts)):
            raise ValueError('candidate branch cap must fit the global attempt budget')
        self.endpoints, self.connections, self.certification = endpoint_port, connection_port, certification_port
        self.velocity_scale, self.acceleration_scale = velocity_scale, acceleration_scale
        self.reserve_s = float(certification_reserve_s)
        self.max_candidates, self.max_branch_attempts = max_candidates, max_branch_attempts
        self.candidate_branch_cap = max_branch_attempts_per_candidate or max_branch_attempts
        self.clock = clock

    def prepare(self, identity, start_joints, candidates, *, budget_s, cancelled=lambda: False,
                absolute_deadline=None):
        if not isinstance(identity, OperationIdentity) or not math.isfinite(budget_s) or budget_s <= 0:
            raise ValueError("operation identity and finite positive budget required")
        start = finite_tuple(start_joints, 6, "actual planning start")
        began = self.clock()
        deadline = began+budget_s
        if absolute_deadline is not None:
            if not math.isfinite(absolute_deadline):
                raise ValueError('finite absolute operation deadline required')
            deadline = min(deadline,absolute_deadline)
        planning_deadline = deadline-self.reserve_s
        trace, attempts = [], 0
        from luggage_planning.preparation_profile import StageTimings
        timing = StageTimings('native_planner', trace, clock=self.clock)

        def stopped(limit):
            return cancelled() or self.clock() >= limit

        def finish(status, plan=None):
            return PreparationResult(status, plan, tuple(trace), self.clock()-began)

        for candidate_index, candidate in enumerate(candidates):
            timing.unit = getattr(candidate, 'candidate_id', '<invalid>')
            if stopped(planning_deadline):
                return finish("CANCELLED" if cancelled() else "SEARCH_INCOMPLETE")
            if candidate_index >= self.max_candidates:
                trace.append(("budget", "CANDIDATE_CAP"))
                break
            if not self._candidate_matches(candidate, identity):
                trace.append((getattr(candidate, 'candidate_id', '<invalid>'), "CANDIDATE_IDENTITY_MISMATCH"))
                continue
            evidence = timing.call('local_paths',self.endpoints.solve,candidate, request_id=identity.request_id,
                deadline=planning_deadline, cancelled=cancelled)
            if stopped(planning_deadline):
                return finish("CANCELLED" if cancelled() else "SEARCH_INCOMPLETE")
            if not self._evidence_matches(evidence, candidate, identity):
                trace.append((candidate.candidate_id, "LOCAL_EVIDENCE_REFUSED",
                              getattr(evidence, 'reason', 'INVALID_EVIDENCE_TYPE')))
                continue
            # Both branch sets remain distinct. A failed carrying connection may
            # be recovered by another pick q at exactly the same spatial grasp.
            alignment = {}
            candidate_attempts = 0
            for pick in evidence.pick_branches:
                for place in evidence.place_branches:
                    if (self.candidate_branch_cap < self.max_branch_attempts
                            and candidate_attempts >= self.candidate_branch_cap):
                        break
                    if stopped(planning_deadline) or attempts >= self.max_branch_attempts:
                        trace.append((candidate.candidate_id, "BRANCH_CAP_OR_DEADLINE"))
                        return finish("CANCELLED" if cancelled() else "SEARCH_INCOMPLETE")
                    attempts += 1
                    candidate_attempts += 1
                    branch_id = candidate.candidate_id+":"+pick.branch_id+":"+place.branch_id
                    if pick.branch_id not in alignment:
                        response = timing.call('align_connection',self.connections.connect,
                            candidate.scenes.pick, start, pick.incoming[0],
                            request_id=identity.request_id+":"+branch_id+":align",
                            deadline=planning_deadline, cancelled=cancelled)
                        alignment[pick.branch_id] = _connection_rows(response, start, pick.incoming[0])
                    align = alignment[pick.branch_id]
                    if stopped(planning_deadline):
                        return finish("CANCELLED" if cancelled() else "SEARCH_INCOMPLETE")
                    if align is None:
                        trace.append((branch_id, "ALIGN_CONNECTION_MISS"))
                        # Place alternatives cannot repair this pick approach.
                        # Preserve the remaining budget for another pick branch.
                        break
                    response = timing.call('carry_connection',self.connections.connect,candidate.scenes.carry,
                        pick.outgoing[-1], place.incoming[0],
                        request_id=identity.request_id+":"+branch_id+":carry",
                        deadline=planning_deadline, cancelled=cancelled)
                    if stopped(planning_deadline):
                        return finish("CANCELLED" if cancelled() else "SEARCH_INCOMPLETE")
                    carrying = _connection_rows(response, pick.outgoing[-1], place.incoming[0])
                    if carrying is None:
                        trace.append((branch_id, "CARRY_CONNECTION_MISS"))
                        continue
                    paths = (("align", "pick", align), ("pick_contact", "pick", pick.incoming),
                             ("pick_lift", "carry", pick.outgoing), ("carry", "carry", carrying),
                             ("place_release", "carry", place.incoming),
                             ("place_retreat", "retreat", place.outgoing))
                    legs = timing.call('time_parameterization',lambda:tuple(
                        timed_operation_leg(name, phase, rows, identity,
                            self.velocity_scale, self.acceleration_scale) for name, phase, rows in paths))
                    draft = JointOperationPlan(identity, candidate.candidate_id, "pending-native-certification",
                        candidate.world_from_pick_box, candidate.world_from_pick_tool,
                        candidate.world_from_release_box, candidate.world_from_release_tool,
                        candidate.world_from_settled_box, candidate.predicted_attachment,
                        start, legs, (OperationBarrier(2, "attachment_confirmed"),
                                      OperationBarrier(5, "release_confirmed")), "")
                    # Structural checks before spending native certification work.
                    draft = seal_operation_plan(draft)
                    if stopped(deadline):
                        return finish("CANCELLED" if cancelled() else "SEARCH_INCOMPLETE")
                    receipt = timing.call('whole_chain_certification',self.certification.certify,draft, candidate,
                        deadline=deadline, cancelled=cancelled)
                    if stopped(deadline):
                        return finish("CANCELLED" if cancelled() else "SEARCH_INCOMPLETE")
                    if not self._certificate_matches(receipt, draft, candidate):
                        trace.append((branch_id, "WHOLE_CHAIN_CERTIFICATION_REFUSED"))
                        continue
                    plan = seal_operation_plan(replace(draft, certificate_id=receipt.certificate_id))
                    trace.append((branch_id, "PLAN_READY"))
                    return finish("PLAN_READY", plan)
                if (self.candidate_branch_cap < self.max_branch_attempts
                        and candidate_attempts >= self.candidate_branch_cap):
                    trace.append((candidate.candidate_id, 'CANDIDATE_BRANCH_CAP'))
                    break
        return finish("SEARCH_INCOMPLETE")

    @staticmethod
    def _candidate_matches(candidate, identity):
        if type(candidate) is not OperationCandidate or type(candidate.scenes) is not OperationScenes:
            return False
        scene = candidate.scenes.pick
        return (scene.model_hash == identity.model_sha256
                and hashlib.sha256(repr(candidate.scenes.base_identity).encode()).hexdigest() == identity.snapshot_id
                and scene.scene_revision == identity.scene_revision
                and scene.geometry_hash == identity.geometry_hash
                and scene.map_revision == identity.map_revision
                and all((s.model_hash, s.scene_revision, s.geometry_hash, s.map_revision,
                         s.world_from_container, s.surface_cdr) ==
                        (scene.model_hash, scene.scene_revision, scene.geometry_hash, scene.map_revision,
                         scene.world_from_container, scene.surface_cdr)
                        for s in (candidate.scenes.carry, candidate.scenes.retreat)))

    @staticmethod
    def _evidence_matches(evidence, candidate, identity):
        return (type(evidence) is LocalEndpointEvidence and evidence.complete is True
                and evidence.candidate_id == candidate.candidate_id
                and evidence.model_sha256 == identity.model_sha256
                and evidence.phase_scene_digests == candidate.scenes.digests
                and evidence.attachment_digest == candidate.predicted_attachment.digest()
                and 1 <= len(evidence.pick_branches) <= 2 and 1 <= len(evidence.place_branches) <= 2
                and all(type(branch) is LocalJointBranch for branch in
                        evidence.pick_branches+evidence.place_branches)
                and len({b.branch_id for b in evidence.pick_branches}) == len(evidence.pick_branches)
                and len({b.branch_id for b in evidence.place_branches}) == len(evidence.place_branches))

    @staticmethod
    def _certificate_matches(receipt, draft, candidate):
        return (type(receipt) is OperationCertification and receipt.passed is True
                and receipt.identity == draft.identity and receipt.candidate_id == draft.candidate_id
                and receipt.draft_digest == operation_digest(draft)
                and receipt.phase_scene_digests == candidate.scenes.digests
                and isinstance(receipt.certificate_id, str) and bool(receipt.certificate_id.strip()))
