"""Bounded target/grasp screening on an immutable snapshot.

The collision port owns PRIVATE geometry; it exposes only check(), never apply,
attach, execute or map commit. Passing rigid-tool screening is not arm IK or a
route certificate. Consumers must reconnect from the actual post-pick state and
retain the ordinary complete-route certifier.
"""
from dataclasses import dataclass
import time
from typing import Protocol

from luggage_planning.placement_prior_contracts import (
    GraspCandidate, PlacementCandidate, PriorRequest, Snapshot, Transform,
)


@dataclass(frozen=True)
class CollisionVerdict:
    valid: bool
    reason: str
    contacts: tuple = ()
    local_paths_json: str = ""

    def __post_init__(self):
        object.__setattr__(self, 'contacts', tuple(tuple(p) for p in self.contacts))
        if type(self.valid) is not bool or not self.reason:
            raise ValueError('explicit collision verdict required')


class ToolCollisionPort(Protocol):
    def check(self, snapshot: Snapshot, world_from_tcp: Transform,
              world_from_box: Transform, box_wdh: tuple, *, deadline: float,
              cancelled) -> CollisionVerdict:
        """Check full rigid tool/payload in private scene, within deadline.

        The implementation must preserve robot collision geometry, padding,
        uncertainty policy and allowed-contact semantics. It must fail closed
        on missing geometry, unknown required occupancy, cancellation or timeout.
        """


@dataclass(frozen=True)
class CandidateTrace:
    slot_id: str
    candidate_id: str
    stage_index: int
    verdict: CollisionVerdict


@dataclass(frozen=True)
class PriorChoice:
    placement: PlacementCandidate
    grasp: GraspCandidate
    world_from_place_tcp: Transform
    # Ordered TCP references only, NOT an executable joint trajectory.
    tcp_references: tuple

    def pickup_tcp(self, world_from_pick_box):
        return world_from_pick_box.compose(self.grasp.box_from_tcp)


@dataclass(frozen=True)
class PriorResult:
    request_id: str
    reuse_key: tuple
    status: str
    choices: tuple
    trace: tuple
    elapsed_s: float
    pairs_checked: int

    def adopt(self, current: PriorRequest):
        """Return hints only. Caller still validates actual attachment/start/route."""
        if not current.measured or self.reuse_key != current.reuse_key:
            return ()
        if self.status not in ('READY', 'CANDIDATE_LIMIT'):
            return ()
        return self.choices


def compute_prior(request: PriorRequest, collision: ToolCollisionPort, *,
                  cancelled=lambda: False, clock=time.monotonic):
    started = clock()
    deadline = started + request.budget_s
    choices, trace = [], []
    pairs = 0

    def finish(status):
        # Never expose partially completed work after timeout/cancellation.
        usable = tuple(choices) if status in ('READY', 'CANDIDATE_LIMIT') else ()
        return PriorResult(request.request_id, request.reuse_key, status, usable,
                           tuple(trace), clock()-started, pairs)

    for placement in request.placements:
        for grasp in sorted(request.grasps, key=lambda g: (g.rank, g.candidate_id)):
            if cancelled():
                return finish('CANCELLED')
            if clock() >= deadline:
                return finish('DEADLINE')
            if pairs >= request.candidate_limit:
                return finish('CANDIDATE_LIMIT')
            pairs += 1
            refs = []
            box_poses = placement.critical_box_poses + (placement.container_from_box,)
            for index, box_pose in enumerate(box_poses):
                if cancelled():
                    return finish('CANCELLED')
                if clock() >= deadline:
                    return finish('DEADLINE')
                world_box = request.snapshot.world_from_container.compose(box_pose)
                world_tcp = world_box.compose(grasp.box_from_tcp)
                verdict = collision.check(request.snapshot, world_tcp, world_box,
                                          request.box_wdh, deadline=deadline,
                                          cancelled=cancelled)
                if not isinstance(verdict, CollisionVerdict):
                    raise TypeError('collision port returned no typed verdict')
                trace.append(CandidateTrace(placement.slot_id, grasp.candidate_id,
                                            index, verdict))
                if cancelled():
                    return finish('CANCELLED')
                if clock() >= deadline:
                    return finish('DEADLINE')
                if not verdict.valid:
                    break
                refs.append(world_tcp)
            else:
                if request.world_from_pick_box is not None:
                    local = collision.check_local(request, placement, grasp, deadline=deadline, cancelled=cancelled)
                    trace.append(CandidateTrace(placement.slot_id, grasp.candidate_id, -1, local))
                    if not local.valid:
                        continue
                choices.append(PriorChoice(placement, grasp, refs[-1], tuple(refs)))
        # Evaluate every measured G for this slot, then return its fully
        # witnessed alternatives before spending budget on another placement.
        if request.world_from_pick_box is not None and choices:
            return finish("READY")
    return finish('READY' if choices else 'NO_VALID_GRASP')
