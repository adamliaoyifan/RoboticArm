#!/usr/bin/env python3
"""Pure place-target synthesis and IK goal sets for the ST09-B route planner.

The 5.1 target math (``T_world_tcp = T_world_box * inverse(T_tcp_box)``,
gate order, theta/theta+pi expansion for declared symmetry) lives in
``luggage_planning.planning_contract.attachment.resolve_place_targets`` and is never
reimplemented here. This module adds the planning-side pieces the route
planner needs around it:

- ``box_target_from_candidate``: packing's container-frame
  ``PlacementCandidate`` pose -> a ``BoxTarget`` in the planning frame.
- ``expand_place_targets``: the contract call wrapped so every refusal
  arrives as a :class:`TargetSynthesisError` carrying the frozen
  :class:`~luggage_planning.planning_contract.reasons.Reason` (zero business effect).
- ``goal_joint_sets``: multi-goal IK expansion without tool-yaw pruning —
  every injected seed family is refined onto the exact TCP target, q/-q and
  yaw+pi representations stay equivalent, and goals that share the TCP
  position but differ in q remain distinct goals.

No TF, MoveIt, atlas or GT access happens in this module; seed selection is
injected through ``seeds_for_target`` so the module stays pure and testable.
"""

from __future__ import division

import math
from dataclasses import dataclass
from typing import Callable, Optional, Sequence, Tuple

from luggage_planning.planning_contract._base import ContractViolation
from luggage_planning.planning_contract.attachment import (
    ATTACHMENT_MAX_AGE_NS,
    AttachmentRecord,
    BoxTarget,
    PlaceTarget,
    compose_transform,
    resolve_place_targets,
)
from luggage_planning.planning_contract.planning import PlacementCandidate
from luggage_planning.planning_contract.reasons import Reason

#: Canonical arm joint count (elfin_joint1..6, order preserved downstream).
ARM_JOINT_COUNT = 6

#: Goal dedupe tolerance over raw joint values (same branch, same values).
GOAL_DEDUPE_TOL = 1e-9

#: Default IK refine tolerances, mirroring offline_ik.refine_to_pose.
DEFAULT_POS_TOL = 5e-4
DEFAULT_ROT_TOL = 5e-3


class TargetSynthesisError(Exception):
    """Named refusal from place-target synthesis (frozen Reason taxonomy).

    The original :class:`ContractViolation` is preserved as ``__cause__``;
    ``detail`` keeps its message verbatim. Raising this is the only effect:
    no partial target is returned and no state is mutated.
    """

    def __init__(self, reason: Reason, detail: str):
        self.reason = reason
        self.detail = str(detail)
        super(TargetSynthesisError, self).__init__(
            "%s: %s" % (reason.value, self.detail))

    @property
    def code(self) -> str:
        return self.reason.value


def _named_reason(message: str) -> Reason:
    """Extract the leading ``REASON:`` code from a contract violation."""
    head = str(message).split(":", 1)[0].strip()
    try:
        return Reason(head)
    except ValueError:
        return Reason.SCHEMA_MISMATCH


def box_target_from_candidate(
    candidate: PlacementCandidate,
    attachment: AttachmentRecord,
    *,
    stamp_ns: int,
    t_frame_container: Sequence[float] = (0.0, 0.0, 0.0),
    q_frame_container: Sequence[float] = (0.0, 0.0, 0.0, 1.0),
    frame_id: str = "world",
    request_id: Optional[str] = None,
    scene_revision: Optional[str] = None,
) -> BoxTarget:
    """Compose the candidate's container-frame box pose into ``frame_id``.

    ``t_frame_container``/``q_frame_container`` is the planning-frame <-
    container transform (identity keeps container coordinates). The heading
    passes through unmodified — theta/theta+pi expansion belongs to the
    contract call, not to this construction. ``request_id``/``scene_revision``
    default to the attachment's values because a carried route is planned
    for that attachment's request and scene.
    """
    position, quaternion = compose_transform(
        tuple(float(v) for v in t_frame_container),
        tuple(float(v) for v in q_frame_container),
        tuple(float(v) for v in candidate.position_container),
        tuple(float(v) for v in candidate.quaternion_xyzw_container),
    )
    return BoxTarget(
        candidate_id=candidate.candidate_id,
        request_id=request_id if request_id is not None else attachment.request_id,
        frame_id=str(frame_id),
        scene_revision=(
            scene_revision if scene_revision is not None else attachment.scene_revision),
        heading_theta=float(candidate.heading_theta),
        position=position,
        quaternion_xyzw=quaternion,
        stamp_ns=int(stamp_ns),
    )


def expand_place_targets(
    box_target: BoxTarget,
    attachment: AttachmentRecord,
    *,
    require_attach_confirmed: bool = True,
    max_age_ns: int = ATTACHMENT_MAX_AGE_NS,
) -> Tuple[PlaceTarget, ...]:
    """Delegate to the frozen 5.1 contract call with named refusals.

    Returns one :class:`PlaceTarget` per heading allowed by the declared
    symmetry (``none`` -> 1, ``pi`` -> 2). The theta+pi heading is a
    genuinely different physical rotation: through an eccentric attachment
    it yields a different TCP pose, which callers must keep as a separate
    goal rather than folding onto the nearest tool yaw.
    """
    try:
        return resolve_place_targets(
            box_target, attachment,
            require_attach_confirmed=require_attach_confirmed,
            max_age_ns=max_age_ns)
    except ContractViolation as err:
        raise TargetSynthesisError(_named_reason(err), str(err)) from err


@dataclass(frozen=True)
class GoalSeed:
    """One IK seed for a TCP target: q plus search/diagnostic labels."""

    q: Tuple[float, ...]
    ik_family: str = ""
    provenance: str = ""

    def __post_init__(self) -> None:
        values = tuple(float(v) for v in self.q)
        if len(values) != ARM_JOINT_COUNT or not all(
                math.isfinite(v) for v in values):
            raise ValueError("GoalSeed.q: %d finite values required" % ARM_JOINT_COUNT)
        object.__setattr__(self, "q", values)
        object.__setattr__(self, "ik_family", str(self.ik_family or ""))
        object.__setattr__(self, "provenance", str(self.provenance or ""))


@dataclass(frozen=True)
class JointGoal:
    """One refined joint solution for one TCP target (exact-q identity)."""

    q: Tuple[float, ...]
    ik_family: str
    provenance: str
    position_error_m: float
    rotation_error_rad: float


@dataclass(frozen=True)
class GoalSet:
    """All refined joint solutions found for one :class:`PlaceTarget`."""

    target: PlaceTarget
    goals: Tuple[JointGoal, ...]
    note: str = ""

    @property
    def heading_theta(self) -> float:
        return self.target.heading_theta


#: refine(q_target...) callable: (target_pos, target_quat, seed_q) ->
#: (q | None, info dict with pos_err_m / rot_err_rad).
RefineCallable = Callable[[Sequence[float], Sequence[float], Sequence[float]], tuple]


def goal_joint_sets(
    targets: Sequence[PlaceTarget],
    *,
    seeds_for_target: Callable[[PlaceTarget], Sequence[GoalSeed]],
    refine: Optional[RefineCallable] = None,
    urdf=None,
    tcp_path=None,
    pos_tol: float = DEFAULT_POS_TOL,
    rot_tol: float = DEFAULT_ROT_TOL,
    dedupe_tol: float = GOAL_DEDUPE_TOL,
) -> Tuple[GoalSet, ...]:
    """Refine every injected seed onto its exact TCP target, per target.

    Semantics the route planner relies on:

    - no tool-yaw pruning: every seed is attempted, in the given order;
    - goals with the same TCP position but different q stay distinct
      (dedupe is exact-q at ``dedupe_tol`` only);
    - q/-q target quaternions produce identical solutions because the
      refine residual is rotation-valued, not quaternion-valued;
    - a seed that fails to converge contributes nothing — an empty goal
      set is a prior gap (MoveIt supplementation territory), not an error.

    ``refine`` overrides the default ``offline_ik.refine_to_pose`` call
    (which then needs ``urdf`` and ``tcp_path``).
    """
    if refine is None:
        if urdf is None or tcp_path is None:
            raise ValueError("goal_joint_sets: urdf and tcp_path required without refine")
        from luggage_planning.motion.offline_ik import refine_to_pose

        def refine(target_pos, target_quat, seed_q, _refine=refine_to_pose):
            return _refine(
                urdf, tcp_path, target_pos, target_quat, list(seed_q),
                pos_tol=pos_tol, rot_tol=rot_tol)

    sets = []
    for target in targets:
        goals = []
        seen_q = []
        for seed in seeds_for_target(target):
            joints, info = refine(target.position, target.quaternion_xyzw, seed.q)
            if joints is None:
                continue
            q = tuple(float(v) for v in joints[:ARM_JOINT_COUNT])
            if len(q) != ARM_JOINT_COUNT or not all(math.isfinite(v) for v in q):
                continue
            if any(max(abs(a - b) for a, b in zip(q, prior)) <= dedupe_tol
                   for prior in seen_q):
                continue
            seen_q.append(q)
            goals.append(JointGoal(
                q=q,
                ik_family=seed.ik_family,
                provenance=seed.provenance,
                position_error_m=float(info.get("pos_err_m", float("nan"))),
                rotation_error_rad=float(info.get("rot_err_rad", float("nan"))),
            ))
        sets.append(GoalSet(
            target=target,
            goals=tuple(goals),
            note="seeds=%d goals=%d" % (
                len(seeds_for_target(target)), len(goals)),
        ))
    return tuple(sets)
