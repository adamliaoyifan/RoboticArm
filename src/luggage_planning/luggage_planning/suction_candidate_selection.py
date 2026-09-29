#!/usr/bin/env python3
"""Bounded suction-candidate selection (ROS-free).

Consumes the ST-2 candidate contract: planning evaluates accepted
candidates in perception rank order, skipping any candidate that fails
TF, IK, collision or Cartesian-fraction checks (plan section D). At
most three candidates are attempted per operator request and no
candidate ID repeats. The identity gate reuses the perception
evaluator's ``suction_identity_mismatch`` as the single authority for
``SUCTION_CANDIDATE_IDENTITY_MISMATCH``.
"""

from dataclasses import dataclass, field
from typing import Optional, Tuple

from luggage_perception.suction_patch_evaluator import (
    SUCTION_CANDIDATE_IDENTITY_MISMATCH,
    suction_identity_mismatch,
)
from luggage_description.frame_contracts import ContractError, validate_pose
from luggage_planning.pose import Pose

SUCTION_REJECT_TF = "SUCTION_REJECT_TF"
SUCTION_REJECT_IK = "SUCTION_REJECT_IK"
SUCTION_REJECT_COLLISION = "SUCTION_REJECT_COLLISION"
SUCTION_REJECT_CARTESIAN_FRACTION = "SUCTION_REJECT_CARTESIAN_FRACTION"
SUCTION_CANDIDATES_EXHAUSTED = "SUCTION_CANDIDATES_EXHAUSTED"
#: candidate provenance differs from the loaded contact model config
SUCTION_CONTACT_MODEL_MISMATCH = "SUCTION_CONTACT_MODEL_MISMATCH"

#: at most three candidates per operator request (plan section D)
MAX_CANDIDATE_ATTEMPTS = 3
#: approach, attach and retry retreat require a full Cartesian solution
REQUIRED_CARTESIAN_FRACTION = 1.0

#: ROS-free MoveItErrorCodes collision values: START_STATE_IN_COLLISION
#: and GOAL_IN_COLLISION. IK with these codes is a collision rejection,
#: not an unreachable one. NO_IK_SOLUTION (-31) stays an IK rejection.
COLLISION_ERROR_CODES = frozenset({
    -10,   # START_STATE_IN_COLLISION
    -22,   # GOAL_IN_COLLISION
})
NO_IK_SOLUTION = -31

#: segments whose Cartesian fraction must reach the required value
FRACTION_GATED_SEGMENTS = ("approach", "attach", "retry_reverse")


@dataclass(frozen=True)
class SuctionCandidateView:
    """Plain planning-side view of one ``SuctionCandidate`` message.

    Attribute names ``stamp``/``frame``/``instance_id``/``generation``
    are deliberate: the perception identity gate duck-types them.
    """
    candidate_id: str
    rank: int
    stamp: float
    frame: str
    instance_id: str
    generation: int
    contact: Pose
    model_version: int
    model_hash: str
    score: float = 0.0
    valid_coverage: float = 0.0
    mask_coverage: float = 0.0
    plane_coverage: float = 0.0
    rms_residual: float = 0.0
    p95_residual: float = 0.0
    peak_to_valley: float = 0.0
    normal_deviation_p95: float = 0.0
    max_adjacent_step: float = 0.0
    boundary_clearance: float = 0.0


@dataclass(frozen=True)
class ProbeRecord:
    """One plan-only reachability probe of one candidate segment."""
    candidate_id: str
    segment_name: str
    ik_ok: bool
    fraction: float = -1.0
    moveit_error_code: int = 0


def rank_candidates(candidates):
    """Candidates in perception rank order (rank asc, id tie-break)."""
    return tuple(sorted(
        candidates,
        key=lambda c: (int(c.rank), str(c.candidate_id))))


def observation_identity_consistent(candidates, stamp, frame, instance_id,
                                    generation, tolerance_sec=0.0):
    """None when every candidate echoes the observation identity.

    Otherwise the first ``(SUCTION_CANDIDATE_IDENTITY_MISMATCH, detail)``
    from the perception authority. Called before any waypoint is built.
    The reference identity (stamp/frame from the observation header,
    instance/generation from the producing YOLO instance) must come from
    the observation, never from the candidate under test.
    """
    for candidate in candidates:
        mismatch = suction_identity_mismatch(
            candidate, stamp, frame, instance_id, generation,
            tolerance_sec=tolerance_sec)
        if mismatch is not None:
            return mismatch
    return None


def cross_candidate_identity_consistent(candidates):
    """None when all candidates share one instance/generation identity."""
    if not candidates:
        return None
    first = candidates[0]
    for candidate in candidates[1:]:
        if (str(candidate.instance_id) != str(first.instance_id)
                or int(candidate.generation) != int(first.generation)):
            return (
                SUCTION_CANDIDATE_IDENTITY_MISMATCH,
                "instance/generation differ across candidates")
    return None


def contact_model_matches(candidate, model_version, model_hash):
    return (int(candidate.model_version) == int(model_version)
            and str(candidate.model_hash) == str(model_hash))


def judge_probe(candidate, records, required_fraction=REQUIRED_CARTESIAN_FRACTION,
                planning_frame="world", contact_record=None,
                scene_revision=""):
    """None when the candidate passes every probe, else a reason string.

    TF: the candidate frame must equal the planning frame. IK: a failed
    IK probe is an IK rejection unless the error code marks collision.
    Fraction: ``approach``/``attach``/``retry_reverse`` must reach
    ``required_fraction`` exactly (0.999 fails a 1.0 gate).

    v2 admission: when a v2 contact record is supplied (the session's
    ``contact_record`` port provides one for the candidate under test),
    :func:`contact_record_admission` runs FIRST — a boundary violation
    (stale scene, wrong frame, bad payload) rejects before any probe
    effect, carrying the exact contract code in the reason detail. The v1
    string-compare path is unchanged when no record is supplied.
    """
    if contact_record is not None:
        admission = contact_record_admission(
            contact_record, scene_revision, planning_frame=planning_frame)
        if admission is not None:
            return admission
    if str(candidate.frame) != str(planning_frame):
        return SUCTION_REJECT_TF
    for record in records:
        if record.segment_name not in FRACTION_GATED_SEGMENTS:
            continue
        if not record.ik_ok:
            if record.moveit_error_code in COLLISION_ERROR_CODES:
                return SUCTION_REJECT_COLLISION
            return SUCTION_REJECT_IK
        if float(record.fraction) < float(required_fraction):
            return SUCTION_REJECT_CARTESIAN_FRACTION
    return None


def contact_record_admission(record, scene_revision, planning_frame="world"):
    """None when a v2 contact record is admissible for planning (F04).

    Otherwise ``(SUCTION_REJECT_TF, "<ContractError.code>:<field>")``: the
    documented reason category is preserved while the detail carries the
    exact v2 code (FRAME_CONTRACT_MISMATCH, TIME_BASIS_MISMATCH,
    CALIBRATION_REVISION_MISMATCH, ...). Pure check — building it into the
    selection loop must precede any probe/waypoint/vacuum effect. The v1
    string-compare path in ``judge_probe`` is unchanged for v1 records.
    """
    try:
        validate_pose(record, "contact", scene_revision)
    except ContractError as error:
        return (SUCTION_REJECT_TF, "%s:%s" % (error.code, error.field))
    if str(record.get("frame_id")) != str(planning_frame):
        return (SUCTION_REJECT_TF,
                "FRAME_CONTRACT_MISMATCH:%s" % record.get("frame_id"))
    return None


@dataclass(frozen=True)
class SelectionState:
    """Bounded attempt ledger for one operator request.

    ``attempted_ids`` counts physical attempts only (motion or vacuum
    reached); probe-stage rejections live in ``rejections`` and do NOT
    consume the attempt cap — gate C1 selects the fourth of five ranked
    candidates after three probe failures.
    """
    ranked_ids: Tuple[str, ...] = ()
    attempted_ids: Tuple[str, ...] = ()
    rejections: Tuple[Tuple[str, str, str], ...] = ()
    max_attempts: int = MAX_CANDIDATE_ATTEMPTS

    @property
    def consumed_ids(self):
        return frozenset(self.attempted_ids) | frozenset(
            row[0] for row in self.rejections)


def next_candidate(state):
    """Next ranked candidate id that is neither attempted nor rejected.

    ``SUCTION_CANDIDATES_EXHAUSTED`` when none remains or the physical
    attempt cap is already reached.
    """
    if len(state.attempted_ids) >= state.max_attempts:
        return SUCTION_CANDIDATES_EXHAUSTED
    consumed = state.consumed_ids
    for candidate_id in state.ranked_ids:
        if candidate_id not in consumed:
            return candidate_id
    return SUCTION_CANDIDATES_EXHAUSTED


def record_rejection(state, candidate_id, reason, detail=""):
    return SelectionState(
        ranked_ids=state.ranked_ids,
        attempted_ids=state.attempted_ids,
        rejections=state.rejections + ((candidate_id, reason, detail),),
        max_attempts=state.max_attempts,
    )


def record_attempt(state, candidate_id):
    """Mark a candidate physically attempted (motion or vacuum reached)."""
    if candidate_id in state.consumed_ids:
        raise ValueError("candidate %s already consumed" % candidate_id)
    if len(state.attempted_ids) >= state.max_attempts:
        raise ValueError("attempt cap %d reached" % state.max_attempts)
    return SelectionState(
        ranked_ids=state.ranked_ids,
        attempted_ids=state.attempted_ids + (candidate_id,),
        rejections=state.rejections,
        max_attempts=state.max_attempts,
    )


def exhausted(state):
    """True when no candidate remains or the physical attempt cap is reached."""
    return next_candidate(state) == SUCTION_CANDIDATES_EXHAUSTED
