"""Placement/planning admission gate for the OctoMap reset contract.

Implements plan OCTOMAP_RESET_VALIDITY_NEXT_20261103 §3 (recovery and
downstream admission) as a pure, deterministic state object so the ROS
planner node and the offline reset campaign share one source of truth:

- A validity snapshot (``VALID``/``INVALID``/``BUILDING`` with epoch and a
  monotonically increasing ``generation``) is observed from the observer's
  latched publication or status service.
- Placement stops new computations as soon as invalidation is observed;
  queued/cached candidates and derived results from older generations are
  invalidated. A late old-epoch response is tagged invalid and never
  re-opens the gate.
- Planning admission checks validity and epoch BEFORE planning and again
  BEFORE committing a plan; a mismatch discards the candidate. A delayed old
  response cannot re-open the gate.
- Recovery requires a fresh ``VALID(e)`` map AND a robot joint/EEF sample
  whose stamp is strictly newer than the validity transition. Missing or
  stale inputs keep the gate closed.
- A plan committed in an older generation that has not been retired yet is
  reported as ``execution_active`` — never silently cancelled.

The module never performs IO: every input is an observed record, every
output a decision the caller applies.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass(frozen=True)
class ValidityObservation:
    """One latched validity snapshot as published by the observer."""

    state: str            # VALID | INVALID | BUILDING
    epoch: int
    generation: int
    reason: str = ""
    reset_id: str = ""
    transition_mono_ns: int = 0
    map_revision: int = 0
    observed_revision: int = 0
    geometry_hash: str = ""
    config_digest: str = ""
    clock_domain: str = ""

    @classmethod
    def from_json(cls, payload: Dict[str, Any]) -> "ValidityObservation":
        return cls(
            state=str(payload.get("state", "INVALID")),
            epoch=int(payload.get("epoch", 0)),
            generation=int(payload.get("generation", 0)),
            reason=str(payload.get("reason", "")),
            reset_id=str(payload.get("reset_id", "")),
            transition_mono_ns=int(payload.get("transition_mono_ns", 0)),
            map_revision=int(payload.get("map_revision", 0)),
            observed_revision=int(payload.get("observed_revision", 0)),
            geometry_hash=str(payload.get("geometry_hash", "")),
            config_digest=str(payload.get("config_digest", "")),
            clock_domain=str(payload.get("clock_domain", "")),
        )


@dataclass
class MapObservation:
    """A surface/cargo map offered for admission (identity fields only)."""

    epoch: int
    generation: int = 0            # validity generation it was computed in
    map_revision: int = 0
    geometry_hash: str = ""
    payload: Dict[str, Any] = field(default_factory=dict)


@dataclass
class RobotStateSample:
    """A joint/EEF (or other dynamic planning input) sample."""

    stamp_mono_ns: int
    source: str = "joint_states"   # which contract this sample must satisfy
    content: Dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class AdmissionToken:
    """Immutable per-request binding minted at planning admission (§2/§3).

    The generation captured BEFORE the candidate is computed is the only
    generation that candidate may ever commit against: a reset (or the
    recovery that follows it) voids the token, so an old computation can
    neither commit with its own generation nor with the board's current
    one. Tokens are single-use: one admission buys exactly one commit.
    """

    seq: int
    epoch: int
    generation: int

    def as_dict(self) -> Dict[str, Any]:
        return {"seq": self.seq, "epoch": self.epoch,
                "generation": self.generation}


@dataclass
class Decision:
    """The gate's answer. ``reason`` is always one of the frozen codes."""

    admitted: bool
    reason: str
    validity: ValidityObservation
    execution_active: bool = False
    #: Set on an admitted ``admit_planning`` decision: the token the caller
    #: must carry to the matching ``commit_plan`` (review 2026-11-03: the
    #: commit argument is the captured start generation, never a re-read of
    #: the board's current generation).
    admission_token: Optional[AdmissionToken] = None


class AdmissionGate:
    """Downstream admission boundary. Deterministic function of observations.

    Lock-free in spirit: the ROS node serializes observations through its
    executor; the campaign drives the same methods in-process.
    """

    #: A robot sample must be strictly newer than the validity transition to
    #: prove it reflects the world AFTER recovery, and fresher than this bound
    #: relative to "now" (contract: each sample passes its existing
    #: timestamp/TF contract; the bound is supplied per call site).
    REASON_CLOSED_INVALID = "validity_invalid"
    REASON_CLOSED_BUILDING = "validity_building"
    REASON_NO_MAP = "map_not_admitted"
    REASON_MAP_EPOCH = "map_epoch_mismatch"
    REASON_MAP_GENERATION = "map_generation_mismatch"
    REASON_ROBOT_STALE = "robot_state_stale"
    REASON_ROBOT_MISSING = "robot_state_missing"
    REASON_ROBOT_PRE_TRANSITION = "robot_state_pre_transition"
    REASON_COMMIT_GENERATION = "commit_generation_mismatch"
    REASON_LATE_OLD_RESULT = "late_old_epoch_result"
    REASON_OK = "ok"

    def __init__(self) -> None:
        self._validity = ValidityObservation("INVALID", 0, 0)
        self._map: Optional[MapObservation] = None
        self._robot: Optional[RobotStateSample] = None
        #: Generations with a committed plan not yet retired (reported as
        #: execution_active; the production orchestration boundary owns the
        #: actual executing command).
        self._executing_generations: List[int] = []
        #: Outstanding per-request admission tokens (§2: the candidate
        #: generation from planning admission survives through commit). Any
        #: generation advance voids them.
        self._outstanding: List[AdmissionToken] = []
        self._token_seq = 0
        #: Counters for the judge: late old results must be counted, never
        #: delivered.
        self.rejected_late_results = 0
        self.invalidated_candidates = 0
        self.generation = 0

    # ---- observation inputs -------------------------------------------------
    def observe_validity(self, observation: ValidityObservation) -> None:
        """Consume a latched validity snapshot (idempotent per generation)."""
        if observation.generation < self._validity.generation:
            # A delayed publication can never move the gate backwards.
            return
        previous = self._validity
        self._validity = observation
        if observation.generation != previous.generation:
            # Any generation advance (reset, recovery, re-publish of a newer
            # snapshot) voids every admission minted against the old one: a
            # candidate planned before the advance can never commit.
            if self._outstanding:
                self.invalidated_candidates += len(
                    [t for t in self._outstanding
                     if t.generation != observation.generation])
                self._outstanding = [
                    t for t in self._outstanding
                    if t.generation == observation.generation]
        if observation.state != "VALID":
            # Invalidation observed: placement stops new computations and the
            # cached map/candidates from older generations die here.
            if self._map is not None and self._map.epoch != observation.epoch:
                self.invalidated_candidates += 1
            self._map = None
            if observation.state == "INVALID" and previous.state == "VALID":
                # Robot state from the invalidated generation is no longer a
                # recovery input.
                self._robot = None
        elif previous.state != "VALID" or previous.epoch != observation.epoch:
            # A new VALID epoch: the old map (if any survived) is unusable.
            if self._map is not None and self._map.epoch != observation.epoch:
                self.invalidated_candidates += 1
                self._map = None
            self._robot = None

    def observe_map(self, sample: MapObservation, *, geometry_hash: str = "",
                    config_digest: str = "") -> Decision:
        """Offer a freshly computed surface/cargo map for admission."""
        validity = self._validity
        if validity.state != "VALID":
            self.invalidated_candidates += 1
            return Decision(False, self._closed_reason(validity), validity)
        if sample.epoch != validity.epoch:
            self.rejected_late_results += 1
            self.invalidated_candidates += 1
            return Decision(False, self.REASON_MAP_EPOCH, validity)
        if sample.generation and sample.generation != validity.generation:
            self.rejected_late_results += 1
            return Decision(False, self.REASON_MAP_GENERATION, validity)
        if geometry_hash and validity.geometry_hash and \
                sample.geometry_hash != validity.geometry_hash:
            return Decision(False, self.REASON_MAP_EPOCH, validity)
        self._map = sample
        return Decision(True, self.REASON_OK, validity)

    def observe_robot_state(self, sample: RobotStateSample) -> None:
        """Record the freshest robot joint/EEF sample (per source)."""
        if self._robot is not None and sample.stamp_mono_ns <= self._robot.stamp_mono_ns:
            return
        self._robot = sample

    def retire_execution(self, generation: int) -> None:
        """The production orchestration reported the command finished."""
        if generation in self._executing_generations:
            self._executing_generations.remove(generation)

    # ---- admission checks ---------------------------------------------------
    def _closed_reason(self, validity: ValidityObservation) -> str:
        if validity.state == "INVALID":
            return self.REASON_CLOSED_INVALID
        if validity.state == "BUILDING":
            return self.REASON_CLOSED_BUILDING
        return self.REASON_OK

    def gate_state(self, *, now_mono_ns: Optional[int] = None,
                   robot_max_age_ns: Optional[int] = None) -> Decision:
        """Full recovery gate: fresh VALID map AND post-transition robot state.

        ``robot_max_age_ns`` is the caller's existing timestamp contract for
        the robot sample; None skips only the age half (tests inject None to
        model a source without an age bound).
        """
        validity = self._validity
        if validity.state != "VALID":
            return Decision(False, self._closed_reason(validity), validity,
                            execution_active=bool(self._executing_generations))
        if self._map is None or self._map.epoch != validity.epoch:
            return Decision(False, self.REASON_NO_MAP, validity,
                            execution_active=bool(self._executing_generations))
        if self._robot is None:
            return Decision(False, self.REASON_ROBOT_MISSING, validity,
                            execution_active=bool(self._executing_generations))
        if validity.transition_mono_ns and \
                self._robot.stamp_mono_ns <= validity.transition_mono_ns:
            return Decision(False, self.REASON_ROBOT_PRE_TRANSITION, validity,
                            execution_active=bool(self._executing_generations))
        if robot_max_age_ns is not None and now_mono_ns is not None and \
                now_mono_ns - self._robot.stamp_mono_ns > robot_max_age_ns:
            return Decision(False, self.REASON_ROBOT_STALE, validity,
                            execution_active=bool(self._executing_generations))
        return Decision(True, self.REASON_OK, validity,
                        execution_active=bool(self._executing_generations))

    def admit_planning(self, **kwargs: Any) -> Decision:
        """Check BEFORE planning (§3). Same rule as the recovery gate.

        On success the decision carries an immutable admission token: the
        caller must present exactly that token at commit time. The token is
        the generation captured BEFORE computing — re-reading the board at
        commit and passing the current generation is a contract violation
        the gate itself now rejects (review 2026-11-03, finding 1).
        """
        decision = self.gate_state(**kwargs)
        if not decision.admitted:
            return decision
        self._token_seq += 1
        token = AdmissionToken(self._token_seq, self._validity.epoch,
                               self._validity.generation)
        self._outstanding.append(token)
        decision.admission_token = token
        return decision

    def commit_plan(self, planned_generation: Optional[int] = None,
                    admission_token: Optional[AdmissionToken] = None,
                    **kwargs: Any) -> Decision:
        """Check BEFORE committing a plan: the second epoch/state check.

        The commit must present the generation (and token) captured at
        planning admission. Three things must hold: the recovery gate is
        open, an outstanding admission exists for this candidate, and the
        captured generation still equals the board's current generation. A
        reset during computation voids the admission, so an old candidate
        fails whether it presents its own generation or the recovered
        board's current one.
        """
        validity = self._validity
        decision = self.gate_state(**kwargs)
        if not decision.admitted:
            return decision
        if planned_generation is None and admission_token is not None:
            planned_generation = admission_token.generation
        if planned_generation is None:
            planned_generation = validity.generation
        matches = [t for t in self._outstanding
                   if t.generation == planned_generation]
        if admission_token is not None:
            matches = [t for t in matches if t == admission_token]
        if planned_generation != validity.generation or not matches:
            self.invalidated_candidates += 1
            return Decision(False, self.REASON_COMMIT_GENERATION, validity,
                            execution_active=bool(self._executing_generations))
        self._outstanding.remove(matches[0])
        return decision

    def record_execution(self) -> int:
        """A plan was committed in the current generation; report its id."""
        self._executing_generations.append(self._validity.generation)
        return self._validity.generation

    def accept_result(self, result_epoch: int, result_generation: int = 0) -> Decision:
        """Terminal check for a late-arriving placement/Surface result.

        A result computed against an older generation is tagged invalid and
        counted; it never becomes a valid placement result.
        """
        validity = self._validity
        if result_epoch != validity.epoch or (
                result_generation and result_generation != validity.generation):
            self.rejected_late_results += 1
            return Decision(False, self.REASON_LATE_OLD_RESULT, validity)
        return Decision(True, self.REASON_OK, validity)

    # ---- introspection ------------------------------------------------------
    @property
    def validity(self) -> ValidityObservation:
        return self._validity

    @property
    def admitted_map(self) -> Optional[MapObservation]:
        return self._map

    @property
    def robot_state(self) -> Optional[RobotStateSample]:
        return self._robot

    @property
    def executing_generations(self) -> List[int]:
        return list(self._executing_generations)

    @property
    def outstanding_admissions(self) -> List[AdmissionToken]:
        """Admissions minted but not yet committed (introspection/dumps)."""
        return list(self._outstanding)
