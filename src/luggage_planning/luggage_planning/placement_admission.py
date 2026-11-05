"""Placement/planning admission gate for the reset-validity contract.

Implements plan OCTOMAP_RESET_VALIDITY_NEXT_20261103 §3 (recovery and
downstream admission) as a pure, deterministic state object so the ROS
planner node, the offline reset campaign, and the common-consumer
deployment (OCC-GRID-CONSUMER-COMPAT G1) share one source of truth:

- A validity snapshot (``VALID``/``INVALID``/``BUILDING`` with epoch and a
  monotonically increasing ``generation``) is observed from the producer's
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

G1 common-consumer extensions (contract OCC_GRID_CONSUMER_COMPAT_G1_20261105):

- **Lossless canonical epochs.** Epochs are canonical STRINGS end to end.
  The observer (O) publishes uint64 epochs — canonical decimal strings;
  the grid backend (G) publishes UUID strings — verbatim (lowercased,
  dashed form normalized at the fixed UUID positions, reversible), never
  numerically reinterpreted (an all-digit UUID with leading zeros must
  survive). Normalization is backend-aware; a malformed epoch for its
  declared backend raises instead of defaulting.
- **Backend binding.** Validity and map observations carry a backend tag;
  the gate keys identity by ``(backend, epoch)`` so a G epoch can never be
  spliced into an O stream and vice versa, even when the strings collide.
- **Per-key generation high-water with controlled producer confirmation.**
  A publication that would move the gate "backwards" (lower generation,
  retired epoch, or an epoch never seen while another producer is current)
  is dropped AND recorded as ``pending_confirmation``: it may be a delayed
  old stream OR a genuinely restarted producer, which the stream alone
  cannot distinguish. Switching to such an identity happens ONLY through
  :meth:`confirm_producer`, fed by the node adapter from the producer's
  real status service (get_validity). Nothing here fabricates freshness.
- **Unconditional map->generation binding.** A map whose validity
  generation is missing or zero is rejected (both production backends
  publish it); the historical "when declared" bypass is gone.

The module never performs IO: every input is an observed record, every
output a decision the caller applies.
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

#: Backend tags (I2: identity is never interchangeable across backends).
BACKEND_OBSERVER_O = "observer_o"
BACKEND_GRID_G = "grid_g"

_UUID_HEX_RE = re.compile(r"^[0-9a-f]{32}$")
_UUID_DASHED_RE = re.compile(
    r"^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$")
_UINT64_RE = re.compile(r"^[0-9]+$")


class EpochFormatError(ValueError):
    """An epoch that is not a lossless identity in its declared backend."""


def normalize_epoch(value: Any, backend: str = "") -> str:
    """Canonical string form of one epoch, lossless per backend.

    - int (legacy/observer wire): canonical decimal string.
    - ``observer_o``: uint64 only — digits canonicalized via ``str(int)``
      (leading zeros are not part of a uint64's identity).
    - ``grid_g``: full UUID only — 32 hex or 8-4-4-4-12, lowercased,
      dashes normalized at the fixed positions (reversible); NEVER parsed
      as a number (an all-digit UUID with leading zeros survives verbatim).
    - backend undeclared (tests/legacy constructions): ints canonicalize,
      non-empty strings pass verbatim.
    """
    if isinstance(value, bool):
        raise EpochFormatError("epoch must not be a bool")
    if isinstance(value, int):
        if value < 0:
            raise EpochFormatError("negative epoch: %r" % (value,))
        return str(value)
    if not isinstance(value, str):
        raise EpochFormatError("epoch must be int or str, got %r"
                               % (type(value).__name__,))
    text = value.strip()
    if not text:
        # Missing (not malformed): callers decide — the gate rejects a
        # missing epoch with a dedicated reason instead of fabricating 0.
        return ""
    if backend == BACKEND_GRID_G:
        lowered = text.lower()
        if _UUID_HEX_RE.match(lowered):
            return lowered
        if _UUID_DASHED_RE.match(lowered):
            return lowered.replace("-", "")
        raise EpochFormatError("grid epoch is not a full UUID: %r" % (text,))
    if backend == BACKEND_OBSERVER_O:
        if _UINT64_RE.match(text):
            return str(int(text))
        raise EpochFormatError(
            "observer epoch is not a uint64: %r" % (text,))
    if not text:
        raise EpochFormatError("empty epoch")
    return text


@dataclass(frozen=True)
class ValidityObservation:
    """One latched validity snapshot as published by the producer.

    ``epoch`` is the canonical string (see :func:`normalize_epoch`);
    ``backend`` binds the identity to one producer family. The payload
    field keeps the producer's original transition stamp (the node
    re-stamps ``transition_mono_ns`` in its own clock for the robot
    comparison; the original is judge evidence).
    """

    state: str            # VALID | INVALID | BUILDING
    epoch: str
    generation: int
    reason: str = ""
    reset_id: str = ""
    transition_mono_ns: int = 0
    map_revision: int = 0
    observed_revision: int = 0
    geometry_hash: str = ""
    config_digest: str = ""
    clock_domain: str = ""
    backend: str = ""
    producer_transition_mono_ns: int = 0

    def __post_init__(self) -> None:
        object.__setattr__(self, "state", str(self.state or "INVALID"))
        object.__setattr__(self, "epoch",
                           normalize_epoch(self.epoch, self.backend))
        object.__setattr__(self, "generation", int(self.generation or 0))

    @property
    def identity(self) -> Tuple[str, str]:
        return (self.backend, self.epoch)

    @classmethod
    def from_json(cls, payload: Dict[str, Any],
                  backend: str = "") -> "ValidityObservation":
        raw_epoch = payload.get("epoch")
        if raw_epoch is None or raw_epoch == "":
            # A missing epoch is not a compatible default (I2): required
            # identity fields never fall back to 0.
            raise EpochFormatError("validity payload carries no epoch")
        epoch = normalize_epoch(raw_epoch, backend)
        producer_transition = int(
            payload.get("transition_mono_ns", 0) or 0)
        return cls(
            state=str(payload.get("state", "INVALID")),
            epoch=epoch,
            generation=int(payload.get("generation", 0)),
            reason=str(payload.get("reason", "")),
            reset_id=str(payload.get("reset_id", "")),
            transition_mono_ns=producer_transition,
            map_revision=int(payload.get("map_revision", 0)),
            observed_revision=int(payload.get("observed_revision", 0)),
            geometry_hash=str(payload.get("geometry_hash", "")),
            config_digest=str(payload.get("config_digest", "")),
            clock_domain=str(payload.get("clock_domain", "")),
            backend=str(backend or payload.get("backend", "")),
            producer_transition_mono_ns=producer_transition,
        )


@dataclass
class MapObservation:
    """A surface/cargo map offered for admission (identity fields only)."""

    epoch: str
    generation: int = 0            # validity generation it was computed in
    map_revision: int = 0
    geometry_hash: str = ""
    backend: str = ""
    payload: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.epoch = normalize_epoch(self.epoch, self.backend)
        self.generation = int(self.generation or 0)

    @property
    def identity(self) -> Tuple[str, str]:
        return (self.backend, self.epoch)


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
    epoch: str
    generation: int
    backend: str = ""

    def as_dict(self) -> Dict[str, Any]:
        return {"seq": self.seq, "epoch": self.epoch,
                "generation": self.generation, "backend": self.backend}


@dataclass
class Decision:
    """The gate's answer. ``reason`` is always one of the frozen codes."""

    admitted: bool
    reason: str
    validity: ValidityObservation
    execution_active: bool = False
    #: Set on an admitted ``admit_planning`` decision: the token the caller
    #: must carry to the matching ``commit_plan`` (review 2026-11-03: the
    #; commit argument is the captured start generation, never a re-read of
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
    REASON_MAP_EPOCH_MISSING = "map_epoch_missing"
    REASON_MAP_GENERATION_MISSING = "map_generation_missing"
    REASON_MAP_GENERATION = "map_generation_mismatch"
    REASON_MAP_BACKEND = "map_backend_mismatch"
    REASON_ROBOT_STALE = "robot_state_stale"
    REASON_ROBOT_MISSING = "robot_state_missing"
    REASON_ROBOT_PRE_TRANSITION = "robot_state_pre_transition"
    REASON_COMMIT_GENERATION = "commit_generation_mismatch"
    REASON_LATE_OLD_RESULT = "late_old_epoch_result"
    REASON_UNSEEN_EPOCH = "unseen_epoch_pending_confirmation"
    REASON_CONFIRM_CONFLICT = "same_generation_content_conflict"
    REASON_OK = "ok"

    def __init__(self) -> None:
        # Boot sentinel: epoch "0" under the undeclared backend is never a
        # real identity (production observations always carry a declared
        # backend; the sentinel exists only so pre-first-observation
        # decisions have a validity object to describe).
        self._validity = ValidityObservation("INVALID", "0", 0)
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
        #: Per-(backend, epoch) generation high-water. The stream alone
        #: cannot distinguish a delayed old publication from a restarted
        #: producer, so anything that would move the gate backwards is
        #: parked in ``pending_confirmation`` until the node adapter
        #: confirms the producer's CURRENT identity via its real status
        #: service (I2 controlled confirmation — never a guess).
        self._generation_high_water: Dict[Tuple[str, str], int] = {}
        self._retired_keys: set = set()
        self.pending_confirmation: Dict[Tuple[str, str], ValidityObservation] = {}
        self._ever_observed = False
        #: Counters for the judge: late old results must be counted, never
        #: delivered.
        self.rejected_late_results = 0
        self.invalidated_candidates = 0
        self.rejected_backward_publications = 0
        self.rejected_content_conflicts = 0
        self.generation = 0

    # ---- observation inputs -------------------------------------------------
    def _accept_validity(self, observation: ValidityObservation) -> None:
        """Switch to an observation already judged acceptable."""
        key = observation.identity
        previous = self._validity
        self._validity = observation
        self._generation_high_water[key] = max(
            observation.generation,
            self._generation_high_water.get(key, 0))
        self.pending_confirmation.pop(key, None)
        if previous.identity != key and self._ever_observed:
            self._retired_keys.add(previous.identity)
        self._ever_observed = True
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

    def observe_validity(self, observation: ValidityObservation) -> None:
        """Consume a latched validity snapshot.

        Bootstrap (first observation ever) is accepted directly — the gate
        is closed until a VALID map plus a post-transition robot sample
        arrive, so nothing admitted can be corrupted. Afterwards the
        stream is monotonic per (backend, epoch): a lower generation, a
        retired epoch, or a never-seen epoch while another producer is
        current is parked as ``pending_confirmation`` and never switches
        the gate by itself (a delayed old stream and a restarted producer
        are indistinguishable from the stream alone).
        """
        key = observation.identity
        if not observation.epoch:
            # A missing epoch is not an identity: counted and dropped.
            self.rejected_invalid_observations = \
                getattr(self, "rejected_invalid_observations", 0) + 1
            return
        if not self._ever_observed:
            self._accept_validity(observation)
            return
        if key == self._validity.identity:
            high_water = self._generation_high_water.get(key, 0)
            if observation.generation < high_water:
                # Same identity, older generation: delayed publication OR a
                # restarted producer renumbering from low generations —
                # indistinguishable here; park for controlled confirmation.
                self.pending_confirmation[key] = observation
                self.rejected_backward_publications += 1
                return
            if observation.generation == high_water:
                previous = self._validity
                if (observation.state != previous.state
                        or observation.reset_id != previous.reset_id):
                    # Same generation must mean the same fact; a different
                    # payload at the same generation is a conflict, never a
                    # silent overwrite.
                    self.rejected_content_conflicts += 1
                    return
                return  # duplicate publication, idempotent
            self._accept_validity(observation)
            return
        # A different identity while another producer is current.
        if key in self._retired_keys:
            # Superseded identity: a delayed old publication, whatever its
            # generation says. Only controlled confirmation (the producer's
            # real status service) may bring an identity back.
            self.pending_confirmation[key] = observation
            self.rejected_backward_publications += 1
            return
        # Same-producer epoch rollover (a reset increments BOTH epoch and
        # generation): generation strictly advances, so an unseen identity
        # with a higher generation is the producer moving forward. A
        # restarted producer renumbers from LOW generations and cannot take
        # this branch; a delayed message from a superseded identity is
        # caught by the retired check above (delivery of a dead producer's
        # messages to a subscriber that never saw it is not possible —
        # latch and reliable redelivery are per-live-publisher).
        if observation.generation > self._validity.generation:
            self._accept_validity(observation)
            return
        self.pending_confirmation[key] = observation
        self.rejected_backward_publications += 1
        return

    def confirm_producer(self, status: ValidityObservation) -> Decision:
        """Controlled confirmation of the producer's CURRENT identity.

        The node adapter calls this with the response of the producer's
        REAL status service (observer ``get_validity`` / grid
        ``/cargo_map/get_validity``) — never with a re-cached stream
        message. The status is authoritative: if it confirms a parked
        identity (a restarted producer), the gate switches to it and
        recalibrates that key's high-water; if it re-affirms the current
        identity, the parked publication was a delayed old stream and
        stays rejected.
        """
        if not self._ever_observed:
            self._accept_validity(status)
            return Decision(True, self.REASON_OK, self._validity)
        key = status.identity
        parked = self.pending_confirmation.get(key)
        if parked is not None:
            if status.generation >= parked.generation:
                # The producer itself declares this identity current.
                self._accept_validity(status)
                return Decision(True, self.REASON_OK, self._validity)
            return Decision(False, self.REASON_LATE_OLD_RESULT,
                            self._validity)
        if key == self._validity.identity:
            return Decision(True, self.REASON_OK, self._validity)
        # Status shows a third identity: adopt the producer's now-current
        # state (it outranks every stream message).
        self._accept_validity(status)
        return Decision(True, self.REASON_OK, self._validity)

    def observe_map(self, sample: MapObservation, *, geometry_hash: str = "",
                    config_digest: str = "") -> Decision:
        """Offer a freshly computed surface/cargo map for admission."""
        validity = self._validity
        if validity.state != "VALID":
            self.invalidated_candidates += 1
            return Decision(False, self._closed_reason(validity), validity)
        if (sample.backend and validity.backend
                and sample.backend != validity.backend):
            # Cross-backend splice: a G epoch presented on an O stream (or
            # vice versa) is rejected even if the strings were made to
            # collide (I2).
            self.rejected_late_results += 1
            self.invalidated_candidates += 1
            return Decision(False, self.REASON_MAP_BACKEND, validity)
        if not sample.epoch:
            self.rejected_late_results += 1
            self.invalidated_candidates += 1
            return Decision(False, self.REASON_MAP_EPOCH_MISSING, validity)
        if sample.epoch != validity.epoch:
            self.rejected_late_results += 1
            self.invalidated_candidates += 1
            return Decision(False, self.REASON_MAP_EPOCH, validity)
        if not sample.generation:
            # Unconditional binding (G1): both production backends publish
            # validity_generation; a map without one is not admissible.
            self.rejected_late_results += 1
            self.invalidated_candidates += 1
            return Decision(False, self.REASON_MAP_GENERATION_MISSING,
                            validity)
        if sample.generation != validity.generation:
            self.rejected_late_results += 1
            self.invalidated_candidates += 1
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
                               self._validity.generation,
                               self._validity.backend)
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

    def accept_result(self, result_epoch: Any,
                      result_generation: int = 0,
                      backend: str = "") -> Decision:
        """Terminal check for a late-arriving placement/Surface result.

        A result computed against an older generation is tagged invalid and
        counted; it never becomes a valid placement result.
        """
        validity = self._validity
        epoch = normalize_epoch(result_epoch, backend)
        if epoch != validity.epoch or (
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
