"""Pure suction retry / vacuum-failure state contracts.

The reducer models the plan-section-D retry boundary only. It never
calls ROS, sleeps, reads configuration, looks up TF, or executes
motion. Every ``(state, event)`` pair is either handled explicitly or
raises :class:`UnhandledRetryTransition` — there are no silent
defaults, so an unmodelled situation is a loud, fail-closed stop.

Failure boundaries (C6):

* ``pre_seal`` — before DI0 confirms a seal, failures may only return
  through bounded candidate retry or non-moving recovery; vacuum is
  never preserved.
* ``retry_recovery`` — inside the release/reverse/verify recovery, any
  failure terminates with ``SUCTION_RETRY_RECOVERY_FAILED`` before any
  lateral motion; no next candidate is planned.
* ``carry`` — after DI0 confirms the seal, any pressure loss or motion
  failure enters ``CARRY_FAULT`` with vacuum preserved until an
  explicit recovery confirms release.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from enum import Enum
from typing import FrozenSet, Iterable, Mapping, Optional, Tuple

# --- plan-fixed constants -------------------------------------------------

RELEASE_LOW_HOLD_SEC = 0.5     #: DI0 must be observed low this long, continuously
RELEASE_DEADLINE_SEC = 2.0     #: ... within this deadline after the seal timeout
REVERSE_MIN_M = 0.08           #: minimum reverse retreat along the candidate normal
REQUEST_DEADLINE_SEC = 180.0   #: whole-request budget (C2 exhaustion deadline)
MAX_CANDIDATE_ATTEMPTS = 3     #: at most three candidates per operator request

SUCTION_RETRY_RECOVERY_FAILED = "SUCTION_RETRY_RECOVERY_FAILED"
SUCTION_CANDIDATES_EXHAUSTED = "SUCTION_CANDIDATES_EXHAUSTED"
VACUUM_SEAL_LOST = "VACUUM_SEAL_LOST"

# terminal detail codes (diagnostics carried next to the reason)
RETRY_RELEASE_LOW_TIMEOUT = "RETRY_RELEASE_LOW_TIMEOUT"
RETRY_DI0_STUCK_HIGH = "RETRY_DI0_STUCK_HIGH"
RETRY_DI0_AMBIGUOUS = "RETRY_DI0_AMBIGUOUS"
RETRY_VACUUM_SERVICE_ERROR = "RETRY_VACUUM_SERVICE_ERROR"
RETRY_RECOVERY_FRACTION = "RETRY_RECOVERY_FRACTION"
RETRY_CONTROLLER_FAILURE = "RETRY_CONTROLLER_FAILURE"
RETRY_GENERATION_CHANGED = "RETRY_GENERATION_CHANGED"
RETRY_GRAPH_UNHEALTHY = "RETRY_GRAPH_UNHEALTHY"
RETRY_BUDGET_EXPIRED = "RETRY_BUDGET_EXPIRED"


class _FrozenMapping(tuple):
    pass


class RetryState(str, Enum):
    IDLE = "IDLE"
    SELECTING = "SELECTING"
    PRE_GRASP_MOTION = "PRE_GRASP_MOTION"
    APPROACH_MOTION = "APPROACH_MOTION"
    ATTACH_MOTION = "ATTACH_MOTION"
    VACUUM_SEAL_WAIT = "VACUUM_SEAL_WAIT"
    RELEASE_LOW_WAIT = "RELEASE_LOW_WAIT"
    RETRY_REVERSE_MOTION = "RETRY_REVERSE_MOTION"
    RECOVERY_VERIFY = "RECOVERY_VERIFY"
    SCENE_ATTACH = "SCENE_ATTACH"
    PICK_RETREAT_MOTION = "PICK_RETREAT_MOTION"
    COMPLETED = "COMPLETED"
    CANDIDATES_EXHAUSTED = "CANDIDATES_EXHAUSTED"
    RETRY_RECOVERY_FAILED = "RETRY_RECOVERY_FAILED"
    MOTION_FAILED = "MOTION_FAILED"
    CARRY_FAULT = "CARRY_FAULT"
    REJECTED = "REJECTED"


class RetryEventType(str, Enum):
    REQUEST_START = "REQUEST_START"
    REQUEST_REJECTED = "REQUEST_REJECTED"
    CANDIDATE_SELECTED = "CANDIDATE_SELECTED"
    CANDIDATE_REJECTED = "CANDIDATE_REJECTED"
    SEGMENT_SUCCEEDED = "SEGMENT_SUCCEEDED"
    SEGMENT_FAILED = "SEGMENT_FAILED"
    VACUUM_SEALED = "VACUUM_SEALED"
    VACUUM_SEAL_TIMEOUT = "VACUUM_SEAL_TIMEOUT"
    VACUUM_SERVICE_ERROR = "VACUUM_SERVICE_ERROR"
    DI0_SAMPLE = "DI0_SAMPLE"
    RECOVERY_VERIFIED = "RECOVERY_VERIFIED"
    IDENTITY_CHANGED = "IDENTITY_CHANGED"
    GRAPH_LOST = "GRAPH_LOST"
    SCENE_ATTACH_SUCCEEDED = "SCENE_ATTACH_SUCCEEDED"
    SCENE_ATTACH_FAILED = "SCENE_ATTACH_FAILED"
    BUDGET_EXPIRED = "BUDGET_EXPIRED"


class RetryEffectType(str, Enum):
    EXECUTE_SEGMENT = "EXECUTE_SEGMENT"
    VACUUM_ENABLE = "VACUUM_ENABLE"
    SCENE_ATTACH = "SCENE_ATTACH"
    PUBLISH_STATUS = "PUBLISH_STATUS"


class FailureBoundary(str, Enum):
    PRE_SEAL = "pre_seal"
    RETRY_RECOVERY = "retry_recovery"
    CARRY = "carry"
    NONE = "none"


#: Declarative boundary table (public; tests enumerate every row).
RETRY_BOUNDARIES = {
    FailureBoundary.PRE_SEAL: {
        "states": (
            RetryState.SELECTING,
            RetryState.PRE_GRASP_MOTION,
            RetryState.APPROACH_MOTION,
            RetryState.ATTACH_MOTION,
            RetryState.VACUUM_SEAL_WAIT,
            RetryState.REJECTED,
            RetryState.MOTION_FAILED,
            RetryState.CANDIDATES_EXHAUSTED,
        ),
        "vacuum_preserved": False,
        "handler": "bounded_retry_or_stop",
    },
    FailureBoundary.RETRY_RECOVERY: {
        "states": (
            RetryState.RELEASE_LOW_WAIT,
            RetryState.RETRY_REVERSE_MOTION,
            RetryState.RECOVERY_VERIFY,
            RetryState.RETRY_RECOVERY_FAILED,
        ),
        "vacuum_preserved": False,
        "handler": "stop_no_lateral",
    },
    FailureBoundary.CARRY: {
        "states": (
            RetryState.SCENE_ATTACH,
            RetryState.PICK_RETREAT_MOTION,
            RetryState.CARRY_FAULT,
        ),
        "vacuum_preserved": True,
        "handler": "carry_fault",
    },
    FailureBoundary.NONE: {
        "states": (RetryState.IDLE, RetryState.COMPLETED),
        "vacuum_preserved": False,
        "handler": "n/a",
    },
}


@dataclass(frozen=True)
class RetryEffect:
    effect_type: RetryEffectType
    name: str
    motion: bool = False
    vacuum: bool = False
    detect: bool = False
    scene: bool = False
    lateral: bool = False
    payload: Mapping[str, object] = None

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "payload", _freeze_mapping(self.payload or {}))

    def payload_dict(self) -> dict:
        return _plain_mapping(self.payload)

    def to_dict(self) -> dict:
        return {
            "effect_type": self.effect_type.value,
            "name": self.name,
            "motion": self.motion,
            "vacuum": self.vacuum,
            "detect": self.detect,
            "scene": self.scene,
            "lateral": self.lateral,
            "payload": self.payload_dict(),
        }


@dataclass(frozen=True)
class RetryEvent:
    event_type: RetryEventType
    candidate_id: str = ""
    segment: str = ""
    ok: bool = False
    reason_code: str = ""
    detail: str = ""
    t: float = 0.0
    di0: Optional[int] = None     # 0 low, 1 high, None unknown/ambiguous
    fraction: float = -1.0
    ranked_ids: Tuple[str, ...] = ()


@dataclass(frozen=True)
class RetryModel:
    state: RetryState = RetryState.IDLE
    ranked_ids: Tuple[str, ...] = ()
    attempted_ids: FrozenSet[str] = frozenset()
    rejections: Tuple[Tuple[str, str, str], ...] = ()
    active_candidate: str = ""
    max_attempts: int = MAX_CANDIDATE_ATTEMPTS
    request_started_t: float = 0.0
    vacuum_on: bool = False
    di0_confirmed: bool = False
    release_confirmed: bool = False
    release_window_started: float = 0.0
    release_low_since: Optional[float] = None
    saw_low_sample: bool = False
    vacuum_enable_count: int = 0
    scene_attached: bool = False
    sealed_candidate: str = ""
    reason_code: str = ""
    detail: str = ""

    @property
    def boundary(self) -> FailureBoundary:
        for boundary, spec in RETRY_BOUNDARIES.items():
            if self.state in spec["states"]:
                return boundary
        return FailureBoundary.NONE


@dataclass(frozen=True)
class Transition:
    model: RetryModel
    effects: Tuple[RetryEffect, ...]


class UnhandledRetryTransition(ValueError):
    """No explicit (state, event) handler — a fail-closed modelling gap."""


def initial_retry_model(max_attempts=MAX_CANDIDATE_ATTEMPTS) -> RetryModel:
    return RetryModel(max_attempts=int(max_attempts))


def is_terminal(state) -> bool:
    return state in (
        RetryState.COMPLETED,
        RetryState.CANDIDATES_EXHAUSTED,
        RetryState.RETRY_RECOVERY_FAILED,
        RetryState.MOTION_FAILED,
        RetryState.CARRY_FAULT,
        RetryState.REJECTED,
    )


def has_motion(effects: Iterable[RetryEffect]) -> bool:
    return any(effect.motion for effect in effects)


def has_vacuum(effects: Iterable[RetryEffect]) -> bool:
    return any(effect.vacuum for effect in effects)


def has_detect(effects: Iterable[RetryEffect]) -> bool:
    return any(effect.detect for effect in effects)


def has_lateral(effects: Iterable[RetryEffect]) -> bool:
    return any(effect.lateral for effect in effects)


def reduce_retry_event(model: RetryModel, event: RetryEvent) -> Transition:
    handler = _HANDLERS.get(model.state)
    if handler is None:
        raise UnhandledRetryTransition(
            "no handler for state %s" % model.state.value)
    return handler(model, event)


# --- handlers --------------------------------------------------------------


def _handle_idle(model: RetryModel, event: RetryEvent) -> Transition:
    if event.event_type == RetryEventType.REQUEST_START:
        if not event.ranked_ids:
            return _terminal(
                model, RetryState.REJECTED, SUCTION_CANDIDATES_EXHAUSTED,
                "no candidates on request")
        next_state = replace(
            model,
            state=RetryState.SELECTING,
            ranked_ids=tuple(event.ranked_ids),
            request_started_t=float(event.t),
        )
        return Transition(next_state, (_status("selecting", next_state),))
    if event.event_type == RetryEventType.REQUEST_REJECTED:
        return _terminal(
            model, RetryState.REJECTED,
            event.reason_code or "request_rejected", event.detail)
    raise UnhandledRetryTransition("IDLE cannot handle %s"
                                   % event.event_type.value)


def _unattempted(model: RetryModel):
    consumed = set(model.attempted_ids) | set(
        row[0] for row in model.rejections)
    return tuple(cid for cid in model.ranked_ids if cid not in consumed)


def _handle_selecting(model: RetryModel, event: RetryEvent) -> Transition:
    if event.event_type == RetryEventType.CANDIDATE_SELECTED:
        consumed = set(model.attempted_ids) | set(
            row[0] for row in model.rejections)
        if event.candidate_id in consumed:
            raise UnhandledRetryTransition(
                "candidate %s already attempted or rejected in this request"
                % event.candidate_id)
        if (len(model.attempted_ids) >= model.max_attempts
                or not _unattempted(model)):
            return _terminal(
                model, RetryState.CANDIDATES_EXHAUSTED,
                SUCTION_CANDIDATES_EXHAUSTED, "attempt cap reached")
        next_state = replace(
            model,
            state=RetryState.PRE_GRASP_MOTION,
            attempted_ids=model.attempted_ids | frozenset({event.candidate_id}),
            active_candidate=event.candidate_id,
            vacuum_enable_count=0,
        )
        return Transition(next_state, (
            _execute_segment("pre_grasp", event.candidate_id, lateral=True),
            _status("candidate_selected", next_state),
        ))
    if event.event_type == RetryEventType.CANDIDATE_REJECTED:
        # probe-stage rejection: recorded, but does NOT consume the
        # physical attempt cap (gate C1: fourth of five selected after
        # three probe failures)
        rejections = model.rejections + (
            (event.candidate_id, event.reason_code or "rejected",
             event.detail),)
        next_state = replace(model, rejections=rejections)
        if not _unattempted(next_state):
            return _terminal(next_state, RetryState.CANDIDATES_EXHAUSTED,
                             SUCTION_CANDIDATES_EXHAUSTED,
                             "all candidates probe-rejected")
        return Transition(next_state, (_status("candidate_rejected",
                                               next_state),))
    if event.event_type == RetryEventType.REQUEST_REJECTED:
        return _terminal(model, RetryState.REJECTED,
                         event.reason_code or "request_rejected",
                         event.detail)
    if event.event_type == RetryEventType.BUDGET_EXPIRED:
        return _terminal(model, RetryState.CANDIDATES_EXHAUSTED,
                         SUCTION_CANDIDATES_EXHAUSTED, RETRY_BUDGET_EXPIRED)
    raise UnhandledRetryTransition("SELECTING cannot handle %s"
                                   % event.event_type.value)


def _handle_pre_grasp(model: RetryModel, event: RetryEvent) -> Transition:
    if event.event_type == RetryEventType.SEGMENT_SUCCEEDED:
        next_state = replace(model, state=RetryState.APPROACH_MOTION)
        return Transition(next_state, (
            _execute_segment("approach", model.active_candidate),
            _status("pre_grasp_done", next_state),
        ))
    return _pre_seal_failure(model, event, "PRE_GRASP_MOTION")


def _handle_approach(model: RetryModel, event: RetryEvent) -> Transition:
    if event.event_type == RetryEventType.SEGMENT_SUCCEEDED:
        next_state = replace(model, state=RetryState.ATTACH_MOTION)
        return Transition(next_state, (
            _execute_segment("attach", model.active_candidate),
            _status("approach_done", next_state),
        ))
    return _pre_seal_failure(model, event, "APPROACH_MOTION")


def _handle_attach(model: RetryModel, event: RetryEvent) -> Transition:
    if event.event_type == RetryEventType.SEGMENT_SUCCEEDED:
        next_state = replace(
            model,
            state=RetryState.VACUUM_SEAL_WAIT,
            vacuum_enable_count=model.vacuum_enable_count + 1,
        )
        return Transition(next_state, (
            RetryEffect(RetryEffectType.VACUUM_ENABLE, "VacuumCommand",
                        vacuum=True,
                        payload={"candidate_id": model.active_candidate}),
            _status("attach_done", next_state),
        ))
    return _pre_seal_failure(model, event, "ATTACH_MOTION")


def _pre_seal_failure(model: RetryModel, event: RetryEvent,
                      state_name: str) -> Transition:
    if event.event_type == RetryEventType.SEGMENT_FAILED:
        # Pre-seal motion failure: non-moving stop. No lateral motion, no
        # next candidate, vacuum was never confirmed on.
        return _terminal(model, RetryState.MOTION_FAILED,
                         event.reason_code or "segment_failed",
                         "%s:%s" % (state_name, event.detail))
    if event.event_type == RetryEventType.BUDGET_EXPIRED:
        return _terminal(model, RetryState.CANDIDATES_EXHAUSTED,
                         SUCTION_CANDIDATES_EXHAUSTED, RETRY_BUDGET_EXPIRED)
    raise UnhandledRetryTransition("%s cannot handle %s"
                                   % (state_name, event.event_type.value))


def _handle_seal_wait(model: RetryModel, event: RetryEvent) -> Transition:
    if event.event_type == RetryEventType.VACUUM_SEALED:
        next_state = replace(
            model,
            state=RetryState.SCENE_ATTACH,
            di0_confirmed=True,
            vacuum_on=True,
            sealed_candidate=model.active_candidate,
        )
        return Transition(next_state, (
            RetryEffect(RetryEffectType.SCENE_ATTACH, "AttachSceneObject",
                        scene=True,
                        payload={"candidate_id": model.active_candidate}),
            _status("sealed", next_state),
        ))
    if event.event_type == RetryEventType.VACUUM_SEAL_TIMEOUT:
        # Backend completed its release sequence inside the timed-out
        # attach; from here DI0 must be observed low before anything moves.
        next_state = replace(
            model,
            state=RetryState.RELEASE_LOW_WAIT,
            release_window_started=float(event.t),
            release_low_since=None,
            saw_low_sample=False,
        )
        return Transition(next_state, (_status("seal_timeout", next_state),))
    if event.event_type == RetryEventType.VACUUM_SERVICE_ERROR:
        # Vacuum state unknown with the cup on the surface: stop cold, no
        # further motion, no vacuum commands.
        return _terminal(model, RetryState.RETRY_RECOVERY_FAILED,
                         SUCTION_RETRY_RECOVERY_FAILED,
                         RETRY_VACUUM_SERVICE_ERROR)
    if event.event_type == RetryEventType.BUDGET_EXPIRED:
        return _terminal(model, RetryState.RETRY_RECOVERY_FAILED,
                         SUCTION_RETRY_RECOVERY_FAILED, RETRY_BUDGET_EXPIRED)
    raise UnhandledRetryTransition("VACUUM_SEAL_WAIT cannot handle %s"
                                   % event.event_type.value)


def _release_window_failure(model: RetryModel, t: float) -> Transition:
    if model.saw_low_sample:
        detail = RETRY_RELEASE_LOW_TIMEOUT
    else:
        detail = RETRY_DI0_STUCK_HIGH
    return _terminal(replace(model, release_low_since=None),
                     RetryState.RETRY_RECOVERY_FAILED,
                     SUCTION_RETRY_RECOVERY_FAILED, detail)


def _handle_release_wait(model: RetryModel, event: RetryEvent) -> Transition:
    if event.event_type == RetryEventType.DI0_SAMPLE:
        t = float(event.t)
        if event.di0 is None:
            next_state = replace(model, release_low_since=None)
            return _terminal(next_state, RetryState.RETRY_RECOVERY_FAILED,
                             SUCTION_RETRY_RECOVERY_FAILED,
                             RETRY_DI0_AMBIGUOUS)
        if t - model.release_window_started > RELEASE_DEADLINE_SEC:
            return _release_window_failure(model, t)
        if event.di0 == 1:
            next_state = replace(model, release_low_since=None)
            return Transition(next_state, (_status("di0_high_reset",
                                                   next_state),))
        # DI0 low: hold RELEASE_LOW_HOLD_SEC continuously
        low_since = (model.release_low_since if model.release_low_since
                     is not None else t)
        held = t - low_since
        next_state = replace(model, release_low_since=low_since,
                             saw_low_sample=True)
        if held >= RELEASE_LOW_HOLD_SEC:
            confirmed = replace(next_state, release_confirmed=True,
                                state=RetryState.RETRY_REVERSE_MOTION)
            return Transition(confirmed, (
                _execute_segment("retry_reverse", model.active_candidate),
                _status("release_confirmed", confirmed),
            ))
        return Transition(next_state, (_status("di0_low_holding",
                                               next_state),))
    if event.event_type == RetryEventType.BUDGET_EXPIRED:
        return _terminal(model, RetryState.RETRY_RECOVERY_FAILED,
                         SUCTION_RETRY_RECOVERY_FAILED, RETRY_BUDGET_EXPIRED)
    raise UnhandledRetryTransition("RELEASE_LOW_WAIT cannot handle %s"
                                   % event.event_type.value)


def _handle_retry_reverse(model: RetryModel, event: RetryEvent) -> Transition:
    if event.event_type == RetryEventType.SEGMENT_SUCCEEDED:
        next_state = replace(model, state=RetryState.RECOVERY_VERIFY)
        return Transition(next_state, (_status("recovery_reverse_done",
                                               next_state),))
    if event.event_type == RetryEventType.SEGMENT_FAILED:
        if 0.0 <= float(event.fraction) < 1.0:
            detail = RETRY_RECOVERY_FRACTION
        else:
            detail = RETRY_CONTROLLER_FAILURE
        return _terminal(model, RetryState.RETRY_RECOVERY_FAILED,
                         SUCTION_RETRY_RECOVERY_FAILED, detail)
    if event.event_type == RetryEventType.BUDGET_EXPIRED:
        return _terminal(model, RetryState.RETRY_RECOVERY_FAILED,
                         SUCTION_RETRY_RECOVERY_FAILED, RETRY_BUDGET_EXPIRED)
    raise UnhandledRetryTransition("RETRY_REVERSE_MOTION cannot handle %s"
                                   % event.event_type.value)


def _handle_recovery_verify(model: RetryModel, event: RetryEvent) -> Transition:
    if event.event_type == RetryEventType.RECOVERY_VERIFIED:
        if not _unattempted(model) or (
                len(model.attempted_ids) >= model.max_attempts):
            return _terminal(model, RetryState.CANDIDATES_EXHAUSTED,
                             SUCTION_CANDIDATES_EXHAUSTED, "")
        next_state = replace(model, state=RetryState.SELECTING)
        return Transition(next_state, (_status("recovery_verified",
                                               next_state),))
    if event.event_type == RetryEventType.IDENTITY_CHANGED:
        return _terminal(model, RetryState.RETRY_RECOVERY_FAILED,
                         SUCTION_RETRY_RECOVERY_FAILED,
                         RETRY_GENERATION_CHANGED)
    if event.event_type == RetryEventType.GRAPH_LOST:
        return _terminal(model, RetryState.RETRY_RECOVERY_FAILED,
                         SUCTION_RETRY_RECOVERY_FAILED,
                         RETRY_GRAPH_UNHEALTHY)
    if event.event_type == RetryEventType.BUDGET_EXPIRED:
        return _terminal(model, RetryState.RETRY_RECOVERY_FAILED,
                         SUCTION_RETRY_RECOVERY_FAILED, RETRY_BUDGET_EXPIRED)
    raise UnhandledRetryTransition("RECOVERY_VERIFY cannot handle %s"
                                   % event.event_type.value)


def _carry_boundary(model: RetryModel, event: RetryEvent,
                    state_name: str) -> Transition:
    """Shared handling for post-seal carry states (C6)."""
    if event.event_type == RetryEventType.DI0_SAMPLE:
        if event.di0 == 1:
            # healthy sample while carrying: explicitly ignored
            return Transition(model, ())
        detail = ("" if event.di0 == 0 else RETRY_DI0_AMBIGUOUS)
        return _terminal(
            replace(model, vacuum_on=True), RetryState.CARRY_FAULT,
            VACUUM_SEAL_LOST, detail or "di0_dropped_while_carrying")
    if event.event_type == RetryEventType.BUDGET_EXPIRED:
        return _terminal(replace(model, vacuum_on=True),
                         RetryState.CARRY_FAULT, SUCTION_RETRY_RECOVERY_FAILED,
                         RETRY_BUDGET_EXPIRED)
    raise UnhandledRetryTransition("%s cannot handle %s"
                                   % (state_name, event.event_type.value))


def _handle_scene_attach(model: RetryModel, event: RetryEvent) -> Transition:
    if event.event_type == RetryEventType.SCENE_ATTACH_SUCCEEDED:
        next_state = replace(model, state=RetryState.PICK_RETREAT_MOTION,
                             scene_attached=True)
        return Transition(next_state, (
            _execute_segment("pick_retreat", model.sealed_candidate),
            _status("scene_attached", next_state),
        ))
    if event.event_type == RetryEventType.SCENE_ATTACH_FAILED:
        return _terminal(replace(model, vacuum_on=True),
                         RetryState.CARRY_FAULT,
                         event.reason_code or "scene_attach_failed",
                         event.detail)
    return _carry_boundary(model, event, "SCENE_ATTACH")


def _handle_pick_retreat(model: RetryModel, event: RetryEvent) -> Transition:
    if event.event_type == RetryEventType.SEGMENT_SUCCEEDED:
        next_state = replace(model, state=RetryState.COMPLETED)
        return Transition(next_state, (_status("completed", next_state),))
    if event.event_type == RetryEventType.SEGMENT_FAILED:
        return _terminal(replace(model, vacuum_on=True),
                         RetryState.CARRY_FAULT,
                         event.reason_code or "pick_retreat_failed",
                         event.detail)
    return _carry_boundary(model, event, "PICK_RETREAT_MOTION")


def _handle_terminal(model: RetryModel, event: RetryEvent) -> Transition:
    raise UnhandledRetryTransition(
        "terminal state %s cannot handle %s"
        % (model.state.value, event.event_type.value))


_HANDLERS = {
    RetryState.IDLE: _handle_idle,
    RetryState.SELECTING: _handle_selecting,
    RetryState.PRE_GRASP_MOTION: _handle_pre_grasp,
    RetryState.APPROACH_MOTION: _handle_approach,
    RetryState.ATTACH_MOTION: _handle_attach,
    RetryState.VACUUM_SEAL_WAIT: _handle_seal_wait,
    RetryState.RELEASE_LOW_WAIT: _handle_release_wait,
    RetryState.RETRY_REVERSE_MOTION: _handle_retry_reverse,
    RetryState.RECOVERY_VERIFY: _handle_recovery_verify,
    RetryState.SCENE_ATTACH: _handle_scene_attach,
    RetryState.PICK_RETREAT_MOTION: _handle_pick_retreat,
    RetryState.COMPLETED: _handle_terminal,
    RetryState.CANDIDATES_EXHAUSTED: _handle_terminal,
    RetryState.RETRY_RECOVERY_FAILED: _handle_terminal,
    RetryState.MOTION_FAILED: _handle_terminal,
    RetryState.CARRY_FAULT: _handle_terminal,
    RetryState.REJECTED: _handle_terminal,
}


# --- effect helpers ---------------------------------------------------------


def _execute_segment(segment: str, candidate_id: str,
                     lateral: bool = False) -> RetryEffect:
    return RetryEffect(
        RetryEffectType.EXECUTE_SEGMENT, "PlanMotion",
        motion=True, lateral=lateral,
        payload={"segment": segment, "candidate_id": candidate_id},
    )


def _status(reason_code: str, model: RetryModel,
            **payload: object) -> RetryEffect:
    status = {
        "state": model.state.value,
        "reason_code": reason_code,
        "boundary": model.boundary.value,
        "vacuum_on": model.vacuum_on,
        "di0_confirmed": model.di0_confirmed,
        "active_candidate": model.active_candidate,
    }
    status.update(payload)
    return RetryEffect(RetryEffectType.PUBLISH_STATUS, "RetryStatus",
                       payload=status)


def _terminal(model: RetryModel, state: RetryState, reason_code: str,
              detail: str) -> Transition:
    next_state = replace(model, state=state, reason_code=reason_code,
                         detail=detail)
    return Transition(next_state, (_status(reason_code, next_state,
                                           detail=detail),))


# --- trace audit (C4 invariants) -------------------------------------------


@dataclass(frozen=True)
class AuditCounters:
    order_violations: int = 0
    lateral_before_release_confirmed: int = 0
    vacuum_on_lateral_motion: int = 0
    duplicate_scene_attach: int = 0
    unauthorized_detect: int = 0
    unauthorized_vacuum_during_recovery: int = 0


def audit_trace(records) -> AuditCounters:
    """Count C4 invariant violations in a canonical session trace.

    Records are the session's trace dicts with ``kind`` in
    ``{"event", "effect"}``. Effects carry ``effect_type`` plus the
    capability flags of :class:`RetryEffect`; the interesting events are
    ``seal_timeout`` / ``release_confirmed`` / ``sealed``.
    """
    counters = {
        "order_violations": 0,
        "lateral_before_release_confirmed": 0,
        "vacuum_on_lateral_motion": 0,
        "duplicate_scene_attach": 0,
        "unauthorized_detect": 0,
        "unauthorized_vacuum_during_recovery": 0,
    }
    in_recovery = False
    vacuum_on = False
    scene_attaches = 0
    sealed_seen = False
    for record in records:
        kind = record.get("kind", "")
        if kind == "event":
            name = record.get("name", "")
            if name == "seal_timeout":
                in_recovery = True
                # backend completed its release sequence in the timed-out
                # attach: DO0 is off, DI0 is expected to settle low
                vacuum_on = False
            elif name == "release_confirmed":
                in_recovery = False
            elif name == "sealed":
                sealed_seen = True
                vacuum_on = True
            continue
        if kind != "effect":
            continue
        effect_type = record.get("effect_type", "")
        if record.get("detect", False):
            counters["unauthorized_detect"] += 1
        if effect_type == "SCENE_ATTACH":
            scene_attaches += 1
            if scene_attaches > 1:
                counters["duplicate_scene_attach"] += 1
            if not sealed_seen:
                counters["order_violations"] += 1
        elif effect_type == "VACUUM_ENABLE":
            if in_recovery:
                counters["unauthorized_vacuum_during_recovery"] += 1
            vacuum_on = True
        elif effect_type == "EXECUTE_SEGMENT":
            segment = str(record.get("payload", {}).get("segment", ""))
            lateral = bool(record.get("lateral", False)) or segment == "pre_grasp"
            if in_recovery and segment != "retry_reverse":
                counters["lateral_before_release_confirmed"] += 1
            if lateral and vacuum_on:
                counters["vacuum_on_lateral_motion"] += 1
    return AuditCounters(**counters)


# --- frozen mapping helpers (same contract as orchestration_contracts) ------


def _freeze_plain(value: object) -> object:
    if value is None or isinstance(value, (str, bool)):
        return value
    if isinstance(value, int) and not isinstance(value, bool):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("plain float values must be finite")
        return value
    if isinstance(value, Mapping):
        return _freeze_mapping(value)
    if isinstance(value, (list, tuple)):
        return tuple(_freeze_plain(item) for item in value)
    raise TypeError("unsupported plain value type: %s"
                    % type(value).__name__)


def _plain_value(value: object) -> object:
    if isinstance(value, _FrozenMapping):
        return _plain_mapping(value)
    if isinstance(value, tuple):
        return [_plain_value(item) for item in value]
    return value


def _freeze_mapping(values: Mapping[str, object]):
    frozen = []
    for key, value in values.items():
        if not isinstance(key, str):
            raise TypeError("plain mapping keys must be strings")
        frozen.append((key, _freeze_plain(value)))
    return _FrozenMapping(sorted(frozen))


def _plain_mapping(values) -> dict:
    return {key: _plain_value(value) for key, value in values}
