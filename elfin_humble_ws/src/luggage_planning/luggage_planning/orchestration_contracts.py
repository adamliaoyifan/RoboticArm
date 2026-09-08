"""Pure production orchestrator state/effect contracts.

The reducer in this module models authorization boundaries only. It never calls
ROS, sleeps, reads configuration, looks up TF, or executes motion.
"""

from __future__ import annotations

import math
import uuid
from dataclasses import dataclass, replace
from enum import Enum
from typing import FrozenSet, Iterable, Mapping, Optional, Tuple


SCHEMA_VERSION = 1


class _FrozenMapping(tuple):
    pass


class OrchestratorState(str, Enum):
    WAIT_START = "WAIT_START"
    RESET_CARGO_MAP = "RESET_CARGO_MAP"
    EXPLORE_CONTAINER = "EXPLORE_CONTAINER"
    RETURN_PICK_OBSERVE = "RETURN_PICK_OBSERVE"
    WAIT_PICKUP_READY = "WAIT_PICKUP_READY"
    DETECT = "DETECT"
    COMPUTE_PLACEMENT = "COMPUTE_PLACEMENT"
    PLAN_PICK = "PLAN_PICK"
    EXEC_PICK = "EXEC_PICK"
    PLAN_PLACE = "PLAN_PLACE"
    EXEC_PLACE = "EXEC_PLACE"
    COMMIT_AND_VERIFY = "COMMIT_AND_VERIFY"
    CARRY_FAULT = "CARRY_FAULT"
    WAIT_RECOVERY = "WAIT_RECOVERY"
    ABORTED = "ABORTED"


class EventType(str, Enum):
    STARTUP = "STARTUP"
    SERVICE_READY = "SERVICE_READY"
    SENSOR_READY = "SENSOR_READY"
    TIMER = "TIMER"
    COMMAND = "COMMAND"
    PICKUP_READY = "PICKUP_READY"
    OPERATION_SUCCEEDED = "OPERATION_SUCCEEDED"
    OPERATION_FAILED = "OPERATION_FAILED"
    ABORT = "ABORT"
    RECOVERY_CONFIRMED = "RECOVERY_CONFIRMED"


class EffectType(str, Enum):
    CALL_ACTION = "CALL_ACTION"
    CALL_SERVICE = "CALL_SERVICE"
    PUBLISH_PROMPT = "PUBLISH_PROMPT"
    PUBLISH_STATUS = "PUBLISH_STATUS"


class FailureBoundary(str, Enum):
    PRE_PICK = "pre_pick"
    CARRYING = "carrying"
    RELEASE = "release"
    VERIFICATION = "verification"
    ABORT = "abort"
    RECOVERY = "recovery"


@dataclass(frozen=True)
class Effect:
    effect_type: EffectType
    name: str
    motion: bool = False
    vacuum: bool = False
    detect: bool = False
    payload: Mapping[str, object] = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "payload", _freeze_mapping(self.payload or {}))

    def payload_dict(self) -> dict[str, object]:
        return _plain_mapping(self.payload)

    def to_dict(self) -> dict[str, object]:
        return {
            "effect_type": self.effect_type.value,
            "name": self.name,
            "motion": self.motion,
            "vacuum": self.vacuum,
            "detect": self.detect,
            "payload": self.payload_dict(),
        }


@dataclass(frozen=True)
class OperatorEvent:
    event_type: EventType
    command: str = ""
    request_id: Optional[str] = None
    operator_id: str = ""
    reason_code: str = ""
    operation_id: Optional[str] = None
    box_id: Optional[str] = None
    session_id: Optional[str] = None
    schema_version: int = SCHEMA_VERSION
    payload_released: bool = False


@dataclass(frozen=True)
class PendingOperation:
    operation_id: str
    name: str


@dataclass(frozen=True)
class OrchestratorContractState:
    state: OrchestratorState = OrchestratorState.WAIT_START
    started: bool = False
    carrying: bool = False
    vacuum_enabled: bool = False
    session_id: Optional[str] = None
    current_request_id: Optional[str] = None
    consumed_request_ids: FrozenSet[str] = frozenset()
    prompt_sequence: int = 0
    operation_sequence: int = 0
    pending_operation: Optional[PendingOperation] = None
    active_box_id: Optional[str] = None
    committed_box_ids: FrozenSet[str] = frozenset()

    @property
    def placed_count(self) -> int:
        return len(self.committed_box_ids)

    @property
    def in_initial_exploration(self) -> bool:
        return self.state in {
            OrchestratorState.RESET_CARGO_MAP,
            OrchestratorState.EXPLORE_CONTAINER,
            OrchestratorState.RETURN_PICK_OBSERVE,
        }


@dataclass(frozen=True)
class Transition:
    state: OrchestratorContractState
    effects: Tuple[Effect, ...]


FAILURE_TRANSITIONS = {
    FailureBoundary.PRE_PICK: {
        "states": (
            OrchestratorState.WAIT_PICKUP_READY,
            OrchestratorState.DETECT,
            OrchestratorState.COMPUTE_PLACEMENT,
            OrchestratorState.PLAN_PICK,
            OrchestratorState.EXEC_PICK,
        ),
        "target": OrchestratorState.WAIT_PICKUP_READY,
        "vacuum_preserved": False,
    },
    FailureBoundary.CARRYING: {
        "states": (
            OrchestratorState.PLAN_PLACE,
            OrchestratorState.EXEC_PLACE,
        ),
        "target": OrchestratorState.CARRY_FAULT,
        "vacuum_preserved": True,
    },
    FailureBoundary.RELEASE: {
        "states": (OrchestratorState.EXEC_PLACE,),
        "target": OrchestratorState.CARRY_FAULT,
        "vacuum_preserved": True,
    },
    FailureBoundary.VERIFICATION: {
        "states": (OrchestratorState.COMMIT_AND_VERIFY,),
        "target": OrchestratorState.WAIT_RECOVERY,
        "vacuum_preserved": False,
    },
    FailureBoundary.ABORT: {
        "states": tuple(OrchestratorState),
        "target": OrchestratorState.ABORTED,
        "vacuum_preserved": "if_carrying",
    },
    FailureBoundary.RECOVERY: {
        "states": (
            OrchestratorState.CARRY_FAULT,
            OrchestratorState.WAIT_RECOVERY,
        ),
        "target": OrchestratorState.RETURN_PICK_OBSERVE,
        "vacuum_preserved": "until_explicit_release",
    },
}


def initial_state() -> OrchestratorContractState:
    return OrchestratorContractState()


def command(command_name: str, session_id: Optional[str] = None) -> OperatorEvent:
    if command_name.strip().lower() == "start" and session_id is None:
        session_id = "session-%s" % uuid.uuid4().hex
    return OperatorEvent(EventType.COMMAND, command=command_name, session_id=session_id)


def pickup_ready(
    request_id: Optional[str],
    operator_id: str = "",
    schema_version: int = SCHEMA_VERSION,
) -> OperatorEvent:
    return OperatorEvent(
        EventType.PICKUP_READY,
        request_id=request_id,
        operator_id=operator_id,
        schema_version=schema_version,
    )


def operation_succeeded(
    operation_id: Optional[str] = None, box_id: Optional[str] = None
) -> OperatorEvent:
    return OperatorEvent(
        EventType.OPERATION_SUCCEEDED, operation_id=operation_id, box_id=box_id
    )


def operation_failed(reason_code: str, operation_id: Optional[str] = None) -> OperatorEvent:
    return OperatorEvent(
        EventType.OPERATION_FAILED, reason_code=reason_code, operation_id=operation_id
    )


def has_motion(effects: Iterable[Effect]) -> bool:
    return any(effect.motion for effect in effects)


def has_vacuum(effects: Iterable[Effect]) -> bool:
    return any(effect.vacuum for effect in effects)


def has_detect(effects: Iterable[Effect]) -> bool:
    return any(effect.detect for effect in effects)


def reduce_event(
    model: OrchestratorContractState, event: OperatorEvent
) -> Transition:
    if event.event_type == EventType.COMMAND:
        return _handle_command(model, event)

    if event.event_type == EventType.PICKUP_READY:
        return _handle_pickup_ready(model, event)

    if event.event_type == EventType.OPERATION_SUCCEEDED:
        return _handle_success(model, event)

    if event.event_type == EventType.OPERATION_FAILED:
        return _handle_failure(model, event.reason_code or "operation_failed", event)

    if event.event_type == EventType.ABORT:
        return _handle_abort(model, event.reason_code or "operator_abort")

    if event.event_type == EventType.RECOVERY_CONFIRMED:
        return _handle_recovery(model, event)

    return _status(model, "ignored_before_start")


def _handle_command(
    model: OrchestratorContractState, event: OperatorEvent
) -> Transition:
    name = event.command.strip().lower()
    if model.state == OrchestratorState.WAIT_START and name == "start":
        session_id = _valid_session_id(event.session_id)
        if session_id is None:
            return _status(model, "start_session_id_required")
        base_state = replace(
            model,
            state=OrchestratorState.RESET_CARGO_MAP,
            started=True,
            session_id=session_id,
            pending_operation=None,
        )
        next_state, reset_effect = _begin_service(base_state, "ResetCargoMap")
        return Transition(
            next_state,
            (
                reset_effect,
                _publish_status("start_accepted", next_state),
            ),
        )

    if name == "start":
        return _status(model, "duplicate_start_rejected")

    if model.state == OrchestratorState.WAIT_START and name in {"run", "step"}:
        return _status(model, "%s_rejected_wait_start" % name)

    if name in {"pause", "status"}:
        return _status(model, name)

    if name == "abort":
        return _handle_abort(model, "operator_abort")

    return _status(model, "command_rejected")


def _handle_pickup_ready(
    model: OrchestratorContractState, event: OperatorEvent
) -> Transition:
    request_id = event.request_id or ""
    if event.schema_version != SCHEMA_VERSION or not request_id.strip():
        return _status(model, "pickup_ready_malformed")

    if request_id in model.consumed_request_ids:
        return _status(model, "pickup_ready_duplicate")

    if (
        model.state != OrchestratorState.WAIT_PICKUP_READY
        or request_id != model.current_request_id
    ):
        return _status(model, "pickup_ready_stale_or_wrong_id")

    base_state = replace(
        model,
        state=OrchestratorState.DETECT,
        current_request_id=None,
        consumed_request_ids=model.consumed_request_ids | frozenset({request_id}),
        active_box_id=None,
    )
    next_state, detect_effect = _begin_service(
        base_state, "DetectLuggage", detect=True
    )
    return Transition(
        next_state,
        (
            detect_effect,
            _publish_status("pickup_ready_accepted", next_state),
        ),
    )


def _handle_success(
    model: OrchestratorContractState, event: OperatorEvent
) -> Transition:
    if not _operation_matches(model, event):
        return _status(model, "operation_stale_or_wrong_id")
    completed = model.pending_operation
    model = replace(model, pending_operation=None)
    state = model.state
    operation_name = completed.name

    if state == OrchestratorState.RESET_CARGO_MAP and operation_name == "ResetCargoMap":
        base_state = replace(model, state=OrchestratorState.EXPLORE_CONTAINER)
        next_state, view_effect = _begin_action(base_state, "PlanNextCargoView")
        return Transition(
            next_state,
            (
                view_effect,
                _publish_status("cargo_map_reset", next_state),
            ),
        )

    if state == OrchestratorState.EXPLORE_CONTAINER and operation_name == "PlanNextCargoView":
        return _return_pick_observe(model, "exploration_complete")

    if (
        state == OrchestratorState.RETURN_PICK_OBSERVE
        and operation_name == "GoToRobotPose"
    ):
        return _enter_wait_pickup_ready(model, "pickup_prompt")

    if state == OrchestratorState.DETECT and operation_name == "DetectLuggage":
        box_id = _valid_identity(event.box_id)
        if box_id is None:
            return _return_pick_observe(model, "box_identity_required")
        if box_id in model.committed_box_ids:
            return _return_pick_observe(model, "box_identity_already_committed")
        base_state = replace(
            model, state=OrchestratorState.COMPUTE_PLACEMENT, active_box_id=box_id
        )
        next_state, compute_effect = _begin_service(base_state, "ComputePlacement")
        return Transition(
            next_state,
            (
                compute_effect,
                _publish_status("detection_complete", next_state),
            ),
        )

    if (
        state == OrchestratorState.COMPUTE_PLACEMENT
        and operation_name == "ComputePlacement"
    ):
        base_state = replace(model, state=OrchestratorState.PLAN_PICK)
        next_state, plan_effect = _begin_service(base_state, "BuildMotionSequence")
        return Transition(
            next_state,
            (
                plan_effect,
                _publish_status("placement_computed", next_state),
            ),
        )

    if state == OrchestratorState.PLAN_PICK and operation_name == "BuildMotionSequence":
        base_state = replace(model, state=OrchestratorState.EXEC_PICK)
        next_state, pick_effect = _begin_action(
            base_state, "PlanMotion", motion=True, segment="pick"
        )
        return Transition(
            next_state,
            (
                pick_effect,
                _publish_status("pick_plan_ready", next_state),
            ),
        )

    if state == OrchestratorState.EXEC_PICK and operation_name == "PlanMotion":
        next_state, vacuum_effect = _begin_service(
            model, "VacuumCommand", vacuum=True, enable=True
        )
        return Transition(
            next_state,
            (
                vacuum_effect,
                _publish_status("pick_motion_complete", next_state),
            ),
        )

    if state == OrchestratorState.EXEC_PICK and operation_name == "VacuumCommand":
        base_state = replace(
            model,
            state=OrchestratorState.PLAN_PLACE,
            carrying=True,
            vacuum_enabled=True,
        )
        next_state, plan_effect = _begin_service(base_state, "BuildMotionSequence")
        return Transition(
            next_state,
            (
                plan_effect,
                _publish_status("payload_attached", next_state),
            ),
        )

    if state == OrchestratorState.PLAN_PLACE and operation_name == "BuildMotionSequence":
        base_state = replace(model, state=OrchestratorState.EXEC_PLACE)
        next_state, place_effect = _begin_action(
            base_state, "PlanMotion", motion=True, segment="place"
        )
        return Transition(
            next_state,
            (
                place_effect,
                _publish_status("place_plan_ready", next_state),
            ),
        )

    if state == OrchestratorState.EXEC_PLACE and operation_name == "PlanMotion":
        next_state, release_effect = _begin_service(
            model, "VacuumCommand", vacuum=True, enable=False
        )
        return Transition(
            next_state,
            (
                release_effect,
                _publish_status("place_motion_complete", next_state),
            ),
        )

    if state == OrchestratorState.EXEC_PLACE and operation_name == "VacuumCommand":
        base_state = replace(
            model,
            state=OrchestratorState.COMMIT_AND_VERIFY,
            carrying=False,
            vacuum_enabled=False,
        )
        next_state, verify_effect = _begin_service(base_state, "VerifyPlacedBox")
        return Transition(
            next_state,
            (
                verify_effect,
                _publish_status("payload_released", next_state),
            ),
        )

    if (
        state == OrchestratorState.COMMIT_AND_VERIFY
        and operation_name == "VerifyPlacedBox"
    ):
        if not model.active_box_id:
            return _status(model, "box_identity_required")
        next_state, finalize_effect = _begin_service(
            model, "FinalizeCurrentBox", box_id=model.active_box_id
        )
        return Transition(
            next_state,
            (
                finalize_effect,
                _publish_status("placement_verified", next_state),
            ),
        )

    if (
        state == OrchestratorState.COMMIT_AND_VERIFY
        and operation_name == "FinalizeCurrentBox"
    ):
        box_id = model.active_box_id
        if not box_id:
            return _status(model, "box_identity_required")
        committed = model.committed_box_ids | frozenset({box_id})
        base_state = replace(model, committed_box_ids=committed, active_box_id=None)
        return _return_pick_observe(base_state, "placement_committed")

    if state == OrchestratorState.WAIT_RECOVERY and operation_name == "VacuumCommand":
        return _return_pick_observe(model, "recovery_release_complete")

    return _status(model, "success_ignored")


def _handle_failure(
    model: OrchestratorContractState, reason_code: str, event: OperatorEvent
) -> Transition:
    if not _operation_matches(model, event):
        return _status(model, "operation_stale_or_wrong_id")
    model = replace(model, pending_operation=None)

    if model.carrying or model.vacuum_enabled:
        next_state = replace(
            model,
            state=OrchestratorState.CARRY_FAULT,
            carrying=True,
            vacuum_enabled=True,
        )
        return Transition(
            next_state,
            (_publish_status(reason_code, next_state, boundary="carrying_fault"),),
        )

    if model.state == OrchestratorState.COMMIT_AND_VERIFY:
        next_state = replace(model, state=OrchestratorState.WAIT_RECOVERY)
        return Transition(
            next_state,
            (_publish_status(reason_code, next_state, boundary="verification"),),
        )

    if model.state == OrchestratorState.WAIT_PICKUP_READY:
        return _status(model, reason_code)

    if model.state in FAILURE_TRANSITIONS[FailureBoundary.PRE_PICK]["states"]:
        return _return_pick_observe(model, reason_code)

    return _status(model, reason_code)


def _handle_abort(
    model: OrchestratorContractState, reason_code: str
) -> Transition:
    if model.carrying or model.vacuum_enabled:
        next_state = replace(
            model,
            state=OrchestratorState.CARRY_FAULT,
            carrying=True,
            vacuum_enabled=True,
            pending_operation=None,
        )
        return Transition(
            next_state,
            (_publish_status(reason_code, next_state, boundary="abort_carrying"),),
        )
    next_state = replace(model, state=OrchestratorState.ABORTED, pending_operation=None)
    return Transition(
        next_state, (_publish_status(reason_code, next_state, boundary="abort"),)
    )


def _handle_recovery(
    model: OrchestratorContractState, event: OperatorEvent
) -> Transition:
    if model.state not in {
        OrchestratorState.CARRY_FAULT,
        OrchestratorState.WAIT_RECOVERY,
    }:
        return _status(model, "recovery_rejected")

    if model.state == OrchestratorState.CARRY_FAULT and not event.payload_released:
        return _status(model, "recovery_requires_explicit_release")

    base_state = replace(
        model,
        state=OrchestratorState.WAIT_RECOVERY,
        carrying=False,
        vacuum_enabled=False,
        pending_operation=None,
    )
    if model.vacuum_enabled:
        next_state, vacuum_effect = _begin_service(
            base_state, "VacuumCommand", vacuum=True, enable=False
        )
        return Transition(
            next_state,
            (
                vacuum_effect,
                _publish_status("recovery_confirmed", next_state, boundary="recovery"),
            ),
        )
    return _return_pick_observe(base_state, "recovery_confirmed")


def _return_pick_observe(
    model: OrchestratorContractState, reason_code: str
) -> Transition:
    base_state = replace(
        model,
        state=OrchestratorState.RETURN_PICK_OBSERVE,
        current_request_id=None,
        active_box_id=(
            model.active_box_id if model.carrying or model.vacuum_enabled else None
        ),
    )
    next_state, observe_effect = _begin_action(
        base_state, "GoToRobotPose", motion=True, pose_name="pick_observe_pose"
    )
    return Transition(
        next_state,
        (
            observe_effect,
            _publish_status(reason_code, next_state),
        ),
    )


def _enter_wait_pickup_ready(
    model: OrchestratorContractState, reason_code: str
) -> Transition:
    sequence = model.prompt_sequence + 1
    request_id = "%s:pickup-%06d" % (model.session_id, sequence)
    next_state = replace(
        model,
        state=OrchestratorState.WAIT_PICKUP_READY,
        current_request_id=request_id,
        prompt_sequence=sequence,
        carrying=False,
    )
    return Transition(
        next_state,
        (
            Effect(
                EffectType.PUBLISH_PROMPT,
                "OperatorPrompt",
                payload={
                    "schema_version": SCHEMA_VERSION,
                    "request_id": request_id,
                    "kind": "pickup_ready",
                    "state": next_state.state.value,
                    "reason_code": reason_code,
                },
            ),
            _publish_status(reason_code, next_state),
        ),
    )


def _status(
    model: OrchestratorContractState, reason_code: str
) -> Transition:
    return Transition(model, (_publish_status(reason_code, model),))


def _call_action(name: str, motion: bool = False, **payload: object) -> Effect:
    return Effect(EffectType.CALL_ACTION, name, motion=motion, payload=payload)


def _call_service(
    name: str,
    vacuum: bool = False,
    detect: bool = False,
    **payload: object,
) -> Effect:
    return Effect(
        EffectType.CALL_SERVICE,
        name,
        vacuum=vacuum,
        detect=detect,
        payload=payload,
    )


def _publish_status(
    reason_code: str,
    model: OrchestratorContractState,
    **payload: object,
) -> Effect:
    status = {
        "state": model.state.value,
        "reason_code": reason_code,
        "carrying": model.carrying,
        "vacuum_enabled": model.vacuum_enabled,
        "placed_count": model.placed_count,
    }
    status.update(payload)
    return Effect(EffectType.PUBLISH_STATUS, "LoadTaskStatus", payload=status)


def _begin_action(
    model: OrchestratorContractState,
    name: str,
    motion: bool = False,
    **payload: object,
) -> tuple[OrchestratorContractState, Effect]:
    return _begin_operation(model, EffectType.CALL_ACTION, name, motion=motion, **payload)


def _begin_service(
    model: OrchestratorContractState,
    name: str,
    vacuum: bool = False,
    detect: bool = False,
    **payload: object,
) -> tuple[OrchestratorContractState, Effect]:
    return _begin_operation(
        model, EffectType.CALL_SERVICE, name, vacuum=vacuum, detect=detect, **payload
    )


def _begin_operation(
    model: OrchestratorContractState,
    effect_type: EffectType,
    name: str,
    motion: bool = False,
    vacuum: bool = False,
    detect: bool = False,
    **payload: object,
) -> tuple[OrchestratorContractState, Effect]:
    if not model.session_id:
        raise ValueError("operation requires a started session")
    sequence = model.operation_sequence + 1
    operation_id = "%s:%s-%06d" % (model.session_id, name, sequence)
    operation = PendingOperation(operation_id=operation_id, name=name)
    next_state = replace(
        model, operation_sequence=sequence, pending_operation=operation
    )
    payload = dict(payload)
    payload["operation_id"] = operation_id
    return next_state, Effect(
        effect_type,
        name,
        motion=motion,
        vacuum=vacuum,
        detect=detect,
        payload=payload,
    )


def _operation_matches(
    model: OrchestratorContractState, event: OperatorEvent
) -> bool:
    return (
        model.pending_operation is not None
        and event.operation_id == model.pending_operation.operation_id
    )


def _valid_session_id(value: Optional[str]) -> Optional[str]:
    if value is None:
        return None
    session_id = value.strip()
    if not session_id:
        return None
    return session_id


def _valid_identity(value: Optional[str]) -> Optional[str]:
    if value is None:
        return None
    identity = value.strip()
    if not identity:
        return None
    return identity


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
    raise TypeError("unsupported plain value type: %s" % type(value).__name__)


def _plain_value(value: object) -> object:
    if isinstance(value, _FrozenMapping):
        return _plain_mapping(value)
    if isinstance(value, tuple):
        return [_plain_value(item) for item in value]
    return value


def _freeze_mapping(values: Mapping[str, object]) -> Tuple[Tuple[str, object], ...]:
    frozen = []
    for key, value in values.items():
        if not isinstance(key, str):
            raise TypeError("plain mapping keys must be strings")
        frozen.append((key, _freeze_plain(value)))
    return _FrozenMapping(sorted(frozen))


def _plain_mapping(values: Tuple[Tuple[str, object], ...]) -> dict[str, object]:
    return {key: _plain_value(value) for key, value in values}
