"""Pure production orchestrator state/effect contracts.

The reducer in this module models authorization boundaries only. It never calls
ROS, sleeps, reads configuration, looks up TF, or executes motion.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
from typing import FrozenSet, Iterable, Mapping, Optional, Tuple


SCHEMA_VERSION = 1


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
        object.__setattr__(
            self,
            "payload",
            tuple(sorted((str(key), value) for key, value in (self.payload or {}).items())),
        )

    def payload_dict(self) -> dict[str, object]:
        return dict(self.payload)

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
    schema_version: int = SCHEMA_VERSION
    payload_released: bool = False


@dataclass(frozen=True)
class OrchestratorContractState:
    state: OrchestratorState = OrchestratorState.WAIT_START
    started: bool = False
    carrying: bool = False
    vacuum_enabled: bool = False
    current_request_id: Optional[str] = None
    consumed_request_ids: FrozenSet[str] = frozenset()
    prompt_sequence: int = 0
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


def command(command_name: str) -> OperatorEvent:
    return OperatorEvent(EventType.COMMAND, command=command_name)


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


def operation_succeeded() -> OperatorEvent:
    return OperatorEvent(EventType.OPERATION_SUCCEEDED)


def operation_failed(reason_code: str) -> OperatorEvent:
    return OperatorEvent(EventType.OPERATION_FAILED, reason_code=reason_code)


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
        return _handle_success(model)

    if event.event_type == EventType.OPERATION_FAILED:
        return _handle_failure(model, event.reason_code or "operation_failed")

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
        next_state = replace(
            model, state=OrchestratorState.RESET_CARGO_MAP, started=True
        )
        return Transition(
            next_state,
            (
                _call_service("ResetCargoMap"),
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

    next_state = replace(
        model,
        state=OrchestratorState.DETECT,
        current_request_id=None,
        consumed_request_ids=model.consumed_request_ids | frozenset({request_id}),
    )
    return Transition(
        next_state,
        (
            _call_service("DetectLuggage", detect=True),
            _publish_status("pickup_ready_accepted", next_state),
        ),
    )


def _handle_success(model: OrchestratorContractState) -> Transition:
    state = model.state
    if state == OrchestratorState.RESET_CARGO_MAP:
        next_state = replace(model, state=OrchestratorState.EXPLORE_CONTAINER)
        return Transition(
            next_state,
            (
                _call_action("PlanNextCargoView"),
                _publish_status("cargo_map_reset", next_state),
            ),
        )

    if state == OrchestratorState.EXPLORE_CONTAINER:
        next_state = replace(model, state=OrchestratorState.RETURN_PICK_OBSERVE)
        return Transition(
            next_state,
            (
                _call_action("GoToRobotPose", motion=True, pose_name="pick_observe_pose"),
                _publish_status("exploration_complete", next_state),
            ),
        )

    if state == OrchestratorState.RETURN_PICK_OBSERVE:
        return _enter_wait_pickup_ready(model, "pickup_prompt")

    if state == OrchestratorState.DETECT:
        next_state = replace(model, state=OrchestratorState.COMPUTE_PLACEMENT)
        return Transition(
            next_state,
            (
                _call_service("ComputePlacement"),
                _publish_status("detection_complete", next_state),
            ),
        )

    if state == OrchestratorState.COMPUTE_PLACEMENT:
        next_state = replace(model, state=OrchestratorState.PLAN_PICK)
        return Transition(
            next_state,
            (
                _call_service("BuildMotionSequence"),
                _publish_status("placement_computed", next_state),
            ),
        )

    if state == OrchestratorState.PLAN_PICK:
        next_state = replace(model, state=OrchestratorState.EXEC_PICK)
        return Transition(
            next_state,
            (
                _call_action("PlanMotion", motion=True, segment="pick"),
                _publish_status("pick_plan_ready", next_state),
            ),
        )

    if state == OrchestratorState.EXEC_PICK:
        next_state = replace(
            model,
            state=OrchestratorState.PLAN_PLACE,
            carrying=True,
            vacuum_enabled=True,
        )
        return Transition(
            next_state,
            (
                _call_service("VacuumCommand", vacuum=True, enable=True),
                _call_service("BuildMotionSequence"),
                _publish_status("payload_attached", next_state),
            ),
        )

    if state == OrchestratorState.PLAN_PLACE:
        next_state = replace(model, state=OrchestratorState.EXEC_PLACE)
        return Transition(
            next_state,
            (
                _call_action("PlanMotion", motion=True, segment="place"),
                _publish_status("place_plan_ready", next_state),
            ),
        )

    if state == OrchestratorState.EXEC_PLACE:
        next_state = replace(
            model,
            state=OrchestratorState.COMMIT_AND_VERIFY,
            carrying=False,
            vacuum_enabled=False,
        )
        return Transition(
            next_state,
            (
                _call_service("VacuumCommand", vacuum=True, enable=False),
                _call_service("VerifyPlacedBox"),
                _publish_status("payload_released", next_state),
            ),
        )

    if state == OrchestratorState.COMMIT_AND_VERIFY:
        box_id = "box-%06d" % (model.placed_count + 1)
        next_state = replace(
            model,
            state=OrchestratorState.RETURN_PICK_OBSERVE,
            committed_box_ids=model.committed_box_ids | frozenset({box_id}),
        )
        return Transition(
            next_state,
            (
                _call_service("FinalizeCurrentBox"),
                _call_action("GoToRobotPose", motion=True, pose_name="pick_observe_pose"),
                _publish_status("placement_committed", next_state),
            ),
        )

    return _status(model, "success_ignored")


def _handle_failure(
    model: OrchestratorContractState, reason_code: str
) -> Transition:
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

    if model.state in FAILURE_TRANSITIONS[FailureBoundary.PRE_PICK]["states"]:
        return _enter_wait_pickup_ready(model, reason_code)

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
        )
        return Transition(
            next_state,
            (_publish_status(reason_code, next_state, boundary="abort_carrying"),),
        )
    next_state = replace(model, state=OrchestratorState.ABORTED)
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

    next_state = replace(
        model,
        state=OrchestratorState.RETURN_PICK_OBSERVE,
        carrying=False,
        vacuum_enabled=False,
    )
    return Transition(
        next_state,
        (
            _call_service("VacuumCommand", vacuum=True, enable=False),
            _publish_status("recovery_confirmed", next_state, boundary="recovery"),
        ),
    )


def _enter_wait_pickup_ready(
    model: OrchestratorContractState, reason_code: str
) -> Transition:
    sequence = model.prompt_sequence + 1
    request_id = "pickup-%06d" % sequence
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
