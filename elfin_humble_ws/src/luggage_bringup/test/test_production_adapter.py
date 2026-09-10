#!/usr/bin/env python3
"""Pure R5 tests for the thin production adapter."""

from luggage_bringup.production_adapter import ProductionAdapter
from luggage_planning.orchestration_contracts import (
    EffectType,
    EventType,
    OperatorEvent,
    OrchestratorState,
    command,
    operation_failed,
    operation_succeeded,
    pickup_ready,
)


def _pending_id(adapter):
    assert adapter.state.pending_operation is not None
    return adapter.state.pending_operation.operation_id


def test_readiness_and_timers_do_not_dispatch_work_before_start():
    effects = []
    adapter = ProductionAdapter(effects.append)
    for event_type in (
        EventType.STARTUP,
        EventType.SERVICE_READY,
        EventType.SENSOR_READY,
        EventType.TIMER,
    ):
        transition = adapter.accept(OperatorEvent(event_type))
        assert transition.state.state == OrchestratorState.WAIT_START
    assert not [
        effect
        for effect in effects
        if effect.effect_type in {EffectType.CALL_ACTION, EffectType.CALL_SERVICE}
    ]


def test_start_is_explicit_one_shot_and_initial_effects_are_declarative():
    effects = []
    adapter = ProductionAdapter(effects.append)
    started = adapter.accept(command("start", session_id="session-r5"))
    calls = [
        effect
        for effect in started.effects
        if effect.effect_type in {EffectType.CALL_ACTION, EffectType.CALL_SERVICE}
    ]
    assert [(effect.effect_type, effect.name) for effect in calls] == [
        (EffectType.CALL_SERVICE, "ResetCargoMap")
    ]
    assert calls[0].motion is False
    assert calls[0].vacuum is False

    duplicate = adapter.accept(command("start", session_id="other"))
    assert duplicate.state.state == OrchestratorState.RESET_CARGO_MAP
    assert not [
        effect
        for effect in duplicate.effects
        if effect.effect_type in {EffectType.CALL_ACTION, EffectType.CALL_SERVICE}
    ]

    reset_done = adapter.accept(operation_succeeded(_pending_id(adapter)))
    calls = [
        effect
        for effect in reset_done.effects
        if effect.effect_type in {EffectType.CALL_ACTION, EffectType.CALL_SERVICE}
    ]
    assert [(effect.effect_type, effect.name) for effect in calls] == [
        (EffectType.CALL_ACTION, "PlanNextCargoView")
    ]


def test_pickup_confirmation_is_exact_session_correlated_and_one_shot():
    effects = []
    adapter = ProductionAdapter(effects.append)
    adapter.accept(command("start", session_id="session-r5"))
    adapter.accept(operation_succeeded(_pending_id(adapter)))
    adapter.accept(operation_succeeded(_pending_id(adapter)))
    adapter.accept(operation_succeeded(_pending_id(adapter)))
    assert adapter.state.state == OrchestratorState.WAIT_PICKUP_READY
    request_id = adapter.state.current_request_id

    for invalid in (
        pickup_ready(None),
        pickup_ready(""),
        pickup_ready("wrong-session:pickup-000001"),
        pickup_ready(request_id, schema_version=999),
    ):
        transition = adapter.accept(invalid)
        assert transition.state.state == OrchestratorState.WAIT_PICKUP_READY
        assert not any(effect.detect or effect.motion for effect in transition.effects)

    accepted = adapter.accept(pickup_ready(request_id, operator_id="operator-r5"))
    assert accepted.state.state == OrchestratorState.DETECT
    assert [effect.name for effect in accepted.effects if effect.detect] == [
        "DetectLuggage"
    ]
    duplicate = adapter.accept(pickup_ready(request_id))
    assert duplicate.state.state == OrchestratorState.DETECT
    assert not any(effect.detect or effect.motion for effect in duplicate.effects)


def test_pre_pick_and_carry_failures_remain_observable_and_fail_safe():
    effects = []
    adapter = ProductionAdapter(effects.append)
    adapter.accept(command("start", session_id="session-r5"))
    adapter.accept(operation_succeeded(_pending_id(adapter)))
    adapter.accept(operation_succeeded(_pending_id(adapter)))
    adapter.accept(operation_succeeded(_pending_id(adapter)))
    adapter.accept(pickup_ready(adapter.state.current_request_id))

    pre_pick = adapter.accept(
        operation_failed("detect_failed_stable", _pending_id(adapter))
    )
    assert pre_pick.state.state == OrchestratorState.RETURN_PICK_OBSERVE
    assert [effect.name for effect in pre_pick.effects if effect.motion] == [
        "GoToRobotPose"
    ]
    assert any(
        effect.payload_dict().get("reason_code") == "detect_failed_stable"
        for effect in pre_pick.effects
    )
    non_moving_wait = adapter.accept(operation_succeeded(_pending_id(adapter)))
    assert non_moving_wait.state.state == OrchestratorState.WAIT_PICKUP_READY
    assert not any(
        effect.motion or effect.vacuum or effect.detect
        for effect in non_moving_wait.effects
    )

    # Build a fresh cycle through payload attachment.
    adapter = ProductionAdapter(effects.append)
    adapter.accept(command("start", session_id="session-carry"))
    for _ in range(3):
        adapter.accept(operation_succeeded(_pending_id(adapter)))
    adapter.accept(pickup_ready(adapter.state.current_request_id))
    adapter.accept(operation_succeeded(_pending_id(adapter), box_id="box-r5"))
    for _ in range(4):
        adapter.accept(operation_succeeded(_pending_id(adapter)))
    assert adapter.state.state == OrchestratorState.PLAN_PLACE
    assert adapter.state.vacuum_enabled is True
    fault = adapter.accept(
        operation_failed("place_plan_failed_stable", _pending_id(adapter))
    )
    assert fault.state.state == OrchestratorState.CARRY_FAULT
    assert fault.state.vacuum_enabled is True
    assert fault.state.placed_count == 0
    assert not any(effect.vacuum for effect in fault.effects)


def test_placement_commit_callback_is_idempotent():
    effects = []
    adapter = ProductionAdapter(effects.append)
    adapter.accept(command("start", session_id="session-commit"))
    for _ in range(3):
        adapter.accept(operation_succeeded(_pending_id(adapter)))
    adapter.accept(pickup_ready(adapter.state.current_request_id))
    adapter.accept(operation_succeeded(_pending_id(adapter), box_id="box-commit"))
    while adapter.state.state != OrchestratorState.COMMIT_AND_VERIFY:
        adapter.accept(operation_succeeded(_pending_id(adapter)))
    adapter.accept(operation_succeeded(_pending_id(adapter)))
    finalize_id = _pending_id(adapter)
    committed = adapter.accept(operation_succeeded(finalize_id))
    assert committed.state.placed_count == 1
    assert committed.state.committed_box_ids == frozenset({"box-commit"})

    duplicate = adapter.accept(operation_succeeded(finalize_id))
    assert duplicate.state.placed_count == 1
    assert not any(
        effect.effect_type in {EffectType.CALL_ACTION, EffectType.CALL_SERVICE}
        and effect.name == "FinalizeCurrentBox"
        for effect in duplicate.effects
    )
