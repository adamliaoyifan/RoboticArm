#!/usr/bin/env python3
"""SIM-R1 Gate R1 contract tests."""

import dataclasses
import importlib
import unittest

from luggage_planning.exploration_contracts import (
    AcquisitionStamp,
    CandidateView,
    ExplorationContext,
    ExplorationSnapshot,
    PolicyProposal,
    ViewOutcome,
    ViewOutcomeStatus,
)
from luggage_planning.orchestration_contracts import (
    EventType,
    FailureBoundary,
    OperatorEvent,
    OrchestratorState,
    command,
    has_detect,
    has_motion,
    has_vacuum,
    initial_state,
    operation_failed,
    operation_succeeded,
    pickup_ready,
    reduce_event,
)


def _pending_id(model):
    assert model.pending_operation is not None
    return model.pending_operation.operation_id


def _succeed(model, box_id=None):
    return reduce_event(
        model, operation_succeeded(_pending_id(model), box_id=box_id)
    )


def _fail(model, reason):
    return reduce_event(model, operation_failed(reason, _pending_id(model)))


def _advance_to_pickup_wait(session_id="session-a"):
    return _transition_to_pickup_wait(session_id).state


def _transition_to_pickup_wait(session_id="session-a"):
    model = reduce_event(initial_state(), command("start", session_id=session_id)).state
    transition = None
    while model.state != OrchestratorState.WAIT_PICKUP_READY:
        transition = _succeed(model)
        model = transition.state
    assert transition is not None
    return transition


def _advance_to_state(target_state, session_id="session-a", box_id="box-actual-1"):
    model = _advance_to_pickup_wait(session_id)
    model = reduce_event(model, pickup_ready(model.current_request_id)).state
    while model.state != target_state:
        model = _succeed(
            model, box_id=box_id if model.state == OrchestratorState.DETECT else None
        ).state
    return model


def _effect_payload(effect_name, effects):
    for effect in effects:
        if effect.name == effect_name:
            return effect.payload_dict()
    raise AssertionError("missing effect %s" % effect_name)


class TestSimR1OrchestrationContracts(unittest.TestCase):
    def test_wait_start_events_emit_no_motion_vacuum_or_detect(self):
        model = initial_state()
        self.assertEqual(model.state, OrchestratorState.WAIT_START)
        events = [
            EventType.STARTUP,
            EventType.SERVICE_READY,
            EventType.SENSOR_READY,
            EventType.TIMER,
        ]
        for event_type in events:
            with self.subTest(event_type=event_type):
                transition = reduce_event(model, pickup_ready("unused"))
                if event_type != EventType.PICKUP_READY:
                    from luggage_planning.orchestration_contracts import OperatorEvent

                    transition = reduce_event(model, OperatorEvent(event_type))
                self.assertIs(transition.state, model)
                self.assertFalse(has_motion(transition.effects))
                self.assertFalse(has_vacuum(transition.effects))
                self.assertFalse(has_detect(transition.effects))
        for name in ("run", "step"):
            transition = reduce_event(model, command(name))
            self.assertEqual(transition.state.state, OrchestratorState.WAIT_START)
            self.assertFalse(has_motion(transition.effects))
            self.assertFalse(has_vacuum(transition.effects))
            self.assertFalse(has_detect(transition.effects))

    def test_only_one_explicit_start_enters_initial_exploration(self):
        model = initial_state()
        started = reduce_event(model, command("start", session_id="session-a"))
        self.assertEqual(started.state.state, OrchestratorState.RESET_CARGO_MAP)
        self.assertTrue(started.state.started)
        self.assertTrue(started.state.in_initial_exploration)
        self.assertEqual(started.state.session_id, "session-a")
        self.assertIsNotNone(started.state.pending_operation)
        self.assertFalse(has_motion(started.effects))
        self.assertFalse(has_vacuum(started.effects))
        duplicate = reduce_event(started.state, command("start"))
        self.assertEqual(duplicate.state.state, OrchestratorState.RESET_CARGO_MAP)
        self.assertFalse(has_motion(duplicate.effects))

    def test_pickup_ready_exact_request_id_authorizes_one_detection(self):
        waiting = _transition_to_pickup_wait()
        model = waiting.state
        self.assertEqual(model.state, OrchestratorState.WAIT_PICKUP_READY)
        request_id = model.current_request_id
        prompt = next(effect for effect in waiting.effects if effect.name == "OperatorPrompt")
        prompt_payload = prompt.payload_dict()
        self.assertEqual(prompt_payload["request_id"], request_id)
        self.assertNotIn("geometry", prompt_payload)
        self.assertNotIn("pose", prompt_payload)
        self.assertNotIn("dimensions", prompt_payload)

        for bad in (None, "", "wrong-id"):
            blocked = reduce_event(model, pickup_ready(bad))
            self.assertEqual(blocked.state.state, OrchestratorState.WAIT_PICKUP_READY)
            self.assertFalse(has_detect(blocked.effects))
            self.assertFalse(has_motion(blocked.effects))

        accepted = reduce_event(model, pickup_ready(request_id, operator_id="operator-a"))
        self.assertEqual(accepted.state.state, OrchestratorState.DETECT)
        self.assertTrue(has_detect(accepted.effects))
        self.assertIsNotNone(accepted.state.pending_operation)
        self.assertFalse(has_motion(accepted.effects))

        duplicate = reduce_event(accepted.state, pickup_ready(request_id))
        self.assertEqual(duplicate.state.state, OrchestratorState.DETECT)
        self.assertFalse(has_detect(duplicate.effects))
        self.assertFalse(has_motion(duplicate.effects))

    def test_operation_completion_requires_exact_pending_id_and_commits_real_box_once(self):
        started = reduce_event(
            initial_state(), command("start", session_id="session-a")
        )
        reset_id = _pending_id(started.state)
        blocked = reduce_event(started.state, operation_succeeded("wrong-op"))
        self.assertEqual(blocked.state.state, OrchestratorState.RESET_CARGO_MAP)
        self.assertEqual(blocked.state.pending_operation.operation_id, reset_id)

        model = _succeed(started.state).state
        duplicate = reduce_event(model, operation_succeeded(reset_id))
        self.assertEqual(duplicate.state.state, OrchestratorState.EXPLORE_CONTAINER)
        self.assertEqual(duplicate.state.placed_count, 0)

        model = _advance_to_state(OrchestratorState.COMMIT_AND_VERIFY)
        verify_id = _pending_id(model)
        stale = reduce_event(model, operation_succeeded("session-a:FinalizeCurrentBox-999999"))
        self.assertEqual(stale.state.state, OrchestratorState.COMMIT_AND_VERIFY)
        self.assertEqual(stale.state.placed_count, 0)
        self.assertEqual(stale.state.pending_operation.operation_id, verify_id)

        verified = _succeed(model)
        finalize_payload = _effect_payload("FinalizeCurrentBox", verified.effects)
        self.assertEqual(finalize_payload["box_id"], "box-actual-1")
        final_id = _pending_id(verified.state)

        committed = _succeed(verified.state)
        self.assertIn("box-actual-1", committed.state.committed_box_ids)
        self.assertNotIn("box-000001", committed.state.committed_box_ids)
        self.assertEqual(committed.state.placed_count, 1)
        self.assertEqual(committed.state.active_box_id, None)

        duplicate_commit = reduce_event(committed.state, operation_succeeded(final_id))
        self.assertEqual(duplicate_commit.state.placed_count, 1)
        self.assertFalse(
            any(effect.name == "FinalizeCurrentBox" for effect in duplicate_commit.effects)
        )

    def test_out_of_order_completion_cannot_release_or_advance(self):
        model = _advance_to_state(OrchestratorState.EXEC_PLACE)
        place_motion_id = _pending_id(model)
        stale = reduce_event(model, operation_succeeded("session-a:old-op-000001"))
        self.assertEqual(stale.state.state, OrchestratorState.EXEC_PLACE)
        self.assertEqual(stale.state.pending_operation.operation_id, place_motion_id)
        self.assertFalse(has_vacuum(stale.effects))

        release = _succeed(model)
        self.assertEqual(release.state.state, OrchestratorState.EXEC_PLACE)
        self.assertTrue(has_vacuum(release.effects))
        self.assertEqual(
            _effect_payload("VacuumCommand", release.effects)["enable"], False
        )

    def test_pickup_request_ids_are_session_unique_and_cross_session_events_fail(self):
        session_a = _advance_to_pickup_wait("session-a")
        session_b = _advance_to_pickup_wait("session-b")
        self.assertEqual(session_a.current_request_id, "session-a:pickup-000001")
        self.assertEqual(session_b.current_request_id, "session-b:pickup-000001")
        self.assertNotEqual(session_a.current_request_id, session_b.current_request_id)

        blocked = reduce_event(session_b, pickup_ready(session_a.current_request_id))
        self.assertEqual(blocked.state.state, OrchestratorState.WAIT_PICKUP_READY)
        self.assertFalse(has_detect(blocked.effects))
        self.assertFalse(has_motion(blocked.effects))

    def test_wait_pickup_ready_after_failures_requires_correlated_pick_observe(self):
        detecting = _advance_to_state(OrchestratorState.DETECT)
        returned = _fail(detecting, "detect_failed")
        self.assertEqual(returned.state.state, OrchestratorState.RETURN_PICK_OBSERVE)
        self.assertEqual(returned.state.pending_operation.name, "GoToRobotPose")
        self.assertEqual(
            _effect_payload("GoToRobotPose", returned.effects)["pose_name"],
            "pick_observe_pose",
        )
        self.assertFalse(
            any(effect.name == "OperatorPrompt" for effect in returned.effects)
        )

        stale = reduce_event(returned.state, operation_succeeded("wrong-observe"))
        self.assertEqual(stale.state.state, OrchestratorState.RETURN_PICK_OBSERVE)
        self.assertFalse(any(effect.name == "OperatorPrompt" for effect in stale.effects))

        waiting = _succeed(returned.state)
        self.assertEqual(waiting.state.state, OrchestratorState.WAIT_PICKUP_READY)
        self.assertTrue(any(effect.name == "OperatorPrompt" for effect in waiting.effects))

    def test_recovery_paths_return_to_observe_before_prompting(self):
        carrying = _advance_to_state(OrchestratorState.PLAN_PLACE)
        fault = _fail(carrying, "place_plan_failed")
        self.assertEqual(fault.state.state, OrchestratorState.CARRY_FAULT)
        self.assertTrue(fault.state.vacuum_enabled)

        rejected = reduce_event(fault.state, OperatorEvent(EventType.RECOVERY_CONFIRMED))
        self.assertEqual(rejected.state.state, OrchestratorState.CARRY_FAULT)

        release = reduce_event(
            fault.state,
            OperatorEvent(EventType.RECOVERY_CONFIRMED, payload_released=True),
        )
        self.assertEqual(release.state.state, OrchestratorState.WAIT_RECOVERY)
        self.assertEqual(release.state.pending_operation.name, "VacuumCommand")
        observe = _succeed(release.state)
        self.assertEqual(observe.state.state, OrchestratorState.RETURN_PICK_OBSERVE)
        self.assertEqual(observe.state.pending_operation.name, "GoToRobotPose")
        self.assertFalse(any(effect.name == "OperatorPrompt" for effect in observe.effects))
        waiting = _succeed(observe.state)
        self.assertEqual(waiting.state.state, OrchestratorState.WAIT_PICKUP_READY)

    def test_failure_boundaries_cover_required_states_and_preserve_carrying_vacuum(self):
        self.assertEqual(
            set(FailureBoundary),
            {
                FailureBoundary.PRE_PICK,
                FailureBoundary.CARRYING,
                FailureBoundary.RELEASE,
                FailureBoundary.VERIFICATION,
                FailureBoundary.ABORT,
                FailureBoundary.RECOVERY,
            },
        )
        model = _advance_to_state(OrchestratorState.PLAN_PLACE)
        self.assertEqual(model.state, OrchestratorState.PLAN_PLACE)
        self.assertTrue(model.carrying)
        self.assertTrue(model.vacuum_enabled)
        fault = _fail(model, "place_plan_failed")
        self.assertEqual(fault.state.state, OrchestratorState.CARRY_FAULT)
        self.assertTrue(fault.state.carrying)
        self.assertTrue(fault.state.vacuum_enabled)
        self.assertFalse(has_vacuum(fault.effects))

    def test_effect_payload_is_immutable_and_serializable(self):
        transition = reduce_event(initial_state(), command("start"))
        effect = transition.effects[0]
        nested = {"outer": {"items": [1, {"state": "original"}]}}
        nested_effect = type(effect)(
            effect.effect_type, effect.name, payload=nested
        )
        nested["outer"]["items"][1]["state"] = "mutated"
        self.assertEqual(
            nested_effect.payload_dict()["outer"]["items"][1]["state"], "original"
        )
        exported = nested_effect.to_dict()
        exported["payload"]["outer"]["items"][1]["state"] = "changed"
        self.assertEqual(
            nested_effect.payload_dict()["outer"]["items"][1]["state"], "original"
        )
        with self.assertRaises(dataclasses.FrozenInstanceError):
            effect.motion = True
        with self.assertRaises(TypeError):
            effect.payload[0] = ("changed", True)
        self.assertIsInstance(effect.to_dict()["payload"], dict)


class TestSimR1ExplorationContracts(unittest.TestCase):
    def test_exploration_contracts_are_plain_and_immutable(self):
        nested_context = {"schema_version": 1, "bounds": {"xyz": [1, 2, 3]}}
        context = ExplorationContext(
            frames={"world": {"parent": "map", "chain": ["base"]}},
            geometry_descriptor=nested_context,
            geometry_hash="hash-a",
            camera_model={"fx": 500.0},
            policy_config={"policy": "heuristic"},
            budgets={"max_views": 3},
        )
        nested_context["bounds"]["xyz"][0] = 99
        snapshot = ExplorationSnapshot(
            acquisition_stamp_start=AcquisitionStamp(1, 2),
            acquisition_stamp_end={"sec": 1, "nanosec": 3},
            map_revision=7,
            occupancy_summary={"unknown_ratio": 0.4, "cells": [{"state": "unknown"}]},
            visibility_summary={"frontier_count": 10, "frontiers": [1, 2]},
            robot_state={"pose": {"name": "pick_observe"}},
            visited_candidate_ids=["v1"],
            diagnostics={"nested": {"ok": True}},
        )
        candidate = CandidateView(
            candidate_id="v2",
            frame_id="container_link",
            position_xyz=(0.1, 0.2, 0.3),
            orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
            score=1.5,
            diagnostics={"nested": {"score_terms": [1.0, 2.0]}},
        )
        proposal = PolicyProposal(
            "heuristic",
            "sim-r1-view-v1",
            [candidate],
            diagnostics={"nested": {"candidate_ids": ["v2"]}},
        )
        outcome = ViewOutcome(
            ViewOutcomeStatus.INTEGRATED,
            "integrated",
            candidate_id="v2",
            acquisition_stamp=(1, 3),
            prior_map_revision=7,
            resulting_map_revision=8,
            diagnostics={"nested": {"integrated": ["v2"]}},
        )

        for obj in (context, snapshot, candidate, proposal, outcome):
            with self.subTest(obj=type(obj).__name__):
                self.assertIsInstance(obj.to_dict(), dict)
                with self.assertRaises(dataclasses.FrozenInstanceError):
                    obj.schema_version = 99

        self.assertEqual(context.to_dict()["geometry_hash"], "hash-a")
        self.assertEqual(
            context.to_dict()["geometry_descriptor"]["bounds"]["xyz"][0], 1
        )
        for obj, path in (
            (context, ("frames", "world", "chain")),
            (snapshot, ("occupancy_summary", "cells")),
            (candidate, ("diagnostics", "nested", "score_terms")),
            (proposal, ("diagnostics", "nested", "candidate_ids")),
            (outcome, ("diagnostics", "nested", "integrated")),
        ):
            with self.subTest(defensive_copy=type(obj).__name__):
                exported = obj.to_dict()
                target = exported
                for key in path:
                    target = target[key]
                target[0] = "mutated"
                fresh = obj.to_dict()
                target = fresh
                for key in path:
                    target = target[key]
                self.assertNotEqual(target[0], "mutated")
        self.assertEqual(snapshot.to_dict()["visited_candidate_ids"], ["v1"])
        self.assertEqual(proposal.to_dict()["candidates"][0]["candidate_id"], "v2")
        self.assertEqual(outcome.to_dict()["resulting_map_revision"], 8)
        self.assertEqual(
            outcome.to_dict()["acquisition_stamp"], {"sec": 1, "nanosec": 3}
        )

    def test_contracts_reject_non_plain_data_and_float_stamps(self):
        with self.assertRaises(TypeError):
            ExplorationContext(
                frames={"bad": object()},
                geometry_descriptor={"schema_version": 1},
                geometry_hash="hash-a",
                camera_model={"fx": 500.0},
            )
        with self.assertRaises(TypeError):
            ExplorationSnapshot(
                acquisition_stamp_start=1.0,
                acquisition_stamp_end=AcquisitionStamp(1, 1),
                map_revision=1,
                occupancy_summary={},
                visibility_summary={},
                robot_state={},
            )
        with self.assertRaises(ValueError):
            CandidateView("v", "world", (0.0, 0.0, float("nan")), (0.0, 0.0, 0.0, 1.0))

    def test_acquisition_stamp_preserves_adjacent_nanoseconds(self):
        first = AcquisitionStamp(10, 42)
        second = AcquisitionStamp(10, 43)
        self.assertNotEqual(first, second)
        snapshot = ExplorationSnapshot(
            acquisition_stamp_start=first,
            acquisition_stamp_end=second,
            map_revision=1,
            occupancy_summary={},
            visibility_summary={},
            robot_state={},
        )
        self.assertEqual(
            snapshot.to_dict()["acquisition_stamp_start"], {"sec": 10, "nanosec": 42}
        )
        self.assertEqual(
            snapshot.to_dict()["acquisition_stamp_end"], {"sec": 10, "nanosec": 43}
        )

    def test_contract_modules_import_without_ros(self):
        for name in (
            "luggage_planning.exploration_contracts",
            "luggage_planning.orchestration_contracts",
        ):
            module = importlib.import_module(name)
            self.assertNotIn("rclpy", module.__dict__)
            self.assertNotIn("geometry_msgs", module.__dict__)


if __name__ == "__main__":
    unittest.main()
