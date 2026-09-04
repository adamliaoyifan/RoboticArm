#!/usr/bin/env python3
"""SIM-R1 Gate R1 contract tests."""

import dataclasses
import importlib
import unittest

from luggage_planning.exploration_contracts import (
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
        started = reduce_event(model, command("start"))
        self.assertEqual(started.state.state, OrchestratorState.RESET_CARGO_MAP)
        self.assertTrue(started.state.started)
        self.assertTrue(started.state.in_initial_exploration)
        self.assertFalse(has_motion(started.effects))
        self.assertFalse(has_vacuum(started.effects))
        duplicate = reduce_event(started.state, command("start"))
        self.assertEqual(duplicate.state.state, OrchestratorState.RESET_CARGO_MAP)
        self.assertFalse(has_motion(duplicate.effects))

    def test_pickup_ready_exact_request_id_authorizes_one_detection(self):
        model = initial_state()
        model = reduce_event(model, command("start")).state
        model = reduce_event(model, operation_succeeded()).state
        model = reduce_event(model, operation_succeeded()).state
        waiting = reduce_event(model, operation_succeeded())
        model = waiting.state
        self.assertEqual(model.state, OrchestratorState.WAIT_PICKUP_READY)
        request_id = model.current_request_id
        prompt = waiting.effects[0]
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
        self.assertFalse(has_motion(accepted.effects))

        duplicate = reduce_event(accepted.state, pickup_ready(request_id))
        self.assertEqual(duplicate.state.state, OrchestratorState.DETECT)
        self.assertFalse(has_detect(duplicate.effects))
        self.assertFalse(has_motion(duplicate.effects))

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
        model = initial_state()
        model = reduce_event(model, command("start")).state
        for _ in range(3):
            model = reduce_event(model, operation_succeeded()).state
        model = reduce_event(model, pickup_ready(model.current_request_id)).state
        for _ in range(4):
            model = reduce_event(model, operation_succeeded()).state
        self.assertEqual(model.state, OrchestratorState.PLAN_PLACE)
        self.assertTrue(model.carrying)
        self.assertTrue(model.vacuum_enabled)
        fault = reduce_event(model, operation_failed("place_plan_failed"))
        self.assertEqual(fault.state.state, OrchestratorState.CARRY_FAULT)
        self.assertTrue(fault.state.carrying)
        self.assertTrue(fault.state.vacuum_enabled)
        self.assertFalse(has_vacuum(fault.effects))

    def test_effect_payload_is_immutable_and_serializable(self):
        transition = reduce_event(initial_state(), command("start"))
        effect = transition.effects[0]
        with self.assertRaises(dataclasses.FrozenInstanceError):
            effect.motion = True
        with self.assertRaises(TypeError):
            effect.payload[0] = ("changed", True)
        self.assertIsInstance(effect.to_dict()["payload"], dict)


class TestSimR1ExplorationContracts(unittest.TestCase):
    def test_exploration_contracts_are_plain_and_immutable(self):
        context = ExplorationContext(
            frames={"world": "map"},
            geometry_descriptor={"schema_version": 1, "length": 1.0},
            geometry_hash="hash-a",
            camera_model={"fx": 500.0},
            policy_config={"policy": "heuristic"},
            budgets={"max_views": 3},
        )
        snapshot = ExplorationSnapshot(
            acquisition_stamp_start=1.0,
            acquisition_stamp_end=2.0,
            map_revision=7,
            occupancy_summary={"unknown_ratio": 0.4},
            visibility_summary={"frontier_count": 10},
            robot_state={"pose": "pick_observe"},
            visited_candidate_ids=["v1"],
        )
        candidate = CandidateView(
            candidate_id="v2",
            frame_id="container_link",
            position_xyz=(0.1, 0.2, 0.3),
            orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
            score=1.5,
        )
        proposal = PolicyProposal("heuristic", "sim-r1-view-v1", [candidate])
        outcome = ViewOutcome(
            ViewOutcomeStatus.INTEGRATED,
            "integrated",
            candidate_id="v2",
            acquisition_stamp=2.0,
            prior_map_revision=7,
            resulting_map_revision=8,
        )

        for obj in (context, snapshot, candidate, proposal, outcome):
            with self.subTest(obj=type(obj).__name__):
                self.assertIsInstance(obj.to_dict(), dict)
                with self.assertRaises(dataclasses.FrozenInstanceError):
                    obj.schema_version = 99

        self.assertEqual(context.to_dict()["geometry_hash"], "hash-a")
        self.assertEqual(snapshot.to_dict()["visited_candidate_ids"], ["v1"])
        self.assertEqual(proposal.to_dict()["candidates"][0]["candidate_id"], "v2")
        self.assertEqual(outcome.to_dict()["resulting_map_revision"], 8)

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
