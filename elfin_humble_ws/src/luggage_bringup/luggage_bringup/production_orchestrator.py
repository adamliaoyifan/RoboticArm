"""Thin ROS 2 adapter for the production orchestration contract."""

from __future__ import annotations

import re
import threading
from typing import Dict, Iterable, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from luggage_msgs.action import GoToRobotPose, PlanMotion, PlanNextCargoView
from luggage_msgs.msg import LoadTaskStatus, OperatorPrompt, PickupReady
from luggage_msgs.srv import (
    BuildMotionSequence,
    ComputePlacement,
    DetectLuggage,
    FinalizeCurrentBox,
    OrchestratorStep,
    ResetCargoMap,
    VacuumCommand,
    VerifyPlacedBox,
)
from luggage_planning.orchestration_contracts import (
    Effect,
    EffectType,
    EventType,
    OperatorEvent,
    OrchestratorState,
    SCHEMA_VERSION,
    command,
    operation_failed,
    operation_succeeded,
    pickup_ready,
)

from .production_adapter import ProductionAdapter


_SERVICE_TYPES = {
    "ResetCargoMap": ResetCargoMap,
    "DetectLuggage": DetectLuggage,
    "ComputePlacement": ComputePlacement,
    "BuildMotionSequence": BuildMotionSequence,
    "VacuumCommand": VacuumCommand,
    "VerifyPlacedBox": VerifyPlacedBox,
    "FinalizeCurrentBox": FinalizeCurrentBox,
}

_ACTION_TYPES = {
    "PlanNextCargoView": PlanNextCargoView,
    "GoToRobotPose": GoToRobotPose,
    "PlanMotion": PlanMotion,
}

_DEFAULT_ENDPOINTS = {
    "ResetCargoMap": "/cargo_map/reset",
    "PlanNextCargoView": "/cargo_exploration/plan_next_cargo_view",
    "GoToRobotPose": "/motion_planner/go_to_robot_pose",
    "DetectLuggage": "/luggage_detector/detect_luggage",
    "ComputePlacement": "/placement_planner/compute_placement",
    "BuildMotionSequence": "/waypoint_generator/build_motion_sequence",
    "PlanMotion": "/motion_planner/plan_motion",
    "VacuumCommand": "/vacuum/command",
    "VerifyPlacedBox": "/placed_pose_verifier/verify",
    "FinalizeCurrentBox": "/placement_commit/finalize",
}


def _parameter_key(name: str) -> str:
    return "endpoints." + re.sub(r"(?<!^)(?=[A-Z])", "_", name).lower()


def _reason_prefix(name: str) -> str:
    return re.sub(r"(?<!^)(?=[A-Z])", "_", name).lower()


class ProductionOrchestratorNode(Node):
    """Decode ROS events and dispatch reducer effects without policy logic."""

    def __init__(self, node_name: str = "orchestrator", **kwargs) -> None:
        super().__init__(node_name, **kwargs)
        self._group = ReentrantCallbackGroup()
        self._event_lock = threading.RLock()
        self._detected_box = None
        self._planned_slot = None
        self._placed_slots = []
        self._motion_segments: Dict[str, list] = {"pick": [], "place": []}
        self._operation_phase: Dict[str, str] = {}

        self.declare_parameter("exploration_mode", "heuristic")
        self.declare_parameter("geometry_hash", "")
        self.declare_parameter("map_revision", 0)
        for logical_name, endpoint in _DEFAULT_ENDPOINTS.items():
            self.declare_parameter(_parameter_key(logical_name), endpoint)

        prompt_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        pickup_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._prompt_pub = self.create_publisher(
            OperatorPrompt, "/luggage/operator/prompt", prompt_qos
        )
        self._status_pub = self.create_publisher(
            LoadTaskStatus, "/orchestrator/status", status_qos
        )
        self.create_subscription(
            PickupReady,
            "/luggage/operator/pickup_ready",
            self._on_pickup_ready,
            pickup_qos,
            callback_group=self._group,
        )
        self.create_service(
            OrchestratorStep,
            "/orchestrator/step",
            self._on_operator_command,
            callback_group=self._group,
        )

        self._service_clients = {
            name: self.create_client(
                service_type,
                self._endpoint(name),
                callback_group=self._group,
            )
            for name, service_type in _SERVICE_TYPES.items()
        }
        self._action_clients = {
            name: ActionClient(
                self,
                action_type,
                self._endpoint(name),
                callback_group=self._group,
            )
            for name, action_type in _ACTION_TYPES.items()
        }

        self._adapter = ProductionAdapter(self._dispatch_effect)
        self._apply_event(OperatorEvent(EventType.STARTUP))
        self.get_logger().info("production orchestrator ready in WAIT_START")

    @property
    def contract_state(self):
        return self._adapter.state

    def _endpoint(self, logical_name: str) -> str:
        return str(self.get_parameter(_parameter_key(logical_name)).value)

    def _apply_event(self, event: OperatorEvent):
        with self._event_lock:
            return self._adapter.accept(event)

    def _on_pickup_ready(self, message: PickupReady) -> None:
        self._apply_event(
            pickup_ready(
                message.request_id,
                operator_id=message.operator_id,
                schema_version=int(message.schema_version),
            )
        )

    def _on_operator_command(self, request, response):
        name = request.command.strip().lower()
        if name == "recovery_released":
            event = OperatorEvent(
                EventType.RECOVERY_CONFIRMED, payload_released=True
            )
        else:
            event = command(name)
        transition = self._apply_event(event)
        reason = self._last_reason(transition.effects)
        response.success = self._command_accepted(name, reason)
        response.message = reason
        response.paused_state = self.contract_state.state.value
        response.paused = self.contract_state.state in {
            OrchestratorState.WAIT_START,
            OrchestratorState.WAIT_PICKUP_READY,
            OrchestratorState.CARRY_FAULT,
            OrchestratorState.WAIT_RECOVERY,
            OrchestratorState.ABORTED,
        }
        response.placed_count = self.contract_state.placed_count
        response.run_mode = "event_driven"
        response.probe_touched = False
        response.active_breakpoints = []
        return response

    @staticmethod
    def _command_accepted(name: str, reason: str) -> bool:
        if name == "start":
            return reason == "start_accepted"
        if name in {"status", "pause", "abort", "recovery_released"}:
            return not (
                reason.endswith("_rejected")
                or reason.endswith("_required")
            )
        return False

    @staticmethod
    def _last_reason(effects: Iterable[Effect]) -> str:
        for effect in reversed(tuple(effects)):
            if effect.effect_type == EffectType.PUBLISH_STATUS:
                return str(effect.payload_dict().get("reason_code", "unknown"))
        return "status_unavailable"

    def _dispatch_effect(self, effect: Effect) -> None:
        if effect.effect_type == EffectType.PUBLISH_STATUS:
            self._publish_status(effect)
        elif effect.effect_type == EffectType.PUBLISH_PROMPT:
            self._publish_prompt(effect)
        elif effect.effect_type == EffectType.CALL_SERVICE:
            self._dispatch_service(effect)
        elif effect.effect_type == EffectType.CALL_ACTION:
            self._dispatch_action(effect)

    def _publish_status(self, effect: Effect) -> None:
        payload = effect.payload_dict()
        message = LoadTaskStatus()
        message.state = str(payload.get("state", self.contract_state.state.value))
        message.message = str(payload.get("reason_code", "unknown"))
        message.placed_count = int(payload.get("placed_count", 0))
        self._status_pub.publish(message)

    def _publish_prompt(self, effect: Effect) -> None:
        payload = effect.payload_dict()
        message = OperatorPrompt()
        message.schema_version = int(payload["schema_version"])
        message.header.stamp = self.get_clock().now().to_msg()
        message.request_id = str(payload["request_id"])
        message.kind = str(payload["kind"])
        message.state = str(payload["state"])
        message.display_text = (
            "Place one box at the pickup station, then confirm request %s."
            % message.request_id
        )
        self._prompt_pub.publish(message)

    def _dispatch_service(self, effect: Effect) -> None:
        name = effect.name
        payload = effect.payload_dict()
        operation_id = str(payload["operation_id"])
        client = self._service_clients.get(name)
        if client is None:
            self._complete_failure(operation_id, name, "unsupported")
            return
        if not client.service_is_ready():
            self._complete_failure(operation_id, name, "unavailable")
            return
        try:
            request = self._build_service_request(name, payload, operation_id)
            future = client.call_async(request)
            future.add_done_callback(
                lambda done, n=name, op=operation_id: self._service_done(n, op, done)
            )
        except Exception as exc:  # noqa: BLE001 - ROS adapter boundary
            self._complete_failure(operation_id, name, "dispatch_exception", exc)

    def _build_service_request(self, name: str, payload: dict, operation_id: str):
        request = _SERVICE_TYPES[name].Request()
        if name == "ComputePlacement":
            if self._detected_box is None:
                raise RuntimeError("detected box context missing")
            request.box = self._detected_box
            request.placed = list(self._placed_slots)
        elif name == "BuildMotionSequence":
            if self._detected_box is None or self._planned_slot is None:
                raise RuntimeError("motion context missing")
            phase = (
                "pick"
                if self.contract_state.state == OrchestratorState.PLAN_PICK
                else "place"
            )
            request.pick = self._detected_box
            request.place_slot = self._planned_slot
            request.phase = phase
            self._operation_phase[operation_id] = phase
        elif name == "VacuumCommand":
            request.enable = bool(payload.get("enable", False))
        elif name == "VerifyPlacedBox":
            if self._planned_slot is None:
                raise RuntimeError("planned slot context missing")
            request.planned = self._planned_slot
        return request

    def _service_done(self, name: str, operation_id: str, future) -> None:
        with self._event_lock:
            if not self._pending_matches(operation_id):
                return
            try:
                response = future.result()
            except Exception as exc:  # noqa: BLE001 - ROS future boundary
                self._complete_failure(operation_id, name, "response_exception", exc)
                return
            if response is None or not bool(getattr(response, "success", False)):
                self._complete_failure(operation_id, name, "failed")
                return

            box_id: Optional[str] = None
            if name == "DetectLuggage":
                detections = list(response.luggage)
                if len(detections) != 1:
                    self._complete_failure(operation_id, name, "count_invalid")
                    return
                self._detected_box = detections[0]
                box_id = str(detections[0].id)
            elif name == "ComputePlacement":
                self._planned_slot = response.slot
            elif name == "BuildMotionSequence":
                phase = self._operation_phase.pop(operation_id, "")
                if phase not in self._motion_segments or not response.segments:
                    self._complete_failure(operation_id, name, "segments_invalid")
                    return
                self._motion_segments[phase] = list(response.segments)
            elif name == "FinalizeCurrentBox":
                if self._planned_slot is not None:
                    self._placed_slots.append(self._planned_slot)
                self._detected_box = None
                self._planned_slot = None
                self._motion_segments = {"pick": [], "place": []}

            self._apply_event(operation_succeeded(operation_id, box_id=box_id))

    def _dispatch_action(self, effect: Effect) -> None:
        name = effect.name
        payload = effect.payload_dict()
        operation_id = str(payload["operation_id"])
        if name == "PlanMotion":
            phase = str(payload.get("segment", ""))
            segments = list(self._motion_segments.get(phase, []))
            if not segments:
                self._complete_failure(operation_id, name, "segments_missing")
                return
            self._send_motion_segment(operation_id, segments, 0)
            return
        try:
            goal = self._build_action_goal(name, payload)
        except Exception as exc:  # noqa: BLE001 - ROS adapter boundary
            self._complete_failure(operation_id, name, "goal_invalid", exc)
            return
        self._send_action_goal(name, operation_id, goal)

    def _build_action_goal(self, name: str, payload: dict):
        goal = _ACTION_TYPES[name].Goal()
        if name == "PlanNextCargoView":
            goal.schema_version = SCHEMA_VERSION
            goal.mode = str(self.get_parameter("exploration_mode").value)
            goal.session_id = str(self.contract_state.session_id or "")
            goal.geometry_hash = str(self.get_parameter("geometry_hash").value)
            goal.map_revision = int(self.get_parameter("map_revision").value)
            goal.views_used = 0
            goal.reset_session = True
            goal.preview_only = False
        elif name == "GoToRobotPose":
            goal.pose_name = str(payload.get("pose_name", "pick_observe_pose"))
        return goal

    def _send_action_goal(self, name: str, operation_id: str, goal) -> None:
        client = self._action_clients.get(name)
        if client is None:
            self._complete_failure(operation_id, name, "unsupported")
            return
        if not client.server_is_ready():
            self._complete_failure(operation_id, name, "unavailable")
            return
        try:
            future = client.send_goal_async(goal)
            future.add_done_callback(
                lambda done, n=name, op=operation_id: self._goal_response(n, op, done)
            )
        except Exception as exc:  # noqa: BLE001 - ROS action boundary
            self._complete_failure(operation_id, name, "dispatch_exception", exc)

    def _goal_response(self, name: str, operation_id: str, future) -> None:
        with self._event_lock:
            if not self._pending_matches(operation_id):
                return
            try:
                goal_handle = future.result()
            except Exception as exc:  # noqa: BLE001 - ROS future boundary
                self._complete_failure(operation_id, name, "goal_exception", exc)
                return
            if goal_handle is None or not goal_handle.accepted:
                self._complete_failure(operation_id, name, "goal_rejected")
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda done, n=name, op=operation_id: self._action_done(n, op, done)
            )

    def _action_done(self, name: str, operation_id: str, future) -> None:
        with self._event_lock:
            if not self._pending_matches(operation_id):
                return
            try:
                wrapped = future.result()
                result = wrapped.result
            except Exception as exc:  # noqa: BLE001 - ROS future boundary
                self._complete_failure(operation_id, name, "result_exception", exc)
                return
            if not bool(getattr(result, "success", False)):
                self._complete_failure(operation_id, name, "failed")
                return
            self._apply_event(operation_succeeded(operation_id))

    def _send_motion_segment(
        self, operation_id: str, segments: list, index: int
    ) -> None:
        name = "PlanMotion"
        client = self._action_clients[name]
        if not client.server_is_ready():
            self._complete_failure(operation_id, name, "unavailable")
            return
        goal = PlanMotion.Goal()
        goal.segment = segments[index]
        try:
            future = client.send_goal_async(goal)
            future.add_done_callback(
                lambda done, op=operation_id, seq=segments, i=index: (
                    self._motion_goal_response(op, seq, i, done)
                )
            )
        except Exception as exc:  # noqa: BLE001 - ROS action boundary
            self._complete_failure(operation_id, name, "dispatch_exception", exc)

    def _motion_goal_response(
        self, operation_id: str, segments: list, index: int, future
    ) -> None:
        with self._event_lock:
            if not self._pending_matches(operation_id):
                return
            try:
                goal_handle = future.result()
            except Exception as exc:  # noqa: BLE001 - ROS future boundary
                self._complete_failure(
                    operation_id, "PlanMotion", "goal_exception", exc
                )
                return
            if goal_handle is None or not goal_handle.accepted:
                self._complete_failure(operation_id, "PlanMotion", "goal_rejected")
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda done, op=operation_id, seq=segments, i=index: (
                    self._motion_segment_done(op, seq, i, done)
                )
            )

    def _motion_segment_done(
        self, operation_id: str, segments: list, index: int, future
    ) -> None:
        with self._event_lock:
            if not self._pending_matches(operation_id):
                return
            try:
                wrapped = future.result()
                result = wrapped.result
            except Exception as exc:  # noqa: BLE001 - ROS future boundary
                self._complete_failure(
                    operation_id, "PlanMotion", "result_exception", exc
                )
                return
            if not bool(getattr(result, "success", False)):
                self._complete_failure(operation_id, "PlanMotion", "failed")
                return
            next_index = index + 1
            if next_index < len(segments):
                self._send_motion_segment(operation_id, segments, next_index)
            else:
                self._apply_event(operation_succeeded(operation_id))

    def _pending_matches(self, operation_id: str) -> bool:
        pending = self.contract_state.pending_operation
        return pending is not None and pending.operation_id == operation_id

    def _complete_failure(
        self,
        operation_id: str,
        name: str,
        suffix: str,
        exception: Optional[BaseException] = None,
    ) -> None:
        reason = "%s_%s" % (_reason_prefix(name), suffix)
        if exception is not None:
            self.get_logger().error("%s: %s" % (reason, exception))
        self._apply_event(operation_failed(reason, operation_id))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ProductionOrchestratorNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
