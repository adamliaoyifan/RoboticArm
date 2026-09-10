#!/usr/bin/env python3
"""Isolated ROS graph acceptance for the explicit Start boundary."""

import os
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")

from luggage_bringup.production_orchestrator import ProductionOrchestratorNode
from luggage_msgs.action import GoToRobotPose, PlanMotion, PlanNextCargoView
from luggage_msgs.srv import OrchestratorStep, ResetCargoMap, VacuumCommand
from rclpy.action import ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy


NO_START_SECONDS = 60.0


class R5GraphSpies(Node):
    def __init__(self):
        super().__init__("r5_graph_spies")
        self.reset_calls = 0
        self.vacuum_calls = 0
        self.exploration_goals = 0
        self.plan_motion_goals = 0
        self.named_pose_goals = 0
        self.release_exploration = threading.Event()
        self.exploration_finished = threading.Event()
        group = ReentrantCallbackGroup()
        self.create_service(
            ResetCargoMap, "/cargo_map/reset", self._reset, callback_group=group
        )
        self.create_service(
            VacuumCommand, "/vacuum/command", self._vacuum, callback_group=group
        )
        self._exploration = ActionServer(
            self,
            PlanNextCargoView,
            "/cargo_exploration/plan_next_cargo_view",
            execute_callback=self._execute_exploration,
            goal_callback=self._exploration_goal,
            callback_group=group,
        )
        self._plan_motion = ActionServer(
            self,
            PlanMotion,
            "/motion_planner/plan_motion",
            execute_callback=self._execute_plan_motion,
            goal_callback=self._plan_motion_goal,
            callback_group=group,
        )
        self._named_pose = ActionServer(
            self,
            GoToRobotPose,
            "/motion_planner/go_to_robot_pose",
            execute_callback=self._execute_named_pose,
            goal_callback=self._named_pose_goal,
            callback_group=group,
        )

    def _reset(self, _request, response):
        self.reset_calls += 1
        response.success = True
        response.message = "ok"
        return response

    def _vacuum(self, _request, response):
        self.vacuum_calls += 1
        response.success = True
        response.message = "ok"
        return response

    def _exploration_goal(self, _request):
        self.exploration_goals += 1
        return GoalResponse.ACCEPT

    def _plan_motion_goal(self, _request):
        self.plan_motion_goals += 1
        return GoalResponse.ACCEPT

    def _named_pose_goal(self, _request):
        self.named_pose_goals += 1
        return GoalResponse.ACCEPT

    def _execute_exploration(self, goal_handle):
        self.release_exploration.wait(timeout=10.0)
        goal_handle.abort()
        result = PlanNextCargoView.Result()
        result.success = False
        result.reason_code = "test_release"
        self.exploration_finished.set()
        return result

    @staticmethod
    def _execute_plan_motion(goal_handle):
        goal_handle.succeed()
        result = PlanMotion.Result()
        result.success = True
        return result

    @staticmethod
    def _execute_named_pose(goal_handle):
        goal_handle.succeed()
        result = GoToRobotPose.Result()
        result.success = True
        return result


def _spin_until(executor, predicate, timeout_sec):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=min(0.05, deadline - time.monotonic()))
        if predicate():
            return True
    return bool(predicate())


def _call(executor, client, command_name):
    request = OrchestratorStep.Request()
    request.command = command_name
    future = client.call_async(request)
    assert _spin_until(executor, future.done, 5.0)
    return future.result()


def test_sixty_second_no_start_graph_and_explicit_start_sequence():
    os.environ["ROS_DOMAIN_ID"] = str(120 + (os.getpid() % 80))
    log_dir = "/tmp/luggage_bringup_r5_%d" % os.getpid()
    os.makedirs(log_dir, exist_ok=True)
    os.environ["ROS_LOG_DIR"] = log_dir
    rclpy.init()
    orchestrator = ProductionOrchestratorNode()
    spies = R5GraphSpies()
    operator = Node("r5_operator_probe")
    step = operator.create_client(OrchestratorStep, "/orchestrator/step")
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (orchestrator, spies, operator):
        executor.add_node(node)
    try:
        assert _spin_until(
            executor,
            lambda: step.service_is_ready()
            and orchestrator._service_clients["ResetCargoMap"].service_is_ready()
            and orchestrator._action_clients["PlanNextCargoView"].server_is_ready(),
            5.0,
        )

        start = time.monotonic()
        assert _spin_until(executor, lambda: False, NO_START_SECONDS) is False
        assert time.monotonic() - start >= NO_START_SECONDS
        assert spies.reset_calls == 0
        assert spies.exploration_goals == 0
        assert spies.plan_motion_goals == 0
        assert spies.named_pose_goals == 0
        assert spies.vacuum_calls == 0

        prompt_info = spies.get_publishers_info_by_topic(
            "/luggage/operator/prompt"
        )
        status_info = spies.get_publishers_info_by_topic("/orchestrator/status")
        pickup_info = spies.get_subscriptions_info_by_topic(
            "/luggage/operator/pickup_ready"
        )
        assert len(prompt_info) == len(status_info) == len(pickup_info) == 1
        assert prompt_info[0].qos_profile.reliability == ReliabilityPolicy.RELIABLE
        assert prompt_info[0].qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
        assert status_info[0].qos_profile.reliability == ReliabilityPolicy.RELIABLE
        assert status_info[0].qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
        assert pickup_info[0].qos_profile.reliability == ReliabilityPolicy.RELIABLE
        assert pickup_info[0].qos_profile.durability == DurabilityPolicy.VOLATILE

        accepted = _call(executor, step, "start")
        assert accepted.success is True
        assert accepted.message == "start_accepted"
        assert _spin_until(executor, lambda: spies.exploration_goals == 1, 5.0)
        assert spies.reset_calls == 1
        assert spies.plan_motion_goals == 0
        assert spies.named_pose_goals == 0
        assert spies.vacuum_calls == 0

        duplicate = _call(executor, step, "start")
        assert duplicate.success is False
        assert duplicate.message == "duplicate_start_rejected"
        assert spies.reset_calls == 1
        assert spies.exploration_goals == 1
    finally:
        spies.release_exploration.set()
        _spin_until(
            executor,
            lambda: spies.exploration_finished.is_set()
            and orchestrator.contract_state.pending_operation is None,
            3.0,
        )
        executor.shutdown(timeout_sec=3.0)
        for node in (operator, spies, orchestrator):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
