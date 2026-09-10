#!/usr/bin/env python3
"""Visible one-shot operator controls for the production orchestrator."""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from luggage_msgs.msg import PickupReady
from luggage_msgs.srv import OrchestratorStep


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Explicit operator control for the luggage production cell"
    )
    commands = parser.add_subparsers(dest="operation", required=True)
    start = commands.add_parser("start", help="confirm and start one session")
    start.add_argument(
        "--confirm",
        default="",
        help="non-interactive confirmation; the only accepted value is START",
    )
    ready = commands.add_parser(
        "pickup-ready", help="confirm the currently displayed pickup request"
    )
    ready.add_argument("--request-id", required=True)
    ready.add_argument("--operator-id", default="")
    return parser


def _confirmation(value: str) -> bool:
    return value.strip() == "START"


class OperatorControl(Node):
    def __init__(self) -> None:
        super().__init__("operator_control")
        self._step = self.create_client(OrchestratorStep, "/orchestrator/step")
        pickup_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pickup = self.create_publisher(
            PickupReady, "/luggage/operator/pickup_ready", pickup_qos
        )

    def _call(self, command_name: str, timeout_sec: float = 10.0):
        if not self._step.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("orchestrator_control_unavailable")
        request = OrchestratorStep.Request()
        request.command = command_name
        future = self._step.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done() or future.result() is None:
            raise RuntimeError("orchestrator_control_timeout")
        return future.result()

    def show_status(self):
        status = self._call("status")
        print(
            "state=%s placed_count=%d reason=%s"
            % (status.paused_state, status.placed_count, status.message),
            flush=True,
        )
        return status

    def start(self, confirmation: str) -> bool:
        status = self.show_status()
        print(
            "Starting can authorize robot motion; current state is %s."
            % status.paused_state,
            flush=True,
        )
        value = confirmation
        if not value:
            if not sys.stdin.isatty():
                raise RuntimeError("start_confirmation_required")
            value = input("Type START to confirm: ")
        if not _confirmation(value):
            raise RuntimeError("start_confirmation_rejected")
        result = self._call("start")
        print(
            "accepted=%s state=%s reason=%s"
            % (result.success, result.paused_state, result.message),
            flush=True,
        )
        return bool(result.success)

    def pickup_ready(self, request_id: str, operator_id: str) -> None:
        if not request_id.strip():
            raise RuntimeError("pickup_request_id_required")
        message = PickupReady()
        message.schema_version = PickupReady.SCHEMA_VERSION
        message.header.stamp = self.get_clock().now().to_msg()
        message.request_id = request_id
        message.operator_id = operator_id
        deadline = time.monotonic() + 2.0
        while self.count_subscribers("/luggage/operator/pickup_ready") == 0:
            if time.monotonic() >= deadline:
                raise RuntimeError("orchestrator_pickup_subscription_unavailable")
            rclpy.spin_once(self, timeout_sec=0.05)
        self._pickup.publish(message)
        print("published pickup-ready request_id=%s" % request_id, flush=True)


def main(argv=None) -> int:
    raw_args = list(sys.argv if argv is None else argv)
    app_args = rclpy.utilities.remove_ros_args(args=raw_args)[1:]
    options = _parser().parse_args(app_args)
    rclpy.init(args=raw_args)
    node = OperatorControl()
    exit_code = 0
    try:
        if options.operation == "start":
            exit_code = 0 if node.start(options.confirm) else 2
        else:
            node.pickup_ready(options.request_id, options.operator_id)
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr, flush=True)
        exit_code = 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
