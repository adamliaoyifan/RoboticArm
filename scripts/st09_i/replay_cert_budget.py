#!/usr/bin/env python3
"""Replay a captured ValidateRouteV2 request with a wide-open budget.

Reads the ST09_I_DUMP_CERT json (scenes + legs as base64 CDR blobs),
rebuilds the service request with a diagnostic budget, and prints the
per-leg verdicts (reason, first invalid sample, contact bodies, wall
time) so budget/latency effects separate from real collision verdicts.
"""

import base64
import json
import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from rclpy.serialization import deserialize_message
from luggage_msgs.msg import RouteLegV2
from luggage_msgs.srv import ValidateRouteV2
from moveit_msgs.msg import PlanningScene


def main():
    dump_path = sys.argv[1] if len(sys.argv) > 1 else \
        "/tmp/st09_i_cert_dump.json"
    budget = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
    with open(dump_path, encoding="utf-8") as handle:
        dump = json.load(handle)
    rclpy.init()
    node = Node("replay_cert_budget")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    client = node.create_client(ValidateRouteV2, "/validate_route_v2")
    assert client.wait_for_service(timeout_sec=10.0), "service missing"
    request = ValidateRouteV2.Request()
    request.request_id = "replay-budget"
    request.route_digest = "replay"
    request.scene_digest = "replay"
    for blob in dump["scenes"]:
        request.load_state_scenes.append(deserialize_message(
            base64.b64decode(blob), PlanningScene))
    request.max_sample_dt_ns = 1.0e7
    request.max_sample_step_m = 0.0
    request.distance_stride = 64
    request.joint_velocity_limits = [1.57] * 6
    request.joint_acceleration_limits = []
    request.budget_s = budget
    for blob in dump["legs"]:
        request.legs.append(deserialize_message(
            base64.b64decode(blob), RouteLegV2))
    print("legs=%d scenes=%d budget=%.1fs" % (
        len(request.legs), len(request.load_state_scenes), budget))
    future = client.call_async(request)
    deadline = time.monotonic() + budget + 30.0
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.2)
    assert future.done(), "service timeout"
    response = future.result()
    print("result=%d detail=%s all_valid=%s service_ms=%.1f" % (
        response.result, response.detail, response.all_valid,
        response.service_time_ns / 1e6))
    for leg in response.leg_results:
        print("  %-28s n=%-4d valid=%-5s %-18s first=%-4d %5.1fms %s" % (
            leg.edge_id[:28], leg.n_samples, leg.valid, leg.reason or "-",
            leg.first_invalid_sample, leg.leg_time_ns / 1e6,
            (" ".join("%s/%s d=%.3f @%d" % (
                c.body_1[:16], c.body_2[:16], c.depth, c.sample_index)
                for c in leg.contacts[:4]) or "")))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
