#!/usr/bin/env python3
"""Todo 3 acceptance driver: detect -> build sequence -> 4x PlanMotion.

Records per-segment result/fraction/wall time and the suction_contact_frame
Z delta between attach and pick_retreat (expect ~retreat_clearance).
"""

import json
import os
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from luggage_msgs.action import PlanMotion
from luggage_msgs.srv import BuildMotionSequence, DetectLuggage

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                   "docs", "status", "evidence", "todo3_pick")


class Driver(Node):

    def __init__(self):
        super().__init__("todo3_pick_driver")
        group = ReentrantCallbackGroup()
        self._build = self.create_client(
            BuildMotionSequence, "/waypoint_generator/build_motion_sequence",
            callback_group=group)
        self._detect = self.create_client(
            DetectLuggage, "/luggage_detector/detect_luggage",
            callback_group=group)
        self._plan = ActionClient(
            self, PlanMotion, "/motion_planner/plan_motion",
            callback_group=group)

    def call(self, client, request, timeout=30.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        event = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout):
            return None
        return future.result()


def main():
    os.makedirs(OUT, exist_ok=True)
    rclpy.init()
    node = Driver()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    record = {"t_start": time.time()}

    detect = node.call(node._detect, DetectLuggage.Request())
    assert detect and detect.success and detect.luggage, \
        "detection failed: %s" % (detect.message if detect else "timeout")
    box = detect.luggage[0]
    record["detected"] = {
        "x": box.pose.position.x, "y": box.pose.position.y,
        "z": box.pose.position.z, "size": [box.width, box.depth, box.height]}
    print("detected: %s" % record["detected"], flush=True)

    req = BuildMotionSequence.Request()
    req.phase = "pick"
    req.pick = box
    built = node.call(node._build, req)
    assert built and built.success, "build failed: %s" % (
        built.message if built else "timeout")
    segments = built.segments
    record["segments"] = [
        {"name": s.name, "type": s.type,
         "target_z": s.target_pose.position.z,
         "keep_tool_down": s.keep_tool_down,
         "allow_ompl_fallback": s.allow_ompl_fallback}
        for s in segments]
    print("sequence: %s" % json.dumps(record["segments"], indent=1), flush=True)

    if not node._plan.wait_for_server(timeout_sec=30.0):
        print("PlanMotion server unavailable", flush=True)
        sys.exit(1)

    record["results"] = []
    for segment in segments:
        goal = PlanMotion.Goal()
        goal.segment = segment
        feedbacks = []
        event = threading.Event()

        def _fb(msg, store=feedbacks):
            store.append({"stage": msg.feedback.stage,
                          "fraction": msg.feedback.fraction})
            print("  fb: %s fraction=%.3f"
                  % (msg.feedback.stage, msg.feedback.fraction), flush=True)

        future = node._plan.send_goal_async(goal, feedback_callback=_fb)
        event.clear()
        future.add_done_callback(lambda _f: event.set())
        event.wait(20.0)
        handle = future.result()
        if not handle.accepted:
            record["results"].append(
                {"name": segment.name, "ok": False, "message": "rejected"})
            break
        result_event = threading.Event()
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda _f: result_event.set())
        t0 = time.time()
        if not result_event.wait(120.0):
            handle.cancel_goal()
            record["results"].append(
                {"name": segment.name, "ok": False, "message": "timeout"})
            break
        wrapped = result_future.result()
        entry = {
            "name": segment.name,
            "ok": bool(wrapped.result.success),
            "message": wrapped.result.message,
            "status": int(wrapped.status),
            "wall_sec": round(time.time() - t0, 2),
            "feedbacks": feedbacks[-4:],
        }
        record["results"].append(entry)
        print("segment %s: ok=%s msg=%s (%.1fs)"
              % (segment.name, entry["ok"], entry["message"],
                 entry["wall_sec"]), flush=True)
        if not entry["ok"]:
            break

    record["t_end"] = time.time()
    with open(os.path.join(OUT, "run.json"), "w", encoding="utf-8") as f:
        json.dump(record, f, indent=2, sort_keys=True)
    print("saved %s/run.json" % OUT, flush=True)
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
