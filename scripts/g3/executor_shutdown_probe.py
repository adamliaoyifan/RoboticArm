#!/usr/bin/env python3
"""Reproduce driver executor teardown without Gazebo or product inputs.

Compare the historical destroy-node-before-stop sequence with an ordered
executor shutdown. Exceptions are captured before interpreter termination.
"""
import argparse
import json
import threading
import time
import traceback

import rclpy
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


def probe(ordered):
    context = Context()
    rclpy.init(context=context)
    node = Node("executor_shutdown_probe", context=context)
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    executor.add_node(node)
    result = {"ordered": ordered, "exception": None, "callbacks": 0}

    def callback():
        result["callbacks"] += 1

    node.create_timer(0.002, callback)

    def spin():
        try:
            executor.spin()
        except BaseException as exc:
            result["exception"] = type(exc).__name__
            result["traceback"] = traceback.format_exc()

    thread = threading.Thread(target=spin)
    thread.start()
    time.sleep(0.1)
    result["alive_before_cleanup"] = thread.is_alive()
    if ordered:
        executor.shutdown(timeout_sec=5.0)
        thread.join(timeout=5.0)
        node.destroy_node()
        rclpy.shutdown(context=context)
    else:
        node.destroy_node()
        rclpy.shutdown(context=context)
        thread.join(timeout=5.0)
        executor.shutdown(timeout_sec=5.0)
    result["alive_after_cleanup"] = thread.is_alive()
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repetitions", type=int, default=10)
    args = parser.parse_args()
    for ordered in (False, True):
        for iteration in range(args.repetitions):
            print(json.dumps(dict(iteration=iteration, **probe(ordered))), flush=True)
