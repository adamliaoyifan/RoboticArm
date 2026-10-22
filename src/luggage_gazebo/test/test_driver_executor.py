"""Real ROS executor tests, without a simulator or product services."""
import json
from pathlib import Path
import tempfile
import time
import unittest

import rclpy
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from luggage_gazebo.driver_executor import DriverExecutor


class TestDriverExecutor(unittest.TestCase):
    def exercise(self, fail):
        context = Context()
        rclpy.init(context=context)
        node = Node("test_driver_executor", context=context)
        executor = MultiThreadedExecutor(num_threads=4, context=context)
        executor.add_node(node)
        calls = []

        def callback():
            calls.append(1)
            if fail:
                raise ValueError("known callback failure for lifecycle test")

        node.create_timer(0.01, callback)
        with tempfile.TemporaryDirectory() as out:
            owner = DriverExecutor(executor, node, out)
            owner.start()
            try:
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline:
                    if owner.failure if fail else len(calls) >= 3:
                        break
                    time.sleep(0.01)
            finally:
                owner.close()
                rclpy.shutdown(context=context)
            self.assertFalse(owner.thread.is_alive())
            rows = [json.loads(line) for line in
                    (Path(out) / "executor_lifecycle.jsonl").read_text().splitlines()]
            self.assertEqual(rows[-1]["event"], "nodes_destroyed")
            self.assertLess(next(i for i, row in enumerate(rows)
                                 if row["event"] == "spin_return"), len(rows)-1)
            if fail:
                self.assertEqual(owner.failure, "ValueError")
                failure = next(row for row in rows if row["event"] == "exception")
                self.assertEqual(failure["phase"], "running")
                self.assertIn("callback", (Path(out) / "executor_crash.txt").read_text())
            else:
                self.assertGreaterEqual(len(calls), 3)
                self.assertIsNone(owner.failure)
                self.assertFalse((Path(out) / "executor_crash.txt").exists())

    def test_normal_shutdown_after_live_callbacks(self):
        self.exercise(False)

    def test_callback_failure_is_durable_and_fatal(self):
        self.exercise(True)
