#!/usr/bin/env python3
"""Spawn one pickup suitcase and wait. Do not grasp.

Exits 0 when the capture directory has both the first unbound anomaly and
a bound control. Exits 2 when the wait ends without that pair. One shot.
"""
import json
import os
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

from luggage_msgs.srv import SpawnNextBox


def _scan(root):
    found = {"anomaly": None, "control": None}
    base = Path(root)
    if not base.is_dir():
        return found
    for frame in base.rglob("frame.json"):
        try:
            meta = json.loads(frame.read_text())
        except (OSError, json.JSONDecodeError):
            continue
        role = meta.get("role")
        if role in found and found[role] is None:
            found[role] = str(frame.parent)
    return found


def main():
    out = Path(sys.argv[sys.argv.index("--out") + 1])
    timeout = 90.0
    if "--timeout-sec" in sys.argv:
        timeout = float(sys.argv[sys.argv.index("--timeout-sec") + 1])
    root = os.environ.get("ELFIN_AMBIGUITY_CAPTURE", str(out / "ambiguity"))
    rclpy.init()
    node = Node("pickup_observe_capture")
    client = node.create_client(
        SpawnNextBox, "/pickup_box_spawner/spawn_next_box")
    deadline = time.time() + 60.0
    while not client.wait_for_service(timeout_sec=1.0):
        if time.time() > deadline:
            _write(out, "spawn_service_missing", _scan(root))
            node.destroy_node()
            rclpy.shutdown()
            return 1
    future = client.call_async(SpawnNextBox.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
    response = future.result()
    if response is None or not response.success:
        message = "" if response is None else response.message
        _write(out, "spawn_failed", _scan(root), message=message)
        node.destroy_node()
        rclpy.shutdown()
        return 1
    node.get_logger().info("spawned %s" % response.message)
    end = time.time() + timeout
    found = _scan(root)
    while time.time() < end:
        found = _scan(root)
        if found["anomaly"] and found["control"]:
            _write(out, "reproduced", found, message=response.message)
            node.destroy_node()
            rclpy.shutdown()
            return 0
        time.sleep(1.0)
    status = "not_reproduced"
    _write(out, status, found, message=response.message)
    node.destroy_node()
    rclpy.shutdown()
    return 2


def _write(out, status, found, message=""):
    payload = {
        "status": status,
        "spawn_message": message,
        "anomaly": found.get("anomaly"),
        "control": found.get("control"),
        "grasp": False,
    }
    out.mkdir(parents=True, exist_ok=True)
    (out / "observe_result.json").write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n")
    print(json.dumps(payload))


if __name__ == "__main__":
    sys.exit(main())
