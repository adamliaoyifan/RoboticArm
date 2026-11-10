#!/usr/bin/env python3
"""Non-scored preflight: spawn the frozen dynamic box and exit.

Used by preflight_contact_bridge.sh to prove the contact chain
(sensor -> gz topic -> parameter_bridge -> ROS -> recorder JSONL)
carries real content. No arm replay, no metrics, no slot in any batch.
"""
import json
import sys

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity

# Frozen batch-1 input (AB_RUN_ORDER.json frozen_input): the box's
# settle pose inside the container cavity.
POSE = {
    "x": -1.98001, "y": 0.460003, "z": 0.691353,
    "qw": 0.706982, "qx": -0.0136845, "qy": 0.0119861, "qz": 0.707117,
}


def main():
    if len(sys.argv) != 3:
        raise SystemExit("usage: preflight_spawn_box.py <release.sdf> "
                         "<out.json>")
    sdf_path, out_json = sys.argv[1:3]
    rclpy.init()
    node = Node("ab_preflight_spawn")
    cli = node.create_client(
        SpawnEntity, "/world/airport_loading/create")
    if not cli.wait_for_service(timeout_sec=90.0):
        json.dump({"spawned": False, "error": "create service missing"},
                  open(out_json, "w"), indent=2)
        return 2
    req = SpawnEntity.Request()
    req.entity_factory.name = "pickup_box_0001_inch20"
    req.entity_factory.sdf = open(sdf_path).read()
    f = req.entity_factory.pose
    f.position.x, f.position.y, f.position.z = (
        POSE["x"], POSE["y"], POSE["z"])
    f.orientation.w = POSE["qw"]
    f.orientation.x = POSE["qx"]
    f.orientation.y = POSE["qy"]
    f.orientation.z = POSE["qz"]
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
    res = future.result()
    ok = bool(res and getattr(res, "success", False))
    json.dump({"spawned": ok,
               "detail": str(getattr(res, "status_message", ""))},
              open(out_json, "w"), indent=2)
    print(json.dumps({"spawned": ok}))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 3


if __name__ == "__main__":
    sys.exit(main())
