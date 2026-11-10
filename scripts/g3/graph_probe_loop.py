#!/usr/bin/env python3
"""Synchronous external graph probe for the scored runs (r8 diagnostics).

Run ALONGSIDE the driver window (not instead of it): samples the graph
view of a SEPARATE process every few seconds into a JSONL file, so a
driver-side discovery blind spot can be compared against a concurrent
outside view of the same moment. Recording only; never gates anything.
"""
import json
import sys
import time

import rclpy
from rclpy.node import Node

TARGETS = ("/vacuum_controller", "/place_prior", "/trajectory_selector",
           "/cargo_volume_mapper", "/pickup_box_spawner")


def main():
    out_path = sys.argv[1]
    interval = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
    rclpy.init()
    node = Node("graph_probe_external")
    with open(out_path, "a", encoding="utf-8") as handle:
        while True:
            try:
                names = {ns + n for n, ns in
                         node.get_node_names_and_namespaces()}
                row = {
                    "wall": time.time(),
                    "n_nodes": len(names),
                    "targets_seen": sorted(
                        t for t in TARGETS
                        if any(n.endswith(t) for n in names)),
                }
            except Exception as exc:  # noqa: BLE001 - probe only
                row = {"wall": time.time(), "error": repr(exc)[:160]}
            handle.write(json.dumps(row) + "\n")
            handle.flush()
            time.sleep(interval)


if __name__ == "__main__":
    main()
