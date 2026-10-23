#!/usr/bin/env python3
"""No-motion rehearsal of measured GT evidence load alongside real heartbeats."""
import json
from pathlib import Path
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from luggage_planning.typed_driver_adapter import adopt_runtime_identity
from luggage_description.scene_tf_config_utils import load_scene_tf_config, xyz_world_to_base_link
from luggage_gazebo.place_gt_dump import build_place_gt, gt_place_dump, write_ply_xyzrgb


def main():
    output = Path(sys.argv[1])
    rclpy.init()
    node = Node("return_evidence_rehearsal")
    runtime, _registry = adopt_runtime_identity(node, peer="pack_eval_driver")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    rows = []
    try:
        deadline = time.monotonic() + 15
        while runtime.readiness() != "READY" and time.monotonic() < deadline:
            time.sleep(0.05)
        runtime.require_graph_ready()
        scene = load_scene_tf_config(str(output / "scene_tf_used.yaml"))
        world = [-2.0299998897896403, 0.5849961064052055, 0.6655245187812805]
        center = xyz_world_to_base_link(scene, world)
        for index in range(3):
            row = dict(iteration=index, before=runtime.snapshot(), start=time.monotonic())
            gt = build_place_gt(scene, center, [.50,.33,.22], 0.0,
                                box_center_world=world, box_rpy_world=[0,0,0])
            row["build_gt_end"] = time.monotonic()
            envelope = gt_place_dump(scene, scene_revision=runtime.identity["scene_revision"],
                                     request_id="rehearsal-%d" % index,
                                     box_center_base=center, box_size_wdh=[.50,.33,.22])
            (output / "rehearsal_gt.json").write_text(json.dumps(envelope))
            write_ply_xyzrgb(str(output / "rehearsal_gt.ply"), gt["cloud_xyz"], gt["cloud_rgb"])
            row.update(end=time.monotonic(), after=runtime.snapshot())
            rows.append(row)
            runtime.require_graph_ready(scope="unloaded_motion")
            time.sleep(0.5)
        # Verify the exact raw capture path before spending another box.
        from luggage_gazebo.release_capture import record
        from luggage_gazebo.pose_text import PoseTextDecoder
        raw_dir = output / "release_capture_preflight"
        capture = record(str(raw_dir), {
            "poses": ["ign", "topic", "-e", "-t", "/world/airport_loading/pose/info"],
            "contacts": ["ign", "topic", "-e", "-t", "/obs/box_contact"],
        }, duration=3.)
        decoder = PoseTextDecoder()
        for line in (raw_dir / "poses.raw").read_text().splitlines():
            decoder.feed_line(line)
        decoder.finish()
        health = {"pose_bytes": capture["streams"]["poses"]["bytes"],
                  "decoder": decoder.diagnostics()}
        (output / "release_capture_preflight.json").write_text(json.dumps(health, indent=2)+"\n")
        if not health["pose_bytes"] or not decoder.counts["accepted"]:
            raise RuntimeError("raw pose capture health failed")
        runtime.require_graph_ready(scope="unloaded_motion")
        (output / "return_rehearsal.json").write_text(json.dumps(dict(
            success=True, iterations=rows, no_motion=True), indent=2)+"\n")
    finally:
        (output / "return_rehearsal_last.json").write_text(json.dumps(dict(
            iterations=rows, runtime=runtime.snapshot()), indent=2)+"\n")
        executor.shutdown()
        thread.join(timeout=5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
