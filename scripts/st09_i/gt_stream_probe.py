#!/usr/bin/env python3
"""Non-scored check of the M3 GT collector against one spawned model.

Spawns one box through the live spawner, then uses the installed
GtReleaseStream on that model id. Confirms a parsed sample is on disk.
Does not plan, grasp, or place.
"""
import argparse
import importlib.util
import json
import os
import sys
import time


def _load_stream_class(driver_path):
    spec = importlib.util.spec_from_file_location(
        "m3_three_box_driver_installed", driver_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.GtReleaseStream


def _spawn(expect, timeout_s):
    import rclpy
    from rclpy.node import Node
    from luggage_msgs.srv import SpawnNextBox

    rclpy.init()
    node = Node("gt_stream_probe")
    client = node.create_client(
        SpawnNextBox, "/pickup_box_spawner/spawn_next_box")
    deadline = time.monotonic() + float(timeout_s)
    while not client.wait_for_service(timeout_sec=1.0):
        if time.monotonic() > deadline:
            node.destroy_node()
            rclpy.shutdown()
            raise SystemExit("spawn service not ready")
    future = client.call_async(SpawnNextBox.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
    if not future.done() or future.result() is None:
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit("spawn service returned nothing")
    result = future.result()
    model = str(result.box.id or "")
    ok = bool(result.success) and model == expect
    node.destroy_node()
    rclpy.shutdown()
    if not ok:
        raise SystemExit(
            "spawn failed success=%s model=%r expect=%r message=%s"
            % (result.success, model, expect, result.message))
    return model


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True)
    parser.add_argument("--driver", required=True)
    parser.add_argument("--expect", default="pickup_box_0001_carryon")
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--fresh-wait", type=float, default=2.0)
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)
    model = _spawn(args.expect, args.timeout)
    jsonl = os.path.join(args.out, "release_gt.jsonl")
    stream_cls = _load_stream_class(args.driver)
    stream = stream_cls(model, jsonl)
    ready = stream.positive(timeout_s=args.fresh_wait)
    record = stream.failure_record()
    stream.stop()
    samples = []
    if os.path.isfile(jsonl):
        with open(jsonl, encoding="utf-8") as handle:
            samples = [json.loads(line) for line in handle if line.strip()]
    parsed = bool(samples) and len(samples[0].get("xyz") or []) == 3
    report = {
        "model": model,
        "path_ready": bool(ready and parsed),
        "sample_count": len(samples),
        "gap": "" if ready and parsed else record.get("gap"),
        "pose_blocks": record.get("pose_blocks"),
        "stdout_lines": record.get("stdout_lines"),
        "parse_failures": record.get("parse_failures"),
        "seen_names": record.get("seen_names"),
        "ign_returncode": record.get("ign_returncode"),
        "stderr_tail": record.get("stderr_tail"),
        "unparsed_text": record.get("unparsed_text"),
        "jsonl": jsonl,
    }
    with open(os.path.join(args.out, "gt_stream_probe.json"), "w",
              encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, sort_keys=True)
        handle.write("\n")
    print(json.dumps(report, sort_keys=True))
    return 0 if report["path_ready"] else 1


if __name__ == "__main__":
    sys.exit(main())
