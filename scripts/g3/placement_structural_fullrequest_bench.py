#!/usr/bin/env python3
"""Full-request performance benchmark for the structural chain (G3 item 3).

User-authorized for THIS unit (2026-10-25 dispatch record): real ROS
service timing including serialization. Two scenarios per resolution:

- structural_hit: a normal load map; the structural batch answers.
- forced_fallback: every structural proposal is gate-illegal (unobserved
  0.30 m columns everywhere except one observed floor patch), so the
  session must fall back to the grid and still answer.

Formal sampling: 3 warmup + 30 measured service round trips; report
nearest-rank P95 (29/30), max, mean. Stage times (search core, atlas,
serialization) come from the node's own dump. No Gazebo, no MoveIt,
no route execution -- this is the placement service chain only.
"""

import argparse
import json
import os
import statistics
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_WS_SRC = os.path.normpath(os.path.join(_HERE, "..", ".."))
for _p in (("src", "luggage_packing"), ("src", "luggage_planning"),
           ("src", "luggage_description")):
    sys.path.insert(0, os.path.join(_WS_SRC, *_p))
sys.path.insert(0, _WS_SRC)
sys.path.insert(0, os.path.join(_WS_SRC, "src", "luggage_packing", "test"))

import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.qos import (  # noqa: E402
    DurabilityPolicy, QoSProfile, ReliabilityPolicy)
from std_msgs.msg import String  # noqa: E402

from luggage_msgs.msg import DetectedLuggage  # noqa: E402
from luggage_msgs.srv import ComputePlacement  # noqa: E402

from test_placement_hull_gates import BOX, SCENE_PATH  # noqa: E402
from placement_structural_eval import load_map  # noqa: E402

import test_placement_hull_gates as _hg  # noqa: E402


def forced_fallback_map(res):
    """Structural-empty / grid-legal map.

    Unknown 0.30 m columns everywhere except one central observed floor
    patch: boundary proposals land on unknown-above-floor (illegal),
    the single plateau's own edges are the map border (their offsets
    fall outside or onto unknown), and only grid windows fully inside
    the patch are legal.
    """
    surface, _placed = load_map(res, 0)
    nx, ny = surface["nx"], surface["ny"]
    for ix in range(nx):
        for iy in range(ny):
            surface["height"][ix][iy] = 0.30
            surface["state"][ix][iy] = "unknown"
    # Off-centre patch (0.25, -0.35) m: clear of every boundary source
    # line (x in {-0.525, 0, 0.525}; y in {-0.815, 0, 0.72, 0.955}) by
    # more than a half-window, so no structural proposal can land a
    # fully-inside window; only grid windows in the patch are legal.
    center_x, center_y = 0.25, -0.35
    half_l = 0.5 * float(surface["inner_size"][0])
    half_w = 0.5 * float(surface["inner_size"][1])
    cx = int((center_x + half_l) / res)
    cy = int((center_y + half_w) / res)
    sx = max(6, int(0.6 / res) // 2)
    sy = max(6, int(0.5 / res) // 2)
    for ix in range(cx - sx, cx + sx):
        for iy in range(cy - sy, cy + sy):
            surface["height"][ix][iy] = 0.0
            surface["state"][ix][iy] = "sensor"
    return surface


def run(resolutions, out_path, warmup=3, samples=30):
    from luggage_description._share import ENV_SCENE_TF
    module = _hg._load_node_module()

    results = {}
    os.environ[ENV_SCENE_TF] = SCENE_PATH
    rclpy.init()
    try:
        from rclpy.parameter import Parameter
        # Atlas stays on (production-equivalent): the benchmark covers
        # the full request incl. the atlas hook.
        planner = module.PlacementPlannerNode(parameter_overrides=[
            Parameter("structural_search", value=True),
        ])
        executor = SingleThreadedExecutor()
        executor.add_node(planner)
        qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        last = {}
        planner.create_subscription(
            String, "/placement_planner/last_result",
            lambda msg: last.update(payload=json.loads(msg.data)), qos)
        pub = planner.create_publisher(
            String, "/luggage/cargo_map/surface_2d", qos)
        client = planner.create_client(
            ComputePlacement, "/placement_planner/compute_placement")

        def spin_until(pred, timeout=60.0):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.001)
                if pred():
                    return True
            return False

        request = ComputePlacement.Request()
        box = DetectedLuggage()
        box.width, box.depth, box.height = BOX
        box.height_valid = True
        box.height_source = 1
        box.pose.orientation.w = 1.0
        request.box = box
        request.max_candidates = 5
        assert spin_until(client.service_is_ready, 10.0)

        for res in resolutions:
            for scenario, surface in (
                    ("structural_hit", load_map(res, 5)[0]),
                    ("forced_fallback", forced_fallback_map(res))):
                planner._surface = None
                last.clear()
                pub.publish(String(data=json.dumps(surface)))
                assert spin_until(
                    lambda: planner._surface is not None), "map not admitted"
                timings = []
                dump_stages = None
                fallback_used = None
                for _ in range(warmup + samples):
                    t0 = time.perf_counter()
                    future = client.call_async(request)
                    assert spin_until(future.done, 120.0)
                    dt = time.perf_counter() - t0
                    response = future.result()
                    assert response.success, response.message
                    timings.append(dt)
                spin_until(lambda: "payload" in last and last["payload"].get(
                    "structural"), 5.0)
                structural = (last.get("payload", {})
                              .get("structural") or {})
                fallback_used = structural.get("batch_source")
                dump_stages = structural.get("stages")
                measured = timings[warmup:]
                ordered = sorted(measured)
                results["res_%s/%s" % (res, scenario)] = {
                    "n": len(measured),
                    "min_s": round(min(measured), 3),
                    "median_s": round(statistics.median(measured), 3),
                    "p95_s": round(ordered[int(round(0.95 * len(measured)))
                                           - 1], 3),
                    "max_s": round(max(measured), 3),
                    "mean_s": round(statistics.mean(measured), 3),
                    "batch_source": fallback_used,
                    "node_stages": dump_stages,
                }
                print(results[list(results)[-1]])
        planner.destroy_node()
    finally:
        rclpy.shutdown()
    with open(out_path, "w") as handle:
        json.dump({
            "generated": "2026-10-25",
            "mode": "live PlacementPlannerNode (structural_search:=true), "
                    "real map transport + ComputePlacement round trip + "
                    "last_result serialization; 3 warmup + 30 measured",
            "results": results,
        }, handle, indent=1)
    print("saved", out_path)
    return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--resolutions", default="0.05,0.02,0.01")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    return run([float(v) for v in args.resolutions.split(",")], args.out)


if __name__ == "__main__":
    sys.exit(main())
