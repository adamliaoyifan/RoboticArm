#!/usr/bin/env python3
"""No-motion admission/lease diagnostic probe (G3 ADMISSION-DIAG).

Subscribes the production v2 identity heartbeat topic and records, per
peer message: arrival monotonic+wall time, sequence, readiness_code,
and the inter-arrival gap (the receiver-side lease view: lease is 2.0 s
on the RECEIVER's monotonic wall clock). Gaps > lease are flagged.

Also runs a paced /cargo_map/get_stats client and records per-call
round-trip, so stats occupancy can be correlated with heartbeat gaps of
the same process. No motion, no writes to any production service.

Usage: admission_lease_probe.py <out_jsonl> [--duration 120]
       [--stats-period 2.0] [--stats-phase initial|paced]
"""

import argparse
import json
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from luggage_msgs.msg import RuntimeIdentityV2
from luggage_msgs.srv import GetCargoMapStats

LEASE_SECONDS = 2.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("out")
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--stats-period", type=float, default=2.0,
                        help="0 disables the stats probe loop")
    args = parser.parse_args()

    rclpy.init()
    node = Node("admission_lease_probe")
    records = []
    lock = threading.Lock()
    last_by_peer = {}

    def on_identity(msg):
        now_mono = time.monotonic()
        now_wall = time.time()
        peer = str(msg.peer)
        with lock:
            prior = last_by_peer.get(peer)
            gap = (now_mono - prior) if prior is not None else None
            last_by_peer[peer] = now_mono
            records.append({
                "kind": "heartbeat",
                "peer": peer,
                "arrival_monotonic": now_mono,
                "arrival_wall": now_wall,
                "sequence": int(msg.heartbeat_sequence),
                "readiness_code": str(msg.readiness_code),
                "gap_s": round(gap, 4) if gap is not None else None,
                "over_lease": bool(gap is not None and gap > LEASE_SECONDS),
            })

    qos = QoSProfile(
        depth=200, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE)
    node.create_subscription(
        RuntimeIdentityV2, "/frame_contracts/v2/runtime_identity",
        on_identity, qos)

    stats_records = []

    def stats_loop():
        client = node.create_client(
            GetCargoMapStats, "/cargo_map/get_stats")
        while not client.wait_for_service(timeout_sec=5.0):
            pass
        while rclpy.ok():
            t0 = time.monotonic()
            future = client.call_async(GetCargoMapStats.Request())
            deadline = t0 + 30.0
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.01)
            dt = time.monotonic() - t0
            ok = future.done() and bool(future.result().success)
            with lock:
                stats_records.append({
                    "request_monotonic": t0,
                    "wall": time.time(),
                    "round_trip_s": round(dt, 4),
                    "ok": ok,
                    "timed_out": not future.done(),
                })
            time.sleep(max(0.0, args.stats_period - dt))

    stats_thread = None
    if args.stats_period > 0:
        stats_thread = threading.Thread(target=stats_loop, daemon=True)
        stats_thread.start()

    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)

    with lock:
        with open(args.out, "w") as handle:
            for record in records:
                handle.write(json.dumps(record) + "\n")
            for record in stats_records:
                handle.write(json.dumps(
                    dict(record, kind="get_stats")) + "\n")

    peers = sorted(last_by_peer)
    print(json.dumps({"peers": peers, "heartbeats": len(records),
                      "stats_calls": len(stats_records)}))
    for peer in peers:
        gaps = [r["gap_s"] for r in records
                if r["kind"] == "heartbeat" and r["peer"] == peer
                and r["gap_s"] is not None]
        if gaps:
            print(json.dumps({
                "peer": peer,
                "n": len(gaps),
                "max_gap_s": round(max(gaps), 3),
                "gaps_over_lease": sum(1 for g in gaps if g > LEASE_SECONDS),
            }))
    if stats_records:
        rts = [r["round_trip_s"] for r in stats_records]
        print(json.dumps({"get_stats_round_trip_s": {
            "n": len(rts), "min": round(min(rts), 3),
            "median": round(sorted(rts)[len(rts) // 2], 3),
            "max": round(max(rts), 3)}}))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
