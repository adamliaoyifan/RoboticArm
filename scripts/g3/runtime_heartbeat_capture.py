#!/usr/bin/env python3
"""Independent bounded runtime-identity receive trace, no authorization/effects."""
import json
from collections import deque
from pathlib import Path
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from luggage_msgs.msg import RuntimeIdentityV2


def main():
    output = Path(sys.argv[1])
    output.mkdir(parents=True, exist_ok=True)
    rows = deque(maxlen=2048)  # about sixty seconds for the eight-peer graph
    stopped = False
    received = 0
    def stop(_signal, _frame):
        nonlocal stopped
        stopped = True
    rclpy.init()
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    node = Node("runtime_identity_capture")
    def record(msg):
        nonlocal received
        received += 1
        if received == 1:
            (output / "ready.json").write_text(json.dumps({"first_receive_monotonic": time.monotonic()}))
        rows.append(dict(peer=msg.peer, sequence=int(msg.heartbeat_sequence),
                         wall=time.time(), monotonic=time.monotonic(),
                         readiness=msg.readiness_code))
    node.create_subscription(RuntimeIdentityV2, "/frame_contracts/v2/runtime_identity", record,
                             QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
    started = time.monotonic()
    try:
        while not stopped and time.monotonic() - started < 5500:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        with (output / "receive_trace.jsonl").open("w") as stream:
            for row in rows:
                stream.write(json.dumps(row) + "\n")
        (output / "manifest.json").write_text(json.dumps(dict(
            received=received, retained=len(rows), overwritten=max(0, received-len(rows)),
            max_records=2048, first_monotonic=rows[0]["monotonic"] if rows else None,
            last_monotonic=rows[-1]["monotonic"] if rows else None,
            capture_complete=bool(rows), stop="requested" if stopped else "deadline"), indent=2)+"\n")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
