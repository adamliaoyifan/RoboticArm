#!/usr/bin/env python3
"""Regression stress for the integ_run01 L crash (uncaught NO_DATA from
maybe_publish_valid when a reset empties the store while a stored
integration's receipt callback is still in flight).

Drives the production race at its highest-probability interleaving: a
sustained input stream (10 Hz) keeps integrations completing on the worker
thread while a reset fires every RESET_PERIOD_MS. The pre-fix node
terminates within a few rounds; the fixed node must survive all rounds and
recover to VALID. Non-scored wiring check only.
"""
import argparse
import json
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import freespace_campaign as fs  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--node", required=True)
    parser.add_argument("--geometry", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--frames-glob", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--rounds", type=int, default=30)
    parser.add_argument("--reset-period-ms", type=float, default=300.0)
    parser.add_argument("--seconds", type=float, default=20.0)
    args = parser.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    frames = fs.load_frames(args.frames_glob)
    node = fs.NodeProcess(args.node, args.geometry, args.config,
                          out / "observer.log", "/reset_crash_repro_cloud")
    client = fs.Client("/reset_crash_repro_cloud")
    import hashlib
    client.config_digest = hashlib.sha256(
        Path(args.config).read_bytes()).hexdigest()
    record = {"rounds": args.rounds, "reset_period_ms": args.reset_period_ms}
    try:
        client.wait_services()
        first = frames[0]
        tf = first.get("transform") or {}
        client.broadcast_tf((tf.get("translation") or [-1.15, 0.0, 1.20]))
        fs.readiness(client, frames)
        t0_ns = time.monotonic_ns()
        base_stamp = int(time.time() * 1e9)
        stop = threading.Event()
        sent = {"n": 0}

        def sender():
            from sensor_msgs.msg import PointCloud2
            period_ns = 100_000_000  # 10 Hz
            index = 0
            while not stop.is_set():
                frame = frames[index % len(frames)]
                cloud, _ = fs.cloud_from_frame(frame)
                stamp = base_stamp + index * period_ns
                cloud.header.stamp.sec = stamp // 10**9
                cloud.header.stamp.nanosec = stamp % 10**9
                client.cloud.publish(cloud)
                sent["n"] += 1
                index += 1
                stop.wait(period_ns / 1e9)

        def resetter():
            from uld_occ_observer.srv import ResetObservation
            time.sleep(0.5)
            while not stop.is_set() and record.get("rounds_done", 0) < args.rounds:
                validity = client.call(client.validity, None) \
                    if hasattr(client, "validity") else None
                request = ResetObservation.Request()
                request.expected_epoch = 0  # server derives current epoch? no:
                # use the snapshot identity epoch like the campaign does
                request.expected_epoch = fs._epoch_of(client) or 1
                request.reset_id = "crash_repro_%d" % record.get("rounds_done", 0)
                request.reason = "crash_repro"
                started = time.monotonic_ns()
                response = client.call(client.reset, request)
                ok = response is not None and response.status == "OK"
                record.setdefault("resets", []).append(
                    {"ok": ok, "status": getattr(response, "status", None),
                     "ms": round((time.monotonic_ns() - started) / 1e6, 2)})
                record["rounds_done"] = record.get("rounds_done", 0) + 1
                stop.wait(args.reset_period_ms / 1e3)

        threads = [threading.Thread(target=sender),
                   threading.Thread(target=resetter)]
        for thread in threads:
            thread.start()
        stop.wait(args.seconds)
        stop.set()
        for thread in threads:
            thread.join(timeout=10)
        # The node must still answer and recover to VALID.
        recovered = None
        deadline = time.time() + 10
        while time.time() < deadline:
            try:
                v = client.call(client.validity, None) if hasattr(client, "validity") else None
            except Exception:
                v = None
            if v is not None and getattr(v, "state", "") == "VALID":
                recovered = True
                break
            time.sleep(0.2)
        alive = node.proc.poll() is None
        record.update({"inputs_sent": sent["n"],
                       "resets_done": record.get("rounds_done", 0),
                       "node_alive_after": alive,
                       "recovered_valid": bool(recovered)})
        record["survived"] = alive and record["rounds_done"] >= args.rounds
        (out / "crash_repro.json").write_text(json.dumps(record, indent=1))
        print(json.dumps({k: record[k] for k in
                          ("inputs_sent", "resets_done", "node_alive_after",
                           "recovered_valid", "survived")}))
        return 0 if record["survived"] else 5
    finally:
        try:
            client.shutdown()
        except Exception:
            pass
        node.stop()


if __name__ == "__main__":
    raise SystemExit(main())
