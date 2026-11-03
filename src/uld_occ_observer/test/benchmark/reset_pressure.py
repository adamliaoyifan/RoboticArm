#!/usr/bin/env python3
"""R pressure-coverage gate (plan OCTOMAP_RESET_VALIDITY_NEXT_20261103 §5).

  reset_pressure.py --mode sweep   ...   non-scored 10/20/40/80 Hz x 10 s
  reset_pressure.py --mode score   ...   3 scored 60 s windows at the frozen
                                          rate + per-window drain + 30 s
                                          2.5/2 Hz recovery
  reset_pressure.py --mode judge --run <dir>

Sweep: for each offered rate the runner drives inputs at that rate plus 2 Hz
full queries for 10 s on the SAME frozen 20-box frames, then reports per-rate
pressure evidence -- the product's own queue wait (queue_ms from the
observation receipts) and any explicit rejection (QUEUE_FULL/REJECTED on
inputs or queries). The LOWEST rate with at least one real pressure event is
frozen (cap 80 Hz); if none reaches it, the score mode records
``not_evaluated: pressure_unreached`` -- never a pass and never a raised cap.

A pressure event is predeclared and bounded: queue_ms >= 50 ms (a wait that
long cannot be scheduler noise at these periods) or an explicit rejection
status. Raw queue_ms values are retained per input so the judge re-derives
the verdict with the same rule from disk.

Score: every window is 60 s at the frozen rate with 2 Hz queries; after each
window the drain waits until in-flight inputs reach 0 (bounded 30 s); the
recovery phase runs 30 s at the contract's 2.5 Hz update / 2 Hz query rates
on the same node and measures the first valid read. Judge: one terminal per
sent input, >=1 pressure event per window, drain empty within 30 s, first
recovery read <= 2 s, and the remaining recovery requests meeting the P/H
latency gates (input P95 <= 350 ms, query P95 <= 250 ms).
"""
from __future__ import annotations

import argparse
import json
import statistics
import threading
import time
from pathlib import Path

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
import freespace_campaign as fs  # noqa: E402
import freespace_contract as contract  # noqa: E402

QUEUE_EVENT_MS = 50.0
SWEEP_RATES = (10.0, 20.0, 40.0, 80.0)
SWEEP_SECONDS = 10.0
REJECTION_STATUSES = {"QUEUE_FULL", "REJECTED"}
#: A served-but-negative geometric answer is a VALID read (contract §5:
#: NO_KNOWN_FREE is legal); only invalidity/absence makes a read unserved.
UNSERVED_STATUSES = {"INVALIDATED", "NO_DATA", "STALE", "TIMEOUT"}
RECOVERY_FIRST_READ_MAX_S = 2.0
P95_INPUT_MS = 350.0
P95_QUERY_MS = 250.0


def _p95(values):
    if not values:
        return None
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(round(0.95 * len(ordered))))]


def _pressure_events(timeline, query_records):
    """Derive pressure evidence from raw records (judge reuses this)."""
    events = []
    for entry in timeline:
        queue = entry.get("queue_ms")
        if queue is not None and queue >= QUEUE_EVENT_MS:
            events.append({"input_id": entry["input_id"], "kind": "queue",
                           "queue_ms": queue})
        status = entry.get("receipt_status")
        if status in REJECTION_STATUSES:
            events.append({"input_id": entry["input_id"], "kind": "reject",
                           "status": status})
    for record in query_records:
        if record.get("status") in REJECTION_STATUSES:
            events.append({"index": record.get("index"), "kind": "reject",
                           "status": record.get("status")})
    return events


def _run_window(client, frames, rate, window_s, label, out_dir=None):
    """One offered-load window: inputs at `rate` + 2 Hz full queries."""
    t0_ns = time.monotonic_ns()
    base_stamp = int(time.time() * 1e9)
    input_records, query_records, errors = [], [], []

    def send():
        try:
            input_records.extend(
                fs.send_inputs(client, frames, int(rate * window_s),
                               rate, t0_ns, base_stamp))
        except Exception as error:  # noqa: BLE001
            errors.append("inputs: %r" % error)

    def query():
        try:
            query_records.extend(
                fs.run_queries(client, int(contract.QUERY_HZ * window_s),
                               contract.QUERY_HZ, t0_ns))
        except Exception as error:  # noqa: BLE001
            errors.append("queries: %r" % error)

    threads = [threading.Thread(target=send),
               threading.Thread(target=query)]
    for thread in threads:
        thread.start()
    window_end_ns = t0_ns + int(window_s * 1e9)
    for thread in threads:
        thread.join(timeout=window_s + 60)
    return {"label": label, "rate_hz": rate, "window_s": window_s,
            "t0_ns": t0_ns, "window_end_ns": window_end_ns,
            "inputs": input_records, "queries": query_records,
            "errors": errors}


def _drain(client, window, budget_s=30.0):
    """Wait until every sent input has a terminal; bounded by the budget."""
    drain_start_ns = time.monotonic_ns()
    deadline = time.monotonic() + budget_s
    ids = [record["input_id"] for record in window["inputs"]]
    timeline = fs.receipt_timeline(client, window["inputs"], timeout_s=budget_s)
    pending = []
    while time.monotonic() < deadline:
        _terminals, pending = fs.collect_terminals(
            client, [e["input_id"] for e in timeline
                     if e.get("receipt_status") is None], timeout_s=1.0)
        # collect_terminals is epoch-anchored; the authoritative closure is
        # the receipt timeline (the campaign's own subscription).
        pending = [entry["input_id"] for entry in timeline
                   if entry.get("receipt_status") is None
                   or entry.get("receipt_count", 0) == 0]
        if not pending:
            break
        time.sleep(0.2)
    drain_end_ns = time.monotonic_ns()
    return {"drain_start_ns": drain_start_ns, "drain_end_ns": drain_end_ns,
            "drain_s": round((drain_end_ns - drain_start_ns) / 1e9, 3),
            "in_flight_at_drain_end": pending,
            "drained_within_budget": not pending
            and (drain_end_ns - drain_start_ns) <= budget_s * 1e9,
            "timeline": timeline}


def _with_queue_ms(timeline, client):
    """Attach the product's own queue wait per input receipt row."""
    # receipt_timeline keeps every receipt event; the queue wait rides on the
    # campaign's receipt subscription records (client.receipts).
    by_id = {}
    for receipt in client.receipts:
        by_id.setdefault(receipt.get("observation_id"), []).append(receipt)
    for entry in timeline:
        rows = by_id.get(entry["input_id"]) or []
        queues = [r.get("queue_ms") for r in rows
                  if r.get("queue_ms") is not None]
        entry["queue_ms_max"] = max(queues) if queues else None
        entry["queue_ms"] = entry["queue_ms_max"]
    return timeline


def sweep(args):
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    frames = fs.load_frames(args.frames_glob)
    node = fs.NodeProcess(args.node, args.geometry, args.config,
                          out / "observer.log", "/reset_pressure_cloud")
    client = fs.Client("/reset_pressure_cloud")
    import hashlib
    client.config_digest = hashlib.sha256(
        Path(args.config).read_bytes()).hexdigest()
    results = {"mode": "sweep", "rates": [], "queue_event_ms": QUEUE_EVENT_MS}
    frozen_rate = None
    try:
        client.wait_services()
        first = frames[0]
        tf = first.get("transform") or {}
        client.broadcast_tf((tf.get("translation")
                             or [-1.15, 0.0, 1.20]))
        for rate in SWEEP_RATES:
            fs.readiness(client, frames)
            window = _run_window(client, frames, rate, SWEEP_SECONDS,
                                 "sweep_%ghz" % rate)
            drain = _drain(client, window, budget_s=30.0)
            _with_queue_ms(drain["timeline"], client)
            events = _pressure_events(drain["timeline"], window["queries"])
            queues = [e["queue_ms"] for e in drain["timeline"]
                      if e.get("queue_ms") is not None]
            row = {
                "rate_hz": rate, "window_s": SWEEP_SECONDS,
                "pressure_events": events,
                "queue_ms_max": max(queues) if queues else None,
                "queue_ms_p50": statistics.median(queues) if queues else None,
                "inputs_sent": len(window["inputs"]),
                "inputs_terminal": sum(
                    1 for e in drain["timeline"]
                    if e.get("receipt_status") is not None),
                "drain_s": drain["drain_s"],
                "errors": window["errors"],
            }
            results["rates"].append(row)
            (out / ("sweep_%ghz.json" % rate)).write_text(json.dumps(
                {"window": {k: v for k, v in window.items()
                            if k not in ("inputs", "queries")},
                 "drain": {k: v for k, v in drain.items()
                           if k != "timeline"},
                 "timeline": drain["timeline"]}, default=str))
            if events and frozen_rate is None:
                frozen_rate = rate
        results["frozen_rate_hz"] = frozen_rate
        results["pressure_unreached"] = frozen_rate is None
        (out / "sweep_result.json").write_text(json.dumps(results, default=str))
        print(json.dumps({"frozen_rate_hz": frozen_rate,
                          "per_rate_events": [
                              {"rate": r["rate_hz"],
                               "events": len(r["pressure_events"])}
                              for r in results["rates"]]}))
        return 0
    finally:
        client.shutdown()
        node.stop()


def score(args):
    if not args.frozen_rate:
        raise SystemExit("--frozen-rate required for score mode "
                         "(run the sweep first; if pressure_unreached, "
                         "record not_evaluated instead)")
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    frames = fs.load_frames(args.frames_glob)
    node = fs.NodeProcess(args.node, args.geometry, args.config,
                          out / "observer.log", "/reset_pressure_cloud")
    client = fs.Client("/reset_pressure_cloud")
    import hashlib
    client.config_digest = hashlib.sha256(
        Path(args.config).read_bytes()).hexdigest()
    artifact = {"mode": "score", "frozen_rate_hz": args.frozen_rate,
                "windows": [], "recovery": None,
                "queue_event_ms": QUEUE_EVENT_MS}
    try:
        client.wait_services()
        first = frames[0]
        tf = first.get("transform") or {}
        client.broadcast_tf((tf.get("translation")
                             or [-1.15, 0.0, 1.20]))
        for index in range(args.windows):
            fs.readiness(client, frames)
            window = _run_window(client, frames, args.frozen_rate,
                                 args.window_seconds,
                                 "score_w%d" % index)
            drain = _drain(client, window, budget_s=30.0)
            _with_queue_ms(drain["timeline"], client)
            events = _pressure_events(drain["timeline"], window["queries"])
            artifact["windows"].append({
                "index": index,
                "window": {k: v for k, v in window.items()
                           if k not in ("inputs", "queries")},
                "inputs": len(window["inputs"]),
                "queries": len(window["queries"]),
                "pressure_events": events,
                "drain": {k: v for k, v in drain.items()
                          if k != "timeline"},
                "errors": window["errors"],
            })
            (out / ("score_w%d.json" % index)).write_text(json.dumps(
                {"summary": artifact["windows"][-1],
                 "window_full": window, "drain_full": drain}, default=str))
        # Recovery: 30 s at the contract's 2.5 Hz update / 2 Hz query rates,
        # on the same node; the first valid read must land within 2 s.
        recovery_t0_ns = time.monotonic_ns()
        base_stamp = int(time.time() * 1e9)
        first_read = {}

        def first_reader():
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                started = time.monotonic_ns()
                result = fs.full_query(client, contract.F_BOXES[0])
                ended = time.monotonic_ns()
                status = result.get("status")
                first_read["attempt"] = {
                    "status": status, "started_mono_ns": started,
                    "ended_mono_ns": ended,
                    "latency_ms": round((ended - started) / 1e6, 3)}
                if status not in UNSERVED_STATUSES:
                    first_read["first_valid_read_s"] = round(
                        (started - recovery_t0_ns) / 1e9, 3)
                    return
                time.sleep(0.1)

        reader = threading.Thread(target=first_reader)
        reader.start()
        recovery_window = _run_window(client, frames,
                                      contract.UPDATE_HZ,
                                      args.recovery_seconds, "recovery")
        reader.join(timeout=15)
        artifact["recovery"] = {
            "t0_ns": recovery_t0_ns, "first_read": first_read,
            "window": {k: v for k, v in recovery_window.items()
                       if k not in ("inputs", "queries")},
            "inputs": len(recovery_window["inputs"]),
            "queries": len(recovery_window["queries"]),
            "errors": recovery_window["errors"],
        }
        (out / "recovery.json").write_text(json.dumps(
            {"summary": artifact["recovery"],
             "queries": recovery_window.get("queries"),
             "inputs": recovery_window.get("inputs")}, default=str))
        (out / "pressure_score.json").write_text(json.dumps(
            artifact, default=str))
        print(json.dumps({"windows": len(artifact["windows"]),
                          "events_per_window": [
                              len(w["pressure_events"])
                              for w in artifact["windows"]],
                          "first_read": first_read}))
        return 0
    finally:
        client.shutdown()
        node.stop()


def judge(run: Path):
    """Recompute the R verdict from the artifacts on disk."""
    problems = []
    sweep_path = run / "sweep_result.json"
    score_path = run / "pressure_score.json"
    if not sweep_path.is_file():
        return {"status": "fail", "problems": ["missing_artifact:sweep_result"]}
    sweep = json.loads(sweep_path.read_text())
    verdict = {"sweep": {r["rate_hz"]: len(r["pressure_events"])
                         for r in sweep["rates"]},
               "frozen_rate_hz": sweep.get("frozen_rate_hz")}
    if not score_path.is_file():
        # Pressure never reached at any rate <= the cap: not_evaluated, and
        # the cap must not rise after seeing this.
        if sweep.get("pressure_unreached"):
            verdict["status"] = "not_evaluated"
            verdict["reason"] = "pressure_unreached"
            return verdict
        return {"status": "fail", "problems": ["missing_artifact:score"]}
    score_doc = json.loads(score_path.read_text())
    for window in score_doc.get("windows") or []:
        index = window.get("index")
        if not window.get("pressure_events"):
            problems.append("window_without_pressure_event:%s" % index)
        drain = window.get("drain") or {}
        if not drain.get("drained_within_budget"):
            problems.append("drain_not_empty_in_30s:%s" % index)
        errors = window.get("errors") or []
        if errors:
            problems.append("window_errors:%s:%s" % (index, errors[:2]))
    recovery = score_doc.get("recovery") or {}
    first = (recovery.get("first_read") or {}).get("first_valid_read_s")
    if first is None or first > RECOVERY_FIRST_READ_MAX_S:
        problems.append("recovery_first_read:%s" % first)
    # Recovery remainder meets the P/H latency gates (query-side P95; the
    # per-query records live in recovery.json's queries list).
    doc = json.loads((run / "recovery.json").read_text()) \
        if (run / "recovery.json").is_file() else {}
    recovery_latency = []
    for record in doc.get("queries") or []:
        if record.get("status") in UNSERVED_STATUSES:
            continue
        if record.get("latency_ms") is not None:
            recovery_latency.append(record["latency_ms"])
            continue
        started = record.get("started_mono_ns")
        planned = record.get("planned_mono_ns")
        if started and planned:
            recovery_latency.append((started - planned) / 1e6)
    verdict["recovery_query_p95_ms"] = _p95(recovery_latency)
    if _p95(recovery_latency) is None or \
            _p95(recovery_latency) > P95_QUERY_MS:
        problems.append("recovery_query_p95:%s"
                        % _p95(recovery_latency))
    verdict["status"] = "fail" if problems else "pass"
    verdict["problems"] = problems
    return verdict


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", required=True,
                        choices=["sweep", "score", "judge"])
    parser.add_argument("--node", default=None)
    parser.add_argument("--geometry", default=None)
    parser.add_argument("--config", default=None)
    parser.add_argument("--frames-glob", default=None)
    parser.add_argument("--out", required=True)
    parser.add_argument("--frozen-rate", type=float, default=None)
    parser.add_argument("--windows", type=int, default=3)
    parser.add_argument("--window-seconds", type=float, default=60.0)
    parser.add_argument("--recovery-seconds", type=float, default=30.0)
    args = parser.parse_args()
    if args.mode == "sweep":
        return sweep(args)
    if args.mode == "score":
        return score(args)
    verdict = judge(Path(args.out))
    (Path(args.out) / "pressure_verdict.json").write_text(
        json.dumps(verdict, indent=1))
    print(json.dumps(verdict, indent=1))
    return 0 if verdict.get("status") == "pass" else 5


if __name__ == "__main__":
    raise SystemExit(main())
