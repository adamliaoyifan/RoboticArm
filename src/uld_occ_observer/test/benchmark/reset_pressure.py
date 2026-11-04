#!/usr/bin/env python3
"""R pressure-coverage gate (plan OCTOMAP_RESET_VALIDITY_NEXT_20261103 §5).

  reset_pressure.py --mode sweep   ...   non-scored 10/20/40/80 Hz x 10 s
  reset_pressure.py --mode score   ...   3 scored 60 s windows at the frozen
                                          rate, each followed by a bounded
                                          30 s drain, then ONE final 30 s
                                          2.5/2 Hz recovery phase
  reset_pressure.py --mode judge --run <dir>

Recovery scope note (contract wording): the frozen §5 text says "3 scored
windows + 30 s drain + 30 s recovery" -- ONE recovery phase after the
overload sequence, not a recovery per window. The collector implements
exactly that and records every first-read attempt (retries preserved, the
first request bound); "first valid read" is measured to the COMPLETION of a
served read (call end, including latency and release), never to the call
start.

Sweep: for each offered rate the runner drives inputs at that rate plus 2 Hz
full queries for 10 s, then reports per-rate pressure evidence -- the
product's own queue wait (queue_ms from the observation receipts) and any
explicit rejection (QUEUE_FULL/REJECTED on inputs or queries). The LOWEST
rate with at least one real pressure event is frozen (cap 80 Hz); if none
reaches it, the score mode records ``not_evaluated: pressure_unreached`` --
never a pass and never a raised cap.

A pressure event is predeclared and bounded: queue_ms >= 50 ms (a wait that
long cannot be scheduler noise at these periods) or an explicit rejection
status. Raw queue_ms values are retained per input so the judge re-derives
the verdict with the same rule from disk.

Judge (recomputes EVERYTHING from the raw per-window files; the summary
documents are only cross-checked, never trusted): exactly 3 raw 60 s windows
at the sweep-frozen rate with 2 Hz queries, per-ID terminal closure, >=1
REAL pressure event per window recomputed from the raw rows, drain derived
from the offered-load end and the last receipt ARRIVAL (never a poll bool),
one 30 s recovery with 75 inputs / 60 queries all in legal normal outcomes,
first served read COMPLETING within 2 s of recovery start, and the P/H
latency gates over every recovery query (failed queries are findings, never
filtered out of the statistics).
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
#: Legal NORMAL outcomes for recovery queries (served answers; a geometric
#: "no free cell" is served). Anything else is a finding, never filtered.
NORMAL_QUERY_OK = {"OK", "NO_KNOWN_FREE"}
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


def _perf_lock():
    """s2_o9-style exclusive perf lock (blocking, wait recorded). Held for
    the WHOLE run: node startup, offered load, sampling, drain, recovery --
    released only after the node is stopped. The gen3 closeout recorded the
    manual run02 R running WITHOUT the lock as a named limitation; from gen4
    the tool self-locks and the driver releases its own fd before R."""
    import fcntl
    lock_path = Path("/tmp/elfin-occ-backend-perf.lock")
    lock_path.touch()
    handle = lock_path.open("a+")
    waited = time.monotonic()
    fcntl.flock(handle, fcntl.LOCK_EX)
    return handle, {"path": str(lock_path),
                    "wait_s": round(time.monotonic() - waited, 3),
                    "acquired": True}


def _perf_unlock(handle):
    import fcntl
    fcntl.flock(handle, fcntl.LOCK_UN)
    handle.close()


def _formal_frames(args):
    """Contract offered load: the 20-box frame group ONLY, hard-failing on a
    mixed or missing set (run02's R gate failed exactly this way and the
    gen4 plan froze the repair)."""
    frames = fs.load_frames(args.frames_glob)
    groups = fs.group_frames_by_box(frames, strict=True)
    return groups[args.box_tag]


def sweep(args):
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    frames = _formal_frames(args)
    lock_handle, lock_info = _perf_lock()
    node = fs.NodeProcess(args.node, args.geometry, args.config,
                          out / "observer.log", "/reset_pressure_cloud")
    client = fs.Client("/reset_pressure_cloud")
    import hashlib
    client.config_digest = hashlib.sha256(
        Path(args.config).read_bytes()).hexdigest()
    results = {"mode": "sweep", "rates": [], "queue_event_ms": QUEUE_EVENT_MS,
               "perf_lock": lock_info}
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
                            if k != "inputs"},
                 "drain": {k: v for k, v in drain.items()
                           if k != "timeline"},
                 "timeline": drain["timeline"]}, default=str))
            if events and frozen_rate is None:
                frozen_rate = rate
        results["frozen_rate_hz"] = frozen_rate
        results["pressure_unreached"] = frozen_rate is None
        (out / "sweep_result.json").write_text(json.dumps(results, default=str))
        (out / "load_identity.json").write_text(json.dumps({
            "frames_manifest": fs.frames_manifest(frames),
            "perf_lock": lock_info}, default=str))
        print(json.dumps({"frozen_rate_hz": frozen_rate,
                          "per_rate_events": [
                              {"rate": r["rate_hz"],
                               "events": len(r["pressure_events"])}
                              for r in results["rates"]]}))
        return 0
    finally:
        client.shutdown()
        node.stop()
        _perf_unlock(lock_handle)


def score(args):
    if not args.frozen_rate:
        raise SystemExit("--frozen-rate required for score mode "
                         "(run the sweep first; if pressure_unreached, "
                         "record not_evaluated instead)")
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    frames = _formal_frames(args)
    lock_handle, lock_info = _perf_lock()
    node = fs.NodeProcess(args.node, args.geometry, args.config,
                          out / "observer.log", "/reset_pressure_cloud")
    client = fs.Client("/reset_pressure_cloud")
    import hashlib
    client.config_digest = hashlib.sha256(
        Path(args.config).read_bytes()).hexdigest()
    artifact = {"mode": "score", "frozen_rate_hz": args.frozen_rate,
                "windows": [], "recovery": None,
                "queue_event_ms": QUEUE_EVENT_MS,
                "perf_lock": lock_info}
    health_mark = len(client.health)
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
        # on the same node; the first valid read must COMPLETE within 2 s.
        # Every attempt is preserved (retries included) and the first valid
        # read is measured to the served call's END, never its start.
        recovery_t0_ns = time.monotonic_ns()
        base_stamp = int(time.time() * 1e9)
        first_read = {"attempts": []}

        def first_reader():
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                started = time.monotonic_ns()
                result = fs.full_query(client, contract.F_BOXES[0])
                ended = time.monotonic_ns()
                status = result.get("status")
                first_read["attempts"].append({
                    "status": status, "started_mono_ns": started,
                    "ended_mono_ns": ended,
                    "latency_ms": round((ended - started) / 1e6, 3),
                    "release_status": result.get("release_status"),
                    "pages": result.get("pages")})
                if status not in UNSERVED_STATUSES:
                    # Completion time (call end) relative to recovery start.
                    first_read["first_valid_read_s"] = round(
                        (ended - recovery_t0_ns) / 1e9, 3)
                    return
                time.sleep(0.1)

        reader = threading.Thread(target=first_reader)
        reader.start()
        recovery_window = _run_window(client, frames,
                                      contract.UPDATE_HZ,
                                      args.recovery_seconds, "recovery")
        reader.join(timeout=15)
        # Recovery input terminals: EVIDENCE COLLECTION ONLY -- bounded 5 s,
        # run after the window closed. No request deadline moves: every
        # planned/publish stamp was already fixed inside the window; this
        # only joins the receipt rows onto them.
        recovery_receipts = fs.receipt_timeline(
            client, recovery_window["inputs"], timeout_s=5.0)
        # Health over the whole scored sequence (windows + recovery): the
        # Client subscription is live; project like stage_l does.
        health_series = [
            {"stamp_ns": sample.get("stamp_ns"),
             "arrival_mono_ns": sample.get("arrival_mono_ns"),
             "epoch": sample.get("epoch")}
            for sample in client.health[health_mark:]
            if isinstance(sample, dict)
            and isinstance(sample.get("stamp_ns"), int)]
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
             "inputs": recovery_window.get("inputs"),
             "input_receipts": recovery_receipts,
             "health_series": health_series}, default=str))
        (out / "pressure_score.json").write_text(json.dumps(
            artifact, default=str))
        # Load identity binding (post-hoc lesson from run02: the offered-load
        # box mix must be provable from the artifact itself, not from the
        # command line alone).
        import hashlib as _hashlib
        (out / "load_identity.json").write_text(json.dumps({
            "frames_manifest": fs.frames_manifest(frames),
            "perf_lock": lock_info,
            "node_sha256": _hashlib.sha256(
                Path(args.node).read_bytes()).hexdigest(),
            "geometry_sha256": _hashlib.sha256(
                Path(args.geometry).read_bytes()).hexdigest(),
            "config_sha256": _hashlib.sha256(
                Path(args.config).read_bytes()).hexdigest()}, default=str))
        print(json.dumps({"windows": len(artifact["windows"]),
                          "events_per_window": [
                              len(w["pressure_events"])
                              for w in artifact["windows"]],
                          "first_read": first_read}))
        return 0
    finally:
        client.shutdown()
        node.stop()
        _perf_unlock(lock_handle)


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def judge(run: Path, expected_windows=3, window_s=60.0,
          recovery_s=30.0, drain_budget_s=30.0):
    """Recompute the R verdict from the RAW artifacts on disk.

    Nothing is taken from a summary document without re-derivation: pressure
    events, terminal closure, the drain deadline (offered-load end -> last
    receipt ARRIVAL), the frozen rate, the recovery schedule, and the first
    valid read (a SERVED attempt COMPLETING within the bound) are all
    recomputed from the raw rows; summaries are only cross-checked.
    """
    problems = []
    verdict = {"rule_set": "OCTOMAP_RESET_VALIDITY_NEXT_20261103 §5 R"}

    # ---- sweep: recompute events per rate from the RAW timelines ----------
    sweep_raw_events = {}
    sweep_summary = None
    sweep_path = run / "sweep_result.json"
    if not sweep_path.is_file():
        return {"status": "fail", "problems": ["missing_artifact:sweep_result"]}
    sweep_summary = json.loads(sweep_path.read_text())
    for rate in SWEEP_RATES:
        raw_path = run / ("sweep_%ghz.json" % rate)
        if not raw_path.is_file():
            problems.append("missing_artifact:sweep_%ghz" % rate)
            continue
        raw = json.loads(raw_path.read_text())
        timeline = raw.get("timeline") or []
        raw_queries = (raw.get("window") or {}).get("queries") or []
        events = _pressure_events(timeline, raw_queries)
        sweep_raw_events[rate] = len(events)
        # Closure on the sweep rows too: every offered input has a terminal.
        open_rows = [e["input_id"] for e in timeline
                     if not e.get("receipt_status")]
        if open_rows:
            problems.append("sweep_%ghz_open_rows:%d" % (rate, len(open_rows)))
        summary_row = next((r for r in sweep_summary.get("rates") or []
                            if r.get("rate_hz") == rate), None)
        if summary_row is None:
            problems.append("sweep_summary_missing_rate:%s" % rate)
        elif len(summary_row.get("pressure_events") or []) != len(events):
            problems.append("sweep_%ghz_events_summary_mismatch:%s!=%s"
                            % (rate, len(summary_row.get("pressure_events")
                                         or []),
                               len(events)))
    # Frozen rate = LOWEST rate with >=1 recomputed event, capped at 80 Hz.
    derived_frozen = next((rate for rate in SWEEP_RATES
                           if sweep_raw_events.get(rate)), None)
    if derived_frozen is None:
        verdict["status"] = "not_evaluated"
        verdict["reason"] = "pressure_unreached"
        verdict["sweep"] = sweep_raw_events
        verdict["problems"] = problems
        return verdict
    if derived_frozen > 80.0:
        problems.append("frozen_rate_over_cap:%s" % derived_frozen)
    if sweep_summary.get("frozen_rate_hz") != derived_frozen:
        problems.append("frozen_rate_mismatch:%s!=derived_%s"
                        % (sweep_summary.get("frozen_rate_hz"), derived_frozen))
    verdict["sweep"] = sweep_raw_events
    verdict["frozen_rate_hz"] = derived_frozen

    # ---- freeze-before-score binding (rule + rate existed before score) ----
    try:
        sweep_mtime = sweep_path.stat().st_mtime_ns
        score_paths = [run / ("score_w%d.json" % i)
                       for i in range(expected_windows)]
        if all(p.is_file() for p in score_paths):
            first_score_mtime = min(p.stat().st_mtime_ns for p in score_paths)
            verdict["sweep_before_score"] = sweep_mtime < first_score_mtime
            if not verdict["sweep_before_score"]:
                problems.append("score_predates_sweep")
    except OSError:
        pass
    freeze_path = run.parent / "FREEZE_INTEG.json"
    if freeze_path.is_file():
        freeze = json.loads(freeze_path.read_text())
        rule = freeze.get("pressure_rule") or {}
        verdict["freeze_pressure_rule"] = {
            "queue_event_ms": rule.get("queue_event_ms"),
            "cap_hz": rule.get("cap_hz"),
            "sweep_hz": rule.get("sweep_hz")}
        if rule.get("queue_event_ms") != QUEUE_EVENT_MS:
            problems.append("freeze_rule_mismatch:queue_event_ms")
        if sorted(rule.get("sweep_hz") or []) != sorted(SWEEP_RATES):
            problems.append("freeze_rule_mismatch:sweep_hz")
        if rule.get("cap_hz") != 80:
            problems.append("freeze_rule_mismatch:cap_hz")
    else:
        problems.append("freeze_binding_missing:FREEZE_INTEG.json")

    # ---- offered-load identity + perf lock (gen4 gates; missing = evidence
    # incomplete, never a silent pass -- run02's R failed exactly here) -----
    load_path = run / "load_identity.json"
    if not load_path.is_file():
        problems.append("load_identity_missing")
    else:
        load = json.loads(load_path.read_text())
        manifest = load.get("frames_manifest") or []
        if not manifest:
            problems.append("load_identity:frames_manifest_missing")
        bad_tags = sorted({entry.get("box_tag") for entry in manifest}
                          - {20})
        if bad_tags:
            problems.append("load_identity:not_20box:%s" % bad_tags)
        verdict["load_box_tags"] = sorted(
            {entry.get("box_tag") for entry in manifest})
        lock_record = load.get("perf_lock")
        verdict["perf_lock"] = lock_record
        if not isinstance(lock_record, dict) \
                or lock_record.get("acquired") is not True:
            problems.append("perf_lock:not_recorded_as_held")

    # ---- scored windows: recompute from score_w{i}.json --------------------
    score_summary_path = run / "pressure_score.json"
    score_summary = json.loads(score_summary_path.read_text()) \
        if score_summary_path.is_file() else None
    if score_summary is None:
        problems.append("missing_artifact:pressure_score")
    window_epochs = set()
    last_drain_end = None
    for index in range(expected_windows):
        raw_path = run / ("score_w%d.json" % index)
        if not raw_path.is_file():
            problems.append("missing_artifact:score_w%d" % index)
            continue
        raw = json.loads(raw_path.read_text())
        window = raw.get("window_full") or {}
        drain_full = raw.get("drain_full") or {}
        timeline = drain_full.get("timeline") or []
        queries = window.get("queries") or []
        inputs = window.get("inputs") or []
        prefix = "w%d" % index
        if window.get("rate_hz") != derived_frozen:
            problems.append("%s_rate_%s" % (prefix, window.get("rate_hz")))
        if window.get("window_s") != window_s:
            problems.append("%s_window_s_%s" % (prefix, window.get("window_s")))
        if len(inputs) != int(derived_frozen * window_s):
            problems.append("%s_inputs_%d" % (prefix, len(inputs)))
        if len(queries) != int(contract.QUERY_HZ * window_s):
            problems.append("%s_queries_%d" % (prefix, len(queries)))
        if window.get("errors"):
            problems.append("%s_errors:%s" % (prefix, window["errors"][:2]))
        # Per-ID closure + uniqueness on the RAW rows.
        ids = [row.get("input_id") for row in inputs]
        if len(set(ids)) != len(ids):
            problems.append("%s_duplicate_input_id" % prefix)
        by_id = {}
        for row in timeline:
            by_id.setdefault(row.get("input_id"), []).append(row)
        missing_terminal = [i for i in ids
                            if not any(r.get("receipt_status")
                                       for r in by_id.get(i, []))]
        if missing_terminal:
            problems.append("%s_open_inputs:%d" % (prefix,
                                                   len(missing_terminal)))
        # Pressure events recomputed from the raw rows (>=1 per window).
        events = _pressure_events(timeline, queries)
        if not events:
            problems.append("%s_without_pressure_event" % prefix)
        verdict["w%d_pressure_events" % index] = len(events)
        # Drain: derived from the offered-load end and the last receipt
        # ARRIVAL -- never from the poll loop's boolean.
        drain_start = drain_full.get("drain_start_ns")
        arrivals = [r.get("arrival_mono_ns")
                    for row in timeline
                    for r in (row.get("receipt_events") or [])
                    if isinstance(r.get("arrival_mono_ns"), int)]
        arrivals += [row.get("receipt_arrival_mono_ns") for row in timeline
                     if isinstance(row.get("receipt_arrival_mono_ns"), int)]
        if not isinstance(drain_start, int) or not arrivals:
            problems.append("%s_drain_boundaries_missing" % prefix)
        else:
            drain_time_s = (max(arrivals) - drain_start) / 1e9
            verdict["w%d_drain_s" % index] = round(drain_time_s, 3)
            if drain_time_s > drain_budget_s:
                problems.append("%s_drain_%.1fs>%.0fs"
                                % (prefix, drain_time_s, drain_budget_s))
        pending = [row.get("input_id") for row in timeline
                   if not row.get("receipt_status")]
        if pending:
            problems.append("%s_in_flight_at_end:%d" % (prefix,
                                                        len(pending)))
        # Offered-load schedule: stamps step 1/rate, ids == str(stamp).
        stamps = [row.get("stamp_ns") for row in inputs]
        step_ns = int(round(1e9 / derived_frozen))
        if stamps and any(
                s2 - s1 != step_ns for s1, s2 in zip(stamps, stamps[1:])):
            problems.append("%s_stamp_grid_broken" % prefix)
        for row in inputs:
            if row.get("input_id") != str(row.get("stamp_ns")):
                problems.append("%s_id_not_stamp" % prefix)
                break
        # Query terminal + epoch continuity (same node, no reset).
        for row in queries:
            if not row.get("status"):
                problems.append("%s_query_without_terminal" % prefix)
                break
            identity = row.get("lease_identity") or {}
            if isinstance(identity.get("epoch"), int):
                window_epochs.add(identity["epoch"])
        if last_drain_end is not None and \
                isinstance(window.get("t0_ns"), int) and \
                window["t0_ns"] <= last_drain_end:
            problems.append("%s_overlaps_previous_drain" % prefix)
        if isinstance(drain_full.get("drain_end_ns"), int):
            last_drain_end = drain_full["drain_end_ns"]
        # Summary cross-check for this window.
        summary_row = next((w for w in ((score_summary or {}).get("windows")
                                        or [])
                            if w.get("index") == index), None)
        if summary_row is not None:
            if len(summary_row.get("pressure_events") or []) != len(events):
                problems.append("%s_events_summary_mismatch" % prefix)

    # ---- recovery: raw schedule + first read to COMPLETION ------------------
    recovery_path = run / "recovery.json"
    if not recovery_path.is_file():
        problems.append("missing_artifact:recovery")
        recovery = {}
    else:
        recovery = json.loads(recovery_path.read_text())
    rec_queries = recovery.get("queries") or []
    rec_inputs = recovery.get("inputs") or []
    summary = recovery.get("summary") or {}
    expected_inputs = int(contract.UPDATE_HZ * recovery_s)
    expected_queries = int(contract.QUERY_HZ * recovery_s)
    if len(rec_inputs) != expected_inputs:
        problems.append("recovery_inputs_%d" % len(rec_inputs))
    if len(rec_queries) != expected_queries:
        problems.append("recovery_queries_%d" % len(rec_queries))
    if summary.get("inputs") not in (None, len(rec_inputs)):
        problems.append("recovery_inputs_summary_mismatch")
    # Every recovery query judged: a non-normal status is a FINDING here --
    # never filtered out of the statistics.
    bad_status = [r.get("index") for r in rec_queries
                  if r.get("status") not in NORMAL_QUERY_OK]
    if bad_status:
        problems.append("recovery_bad_status:%s" % bad_status[:5])
    # Inherited P/H gates on the recovery INPUTS: per-ID terminal closure
    # (receipt rows joined by input_id -- evidence the run02 collector never
    # wrote), send lateness, and e2e. Deadlines stay the in-window planned
    # stamps; the collector's 5 s post-window evidence wait moves nothing.
    rec_receipts = recovery.get("input_receipts")
    if not isinstance(rec_receipts, list) or not rec_receipts:
        problems.append("recovery_input_receipts_missing")
    else:
        by_id = {}
        for row in rec_receipts:
            by_id.setdefault(row.get("input_id"), []).append(row)
        open_inputs = [row.get("input_id") for row in rec_inputs
                       if not any(r.get("receipt_status")
                                  for r in by_id.get(row.get("input_id"),
                                                     []))]
        if open_inputs:
            problems.append("recovery_inputs_open:%d" % len(open_inputs))
        send_late, e2e = [], []
        for row in rec_inputs:
            published, planned = (row.get("publish_mono_ns"),
                                  row.get("planned_mono_ns"))
            if not isinstance(published, int) or not isinstance(planned, int):
                problems.append("recovery_input_boundaries_missing")
                continue
            send_late.append((published - planned) / 1e6)
            arrivals = [r.get("receipt_arrival_mono_ns")
                        for r in by_id.get(row.get("input_id"), [])
                        if isinstance(r.get("receipt_arrival_mono_ns"), int)]
            if arrivals:
                e2e.append((max(arrivals) - published) / 1e6)
        if send_late:
            if _p95(send_late) > 20.0:
                problems.append("recovery_send_late_p95:%.1f"
                                % _p95(send_late))
            if max(send_late) > 50.0:
                problems.append("recovery_send_late_max:%.1f"
                                % max(send_late))
        if e2e:
            if _p95(e2e) > 350.0:
                problems.append("recovery_e2e_p95:%.1f" % _p95(e2e))
            if max(e2e) > 500.0:
                problems.append("recovery_e2e_max:%.1f" % max(e2e))
        verdict["recovery_input_e2e_p95_ms"] = _p95(e2e)
    # Recovery health liveness (the Client subscription is live; the gen4
    # collector projects it). Missing series = evidence incomplete.
    rec_health = recovery.get("health_series")
    if not isinstance(rec_health, list) or not rec_health:
        problems.append("recovery_health_missing")
    else:
        gaps = [(b.get("arrival_mono_ns") - a.get("arrival_mono_ns")) / 1e9
                for a, b in zip(rec_health, rec_health[1:])
                if isinstance(a.get("arrival_mono_ns"), int)
                and isinstance(b.get("arrival_mono_ns"), int)]
        verdict["recovery_health_gap_max_s"] = round(max(gaps), 3) if gaps \
            else None
        if gaps and max(gaps) >= 1.0:
            problems.append("recovery_health_gap_%.2fs" % max(gaps))
    latencies = []
    for row in rec_queries:
        started, planned, ended = (row.get("started_mono_ns"),
                                   row.get("planned_mono_ns"),
                                   row.get("ended_mono_ns"))
        if not all(isinstance(v, int) for v in (started, planned, ended)):
            problems.append("recovery_query_boundary_missing")
            continue
        if ended < started:
            problems.append("recovery_query_ended_before_started")
            continue
        latencies.append((ended - planned) / 1e6)
    verdict["recovery_query_p95_ms"] = _p95(latencies)
    if latencies:
        if _p95(latencies) > P95_QUERY_MS:
            problems.append("recovery_query_p95:%.1f" % _p95(latencies))
        if max(latencies) > 500.0:
            problems.append("recovery_query_max:%.1f" % max(latencies))
    ages = [row.get("age_ns") / 1e9 for row in rec_queries
            if isinstance(row.get("age_ns"), int)]
    if ages and max(ages) > 1.0:
        problems.append("recovery_age_max:%.2f" % max(ages))
    # First valid read: the first SERVED attempt COMPLETING within 2 s of
    # recovery start. A recorded start-based 0.0 with a TIMEOUT attempt is
    # rejected; retries are preserved and the first served one binds.
    first_read = summary.get("first_read") or {}
    # Cross-check the duplicate first_read copy in the score summary: the two
    # documents must agree -- a forged copy on either side is a finding.
    summary_first = ((score_summary or {}).get("recovery")
                     or {}).get("first_read")
    if summary_first is not None and summary_first != first_read:
        problems.append("first_read_copies_diverge")
    attempts = first_read.get("attempts")
    if attempts is None and first_read.get("attempt") is not None:
        attempts = [first_read["attempt"]]   # pre-fix artifact format
        verdict["first_read_format"] = "legacy_single_attempt(start-based)"
    if not attempts:
        problems.append("recovery_first_read:no_attempt_record")
    else:
        t0 = summary.get("t0_ns")
        first_served = None
        for attempt in attempts:
            status = attempt.get("status")
            started, ended = (attempt.get("started_mono_ns"),
                              attempt.get("ended_mono_ns"))
            if not isinstance(started, int) or not isinstance(ended, int) \
                    or ended < started:
                problems.append("recovery_first_read:attempt_boundaries")
                break
            latency = attempt.get("latency_ms")
            if not _finite(latency) or \
                    abs(latency - (ended - started) / 1e6) > 1.0:
                problems.append("recovery_first_read:latency_not_recomputed")
                break
            if status not in UNSERVED_STATUSES:
                first_served = attempt
                break
        if first_served is None:
            problems.append("recovery_first_read:no_served_attempt")
        elif not isinstance(t0, int):
            problems.append("recovery_first_read:t0_missing")
        else:
            completion_s = (first_served["ended_mono_ns"] - t0) / 1e9
            verdict["first_valid_read_completion_s"] = round(completion_s, 3)
            if completion_s > RECOVERY_FIRST_READ_MAX_S:
                problems.append("recovery_first_read:%.2fs>%.1fs"
                                % (completion_s, RECOVERY_FIRST_READ_MAX_S))
    # Same-node/epoch continuity across windows and recovery.
    for row in rec_queries:
        identity = row.get("lease_identity") or {}
        if isinstance(identity.get("epoch"), int):
            window_epochs.add(identity["epoch"])
    verdict["epochs_seen"] = sorted(window_epochs)
    if len(window_epochs) != 1:
        problems.append("epoch_discontinuity:%s" % sorted(window_epochs))
    # Stale summary window fields (kept for the record, cross-checked).
    if score_summary is not None and \
            score_summary.get("frozen_rate_hz") != derived_frozen:
        problems.append("score_summary_frozen_rate_mismatch")

    verdict["problems"] = problems
    verdict["status"] = "fail" if problems else "pass"
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
    parser.add_argument("--box-tag", type=int, default=20,
                        help="offered-load box count (contract R row: 20-box "
                             "frames; the group is strict -- a mixed or "
                             "missing set aborts instead of falling back)")
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
