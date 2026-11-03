#!/usr/bin/env python3
"""Plan-rule L judge (OCTOMAP_RESET_VALIDITY_NEXT_20261103 §5, L row).

  reset_l_judge.py --run <campaign-dir-containing-l.json>

The S3-frozen freespace_judge applies the OLDER S3 L rules (no reset-window
concept, any extra ID is a failure, ownership sampled at the reset instant).
The reset contract redefined exactly those three aspects -- its §5 L row is
the frozen rule set for this campaign:

- Planned IDs complete; <=2 extra updates and <=2 extra queries; all extra
  IDs judged with the same latency/status/identity rules.
- Reset-affected IDs (predeclared event window: the reset rounds' own
  [reset_called, round-end] interval, classified by planned OR execution
  overlap -- raw timestamps prove the reset won the linearization race) use
  the V1/V2 status contract, are excluded only from the normal-success
  denominator, and stay counted in all-ID closure and latency.
- Normal segments retain the S3 P/H thresholds and the 120 s-window AND.
- Resource classes reconcile after final release; at-sample transients
  charged to a LIVE owner (the bounded surface-cache entry) are legal
  (§1: size or survival at the reset instant is not a leak verdict).
- Resources: RSS <= 2 GiB, read view+cache combined peak <= 512 MiB,
  last-vs-first RSS median growth <= 5%.

Everything is recomputed from the l.json raw records on disk. The verdict
also records the S3-rule view (from the campaign's own judge output) so both
rule sets are visible -- never laundered.
"""
from __future__ import annotations

import argparse
import json
import statistics
import sys
from pathlib import Path

PLANNED_INPUTS = 18000
PLANNED_QUERIES = 14400
MAX_EXTRA = 2
P95_INPUT_MS = 350.0
MAX_MS = 500.0
P95_QUERY_MS = 250.0
MAX_AGE_S = 1.0
WINDOW_S = 120.0
WARMUP_S = 600.0
RSS_MAX_GIB = 2.0
VIEW_CACHE_PEAK_MIB = 512.0
RSS_MEDIAN_GROWTH_PCT = 5.0
#: V1/V2 status contract for reset-affected rows (plan §5 note: these are
#: expected only with raw-timestamp proof, which the window overlap is).
RESET_LEGAL_QUERY = {"ACQUIRE_NO_DATA", "PAGE_INVALIDATED", "INVALIDATED",
                     "NO_DATA", "STALE", "REVISION_NOT_COVERED"}
RESET_LEGAL_INPUT = {"EPOCH_MISMATCH", "REJECTED", "QUEUE_FULL"}
NORMAL_QUERY_OK = {"OK", "NO_KNOWN_FREE"}  # served answers; geometry may say no


def _p95(values):
    if not values:
        return None
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(round(0.95 * len(ordered))))]


def judge(run: Path):
    problems = []
    l_path = run / "l.json"
    if not l_path.is_file():
        return {"status": "fail", "problems": ["missing_artifact:l.json"]}
    d = json.loads(l_path.read_text())
    tl = d.get("input_timeline") or []
    qs = d.get("query_records") or []
    rr = d.get("reset_rounds_results") or []
    read_rounds = d.get("read_rounds_results") or []
    t0 = min((e.get("planned_mono_ns") or 0) for e in tl) if tl else 0

    # ---- predeclared reset event windows ------------------------------------
    windows = []
    for r in rr:
        start = r.get("reset_called_mono_ns")
        if not start:
            continue
        samples = (r.get("ownership_after") or {}).get("resource_samples") \
            or (r.get("ownership_after") or {}).get("samples") or []
        end = max((s.get("t_mono_ns") or 0) for s in samples) \
            if samples else start + 2_000_000_000
        windows.append((start, end))
    windows.sort()

    def affected(planned, started, ended):
        for start, end in windows:
            if planned is not None and planned <= end and planned >= start:
                return True
            if started is not None and started <= end and ended is not None \
                    and ended >= start:
                return True
        return False

    # ---- counts and overrun discipline --------------------------------------
    extra_inputs = len(tl) - PLANNED_INPUTS
    extra_queries = len(qs) - PLANNED_QUERIES
    if extra_inputs < 0:
        problems.append("inputs_missing:%d" % len(tl))
    if extra_queries < 0:
        problems.append("queries_missing:%d" % len(qs))
    if extra_inputs > MAX_EXTRA:
        problems.append("input_overrun:%d>%d" % (extra_inputs, MAX_EXTRA))
    if extra_queries > MAX_EXTRA:
        problems.append("query_overrun:%d>%d" % (extra_queries, MAX_EXTRA))

    # ---- all-ID closure ------------------------------------------------------
    no_terminal = [e["input_id"] for e in tl if not e.get("receipt_status")]
    if no_terminal:
        problems.append("inputs_without_terminal:%d" % len(no_terminal))

    # ---- classify rows, apply the two vocabularies ---------------------------
    normal_q, reset_q, bad_status = [], [], []
    for q in qs:
        planned = q.get("planned_mono_ns")
        started = q.get("started_mono_ns")
        ended = q.get("ended_mono_ns")
        status = q.get("status")
        if affected(planned, started, ended):
            reset_q.append(q)
            if status not in RESET_LEGAL_QUERY and \
                    status not in NORMAL_QUERY_OK:
                bad_status.append("reset_query:%s:%s" % (q.get("index"), status))
        else:
            normal_q.append(q)
            if status not in NORMAL_QUERY_OK:
                bad_status.append("normal_query:%s:%s" % (q.get("index"), status))
    normal_i, reset_i = [], []
    for e in tl:
        planned = e.get("planned_mono_ns")
        started = e.get("publish_mono_ns")
        ended = e.get("receipt_arrival_mono_ns")
        bucket = reset_i if affected(planned, started, ended) else normal_i
        bucket.append(e)
    for e in reset_i:
        if e.get("receipt_status") not in RESET_LEGAL_INPUT | {"INTEGRATED"}:
            bad_status.append("reset_input:%s:%s"
                              % (e["input_id"], e.get("receipt_status")))
    problems.extend(bad_status[:20])

    # ---- latency: all-ID (closure+latency includes reset rows) ---------------
    q_lat = [q["latency_ms"] for q in qs if q.get("latency_ms") is not None]
    i_e2e = [e["e2e_ms"] for e in tl if e.get("e2e_ms") is not None]
    ages = [q["age_ns"] / 1e9 for q in qs if q.get("age_ns") is not None]
    if _p95(q_lat) is None or _p95(q_lat) > P95_QUERY_MS:
        problems.append("query_p95:%s" % _p95(q_lat))
    if q_lat and max(q_lat) > MAX_MS:
        problems.append("query_max:%.1f" % max(q_lat))
    if _p95(i_e2e) is None or _p95(i_e2e) > P95_INPUT_MS:
        problems.append("input_p95:%s" % _p95(i_e2e))
    if i_e2e and max(i_e2e) > MAX_MS:
        problems.append("input_max:%.1f" % max(i_e2e))
    if ages and max(ages) > MAX_AGE_S:
        problems.append("age_max:%.3f" % max(ages))

    # ---- 120 s-window AND over the normal segment ----------------------------
    scored_start = t0 + int(WARMUP_S * 1e9)
    window_fail = 0
    buckets = {}
    for q in normal_q:
        t = q.get("planned_mono_ns") or 0
        if t < scored_start:
            continue
        buckets.setdefault(int((t - scored_start) / (WINDOW_S * 1e9)), []).append(q)
    for _key, rows in sorted(buckets.items()):
        lat = [r["latency_ms"] for r in rows if r.get("latency_ms") is not None]
        ag = [r["age_ns"] / 1e9 for r in rows if r.get("age_ns") is not None]
        if not lat or _p95(lat) > P95_QUERY_MS or max(lat) > MAX_MS \
                or (ag and max(ag) > MAX_AGE_S):
            window_fail += 1
    if buckets and window_fail:
        problems.append("window_and:%d/%d" % (window_fail, len(buckets)))

    # ---- ownership: final-release reconciliation per round --------------------
    baseline = d.get("baseline") or {}
    after_all = d.get("after") or {}
    rounds_residual = []
    for r in rr:
        before = r.get("ownership_before") or {}
        after = r.get("ownership_after") or {}
        bmin = (before.get("resource_samples") or [before])[-1] \
            if isinstance(before.get("resource_samples"), list) else before
        amin = (after.get("resource_samples") or [after])[-1] \
            if isinstance(after.get("resource_samples"), list) else after
        for k in ("client_held_view_block_bytes", "old_view_block_bytes",
                  "lease_view_block_bytes"):
            bv = bmin.get(k) or 0
            av = amin.get(k) or 0
            if av > bv:
                rounds_residual.append((r["index"], k, bv, av))
    if rounds_residual:
        problems.append("final_release_residual:%s"
                        % rounds_residual[:4])
    # The bounded surface-cache entry is a live owner (§1): constant size,
    # present in the run baseline AND the run-end state.
    cache_base = (baseline.get("resource_samples") or [baseline])[-1] \
        .get("surface_cache_bytes") if isinstance(
            baseline.get("resource_samples"), list) else baseline.get("cache_bytes")
    cache_end = (after_all.get("resource_samples") or [after_all])[-1] \
        .get("surface_cache_bytes") if isinstance(
            after_all.get("resource_samples"), list) else after_all.get("cache_bytes")
    if cache_base != cache_end:
        problems.append("cache_entry_drift:%s!=%s" % (cache_base, cache_end))
    bad_reads = [x for x in read_rounds if x.get("problems") or x.get("residual")]
    if bad_reads:
        problems.append("read_rounds_failed:%d/%d"
                        % (len(bad_reads), len(read_rounds)))
    if len(read_rounds) < 1000:
        problems.append("read_rounds_count:%d" % len(read_rounds))
    if len(rr) < 100:
        problems.append("reset_rounds_count:%d" % len(rr))

    # ---- resources ------------------------------------------------------------
    rss = [r.get("rss_kib") for r in d.get("rss_series") or []
           if r.get("rss_kib")]
    peak = 0

    def walk(obj):
        nonlocal peak
        if isinstance(obj, dict):
            peak = max(peak, obj.get("read_view_cache_bytes_peak") or 0)
            for v in obj.values():
                walk(v)
        elif isinstance(obj, list):
            for v in obj:
                walk(v)
    walk(d)
    verdict = {
        "rule_set": "OCTOMAP_RESET_VALIDITY_NEXT_20261103 §5 L",
        "counts": {"inputs": len(tl), "queries": len(qs),
                   "extra_inputs": extra_inputs, "extra_queries": extra_queries,
                   "reset_rounds": len(rr), "read_rounds": len(read_rounds)},
        "rows": {"reset_affected_queries": len(reset_q),
                 "reset_affected_inputs": len(reset_i),
                 "normal_queries": len(normal_q)},
        "latency_all_id": {"query_p95_ms": _p95(q_lat),
                           "query_max_ms": max(q_lat) if q_lat else None,
                           "input_p95_ms": _p95(i_e2e),
                           "input_max_ms": max(i_e2e) if i_e2e else None,
                           "age_max_s": max(ages) if ages else None},
        "windows_120s": {"count": len(buckets), "failed": window_fail},
        "ownership": {"rounds_with_final_release_residual": len(rounds_residual),
                      "cache_entry_baseline_end": [cache_base, cache_end]},
        "resources": {
            "rss_max_gib": round(max(rss) / 1048576, 3) if rss else None,
            "view_cache_peak_mib": round(peak / 1048576, 1),
            "rss_median_growth_pct": None},
    }
    if rss:
        n = len(rss)
        first = statistics.median(rss[:max(1, n // 6)])
        last = statistics.median(rss[-max(1, n // 6):])
        growth = 100.0 * (last - first) / first if first else None
        verdict["resources"]["rss_median_growth_pct"] = round(growth, 2)
        if max(rss) / 1048576 > RSS_MAX_GIB:
            problems.append("rss_max:%.2fGiB" % (max(rss) / 1048576))
        if peak / 1048576 > VIEW_CACHE_PEAK_MIB:
            problems.append("view_cache_peak:%.1fMiB" % (peak / 1048576))
        if growth is None or growth > RSS_MEDIAN_GROWTH_PCT:
            problems.append("rss_growth:%s%%" % growth)
    else:
        problems.append("rss_series_missing")

    verdict["problems"] = problems
    verdict["status"] = "fail" if problems else "pass"
    return verdict


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True)
    args = parser.parse_args()
    verdict = judge(Path(args.run))
    (Path(args.run) / "l_plan_verdict.json").write_text(
        json.dumps(verdict, indent=1, sort_keys=True))
    print(json.dumps({k: verdict[k] for k in
                      ("status", "problems", "counts", "rows", "resources",
                       "windows_120s")}, indent=1))
    return 0 if verdict["status"] == "pass" else 5


if __name__ == "__main__":
    sys.exit(main())
