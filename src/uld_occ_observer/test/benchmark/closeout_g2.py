"""Independent closeout judge for the frozen O6/O7 evidence.

This module rereads original sample arrays. It does not call the historical
verdict objects for a pass, and it does not change the product or the gates.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import fusion_benchmark
import fusion_report
import o6_matrix

# Copied from the frozen O6/O7 plan. RSS is not a substitute for read-view bytes.
CLOSEOUT_GATES = {
    "integrate_30k_p95_ms": 500.0,
    "integrate_30k_max_ms": 1000.0,
    "integrate_100k_p95_ms": 2000.0,
    "integrate_100k_max_ms": 5000.0,
    "query_p95_ms": 100.0,
    "query_max_ms": 250.0,
    "init_limit_s": 30.0,
    "rss_limit_kib": 2 * 1024 * 1024,
    "queue_limit_ms": 1000.0,
    "health_gap_s": 1.0,
    "normal_inputs": 240,
    "normal_queries": 120,
    "normal_duration_s": 120.0,
    "overload_inputs": 600,
    "recovery_inputs": 60,
    "recovery_queries": 30,
    "drain_s": 30.0,
    "scored_samples": 30,
    "warmup_samples": 3,
}
REQUIRED_FAIL_ID = "r01_b00_cuboid_c30000_o50_x50"
HISTORICAL_FAIL_P95_MS = 517.297808


def gates_match_reference():
    reference = fusion_report.REFERENCE_GATES
    return (
        CLOSEOUT_GATES["integrate_30k_p95_ms"] == reference["integrate_30k"]["p95_ms"] == 500.0
        and CLOSEOUT_GATES["integrate_30k_max_ms"] == reference["integrate_30k"]["max_ms"] == 1000.0
        and CLOSEOUT_GATES["integrate_100k_p95_ms"] == reference["integrate_100k"]["p95_ms"] == 2000.0
        and CLOSEOUT_GATES["integrate_100k_max_ms"] == reference["integrate_100k"]["max_ms"] == 5000.0
        and CLOSEOUT_GATES["query_p95_ms"] == reference["query"]["p95_ms"] == 100.0
        and CLOSEOUT_GATES["query_max_ms"] == reference["query"]["max_ms"] == 250.0
    )


def p95_index(count):
    """0-based index of the ceil(0.95*n)th ascending sample."""
    if count <= 0:
        raise ValueError("empty sample")
    return math.ceil(0.95 * count) - 1


def assign_sources(plan_ids, kept, measure):
    """Kept and measured lists must be a partition of the plan. No fill-in."""
    plan = list(plan_ids)
    plan_set = set(plan)
    kept_set = set(kept)
    measure_set = set(measure)
    duplicate = sorted(kept_set & measure_set)
    missing = sorted(plan_set - kept_set - measure_set)
    extra = sorted((kept_set | measure_set) - plan_set)
    repeated = sorted([item for item in plan if plan.count(item) > 1])
    ok = not duplicate and not missing and not extra and not repeated and len(plan) == 144
    sources = {}
    for group_id in plan_set:
        if group_id in duplicate or group_id in missing:
            sources[group_id] = None
        elif group_id in kept_set:
            sources[group_id] = "run01"
        elif group_id in measure_set:
            sources[group_id] = "run03"
    return {
        "partition_ok": ok,
        "duplicate": duplicate,
        "missing": missing,
        "extra": extra,
        "sources": sources,
    }


def _sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_rows(path):
    return fusion_benchmark.complete_documents(Path(path).read_text())


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _gate(values, expected, p95_limit, max_limit):
    stats = fusion_report.exact_stats(values, expected)
    return {
        "n": stats.get("n"),
        "expected_n": expected,
        "status": stats.get("status"),
        "median": stats.get("median"),
        "p95": stats.get("p95"),
        "max": stats.get("max"),
        "limit_p95_ms": p95_limit,
        "limit_max_ms": max_limit,
        "pass": fusion_report.within_gate(stats, p95_limit, max_limit),
        "p95_rank": "sorted[ceil(0.95*n)-1]" if stats.get("status") == "complete" else None,
    }


def _integrate_limits(cloud):
    if cloud >= 100000:
        return CLOSEOUT_GATES["integrate_100k_p95_ms"], CLOSEOUT_GATES["integrate_100k_max_ms"]
    return CLOSEOUT_GATES["integrate_30k_p95_ms"], CLOSEOUT_GATES["integrate_30k_max_ms"]


def score_rows(rows, cloud, expected):
    """Score one array. Fewer than expected samples cannot pass."""
    if len(rows) != expected:
        return {"status": "not_evaluated", "reason": "sample_count", "n": len(rows), "expected_n": expected}
    p95_limit, max_limit = _integrate_limits(cloud)
    query_p95 = CLOSEOUT_GATES["query_p95_ms"]
    query_max = CLOSEOUT_GATES["query_max_ms"]
    gates = {
        "integrate_external_ms": _gate(
            [row["integrate_external_ms"] for row in rows], expected, p95_limit, max_limit),
        "acquire_ms": _gate([row["acquire_ms"] for row in rows], expected, query_p95, query_max),
        "roi_ms": _gate([row["roi_ms"] for row in rows], expected, query_p95, query_max),
        "surface_first_ms": _gate(
            [row["surface_first_call"]["external_ms"] for row in rows], expected, query_p95, query_max),
        "surface_second_ms": _gate(
            [row["surface_second_call"]["external_ms"] for row in rows], expected, query_p95, query_max),
    }
    return {"status": "complete", "gates": gates, "pass": all(item["pass"] for item in gates.values())}


def _startup_finite(values):
    if not isinstance(values, list) or len(values) != CLOSEOUT_GATES["scored_samples"]:
        return False
    limit = CLOSEOUT_GATES["init_limit_s"]
    return all(_finite(value) and 0 <= value <= limit for value in values)


def _identity_ok(rows, manifest):
    by_index = {frame["index"]: frame for frame in manifest["frames"]}
    if len(rows) != len(manifest["frames"]):
        return False, "row_count"
    for row in rows:
        frame = by_index.get(row.get("index"))
        if frame is None:
            return False, "missing_index"
        if row.get("input_sha256") != frame.get("file_sha256"):
            return False, "input_sha256"
        if bool(row.get("warmup")) != bool(frame.get("warmup")):
            return False, "warmup_flag"
        receipt = row.get("receipt") or {}
        if row.get("status") != "OK" or receipt.get("status") != "INTEGRATED":
            return False, "receipt_status"
    return True, "input_sha256 matches load_manifest"


def judge_group(group_dir, cloud, source_run, versions):
    group_dir = Path(group_dir)
    verdict_path = group_dir / "group_verdict.json"
    manifest_path = group_dir / "load" / "load_manifest.json"
    warm_path = group_dir / "warm" / "samples.yaml"
    historical = json.loads(verdict_path.read_text()) if verdict_path.exists() else {}
    if not manifest_path.exists() or not warm_path.exists():
        return _incomplete(group_dir, source_run, versions, "missing warm samples or load manifest")
    manifest = json.loads(manifest_path.read_text())
    warm_rows = _load_rows(warm_path)
    identity_ok, identity_reason = _identity_ok(warm_rows, manifest)
    scored = [row for row in warm_rows if not row.get("warmup")]
    warmup = [row for row in warm_rows if row.get("warmup")]
    cold_rows = []
    cold_dirs = sorted((group_dir / "cold").glob("c*/samples.yaml")) if (group_dir / "cold").exists() else []
    for path in cold_dirs:
        docs = _load_rows(path)
        if len(docs) != 1:
            return _incomplete(group_dir, source_run, versions, "cold sample is not one document: %s" % path.name)
        cold_rows.append(docs[0])
    startup = historical.get("cold_startup_to_first_sample_s")
    startup_ok = _startup_finite(startup)
    tool_void = isinstance(startup, list) and any(value is None for value in startup)
    warm_score = score_rows(scored, cloud, CLOSEOUT_GATES["scored_samples"])
    cold_score = score_rows(cold_rows, cloud, CLOSEOUT_GATES["scored_samples"])
    rss_values = [row.get("process_peak_rss_kib") for row in scored + cold_rows]
    rss_known = all(_finite(value) and value > 0 for value in rss_values)
    rss_pass = rss_known and max(rss_values) <= CLOSEOUT_GATES["rss_limit_kib"]
    query_pass = (
        warm_score.get("status") == "complete"
        and cold_score.get("status") == "complete"
        and all(warm_score["gates"][name]["pass"] for name in ("acquire_ms", "roi_ms", "surface_first_ms", "surface_second_ms"))
        and all(cold_score["gates"][name]["pass"] for name in ("acquire_ms", "roi_ms", "surface_first_ms", "surface_second_ms"))
    )
    integrate_pass = warm_score.get("status") == "complete" and warm_score["gates"]["integrate_external_ms"]["pass"]
    reasons = []
    if not identity_ok:
        reasons.append("input_identity:" + identity_reason)
    if len(warmup) != CLOSEOUT_GATES["warmup_samples"]:
        reasons.append("warmup_count")
    if tool_void or not startup_ok:
        reasons.append("startup_poll_null" if tool_void else "startup_not_finite")
    if warm_score.get("status") != "complete" or cold_score.get("status") != "complete":
        reasons.append("sample_count")
    elif not integrate_pass:
        reasons.append("integrate_external_ms")
    if warm_score.get("status") == "complete" and cold_score.get("status") == "complete" and not query_pass:
        reasons.append("query")
    if not rss_known:
        reasons.append("rss_missing")
    elif not rss_pass:
        reasons.append("rss")
    if reasons:
        status = "not_evaluated" if any(item in reasons for item in (
            "sample_count", "startup_poll_null", "startup_not_finite", "warmup_count", "rss_missing")) or any(
            item.startswith("input_identity") for item in reasons) else "fail"
    else:
        status = "pass"
    historical_p95 = None
    try:
        historical_p95 = historical["warm_latency"]["gates"][0]["stats"]["p95"]
    except (KeyError, IndexError, TypeError):
        historical_p95 = None
    recomputed_p95 = None
    if warm_score.get("status") == "complete":
        recomputed_p95 = warm_score["gates"]["integrate_external_ms"]["p95"]
    return {
        "group_id": group_dir.name,
        "source_run": source_run,
        "source_path": str(group_dir),
        "product_revision": versions.get("product_revision"),
        "tool_revision": versions.get("tool_revision"),
        "profile_cli_sha256": versions.get("profile_cli_sha256"),
        "eval_native_sha256": versions.get("eval_native_sha256"),
        "generator_version": manifest.get("generator_version"),
        "input_manifest_sha256": _sha256(manifest_path),
        "warm_samples_sha256": _sha256(warm_path),
        "config_argv": str(group_dir / "warm" / "argv.json"),
        "historical_status": historical.get("status"),
        "validity": _validity(source_run, startup_ok, tool_void, identity_ok, identity_reason),
        "void_relation": "startup_poll_null_not_a_product_failure" if tool_void else "none",
        "warmup_n": len(warmup),
        "scored_n": len(scored),
        "cold_n": len(cold_rows),
        "warmup_boundary": "warmup true excluded; scored frames only",
        "unit": "ms",
        "recomputed": warm_score,
        "cold_query_recomputed": {
            "status": cold_score.get("status"),
            "integrate_gate": "not_applied",
            "gates": None if cold_score.get("status") != "complete" else {
                name: cold_score["gates"][name] for name in (
                    "acquire_ms", "roi_ms", "surface_first_ms", "surface_second_ms")},
        },
        "historical_integrate_p95_ms": historical_p95,
        "recomputed_integrate_p95_ms": recomputed_p95,
        "p95_delta_ms": None if historical_p95 is None or recomputed_p95 is None else recomputed_p95 - historical_p95,
        "peak_rss_kib": max(rss_values) if rss_known else None,
        "rss_pass": rss_pass if rss_known else None,
        "read_view_and_cache_bytes": {
            "status": "not_evaluated",
            "reason": "frozen interface does not attribute read-view or cache bytes; RSS is not a substitute",
        },
        "closeout_status": status,
        "reasons": reasons,
    }


def _validity(source_run, startup_ok, tool_void, identity_ok, identity_reason):
    if tool_void:
        return "void: cold startup list contains null from the missed 50 ms poll; not a product failure"
    if not startup_ok:
        return "startup array is not 30 finite samples within 30 s"
    if not identity_ok:
        return "input identity failed: " + identity_reason
    if source_run == "run01":
        return (
            "kept from run01 because cold_startup_to_first_sample_s has 30 finite values, "
            "none null, each <= 30 s; the startup-poll defect does not apply; "
            "warm input hashes match load_manifest"
        )
    return "measured in run03 after the startup-poll fix; warm input hashes match load_manifest; " + identity_reason


def _incomplete(group_dir, source_run, versions, reason):
    return {
        "group_id": group_dir.name,
        "source_run": source_run,
        "source_path": str(group_dir),
        "product_revision": versions.get("product_revision"),
        "tool_revision": versions.get("tool_revision"),
        "closeout_status": "not_evaluated",
        "reasons": [reason],
        "read_view_and_cache_bytes": {"status": "not_evaluated"},
        "validity": reason,
    }


def _versions(evidence, run_name):
    path = Path(evidence) / run_name / "VERSIONS.json"
    payload = json.loads(path.read_text())
    profile = payload.get("profile_cli_sha256")
    if profile is None and isinstance(payload.get("profile_cli"), dict):
        profile = payload["profile_cli"].get("sha256")
    native = payload.get("eval_native_sha256")
    if native is None and isinstance(payload.get("eval_native"), dict):
        native = payload["eval_native"].get("sha256")
    payload["profile_cli_sha256"] = profile
    payload["eval_native_sha256"] = native
    return payload


def judge_o6(evidence, assignment):
    evidence = Path(evidence)
    rows = []
    versions = {name: _versions(evidence, name) for name in ("run01", "run03")}
    for group_id in o6_matrix_ids():
        source = assignment["sources"].get(group_id)
        cloud = 100000 if "_c100000_" in group_id else 30000
        if source is None or not assignment["partition_ok"]:
            rows.append({
                "group_id": group_id,
                "source_run": source,
                "closeout_status": "not_evaluated",
                "reasons": ["source_partition"],
                "validity": "116 and 28 are not a proven partition of the 144",
                "read_view_and_cache_bytes": {"status": "not_evaluated"},
            })
            continue
        rows.append(judge_group(evidence / source / "o6" / "groups" / group_id, cloud, source, versions[source]))
    required = next((row for row in rows if row["group_id"] == REQUIRED_FAIL_ID), None)
    latency_fail = [row["group_id"] for row in rows if row["closeout_status"] == "fail"]
    not_evaluated = [row["group_id"] for row in rows if row["closeout_status"] == "not_evaluated"]
    resource_complete = all(
        row.get("read_view_and_cache_bytes", {}).get("status") == "pass" for row in rows)
    return {
        "groups": len(rows),
        "partition_ok": assignment["partition_ok"],
        "duplicate": assignment["duplicate"],
        "missing": assignment["missing"],
        "extra": assignment["extra"],
        "pass_groups": sum(row["closeout_status"] == "pass" for row in rows),
        "fail_groups": latency_fail,
        "not_evaluated_groups": not_evaluated,
        "required_failure_retained": bool(required and required["closeout_status"] == "fail"),
        "required_failure_p95_ms": None if required is None else required.get("recomputed_integrate_p95_ms"),
        "required_failure_historical_p95_ms": HISTORICAL_FAIL_P95_MS,
        "required_failure_p95_delta_ms": None if required is None else required.get("p95_delta_ms"),
        "resource_coverage": "complete" if resource_complete else "incomplete",
        "read_view_and_cache_bytes": "not_evaluated",
        "full_matrix_pass": False if latency_fail or not_evaluated or not resource_complete else True,
        "pooled_latency": "not_computed",
        "rows": rows,
    }


def o6_matrix_ids():
    scenes = []
    for hull in ("chamfer", "cuboid"):
        for resolution in (0.01, 0.02, 0.05):
            for boxes in (0, 5, 20):
                scenes.append({
                    "resolution": resolution, "boxes": boxes, "hull": hull,
                    "geometry_file": "unused", "config_file": "unused",
                })
    return [group["id"] for group in o6_matrix.official_matrix(scenes)]


def _phase_ids(ledger, phase):
    return [input_id for input_id, item in ledger["sent"].items() if item["phase"] == phase]


def _terminal(ledger, input_id):
    sent = ledger["sent"][input_id]
    items = ledger["receipts"].get(input_id) or []
    if not items:
        return {
            "input_id": input_id,
            "planned_s": sent["planned"],
            "sent_actual": sent["actual"],
            "receipts": 0,
            "status": None,
            "last_observed": "sent_no_receipt",
            "completion": "unknown",
        }
    first = items[0]
    return {
        "input_id": input_id,
        "planned_s": sent["planned"],
        "sent_actual": sent["actual"],
        "receipts": len(items),
        "status": first["status"],
        "queue_ms": first.get("queue_ms"),
        "total_ms": first.get("total_ms"),
        "received_at": first.get("received_at"),
        "last_observed": "receipt:" + str(first["status"]),
        "completion": "observed" if first["status"] == "INTEGRATED" else "rejected",
    }


def judge_normal_window(ledger):
    score_ids = _phase_ids(ledger, "score")
    terminals = [_terminal(ledger, input_id) for input_id in score_ids]
    queries = ledger.get("queries") or []
    success = [item for item in terminals if item["completion"] == "observed"]
    integrate_values = [item["total_ms"] for item in success if _finite(item.get("total_ms"))]
    all_integrate = [item.get("total_ms") for item in terminals]
    surface_values = [item.get("surface_ms") for item in queries]
    reasons = []
    if len(score_ids) != len(set(score_ids)):
        reasons.append("duplicate_input_id")
    if len(score_ids) != CLOSEOUT_GATES["normal_inputs"]:
        reasons.append("input_count")
    pending = [item["input_id"] for item in terminals if item["completion"] == "unknown"]
    rejected = [item for item in terminals if item["completion"] == "rejected"]
    duplicates = sum(max(0, item["receipts"] - 1) for item in terminals)
    if pending or rejected or duplicates or len(success) != CLOSEOUT_GATES["normal_inputs"]:
        reasons.append("input_conservation")
    if len(queries) != CLOSEOUT_GATES["normal_queries"] or any(item.get("status") != "OK" for item in queries):
        reasons.append("query_conservation")
    integrate = fusion_report.exact_stats(integrate_values, CLOSEOUT_GATES["normal_inputs"])
    if not fusion_report.within_gate(
            integrate, CLOSEOUT_GATES["integrate_30k_p95_ms"], CLOSEOUT_GATES["integrate_30k_max_ms"]):
        reasons.append("integrate")
    surface = _query_gate(surface_values, "surface_ms")
    acquire = _query_gate([item.get("acquire_ms") for item in queries], "acquire_ms")
    roi = _query_gate([item.get("roi_ms") for item in queries], "roi_ms")
    for gate in (surface, acquire, roi):
        if not gate["pass"]:
            reasons.append(gate["metric"])
    queue_values = [item.get("queue_ms") for item in success]
    if not queue_values or max(queue_values) > CLOSEOUT_GATES["queue_limit_ms"]:
        reasons.append("queue_age")
    client = []
    for item in success:
        if _finite(item.get("received_at")) and _finite(item.get("sent_actual")):
            client.append((item["received_at"] - item["sent_actual"]) * 1000.0)
    client_stats = fusion_report.exact_stats(client, CLOSEOUT_GATES["normal_inputs"]) if len(client) == len(success) else {
        "status": "incomplete", "n": len(client)}
    mean_service = None
    if integrate.get("status") == "complete" and integrate["n"]:
        mean_service = sum(integrate_values) / len(integrate_values)
    return {
        "pass": not reasons,
        "reasons": reasons,
        "sent": len(score_ids),
        "success": len(success),
        "rejected": len(rejected),
        "pending": pending,
        "duplicate_receipts": duplicates,
        "integrate_all_requests": {"n": len(all_integrate), "note": "failures stay in the denominator"},
        "integrate_success_subset": integrate,
        "mean_service_ms": mean_service,
        "estimated_hz_from_mean_service": None if not mean_service else 1000.0 / mean_service,
        "estimated_hz_label": "estimate_not_measured_throughput",
        "scheduled_completion_hz": len(success) / CLOSEOUT_GATES["normal_duration_s"] if not pending and not rejected else None,
        "surface": surface,
        "acquire": acquire,
        "roi": roi,
        "queue_max_ms": max(queue_values) if queue_values else None,
        "client_latency_ms": client_stats,
        "snapshot_age": "not_in_normal_ledger",
    }


def _query_gate(values, metric):
    if any(not _finite(value) for value in values):
        return {"metric": metric, "pass": False, "status": "incomplete", "n": len(values)}
    stats = fusion_report.exact_stats(values, CLOSEOUT_GATES["normal_queries"])
    return {
        "metric": metric,
        "pass": fusion_report.within_gate(stats, CLOSEOUT_GATES["query_p95_ms"], CLOSEOUT_GATES["query_max_ms"]),
        **stats,
    }


def judge_overload(ledger):
    overload_ids = _phase_ids(ledger, "overload")
    recovery_ids = _phase_ids(ledger, "recovery")
    overload = [_terminal(ledger, input_id) for input_id in overload_ids]
    recovery = [_terminal(ledger, input_id) for input_id in recovery_ids]
    pending = [item for item in overload if item["completion"] == "unknown"]
    recovery_queries = ledger.get("recovery_queries") or []
    stale = [item for item in recovery_queries if item.get("status") == "STALE"]
    reasons = []
    if len(overload_ids) != CLOSEOUT_GATES["overload_inputs"]:
        reasons.append("overload_input_count")
    if pending or any(item["receipts"] > 1 for item in overload):
        reasons.append("overload_accounting")
    if len(recovery_ids) != CLOSEOUT_GATES["recovery_inputs"] or any(
            item["completion"] != "observed" or item["receipts"] != 1 for item in recovery):
        reasons.append("recovery_inputs")
    if len(recovery_queries) != CLOSEOUT_GATES["recovery_queries"] or any(
            item.get("status") != "OK" for item in recovery_queries):
        reasons.append("recovery_queries")
    contract_allows_stale = False
    return {
        "status": "pass" if not reasons else "fail",
        "reasons": reasons,
        "overload_sent": len(overload_ids),
        "overload_observed_complete": sum(item["completion"] == "observed" for item in overload),
        "overload_rejected": sum(item["completion"] == "rejected" for item in overload),
        "pending_unfinished": pending,
        "pending_count": len(pending),
        "recovery_inputs": {
            "sent": len(recovery_ids),
            "observed_complete": sum(item["completion"] == "observed" for item in recovery),
        },
        "recovery_queries": {
            "n": len(recovery_queries),
            "first_status": recovery_queries[0]["status"] if recovery_queries else None,
            "stale_count": len(stale),
            "ok_count": sum(item.get("status") == "OK" for item in recovery_queries),
            "contract_allows_stale": contract_allows_stale,
            "contract_note": "recovery requests must complete normally; a later OK does not erase STALE",
        },
        "deadlines": {
            "send_cutoff": "overload planned schedule",
            "drain_cutoff_s": CLOSEOUT_GATES["drain_s"],
            "ledger_close": "file written after recovery drain; no per-id queue snapshot was stored",
        },
    }


def load_selected(evidence):
    payload = json.loads((Path(evidence) / "run03" / "selected_groups.json").read_text())
    return payload["kept_from_run01"], payload["measure"]


def write_reports(evidence, out):
    evidence = Path(evidence)
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    if not gates_match_reference():
        raise RuntimeError("closeout gates do not match the frozen reference")
    kept, measure = load_selected(evidence)
    assignment = assign_sources(o6_matrix_ids(), kept, measure)
    o6 = judge_o6(evidence, assignment)
    windows = {}
    o7_root = evidence / "run03" / "o7"
    for name in (
        "normal_b00_r0", "normal_b00_r1", "normal_b00_r2",
        "normal_b05_r0", "normal_b05_r1", "normal_b05_r2",
        "normal_b20_r0", "normal_b20_r1", "normal_b20_r2",
    ):
        ledger = json.loads((o7_root / name / "ledger.json").read_text())
        windows[name] = judge_normal_window(ledger)
    overload_ledger = json.loads((o7_root / "overload" / "ledger.json").read_text())
    overload = judge_overload(overload_ledger)
    o6_path = out / "GROUP_LEDGER.jsonl"
    with o6_path.open("w") as handle:
        for row in o6["rows"]:
            handle.write(json.dumps(row, sort_keys=True) + "\n")
    window_doc = {"normal": windows, "overload": {key: value for key, value in overload.items() if key != "pending_unfinished"}}
    (out / "WINDOW_LEDGER.json").write_text(json.dumps(window_doc, indent=2) + "\n")
    (out / "PENDING_IDS.json").write_text(json.dumps({
        "count": overload["pending_count"],
        "completion": "unknown",
        "last_observed": "sent_no_receipt",
        "ids": overload["pending_unfinished"],
    }, indent=2) + "\n")
    summary = {key: value for key, value in o6.items() if key != "rows"}
    (out / "O6_JUDGE.json").write_text(json.dumps(summary, indent=2) + "\n")
    (out / "O7_JUDGE.json").write_text(json.dumps({
        "normal_pass": [name for name, item in windows.items() if item["pass"]],
        "normal_fail": {name: item["reasons"] for name, item in windows.items() if not item["pass"]},
        "overload_status": overload["status"],
        "overload_reasons": overload["reasons"],
        "pending_count": overload["pending_count"],
        "recovery_first_status": overload["recovery_queries"]["first_status"],
        "stale_allowed": False,
    }, indent=2) + "\n")
    return {"o6": summary, "windows": windows, "overload": overload}


def main(argv):
    import argparse
    parser = argparse.ArgumentParser(description="Recompute frozen O6/O7 evidence. Does not rescore by rerunning the matrix.")
    parser.add_argument("--evidence", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args(argv)
    summary = write_reports(args.evidence, args.out)
    print("o6_full_matrix_pass", summary["o6"]["full_matrix_pass"])
    print("o6_fail", ",".join(summary["o6"]["fail_groups"]))
    print("o6_required_p95", summary["o6"]["required_failure_p95_ms"])
    print("o7_fail", ",".join(summary["windows"] and [name for name, item in summary["windows"].items() if not item["pass"]]))
    print("overload", summary["overload"]["status"], summary["overload"]["pending_count"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main(__import__("sys").argv[1:]))
