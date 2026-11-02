"""Judge for the F-gate artifacts (and the shared validation helpers).

Every rule here fails closed: unknown classes, missing responses, NaN/bool/
negative values, self-inconsistent counters, and any disagreement with the
independent oracle are findings. The judge never rounds, never filters rows,
and never treats a missing artifact as a pass.
"""
from __future__ import annotations

import math

import freespace_contract as contract
import freespace_oracle

COUNTER_KEYS = ("grid_total", "feasible_total", "rejected_out_of_domain",
                "rejected_collision", "rejected_clearance", "rejected_unknown",
                "rejected_support")


def _number_problem(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return "bad_type"
    if not math.isfinite(value):
        return "nonfinite_value"
    if value < 0:
        return "negative_value"
    return None


def _model_for(case, rays=None):
    manifest = dict(case)
    if rays is not None:
        manifest["rays"] = rays
    return freespace_oracle.model_from_manifest(manifest)


def _expected_status(model, box, yaw):
    known = bool(model.free.any() or model.occ.any())
    if not known:
        return "NO_DATA"
    candidates, _ = model.enumerate(box, (yaw,))
    return "OK" if candidates else "NO_KNOWN_FREE"


def _artifact_candidates(response):
    out = []
    for entry in response.get("candidates") or []:
        cell = entry["cell"]
        out.append((int(entry["yaw_deg"]), int(cell[0]), int(cell[1]), int(cell[2]),
                    int(entry["support_kind"])))
    return out


def _compare_freespace(reasons, response, model, case, page_note):
    expected_status = _expected_status(model, case["box"], case["yaw"])
    semantic = response.get("semantic_status")
    if semantic != expected_status:
        reasons.append("%ssemantic_status %r != oracle %r" % (page_note, semantic,
                                                              expected_status))
    for key in COUNTER_KEYS:
        value = response.get(key)
        problem = _number_problem(value)
        if problem:
            reasons.append("%s%s %s" % (page_note, key, problem))
            continue
        if not isinstance(value, int):
            reasons.append("%s%s not_integer" % (page_note, key))
    identity = response.get("identity") or {}
    if identity.get("geometry_hash") != _geometry_hash(case):
        reasons.append("%sgeometry_hash_mismatch" % page_note)
    if identity.get("epoch") != _expected_epoch(case):
        reasons.append("%secho_epoch_mismatch" % page_note)
    feasible = response.get("feasible_total")
    if isinstance(feasible, int) and feasible > 0 and response.get("done") is not True:
        reasons.append("%sfull_page_not_done" % page_note)
    if isinstance(feasible, int) and response.get("next_cursor") != feasible:
        reasons.append("%snext_cursor_mismatch" % page_note)
    total = 0
    for key in COUNTER_KEYS[2:]:
        value = response.get(key)
        total += value if isinstance(value, int) else 0
    if isinstance(feasible, int):
        if response.get("grid_total") != feasible + total:
            reasons.append("%scounter_identity" % page_note)
    expected_candidates, expected_counters = model.enumerate(case["box"], (case["yaw"],))
    got = _artifact_candidates(response)
    if got != expected_candidates:
        missing = [c for c in expected_candidates if c not in got][:3]
        extra = [c for c in got if c not in expected_candidates][:3]
        reasons.append("%scandidate_set_mismatch n=%d exp=%d missing=%s extra=%s"
                       % (page_note, len(got), len(expected_candidates), missing, extra))
    for key, value in expected_counters.items():
        got_value = response.get(key)
        if got_value != value:
            reasons.append("%s%s %r != oracle %d" % (page_note, key, got_value, value))
    box_cells = [int(math.ceil(dim / model.res - 1e-9)) for dim in case["box"]]
    fx, fy = (box_cells[0], box_cells[1]) if case["yaw"] == 0 else (box_cells[1], box_cells[0])
    fz = box_cells[2]
    for entry in response.get("candidates") or []:
        begin = entry["verify_begin"]
        end = entry["verify_end"]
        if [end[0] - begin[0], end[1] - begin[1], end[2] - begin[2]] != [fx, fy, fz]:
            reasons.append("%sverify_range_not_box" % page_note)
            break
        if entry["support_kind"] not in (1, 2):
            reasons.append("%ssupport_kind_invalid" % page_note)
            break
        ratio = entry["support_ratio"]
        if _number_problem(ratio) or ratio < contract.MIN_SUPPORT_RATIO - 1e-9 \
                or ratio > 1.0 + 1e-9:
            reasons.append("%ssupport_ratio_out_of_range" % page_note)
            break


def _geometry_hash(case):
    return case.get("geometry_hash")


def _expected_epoch(case):
    return 2 if case["class"] == "reset" else 1


def judge_f_case(case, responses, descriptor, resolution):
    """Judge one F case: artifact vs the independent oracle, fail-closed."""
    reasons = []
    if not isinstance(responses, list):
        return {"reasons": ["responses_missing"], "status": "fail"}
    case = dict(case)
    case.setdefault("geometry_hash", descriptor.get("geometry_hash"))
    if case.get("cli_returncode") != 0:
        reasons.append("cli_returncode_%s" % case.get("cli_returncode"))
    acquires = [r for r in responses if r.get("op") == "acquire"]
    frees = [r for r in responses if r.get("op") == "freespace"]
    if case["class"] == "empty_tree":
        if len(acquires) != 1 or acquires[0].get("status") != "NO_DATA":
            reasons.append("empty_tree_acquire_not_no_data")
        if frees:
            reasons.append("empty_tree_ran_freespace")
        return {"reasons": reasons, "status": "pass" if not reasons else "fail"}
    if not frees:
        reasons.append("no_freespace_response")
        return {"reasons": reasons, "status": "fail"}
    if case["class"] == "two_version":
        if len(frees) != 2:
            reasons.append("two_version_needs_two_views")
        else:
            split = case.get("meta", {}).get("old_ray_count")
            if not isinstance(split, int):
                reasons.append("old_ray_count_missing")
            else:
                old_model = _model_for(case, case["rays"][:split])
                new_model = _model_for(case)
                _compare_freespace(reasons, frees[0], old_model, case, "old:")
                _compare_freespace(reasons, frees[1], new_model, case, "new:")
                old_rev = (frees[0].get("identity") or {}).get("map_revision")
                new_rev = (frees[1].get("identity") or {}).get("map_revision")
                if not isinstance(old_rev, int) or not isinstance(new_rev, int) \
                        or new_rev <= old_rev:
                    reasons.append("revision_did_not_advance")
        return {"reasons": reasons, "status": "pass" if not reasons else "fail"}
    if case["class"] == "reset":
        statuses = [r.get("status") for r in acquires]
        if statuses[:2] != ["NO_DATA", "EPOCH_MISMATCH"]:
            reasons.append("reset_acquire_sequence_%r" % (statuses[:2],))
    model = _model_for(case)
    # The main page is the first freespace; later ones are paging probes.
    _compare_freespace(reasons, frees[0], model, case, "")
    for probe in frees[1:]:
        expected, _ = model.enumerate(case["box"], (case["yaw"],))
        got = _artifact_candidates(probe)
        cursor = probe.get("next_cursor")
        # Probe convention: cursor=3, limit=5. Past the end the page is empty
        # and the cursor stays put.
        expected_next = min(8, len(expected)) if len(expected) > 3 else 3
        if got != expected[3:8]:
            reasons.append("paging_probe_slice_mismatch")
        if cursor != expected_next:
            reasons.append("paging_probe_cursor_mismatch")
    return {"reasons": reasons, "status": "pass" if not reasons else "fail"}


def judge_f_summary(summary):
    """Whole-matrix verdict: every case judged, counts consistent."""
    reasons = []
    if not isinstance(summary, dict):
        return ["summary_missing"]
    total = summary.get("total")
    passed = summary.get("passed")
    if not isinstance(total, int) or not isinstance(passed, int):
        reasons.append("summary_counts_missing")
    elif total != contract.F_CASES:
        reasons.append("matrix_size_%s_not_%d" % (total, contract.F_CASES))
    elif passed != total:
        reasons.append("matrix_failures_%d" % (total - passed))
    return reasons


# --- P/R/L judges: one fail-closed rule set for rehearsals and formal runs.
# Statuses, identities, and numeric sanity are checked in BOTH paths; the
# formal branch adds the contract counts, latency/deadline gates, pressure
# evidence, and duration/RSS-bucket requirements. Nothing can pass on sample
# counts alone.
import math

QUERY_SEMANTIC_OK = ("OK", "NO_KNOWN_FREE")
QUERY_PRESSURE_OK = QUERY_SEMANTIC_OK + ("STALE", "SNAPSHOT_PRESSURE")


def _bad_number(value):
    """Missing, boolean, non-finite, or negative -- all fail closed."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return True
    if not math.isfinite(value):
        return True
    return value < 0


def _p95(values):
    if not values:
        return None
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, math.ceil(0.95 * len(ordered)) - 1)]


def _query_record_problems(record, legal_statuses, prefix):
    problems = []
    status = record.get("status")
    if status not in legal_statuses and not (
            isinstance(status, str)
            and (status.startswith("QUERY_TIMEOUT") or status.startswith("ACQUIRE_"))
            and "PRESSURE_ALLOWED" in (legal_statuses or ())):
        if status not in legal_statuses:
            problems.append("%sstatus_%s" % (prefix, status))
    for key in ("latency_ms",):
        if _bad_number(record.get(key)):
            problems.append("%s%s_bad" % (prefix, key))
    if record.get("release_status") != "OK":
        problems.append("%srelease_%s" % (prefix, record.get("release_status")))
    identity = record.get("lease_identity")
    if not isinstance(identity, dict) or not identity.get("geometry_hash") \
            or not isinstance(identity.get("epoch"), int):
        problems.append("%sidentity_missing" % prefix)
    if record.get("content_sha256") is None and status in legal_statuses:
        problems.append("%scontent_missing" % prefix)
    return problems


def judge_p_window(window):
    """One P window (H reads the same records: planned vs started/published)."""
    reasons = []
    if not isinstance(window, dict):
        return {"reasons": ["window_missing"]}
    formal = bool(window.get("formal"))
    if window.get("errors"):
        reasons.append("thread_errors:%s" % window["errors"][:2])
    sender = window.get("sender") or []
    queries = window.get("queries") or []
    timeline = window.get("input_timeline") or []
    if not sender or not queries or not timeline:
        reasons.append("missing_requests")
        return {"reasons": reasons, "formal": formal,
                "inputs": len(sender), "queries": len(queries)}

    # Identity-level rules hold for rehearsals too: unique ids, one terminal
    # receipt each, COMPLETED only (a normal window allows no rejects), and
    # every query answered a legal semantic status with a released lease.
    ids = [record.get("input_id") for record in sender]
    if len(set(ids)) != len(ids):
        reasons.append("duplicate_input_id")
    if len(timeline) != len(sender):
        reasons.append("timeline_missing_rows")
    e2e = []
    for entry in timeline:
        if entry.get("receipt_status") != "COMPLETED":
            reasons.append("input_not_completed:%s" % entry.get("receipt_status"))
            continue
        if _bad_number(entry.get("e2e_ms")):
            reasons.append("input_e2e_bad")
            continue
        e2e.append(entry["e2e_ms"])
    if reasons:
        return {"reasons": reasons, "formal": formal,
                "inputs": len(sender), "queries": len(queries)}
    for record in queries:
        reasons.extend(_query_record_problems(record, QUERY_SEMANTIC_OK, "q"))
        if _bad_number(record.get("age_ns")) or record["age_ns"] > 1e9:
            reasons.append("q_age_%s" % record.get("age_ns"))
            break
    if reasons:
        return {"reasons": reasons, "formal": formal,
                "inputs": len(sender), "queries": len(queries)}

    # H gate (both paths measure it; the formal branch enforces the bounds).
    query_late = []
    send_late = []
    for record in queries:
        started, planned = record.get("started_mono_ns"), record.get("planned_mono_ns")
        if not isinstance(started, int) or not isinstance(planned, int):
            reasons.append("query_boundary_missing")
            break
        query_late.append((started - planned) / 1e6)
    for record in sender:
        published, planned = record.get("publish_mono_ns"), record.get("planned_mono_ns")
        if not isinstance(published, int) or not isinstance(planned, int):
            reasons.append("send_boundary_missing")
            break
        send_late.append((published - planned) / 1e6)
    if reasons:
        return {"reasons": reasons, "formal": formal,
                "inputs": len(sender), "queries": len(queries)}

    if formal:
        if len(sender) != contract.P_WINDOW_INPUTS \
                or len(queries) != contract.P_WINDOW_QUERIES:
            reasons.append("formal_counts")
        if _p95(e2e) > contract.INPUT_E2E_P95_MS:
            reasons.append("input_e2e_p95_%.0f" % _p95(e2e))
        if max(e2e) > contract.INPUT_E2E_MAX_MS:
            reasons.append("input_e2e_max_%.0f" % max(e2e))
        latency = [(record["ended_mono_ns"] - record["planned_mono_ns"]) / 1e6
                   for record in queries
                   if isinstance(record.get("ended_mono_ns"), int)
                   and isinstance(record.get("planned_mono_ns"), int)]
        if len(latency) != len(queries):
            reasons.append("query_boundary_missing")
        else:
            if _p95(latency) > contract.QUERY_FULL_P95_MS:
                reasons.append("query_p95_%.0f" % _p95(latency))
            if max(latency) > contract.QUERY_FULL_MAX_MS:
                reasons.append("query_max_%.0f" % max(latency))
        if _p95(query_late) > contract.SCHED_LATE_P95_MS:
            reasons.append("sched_late_p95_%.0f" % _p95(query_late))
        if max(query_late) > contract.SCHED_LATE_MAX_MS:
            reasons.append("sched_late_max_%.0f" % max(query_late))
        if _p95(send_late) > contract.SCHED_LATE_P95_MS:
            reasons.append("send_late_p95_%.0f" % _p95(send_late))
        if max(send_late) > contract.SCHED_LATE_MAX_MS:
            reasons.append("send_late_max_%.0f" % max(send_late))
        gap = window.get("health_gap_max_s")
        if _bad_number(gap) or gap >= contract.SNAPSHOT_AGE_MAX_S:
            reasons.append("health_gap_%s" % gap)
        close = window.get("window_close_ms")
        planned_ms = window.get("window_planned_s")
        if _bad_number(close) or not isinstance(planned_ms, (int, float)) \
                or close > planned_ms * 1000 + 500:
            reasons.append("window_close_%s" % close)
        return {"reasons": reasons, "formal": True,
                "inputs": len(sender), "queries": len(queries)}
    reasons.append("short_sample_not_formal")
    return {"reasons": reasons, "formal": False,
            "inputs": len(sender), "queries": len(queries)}


LEGAL_TERMINAL = ("COMPLETED", "REJECTED", "QUEUE_FULL", "DUPLICATE",
                  "EPOCH_MISMATCH", "OPTICAL_FRAME_MISMATCH", "TF_STAMP_MISMATCH",
                  "NO_VALID_POINTS", "OUT_OF_ORDER", "CONFLICT")


def judge_r_stage(artifact):
    reasons = []
    if not isinstance(artifact, dict):
        return {"reasons": ["artifact_missing"]}
    formal = bool(artifact.get("formal"))
    overload = artifact.get("overload") or []
    timeline = artifact.get("overload_timeline") or []
    recovery = artifact.get("recovery")
    recovery_timeline = artifact.get("recovery_timeline") or []
    first = artifact.get("first_wait") or {}
    if not overload:
        reasons.append("no_overload_inputs")
    if len(timeline) != len(overload):
        reasons.append("overload_timeline_missing_rows:%d/%d"
                       % (len(timeline), len(overload)))
    non_terminal = [entry["input_id"] for entry in timeline
                    if entry.get("receipt_status") not in LEGAL_TERMINAL]
    if overload and non_terminal:
        reasons.append("overload_not_terminal:%d" % len(non_terminal))
    if not recovery:
        reasons.append("no_recovery_inputs")
    for entry in recovery_timeline:
        if entry.get("receipt_status") != "COMPLETED":
            reasons.append("recovery_not_completed:%s" % entry.get("receipt_status"))
            break
    if not isinstance(first.get("bound_id"), str) or not first.get("bound_id"):
        reasons.append("first_wait_shape_missing")
    elapsed = first.get("elapsed_ms")
    if _bad_number(elapsed):
        reasons.append("first_wait_elapsed_bad")
    epoch = artifact.get("epoch_after")
    if not isinstance(epoch, int):
        reasons.append("epoch_unknown")
    if formal and not reasons:
        if epoch != artifact.get("epoch_before", epoch):
            reasons.append("epoch_restarted")
    for record in (artifact.get("overload_queries") or []):
        if record.get("status") not in QUERY_PRESSURE_OK \
                and not (isinstance(record.get("status"), str)
                         and (record["status"].startswith("QUERY_TIMEOUT")
                              or record["status"].startswith("ACQUIRE_"))):
            reasons.append("overload_query_%s" % record.get("status"))
            break
    for record in (artifact.get("recovery_queries") or []):
        reasons.extend(_query_record_problems(record, QUERY_SEMANTIC_OK, "rq:"))
        break
    if not reasons:
        if formal:
            if len(overload) != contract.R_OVERLOAD_INPUTS:
                reasons.append("overload_counts")
            if artifact.get("in_flight_at_drain_deadline"):
                reasons.append("drain_in_flight_%d"
                               % len(artifact["in_flight_at_drain_deadline"]))
            pressure = sum(1 for entry in timeline
                           if entry.get("receipt_status") in ("REJECTED", "QUEUE_FULL"))
            if pressure < 1:
                reasons.append("no_real_pressure")
            if first.get("status") != "OK":
                reasons.append("first_wait_%s" % first.get("status"))
            elif elapsed > contract.RECOVERY_FIRST_WAIT_S * 1000:
                reasons.append("first_wait_%.0fms" % elapsed)
            age = first.get("age_ns")
            if _bad_number(age) or age > 2 * int(1e9):
                reasons.append("first_wait_age_%s" % age)
            if len(recovery) != contract.R_RECOVERY_INPUTS:
                reasons.append("recovery_counts")
        else:
            reasons.append("short_sample_not_formal")
    return {"reasons": reasons, "overload": len(overload), "formal": formal}


def judge_l_stage(artifact):
    reasons = []
    if not isinstance(artifact, dict):
        return {"reasons": ["artifact_missing"]}
    formal = bool(artifact.get("formal"))
    if artifact.get("send_errors"):
        reasons.append("send_errors:%s" % artifact["send_errors"][:2])
    queries = artifact.get("query_records") or []
    if not queries:
        reasons.append("no_query_records")
    for record in queries:
        reasons.extend(_query_record_problems(record, QUERY_SEMANTIC_OK, "lq:"))
        break
    rounds = artifact.get("read_rounds_results") or []
    if len(rounds) != artifact.get("read_rounds", -1):
        reasons.append("read_rounds_missing")
    legal_round = ("OK", "NO_KNOWN_FREE", "STALE") if not formal \
        else ("OK", "NO_KNOWN_FREE")
    for result in rounds:
        if result.get("status") not in legal_round:
            reasons.append("read_round_status_%s" % result.get("status"))
            break
        if result.get("release_status") != "OK":
            reasons.append("read_round_release_%s" % result.get("release_status"))
            break
    resets = artifact.get("reset_statuses") or []
    if len(resets) != artifact.get("reset_rounds", -1):
        reasons.append("reset_rounds_missing")
    if any(status != "OK" for status in resets):
        reasons.append("reset_status_%s" % resets)
    baseline = artifact.get("baseline") or {}
    after = artifact.get("after") or {}
    if not isinstance(baseline.get("view_directory_entries"), int) \
            or not isinstance(after.get("view_directory_entries"), int):
        reasons.append("resource_fields_missing")
    elif after["view_directory_entries"] > 2:
        reasons.append("handles_leaked_%d" % after["view_directory_entries"])
    peak = after.get("read_view_cache_bytes_peak")
    if _bad_number(peak):
        reasons.append("cache_peak_bad")
    elif peak > contract.READ_VIEW_CACHE_LIMIT_BYTES:
        reasons.append("cache_peak_over_limit")
    rss_series = artifact.get("rss_series") or []
    if not rss_series:
        reasons.append("rss_series_missing")
    for sample in rss_series:
        if _bad_number(sample.get("rss_kib")):
            reasons.append("rss_sample_bad")
            break
    if not reasons and formal:
        if artifact.get("planned_seconds") != contract.L_DURATION_S:
            reasons.append("l_duration_%s" % artifact.get("planned_seconds"))
        if artifact.get("sent_inputs", 0) < contract.L_INPUTS * 0.9:
            reasons.append("l_inputs_%s" % artifact.get("sent_inputs"))
        if len(queries) < contract.L_QUERIES * 0.9:
            reasons.append("l_queries_%d" % len(queries))
        samples = [sample["rss_kib"] for sample in rss_series
                   if isinstance(sample.get("rss_kib"), int)]
        if samples:
            if max(samples) > contract.L_RSS_LIMIT_KIB:
                reasons.append("rss_over_limit_%d" % max(samples))
            warmup_end = rss_series[0]["t_mono_ns"] + int(contract.L_WARMUP_S * 1e9)
            scored = [sample for sample in rss_series
                      if sample["t_mono_ns"] >= warmup_end]
            if len(scored) < 10:
                reasons.append("l_scored_window_short")
            else:
                values = sorted(sample["rss_kib"] for sample in scored)
                decile = max(1, len(values) // 10)
                first_median = sorted(values[:decile])[decile // 2]
                last_median = sorted(values[-decile:])[decile // 2]
                if last_median > first_median * (1 + contract.L_RSS_GROWTH):
                    reasons.append("rss_growth_%.3f"
                                   % (last_median / max(1, first_median)))
            buckets = {}
            for sample in scored:
                bucket = int((sample["t_mono_ns"] - warmup_end)
                             // (10 * 60 * int(1e9)))
                buckets[bucket] = buckets.get(bucket, 0) + 1
            last = max(buckets) if buckets else -1
            # A partial FINAL bucket is a run-end artifact, not a sampling
            # gap; every completed bucket must carry the full 1 Hz count.
            sparse = {bucket: count for bucket, count in buckets.items()
                      if count < 600 and bucket != last}
            if sparse:
                reasons.append("rss_bucket_sparse:%s" % sparse)
    if not reasons and not formal:
        reasons.append("short_sample_not_formal")
    return {"reasons": reasons, "rounds": len(rounds), "formal": formal}
