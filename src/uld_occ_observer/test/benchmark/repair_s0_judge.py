"""Per-gate judge for synthetic and future ledgers.

Errors and timeouts stay in the denominator. One failed window or group is not
averaged away. Missing resource attribution cannot pass.
"""
from __future__ import annotations

import math
import re

import repair_s0_contract as contract
import fusion_report

OBSERVATION_GAPS = {"missing_observation", "missing_plan", "missing_identity"}
GROUP_PARAMETERS = ("resolution", "boxes", "hull", "cloud", "outside", "cross")


def _add(reasons, code):
    if code not in reasons:
        reasons.append(code)


def _number_problem(value):
    """Named problem for one supplied sample. None means the sample is usable."""
    if value is None:
        return "missing_observation"
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return "bad_type"
    if not math.isfinite(value):
        return "nonfinite_value"
    if value < 0:
        return "negative_value"
    return None


def _sample_problems(reasons, values):
    blocked = False
    for value in values:
        problem = _number_problem(value)
        if problem:
            _add(reasons, problem)
            blocked = True
    return blocked


def _score_series(reasons, container, key, expected, limits, metric):
    """Score one required list. A missing list is not_evaluated, not an exception."""
    if not isinstance(container, dict) or key not in container or container[key] is None:
        _add(reasons, "missing_observation")
        return
    values = container[key]
    if not isinstance(values, list):
        _add(reasons, "bad_type")
        return
    if limits is None:
        if _sample_problems(reasons, values) or len(values) != expected:
            if len(values) != expected:
                _add(reasons, "missing_observation")
        return
    _latency_reason(reasons, values, expected, limits, metric)


def _scalar(reasons, container, key, limit=None, fail_code=None):
    if not isinstance(container, dict) or key not in container or container[key] is None:
        _add(reasons, "missing_observation")
        return None
    problem = _number_problem(container[key])
    if problem:
        _add(reasons, problem)
        return None
    if limit is not None and container[key] > limit:
        _add(reasons, fail_code)
        return None
    return container[key]


def _query_numbers(reasons, queries):
    for item in queries:
        if not isinstance(item, dict):
            _add(reasons, "missing_observation")
            continue
        for key in ("latency_ms", "age_ms", "acquire_ms", "roi_ms", "surface_ms"):
            if key not in item or item[key] is None:
                _add(reasons, "missing_observation")
            else:
                problem = _number_problem(item[key])
                if problem:
                    _add(reasons, problem)


def _stats(values, expected):
    if any(_number_problem(value) for value in values):
        return {"n": len(values), "expected_n": expected, "status": "incomplete"}
    return fusion_report.exact_stats(values, expected)


def _latency_reason(reasons, values, expected, limits, metric):
    if _sample_problems(reasons, values):
        return {"n": len(values), "expected_n": expected, "status": "incomplete"}
    if len(values) != expected:
        _add(reasons, "missing_observation")
        return {"n": len(values), "expected_n": expected, "status": "incomplete"}
    stats = _stats(values, expected)
    if not fusion_report.within_gate(stats, limits["p95_ms"], limits["max_ms"]):
        _add(reasons, metric)
    return stats


def _integrate_limits(cloud):
    if cloud >= 100000:
        return contract.INTEGRATE_100K
    return contract.INTEGRATE_30K


def _health(times, duration):
    if times is None:
        return ["missing_observation"]
    if not isinstance(times, list) or len(times) < 2:
        return ["missing_observation"]
    problems = []
    for value in times:
        problem = _number_problem(value)
        if problem:
            _add(problems, problem)
    if problems:
        return problems
    if any(times[index] > times[index + 1] for index in range(len(times) - 1)):
        return ["health_gap"]
    if times[0] > 0 or times[-1] < duration:
        return ["health_gap"]
    # A gap of exactly one second fails. The contract is a strict less-than.
    if max(times[index + 1] - times[index] for index in range(len(times) - 1)) >= contract.HEALTH_GAP_S:
        return ["health_gap"]
    return []


def _resources(resources):
    """Deduped counts are supplied by the caller. This judge does not walk a heap."""
    if not isinstance(resources, dict):
        return ["resource_missing"], "not_evaluated"
    complete = resources.get("attribution_complete") is True and resources.get("unmarked_allocations") is False
    read_view = resources.get("read_view_bytes")
    cache = resources.get("cache_bytes")
    peak_combined = resources.get("read_view_cache_bytes_peak")
    if not complete or "read_view_bytes" not in resources or "cache_bytes" not in resources:
        return ["resource_missing"], "not_evaluated"
    # The occupancy peaks are required: without them a transient between
    # samples is invisible and the gate says nothing.
    if "read_view_cache_bytes_peak" not in resources:
        return ["resource_peak_missing"], "not_evaluated"
    for value in (read_view, cache, peak_combined):
        problem = _number_problem(value)
        if problem == "missing_observation":
            return ["missing_observation"], "not_evaluated"
        if problem:
            return [problem], "fail"
    if any(isinstance(value, bool) or not isinstance(value, int)
           for value in (read_view, cache, peak_combined)):
        return ["bad_type"], "fail"
    if read_view < 0 or cache < 0 or peak_combined < 0:
        return ["resource_bytes"], "fail"
    if read_view + cache > contract.READ_VIEW_CACHE_LIMIT_BYTES:
        return ["resource_bytes"], "fail"
    # The same-instant occupancy peak gates exactly like the live sum: a
    # 1 GiB transient over a 2 KiB sampled current value fails here.
    if peak_combined > contract.READ_VIEW_CACHE_LIMIT_BYTES:
        return ["resource_peak"], "fail"
    if peak_combined < read_view + cache:
        return ["resource_peak_inconsistent"], "fail"
    return [], "pass"


def _match_ids(reasons, actual, planned, duplicate_code, missing_code, unknown_code):
    if len(actual) != len(set(actual)):
        _add(reasons, duplicate_code)
    if planned is None:
        _add(reasons, "missing_plan")
        return
    actual_set = set(actual)
    planned_set = set(planned)
    if len(actual) != len(planned) or actual_set != planned_set:
        if planned_set - actual_set:
            _add(reasons, missing_code)
        if actual_set - planned_set:
            _add(reasons, unknown_code)


def _query_identity(reasons, queries):
    for item in queries:
        required = ("covers_observation", "epoch", "observed_revision", "content_hash")
        if any(key not in item for key in required):
            _add(reasons, "missing_identity")
            continue
        if item["covers_observation"] is False:
            _add(reasons, "wrong_snapshot")
        elif item["covers_observation"] is not True:
            _add(reasons, "bad_type")
        for key in ("epoch", "observed_revision"):
            problem = _number_problem(item[key])
            if problem:
                _add(reasons, "missing_identity" if problem == "missing_observation" else problem)
        if not isinstance(item["content_hash"], str) or item["content_hash"] == "":
            _add(reasons, "missing_identity")


def judge_inputs(inputs, expected, phase, allow_reject=False):
    reasons = []
    rows = [row for row in inputs if row.get("phase") == phase]
    ids = [row.get("input_id") for row in rows]
    if len(rows) != expected:
        _add(reasons, "missing_frame" if len(rows) < expected else "input_count")
    if len(ids) != len(set(ids)):
        _add(reasons, "duplicate_input_id")
    rejected = [row for row in rows if row.get("disposition") == "rejected"]
    if rejected and not allow_reject:
        if len(rejected) == expected and len(rows) == expected:
            _add(reasons, "all_rejected")
        else:
            _add(reasons, "input_rejected")
    for row in rows:
        disposition = row.get("disposition")
        if disposition == "not_entered" or row.get("callback_enter_ns") is None:
            _add(reasons, "pre_callback_gap")
        elif disposition == "rejected":
            if row.get("terminal_count") != 1 or not row.get("reject_reason"):
                _add(reasons, "input_conservation")
        elif disposition == "integrated":
            if row.get("terminal_count", 0) > 1:
                _add(reasons, "duplicate_terminal")
            elif row.get("terminal_count") != 1 or not row.get("receipt_received"):
                _add(reasons, "input_conservation")
        else:
            _add(reasons, "input_conservation")
    return reasons, rows


def judge_o7_normal(case):
    reasons, rows = judge_inputs(case["inputs"], contract.NORMAL_INPUTS, "score")
    _match_ids(reasons, [row.get("input_id") for row in rows], case.get("planned_input_ids"),
               "duplicate_input_id", "missing_frame", "unknown_input")
    queries = case.get("queries") or []
    _match_ids(reasons, [item.get("query_id") for item in queries], case.get("planned_query_ids"),
               "duplicate_query_id", "missing_query", "unknown_query")
    _query_identity(reasons, queries)
    _query_numbers(reasons, queries)
    if len(queries) != contract.NORMAL_QUERIES or any(item.get("status") != "OK" for item in queries):
        _add(reasons, "query_status")
    blocked = {"all_rejected", "missing_frame", "pre_callback_gap", "input_conservation",
               "duplicate_terminal", "input_rejected"}
    if not blocked.intersection(reasons):
        _latency_reason(reasons, [row.get("integrate_ms") for row in rows], contract.NORMAL_INPUTS,
                        contract.INTEGRATE_30K, "integrate")
        ages = [row.get("queue_age_ms") for row in rows]
        if not _sample_problems(reasons, ages) and (not ages or max(ages) > contract.QUEUE_LIMIT_MS):
            _add(reasons, "queue_age")
    if "query_status" not in reasons and queries:
        for metric in ("acquire_ms", "roi_ms", "surface_ms"):
            _latency_reason(reasons, [item.get(metric) for item in queries], contract.NORMAL_QUERIES,
                            contract.QUERY, metric)
    reasons.extend(code for code in _health(case["health_times_s"] if "health_times_s" in case else None,
                                            contract.NORMAL_DURATION_S)
                   if code not in reasons)
    return _result(reasons)


def judge_o7_overload(case):
    reasons, rows = judge_inputs(case["inputs"], contract.OVERLOAD_INPUTS, "overload", allow_reject=True)
    _match_ids(reasons, [row.get("input_id") for row in rows], case.get("planned_input_ids"),
               "duplicate_input_id", "missing_frame", "unknown_input")
    if _sample_problems(reasons, [row.get("queue_age_ms") for row in rows if row.get("disposition") != "rejected"]):
        pass
    observed = any(row.get("disposition") == "rejected" or
                   (_number_problem(row.get("queue_age_ms")) is None and row.get("queue_age_ms", 0) > 0)
                   for row in rows)
    in_flight = case.get("in_flight_after_drain")
    if in_flight != 0:
        _add(reasons, "drain_inflight")
    hard = [code for code in reasons if code not in OBSERVATION_GAPS]
    if hard:
        status = "fail"
    elif not observed:
        status = "not_evaluated"
        _add(reasons, "overload_not_observed")
    elif reasons:
        status = "not_evaluated"
    else:
        status = "pass"
    return _result(reasons, status)


def judge_o7_recovery(case):
    reasons, rows = judge_inputs(case["inputs"], contract.RECOVERY_INPUTS, "recovery")
    _match_ids(reasons, [row.get("input_id") for row in rows], case.get("planned_input_ids"),
               "duplicate_input_id", "missing_frame", "unknown_input")
    # The recovery window must be the overloaded node continuing after the
    # fixed drain, not a fresh observer. Process identity and boundary stamps
    # prove it; a restart cannot sign this gate.
    continuity = case.get("continuity")
    if not isinstance(continuity, dict) or continuity.get("reused_observer") is not True:
        _add(reasons, "continuity_missing")
    else:
        pid = continuity.get("observer_pid")
        epoch = continuity.get("epoch")
        revision = continuity.get("observed_revision_at_recovery_open")
        drain = continuity.get("drain_boundary_mono_ns")
        origin = continuity.get("recovery_origin_mono_ns")
        for name, value in (("observer_pid", pid), ("epoch", epoch),
                            ("observed_revision_at_recovery_open", revision),
                            ("drain_boundary_mono_ns", drain),
                            ("recovery_origin_mono_ns", origin)):
            if not isinstance(value, int) or isinstance(value, bool):
                _add(reasons, "continuity_missing")
                break
        if all(isinstance(value, int) and not isinstance(value, bool)
               for value in (drain, origin)) and origin < drain:
            _add(reasons, "continuity_boundary")
    queries = case.get("queries") or []
    _match_ids(reasons, [item.get("query_id") for item in queries], case.get("planned_query_ids"),
               "duplicate_query_id", "missing_query", "unknown_query")
    _query_identity(reasons, queries)
    _query_numbers(reasons, queries)
    if len(queries) != contract.RECOVERY_QUERIES:
        _add(reasons, "recovery_queries")
    else:
        first = queries[0]
        latency = first.get("latency_ms") if "latency_ms" in first else None
        age = first.get("age_ms") if "age_ms" in first else None
        latency_problem = _number_problem(latency) if "latency_ms" in first else "missing_observation"
        age_problem = _number_problem(age) if "age_ms" in first else "missing_observation"
        if latency_problem:
            _add(reasons, latency_problem)
        elif first.get("op") != "wait_snapshot" or first.get("status") == "TIMEOUT" \
                or latency > contract.WAIT_TIMEOUT_MS:
            _add(reasons, "recovery_timeout")
        elif age_problem:
            _add(reasons, age_problem)
        elif first.get("status") != "OK" or first.get("covers_observation") is not True \
                or age > contract.WAIT_MAX_AGE_MS \
                or not rows or first.get("expected_observation_id") != rows[0]["input_id"]:
            _add(reasons, "wrong_snapshot")
        later = queries[1:]
        if any(item.get("op") != "immediate" or item.get("status") != "OK" for item in later):
            _add(reasons, "recovery_queries")
        elif later:
            for metric in ("acquire_ms", "roi_ms", "surface_ms"):
                _latency_reason(reasons, [item.get(metric) for item in later], contract.RECOVERY_QUERIES - 1,
                                contract.QUERY, metric)
    result = _result(reasons)
    legacy = case.get("legacy_immediate_first")
    result["legacy_immediate_first"] = legacy if isinstance(legacy, dict) else "not_recorded"
    result["legacy_changes_verdict"] = False
    return result


def judge_o7_campaign(case):
    windows = case.get("windows") or []
    ids = [window.get("id") for window in windows]
    reasons = []
    if len(windows) != contract.O7_WINDOWS:
        _add(reasons, "window_count")
    if len(ids) != len(set(ids)):
        _add(reasons, "duplicate_window")
    failed = []
    if len(windows) == contract.O7_WINDOWS:
        for window in windows:
            verdict = judge_o7_normal(window)
            if not verdict["pass"]:
                failed.append({"id": window.get("id"), "reasons": verdict["reasons"]})
    for item in failed:
        for code in item["reasons"]:
            _add(reasons, "%s:%s" % (item["id"], code))
    planned = case.get("planned_window_ids")
    if planned is not None and (set(ids) != set(planned) or len(ids) != len(planned)):
        if set(ids) - set(planned):
            _add(reasons, "unknown_window")
        if set(planned) - set(ids):
            _add(reasons, "missing_window")
    elif planned is None and not failed and "duplicate_window" not in reasons and "window_count" not in reasons:
        _add(reasons, "missing_plan")
    result = _result(reasons)
    result["failed_windows"] = [item["id"] for item in failed]
    result["pooled_latency"] = "not_computed"
    return result


def judge_latency_group(case):
    reasons = []
    cloud_problem = "missing_observation" if "cloud" not in case else _number_problem(case.get("cloud"))
    if cloud_problem:
        _add(reasons, cloud_problem)
    limits = None if cloud_problem else _integrate_limits(case["cloud"])
    _score_series(reasons, case, "integrate_ms", contract.SCORED_SAMPLES, limits, "integrate")
    for metric in contract.QUERY_METRICS:
        _score_series(reasons, case, metric, contract.SCORED_SAMPLES, contract.QUERY, metric)
    if "init_s" not in case or case.get("init_s") is None:
        _add(reasons, "missing_observation")
    else:
        init = case["init_s"]
        if not isinstance(init, list):
            _add(reasons, "bad_type")
        elif not _sample_problems(reasons, init) and (
                len(init) != contract.SCORED_SAMPLES or any(value > contract.INIT_LIMIT_S for value in init)):
            _add(reasons, "init")
    warm = _scalar(reasons, case, "warm_init_s", contract.INIT_LIMIT_S, "init")
    del warm
    if "rss_kib" not in case or case.get("rss_kib") is None:
        _add(reasons, "missing_observation")
    else:
        rss_problem = _number_problem(case["rss_kib"])
        if rss_problem:
            _add(reasons, rss_problem)
        elif not isinstance(case["rss_kib"], int) or case["rss_kib"] > contract.RSS_LIMIT_KIB:
            _add(reasons, "rss")
    resource_reasons, resource_status = _resources(case.get("resources"))
    for code in resource_reasons:
        _add(reasons, code)
    result = _result(reasons)
    result["resource_gate"] = resource_status
    return result


def judge_named_campaign(case, expected, judge_child):
    children = case.get("groups") or []
    ids = [child.get("id") for child in children]
    reasons = []
    if len(ids) != len(set(ids)):
        _add(reasons, "duplicate_group")
    failed = []
    for child in children:
        verdict = judge_child(child)
        if not verdict["pass"]:
            failed.append(child.get("id"))
    planned = case.get("planned_groups")
    if planned is not None:
        planned_ids = [item.get("id") for item in planned]
        if set(ids) != set(planned_ids) or len(ids) != len(planned_ids) or len(planned_ids) != expected:
            if set(ids) - set(planned_ids):
                _add(reasons, "unknown_group")
            if set(planned_ids) - set(ids) or len(planned_ids) != expected:
                _add(reasons, "missing_group")
        by_id = {}
        for item in planned:
            by_id.setdefault(item.get("id"), item)
        for child in children:
            plan = by_id.get(child.get("id"))
            if plan is None:
                continue
            if any(child.get(key) != plan.get(key) for key in GROUP_PARAMETERS):
                _add(reasons, "parameter_mismatch")
    elif not failed and "duplicate_group" not in reasons:
        _add(reasons, "missing_plan")
    for group_id in failed:
        _add(reasons, "group:%s" % group_id)
    result = _result(reasons)
    result["failed_groups"] = failed
    result["groups"] = len(children)
    result["expected_groups"] = expected
    result["pooled_latency"] = "not_computed"
    return result


def _rss_medians(reasons, run):
    """Recompute the window medians from the raw timestamped samples.

    The artifact's medians are only accepted when the judge's own
    re-bucketing of rss_samples into the three fixed windows reproduces them.
    """
    raw = run.get("rss_samples")
    if not isinstance(raw, list) or not raw:
        _add(reasons, "missing_observation")
        return
    samples = []
    for item in raw:
        if (not isinstance(item, (list, tuple)) or len(item) != 2
                or isinstance(item[1], bool) or not isinstance(item[1], int)
                or isinstance(item[0], bool) or not isinstance(item[0], (int, float))):
            _add(reasons, "bad_type")
            return
        samples.append((float(item[0]), int(item[1])))
    window_s = contract.O8_RSS_WINDOW_S
    recomputed = []
    counts = []
    for index in range(3):
        start = index * window_s
        bucket = [value for elapsed, value in samples if start <= elapsed < start + window_s]
        counts.append(len(bucket))
        recomputed.append(sorted(bucket)[(len(bucket) - 1) // 2] if bucket else None)
    medians = run.get("rss_median_kib")
    if not isinstance(medians, list) or len(medians) != 3:
        _add(reasons, "missing_observation")
        return
    if _sample_problems(reasons, medians):
        return
    if any(count < 1 for count in counts):
        _add(reasons, "rss_span")
        return
    if medians != recomputed:
        _add(reasons, "rss_recompute")
        return
    declared_counts = run.get("rss_window_counts")
    if declared_counts != counts:
        _add(reasons, "rss_recompute")
        return
    if medians[1] <= 0 or (medians[2] - medians[1]) / medians[1] > contract.O8_RSS_GROWTH:
        _add(reasons, "rss_growth")


def _judge_saturation_run(reasons, run):
    """Every saturation frame needs THE frame that was actually sent.

    The frame count is the contract's fixed 30 (the artifact's own claim must
    agree), each receipt must carry the sent identity (unique, matching the
    recorded stamp, with epoch/sensor), exactly one terminal event, a
    COMPLETED/INTEGRATED disposition, and zero change counters.
    """
    expected = contract.O8_SATURATION_FRAMES
    if run.get("saturation_frames_expected") != expected:
        _add(reasons, "saturation_frames")
        return
    receipts = run.get("saturation_receipts")
    stamps = run.get("saturation_stamps")
    if (not isinstance(receipts, list) or len(receipts) != expected
            or not isinstance(stamps, list) or len(stamps) != expected):
        _add(reasons, "saturation_frames")
        return
    ids = [entry.get("observation_id") if isinstance(entry, dict) else None
           for entry in receipts]
    if any(not isinstance(item, str) or not item for item in ids):
        _add(reasons, "saturation_identity")
    elif len(set(ids)) != expected or sorted(ids) != sorted(str(item) for item in stamps):
        _add(reasons, "saturation_identity")
    for entry in receipts:
        if not isinstance(entry, dict):
            _add(reasons, "missing_observation")
            continue
        events = entry.get("events")
        if not isinstance(events, int) or isinstance(events, bool) or events < 1:
            _add(reasons, "saturation_receipt_missing")
        elif events != 1:
            _add(reasons, "saturation_duplicate")
        if entry.get("status") != "COMPLETED" or entry.get("disposition") != "integrated":
            _add(reasons, "saturation_receipt_rejected")
        if entry.get("epoch") != 1 or entry.get("sensor_id") != "cam0":
            _add(reasons, "saturation_identity")
        if entry.get("receipt_observation_id") != entry.get("observation_id"):
            _add(reasons, "saturation_identity")
        changed = entry.get("changed_keys")
        rebuilt = entry.get("rebuilt_blocks")
        changed_problem = _number_problem(changed) if changed is not None else "missing_observation"
        rebuilt_problem = _number_problem(rebuilt) if rebuilt is not None else "missing_observation"
        if changed_problem:
            _add(reasons, "saturation_count_missing" if changed_problem == "missing_observation"
                 else changed_problem)
        elif changed != 0:
            _add(reasons, "saturation_keys")
        if rebuilt_problem:
            _add(reasons, "saturation_count_missing" if rebuilt_problem == "missing_observation"
                 else rebuilt_problem)
        elif rebuilt != 0:
            _add(reasons, "saturation_blocks")
    # The aggregates must be exactly what the receipts say.
    changed_values = [entry.get("changed_keys") for entry in receipts
                      if isinstance(entry, dict) and isinstance(entry.get("changed_keys"), int)]
    rebuilt_values = [entry.get("rebuilt_blocks") for entry in receipts
                      if isinstance(entry, dict) and isinstance(entry.get("rebuilt_blocks"), int)]
    if (len(changed_values) == expected
            and run.get("saturation_probability_keys") != max(changed_values)):
        _add(reasons, "saturation_aggregate")
    if (len(rebuilt_values) == expected
            and run.get("saturation_geometry_blocks") != max(rebuilt_values)):
        _add(reasons, "saturation_aggregate")


def _judge_node_stability(reasons, run):
    if "node_count_stable" not in run:
        _add(reasons, "missing_observation")
        return
    times = run.get("node_health_times_s")
    values = run.get("node_health_values")
    if (not isinstance(times, list) or not isinstance(values, list)
            or len(times) != len(values)
            or len(times) < contract.O8_HEALTH_MIN_SAMPLES):
        # A single lucky sample cannot certify stability: the post-warmup
        # window must actually be covered.
        _add(reasons, "node_health_span")
        return
    if _sample_problems(reasons, times):
        return
    if _sample_problems(reasons, values):
        return
    # Strictly increasing sample times: duplicated timestamps are one sample
    # counted twice and do not extend coverage.
    if any(times[index] >= times[index + 1] for index in range(len(times) - 1)):
        _add(reasons, "node_health_duplicate")
        return
    warmup = getattr(contract, "O8_RSS_WINDOW_S", 60.0)
    duration = 3 * warmup
    if times[0] > warmup + 5.0 or times[-1] < duration - 5.0:
        _add(reasons, "node_health_span")
    # Stability is recomputed from the raw values, not taken on trust.
    if len(set(values)) != 1 or run["node_count_stable"] is not True:
        _add(reasons, "node_count")


def _load_probe_content(probe, artifact_dir):
    """Independently recompute a probe's evidence from its saved content.

    Returns (states_bytes, odds_bytes, problems). The payload comes from the
    referenced lossless files (verified against their declared sha256) or
    from inline base64 (raw or zlib-compressed); the canonical content hash
    is recomputed from the bytes, never taken from the artifact's own string.
    """
    import base64
    import hashlib
    import zlib
    from pathlib import Path
    problems = []
    content = probe.get("content")
    if not isinstance(content, dict):
        return None, None, ["content_missing"]

    def _decode(*keys):
        for key in keys:
            if key not in content:
                continue
            raw = content.get(key)
            if not isinstance(raw, str):
                return None
            try:
                payload = base64.b64decode(raw, validate=True)
            except Exception:
                return None
            if key.endswith("_zb64"):
                try:
                    return zlib.decompress(payload)
                except Exception:
                    return None
            return payload
        return None

    states = _decode("states_b64", "states_zb64")
    odds = _decode("odds_b64", "odds_zb64")
    if states is None and odds is None and (
            isinstance(content.get("states_file"), str)
            and isinstance(content.get("odds_file"), str)):
        if artifact_dir is None:
            return None, None, ["content_file_unresolved"]
        base = Path(artifact_dir).resolve()
        states_path = (base / content["states_file"]).resolve()
        odds_path = (base / content["odds_file"]).resolve()
        # The references must stay inside the artifact, under a probe-content
        # directory of one of the runs.
        for resolved in (states_path, odds_path):
            if base not in resolved.parents or resolved.parent.name != "probe-content":
                return None, None, ["content_path_escape"]
        try:
            states = states_path.read_bytes()
            odds = odds_path.read_bytes()
        except OSError:
            return None, None, ["content_file_missing"]
    elif states is None or odds is None:
        if states is None and odds is None:
            return None, None, ["content_missing"]
        return None, None, ["content_missing"]
    # The per-payload shas are verified from the decoded bytes themselves,
    # for inline and file-backed content alike.
    for payload, key in ((states, "states_sha256"), (odds, "odds_sha256")):
        declared = content.get(key)
        actual = hashlib.sha256(payload).hexdigest()
        if not isinstance(declared, str) or declared != actual:
            problems.append("content_sha_mismatch:%s" % key)
    if problems:
        return None, None, problems
    recomputed = contract.probe_content_sha256(states, odds)
    if content.get("content_sha256") != recomputed:
        problems.append("content_hash_mismatch")
    return states, odds, problems


def _verify_probe_content(reasons, hold, artifact_dir):
    """Full fail-closed verification of one hold's probability evidence."""
    import math
    from array import array
    problems = []
    expected_region = contract.O8_ORACLE_REGION
    # The region alone fixes the required cell count: a probe may not claim
    # the full region while carrying fewer (or more) cells, whatever its
    # self-reported count says.
    expected_cells = contract.O8_ORACLE_CELLS
    probes = {}
    for name in ("probe_before", "probe_after"):
        probe = hold.get(name)
        if not isinstance(probe, dict):
            problems.append("probe_incomplete")
            continue
        region = probe.get("region")
        if not isinstance(region, dict) or \
                region.get("begin") != expected_region["begin"] or \
                region.get("end") != expected_region["end"]:
            problems.append("oracle_region")
        cells = probe.get("cells")
        if not isinstance(cells, int) or isinstance(cells, bool) or cells <= 0:
            problems.append("oracle_empty")
            probes[name] = None
            continue
        if cells != expected_cells:
            problems.append("oracle_cell_count")
        states, odds, load_problems = _load_probe_content(probe, artifact_dir)
        problems.extend(load_problems)
        if states is None:
            probes[name] = None
            continue
        if len(states) != expected_cells or len(odds) != 4 * expected_cells:
            problems.append("content_length_mismatch")
            probes[name] = None
            continue
        state_values = list(states)
        if any(value > 3 for value in state_values):
            problems.append("content_state_domain")
        odds_values = array("f")
        odds_values.frombytes(odds)
        if any(not math.isfinite(value) for value in odds_values):
            problems.append("nonfinite_value")
        probes[name] = (bytes(states), odds)
    if problems:
        for problem in problems:
            _add(reasons, problem)
        return
    if probes.get("probe_before") is None or probes.get("probe_after") is None:
        return None
    # The probability verdict is computed from the decoded content.
    return probes["probe_before"], probes["probe_after"]


def judge_o8(case):
    reasons = []
    runs = case.get("runs") if isinstance(case.get("runs"), list) else None
    if runs is None:
        _add(reasons, "missing_observation")
        runs = []
    elif len(runs) != contract.O8_RUNS:
        _add(reasons, "run_count")
    for run in runs:
        if not isinstance(run, dict):
            _add(reasons, "missing_observation")
            continue
        for key, fail_code in (("saturation_probability_keys", "saturation_keys"),
                               ("saturation_geometry_blocks", "saturation_blocks")):
            if key not in run or run[key] is None:
                _add(reasons, "missing_observation")
            else:
                problem = _number_problem(run[key])
                if problem:
                    _add(reasons, problem)
                elif run[key] != 0:
                    _add(reasons, fail_code)
        _judge_saturation_run(reasons, run)
        _judge_node_stability(reasons, run)
        _rss_medians(reasons, run)
    holds = case.get("holds") if isinstance(case.get("holds"), list) else None
    if holds is None:
        _add(reasons, "missing_observation")
        holds = []
    else:
        for kind in contract.O8_HOLD_KINDS:
            if sum(item.get("kind") == kind for item in holds
                   if isinstance(item, dict)) != contract.O8_HOLDS:
                _add(reasons, "hold_count")
    for hold in holds:
        if not isinstance(hold, dict):
            _add(reasons, "missing_observation")
            continue
        kind = hold.get("kind")
        # Measured hold duration: numeric, finite, non-negative, and at
        # least the contract seconds with no tolerance豁免.
        measured = hold.get("duration_measured_s")
        required = contract.O8_HOLD_S
        measured_problem = _number_problem(measured) if measured is not None else "missing_observation"
        if measured_problem:
            _add(reasons, "hold_duration" if measured_problem == "missing_observation"
                 else measured_problem)
        elif measured < required:
            _add(reasons, "hold_duration")
        # Both probes must be complete reads of a non-empty oracle: error
        # probes, empty regions, or a failed release cannot compare equal.
        for name in ("probe_before", "probe_after"):
            probe = hold.get(name)
            if not isinstance(probe, dict) or probe.get("error") is not None:
                _add(reasons, "probe_incomplete")
            elif not isinstance(probe.get("cells"), int) or probe["cells"] <= 0:
                _add(reasons, "oracle_empty")
            elif probe.get("release") != "OK" or probe.get("pages_done") is not True:
                _add(reasons, "probe_incomplete")
        # Independently recompute the content: region, lengths, state domain,
        # finite odds, file shas, and the canonical content hash. The verdict
        # comes from the decoded bytes, never from the declared strings.
        decoded = _verify_probe_content(reasons, hold, case.get("artifact_dir"))
        if decoded is not None:
            (before_states, before_odds), (after_states, after_odds) = decoded
            unchanged = before_states == after_states and before_odds == after_odds
            flag_name = ("region_probability_unchanged" if hold.get("kind") == "occlusion"
                         else "probability_unchanged")
            if unchanged is not (hold.get(flag_name) is True):
                _add(reasons, "probability")
        if kind == "occlusion":
            # A real blocking input: accepted new data whose rays never cross
            # the old region. The probability oracle must be recorded and
            # unchanged at the voxel level; freshness may legitimately advance
            # (the occluder is new data).
            record = hold.get("occlusion_input")
            if not isinstance(record, dict) or not isinstance(
                    record.get("points_sha256"), str) or not record.get("point_count"):
                _add(reasons, "occlusion_input_missing")
            else:
                oracle = record.get("oracle_min_cell_x")
                plane = record.get("plane_max_cell_x")
                if not isinstance(oracle, int) or not isinstance(plane, int) \
                        or oracle <= plane:
                    _add(reasons, "occlusion_geometry")
            if hold.get("input_accepted") is not True:
                _add(reasons, "occlusion_not_accepted")
            changed = hold.get("occluder_receipt", {}).get("changed_keys") \
                if isinstance(hold.get("occluder_receipt"), dict) else None
            if not isinstance(changed, int) or isinstance(changed, bool) or changed <= 0:
                _add(reasons, "occlusion_no_effect")
            if hold.get("region_probability_unchanged") is not True:
                _add(reasons, "probability")
        else:
            # no_input and duplicate_id: nothing may be accepted, so the map
            # must not move and freshness must not advance. The duplicate
            # replay additionally must be rejected at the ledger.
            if "probability_unchanged" not in hold:
                _add(reasons, "missing_observation")
            elif hold["probability_unchanged"] is not True:
                _add(reasons, "probability")
            if "freshness_advanced" not in hold:
                _add(reasons, "missing_observation")
            elif hold["freshness_advanced"] is not False:
                _add(reasons, "freshness")
            if kind == "duplicate_id" and hold.get("duplicate_rejected") is not True:
                _add(reasons, "duplicate_not_rejected")
    return _result(reasons)


def judge_o9(case):
    reasons = []
    injected = []
    operations = case.get("operations") or {}
    if set(operations) != {"roi", "integrate_30k", "surface_first"}:
        _add(reasons, "missing_pair")
    for pairs in operations.values():
        if len(pairs) != contract.O9_PAIRS:
            _add(reasons, "missing_pair")
            continue
        for index, pair in enumerate(pairs):
            if not isinstance(pair, dict):
                _add(reasons, "missing_observation")
                continue
            expected = "AB" if index % 2 == 0 else "BA"
            if pair.get("order") != expected:
                _add(reasons, "schedule")
            if pair.get("missing") is True:
                _add(reasons, "missing_pair")
            if pair.get("mismatched") is True:
                _add(reasons, "mismatched")
            if pair.get("injected_fault"):
                injected.append(pair["injected_fault"])
                _add(reasons, "injected_fault")
        if "missing_pair" not in reasons and "mismatched" not in reasons:
            off = []
            on = []
            complete = True
            for pair in pairs:
                if not isinstance(pair, dict) or "off_ms" not in pair or "on_ms" not in pair \
                        or pair.get("off_ms") is None or pair.get("on_ms") is None:
                    _add(reasons, "missing_observation")
                    complete = False
                    continue
                off.append(pair["off_ms"])
                on.append(pair["on_ms"])
            if not complete or _sample_problems(reasons, off + on):
                continue
            off_stats = _stats(off, contract.O9_PAIRS)
            on_stats = _stats(on, contract.O9_PAIRS)
            if off_stats.get("status") != "complete" or on_stats.get("status") != "complete" or off_stats["p95"] <= 0 \
                    or on_stats["p95"] / off_stats["p95"] > contract.O9_RATIO:
                _add(reasons, "perturbation_ratio")
    result = _result(reasons)
    result["injected_faults"] = injected
    result["performance_certified"] = result["pass"]
    return result


def judge_case(case):
    gate = case["gate"]
    if gate == "o7_normal":
        result = judge_o7_normal(case)
    elif gate == "o7_overload":
        result = judge_o7_overload(case)
    elif gate == "o7_recovery":
        result = judge_o7_recovery(case)
    elif gate == "o7_campaign":
        result = judge_o7_campaign(case)
    elif gate in ("o5_group", "o6_group"):
        result = judge_latency_group(case)
    elif gate == "o5_campaign":
        result = judge_named_campaign(case, contract.O5_GROUPS, judge_latency_group)
    elif gate == "o6_campaign":
        result = judge_named_campaign(case, contract.O6_GROUPS, judge_latency_group)
    elif gate == "o8":
        result = judge_o8(case)
    elif gate == "o9":
        result = judge_o9(case)
    else:
        result = _result(["unknown_gate"])
    result["gate"] = gate
    result["capture_certified"] = False
    return result


def _result(reasons, status=None):
    if status is None:
        if any(code not in OBSERVATION_GAPS for code in reasons):
            status = "fail"
        elif reasons:
            status = "not_evaluated"
        else:
            status = "pass"
    return {"pass": status == "pass", "status": status, "reasons": reasons}


def parameters_from_id(group_id):
    match = re.fullmatch(r"r(\d\d)_b(\d+)_([a-z]+)_c(\d+)_o(\d+)_x(\d+)", group_id)
    if not match:
        raise ValueError("group id is not a frozen matrix id")
    return {
        "id": group_id,
        "resolution": int(match.group(1)) / 100.0,
        "boxes": int(match.group(2)),
        "hull": match.group(3),
        "cloud": int(match.group(4)),
        "outside": int(match.group(5)) / 100.0,
        "cross": int(match.group(6)) / 100.0,
    }


def _health_times():
    return [index * 0.5 for index in range(int(contract.NORMAL_DURATION_S * 2) + 1)]


def _input(index, phase, disposition="integrated"):
    row = {
        "input_id": "%s-%d" % (phase, index),
        "phase": phase,
        "publish_end_ns": 1000 + index,
        "callback_enter_ns": 1100 + index,
        "disposition": disposition,
        "reject_reason": None,
        "terminal_count": 1,
        "integrate_ms": 40.0,
        "queue_age_ms": 0.2,
        "observed_revision": index + 1,
        "receipt_received": True,
    }
    if disposition == "rejected":
        row["reject_reason"] = "QUEUE_FULL"
        row["integrate_ms"] = None
    if disposition == "not_entered":
        row["callback_enter_ns"] = None
        row["terminal_count"] = 0
        row["receipt_received"] = False
        row["integrate_ms"] = None
    return row


def _query(index, op="immediate"):
    return {
        "query_id": "q-%d" % index,
        "op": op,
        "status": "OK",
        "latency_ms": 20.0,
        "age_ms": 20.0,
        "covers_observation": True,
        "expected_observation_id": None,
        "acquire_ms": 5.0,
        "roi_ms": 8.0,
        "surface_ms": 15.0,
        "epoch": 1,
        "observed_revision": index + 1,
        "content_hash": "chain-%d" % index,
    }


def _resources_ok():
    return {
        "rss_kib": 100000,
        "read_view_bytes": 1024,
        "cache_bytes": 1024,
        "read_view_bytes_peak": 2048,
        "cache_bytes_peak": 2048,
        "read_view_cache_bytes_peak": 3072,
        "attribution_complete": True,
        "unmarked_allocations": False,
    }


def healthy_o7_normal(window_id="normal_b00_r0"):
    inputs = [_input(index, "score") for index in range(contract.NORMAL_INPUTS)]
    queries = [_query(index) for index in range(contract.NORMAL_QUERIES)]
    return {
        "gate": "o7_normal",
        "id": window_id,
        "inputs": inputs,
        "queries": queries,
        "planned_input_ids": [row["input_id"] for row in inputs],
        "planned_query_ids": [row["query_id"] for row in queries],
        "health_times_s": _health_times(),
    }


def healthy_o7_overload():
    inputs = [_input(index, "overload") for index in range(contract.OVERLOAD_INPUTS - 1)]
    inputs.append(_input(contract.OVERLOAD_INPUTS - 1, "overload", "rejected"))
    return {
        "gate": "o7_overload",
        "inputs": inputs,
        "planned_input_ids": [row["input_id"] for row in inputs],
        "in_flight_after_drain": 0,
    }


def healthy_o7_recovery():
    inputs = [_input(index, "recovery") for index in range(contract.RECOVERY_INPUTS)]
    queries = [_query(0, "wait_snapshot")]
    queries[0]["latency_ms"] = 300.0
    queries[0]["age_ms"] = 300.0
    queries[0]["expected_observation_id"] = inputs[0]["input_id"]
    queries.extend(_query(index + 1) for index in range(contract.RECOVERY_QUERIES - 1))
    return {
        "gate": "o7_recovery",
        "inputs": inputs,
        "queries": queries,
        "planned_input_ids": [row["input_id"] for row in inputs],
        "planned_query_ids": [row["query_id"] for row in queries],
        "continuity": {
            "reused_observer": True,
            "observer_pid": 4242,
            "overload_window": "overload",
            "recovery_window": "recovery",
            "drain_boundary_mono_ns": 10_000_000_000,
            "recovery_origin_mono_ns": 10_050_000_000,
            "epoch": 1,
            "observed_revision_at_recovery_open": 600,
        },
        "legacy_immediate_first": {"status": "STALE", "age_ms": 30400.0, "contract": "get_snapshot"},
    }


def healthy_latency_group(group_id, cloud, parameters=None):
    sample = [10.0] * contract.SCORED_SAMPLES
    case = {
        "gate": "o6_group",
        "id": group_id,
        "cloud": cloud,
        "integrate_ms": list(sample),
        "init_s": [1.0] * contract.SCORED_SAMPLES,
        "warm_init_s": 1.0,
        "rss_kib": 100000,
        "resources": _resources_ok(),
    }
    if parameters:
        case.update(parameters)
    case["id"] = group_id
    case["cloud"] = cloud
    for metric in contract.QUERY_METRICS:
        case[metric] = list(sample)
    return case


def healthy_o8():
    def run(run_index):
        frames = contract.O8_SATURATION_FRAMES
        stamps = ["170000000000000000%d-%d" % (run_index, index)
                  for index in range(frames)]
        return {
            "run_id": "o8_r%d" % run_index,
            "saturation_frames_expected": frames,
            "saturation_frames_verified": frames,
            "saturation_stamps": stamps,
            "saturation_receipts": [
                {"index": index, "observation_id": stamps[index], "events": 1,
                 "status": "COMPLETED", "disposition": "integrated", "epoch": 1,
                 "sensor_id": "cam0", "receipt_observation_id": stamps[index],
                 "changed_keys": 0, "rebuilt_blocks": 0}
                for index in range(frames)],
            "saturation_probability_keys": 0,
            "saturation_geometry_blocks": 0,
            "node_count_stable": True,
            "node_health_times_s": [round(60.0 + index * 0.5, 3)
                                    for index in range(240)],
            "node_health_values": [4321] * 240,
            "node_health_count": 240,
            "rss_median_kib": [1000, 1000, 1004],
            "rss_window_counts": [30, 30, 30],
            "rss_samples": [[round(index * 2.0, 3), 1000 + (4 if index >= 60 else 0)]
                            for index in range(90)],
        }

    import base64
    import hashlib
    import struct as _struct
    import zlib

    def probe(marker, flip_odds=False):
        # The full frozen region: every cell present (mostly unknown, a few
        # marked states), lossless, zlib-compressed inline so the fixture
        # stays small while carrying the real payload.
        cells = contract.O8_ORACLE_CELLS
        states = bytearray(cells)
        for offset, value in ((0, 1), (1, 2), (2, 3), (cells - 1, 1)):
            states[offset] = value
        states = bytes(states)
        odds = [0.0] * cells
        odds[0], odds[1], odds[2] = -0.9, 0.4, 0.9
        if flip_odds:
            odds[5] = 1e-6
        odds_bytes = _struct.pack("<%df" % cells, *odds)
        content = {
            "states_zb64": base64.b64encode(zlib.compress(states)).decode(),
            "odds_zb64": base64.b64encode(zlib.compress(odds_bytes)).decode(),
            "states_sha256": hashlib.sha256(states).hexdigest(),
            "odds_sha256": hashlib.sha256(odds_bytes).hexdigest(),
            "content_sha256": contract.probe_content_sha256(states, odds_bytes),
        }
        return {
            "cells": cells,
            "region": {"begin": contract.O8_ORACLE_REGION["begin"],
                       "end": contract.O8_ORACLE_REGION["end"],
                       "shape": [75, 99, 74], "resolution_m": 0.02},
            "content": content,
            "surface_hash": "surface-%s" % marker,
            "surface_columns": 6534,
            "identity": {"epoch": 1, "map_revision": 2, "observed_revision": 2,
                         "last_integrated_stamp_ns": 1700000000000000000},
            "release": "OK",
            "pages": 8,
            "pages_done": True,
            "probe_mono_ns": 1,
            "duration_ms": 120.0,
            "last_integrated_stamp_ns": 1700000000000000000,
        }

    def hold(kind):
        before = probe("b")
        after = dict(probe("b"))
        after["probe_mono_ns"] = 2
        after["identity"] = dict(before["identity"])
        row = {"kind": kind, "duration_s": 10, "duration_measured_s": 10.4,
               "probe_before": before, "probe_after": after}
        if kind == "occlusion":
            after_occl = dict(probe("b"))
            after_occl["probe_mono_ns"] = 3
            after_occl["surface_hash"] = "surface-b"  # region-filtered: unchanged
            after_occl["identity"] = {"epoch": 1, "map_revision": 3,
                                      "observed_revision": 3,
                                      "last_integrated_stamp_ns": 1700000001000000000}
            after_occl["last_integrated_stamp_ns"] = 1700000001000000000
            row.update({
                "occlusion_input": {"shape": "plane", "plane_uld_x_m": -0.62,
                                    "plane_max_cell_x": 7, "oracle_min_cell_x": 9,
                                    "point_count": 1700, "points_sha256": "fixture",
                                    "rays_file": "fixture"},
                "input_accepted": True,
                "occluder_receipt": {"status": "COMPLETED", "changed_keys": 700},
                "region_probability_unchanged": True,
                "surface_geometry_unchanged": True,
                "freshness_advanced": True,
            })
            row["probe_after"] = after_occl
        elif kind == "duplicate_id":
            row.update({
                "duplicate_receipt": {"status": "COMPLETED", "disposition": "rejected"},
                "duplicate_rejected": True,
                "probability_unchanged": True,
                "surface_geometry_unchanged": True,
                "freshness_advanced": False,
            })
        else:
            row.update({"probability_unchanged": True,
                        "surface_geometry_unchanged": True,
                        "freshness_advanced": False})
        return row

    return {
        "gate": "o8",
        "runs": [run(index) for index in range(contract.O8_RUNS)],
        "holds": [hold(kind) for kind in contract.O8_HOLD_KINDS
                  for _ in range(contract.O8_HOLDS)],
    }


def healthy_o9():
    def pairs():
        return [{"order": "AB" if index % 2 == 0 else "BA", "off_ms": 10.0, "on_ms": 10.5,
                 "missing": False, "mismatched": False, "injected_fault": None}
                for index in range(contract.O9_PAIRS)]
    return {"gate": "o9", "operations": {"roi": pairs(), "integrate_30k": pairs(), "surface_first": pairs()}}
