"""Per-gate judge for synthetic and future ledgers.

Errors and timeouts stay in the denominator. One failed window or group is not
averaged away. Missing resource attribution cannot pass.
"""
from __future__ import annotations

import repair_s0_contract as contract
import fusion_report


def _add(reasons, code):
    if code not in reasons:
        reasons.append(code)


def _stats(values, expected):
    if any(isinstance(value, bool) or not isinstance(value, (int, float)) for value in values):
        return {"n": len(values), "expected_n": expected, "status": "incomplete"}
    return fusion_report.exact_stats(values, expected)


def _latency_reason(reasons, values, expected, limits, metric):
    stats = _stats(values, expected)
    if not fusion_report.within_gate(stats, limits["p95_ms"], limits["max_ms"]):
        _add(reasons, metric)
    return stats


def _integrate_limits(cloud):
    if cloud >= 100000:
        return contract.INTEGRATE_100K
    return contract.INTEGRATE_30K


def _health(times, duration):
    if not isinstance(times, list) or len(times) < 2:
        return ["health_gap"]
    if any(not isinstance(value, (int, float)) or isinstance(value, bool) for value in times):
        return ["health_gap"]
    if any(times[index] > times[index + 1] for index in range(len(times) - 1)):
        return ["health_gap"]
    if times[0] > 0 or times[-1] < duration:
        return ["health_gap"]
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
    if not complete or not isinstance(read_view, int) or not isinstance(cache, int):
        return ["resource_missing"], "not_evaluated"
    if read_view < 0 or cache < 0 or read_view + cache > contract.READ_VIEW_CACHE_LIMIT_BYTES:
        return ["resource_bytes"], "fail"
    return [], "pass"


def judge_inputs(inputs, expected, phase):
    reasons = []
    rows = [row for row in inputs if row.get("phase") == phase]
    ids = [row.get("input_id") for row in rows]
    if len(rows) != expected:
        _add(reasons, "missing_frame" if len(rows) < expected else "input_count")
    if len(ids) != len(set(ids)):
        _add(reasons, "duplicate_input_id")
    if rows and all(row.get("disposition") == "rejected" for row in rows) and len(rows) == expected:
        _add(reasons, "all_rejected")
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
    queries = case.get("queries") or []
    if len(queries) != contract.NORMAL_QUERIES or any(item.get("status") != "OK" for item in queries):
        _add(reasons, "query_status")
    if "all_rejected" not in reasons and "missing_frame" not in reasons and "pre_callback_gap" not in reasons \
            and "input_conservation" not in reasons and "duplicate_terminal" not in reasons:
        _latency_reason(reasons, [row["integrate_ms"] for row in rows], contract.NORMAL_INPUTS,
                        contract.INTEGRATE_30K, "integrate")
        ages = [row["queue_age_ms"] for row in rows]
        if not ages or max(ages) > contract.QUEUE_LIMIT_MS:
            _add(reasons, "queue_age")
    if "query_status" not in reasons:
        for metric in ("acquire_ms", "roi_ms", "surface_ms"):
            _latency_reason(reasons, [item[metric] for item in queries], contract.NORMAL_QUERIES,
                            contract.QUERY, metric)
    reasons.extend(code for code in _health(case.get("health_times_s"), contract.NORMAL_DURATION_S)
                   if code not in reasons)
    return _result(reasons)


def judge_o7_overload(case):
    reasons, rows = judge_inputs(case["inputs"], contract.OVERLOAD_INPUTS, "overload")
    if "all_rejected" in reasons:
        reasons = [code for code in reasons if code != "all_rejected"]
    observed = any(row.get("disposition") == "rejected" or (row.get("queue_age_ms") or 0) > 0 for row in rows)
    in_flight = case.get("in_flight_after_drain")
    if in_flight != 0:
        _add(reasons, "drain_inflight")
    if reasons:
        status = "fail"
    elif not observed:
        status = "not_evaluated"
        _add(reasons, "overload_not_observed")
    else:
        status = "pass"
    return _result(reasons, status)


def judge_o7_recovery(case):
    reasons, rows = judge_inputs(case["inputs"], contract.RECOVERY_INPUTS, "recovery")
    if "all_rejected" in reasons:
        reasons = [code for code in reasons if code != "all_rejected"]
        _add(reasons, "recovery_inputs")
    queries = case.get("queries") or []
    if len(queries) != contract.RECOVERY_QUERIES:
        _add(reasons, "recovery_queries")
    else:
        first = queries[0]
        latency = first.get("latency_ms")
        age = first.get("age_ms")
        timed_out = first.get("op") != "wait_snapshot" or first.get("status") == "TIMEOUT" \
            or not isinstance(latency, (int, float)) or isinstance(latency, bool) \
            or latency > contract.WAIT_TIMEOUT_MS
        identity_ok = first.get("status") == "OK" and first.get("covers_observation") is True \
            and isinstance(age, (int, float)) and not isinstance(age, bool) \
            and age <= contract.WAIT_MAX_AGE_MS \
            and rows and first.get("expected_observation_id") == rows[0]["input_id"]
        if timed_out:
            _add(reasons, "recovery_timeout")
        elif not identity_ok:
            _add(reasons, "wrong_snapshot")
        later = queries[1:]
        if any(item.get("op") != "immediate" or item.get("status") != "OK" for item in later):
            _add(reasons, "recovery_queries")
        elif later:
            for metric in ("acquire_ms", "roi_ms", "surface_ms"):
                _latency_reason(reasons, [item[metric] for item in later], contract.RECOVERY_QUERIES - 1,
                                contract.QUERY, metric)
    result = _result(reasons)
    legacy = case.get("legacy_immediate_first")
    result["legacy_immediate_first"] = legacy if isinstance(legacy, dict) else "not_recorded"
    result["legacy_changes_verdict"] = False
    return result


def judge_o7_campaign(case):
    windows = case["windows"]
    failed = []
    if len(windows) != contract.O7_WINDOWS:
        return _result(["window_count"], "not_evaluated")
    for window in windows:
        verdict = judge_o7_normal(window)
        if not verdict["pass"]:
            failed.append({"id": window["id"], "reasons": verdict["reasons"]})
    reasons = ["%s:%s" % (item["id"], code) for item in failed for code in item["reasons"]]
    result = _result(reasons)
    result["failed_windows"] = [item["id"] for item in failed]
    result["pooled_latency"] = "not_computed"
    return result


def judge_latency_group(case):
    reasons = []
    cloud = case["cloud"]
    _latency_reason(reasons, case["integrate_ms"], contract.SCORED_SAMPLES, _integrate_limits(cloud), "integrate")
    for metric in contract.QUERY_METRICS:
        _latency_reason(reasons, case[metric], contract.SCORED_SAMPLES, contract.QUERY, metric)
    init = case.get("init_s") or []
    if len(init) != contract.SCORED_SAMPLES or any(value > contract.INIT_LIMIT_S for value in init):
        _add(reasons, "init")
    if case.get("warm_init_s", contract.INIT_LIMIT_S + 1) > contract.INIT_LIMIT_S:
        _add(reasons, "init")
    rss = case.get("rss_kib")
    if not isinstance(rss, int) or rss > contract.RSS_LIMIT_KIB:
        _add(reasons, "rss")
    resource_reasons, resource_status = _resources(case.get("resources"))
    for code in resource_reasons:
        _add(reasons, code)
    result = _result(reasons)
    result["resource_gate"] = resource_status
    return result


def judge_named_campaign(case, expected, judge_child):
    children = case["groups"]
    failed = []
    for child in children:
        verdict = judge_child(child)
        if not verdict["pass"]:
            failed.append(child["id"])
    if failed:
        result = _result(["group:%s" % group_id for group_id in failed])
    elif len(children) != expected:
        result = _result(["group_count"], "not_evaluated")
    else:
        result = _result([])
    result["failed_groups"] = failed
    result["groups"] = len(children)
    result["expected_groups"] = expected
    result["pooled_latency"] = "not_computed"
    return result


def judge_o8(case):
    reasons = []
    runs = case.get("runs") or []
    if len(runs) != contract.O8_RUNS:
        _add(reasons, "run_count")
    for run in runs:
        if run.get("saturation_probability_keys") != 0:
            _add(reasons, "saturation_keys")
        if run.get("saturation_geometry_blocks") != 0:
            _add(reasons, "saturation_blocks")
        if run.get("node_count_stable") is not True:
            _add(reasons, "node_count")
        medians = run.get("rss_median_kib") or []
        if len(medians) != 3 or medians[1] <= 0 or (medians[2] - medians[1]) / medians[1] > contract.O8_RSS_GROWTH:
            _add(reasons, "rss_growth")
    holds = case.get("holds") or []
    if sum(item.get("kind") == "no_input" for item in holds) != contract.O8_HOLDS \
            or sum(item.get("kind") == "occlusion" for item in holds) != contract.O8_HOLDS:
        _add(reasons, "hold_count")
    for hold in holds:
        if hold.get("probability_unchanged") is not True:
            _add(reasons, "probability")
        if hold.get("freshness_advanced") is not False:
            _add(reasons, "freshness")
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
            off = _stats([pair["off_ms"] for pair in pairs], contract.O9_PAIRS)
            on = _stats([pair["on_ms"] for pair in pairs], contract.O9_PAIRS)
            if off.get("status") != "complete" or on.get("status") != "complete" or off["p95"] <= 0 \
                    or on["p95"] / off["p95"] > contract.O9_RATIO:
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
        status = "pass" if not reasons else "fail"
    return {"pass": status == "pass", "status": status, "reasons": reasons}


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
    }


def _resources_ok():
    return {
        "rss_kib": 100000,
        "read_view_bytes": 1024,
        "cache_bytes": 1024,
        "attribution_complete": True,
        "unmarked_allocations": False,
    }


def healthy_o7_normal(window_id="normal_b00_r0"):
    return {
        "gate": "o7_normal",
        "id": window_id,
        "inputs": [_input(index, "score") for index in range(contract.NORMAL_INPUTS)],
        "queries": [_query(index) for index in range(contract.NORMAL_QUERIES)],
        "health_times_s": _health_times(),
    }


def healthy_o7_overload():
    inputs = [_input(index, "overload") for index in range(contract.OVERLOAD_INPUTS - 1)]
    inputs.append(_input(contract.OVERLOAD_INPUTS - 1, "overload", "rejected"))
    return {"gate": "o7_overload", "inputs": inputs, "in_flight_after_drain": 0}


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
        "legacy_immediate_first": {"status": "STALE", "age_ms": 30400.0, "contract": "get_snapshot"},
    }


def healthy_latency_group(group_id, cloud):
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
    for metric in contract.QUERY_METRICS:
        case[metric] = list(sample)
    return case


def healthy_o8():
    return {
        "gate": "o8",
        "runs": [{
            "saturation_probability_keys": 0,
            "saturation_geometry_blocks": 0,
            "node_count_stable": True,
            "rss_median_kib": [1000, 1000, 1040],
        } for _ in range(contract.O8_RUNS)],
        "holds": [{"kind": kind, "probability_unchanged": True, "freshness_advanced": False}
                  for kind in ("no_input", "occlusion") for _ in range(contract.O8_HOLDS)],
    }


def healthy_o9():
    def pairs():
        return [{"order": "AB" if index % 2 == 0 else "BA", "off_ms": 10.0, "on_ms": 10.5,
                 "missing": False, "mismatched": False, "injected_fault": None}
                for index in range(contract.O9_PAIRS)]
    return {"gate": "o9", "operations": {"roi": pairs(), "integrate_30k": pairs(), "surface_first": pairs()}}
