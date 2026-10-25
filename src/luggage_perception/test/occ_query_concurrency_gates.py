"""Pure, fail-closed scoring of retained fixed-load concurrency observations.

No ROS node or runtime policy is changed here. Early stops retain measurable
failures, while missing 120-second coverage explicitly remains not_evaluated.
"""

import math

RATES = {"integrate": 2, "summary": 10, "roi": 5, "distance": 2, "surface": 1}
LIMITS = {"summary": (50, 200), "roi": (100, 250), "distance": (50, 200), "surface": (100, 250)}


def verdict(observed, expected):
    return "not_evaluated" if observed is None else "pass" if expected(observed) else "fail"


def heartbeat_gate(times, start, end):
    if start is None or end is None:
        return dict(outcome="not_evaluated", samples=len(times), max_gap_s=None)
    # Include the nearest samples bracketing the required pre/post window.
    times = sorted(times)
    before = [t for t in times if t <= start - 10]
    after = [t for t in times if t >= end + 5]
    window = ([before[-1]] if before else []) + [t for t in times if start - 10 < t < end + 5]
    if after:
        window.append(after[0])
    gap = max((b - a for a, b in zip(window, window[1:])), default=None)
    covered = bool(window) and window[0] <= start - 9.5 and window[-1] >= end + 4.5
    outcome = (
        "fail"
        if gap is not None and gap >= 1
        else "pass" if covered and len(window) >= 2 else "not_evaluated"
    )
    return dict(outcome=outcome, samples=len(window), max_gap_s=gap, window_covered=covered)


def lease_faults(value):
    """Inspect receive statuses, transitions, current readiness, and counters."""
    if isinstance(value, dict):
        return sum(
            (
                int(k == "PEER_LEASE_EXPIRED" and isinstance(v, (int, float)) and v > 0)
                + lease_faults(v)
            )
            for k, v in value.items()
        )
    if isinstance(value, list):
        return sum(lease_faults(v) for v in value)
    return int(isinstance(value, str) and value == "PEER_LEASE_EXPIRED")


def window_records(records, low, high, field):
    """Missing timestamps cannot erase recorded failures in old captures."""
    return [row for row in records if field not in row or low <= row[field] <= high]


def runtime_lease_faults(runtime, low, high, observed_at=None):
    observed_at = runtime.get("observed_at_monotonic", observed_at)
    # Counters do not carry per-event timestamps. Preserve their failure
    # evidence conservatively; the complete passive timeline provides scope.
    faults = lease_faults(runtime.get("counters", {}))
    if observed_at is None or low <= observed_at <= high:
        faults += lease_faults(runtime.get("readiness"))
    faults += lease_faults(
        window_records(runtime.get("transitions", []), low, high, "at_monotonic")
    )
    faults += lease_faults(
        window_records(runtime.get("receive_trace", []), low, high, "received_monotonic")
    )
    for scope in runtime.get("scopes", {}).values():
        faults += runtime_lease_faults(scope, low, high, observed_at)
    return faults


def evaluate(
    result,
    rows,
    sent,
    queued,
    client_heartbeat,
    owner_trace,
    owner_runtime,
    semantic,
    *,
    required_duration=120.0,
    owner_manifest=None,
    resources=None,
):
    start, end = result.get("scored_start"), result.get("ended_scoring")
    duration = end - start if start is not None and end is not None else None
    gates = {}
    gates["window"] = verdict(duration, lambda value: value >= required_duration)
    if duration is not None and duration < required_duration:
        gates["window"] = "not_evaluated"
    sent_keys = [(r["case"], r["sequence"]) for r in sent]
    row_keys = [(r["case"], r["sequence"]) for r in rows]
    joined = (
        set(sent_keys) == set(row_keys)
        and len(sent_keys) == len(set(sent_keys))
        and len(row_keys) == len(set(row_keys))
    )
    gates["all_requests_completed"] = "pass" if joined and not queued else "fail"
    gates["all_requests_successful"] = (
        "pass" if rows and all(r["status"] == "OK" for r in rows) else "fail"
    )
    scoring = [r for r in rows if r["phase"] == "scoring"]
    loads = {}
    for kind, rate in RATES.items():
        expected = math.ceil(required_duration * rate)
        scheduled = result.get("counts_scheduled", {}).get(kind, 0)
        completed = sum(r["case"] == kind for r in scoring)
        loads[kind] = dict(expected=expected, scheduled=scheduled, completed=completed)
    gates["fixed_load"] = (
        "pass"
        if all(v["expected"] == v["scheduled"] == v["completed"] for v in loads.values())
        else "not_evaluated" if gates["window"] != "pass" else "fail"
    )
    schedules = [r for r in sent if r.get("phase") == "scoring"]
    exact_schedule = start is not None and all(
        math.isclose(
            r["scheduled"], start + r["sequence"] / RATES[r["case"]], abs_tol=1e-6, rel_tol=0.0
        )
        for r in schedules
    )
    gates["schedule"] = (
        "pass"
        if exact_schedule and len(schedules) == sum(v["expected"] for v in loads.values())
        else "not_evaluated" if gates["window"] != "pass" else "fail"
    )
    latency = {}
    for kind, (p95_limit, max_limit) in LIMITS.items():
        values = sorted(r["gated_ms"] for r in scoring if r["case"] == kind and r["status"] == "OK")
        p95 = values[math.ceil(0.95 * len(values)) - 1] if values else None
        maximum = max(values, default=None)
        finite = all(math.isfinite(v) and v >= 0 for v in values)
        enough = len(values) >= math.ceil(required_duration * RATES[kind])
        outcome = (
            "fail"
            if not finite or maximum is not None and maximum > max_limit
            else "not_evaluated" if not enough else "pass" if p95 <= p95_limit else "fail"
        )
        latency[kind] = dict(outcome=outcome, samples=len(values), p95_ms=p95, max_ms=maximum)
        gates["latency_" + kind] = outcome
    backlog = max([result.get("backlog_max_s", 0.0)] + [r["total_ms"] / 1000 for r in scoring])
    gates["backlog"] = "fail" if backlog > 1 else "pass" if scoring else "not_evaluated"
    heartbeat = {
        "mapper_to_load": heartbeat_gate(
            [
                r["received_monotonic"]
                for r in client_heartbeat
                if r.get("peer") == "cargo_volume_mapper" and r.get("status") == "accepted"
            ],
            start,
            end,
        ),
        "load_to_mapper": heartbeat_gate(
            [
                r["monotonic_ns"] / 1e9
                for r in owner_trace
                if r.get("stage") == "heartbeat"
                and r.get("peer") == "occ_g2_load"
                and r.get("status") == "accepted"
            ],
            start,
            end,
        ),
    }
    for direction, item in heartbeat.items():
        gates["heartbeat_" + direction] = item["outcome"]
    low = start - 10 if start is not None else -math.inf
    high = result.get("observation_end", end + 5 if end is not None else math.inf)
    owner_heartbeat = [row for row in owner_trace if row.get("stage") == "heartbeat"]
    timed_owner = [
        dict(row, received_monotonic=row["monotonic_ns"] / 1e9) for row in owner_heartbeat
    ]
    faults = lease_faults(
        window_records(client_heartbeat + timed_owner, low, high, "received_monotonic")
    )
    faults += runtime_lease_faults(result.get("runtime", {}), low, high)
    faults += runtime_lease_faults(owner_runtime, low, high)
    gates["lease"] = (
        "fail"
        if faults or result.get("lease_expiry", 0)
        else "pass" if owner_runtime and result.get("runtime") else "not_evaluated"
    )
    gates["semantic"] = semantic.get("outcome", "not_evaluated")
    if gates["semantic"] not in ("pass", "fail", "not_evaluated"):
        gates["semantic"] = "fail"
    gates["capture"] = (
        "pass"
        if result.get("capture_complete") and semantic.get("frozen_source_unchanged")
        else "fail"
    )
    owner_manifest = owner_manifest or {}
    gates["rss"] = verdict(owner_manifest.get("rss_peak_kib"), lambda value: value <= 2 * 1024**2)
    if not result.get("baseline_adapter"):
        resources = resources or {}
        gates["derived_memory"] = verdict(
            resources.get("conservative_retained_bytes"), lambda value: value <= 512 * 1024**2
        )
        gates["retained_generations"] = verdict(
            resources.get("retained_generations"), lambda value: value <= 2
        )
        gates["sessions"] = verdict(resources.get("sessions"), lambda value: value <= 128)
    if result.get("first_failure"):
        gates["recorded_failure"] = "fail"
    outcome = (
        "fail"
        if "fail" in gates.values()
        else "not_evaluated" if "not_evaluated" in gates.values() else "pass"
    )
    return dict(
        outcome=outcome,
        gates=gates,
        loads=loads,
        latency=latency,
        heartbeat=heartbeat,
        backlog_max_s=backlog,
        lease_fault_observations=faults,
        duration_s=duration,
        required_duration_s=required_duration,
        reliability="AND over all units; no credit for an interrupted window",
        overall_g2="not_evaluated",
    )
