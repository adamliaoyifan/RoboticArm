"""The gate must distinguish healthy full load, broken units, and missing data."""

from copy import deepcopy

import pytest

from occ_query_concurrency_gates import RATES, evaluate


@pytest.fixture
def observations():
    counts = {kind: int(rate * 120) for kind, rate in RATES.items()}
    rows = [
        dict(
            case=kind,
            sequence=i,
            phase="scoring",
            scheduled=100.0 + i / RATES[kind],
            completion_phase="post_window",
            status="OK",
            gated_ms=1.0,
            total_ms=2.0,
        )
        for kind, count in counts.items()
        for i in range(count)
    ]
    times = [90 + i * 0.25 for i in range(541)]
    return dict(
        owner_manifest=dict(rss_peak_kib=1000000),
        resources=dict(conservative_retained_bytes=1000000, retained_generations=2, sessions=0),
        result=dict(
            scored_start=100.0,
            ended_scoring=220.0,
            counts_scheduled=counts,
            capture_complete=True,
            runtime={"readiness": "READY"},
        ),
        rows=rows,
        sent=[
            dict(case=r["case"], sequence=r["sequence"], phase="scoring", scheduled=r["scheduled"])
            for r in rows
        ],
        queued=[],
        client_heartbeat=[
            dict(peer="cargo_volume_mapper", status="accepted", received_monotonic=t) for t in times
        ],
        owner_trace=[
            dict(
                stage="heartbeat", peer="occ_g2_load", status="accepted", monotonic_ns=int(t * 1e9)
            )
            for t in times
        ],
        owner_runtime={"readiness": "READY"},
        semantic=dict(outcome="pass", frozen_source_unchanged=True),
    )


def test_complete_healthy_load_can_pass(observations):
    result = evaluate(**observations)
    assert result["outcome"] == "pass"
    assert all(v == "pass" for v in result["gates"].values())
    assert result["overall_g2"] == "not_evaluated"


@pytest.mark.parametrize(
    "fault,gate",
    [
        ("late_max", "latency_summary"),
        ("backlog", "backlog"),
        ("missing", "all_requests_completed"),
        ("queued", "all_requests_completed"),
        ("duplicate", "all_requests_completed"),
        ("refusal", "all_requests_successful"),
        ("lease_transition", "lease"),
        ("lease_counter", "lease"),
        ("owner_gap", "heartbeat_load_to_mapper"),
        ("client_gap", "heartbeat_mapper_to_load"),
        ("semantic", "semantic"),
        ("capture", "capture"),
        ("rss", "rss"),
        ("memory", "derived_memory"),
        ("schedule", "schedule"),
        ("nan", "latency_summary"),
    ],
)
def test_broken_load_cannot_pass(observations, fault, gate):
    data = deepcopy(observations)
    if fault == "late_max":
        next(r for r in data["rows"] if r["case"] == "summary")["gated_ms"] = 201.0
    elif fault == "backlog":
        data["rows"][0]["total_ms"] = 1001.0
    elif fault == "missing":
        data["rows"].pop()
    elif fault == "queued":
        data["queued"] = [{"sequence": 240}]
    elif fault == "duplicate":
        data["rows"].append(data["rows"][0])
    elif fault == "refusal":
        data["rows"][0]["status"] = "QUERY_BUDGET_EXHAUSTED"
    elif fault == "lease_transition":
        data["owner_runtime"]["transitions"] = [{"reason": "PEER_LEASE_EXPIRED"}]
    elif fault == "lease_counter":
        data["owner_runtime"]["counters"] = {"PEER_LEASE_EXPIRED": 1}
    elif fault == "owner_gap":
        data["owner_trace"] = [
            r for r in data["owner_trace"] if not 110 <= r["monotonic_ns"] / 1e9 <= 111
        ]
    elif fault == "client_gap":
        data["client_heartbeat"] = [
            r for r in data["client_heartbeat"] if not 110 <= r["received_monotonic"] <= 111
        ]
    elif fault == "semantic":
        data["semantic"]["outcome"] = "fail_or_incomplete"
    elif fault == "rss":
        data["owner_manifest"]["rss_peak_kib"] = 2 * 1024**2 + 1
    elif fault == "schedule":
        data["sent"][0]["scheduled"] += 0.1
    elif fault == "nan":
        next(r for r in data["rows"] if r["case"] == "summary")["gated_ms"] = float("nan")
    elif fault == "memory":
        data["resources"]["conservative_retained_bytes"] = 512 * 1024**2 + 1
    else:
        data["result"]["capture_complete"] = False
    result = evaluate(**data)
    assert result["outcome"] == "fail"
    assert result["gates"][gate] == "fail"


def test_interrupted_window_and_missing_samples_are_not_certified(observations):
    observations["result"]["ended_scoring"] = 101.0
    observations["rows"] = observations["rows"][:1]
    observations["sent"] = observations["sent"][:1]
    result = evaluate(**observations)
    assert result["gates"]["window"] == result["gates"]["fixed_load"] == "not_evaluated"
    assert result["gates"]["latency_roi"] == "not_evaluated"
    assert result["outcome"] == "not_evaluated"


def test_teardown_expiry_after_observation_window_is_not_pressure_failure(observations):
    observations["owner_runtime"] = dict(
        observed_at_monotonic=226.0,
        readiness="PEER_LEASE_EXPIRED",
        counters={},
        transitions=[dict(at_monotonic=226.0, reason="PEER_LEASE_EXPIRED")],
    )
    assert evaluate(**observations)["gates"]["lease"] == "pass"


def test_expiry_observed_inside_window_still_fails(observations):
    observations["owner_runtime"] = dict(
        observed_at_monotonic=224.0,
        readiness="PEER_LEASE_EXPIRED",
        counters={},
        transitions=[dict(at_monotonic=224.0, reason="PEER_LEASE_EXPIRED")],
    )
    assert evaluate(**observations)["gates"]["lease"] == "fail"
