"""gen2 tool-repair tests: every review counterexample is a permanent red
test with a healthy green control, and full_query's own fault paths are
exercised against injected fake services.

The review's audit.py injected: 240 INTERNAL_ERROR queries under a formal P
window, an R artifact with 599 missing overload ids and a NaN first-wait, an
L artifact with zero seconds/traffic and a NaN cache peak, a query page that
returned 1 of 100 candidates with the wrong identity and a failing release,
and a query timeout that leaked the lease. None may ever pass again.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path
from types import SimpleNamespace as S

import pytest

BENCH = Path(__file__).resolve().parent / "benchmark"
sys.path.insert(0, str(BENCH))
import freespace_campaign as campaign  # noqa: E402
import freespace_contract as contract  # noqa: E402
import freespace_judge as judge  # noqa: E402

GEOMETRY_HASH = "cb41f55f243786d50ab30997291083ab40467507abcf78a00e55da77ffc00856"
IDENTITY = {"epoch": 1, "map_revision": 5, "observed_revision": 9,
            "geometry_hash": GEOMETRY_HASH, "last_integrated_stamp_ns": 10**9}


# --- the review's counterexamples, verbatim in spirit ----------------------
def audit_p():
    return {"formal": True,
            "sender": [{"input_id": str(i), "planned_mono_ns": i * 400_000_000,
                        "publish_mono_ns": i * 400_000_000} for i in range(300)],
            "queries": [{"planned_mono_ns": i * 500_000_000,
                         "started_mono_ns": i * 500_000_000,
                         "ended_mono_ns": i * 500_000_000 + 1_000_000,
                         "status": "INTERNAL_ERROR"} for i in range(240)],
            "input_timeline": [{"input_id": str(i), "receipt_status": "COMPLETED",
                                "e2e_ms": 1.0} for i in range(300)]}


def audit_r():
    return {"formal": True, "overload": [{}] * 600,
            "overload_timeline": [{"input_id": "x", "receipt_status": None}] * 600,
            "overload_terminals": {"unrelated": "COMPLETED"},
            "first_wait": {"status": "OK", "elapsed_ms": float("nan")},
            "epoch_after": 1, "recovery": []}


def audit_l():
    return {"formal": True, "planned_seconds": 0, "read_rounds": 0,
            "read_rounds_results": [], "reset_rounds": 0, "reset_statuses": [],
            "baseline": {"view_directory_entries": 0},
            "after": {"view_directory_entries": 0,
                      "read_view_cache_bytes_peak": float("nan")},
            "query_records": [], "rss_series": []}


def test_audit_p_internal_error_queries_fail():
    reasons = judge.judge_p_window(audit_p())["reasons"]
    assert any("status_INTERNAL_ERROR" in r for r in reasons)


def test_audit_r_missing_recovery_and_nan_wait_fail():
    reasons = judge.judge_r_stage(audit_r())["reasons"]
    assert any("not_terminal" in r for r in reasons)
    assert any("recovery" in r for r in reasons)
    assert any("first_wait" in r for r in reasons)


def test_audit_l_zero_second_nan_peak_fails():
    reasons = judge.judge_l_stage(audit_l())["reasons"]
    assert reasons
    assert any("cache_peak_bad" in r or "rss" in r or "no_query_records" in r
               for r in reasons)


# --- full_query fault injections (fake service) -----------------------------
class FakeService:
    get_snapshot = "get"
    freespace = "query"
    release = "release"

    def __init__(self, mode="healthy", candidates=100, feasible=None):
        self.mode = mode
        self.candidates = candidates
        self.feasible = feasible if feasible is not None else candidates
        self.released = False
        self.calls = 0

    def call(self, service, request, timeout=10.0):
        if service == "get":
            return S(status="OK", handle="1:1", identity=S(**IDENTITY),
                     age_ns=10**9)
        if service == "release":
            self.released = True
            return S(status="INTERNAL_ERROR" if self.mode == "release_fails"
                     else "OK")
        if self.mode == "timeout":
            raise TimeoutError("injected")
        self.calls += 1
        count = 1 if self.mode == "short_page" else self.candidates
        identity = S(**IDENTITY)
        if self.mode == "wrong_identity":
            identity = S(epoch=99, map_revision=99, observed_revision=99,
                         geometry_hash="wrong", last_integrated_stamp_ns=0)
        done = True
        next_cursor = self.feasible
        if self.mode == "cursor_stall":
            next_cursor = 0
            done = False
        return S(status="OK", identity=identity,
                 cell_x=[0] * count, cell_y=[0] * count, cell_z=[0] * count,
                 origin_x=[0.0] * count, origin_y=[0.0] * count,
                 origin_z=[0.0] * count, yaw_deg=[0] * count,
                 support_ratio=[1.0] * count, support_kind=[2] * count,
                 verify_begin=[0, 0, 0] * count, verify_end=[15, 10, 10] * count,
                 next_cursor=next_cursor, done=done, feasible_total=self.feasible)


def test_full_query_rejects_short_page(monkeypatch):
    patched = None
    monkeypatch.setattr(campaign, "_snapshot_request", lambda x: x)
    monkeypatch.setattr(campaign, "_freespace_request", lambda *a: a)
    monkeypatch.setattr(campaign, "_release_request", lambda x: x)
    fake = FakeService(mode="short_page", candidates=100, feasible=100)
    record = campaign.full_query(fake, (0.30, 0.20, 0.20))
    assert record["status"] == "CANDIDATE_COUNT_MISMATCH"
    assert fake.released, "fault path must still release the lease"


def test_full_query_rejects_identity_drift(monkeypatch):
    monkeypatch.setattr(campaign, "_snapshot_request", lambda x: x)
    monkeypatch.setattr(campaign, "_freespace_request", lambda *a: a)
    monkeypatch.setattr(campaign, "_release_request", lambda x: x)
    fake = FakeService(mode="wrong_identity", candidates=5, feasible=5)
    record = campaign.full_query(fake, (0.30, 0.20, 0.20))
    assert record["status"] == "PAGE_IDENTITY_DRIFT"
    assert fake.released


def test_full_query_rejects_cursor_stall(monkeypatch):
    monkeypatch.setattr(campaign, "_snapshot_request", lambda x: x)
    monkeypatch.setattr(campaign, "_freespace_request", lambda *a: a)
    monkeypatch.setattr(campaign, "_release_request", lambda x: x)
    fake = FakeService(mode="cursor_stall", candidates=0, feasible=100)
    record = campaign.full_query(fake, (0.30, 0.20, 0.20))
    assert record["status"] == "PAGE_CURSOR_PROGRESS"
    assert fake.released


def test_full_query_records_release_failure(monkeypatch):
    monkeypatch.setattr(campaign, "_snapshot_request", lambda x: x)
    monkeypatch.setattr(campaign, "_freespace_request", lambda *a: a)
    monkeypatch.setattr(campaign, "_release_request", lambda x: x)
    fake = FakeService(mode="release_fails", candidates=3, feasible=3)
    record = campaign.full_query(fake, (0.30, 0.20, 0.20))
    assert record["status"] == "OK"
    assert record["release_status"] == "INTERNAL_ERROR"
    assert record["release_fault"] == "INTERNAL_ERROR"
    assert record["content_sha256"]


def test_full_query_releases_on_timeout(monkeypatch):
    monkeypatch.setattr(campaign, "_snapshot_request", lambda x: x)
    monkeypatch.setattr(campaign, "_freespace_request", lambda *a: a)
    monkeypatch.setattr(campaign, "_release_request", lambda x: x)
    fake = FakeService(mode="timeout")
    record = campaign.full_query(fake, (0.30, 0.20, 0.20))
    assert record["status"].startswith("QUERY_TIMEOUT")
    assert fake.released, "a timed-out query must not leak the lease"
    assert record["release_status"] == "OK"


def test_full_query_healthy_materializes_all(monkeypatch):
    monkeypatch.setattr(campaign, "_snapshot_request", lambda x: x)
    monkeypatch.setattr(campaign, "_freespace_request", lambda *a: a)
    monkeypatch.setattr(campaign, "_release_request", lambda x: x)
    fake = FakeService(mode="healthy", candidates=1000, feasible=1000)
    record = campaign.full_query(fake, (0.30, 0.20, 0.20))
    assert record["status"] == "OK"
    assert len(record["candidates"]) == 1000
    assert record["pages"] == 1
    assert record["lease_identity"] == IDENTITY
    assert record["age_ns"] == 10**9


# --- healthy formal controls: the judges accept genuinely good artifacts ---
def healthy_p_window():
    inputs = contract.P_WINDOW_INPUTS
    queries = contract.P_WINDOW_QUERIES
    period_in = int(1e9 / contract.UPDATE_HZ)
    period_q = int(1e9 / contract.QUERY_HZ)
    sender = [{"input_id": str(i), "planned_mono_ns": i * period_in,
               "publish_mono_ns": i * period_in + 2_000_000} for i in range(inputs)]
    timeline = [{"input_id": str(i), "receipt_status": "COMPLETED",
                 "e2e_ms": 120.0} for i in range(inputs)]
    good_query = {"planned_mono_ns": 0, "started_mono_ns": 1,
                  "ended_mono_ns": 2, "status": "OK", "latency_ms": 100.0,
                  "release_status": "OK", "lease_identity": IDENTITY,
                  "content_sha256": "x" * 64, "age_ns": 10**9}
    queries_list = []
    for i in range(queries):
        record = dict(good_query)
        record["planned_mono_ns"] = i * period_q
        record["started_mono_ns"] = i * period_q + 3_000_000
        record["ended_mono_ns"] = i * period_q + 100_000_000
        queries_list.append(record)
    return {"formal": True, "errors": [], "sender": sender,
            "queries": queries_list, "input_timeline": timeline,
            "terminals": {}, "pending": [], "health_samples": 240,
            "health_gap_max_s": 0.5, "window_close_ms": 120_000.0,
            "window_planned_s": 120.0}


def test_healthy_formal_p_window_passes():
    verdict = judge.judge_p_window(healthy_p_window())
    assert verdict["reasons"] == [], verdict["reasons"]


def test_formal_p_rejects_over_gate_e2e():
    window = healthy_p_window()
    for entry in window["input_timeline"]:
        entry["e2e_ms"] = 360.0  # P95 over the 350 ms gate
    reasons = judge.judge_p_window(window)["reasons"]
    assert any("input_e2e_p95" in r for r in reasons)


def test_formal_p_rejects_sched_late():
    window = healthy_p_window()
    for record in window["queries"]:
        record["started_mono_ns"] = record["planned_mono_ns"] + 60_000_000
    reasons = judge.judge_p_window(window)["reasons"]
    assert any("sched_late" in r for r in reasons)


def test_formal_p_rejects_count_lie_and_bad_age():
    window = healthy_p_window()
    window["queries"] = window["queries"][:100]
    assert "formal_counts" in judge.judge_p_window(window)["reasons"]
    window = healthy_p_window()
    for record in window["queries"]:
        record["age_ns"] = 5 * 10**9
    assert any("q_age" in r for r in judge.judge_p_window(window)["reasons"])


def healthy_r_artifact():
    overload = [{"input_id": "o%d" % i, "planned_mono_ns": i * 100_000_000,
                 "publish_mono_ns": i * 100_000_000}
                for i in range(contract.R_OVERLOAD_INPUTS)]
    timeline = [{"input_id": "o%d" % i,
                 "receipt_status": "REJECTED" if i == 0 else "COMPLETED",
                 "e2e_ms": 100.0}
                for i in range(contract.R_OVERLOAD_INPUTS)]
    recovery = [{"input_id": "r%d" % i, "planned_mono_ns": i * 400_000_000,
                 "publish_mono_ns": i * 400_000_000}
                for i in range(contract.R_RECOVERY_INPUTS)]
    recovery_timeline = [{"input_id": "r%d" % i, "receipt_status": "COMPLETED",
                          "e2e_ms": 90.0} for i in range(contract.R_RECOVERY_INPUTS)]
    good = {"status": "OK", "latency_ms": 100.0, "release_status": "OK",
            "lease_identity": IDENTITY, "content_sha256": "y" * 64,
            "age_ns": 10**9, "planned_mono_ns": 0, "started_mono_ns": 1,
            "ended_mono_ns": 2}
    return {"formal": True, "overload": overload, "overload_timeline": timeline,
            "overload_terminals": {}, "overload_pending": [],
            "in_flight_at_drain_deadline": [], "epoch_before": 1,
            "epoch_after": 1, "recovery": recovery,
            "recovery_timeline": recovery_timeline,
            "overload_queries": [dict(good, status="STALE")],
            "recovery_queries": [dict(good)],
            "first_wait": {"status": "OK", "elapsed_ms": 800.0, "age_ns": 10**9,
                           "bound_id": "r0"}}


def test_healthy_formal_r_passes():
    verdict = judge.judge_r_stage(healthy_r_artifact())
    assert verdict["reasons"] == [], verdict["reasons"]


def test_formal_r_rejects_in_flight_and_restarted_epoch():
    artifact = healthy_r_artifact()
    artifact["in_flight_at_drain_deadline"] = ["o599"]
    assert any("drain_in_flight" in r for r in judge.judge_r_stage(artifact)["reasons"])
    artifact = healthy_r_artifact()
    artifact["epoch_after"] = 2
    assert any("epoch_restarted" in r for r in judge.judge_r_stage(artifact)["reasons"])


def healthy_l_artifact():
    good = {"status": "OK", "latency_ms": 100.0, "release_status": "OK",
            "lease_identity": IDENTITY, "content_sha256": "z" * 64,
            "age_ns": 10**9}
    total = int(contract.L_DURATION_S)
    # 1 Hz sampling for the whole 120 min: >=600 samples per 10-min bucket.
    rss = [{"t_mono_ns": i * 10**9, "rss_kib": 300_000}
           for i in range(0, total + 2)]
    return {"formal": True, "send_errors": [], "sent_inputs": contract.L_INPUTS,
            "query_records": [dict(good) for _ in range(contract.L_QUERIES)],
            "rss_series": rss, "health_samples": 7000,
            "read_rounds": 2, "reset_rounds": 1,
            "read_rounds_results": [dict(good), dict(good)],
            "reset_statuses": ["OK"],
            "baseline": {"view_directory_entries": 1},
            "after": {"view_directory_entries": 1,
                      "read_view_cache_bytes_peak": 10_000_000},
            "planned_seconds": contract.L_DURATION_S}


def test_healthy_formal_l_passes():
    verdict = judge.judge_l_stage(healthy_l_artifact())
    assert verdict["reasons"] == [], verdict["reasons"]


def test_formal_l_rejects_rss_growth_and_duration_lie():
    artifact = healthy_l_artifact()
    series = artifact["rss_series"]
    for sample in series[-800:]:
        sample["rss_kib"] = 400_000  # > 5% growth over the first decile
    assert any("rss_growth" in r for r in judge.judge_l_stage(artifact)["reasons"])
    artifact = healthy_l_artifact()
    artifact["planned_seconds"] = 60
    assert any("l_duration" in r for r in judge.judge_l_stage(artifact)["reasons"])


def test_short_samples_still_carry_the_marker():
    window = healthy_p_window()
    window["formal"] = False
    reasons = judge.judge_p_window(window)["reasons"]
    assert reasons == ["short_sample_not_formal"]


# --- COMMANDS guard behavior -------------------------------------------------
COMMANDS = Path(__file__).resolve().parents[2] / \
    "docs/status/evidence/occ_freespace_final_20261102/implementation_g2/COMMANDS.sh"


def test_score_entry_refuses_without_authorization():
    if not COMMANDS.is_file():
        pytest.skip("gen2 COMMANDS.sh not present")
    import subprocess
    completed = subprocess.run(["bash", str(COMMANDS), "score"],
                               capture_output=True, text=True,
                               env={"PATH": "/usr/bin:/bin"})
    assert completed.returncode == 3
    assert "REFUSING" in completed.stderr
