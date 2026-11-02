"""gen2 tool-repair tests, realigned for gen3.

The 1621 review rejected the gen2 "formal healthy controls" as short of the
contract (one recovery query, two read rounds, a lone STALE overload query);
the contract-GRADE controls and gate rejections now live in
test_freespace_g3_offline.py. What stays here must keep holding:

- the original audit injections keep ADDING named failures under whatever
  judge version is current (the fixtures are deliberately pre-gen3-shaped);
- full_query's own fault paths against the injected fake service;
- the score entry stays closed without authorization.
"""
from __future__ import annotations

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


# --- the review's counterexamples, verbatim in spirit ------------------------
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


def test_candidate_digest_recomputes_from_raw_cells():
    cells = [(0, 1, 2, 3, 1), (90, 4, 5, 6, 2)]
    digest = judge.candidate_digest(cells)
    assert digest == judge.candidate_digest([list(cell) for cell in cells])
    assert judge.candidate_digest([(0, 1, 2, 3)]) is None  # not five numbers
    assert judge.candidate_digest([(0, "x", 2, 3, 1)]) is None
    assert judge.candidate_digest(None) == judge.candidate_digest([])


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
