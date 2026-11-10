"""gen4 qualification tests: the 1938 review's SEVEN bad cases must die with
NAMED reasons, the new fault classes must be rejected at every position
(first/middle/last), service-level lies must not survive the real
capture->artifact->disk-verdict path, the controlled counterexample must
prove checker discrimination, and the verdict framework must keep
not_evaluated honest (a missing source is never a pass).
"""
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace as S

import pytest

BENCH = Path(__file__).resolve().parent / "benchmark"
sys.path.insert(0, str(BENCH))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import freespace_campaign as campaign  # noqa: E402
import freespace_contract as contract  # noqa: E402
import freespace_judge as judge  # noqa: E402
import freespace_verdict as verdict  # noqa: E402
import test_freespace_g3_offline as g3  # noqa: E402

GEOMETRY_HASH = g3.GEOMETRY_HASH
MIB = 1024 * 1024
EVIDENCE = Path(__file__).resolve().parents[3] / "docs/status/evidence/" \
    "occ_freespace_final_20261102/implementation_g4"


# --- the review's seven cases, each dying with its own name ---------------
def test_review_seven_named_rejections():
    window = g3.full_p_window()
    for record in window["queries"]:
        record.update(status="ACQUIRE_NO_DATA", release_status=None)
    reasons = judge.judge_p_window(window)["reasons"]
    assert "q:status_ACQUIRE_NO_DATA" in reasons

    window = g3.full_p_window()
    for record in window["queries"]:
        record.update(candidates=[],
                      content_sha256=judge.candidate_digest([]),
                      feasible_total=100000)
    reasons = judge.judge_p_window(window)["reasons"]
    assert any("result_count_0_ne_feasible_total_100000" in r
               for r in reasons)
    assert any("result_count_3_ne_candidates_0" in r for r in reasons)

    window = g3.full_p_window()
    for record in window["queries"]:
        record["lease_identity"].update(epoch=-999)
    reasons = judge.judge_p_window(window)["reasons"]
    assert any("identity_epoch_out_of_range" in r for r in reasons)
    # a VALID but unbound epoch dies on the binding, not the range
    window = g3.full_p_window()
    for record in window["queries"]:
        record["lease_identity"].update(epoch=2)
    assert any("q:epoch_not_bound:2!=1" in r
               for r in judge.judge_p_window(window)["reasons"])

    artifact = g3.full_l_artifact()
    for record in artifact["query_records"]:
        record.update(age_ns=30_000_000_000)
    assert any(reason.startswith("lq:age_")
               for reason in judge.judge_l_stage(artifact)["reasons"])

    artifact = g3.full_l_artifact()
    artifact.pop("health_series")
    artifact.update(health_samples=0, health_gap_max_s=100000)
    assert any("l_health_series_missing" in reason
               for reason in judge.judge_l_stage(artifact)["reasons"])

    artifact = g3.full_l_artifact()
    for round_record in artifact["read_rounds_results"]:
        for sample in round_record["ownership_after"]["resource_samples"]:
            sample["external_surface_buffer_bytes"] = \
                sample.get("external_surface_buffer_bytes", 0) + 100 * MIB
    reasons = judge.judge_l_stage(artifact)["reasons"]
    assert any(reason.startswith("read_round_0_class_external_surface_buffer"
                                 "_bytes_residual")
               for reason in reasons)

    artifact = g3.full_l_artifact()
    for round_record in artifact["read_rounds_results"]:
        round_record.update(status="ACQUIRE_INTERNAL_ERROR",
                            release_status=None)
    assert any(reason.startswith("read_round_no_real_lease_0_")
               for reason in judge.judge_l_stage(artifact)["reasons"])


# --- new fault classes x position (first/middle/last) ----------------------
POSITIONS = ("first", "middle", "last")


def _pick(records, position):
    if position == "first":
        return records[0]
    if position == "last":
        return records[-1]
    return records[len(records) // 2]


def _rehash(record):
    record["content_sha256"] = judge.candidate_digest(record["candidates"])


QUERY_FAULTS = {
    "no_lease_scored": lambda record: record.update(
        status="ACQUIRE_NO_DATA", release_status=None),
    "count_ne_feasible": lambda record: record.update(
        feasible_total=record["feasible_total"] + 1),
    "duplicate_candidate": lambda record: (
        record["candidates"].append(record["candidates"][0]),
        _rehash(record)),
    "yaw_out_of_domain": lambda record: (
        record["candidates"].__setitem__(
            0, (45,) + tuple(record["candidates"][0][1:])),
        _rehash(record)),
    "cursor_skip": lambda record: record["pages_detail"][0].update(cursor=7),
    "epoch_not_bound": lambda record: record["lease_identity"].update(epoch=2),
    "age_over": lambda record: record.update(age_ns=int(1.2e9)),
    "page_not_done": lambda record: record["pages_detail"][0].update(
        done=False),
}


@pytest.mark.parametrize("position", POSITIONS)
def test_p_new_query_faults(position):
    for name, mutate in QUERY_FAULTS.items():
        window = g3.full_p_window()
        mutate(_pick(window["queries"], position))
        reasons = judge.judge_p_window(window)["reasons"]
        assert reasons, "%s at %s passed" % (name, position)


@pytest.mark.parametrize("position", POSITIONS)
def test_revision_regression_and_health_gap(position):
    window = g3.full_p_window()
    record = _pick(window["queries"], position)
    # a regression needs a neighbour: raise the picked revision so its
    # successor regresses, except at the last position where the picked
    # record itself regresses against its predecessor
    record["lease_identity"].update(
        observed_revision=0 if position == "last" else 99)
    assert any("observed_revision_regressed" in reason
               for reason in judge.judge_p_window(window)["reasons"])

    window = g3.full_p_window()
    series = window["health_series"]
    index = {"first": 1, "middle": len(series) // 2, "last": len(series) - 1}[
        position]
    for sample in series[index:]:
        sample["stamp_ns"] += int(2.0e9)
    reasons = judge.judge_p_window(window)["reasons"]
    assert any("health_gap" in reason for reason in reasons)

    window = g3.full_p_window()
    series = window["health_series"]
    series[index]["stamp_ns"] = series[index - 1]["stamp_ns"] - 1
    assert any("health_stamp_not_monotonic" in reason
               for reason in judge.judge_p_window(window)["reasons"])


@pytest.mark.parametrize("position", POSITIONS)
def test_l_new_round_faults(position):
    artifact = g3.full_l_artifact()
    round_record = _pick(artifact["read_rounds_results"], position)
    round_record.update(status="ACQUIRE_STALE", release_status=None)
    assert any(reason.startswith("read_round_no_real_lease_")
               for reason in judge.judge_l_stage(artifact)["reasons"])

    artifact = g3.full_l_artifact()
    g3._residual_class(_pick(artifact["read_rounds_results"], position),
                       "lease_view_block_bytes", 125520)
    assert any("class_lease_view_block_bytes_residual" in reason
               for reason in judge.judge_l_stage(artifact)["reasons"])

    artifact = g3.full_l_artifact()
    reset = _pick(artifact["reset_rounds_results"], position)
    reset["ownership_after_reset"]["retained_view_block_bytes"] = 512
    assert any("retained_view_block_bytes_survived_reset" in reason
               for reason in judge.judge_l_stage(artifact)["reasons"])

    artifact = g3.full_l_artifact()
    reset = _pick(artifact["reset_rounds_results"], position)
    reset.pop("ownership_before")
    assert any("before_missing" in reason
               for reason in judge.judge_l_stage(artifact)["reasons"])

    artifact = g3.full_l_artifact()
    artifact.pop("read_failed_attempts")
    assert any("read_failed_attempts_missing" in reason
               for reason in judge.judge_l_stage(artifact)["reasons"])

    artifact = g3.full_l_artifact()
    artifact.pop("identity_binding")
    assert any("identity_binding_missing" in reason
               for reason in judge.judge_l_stage(artifact)["reasons"])


def test_reset_gap_tolerates_only_warmup_no_data():
    # An ACQUIRE_NO_DATA query inside a recomputed reset gap of the WARMUP
    # is the honest just-reset answer; the same status in the scored window
    # is a finding.
    artifact = g3.full_l_artifact()
    reset = artifact["reset_rounds_results"][0]
    gap_start = reset["started_mono_ns"]
    warm = artifact["query_records"][0]
    warm.update(planned_mono_ns=gap_start + 1_000_000,
                status="ACQUIRE_NO_DATA", release_status=None)
    warm.pop("lease_identity", None)
    warm.pop("age_ns", None)
    reasons = judge.judge_l_stage(artifact)["reasons"]
    assert not any("lq:status_ACQUIRE_NO_DATA" in reason
                   for reason in reasons)

    artifact = g3.full_l_artifact()
    scored = artifact["query_records"][-1]
    scored.update(status="ACQUIRE_NO_DATA", release_status=None)
    assert any("lq:status_ACQUIRE_NO_DATA" in reason
               for reason in judge.judge_l_stage(artifact)["reasons"])


# --- service-level lies cannot pass the real capture->disk path ------------
class FakeQueryService:
    get_snapshot = "get"
    freespace = "query"
    release = "release"

    def __init__(self, mode="healthy", candidates=3):
        self.mode = mode
        self.candidates = candidates
        self.released = False

    def call(self, service, request, timeout=10.0):
        if service == "get":
            if self.mode == "acquire_refuses":
                return S(status="STALE")
            return S(status="OK", handle="1:1",
                     identity=S(**{"epoch": 1, "map_revision": 5,
                                   "observed_revision": 9,
                                   "geometry_hash": GEOMETRY_HASH,
                                   "last_integrated_stamp_ns": 10**9,
                                   "config_digest": "c" * 64,
                                   "clock_domain": "steady_clock"}),
                     age_ns=4 * 10**8)
        if service == "release":
            self.released = True
            return S(status="OK")
        identity = S(**{"epoch": 1, "map_revision": 5, "observed_revision": 9,
                        "geometry_hash": GEOMETRY_HASH,
                        "last_integrated_stamp_ns": 10**9,
                        "config_digest": "c" * 64,
                        "clock_domain": "steady_clock"})
        count = self.candidates
        feasible = count + 2 if self.mode == "lied_feasible" else count
        if self.mode == "wrong_epoch":
            identity = S(**{"epoch": 99, "map_revision": 5,
                            "observed_revision": 9,
                            "geometry_hash": GEOMETRY_HASH,
                            "last_integrated_stamp_ns": 10**9,
                            "config_digest": "c" * 64,
                            "clock_domain": "steady_clock"})
        return S(status="OK", identity=identity,
                 cell_x=list(range(count)), cell_y=[1] * count,
                 cell_z=[3] * count,
                 origin_x=[0.0] * count, origin_y=[0.0] * count,
                 origin_z=[0.0] * count, yaw_deg=[0] * count,
                 support_ratio=[1.0] * count, support_kind=[2] * count,
                 verify_begin=[0, 0, 0] * count,
                 verify_end=[15, 10, 10] * count,
                 region_begin=[0, 0, 0], region_end=[64, 40, 24],
                 repository_wait_ms=1.0, enumerate_ms=2.0,
                 response_fill_ms=1.5, server_total_ms=5.0,
                 next_cursor=feasible, done=True, feasible_total=feasible)


def _bounded_window(query_record):
    return {"stage": "p", "formal": False, "box_count": 5,
            "window_index": 0, "window_id": "b05_w0", "inputs": 2,
            "queries": 1, "update_hz": contract.UPDATE_HZ,
            "query_hz": contract.QUERY_HZ, "window_planned_s": 0.8,
            "identity_binding": g3.binding(),
            "frames_manifest": [{"name": "b5_0.yaml", "sha256": "a" * 64,
                                 "box_tag": 5}],
            "sender": [g3.sender_row(0), g3.sender_row(1)],
            "queries": [dict({"index": 0, "box": list(contract.F_BOXES[0]),
                              "planned_mono_ns": g3.T0,
                              "started_mono_ns": g3.T0 + 3_000_000,
                              "ended_mono_ns": g3.T0 + 100_000_000},
                             **query_record)],
            "errors": [],
            "input_timeline": [g3.input_row(0), g3.input_row(1)],
            "terminals": {}, "pending": [],
            "health_series": g3.health_series(2),
            "window_close_ms": 900.0}


@pytest.fixture
def patched_requests(monkeypatch):
    monkeypatch.setattr(campaign, "_snapshot_request", lambda x: x)
    monkeypatch.setattr(campaign, "_freespace_request", lambda *a: a)
    monkeypatch.setattr(campaign, "_release_request", lambda x: x)


def test_service_faults_reach_the_disk_verdict(tmp_path, patched_requests):
    campaign_dir = tmp_path / "rehearsal" / "campaign"
    campaign_dir.mkdir(parents=True)

    def write_windows(record):
        # one bounded window per box count: the P row checks the box set too
        for box in contract.P_BOX_COUNTS:
            window = _bounded_window(record)
            window.update({"box_count": box,
                           "window_id": "b%02d_w0" % box})
            (campaign_dir / ("p_b%02d_w0.json" % box)).write_text(
                json.dumps(window))

    record = campaign.full_query(FakeQueryService(), contract.F_BOXES[0])
    write_windows(record)
    document = verdict.judge_evidence_root(tmp_path)
    row = next(item for item in document["rows"]
               if item["row"] == "P.record_contract")
    assert row["status"] == "pass", row["reasons"]

    for mode in ("acquire_refuses", "wrong_epoch", "lied_feasible"):
        record = campaign.full_query(FakeQueryService(mode=mode),
                                     contract.F_BOXES[0])
        write_windows(record)
        document = verdict.judge_evidence_root(tmp_path)
        row = next(item for item in document["rows"]
                   if item["row"] == "P.record_contract")
        assert row["status"] == "fail", mode


# --- the controlled counterexample judge ------------------------------------
def _snapshot(**overrides):
    base = {"lease_view_block_bytes": 0,
            "lease_view_directory_entries": 0,
            "surface_cache_bytes": 2048, "read_view_bytes": 503296,
            "uncovered_allocations": False}
    base.update(overrides)
    return {"resource_samples": [dict(base) for _ in range(3)]}


def test_counterexample_judge_discriminates():
    healthy = {"baseline": _snapshot(),
               "residue_with_lease": _snapshot(
                   lease_view_block_bytes=125520,
                   lease_view_directory_entries=2),
               "after_release": _snapshot()}
    assert verdict.counterexample_problems(healthy) == []

    blind = {"baseline": _snapshot(),
             "residue_with_lease": _snapshot(),  # residue never reported
             "after_release": _snapshot()}
    assert any("residue_not_reported" in problem
               for problem in verdict.counterexample_problems(blind))

    leaked = {"baseline": _snapshot(),
              "residue_with_lease": _snapshot(
                  lease_view_block_bytes=125520,
                  lease_view_directory_entries=2),
              "after_release": _snapshot(  # never recovered
                  lease_view_block_bytes=125520,
                  lease_view_directory_entries=2)}
    assert any("lease_not_recovered" in problem
               for problem in verdict.counterexample_problems(leaked))

    smeared = {"baseline": _snapshot(),
               "residue_with_lease": _snapshot(
                   lease_view_block_bytes=125520,
                   lease_view_directory_entries=2,
                   client_held_view_block_bytes=512),
               "after_release": _snapshot()}
    assert any("residue_smear" in problem
               for problem in verdict.counterexample_problems(smeared))


# --- the verdict framework stays honest -------------------------------------
def test_verdict_not_evaluated_is_never_a_pass(tmp_path):
    document = verdict.judge_evidence_root(tmp_path)
    formal = {row["row"]: row["status"] for row in document["rows"]
              if row["group"] == "formal"}
    assert set(formal.values()) == {"not_evaluated"}
    assert document["overall"]["formal"] == "not_evaluated"
    assert "Formal.O5" in document["overall"]["not_evaluated_rows"]


def test_verdict_row_tamper_is_a_mismatch(tmp_path):
    document = verdict.judge_evidence_root(tmp_path)
    for row in document["rows"]:
        if row["row"] == "P.record_contract":
            row["status"] = "pass" if row["status"] != "pass" else "fail"
    (tmp_path / "verdict.json").write_text(json.dumps(document))
    proc = subprocess.run(
        [sys.executable, str(BENCH / "freespace_verdict.py"),
         str(tmp_path), "check"], capture_output=True, text=True)
    assert proc.returncode != 0
    assert "verdict_row_mismatch_P.record_contract" in proc.stdout


def test_row_table_bound_to_coverage_document():
    coverage = (EVIDENCE / "COVERAGE.md").read_text()
    for row_id, _group, _main, _evaluator, _note in verdict.ROW_TABLE:
        assert row_id in coverage, "%s missing from COVERAGE.md" % row_id


def test_red_baseline_artifact_recorded():
    path = EVIDENCE / "red_baseline" / "RESULTS_red.json"
    assert path.is_file(), "red baseline must be recorded before the repair"
    rows = json.loads(path.read_text())
    mutants = [row for row in rows if not row.get("control")]
    controls = [row for row in rows if row.get("control")]
    assert len(mutants) == 7 and all(row["accepted"] for row in mutants), \
        "the g3 judge must be recorded accepting all seven mutations"
    assert len(controls) == 2 and all(row["accepted"] for row in controls)
