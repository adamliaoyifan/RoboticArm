#!/usr/bin/env python3
"""Behavioral fixtures for the reset-contract L judge (offline, no ROS).

Every fixture is a real copy of the run02 l.json (or a real-data pressure of
it) with ONE bounded mutation; the judge must reject each corruption and
must keep accepting the honest artifacts. The baseline expectation itself is
part of the contract: the pristine run02 record FAILS on exactly one honest
finding (w59 sched_late_max 107 ms > 50 ms) -- never laundered to green.

Families (reviews 2026-11-04_1000 §1-3 and complete-evidence items 6/7/8):
all inputs REJECTED / all input IDs duplicated / health removed /
planned-only reset overlap / overrun 3 / missing planned ID / bad normal
terminal / scored non-normal status / owner evidence missing or corrupt /
unowned external byte / RSS drift / reset epoch broken / 0-extra healthy.
"""
from __future__ import annotations

import json
import shutil
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import reset_l_judge as judge_module  # noqa: E402

_ROOT = next(p for p in HERE.parents
             if (p / "docs/status/evidence").is_dir())
REAL_L = _ROOT / "docs/status/evidence/occ_reset_validity_20261103" \
    / "integ_run02/campaign/l.json"
#: The pristine artifact's single honest finding (kept verbatim; if this
#: baseline ever changes, the change must be explained, not absorbed).
BASELINE_PROBLEMS = {"window_and:w59:['sched_late_max_107']",
                     "window_and_count:1"}


def _load():
    if not REAL_L.is_file():
        pytest.skip("run02 l.json not present on this machine")
    return json.loads(REAL_L.read_text())


def _write(tmp_path, doc):
    run = tmp_path / "campaign"
    run.mkdir(exist_ok=True)
    (run / "l.json").write_text(json.dumps(doc))
    return run


def _problems_extra(verdict):
    """Problems beyond the known honest baseline finding."""
    return {p for p in verdict["problems"]
            if p not in BASELINE_PROBLEMS}


def test_pristine_baseline_single_honest_finding(tmp_path):
    verdict = judge_module.judge(_write(tmp_path, _load()))
    assert verdict["status"] == "fail"
    assert set(verdict["problems"]) == BASELINE_PROBLEMS
    assert verdict["windows_120s"] == {"expected": 55, "failed": 1}
    assert verdict["counts"]["extra_inputs"] == 2
    assert verdict["counts"]["extra_queries"] == 2


def test_zero_extra_still_judged_same(tmp_path):
    doc = _load()
    doc["query_records"] = [q for q in doc["query_records"]
                            if q["index"] < 14400]
    dropped = {e["input_id"] for e in doc["input_timeline"][-2:]}
    doc["input_timeline"] = doc["input_timeline"][:-2]
    doc["input_records"] = [e for e in doc["input_records"]
                            if e["input_id"] not in dropped]
    verdict = judge_module.judge(_write(tmp_path, doc))
    assert verdict["counts"]["extra_inputs"] == 0
    assert verdict["counts"]["extra_queries"] == 0
    assert set(verdict["problems"]) == BASELINE_PROBLEMS


def test_all_inputs_rejected_rejected(tmp_path):
    doc = _load()
    for row in doc["input_timeline"]:
        row["receipt_status"] = "REJECTED"
        row["receipt_events"] = [{"status": "REJECTED",
                                 "arrival_mono_ns":
                                 row["receipt_arrival_mono_ns"]}]
    verdict = judge_module.judge(_write(tmp_path, doc))
    assert verdict["status"] == "fail"
    assert any("not_terminal_REJECTED" in p or
               "without_reset_proof" in p
               for p in verdict["problems"])


def test_all_input_ids_duplicated(tmp_path):
    doc = _load()
    same = doc["input_timeline"][0]["input_id"]
    for row in doc["input_timeline"]:
        row["input_id"] = same
    verdict = judge_module.judge(_write(tmp_path, doc))
    assert verdict["status"] == "fail"
    assert any("duplicate" in p for p in verdict["problems"])


def test_health_removed(tmp_path):
    doc = _load()
    doc["health_series"] = []
    verdict = judge_module.judge(_write(tmp_path, doc))
    assert verdict["status"] == "fail"
    assert any("health_series_missing" in p for p in verdict["problems"])
    assert any("health_epoch_continuity" in p for p in verdict["problems"])


def test_planned_only_overlap_never_exempts(tmp_path):
    """Reviews §3: planned time inside the window with execution outside it
    must NOT be exempted -- the reset must provably win the race."""
    doc = _load()
    row = next(q for q in doc["query_records"]
               if q["status"] == "ACQUIRE_NO_DATA")
    row["started_mono_ns"] += 30 * 10**9
    row["ended_mono_ns"] += 30 * 10**9
    verdict = judge_module.judge(_write(tmp_path, doc))
    assert verdict["status"] == "fail"
    assert any("nonnormal_query_without_reset_proof" in p
               for p in verdict["problems"])


def test_overrun_three_queries(tmp_path):
    doc = _load()
    extra = json.loads(json.dumps(doc["query_records"][-1]))
    extra["index"] = 14402
    extra["planned_mono_ns"] += 500 * 10**6
    extra["started_mono_ns"] += 500 * 10**6
    extra["ended_mono_ns"] += 500 * 10**6
    doc["query_records"].append(extra)
    verdict = judge_module.judge(_write(tmp_path, doc))
    assert verdict["status"] == "fail"
    assert any("query_overrun" in p for p in verdict["problems"])


def test_missing_planned_input(tmp_path):
    doc = _load()
    row = doc["input_timeline"][9000]
    doc["input_timeline"] = [e for e in doc["input_timeline"]
                             if e["input_id"] != row["input_id"]]
    doc["input_records"] = [e for e in doc["input_records"]
                            if e["input_id"] != row["input_id"]]
    verdict = judge_module.judge(_write(tmp_path, doc))
    assert verdict["status"] == "fail"
    assert any("planned_inputs_missing" in p or
               "timeline_records_mismatch" in p
               for p in verdict["problems"])


def test_bad_normal_terminal(tmp_path):
    doc = _load()
    # A scored-segment input (no reset window anywhere near it).
    row = next(e for e in doc["input_timeline"]
               if e["receipt_status"] == "INTEGRATED"
               and e["planned_mono_ns"] > doc["input_timeline"][0]
               ["planned_mono_ns"] + 3600 * 10**9)
    row["receipt_status"] = None
    verdict = judge_module.judge(_write(tmp_path, doc))
    assert verdict["status"] == "fail"
    assert any("not_terminal_None" in p or "without_terminal" in p
               for p in verdict["problems"])


def test_scored_query_non_normal_status(tmp_path):
    doc = _load()
    row = next(q for q in doc["query_records"]
               if q["index"] == 10000)
    row["status"] = "STALE"
    verdict = judge_module.judge(_write(tmp_path, doc))
    assert verdict["status"] == "fail"
    assert any("nonnormal_query_without_reset_proof:10000" in p
               for p in verdict["problems"])


def test_owner_evidence_missing(tmp_path):
    doc = _load()
    doc["read_rounds_results"] = []
    verdict = judge_module.judge(_write(tmp_path, doc))
    assert verdict["status"] == "fail"
    assert any("read_rounds_0" in p for p in verdict["problems"])


def test_read_round_release_corrupt(tmp_path):
    doc = _load()
    doc["read_rounds_results"][500]["release_status"] = "LEAKED"
    verdict = judge_module.judge(_write(tmp_path, doc))
    assert verdict["status"] == "fail"
    assert any("read_round_release_500" in p for p in verdict["problems"])


def test_external_byte_without_owner(tmp_path):
    doc = _load()
    samples = doc["reset_rounds_results"][50]["ownership_after"] \
        ["resource_samples"]
    samples[-1]["external_surface_buffer_bytes"] = 4096
    verdict = judge_module.judge(_write(tmp_path, doc))
    assert verdict["status"] == "fail"
    assert any("external_byte_without_owner" in p
               for p in verdict["problems"])


def test_rss_growth_drift(tmp_path):
    doc = _load()
    for sample in doc["rss_series"][-600:]:
        sample["rss_kib"] = int(sample["rss_kib"] * 1.25)
    verdict = judge_module.judge(_write(tmp_path, doc))
    assert verdict["status"] == "fail"
    assert any("rss_growth" in p for p in verdict["problems"])


def test_reset_epoch_broken(tmp_path):
    doc = _load()
    doc["reset_rounds_results"][50]["epoch"] = 99
    verdict = judge_module.judge(_write(tmp_path, doc))
    assert verdict["status"] == "fail"
    assert any("reset_round_50_epoch" in p for p in verdict["problems"])


def test_missing_artifact(tmp_path):
    verdict = judge_module.judge(tmp_path / "nowhere")
    assert verdict["status"] == "fail"
    assert "missing_artifact:l.json" in verdict["problems"]


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
