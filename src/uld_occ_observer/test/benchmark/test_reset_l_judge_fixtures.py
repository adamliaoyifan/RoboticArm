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


#: The pristine artifact's honest findings (kept verbatim): the gen3 w59
#: gate violation PLUS the three gen4 evidence classes the run02 collector
#: predates (board trace, ledger loss counters, stage-end ledger table).
#: Missing evidence is a named failure, never a silent pass.
BASELINE_PROBLEMS = {"window_and:w59:['sched_late_max_107']",
                     "window_and_count:1",
                     "validity_trace_missing",
                     "ledger_counters_missing",
                     "ledger_end_table_missing"}


def _load():
    if not REAL_L.is_file():
        pytest.skip("run02 l.json not present on this machine")
    return json.loads(REAL_L.read_text())


def _synthetic_validity_trace(doc, drop_generation=None,
                              shift_round_linearization=None):
    """A well-formed trace derived from the artifact's OWN reset rounds,
    following the board's real numbering: `generation` is a TRANSITION
    counter (boot BUILDING=1, VALID=2, then each reset appends
    INVALID/BUILDING/VALID at +3 generations; 100 resets -> 1..302), with
    exactly len(rounds) INVALID events whose epochs run 2..101 and whose
    transitions land inside their round's [called, end] window."""
    trace = [{"generation": 1, "state": "BUILDING", "epoch": 1,
              "reason": "boot", "reset_id": "", "transition_mono_ns": 0},
             {"generation": 2, "state": "VALID", "epoch": 1,
              "reason": "store_ready", "reset_id": "",
              "transition_mono_ns": doc["reset_rounds_results"][0]
              ["started_mono_ns"]}]
    generation = 2
    for index, record in enumerate(doc["reset_rounds_results"]):
        called = record["reset_called_mono_ns"]
        # +5 s: past the round's sampling end (~1.7 s), so the transition
        # provably does NOT linearize inside its own round window.
        shift = 5 * 10**9 if index + 1 == shift_round_linearization else 0
        epoch = index + 2
        for state, offset in (("INVALID", 5), ("BUILDING", 6), ("VALID", 7)):
            generation += 1
            if generation == drop_generation:
                continue
            trace.append({"generation": generation, "state": state,
                          "epoch": epoch,
                          "reason": "reset" if state == "INVALID"
                          else "store_ready",
                          "reset_id": "l_reset_%d" % index
                          if state == "INVALID" else "",
                          "transition_mono_ns": called
                          + offset * 10**6
                          + (shift if state == "INVALID" else 0)})
    trace.sort(key=lambda e: (e["generation"], e["transition_mono_ns"]))
    return trace


def _inject_ledger_counters(doc, dropped_live=0):
    """Put gen4 loss counters on every flat snapshot that carries class
    fields (baseline, after, each round's nested samples get them via the
    top-level dicts the judge walks)."""
    doc.setdefault("baseline", {})["ownership_dropped_live"] = dropped_live
    doc["baseline"]["ownership_allocated_total"] = 10
    doc["baseline"]["ownership_released_total"] = 10
    doc.setdefault("after", {})["ownership_dropped_live"] = dropped_live
    doc["after"]["ownership_allocated_total"] = 11
    doc["after"]["ownership_released_total"] = 11


def _write(tmp_path, doc):
    run = tmp_path / "campaign"
    run.mkdir(exist_ok=True)
    (run / "l.json").write_text(json.dumps(doc))
    return run


def _problems_extra(verdict):
    """Problems beyond the known honest baseline findings."""
    return {p for p in verdict["problems"]
            if p not in BASELINE_PROBLEMS}


def test_pristine_baseline_named_gaps_plus_w59(tmp_path):
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


# ---- gen4 gates: board trace continuity + ledger no-loss / no-misjudge ----

def test_gen4_trace_generation_gap_fails(tmp_path):
    doc = _load()
    doc["validity_trace"] = _synthetic_validity_trace(doc, drop_generation=50)
    verdict = judge_module.judge(_write(tmp_path, doc))
    assert verdict["status"] == "fail"
    assert any("validity_trace_generation_gap" in p
               for p in verdict["problems"])


def test_gen4_trace_chain_incomplete_fails(tmp_path):
    """A missing VALID transition breaks the chain: round 29 (generation
    3*29+5=92) never provably recovered."""
    doc = _load()
    trace = _synthetic_validity_trace(doc)
    doc["validity_trace"] = [e for e in trace if e["generation"] != 92]
    verdict = judge_module.judge(_write(tmp_path, doc))
    assert verdict["status"] == "fail"
    assert any("validity_trace_generation_gap" in p
               for p in verdict["problems"])


def test_gen4_round_linearization_unproven_fails(tmp_path):
    """The INVALID transition must land inside its round's own window: a
    shifted transition is a gap in the raw linearization proof."""
    doc = _load()
    doc["validity_trace"] = _synthetic_validity_trace(
        doc, shift_round_linearization=41)
    verdict = judge_module.judge(_write(tmp_path, doc))
    assert verdict["status"] == "fail"
    assert any("reset_round_40_linearization_unproven" in p
               for p in verdict["problems"])


def test_gen4_ledger_drop_fails(tmp_path):
    doc = _load()
    doc["validity_trace"] = _synthetic_validity_trace(doc)
    _inject_ledger_counters(doc, dropped_live=3)
    verdict = judge_module.judge(_write(tmp_path, doc))
    assert verdict["status"] == "fail"
    assert any("ledger_dropped_live:3" in p for p in verdict["problems"])


def test_gen4_ledger_balance_break_fails(tmp_path):
    doc = _load()
    doc["validity_trace"] = _synthetic_validity_trace(doc)
    _inject_ledger_counters(doc)
    # Stage-end table disagrees with the counters by one live object.
    doc["after"]["ownership_trace"] = [
        {"object_class": "surface_buffer", "object_id": 9, "bytes": 72,
         "allocated_mono_ns": 5, "released_mono_ns": 0, "handle": "",
         "epoch": 3, "refcount": 1},
        {"object_class": "surface_buffer", "object_id": 10, "bytes": 72,
         "allocated_mono_ns": 6, "released_mono_ns": 0, "handle": "",
         "epoch": 4, "refcount": 1}]
    verdict = judge_module.judge(_write(tmp_path, doc))
    assert verdict["status"] == "fail"
    assert any("ledger_balance" in p or "ledger_end" in p
               for p in verdict["problems"])


def test_gen4_legal_holder_not_misjudged_as_leak(tmp_path):
    """§1: the cache entry's own surface buffer LIVE at stage end, with the
    cache entry present baseline..end, is a legal holder -- the judge must
    NOT add a leak problem for it (the run still fails on its honest w59,
    and on nothing ledger-related)."""
    doc = _load()
    doc["validity_trace"] = _synthetic_validity_trace(doc)
    _inject_ledger_counters(doc)
    doc["after"]["ownership_allocated_total"] = 12
    doc["after"]["ownership_released_total"] = 11
    doc["after"]["ownership_trace"] = [
        {"object_class": "surface_buffer", "object_id": 11, "bytes": 72,
         "allocated_mono_ns": 5, "released_mono_ns": 0, "handle": "",
         "epoch": 5, "refcount": 1}]
    # baseline/after cache entry present and equal (real data: 72 == 72).
    verdict = judge_module.judge(_write(tmp_path, doc))
    assert verdict["status"] == "fail"          # honest w59 stands
    assert not any("ledger_end" in p for p in verdict["problems"])
    assert verdict["ledger"]["end_live_surface_buffers"] == 1
    assert verdict["ledger"]["end_live_views"] == 0


def test_gen4_end_live_view_is_residual(tmp_path):
    """A live VIEW at stage end (all workers joined) is a residual, not a
    legal holder -- must be flagged."""
    doc = _load()
    doc["validity_trace"] = _synthetic_validity_trace(doc)
    _inject_ledger_counters(doc)
    doc["after"]["ownership_allocated_total"] = 12
    doc["after"]["ownership_released_total"] = 11
    doc["after"]["ownership_trace"] = [
        {"object_class": "view", "object_id": 12, "bytes": 3363936,
         "allocated_mono_ns": 5, "released_mono_ns": 0, "handle": "h",
         "epoch": 5, "refcount": 1}]
    verdict = judge_module.judge(_write(tmp_path, doc))
    assert verdict["status"] == "fail"
    assert any("ledger_end_live_views:1" in p for p in verdict["problems"])


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
