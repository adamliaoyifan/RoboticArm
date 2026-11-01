"""S2 gen8 regressions: the gen7 review's O8 misjudgments, inverted.

Each test perturbs a healthy artifact the way the review's audit did and
requires the judge to reject it; the healthy control must still pass. The
probability-oracle tests prove the hold comparison reads voxel states and
log-odds, not the surface geometry summary.
"""
import copy
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent / "benchmark"))

import campaign_executor as executor
import live_schedule
import repair_s0_contract as contract
import repair_s0_judge as judge

GEN7_ARTIFACT = Path(
    "/home/adamliao/work/elfin_humble_ws/docs/status/evidence/"
    "occ_octomap_repair_2cm_20261031/s2_gen7/stage_rehearsal/o8/runs.json")


def _perturbed(**changes):
    case = judge.healthy_o8()
    for key, value in changes.items():
        case[key] = value
    return case


def _perturb_runs(case, mutate):
    case = copy.deepcopy(case)
    for run in case["runs"]:
        mutate(run)
    return case


def _perturb_holds(case, kind, mutate):
    case = copy.deepcopy(case)
    for hold in case["holds"]:
        if hold["kind"] == kind:
            mutate(hold)
    return case


# ------------------------------------------------------------ healthy control
def test_healthy_o8_fixture_passes():
    verdict = judge.judge_o8(judge.healthy_o8())
    assert verdict["pass"], verdict["reasons"]


def test_gen7_saved_artifact_lacks_the_new_evidence_and_cannot_pass():
    """The gen7 artifact (surface-hash holds, no raw values) must be rejected:
    this is the acceptance control for upgrading the evidence shape."""
    if not GEN7_ARTIFACT.is_file():
        pytest.fail("gen7 artifact missing: %s" % GEN7_ARTIFACT)
    case = json.loads(GEN7_ARTIFACT.read_text())
    verdict = judge.judge_o8(case)
    assert not verdict["pass"]
    assert "saturation_frames" in verdict["reasons"] or \
        "probe_incomplete" in verdict["reasons"] or \
        "hold_duration" in verdict["reasons"]


# ------------------------------------------------- review counterexample 1-4
def test_zero_expected_frames_with_empty_receipts_fails():
    def mutate(run):
        run["saturation_frames_expected"] = 0
        run["saturation_frames_verified"] = 0
        run["saturation_receipts"] = []
        run["saturation_stamps"] = []
    verdict = judge.judge_o8(_perturb_runs(judge.healthy_o8(), mutate))
    assert not verdict["pass"]
    assert "saturation_frames" in verdict["reasons"]


@pytest.mark.parametrize("count,code", [(29, "saturation_frames"), (31, "saturation_frames")])
def test_wrong_frame_count_fails(count, code):
    def mutate(run):
        run["saturation_frames_expected"] = count
        cut = run["saturation_receipts"][:count]
        run["saturation_receipts"] = cut
        run["saturation_stamps"] = run["saturation_stamps"][:count]
    verdict = judge.judge_o8(_perturb_runs(judge.healthy_o8(), mutate))
    assert code in verdict["reasons"]


def test_boolean_expected_frames_fails():
    def mutate(run):
        run["saturation_frames_expected"] = True
    verdict = judge.judge_o8(_perturb_runs(judge.healthy_o8(), mutate))
    assert "saturation_frames" in verdict["reasons"]


def test_wrong_identities_all_same_id_fail():
    def mutate(run):
        for entry in run["saturation_receipts"]:
            entry["observation_id"] = "same-wrong-id"
        run["saturation_stamps"] = ["same-wrong-id"] * contract.O8_SATURATION_FRAMES
    verdict = judge.judge_o8(_perturb_runs(judge.healthy_o8(), mutate))
    assert "saturation_identity" in verdict["reasons"]


def test_receipt_identity_mismatching_the_sent_stamp_fails():
    def mutate(run):
        run["saturation_receipts"][5]["observation_id"] = "not-the-sent-stamp"
    verdict = judge.judge_o8(_perturb_runs(judge.healthy_o8(), mutate))
    assert "saturation_identity" in verdict["reasons"]


def test_receipt_identity_field_mismatch_fails():
    def mutate(run):
        run["saturation_receipts"][5]["receipt_observation_id"] = "someone-elses-id"
    verdict = judge.judge_o8(_perturb_runs(judge.healthy_o8(), mutate))
    assert "saturation_identity" in verdict["reasons"]


def test_wrong_epoch_or_sensor_fails():
    def mutate(run):
        run["saturation_receipts"][0]["epoch"] = 2
        run["saturation_receipts"][1]["sensor_id"] = "cam1"
    verdict = judge.judge_o8(_perturb_runs(judge.healthy_o8(), mutate))
    assert "saturation_identity" in verdict["reasons"]


def test_duplicate_receipt_events_fail():
    def mutate(run):
        run["saturation_receipts"][3]["events"] = 2
    verdict = judge.judge_o8(_perturb_runs(judge.healthy_o8(), mutate))
    assert "saturation_duplicate" in verdict["reasons"]


def test_unknown_disposition_fails():
    def mutate(run):
        run["saturation_receipts"][7]["disposition"] = "unknown"
    verdict = judge.judge_o8(_perturb_runs(judge.healthy_o8(), mutate))
    assert "saturation_receipt_rejected" in verdict["reasons"]


def test_aggregate_not_matching_receipts_fails():
    def mutate(run):
        run["saturation_probability_keys"] = 0
        run["saturation_receipts"][0]["changed_keys"] = 2
    verdict = judge.judge_o8(_perturb_runs(judge.healthy_o8(), mutate))
    assert "saturation_aggregate" in verdict["reasons"]


def test_duplicated_health_timestamps_fail():
    def mutate(run):
        run["node_health_times_s"] = [60.0] * contract.O8_HEALTH_MIN_SAMPLES
        run["node_health_values"] = [4321] * contract.O8_HEALTH_MIN_SAMPLES
    verdict = judge.judge_o8(_perturb_runs(judge.healthy_o8(), mutate))
    assert "node_health_duplicate" in verdict["reasons"]


def test_health_values_mismatching_stability_flag_fail():
    def mutate(run):
        run["node_health_values"] = [4321] * 100 + [4322] * 140
        run["node_count_stable"] = True
    verdict = judge.judge_o8(_perturb_runs(judge.healthy_o8(), mutate))
    assert "node_count" in verdict["reasons"]


def test_health_coverage_gap_fails():
    def mutate(run):
        # Covers 60-120s only: the 120-180s half of the window is missing.
        run["node_health_times_s"] = [round(60.0 + index * 0.5, 3) for index in range(120)]
        run["node_health_values"] = [4321] * 120
    verdict = judge.judge_o8(_perturb_runs(judge.healthy_o8(), mutate))
    assert "node_health_span" in verdict["reasons"]


def test_rss_medians_not_reproducible_from_raw_fail():
    def mutate(run):
        run["rss_median_kib"] = [1000, 1000, 1005]  # raw samples say 1004
    verdict = judge.judge_o8(_perturb_runs(judge.healthy_o8(), mutate))
    assert "rss_recompute" in verdict["reasons"]


def test_rss_raw_missing_fails():
    def mutate(run):
        del run["rss_samples"]
    verdict = judge.judge_o8(_perturb_runs(judge.healthy_o8(), mutate))
    assert "missing_observation" in verdict["reasons"]


def test_rss_empty_window_fails():
    def mutate(run):
        run["rss_samples"] = [[t, v] for t, v in run["rss_samples"] if t < 120]
        run["rss_window_counts"] = [30, 30, 0]
        run["rss_median_kib"] = [1000, 1000, 1000]
    verdict = judge.judge_o8(_perturb_runs(judge.healthy_o8(), mutate))
    assert "rss_span" in verdict["reasons"]


# ------------------------------------------------ hold / probe integrity
def _error_probe(marker="err"):
    return {"error": "cells_page_INTERNAL_ERROR", "cells": 0}


def test_internal_error_empty_surface_response_cannot_compare_equal():
    """The review injected status=INTERNAL_ERROR with empty arrays: the two
    error probes hashed equal and probability_unchanged passed. Now the
    probes are errors and both the adapter comparison and the judge reject."""
    before = _error_probe()
    after = _error_probe()
    assert live_schedule._probability_unchanged(before, after) is False
    def mutate(hold):
        hold["probe_before"] = dict(before)
        hold["probe_after"] = dict(after)
        hold["probability_unchanged"] = True  # a forged flag cannot help
    verdict = judge.judge_o8(_perturb_holds(judge.healthy_o8(), "no_input", mutate))
    assert "probe_incomplete" in verdict["reasons"]
    assert not verdict["pass"]


def test_empty_oracle_fails():
    def mutate(hold):
        hold["probe_after"]["cells"] = 0
        hold["probe_after"]["states_hash"] = ""
    verdict = judge.judge_o8(_perturb_holds(judge.healthy_o8(), "no_input", mutate))
    assert "oracle_empty" in verdict["reasons"]


def test_failed_release_or_incomplete_paging_fails():
    def mutate(hold):
        hold["probe_after"]["release"] = "INVALID_OR_EXPIRED_HANDLE"
    verdict = judge.judge_o8(_perturb_holds(judge.healthy_o8(), "no_input", mutate))
    assert "probe_incomplete" in verdict["reasons"]


def test_short_measured_hold_fails():
    def mutate(hold):
        hold["duration_measured_s"] = 0.0
    verdict = judge.judge_o8(_perturb_holds(judge.healthy_o8(), "no_input", mutate))
    assert "hold_duration" in verdict["reasons"]


# ------------------------------------------- probability oracle semantics
def test_probability_comparison_reads_states_and_log_odds_not_surface():
    before = {"cells": 100, "states_hash": "s1", "log_odds_hash": "o1",
              "surface_hash": "g1"}
    # Identical everything -> unchanged.
    assert live_schedule._probability_unchanged(before, dict(before)) is True
    # One cell's log-odds changed below every threshold: the surface summary
    # is identical, the probability comparison must differ.
    changed_odds = dict(before, surface_hash="g1", log_odds_hash="o2")
    assert live_schedule._probability_unchanged(before, changed_odds) is False
    # A state flip with the surface unchanged.
    changed_state = dict(before, surface_hash="g1", states_hash="s2")
    assert live_schedule._probability_unchanged(before, changed_state) is False
    # Surface changed but voxels identical: probability unchanged stays True
    # (the geometric summary is secondary).
    surface_only = dict(before, surface_hash="g2")
    assert live_schedule._probability_unchanged(before, surface_only) is True
    assert live_schedule._surface_unchanged(before, surface_only) is False


def test_judge_rejects_a_forged_flag_over_changed_voxel_probabilities():
    def mutate(hold):
        # Surface geometry identical (forged), voxel probability differs, and
        # the flag lies. The judge recomputes from the probe hashes.
        hold["probe_after"]["log_odds_hash"] = "odds-b2"
        hold["probability_unchanged"] = True
    verdict = judge.judge_o8(_perturb_holds(judge.healthy_o8(), "no_input", mutate))
    assert not verdict["pass"]
    assert "probability" in verdict["reasons"]


def test_judge_rejects_state_changes_under_identical_surface_for_occlusion():
    def mutate(hold):
        hold["probe_after"]["states_hash"] = "states-b2"
        hold["region_probability_unchanged"] = True
    verdict = judge.judge_o8(_perturb_holds(judge.healthy_o8(), "occlusion", mutate))
    assert "probability" in verdict["reasons"]


def test_occlusion_hold_still_requires_all_own_evidence():
    def strip(hold):
        hold["input_accepted"] = False
    verdict = judge.judge_o8(_perturb_holds(judge.healthy_o8(), "occlusion", strip))
    assert "occlusion_not_accepted" in verdict["reasons"]


# ------------------------------------------------------- adapter helpers
def test_probability_helpers_reject_error_and_mismatched_probes():
    error = {"error": "x"}
    good = {"cells": 10, "states_hash": "s", "log_odds_hash": "o"}
    assert live_schedule._probability_unchanged(error, dict(good)) is False
    assert live_schedule._probability_unchanged(dict(good), error) is False
    mismatched_cells = dict(good, cells=11)
    assert live_schedule._probability_unchanged(good, mismatched_cells) is False
    missing_hash = {"cells": 10, "log_odds_hash": "o"}
    assert live_schedule._probability_unchanged(good, missing_hash) is False
