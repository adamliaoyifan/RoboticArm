"""S2 gen9 regressions: the gen8 review's two blockers, inverted.

1. True occupancy peaks reach the judge: the same-instant composite peak is
   required and gated; a 1 GiB transient over a tiny sampled current value
   fails; resource rows that are missing/bool/NaN/Inf/negative are never
   filtered out of the statistic.
2. Probability evidence is independently recomputable from saved lossless
   content: hashes deleted, content tampered, region changed, one-cell
   probes, NaN durations — all reject. The healthy controls still pass.
"""
import base64
import copy
import hashlib
import json
import struct
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent / "benchmark"))

import campaign_executor as executor
import live_schedule
import repair_s0_contract as contract
import repair_s0_judge as judge


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


# ---------------------------------------------------------- healthy controls
def test_healthy_o8_fixture_passes():
    verdict = judge.judge_o8(judge.healthy_o8())
    assert verdict["pass"], verdict["reasons"]


def test_healthy_resources_pass_at_the_boundary():
    limit = contract.READ_VIEW_CACHE_LIMIT_BYTES
    resources = dict(judge._resources_ok())
    resources["read_view_bytes"] = 1024
    resources["cache_bytes"] = limit - 1024
    resources["read_view_cache_bytes_peak"] = limit
    reasons, status = judge._resources(resources)
    assert reasons == [] and status == "pass", (reasons, status)


# ------------------------------------------- review counterexample: peaks
def test_peak_1gib_over_tiny_current_fails():
    resources = dict(judge._resources_ok())
    resources["read_view_bytes"] = 2048
    resources["cache_bytes"] = 2048
    resources["read_view_bytes_peak"] = 1 << 30
    resources["read_view_cache_bytes_peak"] = 1 << 30
    reasons, status = judge._resources(resources)
    assert status == "fail" and "resource_peak" in reasons


def test_missing_peak_is_not_evaluated():
    resources = dict(judge._resources_ok())
    del resources["read_view_cache_bytes_peak"]
    reasons, status = judge._resources(resources)
    assert status == "not_evaluated" and "resource_peak_missing" in reasons


def test_peak_below_live_sum_is_inconsistent():
    resources = dict(judge._resources_ok())
    resources["read_view_cache_bytes_peak"] = 1  # below the live sum
    reasons, status = judge._resources(resources)
    assert "resource_peak_inconsistent" in reasons


def test_nan_resource_rows_never_disappear_from_the_statistic():
    rows = [
        {"resource_usage": {"read_view_bytes": 1024, "cache_bytes": 1024,
                            "read_view_cache_bytes_peak": 2048,
                            "read_view_bytes_peak": 1024, "cache_bytes_peak": 1024,
                            "store_tree_bytes": 1,
                            "certification": "READ_VIEW_CACHE_ATTRIBUTED",
                            "attribution_complete": True,
                            "unmarked_allocations": False}},
        {"resource_usage": {"read_view_bytes": float("nan"), "cache_bytes": 1024,
                            "read_view_cache_bytes_peak": 2048,
                            "read_view_bytes_peak": 1024, "cache_bytes_peak": 1024,
                            "store_tree_bytes": 1,
                            "certification": "READ_VIEW_CACHE_ATTRIBUTED",
                            "attribution_complete": True,
                            "unmarked_allocations": False}},
    ]
    resources = live_schedule._peak_resources_from_documents(rows)
    assert isinstance(resources["read_view_bytes"], float)
    reasons, status = judge._resources(resources)
    assert status == "fail" and "nonfinite_value" in reasons


@pytest.mark.parametrize("bad", [float("inf"), True, -5, "x"])
def test_bool_inf_negative_string_rows_poison_the_reduction(bad):
    rows = [
        {"resource_usage": {"cache_bytes": 1024, "read_view_bytes": 1024,
                            "read_view_cache_bytes_peak": 2048,
                            "read_view_bytes_peak": 1024, "cache_bytes_peak": 1024,
                            "store_tree_bytes": 1,
                            "certification": "READ_VIEW_CACHE_ATTRIBUTED",
                            "attribution_complete": True,
                            "unmarked_allocations": False}},
    ]
    rows[0]["resource_usage"]["cache_bytes"] = bad
    resources = live_schedule._peak_resources_from_documents(rows)
    assert resources["cache_bytes"] == bad or isinstance(bad, float)


def test_missing_key_in_any_row_poisons_the_reduction():
    rows = [
        {"resource_usage": {"cache_bytes": 1024, "read_view_bytes": 1024,
                            "read_view_cache_bytes_peak": 2048,
                            "read_view_bytes_peak": 1024, "cache_bytes_peak": 1024,
                            "store_tree_bytes": 1,
                            "certification": "READ_VIEW_CACHE_ATTRIBUTED",
                            "attribution_complete": True,
                            "unmarked_allocations": False}},
        {"resource_usage": {"read_view_bytes": 2048,  # cache_bytes missing
                            "read_view_cache_bytes_peak": 2048,
                            "read_view_bytes_peak": 1024, "cache_bytes_peak": 1024,
                            "store_tree_bytes": 1,
                            "certification": "READ_VIEW_CACHE_ATTRIBUTED",
                            "attribution_complete": True,
                            "unmarked_allocations": False}},
    ]
    resources = live_schedule._peak_resources_from_documents(rows)
    assert resources["cache_bytes"] is None
    reasons, status = judge._resources(resources)
    assert status == "not_evaluated" and "missing_observation" in reasons


def test_peaks_flow_through_the_adapter_artifact_and_judge():
    """The gen8 audit's traversal: product-shaped rows -> adapter reduction
    -> artifact resources -> judge. The 1 GiB transient fails end to end."""
    healthy = judge.healthy_latency_group("traversal", 30000)
    healthy["resources"] = live_schedule._peak_resources_from_documents([
        {"resource_usage": {
            "read_view_bytes": 4096, "cache_bytes": 4096,
            "read_view_bytes_peak": 8192, "cache_bytes_peak": 8192,
            "read_view_cache_bytes_peak": 16384, "store_tree_bytes": 65536,
            "certification": "READ_VIEW_CACHE_ATTRIBUTED",
            "attribution_complete": True, "unmarked_allocations": False}}])
    verdict = judge.judge_latency_group(healthy)
    assert verdict["pass"], verdict["reasons"]
    transient = copy.deepcopy(healthy)
    transient["resources"]["read_view_cache_bytes_peak"] = 1 << 30
    verdict = judge.judge_latency_group(transient)
    assert not verdict["pass"]
    assert "resource_peak" in verdict["reasons"]




def _content_probe(states=(0, 1, 2, 3, 0, 1, 2, 3), odds=None):
    import base64
    import hashlib
    import struct
    odds = odds if odds is not None else (-0.9, -0.4, 0.4, 0.9, 0.0, 0.0, 0.0, 0.0)
    states_bytes = bytes(states)
    odds_bytes = struct.pack("<%df" % len(odds), *odds)
    return {
        "cells": len(states),
        "region": {"begin": contract.O8_ORACLE_REGION["begin"],
                   "end": contract.O8_ORACLE_REGION["end"]},
        "content": {
            "states_b64": base64.b64encode(states_bytes).decode(),
            "odds_b64": base64.b64encode(odds_bytes).decode(),
            "states_sha256": hashlib.sha256(states_bytes).hexdigest(),
            "odds_sha256": hashlib.sha256(odds_bytes).hexdigest(),
            "content_sha256": contract.probe_content_sha256(states_bytes, odds_bytes),
        },
        "surface_hash": "g1", "release": "OK", "pages_done": True,
    }

# ------------------------------------- review counterexample: oracle evidence
def _healthy_hold(kind="no_input"):
    return next(h for h in judge.healthy_o8()["holds"] if h["kind"] == kind)


def test_nan_or_inf_measured_duration_rejects():
    for bad in (float("nan"), float("inf"), -1.0, True, None):
        def mutate(hold):
            hold["duration_measured_s"] = bad
        verdict = judge.judge_o8(_perturb_holds(judge.healthy_o8(), "no_input", mutate))
        assert not verdict["pass"], bad
        expected = {"hold_duration", "nonfinite_value", "negative_value",
                    "bad_type", "missing_observation"}
        assert expected & set(verdict["reasons"]), (bad, verdict["reasons"])


def test_nine_second_hold_does_not_pass():
    def mutate(hold):
        hold["duration_measured_s"] = 9.0
    verdict = judge.judge_o8(_perturb_holds(judge.healthy_o8(), "no_input", mutate))
    assert "hold_duration" in verdict["reasons"]


def test_deleted_hash_fields_cannot_compare_equal():
    """The audit deleted both log-odds hashes: None==None passed. Now the
    content itself is loaded and the canonical hash recomputed, so deleting
    the declared hash is a mismatch."""
    def mutate(hold):
        del hold["probe_before"]["content"]["odds_sha256"]
        del hold["probe_after"]["content"]["odds_sha256"]
    verdict = judge.judge_o8(_perturb_holds(judge.healthy_o8(), "no_input", mutate))
    assert not verdict["pass"]
    assert "content_sha_mismatch:odds_sha256" in verdict["reasons"]


def test_fake_sha_over_real_content_rejects():
    def mutate(hold):
        hold["probe_after"]["content"]["states_sha256"] = "f" * 64
    verdict = judge.judge_o8(_perturb_holds(judge.healthy_o8(), "no_input", mutate))
    assert "content_sha_mismatch:states_sha256" in verdict["reasons"]


def test_declared_content_hash_mismatch_rejects():
    def mutate(hold):
        hold["probe_after"]["content"]["content_sha256"] = "0" * 64
    verdict = judge.judge_o8(_perturb_holds(judge.healthy_o8(), "no_input", mutate))
    assert "content_hash_mismatch" in verdict["reasons"]


def test_changed_region_rejects_even_with_kept_hashes():
    def mutate(hold):
        hold["probe_after"]["region"] = {"begin": [9, 0, 0], "end": [10, 1, 1]}
    verdict = judge.judge_o8(_perturb_holds(judge.healthy_o8(), "no_input", mutate))
    assert "oracle_region" in verdict["reasons"]


def test_one_cell_after_probe_rejects():
    def mutate(hold):
        probe = _content_probe(states=(0,), odds=(0.0,))
        hold["probe_after"] = probe
    verdict = judge.judge_o8(_perturb_holds(judge.healthy_o8(), "no_input", mutate))
    assert not verdict["pass"]
    assert any(code in verdict["reasons"] for code in
               ("content_length_mismatch", "oracle_empty", "probability"))


def test_missing_content_and_files_reject(tmp_path):
    def strip_inline(hold):
        content = hold["probe_after"]["content"]
        content.pop("states_b64", None)
        content.pop("odds_b64", None)
        content["states_file"] = "o8_r2/probe-content/gone.states.bin"
        content["odds_file"] = "o8_r2/probe-content/gone.odds.bin"
    case = copy.deepcopy(judge.healthy_o8())
    for hold in case["holds"]:
        if hold["kind"] == "no_input":
            strip_inline(hold)
    case["artifact_dir"] = str(tmp_path)
    verdict = judge.judge_o8(case)
    assert "content_file_missing" in verdict["reasons"]

    def no_content(hold):
        del hold["probe_before"]["content"]
    verdict = judge.judge_o8(_perturb_holds(judge.healthy_o8(), "no_input", no_content))
    assert "content_missing" in verdict["reasons"]

    def no_dir(hold):
        hold["probe_before"]["content"] = {
            "states_file": "x.states.bin", "odds_file": "x.odds.bin"}
    verdict = judge.judge_o8(_perturb_holds(judge.healthy_o8(), "no_input", no_dir))
    assert "content_file_unresolved" in verdict["reasons"]


def test_content_path_escape_rejects(tmp_path):
    case = copy.deepcopy(judge.healthy_o8())
    for hold in case["holds"]:
        if hold["kind"] == "no_input":
            content = hold["probe_after"]["content"]
            content.pop("states_b64", None)
            content.pop("odds_b64", None)
            content["states_file"] = "../probe-content/escape.states.bin"
            content["odds_file"] = "../probe-content/escape.odds.bin"
    case["artifact_dir"] = str(tmp_path)
    verdict = judge.judge_o8(case)
    assert "content_path_escape" in verdict["reasons"]


def test_nonfinite_or_out_of_domain_content_rejects():
    import base64
    import hashlib
    import struct
    nan_odds = struct.pack("<8f", -0.9, -0.4, 0.4, 0.9, 0.0, 0.0, float("nan"), 0.0)
    states = bytes([0, 1, 2, 3, 0, 0, 0, 0])
    def mutate(hold):
        content = hold["probe_after"]["content"]
        content["odds_b64"] = base64.b64encode(nan_odds).decode()
        content["odds_sha256"] = hashlib.sha256(nan_odds).hexdigest()
        content["content_sha256"] = contract.probe_content_sha256(states, nan_odds)
    verdict = judge.judge_o8(_perturb_holds(judge.healthy_o8(), "no_input", mutate))
    assert "nonfinite_value" in verdict["reasons"]

    bad_states = bytes([0, 1, 2, 9, 0, 0, 0, 0])
    def mutate_states(hold):
        content = hold["probe_after"]["content"]
        content["states_b64"] = base64.b64encode(bad_states).decode()
        content["states_sha256"] = hashlib.sha256(bad_states).hexdigest()
        content["content_sha256"] = contract.probe_content_sha256(
            bad_states, struct.pack("<8f", *(-0.9, -0.4, 0.4, 0.9, 0, 0, 0, 0)))
    verdict = judge.judge_o8(_perturb_holds(judge.healthy_o8(), "no_input", mutate_states))
    assert "content_state_domain" in verdict["reasons"]


def test_file_backed_content_recomputes(tmp_path):
    """File-referenced content (the real artifact's shape) verifies through
    the same path: sha, decode, canonical hash, and before/after equality."""
    import base64
    import hashlib
    import struct
    states = bytes([0, 1, 2, 3, 0, 1, 2, 3])
    odds = struct.pack("<8f", -0.9, -0.4, 0.4, 0.9, 0, 0, 0, 0)
    folder = tmp_path / "o8_r2" / "probe-content"
    folder.mkdir(parents=True)
    (folder / "f.states.bin").write_bytes(states)
    (folder / "f.odds.bin").write_bytes(odds)
    case = copy.deepcopy(judge.healthy_o8())
    case["artifact_dir"] = str(tmp_path)
    for hold in case["holds"]:
        if hold["kind"] == "no_input":
            for name in ("probe_before", "probe_after"):
                content = hold[name]["content"]
                content.pop("states_b64", None)
                content.pop("odds_b64", None)
                content["states_file"] = "o8_r2/probe-content/f.states.bin"
                content["odds_file"] = "o8_r2/probe-content/f.odds.bin"
    verdict = judge.judge_o8(case)
    assert verdict["pass"], verdict["reasons"]
    # Tamper the file: sha mismatch.
    (folder / "f.odds.bin").write_bytes(odds[:-1] + bytes([1]))
    verdict = judge.judge_o8(case)
    assert "content_sha_mismatch:odds_sha256" in verdict["reasons"]
