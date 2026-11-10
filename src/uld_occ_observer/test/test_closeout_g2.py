"""Behavior tests for the O6/O7 closeout judge. Synthetic ledgers only."""
import json
import sys
from pathlib import Path

import pytest

BENCH = Path(__file__).resolve().parent / "benchmark"
sys.path.insert(0, str(BENCH))

import closeout_g2  # noqa: E402
import fusion_report  # noqa: E402
import surface_diag  # noqa: E402


def _ids():
    return closeout_g2.o6_matrix_ids()


def _row(value, index=0, warmup=False, sha="hash"):
    return {
        "index": index,
        "warmup": warmup,
        "status": "OK",
        "input_sha256": sha,
        "integrate_external_ms": value,
        "acquire_ms": 1.0,
        "roi_ms": 1.0,
        "process_peak_rss_kib": 1000,
        "surface_first_call": {"external_ms": 1.0},
        "surface_second_call": {"external_ms": 0.001},
        "receipt": {"status": "INTEGRATED"},
    }


def test_gates_match_the_frozen_reference_and_rank_29():
    assert closeout_g2.gates_match_reference()
    values = [float(index) for index in range(30)]
    stats = fusion_report.exact_stats(values, 30)
    assert stats["p95"] == values[closeout_g2.p95_index(30)] == 28.0
    short = closeout_g2.score_rows([_row(1.0, index=i) for i in range(29)], 30000, 30)
    assert short["status"] == "not_evaluated"
    assert short.get("pass") is not True


def test_threshold_equality_passes_and_one_p95_miss_fails():
    equal = closeout_g2.score_rows([_row(500.0, index=i) for i in range(30)], 30000, 30)
    assert equal["pass"] is True
    missed = [_row(1.0, index=i) for i in range(28)] + [_row(517.3, index=28), _row(527.1, index=29)]
    scored = closeout_g2.score_rows(missed, 30000, 30)
    assert scored["gates"]["integrate_external_ms"]["p95"] == 517.3
    assert scored["gates"]["integrate_external_ms"]["max"] == 527.1
    assert scored["pass"] is False


def test_partition_rejects_duplicates_missing_and_extra():
    plan = ["a", "b", "c"] + ["x"] * 141
    # plan itself is not 144 unique, so it is not a partition
    broken = closeout_g2.assign_sources(plan, ["a"], ["b"])
    assert broken["partition_ok"] is False
    ids = _ids()
    kept = ids[:28]
    measure = ids[28:]
    assert closeout_g2.assign_sources(ids, kept, measure)["partition_ok"] is True
    overlap = closeout_g2.assign_sources(ids, ids[:28], ids[27:])
    assert overlap["partition_ok"] is False
    assert ids[27] in overlap["duplicate"]
    hole = closeout_g2.assign_sources(ids, ids[:27], ids[28:])
    assert hole["partition_ok"] is False
    assert ids[27] in hole["missing"]
    evidence = Path("/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run03/selected_groups.json")
    selected = json.loads(evidence.read_text())
    recorded = closeout_g2.assign_sources(ids, selected["kept_from_run01"], selected["measure"])
    assert recorded["partition_ok"] is True
    assert len(selected["kept_from_run01"]) == 28
    assert len(selected["measure"]) == 116
    assert closeout_g2.REQUIRED_FAIL_ID in selected["kept_from_run01"]


def test_wrong_input_identity_and_rss_unknown_do_not_pass(tmp_path):
    group = tmp_path / "r01_b00_cuboid_c30000_o50_x50"
    (group / "warm").mkdir(parents=True)
    (group / "load").mkdir()
    (group / "cold").mkdir()
    frames = []
    docs = []
    for index in range(33):
        warmup = index < 3
        frames.append({
            "index": index, "warmup": warmup, "file_sha256": "expected-%02d" % index,
        })
        row = _row(100.0, index=index, warmup=warmup, sha="other-%02d" % index)
        docs.append(row)
    (group / "load" / "load_manifest.json").write_text(json.dumps({"frames": frames, "generator_version": "test"}))
    text = "".join("---\n" + _yaml_row(row) for row in docs)
    (group / "warm" / "samples.yaml").write_text(text)
    for index in range(3, 33):
        folder = group / "cold" / ("c%02d" % index)
        folder.mkdir()
        cold = _row(100.0, index=index, sha="cold")
        (folder / "samples.yaml").write_text("---\n" + _yaml_row(cold))
    (group / "group_verdict.json").write_text(json.dumps({
        "status": "pass",
        "cold_startup_to_first_sample_s": [0.4] * 30,
        "warm_latency": {"gates": [{"stats": {"p95": 100.0}}]},
    }))
    judged = closeout_g2.judge_group(group, 30000, "run01", {"product_revision": "abc", "tool_revision": "def"})
    assert judged["closeout_status"] == "not_evaluated"
    assert any(item.startswith("input_identity") for item in judged["reasons"])
    assert judged["read_view_and_cache_bytes"]["status"] == "not_evaluated"


def test_queries_and_recovery_keep_failures_in_the_denominator():
    ledger = {"sent": {}, "receipts": {}, "queries": [], "recovery_queries": []}
    for index in range(240):
        input_id = "s%03d" % index
        ledger["sent"][input_id] = {"planned": index * 0.5, "actual": float(index), "phase": "score"}
        if index == 5:
            continue
        status = "QUEUE_FULL" if index == 6 else "INTEGRATED"
        ledger["receipts"][input_id] = [{"status": status, "queue_ms": 0.01, "total_ms": 10.0, "received_at": float(index) + 0.01}]
        if index == 7:
            ledger["receipts"][input_id].append(dict(ledger["receipts"][input_id][0]))
    ledger["queries"] = [{"status": "ERROR", "acquire_ms": None, "roi_ms": None, "surface_ms": None} for _ in range(120)]
    normal = closeout_g2.judge_normal_window(ledger)
    assert normal["pass"] is False
    assert "input_conservation" in normal["reasons"]
    assert "query_conservation" in normal["reasons"]
    assert normal["success"] == 238
    assert len(normal["pending"]) == 1
    overload = {"sent": {}, "receipts": {}, "recovery_queries": [{"status": "STALE"}]}
    for index in range(600):
        input_id = "o%03d" % index
        overload["sent"][input_id] = {"planned": index * 0.1, "actual": float(index), "phase": "overload"}
        if index >= 523:
            continue
        overload["receipts"][input_id] = [{"status": "INTEGRATED", "queue_ms": 0.0, "total_ms": 1.0, "received_at": 1.0}]
    for index in range(60):
        input_id = "r%03d" % index
        overload["sent"][input_id] = {"planned": index, "actual": float(index), "phase": "recovery"}
        overload["receipts"][input_id] = [{"status": "INTEGRATED", "queue_ms": 0.0, "total_ms": 1.0, "received_at": 1.0}]
    overload["recovery_queries"] = [{"status": "STALE"}] + [{"status": "OK"} for _ in range(29)]
    judged = closeout_g2.judge_overload(overload)
    assert judged["status"] == "fail"
    assert "recovery_queries" in judged["reasons"]
    assert "overload_accounting" in judged["reasons"]
    assert judged["pending_count"] == 77
    assert judged["recovery_queries"]["contract_allows_stale"] is False
    assert judged["pending_unfinished"][0]["completion"] == "unknown"


def _yaml_row(row):
    return (
        "index: %d\nwarmup: %s\nstatus: %s\ninput_sha256: %s\n"
        "integrate_external_ms: %s\nacquire_ms: %s\nroi_ms: %s\nprocess_peak_rss_kib: %s\n"
        "surface_first_call:\n  external_ms: %s\nsurface_second_call:\n  external_ms: %s\n"
        "receipt:\n  status: %s\n"
    ) % (
        row["index"], "true" if row["warmup"] else "false", row["status"], row["input_sha256"],
        row["integrate_external_ms"], row["acquire_ms"], row["roi_ms"], row["process_peak_rss_kib"],
        row["surface_first_call"]["external_ms"], row["surface_second_call"]["external_ms"],
        row["receipt"]["status"],
    )


def test_surface_stages_stay_unknown_and_reads_stay_split():
    record = surface_diag.stage_record(140.0)
    assert record["lock_wait_ms"] == "unknown"
    assert record["surface_build_ms"] == "unknown"
    assert record["copy_convert_ms"] == "unknown"
    assert record["serialization_ms"] == "unknown"
    assert record["unknown_note"].endswith("not an integrate cost")
    assert "integrate_ms" not in record
    classes = surface_diag.classify_reads([
        {"map_revision": 3}, {"map_revision": 3}, {"map_revision": 4},
    ])
    assert len(classes["first"]) == 1
    assert len(classes["same_revision"]) == 1
    assert len(classes["revision_changed"]) == 1


def test_positive_recovery_and_complete_inputs_can_pass_their_own_gates():
    ledger = {"sent": {}, "receipts": {}, "queries": []}
    for index in range(240):
        input_id = "s%03d" % index
        ledger["sent"][input_id] = {"planned": index * 0.5, "actual": 1000.0 + index * 0.5, "phase": "score"}
        ledger["receipts"][input_id] = [{
            "status": "INTEGRATED", "queue_ms": 0.02, "total_ms": 50.0,
            "received_at": 1000.0 + index * 0.5 + 0.05,
        }]
    ledger["queries"] = [{
        "status": "OK", "acquire_ms": 1.0, "roi_ms": 1.0, "surface_ms": 20.0,
    } for _ in range(120)]
    assert closeout_g2.judge_normal_window(ledger)["pass"] is True
    overload = {"sent": {}, "receipts": {}, "recovery_queries": [{"status": "OK"} for _ in range(30)]}
    for index in range(600):
        input_id = "o%03d" % index
        overload["sent"][input_id] = {"planned": index * 0.1, "actual": float(index), "phase": "overload"}
        overload["receipts"][input_id] = [{"status": "INTEGRATED", "queue_ms": 0.0, "total_ms": 1.0, "received_at": 1.0}]
    for index in range(60):
        input_id = "r%03d" % index
        overload["sent"][input_id] = {"planned": index, "actual": float(index), "phase": "recovery"}
        overload["receipts"][input_id] = [{"status": "INTEGRATED", "queue_ms": 0.0, "total_ms": 1.0, "received_at": 1.0}]
    assert closeout_g2.judge_overload(overload)["status"] == "pass"
