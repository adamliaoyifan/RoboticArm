#!/usr/bin/env python3
"""Behavioral fixtures for the R pressure judge (offline, no ROS).

Every fixture copies the REAL run02 pressure directory and applies ONE
bounded mutation; the judge must reject each corruption and keep accepting
the pristine record. The three mutations reproduce the reviews' audit
verbatim (complete_review/audit.py): deleting every scored window, keeping
only one good recovery query while deleting the recovery inputs, and
fabricating first-read success over a TIMEOUT attempt -- all three passed
the pre-fix judge and must fail now.

Extra families (complete-evidence items 9/10): fake pressure (summary events
without raw rows), fake drain (open rows / late arrival), wrong frozen rate
or window length, missing raw window file, recovery input shortfall, failed
recovery query not filtered from statistics, first read beyond 2 s, epoch
discontinuity (implied reset), sweep summary/raw divergence.
"""
from __future__ import annotations

import json
import shutil
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import reset_pressure as judge_module  # noqa: E402

_ROOT = next(p for p in HERE.parents
             if (p / "docs/status/evidence").is_dir())
REAL = _ROOT / "docs/status/evidence/occ_reset_validity_20261103" \
    / "integ_run02/pressure"


@pytest.fixture()
def run(tmp_path):
    if not REAL.is_dir():
        pytest.skip("run02 pressure artifacts not present")
    dest = tmp_path / "integ_run02"
    dest.mkdir()
    shutil.copytree(REAL, dest / "pressure")
    shutil.copy(REAL.parent / "FREEZE_INTEG.json", dest / "FREEZE_INTEG.json")
    return dest / "pressure"


def _load(run, name):
    return json.loads((run / name).read_text())


def _save(run, name, doc):
    (run / name).write_text(json.dumps(doc))


def test_pristine_baseline_gen4(run):
    """Legacy run02 data under the gen4 judge: all mechanics recomputed and
    green, but the three evidence classes the run02 collector never wrote
    (offered-load identity, recovery input receipts, recovery health) are
    NAMED failures -- evidence incomplete is never a silent pass."""
    verdict = judge_module.judge(run)
    assert verdict["status"] == "fail"
    assert set(verdict["problems"]) == {"load_identity_missing",
                                        "recovery_input_receipts_missing",
                                        "recovery_health_missing"}
    assert verdict["frozen_rate_hz"] == 20.0
    assert verdict["first_valid_read_completion_s"] <= 2.0


def test_audit_no_windows(run):
    """Reviews audit case 1: all scored windows deleted -> still pass (bug)."""
    doc = _load(run, "pressure_score.json")
    doc["windows"] = []
    _save(run, "pressure_score.json", doc)
    for index in range(3):
        (run / ("score_w%d.json" % index)).unlink()
    verdict = judge_module.judge(run)
    assert verdict["status"] == "fail"
    assert any("missing_artifact:score_w" in p for p in verdict["problems"])


def test_audit_recovery_only_one_good_query(run):
    """Reviews audit case 2: one good query, rest TIMEOUT, inputs deleted."""
    doc = _load(run, "recovery.json")
    doc["inputs"] = []
    for query in doc["queries"][1:]:
        query["status"] = "TIMEOUT"
    _save(run, "recovery.json", doc)
    verdict = judge_module.judge(run)
    assert verdict["status"] == "fail"
    assert any("recovery_inputs_0" in p for p in verdict["problems"])
    assert any("recovery_bad_status" in p for p in verdict["problems"])


def test_audit_fake_first_read(run):
    """Reviews audit case 3: first_valid_read_s=0 over a TIMEOUT attempt."""
    doc = _load(run, "pressure_score.json")
    doc["recovery"]["first_read"] = {
        "first_valid_read_s": 0,
        "attempt": {"status": "TIMEOUT", "latency_ms": 100000}}
    _save(run, "pressure_score.json", doc)
    verdict = judge_module.judge(run)
    assert verdict["status"] == "fail"
    assert any("first_read" in p for p in verdict["problems"])


def test_fake_pressure_summary_without_raw(run):
    """Summary claims events; the raw rows say otherwise -> reject."""
    raw = _load(run, "score_w0.json")
    for row in raw["drain_full"]["timeline"]:
        row["queue_ms"] = 0.0
        row["queue_ms_max"] = 0.0
        for event in row.get("receipt_events") or []:
            event.pop("queue_ms", None)
    _save(run, "score_w0.json", raw)
    verdict = judge_module.judge(run)
    assert verdict["status"] == "fail"
    assert any("w0_without_pressure_event" in p
               or "w0_events_summary_mismatch" in p
               for p in verdict["problems"])


def test_fake_drain_open_row(run):
    raw = _load(run, "score_w1.json")
    row = raw["drain_full"]["timeline"][700]
    row["receipt_status"] = None
    row["receipt_count"] = 0
    row["receipt_events"] = []
    row["receipt_arrival_mono_ns"] = None
    _save(run, "score_w1.json", raw)
    verdict = judge_module.judge(run)
    assert verdict["status"] == "fail"
    assert any("w1_open_inputs" in p or "w1_in_flight_at_end" in p
               for p in verdict["problems"])


def test_fake_drain_late_arrival(run):
    raw = _load(run, "score_w2.json")
    drain = raw["drain_full"]
    late = drain["drain_start_ns"] + 45 * 10**9
    row = drain["timeline"][500]
    row["receipt_arrival_mono_ns"] = late
    row["receipt_events"] = [{"status": row.get("receipt_status")
                              or "INTEGRATED",
                              "arrival_mono_ns": late}]
    _save(run, "score_w2.json", raw)
    verdict = judge_module.judge(run)
    assert verdict["status"] == "fail"
    assert any("w2_drain_4" in p for p in verdict["problems"])


def test_wrong_frozen_rate_binding(run):
    doc = _load(run, "pressure_score.json")
    doc["frozen_rate_hz"] = 40.0
    _save(run, "pressure_score.json", doc)
    verdict = judge_module.judge(run)
    assert verdict["status"] == "fail"
    assert any("score_summary_frozen_rate_mismatch" in p
               for p in verdict["problems"])


def test_window_length_changed(run):
    raw = _load(run, "score_w0.json")
    raw["window_full"]["window_s"] = 30.0
    _save(run, "score_w0.json", raw)
    verdict = judge_module.judge(run)
    assert verdict["status"] == "fail"
    assert any("w0_window_s_30" in p for p in verdict["problems"])


def test_sweep_summary_raw_divergence(run):
    doc = _load(run, "sweep_result.json")
    doc["rates"][0]["pressure_events"] = [
        {"input_id": "fake", "kind": "queue", "queue_ms": 80.0}]
    _save(run, "sweep_result.json", doc)
    # Keep the freeze-before-score file ordering intact after the rewrite.
    for name in ("score_w0.json", "score_w1.json", "score_w2.json"):
        (run / name).touch()
    verdict = judge_module.judge(run)
    assert verdict["status"] == "fail"
    assert any("sweep_10hz_events_summary_mismatch" in p
               for p in verdict["problems"])


def test_recovery_input_shortfall(run):
    doc = _load(run, "recovery.json")
    doc["inputs"] = doc["inputs"][:-10]
    _save(run, "recovery.json", doc)
    verdict = judge_module.judge(run)
    assert verdict["status"] == "fail"
    assert any("recovery_inputs_65" in p for p in verdict["problems"])


def test_failed_recovery_query_not_filtered(run):
    doc = _load(run, "recovery.json")
    doc["queries"][30]["status"] = "TIMEOUT"
    _save(run, "recovery.json", doc)
    verdict = judge_module.judge(run)
    assert verdict["status"] == "fail"
    assert any("recovery_bad_status:[30]" in p for p in verdict["problems"])


def test_first_read_completion_beyond_bound(run):
    doc = _load(run, "recovery.json")
    summary = doc["summary"]
    attempt = summary["first_read"]["attempt"]
    attempt["started_mono_ns"] = summary["t0_ns"] + 5 * 10**9
    attempt["ended_mono_ns"] = attempt["started_mono_ns"] + 50 * 10**6
    attempt["latency_ms"] = 50.0
    summary["first_read"]["first_valid_read_s"] = 0.0
    _save(run, "recovery.json", doc)
    verdict = judge_module.judge(run)
    assert verdict["status"] == "fail"
    assert any("recovery_first_read:5.0" in p or
               "recovery_first_read" in p
               for p in verdict["problems"])


def test_epoch_discontinuity_means_reset(run):
    raw = _load(run, "score_w1.json")
    identity = raw["window_full"]["queries"][10].get("lease_identity")
    if identity is None:
        pytest.skip("no lease identity on this row")
    identity["epoch"] = 7
    _save(run, "score_w1.json", raw)
    verdict = judge_module.judge(run)
    assert verdict["status"] == "fail"
    assert any("epoch_discontinuity" in p for p in verdict["problems"])


def test_missing_sweep_raw(run):
    (run / "sweep_40hz.json").unlink()
    verdict = judge_module.judge(run)
    assert verdict["status"] == "fail"
    assert any("missing_artifact:sweep_40hz" in p
               for p in verdict["problems"])


# ---- gen4 gates: offered-load identity, perf lock, recovery P/H -----------

def _gen4_evidence(run):
    """Bring a legacy run02 copy up to gen4 evidence completeness so the new
    gates can be tested on their own merits."""
    import hashlib
    doc = _load(run, "recovery.json")
    inputs = doc["inputs"]
    t0 = doc["summary"]["window"]["t0_ns"]
    receipts = []
    for row in inputs:
        published = row["publish_mono_ns"]
        receipts.append({
            "input_id": row["input_id"],
            "receipt_status": "INTEGRATED", "receipt_count": 1,
            "receipt_events": [{"status": "INTEGRATED",
                                "arrival_mono_ns": published + 40 * 10**6}],
            "receipt_arrival_mono_ns": published + 40 * 10**6})
    doc["input_receipts"] = receipts
    doc["health_series"] = [
        {"stamp_ns": t0 + i * 500 * 10**6,
         "arrival_mono_ns": t0 + i * 500 * 10**6, "epoch": 1}
        for i in range(60)]
    _save(run, "recovery.json", doc)
    frames = [{"box_tag": 20, "name": "b20_f000.yaml", "sha256": "x"},
              {"box_tag": 20, "name": "b20_f001.yaml", "sha256": "y"}]
    (run / "load_identity.json").write_text(json.dumps({
        "frames_manifest": frames,
        "perf_lock": {"path": "/tmp/elfin-occ-backend-perf.lock",
                      "wait_s": 0.0, "acquired": True}}))


def test_gen4_full_evidence_passes(run):
    _gen4_evidence(run)
    verdict = judge_module.judge(run)
    assert verdict["status"] == "pass", verdict["problems"]
    assert verdict["load_box_tags"] == [20]
    assert verdict["recovery_input_e2e_p95_ms"] <= 350


def test_gen4_mixed_box_load_rejected(run):
    _gen4_evidence(run)
    doc = json.loads((run / "load_identity.json").read_text())
    doc["frames_manifest"][0]["box_tag"] = 0
    (run / "load_identity.json").write_text(json.dumps(doc))
    verdict = judge_module.judge(run)
    assert verdict["status"] == "fail"
    assert any("load_identity:not_20box" in p for p in verdict["problems"])


def test_gen4_lock_not_held_rejected(run):
    _gen4_evidence(run)
    doc = json.loads((run / "load_identity.json").read_text())
    doc["perf_lock"] = {"acquired": False}
    (run / "load_identity.json").write_text(json.dumps(doc))
    verdict = judge_module.judge(run)
    assert verdict["status"] == "fail"
    assert any("perf_lock:not_recorded_as_held" in p
               for p in verdict["problems"])


def test_gen4_recovery_input_open_rejected(run):
    _gen4_evidence(run)
    doc = _load(run, "recovery.json")
    doc["input_receipts"][10]["receipt_status"] = None
    doc["input_receipts"][10]["receipt_events"] = []
    _save(run, "recovery.json", doc)
    verdict = judge_module.judge(run)
    assert verdict["status"] == "fail"
    assert any("recovery_inputs_open:1" in p for p in verdict["problems"])


def test_gen4_recovery_send_late_rejected(run):
    _gen4_evidence(run)
    doc = _load(run, "recovery.json")
    doc["inputs"][20]["publish_mono_ns"] += 80 * 10**6
    _save(run, "recovery.json", doc)
    verdict = judge_module.judge(run)
    assert verdict["status"] == "fail"
    assert any("recovery_send_late_max" in p for p in verdict["problems"])


def test_gen4_recovery_health_gap_rejected(run):
    _gen4_evidence(run)
    doc = _load(run, "recovery.json")
    doc["health_series"][30]["arrival_mono_ns"] += 2 * 10**9
    doc["health_series"].sort(key=lambda s: s["arrival_mono_ns"])
    _save(run, "recovery.json", doc)
    verdict = judge_module.judge(run)
    assert verdict["status"] == "fail"
    assert any("recovery_health_gap_" in p for p in verdict["problems"])


# ---- perf lock behavioral checks (contention + abnormal release) ----------

def test_perf_lock_contention_and_release(tmp_path):
    """A second exclusive acquisition must block (contention observed), and
    an explicit unlock must free it again."""
    import fcntl
    handle, info = judge_module._perf_lock()
    assert info["acquired"] is True
    try:
        path = Path(info["path"])
        second = path.open("a+")
        try:
            fcntl.flock(second, fcntl.LOCK_EX | fcntl.LOCK_NB)
            blocked = False
        except OSError:
            blocked = True
        second.close()
        assert blocked, "second exclusive acquisition unexpectedly succeeded"
    finally:
        judge_module._perf_unlock(handle)
    handle2, info2 = judge_module._perf_lock()
    assert info2["acquired"] is True
    judge_module._perf_unlock(handle2)


def test_perf_lock_released_after_sigkill(tmp_path):
    """The abnormal-release path: a holder killed with SIGKILL must leave
    the lock reclaimable (fd closed by the OS)."""
    import fcntl
    import subprocess
    code = ("import sys, time; sys.path.insert(0, %r); "
            "import reset_pressure as J; "
            "h, i = J._perf_lock(); "
            "print(i['acquired'], flush=True); "
            "time.sleep(300)" % str(HERE))
    proc = subprocess.Popen([sys.executable, "-c", code],
                            stdout=subprocess.PIPE, text=True)
    assert proc.stdout.readline().strip() == "True"
    proc.kill()
    proc.wait(timeout=10)
    handle, info = judge_module._perf_lock()
    assert info["acquired"] is True
    judge_module._perf_unlock(handle)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
