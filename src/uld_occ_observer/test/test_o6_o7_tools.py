"""Tool behavior for the O6 matrix and the O7 sustained ledger."""
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
BENCH = Path(__file__).resolve().parent / "benchmark"
sys.path.insert(0, str(BENCH))

import fusion_report  # noqa: E402
import o6_matrix  # noqa: E402
import sustained_run  # noqa: E402

EVAL = ROOT / "scripts" / "g3" / "occ_observer_eval.py"
NATIVE = os.environ.get("OBSERVER_EVAL_NATIVE")


def _scenes():
    scenes = []
    for hull in ("chamfer", "cuboid"):
        for resolution in (0.01, 0.02, 0.05):
            for boxes in (0, 5, 20):
                scenes.append({
                    "id": "r%s_b%s_%s" % (resolution, boxes, hull),
                    "resolution": resolution,
                    "boxes": boxes,
                    "hull": hull,
                    "geometry_file": "geometry_%s.yaml" % hull,
                    "config_file": "config_%s.yaml" % hull,
                })
    return scenes


def test_matrix_has_144_groups_and_both_hulls():
    groups = o6_matrix.official_matrix(_scenes())
    assert len(groups) == 144
    assert {group["hull"] for group in groups} == {"chamfer", "cuboid"}
    full = o6_matrix.select_benchmark_scenes({"scenes": _scenes()}, "full")
    assert len(full) == 144
    assert {item["scene"]["hull"] for item in full} == {"chamfer", "cuboid"}


def test_representative_selector_stays_on_one_load_pair():
    selected = o6_matrix.select_benchmark_scenes({"scenes": _scenes()}, "representative")
    assert selected
    assert {(item["outside"], item["cross"]) for item in selected} == {(0.5, 0.0)}
    assert {item["scene"]["hull"] for item in selected} == {"chamfer"}


def test_remainder_of_the_load_stays_inside_and_29_samples_do_not_pass():
    assert o6_matrix.quotas_for(10, 0.5, 0.5) == {"interior": 6, "cross": 2, "miss": 2}
    assert o6_matrix.quotas_for(30000, 0.9, 0.5)["cross"] == 13500
    short = fusion_report.exact_stats(list(range(29)), 30)
    assert short["status"] == "incomplete"
    assert fusion_report.within_gate(short, 500, 1000) is False
    exact = fusion_report.exact_stats([500.0] * 30, 30)
    assert fusion_report.within_gate(exact, 500, 1000) is True


def test_one_failed_group_is_not_averaged_away():
    groups = [{"id": "a", "status": "pass"}, {"id": "b", "status": "fail"}]
    verdict = o6_matrix.campaign_and(groups)
    assert verdict["verdict"] == "fail"
    assert verdict["pooled_latency"] == "not_computed"
    missing = o6_matrix.campaign_and([{"id": "a", "status": "pass"}])
    assert missing["verdict"] == "not_evaluated"


def test_fixture_manifest_does_not_hardcode_product_or_plan_sha(tmp_path):
    result = subprocess.run(
        [sys.executable, str(EVAL), "fixtures", "--out", str(tmp_path),
         "--resolutions", "0.01", "--boxes", "0", "--hulls", "chamfer,cuboid"],
        cwd=str(ROOT), capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stderr
    manifest = json.loads((tmp_path / "TEST_MANIFEST.json").read_text())
    assert "implementation_revision" not in manifest
    assert "plan_revision" not in manifest
    assert {scene["hull"] for scene in manifest["scenes"]} == {"chamfer", "cuboid"}


def test_schedule_counts_and_send_does_not_wait_for_completion():
    normal = sustained_run.normal_schedule()
    assert len(normal["inputs"]) == 240
    assert len(normal["queries"]) == 120
    overload = sustained_run.overload_schedule()
    assert len(overload["overload_inputs"]) == 600
    assert len(overload["recovery_inputs"]) == 60
    inputs = [{"planned": index * 0.5, "id": "n%d" % index} for index in range(4)]
    ledger = sustained_run.simulate_publish(inputs, complete_after_s=10.0)
    assert [ledger.sent["n%d" % index]["actual"] for index in range(4)] == [0.0, 0.5, 1.0, 1.5]


def test_rejection_duplicate_loss_and_drain_keep_a_single_count():
    rejected = sustained_run.simulate_publish(
        [{"planned": 0.0, "id": "a"}], 0.1, reject_ids={"a"}, duplicate_ids={"a"})
    summary = rejected.summary()
    assert summary["sent"] == 1
    assert summary["rejected"] == 1
    assert summary["success"] == 0
    assert summary["duplicate_receipts"] == 1
    assert summary["balanced"] is True
    lost = sustained_run.simulate_publish(
        [{"planned": 0.0, "id": "gone"}, {"planned": 0.5, "id": "kept"}], 0.1, drop_ids={"gone"}, drain_s=30)
    lost_summary = lost.summary()
    assert lost_summary["pending"] == 1
    assert lost_summary["success"] == 1
    assert lost_summary["balanced"] is True
    late = sustained_run.simulate_publish([{"planned": 0.0, "id": "late"}], complete_after_s=100, drain_s=30)
    assert late.summary()["pending"] == 1


def test_health_gap_must_cover_both_ends_and_equality_does_not_pass():
    assert sustained_run.health_covers([0.0, 0.4, 0.8, 1.2], 0.1, 1.0) is True
    assert sustained_run.health_covers([0.0, 1.0], 0.0, 1.0) is False
    assert sustained_run.health_covers([0.2, 0.6], 0.0, 1.0) is False


def test_overload_without_a_queue_is_not_evaluated_and_recovery_is_separate():
    ledger = sustained_run.Ledger()
    for index in range(600):
        ledger.send("o%d" % index, index / 10.0, index / 10.0, "overload")
        ledger.receipt("o%d" % index, "INTEGRATED", 0.0, 1.0, index / 10.0)
    for index in range(60):
        ledger.send("r%d" % index, index / 2.0, index / 2.0, "recovery")
        ledger.receipt("r%d" % index, "INTEGRATED", 0.0, 1.0, index / 2.0)
    queries = [{"status": "OK"} for _ in range(30)]
    hidden = sustained_run.judge_overload(ledger, queries, observed_queue_or_reject=False)
    assert hidden["status"] == "not_evaluated"
    seen = sustained_run.judge_overload(ledger, queries, observed_queue_or_reject=True)
    assert seen["status"] == "pass"


@pytest.mark.skipif(not NATIVE, reason="set OBSERVER_EVAL_NATIVE")
def test_native_self_check_and_flags_change_the_cloud(tmp_path):
    fixtures = subprocess.run(
        [sys.executable, str(EVAL), "fixtures", "--out", str(tmp_path / "fixtures"),
         "--resolutions", "0.01", "--boxes", "0,5", "--hulls", "chamfer,cuboid"],
        cwd=str(ROOT), capture_output=True, text=True, check=False)
    assert fixtures.returncode == 0, fixtures.stderr
    manifest = json.loads((tmp_path / "fixtures" / "TEST_MANIFEST.json").read_text())
    chamfer = next(scene for scene in manifest["scenes"] if scene["hull"] == "chamfer" and scene["boxes"] == 0)
    cuboid = next(scene for scene in manifest["scenes"] if scene["hull"] == "cuboid" and scene["boxes"] == 0)
    check = subprocess.run(
        [NATIVE, "--geometry", chamfer["geometry_file"], "--config", chamfer["config_file"],
         "--gate", "self-check", "--out", str(tmp_path / "self.json")],
        capture_output=True, text=True, check=False)
    assert check.returncode == 0, check.stderr
    assert json.loads((tmp_path / "self.json").read_text())["pass"] is True

    def counts(scene, cross):
        report = tmp_path / ("bench_%s_%s.json" % (scene["hull"], cross))
        result = subprocess.run(
            [NATIVE, "--geometry", scene["geometry_file"], "--config", scene["config_file"],
             "--gate", "bench", "--out", str(report), "--boxes", "0", "--cloud", "64",
             "--outside", "0.5", "--cross", str(cross), "--warmup", "0", "--repeat", "1"],
            capture_output=True, text=True, check=False)
        assert result.returncode == 0, result.stderr
        return json.loads(report.read_text())

    zero = counts(chamfer, 0.0)
    half = counts(chamfer, 0.5)
    assert zero["class_miss"] != half["class_miss"]
    assert zero["class_invalid"] == "0"
    assert counts(cuboid, 0.0)["pass"] is True
    first = tmp_path / "emit-a"
    second = tmp_path / "emit-b"
    for dest in (first, second):
        emitted = subprocess.run(
            [NATIVE, "--geometry", chamfer["geometry_file"], "--config", chamfer["config_file"],
             "--gate", "emit-window", "--out", str(dest), "--group", "r01_b00_chamfer_c64_o50_x00",
             "--boxes", "0", "--cloud", "64", "--outside", "0.5", "--cross", "0",
             "--warmup", "1", "--repeat", "2"],
            capture_output=True, text=True, check=False)
        assert emitted.returncode == 0, emitted.stderr
    left = json.loads((first / "load_manifest.json").read_text())
    right = json.loads((second / "load_manifest.json").read_text())
    assert left["frames"][0]["point_sha256"] == right["frames"][0]["point_sha256"]
    assert "implementation_revision" not in left
