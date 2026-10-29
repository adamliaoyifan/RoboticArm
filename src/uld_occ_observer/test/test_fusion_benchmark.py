"""Fusion breakdown runner checks. A missing profile CLI is a failure, not a skip."""
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest
import yaml

BENCH = Path(__file__).resolve().parent / "benchmark"
sys.path.insert(0, str(BENCH))
import fusion_benchmark  # noqa: E402
import fusion_report  # noqa: E402

BINARY = os.environ.get("OBSERVER_PROFILE_CLI")
STAGES = (
    "lock_wait", "validation", "fingerprint", "dedup", "depth_filter", "transform",
    "ray_preparation", "fusion", "finalization",
)


def _row(index=0, warmup=False, mode="stages", changed=1, tree="tree", content="content", sha="input"):
    enabled = mode != "off"
    profile = {
        "enabled": enabled,
        "mode": mode,
        "stage_visited": {stage: enabled for stage in STAGES},
        "stage_ms": {stage: (1.0 if enabled else 0.0) for stage in STAGES},
        "residual_ms": 0.2 if enabled else 0.0,
        "copied_blocks": 1 if enabled else 0,
        "copied_cells": 4 if enabled else 0,
        "fusion_breakdown": {
            "collected": enabled,
            "apply_batch_visited": True if enabled else None,
            "block_batch_visited": True if enabled else None,
            "apply_batch_ms": 0.4 if enabled else None,
            "block_batch_ms": 0.3 if enabled else None,
            "mutation_storage": "collected" if enabled else "not_collected",
            "mutation_size": changed if enabled else None,
            "mutation_capacity": max(changed, 1) if enabled else None,
            "mutation_element_bytes": 32 if enabled else None,
        },
    }
    document = {
        "index": index,
        "warmup": warmup,
        "mode": mode,
        "status": "OK",
        "input_sha256": sha,
        "known_cells": 10,
        "tree_nodes": 20,
        "block_count": 2,
        "integrate_external_ms": 4.0 + index / 1000.0,
        "acquire_ms": 0.1,
        "roi_ms": 0.1,
        "surface_first_call": {"external_ms": 0.2},
        "surface_second_call": {"external_ms": 0.05},
        "process_peak_rss_kib": 1000 + index,
        "probability_tree_sha256": tree,
        "receipt": {
            "status": "INTEGRATED",
            "raw_points": 3,
            "accepted_points": 3,
            "free_keys": 2,
            "hit_keys": 1,
            "changed_keys": changed,
            "occupancy_changed_keys": changed,
            "rebuilt_blocks": 1,
            "total_ms": 8.0,
            "preparation_ms": 1.0,
            "fusion_ms": 3.0,
            "epoch": 1,
            "map_revision": 1,
            "observed_revision": index + 1,
            "last_integrated_stamp_ns": 1000 + index,
            "geometry_hash": "g",
            "config_digest": "c",
            "clock_domain": "ros",
            "profile": profile,
        },
    }
    if content is not None:
        document["roi_surface_content_sha256"] = content
    return document


def _window(rows, mode="stages", require_content=False, status="ok"):
    return fusion_report.WindowResult("case", 0, mode, rows, status, require_content)


def test_unquoted_yaml_off_stays_the_off_mode():
    docs = fusion_benchmark.complete_documents("---\nmode: off\nreceipt:\n  profile:\n    mode: off\n")
    assert docs[0]["mode"] == "off"
    assert docs[0]["receipt"]["profile"]["mode"] == "off"


def test_p95_is_29th_of_30_and_median_averages_15th_and_16th():
    stats = fusion_report.official_stats(list(range(30)))
    assert stats["p95"] == 28
    assert stats["median"] == 14.5
    assert stats["max"] == 29
    incomplete = fusion_report.official_stats(list(range(29)))
    assert incomplete["status"] == "incomplete"
    assert "p95" not in incomplete


def test_29_scored_frames_are_incomplete():
    rows = [_row(index, warmup=False) for index in range(29)]
    summary = fusion_report.summarize_window(_window(rows), expect_scored=30)
    assert summary["status"] == "incomplete"
    assert "p95" not in summary.get("metrics", {})


def test_off_uncollected_phase_is_not_a_zero():
    frame = _row(mode="off", content=None)
    assert fusion_report.metric_value(frame, "apply_batch_ms") == fusion_report.NOT_COLLECTED
    assert fusion_report.metric_value(frame, "block_batch_ms") == fusion_report.NOT_COLLECTED
    assert fusion_report.metric_value(frame, "fusion_remainder_ms") == fusion_report.NOT_COLLECTED
    assert fusion_report.metric_value(frame, "legacy_receipt_fusion_ms") == 3.0
    assert fusion_report.mutation_fields(frame)["mutation_effective_bytes"] == fusion_report.NOT_COLLECTED


def test_require_content_rejects_a_missing_query_digest():
    missing = _window([_row(0, content=None)], require_content=True)
    present = _window([_row(0)], require_content=False)
    assert fusion_report.compare_frames(missing, present)["reason"] == "missing_content_digest"
    assert fusion_report.compare_frames(present, missing)["reason"] == "missing_content_digest"
    assert fusion_benchmark.repetition_count("baseline", None) == 1
    assert fusion_benchmark.repetition_count("stages", None) == 3
    assert fusion_benchmark.repetition_count("rehearsal", None) == 1
    assert fusion_benchmark.repetition_count("baseline", 3) == 3
    with pytest.raises(ValueError):
        fusion_benchmark.repetition_count("baseline", 0)
    assert fusion_benchmark.content_required(False, False) is False
    assert fusion_benchmark.content_required(False, True) is True
    assert fusion_benchmark.content_required(True, False) is True


def test_compare_rejects_incomplete_identity_and_detects_digest_mismatch():
    left = _window([_row(0), _row(1)], require_content=True)
    short = _window([_row(0)], require_content=True)
    assert fusion_report.compare_frames(left, short)["status"] == "not_comparable"
    bad_mode = _window([_row(0, mode="off")], mode="stages")
    assert fusion_report.compare_frames(bad_mode, bad_mode)["reason"] == "mode"
    hashed = _window([_row(0, sha="aaa")])
    other = _window([_row(0, sha="bbb")])
    assert fusion_report.compare_frames(hashed, other)["reason"] == "input_hash"
    assert fusion_report.compare_frames(left, _window([_row(0, tree="other"), _row(1)], require_content=True))["status"] == "mismatch"
    content = _window([_row(0, content="left")], require_content=True)
    assert fusion_report.compare_frames(content, _window([_row(0, content="right")], require_content=True))["field"] == "roi_surface_content_sha256"
    baseline = _window([_row(0, mode="off", content=None)], mode="off", require_content=False)
    candidate = _window([_row(0, mode="off", content="only-candidate")], mode="off", require_content=False)
    compared = fusion_report.compare_frames(baseline, candidate)
    assert compared["status"] == "comparable"
    assert compared["content_compared"] is False


@pytest.mark.parametrize("canonical", [False, True])
def test_structural_differences_require_two_canonical_digests(canonical):
    left, right = _row(tree="raw-left"), _row(tree="raw-right")
    right["tree_nodes"] = 10
    if canonical:
        left["canonical_probability_sha256"] = right["canonical_probability_sha256"] = "same"
    result = fusion_report.compare_frames(_window([left]), _window([right]))
    assert result["status"] == ("comparable" if canonical else "mismatch")
    if canonical:
        assert {item["field"] for item in result["structural_differences"]} == {
            "tree_nodes", "probability_tree_sha256"}
        assert result["canonical_frames"] == 1


@pytest.mark.parametrize("missing_left", [False, True])
def test_one_canonical_digest_is_not_comparable(missing_left):
    left, right = _row(), _row()
    (right if missing_left else left)["canonical_probability_sha256"] = "canonical"
    result = fusion_report.compare_frames(_window([left]), _window([right]))
    assert result["status"] == "not_comparable"
    assert result["reason"] == "missing_canonical_digest"


def test_canonical_comparison_preserves_probability_and_content_requirements():
    left, right = _row(), _row()
    left["canonical_probability_sha256"] = "left"
    right["canonical_probability_sha256"] = "right"
    assert fusion_report.compare_frames(_window([left]), _window([right]))["field"] == "canonical_probability_sha256"
    right["canonical_probability_sha256"] = "left"
    del right["roi_surface_content_sha256"]
    assert fusion_report.compare_frames(_window([left], require_content=True), _window([right]))["reason"] == "missing_content_digest"
    right["roi_surface_content_sha256"] = "different"
    assert fusion_report.compare_frames(_window([left]), _window([right]))["field"] == "roi_surface_content_sha256"
    right["roi_surface_content_sha256"] = left["roi_surface_content_sha256"]
    right["known_cells"] += 1
    assert fusion_report.compare_frames(_window([left]), _window([right]))["field"] == "known_cells"


def test_apply_breakdown_extraction_and_summary():
    rows = [_row(index) for index in range(30)]
    for row in rows:
        row["receipt"]["profile"]["apply_breakdown"] = dict(
            collected=True, write_loop_ms=0.2, inner_update_ms=0.1, prune_ms=0.05,
            keys_considered=10, unchanged_keys=4, written_keys=6)
    summary = fusion_report.summarize_window(_window(rows), expect_scored=30)
    for key in fusion_report.APPLY_FIELDS:
        expected = rows[0]["receipt"]["profile"]["apply_breakdown"][key]
        assert summary["metrics"][key]["median"] == expected
        assert summary["frames_extracted"][0][key] == expected
        assert fusion_report.metric_value(_row(mode="off"), key) == "not_collected"
        assert fusion_report.metric_value(_row(), key) == "not_collected"


def test_failed_window_is_not_averaged_away():
    assert fusion_report.campaign_status(["complete", "failed"]) == "failed"
    assert fusion_report.campaign_status(["complete", "complete"]) == "complete"
    assert "pooled" not in fusion_report.campaign_status(["complete", "mismatch"])


def test_order_and_budget_helpers():
    assert fusion_report.execution_order(0, 0) == ("A", "B")
    assert fusion_report.execution_order(0, 1) == ("B", "A")
    assert fusion_report.execution_order(1, 0) == ("B", "A")
    assert fusion_report.budget_allows(False, 100, 0) is True
    assert fusion_report.budget_allows(True, 100, 0) is False


def test_timeout_kills_process_group(tmp_path):
    script = tmp_path / "sleep.sh"
    pidfile = tmp_path / "child"
    script.write_text("#!/bin/sh\nsleep 60 &\necho $! > \"$1\"\nwait\n")
    script.chmod(0o755)
    result = fusion_benchmark.run_process_group([str(script), str(pidfile)], 0.4, tmp_path / "logs")
    assert result.timed_out
    deadline = time.monotonic() + 2
    child = int(pidfile.read_text().strip())
    while time.monotonic() < deadline:
        try:
            os.kill(child, 0)
        except ProcessLookupError:
            break
        time.sleep(0.05)
    else:
        os.kill(child, signal.SIGKILL)
        raise AssertionError("timed out child was still running")


def _fixture(root):
    descriptor = dict(schema_version=1, frame_id="container_link", length=0.8, width=0.8,
                      floor_z=0, ceiling_z=1.2, geometry_hash="fixture")
    data = json.dumps(descriptor, sort_keys=True, separators=(",", ":"))
    geometry = root / "geometry"
    geometry.write_text(yaml.safe_dump(dict(
        schema="uld_geometry_export_v1", descriptor_json=data,
        descriptor_sha256=hashlib.sha256(data.encode()).hexdigest())))
    config = root / "config"
    config.write_text("resolution_m: 0.05\ndepth_z_min_m: 0.52\ndepth_z_max_m: 6.0\nclock_domain: ros\n")
    case = root / "fixtures" / "tiny"
    case.mkdir(parents=True)
    frame = dict(sensor_id="test", observation_id="1", optical_frame="optical", clock_domain="ros",
                 geometry_hash="fixture", epoch=1, stamp_ns=1, points_m=[[0.2, 0.2, 0.8]],
                 transform=dict(stamp_ns=1, translation=[0, 0, 0], rotation=[1, 0, 0, 0, 1, 0, 0, 0, 1]))
    frame_path = case / "frame_0.yaml"
    frame_path.write_text(yaml.safe_dump(frame))
    frame["observation_id"] = "2"
    frame["stamp_ns"] = 2
    frame["transform"]["stamp_ns"] = 2
    second = case / "frame_1.yaml"
    second.write_text(yaml.safe_dump(frame))
    manifest = dict(case_id="tiny", mode="off", surface=True, state_digest=False, frames=[
        dict(path="frame_0.yaml", sha256=hashlib.sha256(frame_path.read_bytes()).hexdigest(), warmup=True),
        dict(path="frame_1.yaml", sha256=hashlib.sha256(second.read_bytes()).hexdigest(), warmup=False),
    ])
    (case / "manifest.yaml").write_text(yaml.safe_dump(manifest))
    return geometry, config, root / "fixtures"


def _exe(path):
    path.write_text("#!/bin/sh\nexit 0\n")
    path.chmod(0o755)


def test_validation_rejects_bad_hash_and_missing_official_case(tmp_path):
    geometry, config, fixtures = _fixture(tmp_path)
    binary = tmp_path / "cli"
    _exe(binary)
    suite = fusion_benchmark.FusionBenchmarkSuite(
        binary, binary, fixtures, geometry, config, tmp_path / "out", "baseline", lock_path=tmp_path / "lock")
    with pytest.raises(ValueError, match="o5_b0_c30000"):
        suite.validate_inputs()
    case = fixtures / "tiny"
    text = (case / "manifest.yaml").read_text().replace(
        hashlib.sha256((case / "frame_0.yaml").read_bytes()).hexdigest(), "0" * 64)
    (case / "manifest.yaml").write_text(text)
    rehearsal = fusion_benchmark.FusionBenchmarkSuite(
        binary, binary, fixtures, geometry, config, tmp_path / "rehearsal-out", "rehearsal",
        lock_path=tmp_path / "lock2")
    with pytest.raises(ValueError, match="hash mismatch"):
        rehearsal.validate_inputs()


def test_busy_lock_starts_no_window(tmp_path):
    geometry, config, fixtures = _fixture(tmp_path)
    binary = tmp_path / "cli"
    _exe(binary)
    lock = tmp_path / "held.lock"
    holder = subprocess.Popen(
        [sys.executable, "-c",
         "import fcntl,sys,time; f=open(sys.argv[1],'a'); fcntl.flock(f,fcntl.LOCK_EX); time.sleep(30)",
         str(lock)])
    try:
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            probe = open(lock, "a")
            try:
                import fcntl
                fcntl.flock(probe, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                probe.close()
                break
            fcntl.flock(probe, fcntl.LOCK_UN)
            probe.close()
            time.sleep(0.05)
        else:
            raise AssertionError("holder did not take the lock")
        suite = fusion_benchmark.FusionBenchmarkSuite(
            binary, binary, fixtures, geometry, config, tmp_path / "out", "rehearsal",
            lock_path=lock, lock_wait_s=0)
        report = suite.run()
        assert report["status"] == "lock_busy"
        assert report["pairs"] == []
        assert not (tmp_path / "out").exists()
    finally:
        holder.send_signal(signal.SIGTERM)
        holder.wait(timeout=5)


def test_rehearsal_records_frames_and_is_not_a_performance_sample(tmp_path):
    if not BINARY:
        raise AssertionError("OBSERVER_PROFILE_CLI is required")
    geometry, config, fixtures = _fixture(tmp_path)
    suite = fusion_benchmark.FusionBenchmarkSuite(
        BINARY, BINARY, fixtures, geometry, config, tmp_path / "out", "rehearsal",
        timeout_s=60, lock_path=tmp_path / "lock")
    report = suite.run()
    assert report["status"] == "not_a_performance_sample", report
    assert report["product_acceptance"] == "not_evaluated"
    assert report["performance_sample"] is False
    assert report["pairs"][0]["comparison"]["content_compared"] is True
    assert report["pairs"][0]["comparison"]["status"] == "comparable"
    samples = list((tmp_path / "out" / "windows").glob("*/samples.yaml"))
    assert len(samples) == 2
    stages = [path for path in samples if "_stages_" in path.name or path.parent.name.endswith("_stages_" + path.parent.name.split("_")[-1])]
    # Directory name is case_rRep_mode_tag.
    stages = [path for path in samples if "_stages_" in path.parent.name]
    document = list(yaml.safe_load_all(stages[0].read_text()))[0]
    fusion = document["receipt"]["profile"]["fusion_breakdown"]
    assert fusion["collected"] is True
    assert fusion["apply_batch_visited"] is True and fusion["block_batch_visited"] is True
    assert fusion["apply_batch_ms"] >= 0 and fusion["block_batch_ms"] >= 0
    assert fusion["mutation_storage"] == "collected"
    off = [path for path in samples if "_off_" in path.parent.name][0]
    off_row = list(yaml.safe_load_all(off.read_text()))[0]
    off_fusion = off_row["receipt"]["profile"]["fusion_breakdown"]
    assert off_fusion["collected"] is False
    assert off_fusion["apply_batch_ms"] is None
    assert off_fusion["mutation_size"] is None
    assert "roi_surface_content_sha256" in document
    assert document["content_digest_ms"] >= 0


@pytest.mark.parametrize("campaign", ["memory", "baseline", "stages", "rehearsal"])
def test_memory_alone_disables_manifest_state_digest(tmp_path, campaign):
    geometry, config, fixtures = _fixture(tmp_path)
    suite = fusion_benchmark.FusionBenchmarkSuite(
        "unused", "unused", fixtures, geometry, config, tmp_path / "out", campaign)
    case = fusion_benchmark._load_case(fixtures, "tiny", True)
    path = suite._write_manifest(case, "off", tmp_path / "window")
    manifest = yaml.safe_load(path.read_text())
    assert manifest["state_digest"] is (campaign != "memory")
    assert manifest["surface"] is True
    assert len(manifest["frames"]) == 2


@pytest.mark.parametrize("repetitions", [0, 2, 3, True])
def test_memory_rejects_wrong_repetitions(repetitions):
    with pytest.raises(ValueError):
        fusion_benchmark.repetition_count("memory", repetitions)
    assert fusion_benchmark.repetition_count("memory", None) == 1
    assert fusion_benchmark.repetition_count("memory", 1) == 1


def _memory_suite(tmp_path):
    geometry, config, fixtures = _fixture(tmp_path)
    source = yaml.safe_load((fixtures / "tiny" / "manifest.yaml").read_text())["frames"][0]
    for name in fusion_benchmark.OFFICIAL_CASES:
        folder = fixtures / name
        folder.mkdir()
        frames = [dict(source, path="../tiny/frame_0.yaml", warmup=index < 3) for index in range(33)]
        (folder / "manifest.yaml").write_text(yaml.safe_dump(dict(case_id=name, frames=frames)))
    binary = tmp_path / "cli"
    _exe(binary)
    return fusion_benchmark.FusionBenchmarkSuite(
        "unused-baseline", binary, fixtures, geometry, config, tmp_path / "out", "memory",
        lock_path=tmp_path / "lock")


def test_memory_runs_four_off_windows_without_semantic_comparison(tmp_path, monkeypatch):
    suite = _memory_suite(tmp_path)
    calls = []
    def process(argv, timeout, dest):
        manifest = yaml.safe_load(Path(argv[argv.index("--manifest") + 1]).read_text())
        assert manifest["state_digest"] is False
        assert manifest["surface"] is True
        assert manifest["mode"] == "off"
        assert [item["warmup"] for item in manifest["frames"]] == [True] * 3 + [False] * 30
        rows = [_row(index, warmup=index < 3, mode="off", content=None) for index in range(33)]
        for row in rows:
            del row["probability_tree_sha256"]
        Path(argv[argv.index("--out") + 1]).write_text(yaml.safe_dump_all(rows, explicit_start=True))
        (dest / "stdout.txt").write_text("")
        (dest / "stderr.txt").write_text("")
        calls.append(argv)
        return fusion_benchmark.ProcessResult(0, False, "", "", 0.01)
    monkeypatch.setattr(fusion_benchmark, "run_process_group", process)
    monkeypatch.setattr(suite, "compare_frames", lambda *args: pytest.fail("memory must not compare"))
    monkeypatch.setattr(fusion_benchmark, "libraries", lambda _: [])
    report = suite.run()
    assert len(calls) == 4
    assert report["status"] == "complete"
    assert report["pairs"] == []
    assert report["semantic_equivalence"] == "not_evaluated"
    assert report["product_acceptance"] == "not_evaluated"
    for window in report["windows"]:
        assert window["scored"] == 30
        assert window["process_peak_rss_kib"] == 1032
        assert window["semantic_equivalence"] == "not_evaluated"
        assert len(Path(window["sample_path"]).with_name("frames.jsonl").read_text().splitlines()) == 33
    assert suite.run()["status"] == "validation_failed"  # never overwrite


def test_memory_rejects_content_requirement_and_honors_budget(tmp_path, monkeypatch):
    suite = _memory_suite(tmp_path)
    suite.require_content = True
    assert suite.run()["status"] == "validation_failed"
    suite.require_content = False
    suite.budget_s = 0
    calls = []
    def run(binary, mode, case, repetition):
        calls.append(case["name"])
        return fusion_report.WindowResult(case["name"], repetition, mode,
                                         [_row(i, warmup=i < 3, mode="off") for i in range(33)])
    monkeypatch.setattr(suite, "run_window", run)
    monkeypatch.setattr(fusion_benchmark, "libraries", lambda _: [])
    report = suite.run()
    assert len(calls) == 1  # finish first window, refuse subsequent windows
    assert report["status"] == "not_evaluated"


def test_failure_replay_preserves_prefix_outputs_and_missing_checkpoints(tmp_path, monkeypatch):
    suite = _memory_suite(tmp_path)
    manifest = suite.validate_inputs()
    suite.out.mkdir()
    case = manifest.cases[0]
    dest = suite.out / "window"
    dest.mkdir()
    samples = dest / "samples.yaml"
    samples.write_text("preserved partial output")
    window = fusion_report.WindowResult(case["name"], 0, "off", [_row(0), _row(1)],
                                        sample_path=str(samples), binary=suite.candidate_cli)
    result = suite._write_failure_replay(case, 0, {"A": window},
                                         {"status": "mismatch", "frame": 1}, manifest)
    document = json.loads(Path(result["path"]).read_text())
    assert document["first_failing_index"] == 1
    assert [item["index"] for item in document["frames"]] == [0, 1]
    assert document["frames"][0]["sha256"] == case["frames"][0]["sha256"]
    assert document["windows"]["A"]["outputs"][0]["sha256"] == fusion_benchmark.sha256_file(samples)
    assert len(document["windows"]["A"]["replay_manifest"]["frames"]) == 2
    assert document["replay_possible"] is True
    assert document["capture_complete"] is False
    assert document["missing_artifacts"] == ["pre_checkpoint", "post_checkpoint"]
    assert document["budget"]["status"] == "within_limits"
    with pytest.raises(FileExistsError):
        suite._write_failure_replay(case, 0, {"A": window}, {}, manifest)
    monkeypatch.setattr(fusion_benchmark, "FAILURE_BYTES_LIMIT", 1)
    exceeded = suite._write_failure_replay(case, 1, {"A": window}, {}, manifest)
    assert exceeded["budget"] == "exceeded"
    assert suite._evidence_budget_exceeded
    assert samples.read_text() == "preserved partial output"


def test_failed_pairs_write_replay_and_stop_on_campaign_budget(tmp_path, monkeypatch):
    suite = _memory_suite(tmp_path)
    suite.campaign = "baseline"
    suite.baseline_cli = suite.candidate_cli
    calls = []
    def run(binary, mode, case, repetition):
        calls.append(case["name"])
        rows = [_row(i, warmup=i < 3, mode=mode) for i in range(33)]
        rows[0]["probability_tree_sha256"] = str(len(calls))
        return fusion_report.WindowResult(case["name"], repetition, mode, rows)
    monkeypatch.setattr(suite, "run_window", run)
    monkeypatch.setattr(fusion_benchmark, "libraries", lambda _: [])
    # Allow the input manifest and two windows, but not a replay manifest.
    original = suite._write_failure_replay
    def replay(*args):
        monkeypatch.setattr(fusion_benchmark, "CAMPAIGN_BYTES_LIMIT", 1)
        return original(*args)
    monkeypatch.setattr(suite, "_write_failure_replay", replay)
    report = suite.run()
    assert len(calls) == 2
    assert report["status"] == "failed"
    assert report["pairs"][0]["failure_replay"]["budget"] == "exceeded"
    assert report["pairs"][0]["comparison"]["status"] == "mismatch"


@pytest.mark.parametrize("latency", [True, False])
def test_reference_replay_is_complete_and_non_gating_but_mismatch_stops(tmp_path, monkeypatch, latency):
    suite = _memory_suite(tmp_path)
    suite.campaign = "baseline"
    suite.baseline_cli = suite.candidate_cli
    calls = []
    def run(binary, mode, case, repetition):
        calls.append(case["name"])
        rows = [_row(i, warmup=i < 3, mode=mode) for i in range(33)]
        if latency:
            for row in rows:
                row["roi_ms"] = 300.0
        else:
            rows[0]["probability_tree_sha256"] = str(len(calls))
        return fusion_report.WindowResult(case["name"], repetition, mode, rows)
    monkeypatch.setattr(suite, "run_window", run)
    monkeypatch.setattr(fusion_benchmark, "libraries", lambda _: [])
    report = suite.run()
    assert len(calls) == (8 if latency else 2)
    assert report["status"] == ("complete" if latency else "failed")
    assert report["product_acceptance"] == "not_evaluated"
    replay = report["pairs"][0]["failure_replay"]
    document = json.loads(Path(replay["path"]).read_text())
    assert replay["capture_complete"] is latency
    assert report["incomplete_failure_capture"] is (not latency)
    if latency:
        assert replay["trigger"] == "latency_reference"
        assert len(document["frames"]) == 33
        assert len(document["timing_evidence"]["A"]["frames"]) == 33
        assert document["first_failing_index"] == 3
        assert document["missing_artifacts"] == []
    else:
        assert document["missing_artifacts"] == ["pre_checkpoint", "post_checkpoint"]
        assert replay["replay_possible"] is True
