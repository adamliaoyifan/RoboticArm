"""S2 gen4: strict capture judge, S0 window judge, and the real scheduler.

Live CLI/ROS smokes are non-scored. A short sample must not pass the formal judge.
"""
import json
import os
import sys
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent / "benchmark"))
import campaign_entry  # noqa: E402
import campaign_executor  # noqa: E402
import capture_replay  # noqa: E402
import repair_s0_contract as contract  # noqa: E402
import repair_s0_judge  # noqa: E402

FIXTURES = Path("/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run01/fixtures")
REVIEW = Path("/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_g3_review")
GEOMETRY = FIXTURES / "geometry_chamfer.yaml"
CONFIG = FIXTURES / "config_chamfer_r0p02.yaml"
CLI = os.environ.get(
    "OCC_S2_PROFILE_CLI",
    "/tmp/s2-install/uld_occ_observer/lib/uld_occ_observer/observer_profile_cli")
EVAL = os.environ.get(
    "OCC_S2_EVAL_NATIVE",
    "/tmp/s2-install/uld_occ_observer/lib/uld_occ_observer/observer_eval_native")
NODE = os.environ.get(
    "OCC_S2_NODE",
    "/tmp/s2-install/uld_occ_observer/lib/uld_occ_observer/observer_node")
OBS = "r02_b00_chamfer_c30000_o50_x00-0"


def _plan():
    return campaign_entry.load_plan(FIXTURES)


def _stage(result, name):
    return next(item for item in result["stages"] if item["stage"] == name)


def _trace_line(seq, event, request, status="", run="run-a", mono=None):
    return {
        "schema_version": 1,
        "run_id": run,
        "phase": "capture" if event == "trace_footer" else "writer",
        "event_seq": seq,
        "event": event,
        "mono_ns": seq if mono is None else mono,
        "pid": 1,
        "tid": 1,
        "thread_cpu_ns": seq if mono is None else mono,
        "request_id": request,
        "status": status,
    }


def _write_trace(path, events):
    path.write_text("".join(json.dumps(item) + "\n" for item in events))


def _healthy_events():
    expected = capture_replay.expected_profile_trace(OBS, run_id="run-a")
    events = [
        _trace_line(index + 1, item["event"], item["request_id"], item.get("status", ""))
        for index, item in enumerate(expected["operations"])
    ]
    events.append(_trace_line(len(events) + 1, "trace_footer", None, "0"))
    return expected, events


def test_footer_only_and_trace_identity_failures_are_rejected(tmp_path):
    expected = capture_replay.expected_profile_trace(OBS, run_id="run-a")
    footer = capture_replay.certify_trace(REVIEW / "footer_only.jsonl", expected)
    assert footer["capture_complete"] is False
    assert footer["replay_possible"] is False
    bare = capture_replay.certify_trace(REVIEW / "footer_only.jsonl")
    assert bare["capture_complete"] is False
    assert "missing_expected_operations" in bare["errors"]

    healthy_expected, events = _healthy_events()
    good = tmp_path / "good.jsonl"
    _write_trace(good, events)
    assert capture_replay.certify_trace(good, healthy_expected)["capture_complete"] is True

    def rejected(name, mutated):
        path = tmp_path / ("%s.jsonl" % name)
        _write_trace(path, mutated)
        verdict = capture_replay.certify_trace(path, healthy_expected)
        assert verdict["capture_complete"] is False, name
        assert verdict["replay_possible"] is False

    rejected("no-start", events[1:])
    rejected("no-surface", [item for item in events if item["event"] not in ("surface_begin", "surface_end")])
    replaced = [dict(item) for item in events]
    replaced[0]["request_id"] = "other-id"
    rejected("replaced-id", replaced)
    duplicated = [dict(events[0]), *events]
    for index, item in enumerate(duplicated, start=1):
        item["event_seq"] = index
    rejected("duplicate", duplicated)
    mixed = [dict(item) for item in events]
    mixed[3]["run_id"] = "run-b"
    rejected("mixed-run", mixed)
    negative = [dict(item) for item in events]
    negative[0]["mono_ns"] = -1
    rejected("negative-time", negative)


def test_review_windows_and_healthy_group_use_the_s0_judge():
    def samples(value):
        return {"integrate_ms": [value] * 30, "resource_certification": "pass"}

    over = campaign_entry.judge_recorded_window(samples(100000.0))
    assert over["pass"] is False
    assert "integrate" in over["reasons"]
    nan = campaign_entry.judge_recorded_window(samples(float("nan")))
    assert nan["pass"] is False
    assert "nonfinite_value" in nan["reasons"]
    infinite = campaign_entry.judge_recorded_window(samples(float("inf")))
    assert infinite["pass"] is False
    assert "nonfinite_value" in infinite["reasons"]
    boolean = campaign_entry.judge_recorded_window(samples(True))
    assert boolean["pass"] is False
    assert "bad_type" in boolean["reasons"]

    healthy = repair_s0_judge.healthy_latency_group(
        "r02_b00_chamfer_c30000_o50_x00", 30000,
        repair_s0_judge.parameters_from_id("r02_b00_chamfer_c30000_o50_x00"))
    assert campaign_entry.judge_recorded_window(healthy)["pass"] is True
    healthy["integrate_ms"] = [100000.0] * contract.SCORED_SAMPLES
    over_full = campaign_entry.judge_recorded_window(healthy)
    assert over_full["pass"] is False
    assert "integrate" in over_full["reasons"]
    healthy["integrate_ms"] = [float("nan")] * contract.SCORED_SAMPLES
    assert "nonfinite_value" in campaign_entry.judge_recorded_window(healthy)["reasons"]
    missing_resource = repair_s0_judge.healthy_latency_group("r02_b00_chamfer_c30000_o50_x00", 30000)
    missing_resource.pop("resources")
    resource = campaign_entry.judge_recorded_window(missing_resource)
    assert resource["pass"] is False
    assert "resource_missing" in resource["reasons"]


def test_scheduler_runs_the_matrix_and_stops_for_o5_budget_and_faults(tmp_path):
    plan = _plan()
    os.environ.pop("ULD_OCC_S3_OPEN", None)
    adapter = campaign_executor.OfflineAdapter()
    result = campaign_executor.schedule(plan, adapter, tmp_path / "full")
    assert result["official_samples"] == "not_run"
    assert result["product_resource_certification"] == "NOT_EVALUATED"
    assert result["formal_pass"] is False
    assert result["dependency_gate"] == "enforced"
    assert _stage(result, "o5")["started"] == 4 * 33
    assert _stage(result, "o6")["started"] == 96 * 33
    assert _stage(result, "o7")["started"] == 9 * 240 + 600 + 60
    assert _stage(result, "o8")["started"] == 3 * 30 + 3 + 3 + 3
    assert all(item["judge"]["pass"] for item in result["stages"])
    assert _stage(result, "o7")["recovery_first_query"] == "wait_snapshot"
    assert _stage(result, "o7")["overload"]["pass"] is True
    assert adapter.sender_calls == adapter.executor_calls
    assert campaign_executor.OfflineAdapter.ros_send is not campaign_executor.OfflineAdapter.ros_collect
    o5_ids = {case["id"] for case in plan["o5_cases"]}
    o6_ids = {group["id"] for group in plan["o6_groups"]}
    assert o5_ids <= o6_ids
    profile_ids = {item["group"] for item in adapter.profile_calls}
    assert profile_ids == o6_ids
    assert not any(item.startswith("r01_") for item in profile_ids)
    scored = {}
    for item in adapter.profile_calls:
        if item["warmup"]:
            continue
        scored[item["group"]] = scored.get(item["group"], 0) + 1
    assert set(scored) == o6_ids
    assert all(count == (60 if group in o5_ids else 30) for group, count in scored.items())
    assert len(adapter.hold_calls) == 9
    assert len(adapter.stability_calls) == 90

    stopped = campaign_executor.schedule(
        plan, campaign_executor.OfflineAdapter(slow_groups=o5_ids), tmp_path / "stop")
    assert _stage(stopped, "o5")["judge"]["pass"] is False
    assert _stage(stopped, "o6")["started"] == 0
    assert _stage(stopped, "o7")["started"] == 0
    assert _stage(stopped, "o6")["reason"] == "o5"
    assert _stage(stopped, "o8")["started"] == 3 * 30 + 3 + 3 + 3

    budget = campaign_executor.schedule(
        plan, campaign_executor.OfflineAdapter(clock=campaign_executor.ManualClock(step_s=3600)),
        tmp_path / "budget", budget_s=8 * 3600)
    assert _stage(budget, "o5")["started"] == 8
    assert _stage(budget, "o6")["started"] == 0
    assert _stage(budget, "o6")["reason"] == "budget"

    closed = campaign_executor.schedule(plan, campaign_executor.OfflineAdapter(), tmp_path / "s3", scored=True)
    assert closed["reason"] == "s3_closed"
    assert closed["started"] == {"o5": 0, "o6": 0, "o7": 0, "o8": 0}
    with pytest.raises(RuntimeError):
        campaign_executor.schedule(
            plan, campaign_executor.OfflineAdapter(), tmp_path / "skip",
            scored=True, allow_incomplete_dependencies=True)


def test_scheduler_faults_fail_the_formal_judge_without_dropping_counts(tmp_path):
    plan = _plan()
    one = campaign_executor.schedule(
        plan, campaign_executor.OfflineAdapter(fail_window="normal_b00_r1"), tmp_path / "one")
    campaign = _stage(one, "o7")["judge"]
    assert campaign["pass"] is False
    assert campaign["failed_windows"] == ["normal_b00_r1"]
    assert _stage(one, "o7")["started"] == 9 * 240 + 600 + 60

    short = campaign_executor.schedule(
        plan, campaign_executor.OfflineAdapter(), tmp_path / "short",
        stages=("o5",), frame_limit=1)
    assert short["formal_pass"] is False
    assert short["scoring"] == "non_scored"
    assert _stage(short, "o5")["judge"]["pass"] is False
    assert len(_stage(short, "o5")["judge"]["failed_groups"]) == 4
    saved = json.loads(next((tmp_path / "short" / "o5").glob("*.json")).read_text())
    assert "missing_observation" in repair_s0_judge.judge_case(saved)["reasons"]

    recovery = campaign_executor.schedule(
        plan, campaign_executor.OfflineAdapter(recovery_first="immediate"), tmp_path / "timeout")
    assert _stage(recovery, "o7")["recovery"]["pass"] is False
    assert "recovery_timeout" in _stage(recovery, "o7")["recovery"]["reasons"]

    wrong = campaign_executor.schedule(
        plan, campaign_executor.OfflineAdapter(recovery_fault="wrong_snapshot"), tmp_path / "wrong")
    assert "wrong_snapshot" in _stage(wrong, "o7")["recovery"]["reasons"]
    timed = campaign_executor.schedule(
        plan, campaign_executor.OfflineAdapter(recovery_fault="timeout"), tmp_path / "timed")
    assert "recovery_timeout" in _stage(timed, "o7")["recovery"]["reasons"]

    drained = campaign_executor.schedule(
        plan, campaign_executor.OfflineAdapter(inflight_after_drain=1), tmp_path / "drain")
    assert drained["stages"][2]["overload"]["pass"] is False
    assert "drain_inflight" in _stage(drained, "o7")["overload"]["reasons"]

    held = campaign_executor.schedule(
        plan, campaign_executor.OfflineAdapter(hold_ok=False), tmp_path / "hold")
    assert _stage(held, "o8")["judge"]["pass"] is False
    assert "probability" in _stage(held, "o8")["judge"]["reasons"]


def _epoch_bundle(tmp_path):
    if not Path(CLI).is_file():
        pytest.fail("profile CLI is not built: %s" % CLI)
    geometry_hash = json.loads(yaml.safe_load(GEOMETRY.read_text())["descriptor_json"])["geometry_hash"]
    frame = tmp_path / "epoch99.yaml"
    frame.write_text(
        "sensor_id: cam0\n"
        "observation_id: epoch-reject\n"
        "optical_frame: d555_color_optical_frame\n"
        "clock_domain: ros\n"
        "geometry_hash: %s\n"
        "epoch: 99\n"
        "stamp_ns: 1000000000\n"
        "points_m:\n"
        "- [0.0, 0.0, 1.0]\n"
        "transform:\n"
        "  stamp_ns: 1000000000\n"
        "  translation: [-1.15, 0.0, 1.2]\n"
        "  rotation: [0, 0, 1, -1, 0, 0, 0, -1, 0]\n" % geometry_hash)
    first = tmp_path / "first"
    first.mkdir()
    manifest = first / "manifest.yaml"
    manifest.write_text(yaml.safe_dump({
        "case_id": "epoch-reject",
        "mode": "off",
        "surface": False,
        "state_digest": False,
        "frames": [{
            "path": os.path.relpath(frame, first),
            "sha256": capture_replay.sha256_file(frame),
            "warmup": False,
            "expect_status": "EPOCH_MISMATCH",
        }],
    }, sort_keys=False))
    import subprocess
    samples = first / "samples.yaml"
    completed = subprocess.run(
        capture_replay.profile_command(CLI, str(GEOMETRY), str(CONFIG), manifest, samples),
        text=True, capture_output=True)
    assert completed.returncode == 0, completed.stderr
    row = next(yaml.safe_load_all(samples.read_text()))
    receipt = row["receipt"]
    bundle = {
        "observation_id": receipt["observation_id"],
        "epoch": receipt["epoch"],
        "input_epoch": 99,
        "status": receipt["status"],
        "reason": receipt["status"],
        "sensor_id": receipt["sensor_id"],
        "input_file": str(frame),
        "input_sha256": capture_replay.sha256_file(frame),
        "geometry_file": str(GEOMETRY),
        "geometry_sha256": capture_replay.sha256_file(GEOMETRY),
        "config_file": str(CONFIG),
        "config_sha256": capture_replay.sha256_file(CONFIG),
        "command": capture_replay.profile_command(CLI, str(GEOMETRY), str(CONFIG)),
        "tf": {"present": True, "stamp_ns": receipt["input_stamp_ns"], "source": "input_transform"},
        "map_prefix": {"tree_nodes": row["tree_nodes"]},
    }
    path = tmp_path / "bundle.json"
    path.write_text(json.dumps(bundle) + "\n")
    return path, bundle


def test_replay_child_compares_identity_and_rejects_tamper(tmp_path):
    path, bundle = _epoch_bundle(tmp_path)
    replayed = capture_replay.replay_failure(path, tmp_path / "replay")
    assert replayed["executed"] is True
    assert replayed["replay_possible"] is True
    assert replayed["file_reread_is_replay"] is False
    assert replayed["observed"]["pid"] != os.getpid()
    assert replayed["observed"]["status"] == "EPOCH_MISMATCH"
    assert replayed["fault_support"] == "whitelist"

    unsupported = dict(bundle, status="INTEGRATED")
    other = tmp_path / "unsupported.json"
    other.write_text(json.dumps(unsupported) + "\n")
    refused = capture_replay.replay_failure(other, tmp_path / "unsupported")
    assert refused["executed"] is False
    assert refused["replay_possible"] is False
    assert refused["fault_support"] == "not_evaluated"

    def tamper(name, **changes):
        payload = dict(bundle)
        payload.update(changes)
        dest = tmp_path / ("%s.json" % name)
        dest.write_text(json.dumps(payload) + "\n")
        result = capture_replay.replay_failure(dest, tmp_path / name)
        assert result["executed"] is True, name
        assert result["replay_possible"] is False, name
        return result["mismatches"]

    assert "sensor_id" in tamper("sensor", sensor_id="other-cam")
    assert "reason" in tamper("reason", reason="OTHER")
    assert "tf_stamp" in tamper("tf", tf={"present": True, "stamp_ns": 1})
    assert "tree_nodes" in tamper("prefix", map_prefix={"tree_nodes": 9})
    assert "content_sha256" in tamper(
        "content", map_prefix={"tree_nodes": bundle["map_prefix"]["tree_nodes"], "content_sha256": "deadbeef"})


def test_materialize_and_live_smokes_stay_non_scored(tmp_path):
    if not Path(EVAL).is_file() or not Path(CLI).is_file() or not Path(NODE).is_file():
        pytest.fail("install binaries are missing")
    plan = _plan()
    group = next(item["group"] for item in plan["o5_cases"] if item["id"] == "r02_b00_chamfer_c30000_o50_x00")
    emitted = campaign_executor.materialize_group(EVAL, str(GEOMETRY), str(CONFIG), group, tmp_path / "emit")
    assert emitted["materialized"] is True
    assert emitted["formal_scored"] is False
    assert emitted["sha256"]

    profile = campaign_executor.smoke_profile(
        plan, CLI, EVAL, str(GEOMETRY), str(CONFIG), tmp_path / "profile")
    assert profile["non_scored"] is True
    assert profile["formal_pass"] is False
    assert profile["parseable"] is True
    assert profile["processes"] == 1
    assert profile["surfaces"]["surface_first_ms"] is not None
    assert profile["surfaces"]["surface_repeat_ms"] is not None
    assert profile["surfaces"]["surface_after_update_ms"] is not None
    assert profile["judge"]["pass"] is False

    ros = campaign_executor.smoke_ros(plan, NODE, str(GEOMETRY), str(CONFIG), tmp_path / "ros")
    assert ros["non_scored"] is True
    assert ros["formal_pass"] is False
    assert ros["parseable"] is True
    assert ros["terminal"]["status_query"] == "COMPLETED"
    assert ros["terminal"]["receipt_status"] == "INTEGRATED"
    assert ros["terminal"]["receipt_received"] is True
    chain = ros["chain"]
    assert chain["wait_status"] == "OK"
    assert chain["surface_done"] is True
    assert chain["identity_stable"] is True
    assert chain["release_status"] == "OK"
    assert chain["age_within_2s"] is True
    assert ros["stages"][0]["judge"]["pass"] is False
    assert ros["sender_thread"] != ros["spin_thread"]
    assert ros["resource_certification"] in ("NOT_EVALUATED", "READ_VIEW_CACHE_ATTRIBUTED")
    assert ros["dependency_gate"] == "skipped_non_scored"


def test_live_campaign_wiring_stays_non_scored(tmp_path):
    if not Path(EVAL).is_file() or not Path(CLI).is_file() or not Path(NODE).is_file():
        pytest.fail("install binaries are missing")
    wired = campaign_executor.LiveCampaign(
        CLI, EVAL, NODE, str(GEOMETRY), str(CONFIG)).wire(_plan(), tmp_path)
    assert wired["official_samples"] == "not_run"
    assert wired["formal_pass"] is False
    assert wired["short_samples_rejected"] is True
    assert wired["wiring_complete"] is True
    assert wired["product_resource_certification"] == "NOT_EVALUATED"
    by_stage = {item["stage"]: item for item in wired["stages"]}
    assert by_stage["o5"]["processes"] == 1
    assert by_stage["o6"]["processes"] == 1
    assert by_stage["o5"]["started"] == 2
    assert by_stage["o5"]["sample_problems"] == []
    assert by_stage["o5"]["frame_receipts"] == ["INTEGRATED", "INTEGRATED"]
    assert by_stage["o5"]["judge"]["pass"] is False
    assert by_stage["o6"]["judge"]["pass"] is False
    assert by_stage["o7"]["judge"]["pass"] is False
    assert by_stage["o8"]["judge"]["pass"] is False
    assert by_stage["o7"]["chains_stable"] is True
    assert by_stage["o7"]["resource_certification"] in ("NOT_EVALUATED", "READ_VIEW_CACHE_ATTRIBUTED")
    assert by_stage["o7"]["sender"]["sender_thread"] != by_stage["o7"]["sender"]["spin_thread"]
    assert by_stage["o7"]["sender"]["planned_s"] == pytest.approx([0.0, 0.5])
    # gen6: the aggregate is labeled as both chains plus join, and per-request
    # boundaries exist alongside it.
    assert by_stage["o7"]["sender"]["chains_wall_s"] == by_stage["o7"]["sender"]["query_s"]
    assert len(by_stage["o7"]["sender"]["send_query_timeline"]) == 6
    assert by_stage["o8"]["health_valid"] is True
    assert (tmp_path / "failure").exists() is False
