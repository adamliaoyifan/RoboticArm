"""S2 gen6: wiring-level sampling judgment, boundaries, receipts, and failure honesty.

These are the 2026-11-01 review repairs: per-request timing fields, finite
non-negative validation that stands independent of the campaign judge, raw
receipt event counting with duplicate detection, the real publish boundary in
CLOCK_MONOTONIC, O8 executed requiring valid health/RSS, and failure manifests
computed from disk instead of asserted. Live entries hard-require the gen6
install under /tmp/s2g6-install and fail loudly when it is missing.
"""
import json
import math
import os
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent / "benchmark"))
import campaign_entry  # noqa: E402
import campaign_executor as executor  # noqa: E402
import repair_s0_judge  # noqa: E402

FIXTURES = Path("/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run01/fixtures")
INSTALL = os.environ.get("OCC_S2_INSTALL", "/tmp/s2g6-install")
CLI = os.environ.get(
    "OCC_S2_PROFILE_CLI",
    "%s/uld_occ_observer/lib/uld_occ_observer/observer_profile_cli" % INSTALL)
EVAL = os.environ.get(
    "OCC_S2_EVAL_NATIVE",
    "%s/uld_occ_observer/lib/uld_occ_observer/observer_eval_native" % INSTALL)
NODE = os.environ.get(
    "OCC_S2_NODE",
    "%s/uld_occ_observer/lib/uld_occ_observer/observer_node" % INSTALL)


def _plan():
    return campaign_entry.load_plan(FIXTURES)


def _fake_profile(samples, surfaces, receipts=None, receipt_status="INTEGRATED", processes=0):
    holder = SimpleNamespace(
        processes=processes,
        receipt_status=receipt_status,
        surfaces=surfaces,
        sample_problems=[],
        surface_problems=None,
        frame_receipts=receipts,
        run_cumulative=None)

    def run_cumulative(group, frame_count, dest):
        holder.processes += 1
        return samples

    holder.run_cumulative = run_cumulative
    return holder


SURFACE_KEYS = ("surface_first_ms", "surface_repeat_ms", "surface_after_update_ms")
GOOD_SAMPLES = [
    {"integrate_ms": 1.0, "surface_first_ms": 4.0, "surface_repeat_ms": 0.1},
    {"integrate_ms": 1.0, "surface_first_ms": 4.0, "surface_repeat_ms": 0.1},
]


@pytest.mark.parametrize("poison,problem", [
    (float("nan"), "nonfinite_value"),
    (float("inf"), "nonfinite_value"),
    (-1.0, "negative_value"),
    (True, "bad_type"),
    (None, "missing_observation"),
])
def test_wire_profile_executed_false_for_poisoned_surface(poison, problem, tmp_path):
    campaign = executor.LiveCampaign("u", "u", "u", "u", "u")
    surfaces = {"surface_first_ms": poison, "surface_repeat_ms": 1.0, "surface_after_update_ms": 1.0}
    campaign.profile = _fake_profile(GOOD_SAMPLES, surfaces, receipts=["INTEGRATED"] * 2)
    stage = campaign._wire_profile(_plan(), tmp_path, "o5", {})
    assert stage["executed"] is False
    assert {"field": "surface_first_ms", "problem": problem} in stage["sample_problems"]
    # The judge still rejects the short sample, and the sample count is intact:
    # the missing_group rejection does not mask the poisoned sample.
    assert stage["judge"]["pass"] is False
    assert stage["started"] == executor.WIRE_FRAMES


def test_wire_profile_executed_false_when_a_frame_receipt_is_rejected(tmp_path):
    campaign = executor.LiveCampaign("u", "u", "u", "u", "u")
    surfaces = {"surface_first_ms": 1.0, "surface_repeat_ms": 1.0, "surface_after_update_ms": 1.0}
    campaign.profile = _fake_profile(GOOD_SAMPLES, surfaces, receipts=["INTEGRATED", "REJECTED"])
    stage = campaign._wire_profile(_plan(), tmp_path, "o5", {})
    assert stage["executed"] is False
    assert stage["frame_receipts"] == ["INTEGRATED", "REJECTED"]


def test_wire_profile_recomputes_frame_problems_from_samples(tmp_path):
    """An adapter that does not self-report cannot mask a poisoned frame."""
    campaign = executor.LiveCampaign("u", "u", "u", "u", "u")
    surfaces = {"surface_first_ms": 1.0, "surface_repeat_ms": 1.0, "surface_after_update_ms": 1.0}
    poisoned = [
        {"integrate_ms": float("nan"), "surface_first_ms": 4.0, "surface_repeat_ms": 0.1},
        {"integrate_ms": 1.0, "surface_first_ms": 4.0, "surface_repeat_ms": 0.1},
    ]
    profile = _fake_profile(poisoned, surfaces, receipts=["INTEGRATED"] * 2)
    profile.sample_problems = []  # a silent adapter
    campaign.profile = profile
    stage = campaign._wire_profile(_plan(), tmp_path, "o5", {})
    assert stage["executed"] is False
    assert {"field": "integrate_ms", "problem": "nonfinite_value", "frame": 0} in stage["sample_problems"]


def test_healthy_wire_profile_still_executes_and_judge_rejects(tmp_path):
    campaign = executor.LiveCampaign("u", "u", "u", "u", "u")
    surfaces = {"surface_first_ms": 1.0, "surface_repeat_ms": 1.0, "surface_after_update_ms": 1.0}
    campaign.profile = _fake_profile(GOOD_SAMPLES, surfaces, receipts=["INTEGRATED"] * 2)
    stage = campaign._wire_profile(_plan(), tmp_path, "o5", {})
    assert stage["executed"] is True
    assert stage["sample_problems"] == []
    assert stage["judge"]["pass"] is False


def _message(observation_id="obs-1", sensor_id="cam0", **extra):
    return SimpleNamespace(
        observation_id=observation_id, sensor_id=sensor_id,
        status=extra.get("status", "INTEGRATED"),
        queue_ms=extra.get("queue_ms", 0.1), total_ms=extra.get("total_ms", 1.0),
        input_stamp_ns=extra.get("input_stamp_ns", 11), callback_enter_ns=extra.get("callback_enter_ns", 0))


def test_receipt_events_count_and_flag_duplicates():
    session = SimpleNamespace(receipt_events=[
        (100, _message()),
        (200, _message()),
        (300, _message(observation_id="obs-2")),
    ])
    events = executor._receipt_events(session, "obs-1")
    assert len(events) == 2
    assert executor._receipt_event_problems(events, "obs-1") == [
        {"event": "duplicate", "problem": "duplicate_terminal", "observation_id": "obs-1", "count": 2}]
    # A duplicated terminal must fail the formal window judge.
    rows = [{
        "input_id": "score-0", "phase": "score", "publish_end_ns": 1, "callback_enter_ns": 2,
        "disposition": "integrated", "receipt_received": True, "terminal_count": 2,
        "queue_age_ms": 0.1, "integrate_ms": 1.0, "observed_revision": 1,
    }]
    window = {"id": "w0", "kind": "normal", "inputs": 1, "queries": 0}
    artifact = executor._assemble_window(window, rows, [], None, None)
    reasons = repair_s0_judge.judge_o7_normal(artifact)["reasons"]
    assert "duplicate_terminal" in reasons


def test_receipt_events_flag_nonfinite_receipt_fields():
    session = SimpleNamespace(receipt_events=[(100, _message(queue_ms=float("nan")))])
    events = executor._receipt_events(session, "obs-1")
    problems = executor._receipt_event_problems(events, "obs-1")
    assert {"event": 0, "field": "queue_ms", "problem": "nonfinite_value",
            "observation_id": "obs-1"} in problems


def test_receipt_events_zero_callback_enter_maps_to_none():
    session = SimpleNamespace(receipt_events=[(100, _message(callback_enter_ns=0))])
    assert executor._receipt_events(session, "obs-1")[0]["callback_enter_ns"] is None
    session = SimpleNamespace(receipt_events=[(100, _message(callback_enter_ns=12345))])
    assert executor._receipt_events(session, "obs-1")[0]["callback_enter_ns"] == 12345


def test_o8_health_valid_requires_distinct_parseable_samples_and_rss():
    good = [{"health": {"stamp_ns": 1, "tree_nodes": 10}}, {"health": {"stamp_ns": 2, "tree_nodes": 10}}]
    assert executor._o8_health_valid(good, [1, 1]) is True
    assert executor._o8_health_valid(good, [1, None]) is False           # process exited
    assert executor._o8_health_valid([good[0]], [1, 1]) is False          # missing sample
    repeated = [{"health": {"stamp_ns": 1, "tree_nodes": 10}}] * 2
    assert executor._o8_health_valid(repeated, [1, 1]) is False           # repeated timestamp
    poisoned = [{"health": {"stamp_ns": 1, "tree_nodes": float("nan")}},
                {"health": {"stamp_ns": 2, "tree_nodes": 10}}]
    assert executor._o8_health_valid(poisoned, [1, 1]) is False           # bad node count
    unparsed = [{"health": None}, {"health": None}]
    assert executor._o8_health_valid(unparsed, [1, 1]) is False


def test_failure_manifest_is_computed_from_disk(tmp_path):
    out = tmp_path / "run"
    out.mkdir()
    bundle = executor._save_failure(out, "o5", "RuntimeError", {"error": "boom"})
    manifest = json.loads((bundle / "manifest.json").read_text())
    # Nothing was captured: the manifest must not claim completeness.
    assert manifest["capture_complete"] is False
    assert manifest["replay_possible"] is False
    assert manifest["missing"]
    assert manifest["file_count"] == 0

    stage = out / "o5" / "cumulative"
    stage.mkdir(parents=True)
    (stage / "samples.yaml").write_text("row: 1\n")
    (stage / "manifest.yaml").write_text(json.dumps({
        "case_id": "c", "frames": [{"path": "f000.yaml", "sha256": "a" * 64, "warmup": False}]}))
    bundle = executor._save_failure(out, "o5", "RuntimeError", {"error": "boom"})
    manifest = json.loads((bundle / "manifest.json").read_text())
    assert manifest["capture_complete"] is False          # cli.log is still missing
    assert manifest["missing"] == ["cumulative/cli.log"]
    assert manifest["replay_possible"] is True            # hashed input frames are persisted
    assert manifest["replay_inputs"] == ["o5/cumulative/manifest.yaml"]
    assert all("sha256" in entry for entry in manifest["files"])

    (stage / "cli.log").write_text("log\n")
    bundle = executor._save_failure(out, "o5", "RuntimeError", {"error": "boom"})
    manifest = json.loads((bundle / "manifest.json").read_text())
    assert manifest["capture_complete"] is True
    assert manifest["missing"] == []


def test_failure_manifest_bounds_details_without_hiding_them(tmp_path):
    out = tmp_path / "run"
    out.mkdir()
    executor._save_failure(out, "o7", "BigError", {"error": "x" * 20000})
    manifest = json.loads((out / "failure" / "o7" / "manifest.json").read_text())
    assert manifest["details"]["truncated"] is True
    assert len(manifest["details"]["text"]) == 8000


def test_wire_sample_problems_naming():
    problems = executor._wire_sample_problems({
        "surface_first_ms": float("inf"), "surface_repeat_ms": -2.0,
        "surface_after_update_ms": None})
    assert problems == [
        {"field": "surface_after_update_ms", "problem": "missing_observation"},
        {"field": "surface_first_ms", "problem": "nonfinite_value"},
        {"field": "surface_repeat_ms", "problem": "negative_value"},
    ]


def test_cloud_qos_matches_the_frozen_contract():
    pytest.importorskip("rclpy")
    qos = executor._cloud_publisher_qos()
    assert qos.depth == 32
    assert qos.history.name == "KEEP_LAST"
    assert qos.reliability.name == "RELIABLE"


def test_receipts_subscription_depth_is_1024():
    pytest.importorskip("rclpy")
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import s2_rehearsal
    source = Path(s2_rehearsal.__file__).read_text()
    assert 'self._receipt, 1024' in source
    assert "receipt_events" in source


def _gen6_binaries():
    missing = [path for path in (CLI, EVAL, NODE) if not Path(path).is_file()]
    if missing:
        pytest.fail("gen6 install missing: %s (build COMMANDS.sh first)" % missing)


def test_live_wire_reports_per_request_boundaries(tmp_path):
    """Live wiring smoke: per-request fields, real publish boundary, receipt events.

    Needs the freshly built /tmp/s2g6-install. Deselect by node id when the
    install is absent.
    """
    _gen6_binaries()
    plan = _plan()
    result = executor.LiveCampaign(CLI, EVAL, NODE, FIXTURES / "geometry_chamfer.yaml",
                                   FIXTURES / "config_chamfer_r0p02.yaml").wire(plan, tmp_path)
    assert result["reason"] in ("non_scored_wiring",)
    wiring = json.loads((tmp_path / "wiring.json").read_text())
    sender = next(item for item in wiring["stages"] if item["stage"] == "o7")["sender"]
    assert sender["clock_domain"] == "CLOCK_MONOTONIC"
    assert len(sender["intervals_s"]) == executor.WIRE_INPUTS - 1
    assert math.isfinite(sender["chains_wall_s"])
    timeline = sender["send_query_timeline"]
    assert {item["event"] for item in timeline} == {
        "publish_return", "query_start", "query_end"}
    assert timeline == sorted(timeline, key=lambda item: item["mono_ns"])
    # Sender independence: the second publish still starts at the planned
    # 0.5 s slot even though the first query chain ran in between. The
    # deliberately-blocked-query variant is the behavioral rehearsal test.
    assert abs(sender["actual_s"][1] - 0.5) < 0.1
    window = json.loads((tmp_path / "o7" / "window.json").read_text())
    for query in window["queries"]:
        assert query["send_wait_ms"] >= 0
    for row in window["inputs"]:
        assert row["publish_end_ns"] != row["input_stamp_ns"]   # boundary, not the cloud stamp
        assert row["terminal_count"] >= 0 and isinstance(row["terminal_count"], int)
        assert "receipt_problems" in row
    for query in window["queries"]:
        assert query["started_mono_ns"] <= query["ended_mono_ns"]
        assert query["latency_ms"] == (query["ended_mono_ns"] - query["started_mono_ns"]) / 1e6
        assert "chain_ms" in query and "send_wait_ms" in query
    chains = [json.loads((tmp_path / "o7" / ("chain-%d.json" % index)).read_text())
              for index in range(executor.WIRE_INPUTS)]
    for chain in chains:
        assert {"wait_ms", "surface_pages_ms", "cells_ms", "total_ms"} <= set(chain)
        assert all("call_ms" in page for page in chain["page_log"])
        assert all(set(call) >= {"service", "request_id", "started_mono_ns", "ended_mono_ns", "status"}
                   for call in chain["service_calls"])
    o8_stage = next(item for item in wiring["stages"] if item["stage"] == "o8")
    assert o8_stage["executed"] is True and o8_stage["health_valid"] is True
