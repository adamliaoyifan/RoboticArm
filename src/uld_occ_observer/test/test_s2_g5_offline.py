"""S2 gen5: observations pass through the scheduler, and bad windows stay bad."""
import inspect
import json
import os
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent / "benchmark"))
import campaign_entry  # noqa: E402
import campaign_executor  # noqa: E402

FIXTURES = Path("/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run01/fixtures")


def _plan():
    return campaign_entry.load_plan(FIXTURES)


def _window(plan, name):
    return next(item for item in plan["o7_windows"] if item["id"] == name)


def test_assembler_does_not_manufacture_healthy_observations():
    for name in ("_assemble_window", "_assemble_group", "_run_stability"):
        source = inspect.getsource(getattr(campaign_executor, name))
        assert "healthy_" not in source
        assert "_resources_ok" not in source
    plan = _plan()
    window = _window(plan, "normal_b00_r0")
    rows = [{
        "input_id": "score-%d" % index,
        "phase": "score",
        "disposition": "rejected",
        "reject_reason": "QUEUE_FULL",
        "terminal_count": 1,
        "receipt_received": False,
        "callback_enter_ns": 1100 + index,
        "publish_end_ns": 1000 + index,
    } for index in range(240)]
    artifact = campaign_executor._assemble_window(window, rows, [], None, None)
    assert artifact["queries"] == []
    assert artifact["inputs"][0]["disposition"] == "rejected"
    assert "integrate_ms" not in artifact["inputs"][0]
    assert "health_times_s" not in artifact
    verdict = campaign_executor.repair_s0_judge.judge_o7_normal(artifact)
    assert verdict["pass"] is False
    assert "all_rejected" in verdict["reasons"]


def test_rejected_and_unknown_windows_fail_through_schedule(tmp_path):
    plan = _plan()
    rejected = campaign_executor.schedule(
        plan, campaign_executor.OfflineAdapter(input_disposition="rejected", emit_queries=False),
        tmp_path / "rejected", stages=("o7",), allow_incomplete_dependencies=True)
    artifact = json.loads((tmp_path / "rejected" / "o7" / "normal_b00_r0.json").read_text())
    assert artifact["queries"] == []
    assert artifact["inputs"][0]["disposition"] == "rejected"
    assert artifact["inputs"][0]["reject_reason"] == "QUEUE_FULL"
    assert rejected["stages"][0]["judge"]["pass"] is False

    unknown = campaign_executor.schedule(
        plan, campaign_executor.OfflineAdapter(input_disposition="unknown", emit_queries=False),
        tmp_path / "unknown", stages=("o7",), allow_incomplete_dependencies=True)
    saved = json.loads((tmp_path / "unknown" / "o7" / "normal_b00_r0.json").read_text())
    assert saved["inputs"][0]["disposition"] == "unknown"
    assert saved["inputs"][0]["receipt_received"] is False
    assert saved["inputs"][0]["integrate_ms"] == 100000.0
    assert saved["queries"] == []
    assert unknown["stages"][0]["judge"]["pass"] is False


def test_faults_keep_their_fields_on_the_schedule_path(tmp_path):
    plan = _plan()

    def run(name, adapter, stage):
        return campaign_executor.schedule(
            plan, adapter, tmp_path / name, stages=(stage,), allow_incomplete_dependencies=True)

    stale = run("stale", campaign_executor.OfflineAdapter(recovery_fault="stale"), "o7")
    assert stale["stages"][0]["recovery"]["pass"] is False
    missing_health = run("health", campaign_executor.OfflineAdapter(drop_health=True), "o7")
    assert missing_health["stages"][0]["judge"]["pass"] is False
    duplicate = run("dup", campaign_executor.OfflineAdapter(duplicate_input_id=True), "o7")
    assert duplicate["stages"][0]["judge"]["pass"] is False
    resources = run("resources", campaign_executor.OfflineAdapter(omit_resources=True), "o5")
    assert resources["stages"][0]["judge"]["pass"] is False
    growth = run("growth", campaign_executor.OfflineAdapter(rss_growth=True), "o8")
    assert "rss_growth" in growth["stages"][0]["judge"]["reasons"]
    nodes = run("nodes", campaign_executor.OfflineAdapter(node_stable=False), "o8")
    assert "node_count" in nodes["stages"][0]["judge"]["reasons"]


def test_timeline_sends_before_collect_and_uses_the_window_clock(tmp_path):
    plan = _plan()
    adapter = campaign_executor.OfflineAdapter()
    campaign_executor.schedule(
        plan, adapter, tmp_path, stages=("o7",), allow_incomplete_dependencies=True)
    sends = [item for item in adapter.order if item[0] == "send" and item[1] == "normal_b00_r0"]
    collects = [item for item in adapter.order if item[0] == "collect" and item[1] == "normal_b00_r0"]
    assert len(sends) == 240
    assert collects[0][3] >= sends[-1][3]
    assert sends[1][3] - sends[0][3] == pytest.approx(0.5)
    queries = [item for item in adapter.query_calls if item[0] == "normal_b00_r0"]
    assert len(queries) == 120
    assert queries[1][2] - queries[0][2] == pytest.approx(1.0)
    overload = [item for item in adapter.order if item[0] == "send" and item[1] == "overload"]
    assert len(overload) == 600
    assert overload[1][3] - overload[0][3] == pytest.approx(0.1)
    assert adapter.drain_time - overload[-1][3] == pytest.approx(30.0)
    recovery = json.loads((tmp_path / "o7" / "recovery.json").read_text())
    assert recovery["queries"][0]["op"] == "wait_snapshot"
    assert [item["op"] for item in recovery["queries"][1:]] == ["immediate"] * 29


def test_profile_frames_keep_distinct_surface_times_and_directories(tmp_path):
    plan = _plan()
    adapter = campaign_executor.OfflineAdapter()
    campaign_executor.schedule(plan, adapter, tmp_path, stages=("o5",))
    saved = json.loads(next(path for path in (tmp_path / "o5").glob("*.json") if path.name != "verdict.json").read_text())
    assert saved["surface_first_ms"][0] == 10.0
    assert saved["surface_after_update_ms"][0] == 12.0
    assert saved["surface_repeat_ms"][0] == 8.0
    dests = [item["dest"] for item in adapter.profile_calls]
    assert len(dests) == len(set(dests))
    assert sum(item["cold_start"] for item in adapter.profile_calls) == 4 * 30


def test_completed_receipt_is_integrated_without_counting_as_subscription():
    expected = {"epoch": 1, "sensor_id": "cam0", "observation_id": "obs-1"}
    response = {
        "status": "COMPLETED",
        "receipt": {
            "status": "INTEGRATED",
            "sensor_id": "cam0",
            "observation_id": "obs-1",
            "identity": {"epoch": 1},
        },
    }
    got = campaign_executor.interpret_observation_status(response, expected, subscribed_receipt=False)
    assert got["disposition"] == "integrated"
    assert got["receipt_status"] == "INTEGRATED"
    assert got["receipt_received"] is False
    subscribed = campaign_executor.interpret_observation_status(response, expected, subscribed_receipt=True)
    assert subscribed["receipt_received"] is True
    for top in ("REJECTED", "EXPIRED", "NOT_SEEN"):
        failed = campaign_executor.interpret_observation_status(
            {"status": top, "receipt": {"status": top, "sensor_id": "cam0", "observation_id": "obs-1"}},
            expected)
        assert failed["disposition"] == top
        assert failed["disposition"] != "integrated"


def test_injected_query_time_does_not_stretch_sends(tmp_path):
    plan = _plan()
    plan["o7_windows"] = [plan["o7_windows"][0]]

    class SlowQuery(campaign_executor.OfflineAdapter):
        def ros_query(self, window, index):
            result = super().ros_query(window, index)
            self.clock.advance(1.0)
            return result

    adapter = SlowQuery()
    campaign_executor.schedule(
        plan, adapter, tmp_path, stages=("o7",), input_limit=2,
        allow_incomplete_dependencies=True)
    sends = [entry for entry in adapter.order if entry[0] == "send"]
    assert [entry[3] for entry in sends] == pytest.approx([0.0, 0.5])
    assert [entry[4] for entry in sends] == pytest.approx([0.0, 0.5])
    assert adapter.clock.elapsed_s > 1.0


def test_authorized_live_route_does_not_start_official_windows(tmp_path, monkeypatch, capsys):
    monkeypatch.setenv("ULD_OCC_S3_OPEN", "1")
    monkeypatch.setenv("ULD_OCC_S2_CONFIRM_OFFICIAL", "1")
    monkeypatch.setenv("ULD_OCC_S2_ADAPTER", "live")
    code = campaign_executor.main(["--mode", "stage", "--stage", "o5", "--out", str(tmp_path)])
    payload = json.loads(capsys.readouterr().out)
    assert code == 2
    assert payload["adapter"] == "LiveCampaign"
    assert payload["reason"] == "official_long_window_not_started"
    assert payload["started"]["o5"] == 0
    assert payload["official_samples"] == "not_run"


def test_open_stage_dispatches_the_injected_runner(tmp_path, monkeypatch):
    monkeypatch.delenv("ULD_OCC_S3_OPEN", raising=False)
    closed = campaign_executor.main(["--mode", "stage", "--stage", "o5", "--out", str(tmp_path / "closed")])
    assert closed == 2
    monkeypatch.setenv("ULD_OCC_S3_OPEN", "1")
    monkeypatch.delenv("ULD_OCC_S2_CONFIRM_OFFICIAL", raising=False)
    monkeypatch.setenv("ULD_OCC_S2_ADAPTER", "live")
    refused = campaign_executor.main(["--mode", "stage", "--stage", "all", "--out", str(tmp_path / "live")])
    assert refused == 2
    monkeypatch.setenv("ULD_OCC_S2_ADAPTER", "injected")
    opened = campaign_executor.main(["--mode", "stage", "--stage", "o5", "--out", str(tmp_path / "open")])
    assert opened == 0
    verdict = json.loads((tmp_path / "open" / "o5" / "verdict.json").read_text())
    assert verdict["pass"] is True
    assert os.environ["ULD_OCC_S3_OPEN"] == "1"
