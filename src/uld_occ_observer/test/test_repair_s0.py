"""Judge discrimination and frozen 2 cm manifest identity."""
import json
import sys
from pathlib import Path

BENCH = Path(__file__).resolve().parent / "benchmark"
sys.path.insert(0, str(BENCH))

import repair_s0  # noqa: E402
import repair_s0_contract as contract  # noqa: E402
import repair_s0_judge as judge  # noqa: E402
import repair_s0_manifest as manifest  # noqa: E402
import repair_s0_replay as replay  # noqa: E402

HISTORICAL_SEEDS = Path(
    "/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run03/o6/groups/"
    "r02_b00_chamfer_c30000_o50_x00/load/load_manifest.json")


def test_frame_seed_matches_emitted_schedule():
    historical = json.loads(HISTORICAL_SEEDS.read_text())
    schedule = manifest.frame_schedule(historical["group_id"])
    assert [(frame["index"], frame["seed"]) for frame in schedule] == [
        (frame["index"], frame["seed"]) for frame in historical["frames"]]


def test_manifest_keeps_96_groups_and_excludes_1cm():
    document = manifest.build_manifest(manifest.DEFAULT_FIXTURES)
    groups = document["o6_groups"]
    assert len(groups) == 96
    assert len({group["id"] for group in groups}) == 96
    assert not any(group["resolution"] == 0.01 or group["id"].startswith("r01_") for group in groups)
    assert {group["hull"] for group in groups} == {"chamfer", "cuboid"}
    assert {group["resolution"] for group in groups} == {0.02, 0.05}
    assert document["excluded_historical_failure"]["in_new_matrix"] is False
    assert len(document["o5_cases"]) == 4
    assert all(case["physical_roi_in_4096_stats"] is False for case in document["o5_cases"])
    assert sum(window["kind"] == "normal" for window in document["o7_windows"]) == 9
    assert document["o8"]["runs"] == ["o8_r0", "o8_r1", "o8_r2"]
    assert [pair["order"] for pair in document["o9"]["pairs"][:4]] == ["AB", "BA", "AB", "BA"]
    assert document["input_bytes"] == "not_emitted"
    assert document["environment"]["cpu_pinning"] == "none"
    assert document["environment"]["client_executor_threads"] == 4
    assert document["environment"]["integrate_threads"] == 1


def test_inventory_keeps_facts_mechanisms_and_hypotheses_apart():
    document = contract.inventory()
    assert {item["id"] for item in document["facts"]} >= {"o6-historical-fail", "store-lock-samples"}
    assert {item["id"] for item in document["mechanisms"]} >= {"get-takes-store-lock", "health-cache"}
    assert {item["id"] for item in document["hypotheses"]} >= {"residual-surface", "write-loop-split"}
    fact_text = " ".join(item["statement"] for item in document["facts"])
    assert "517.297808" in fact_text
    assert "hypothesis" not in fact_text.lower()


def test_judge_names_each_required_fault_and_replay_matches():
    document = manifest.build_manifest(manifest.DEFAULT_FIXTURES)
    cases = repair_s0.control_cases(
        [group["id"] for group in document["o6_groups"]],
        [case["id"] for case in document["o5_cases"]])
    seen = set()
    for name, case, status, reasons in cases:
        verdict = judge.judge_case(case)
        assert verdict["status"] == status, name
        assert verdict["reasons"] == reasons, name
        assert verdict["capture_certified"] is False
        assert replay.verify_replay(case)["replay_possible"] is True
        seen.add(name)
    assert {"missing_frame", "duplicate_terminal", "all_rejected", "pre_callback_gap",
            "recovery_timeout", "wrong_snapshot", "resource_missing", "health_gap",
            "single_window"} <= seen


def test_health_gap_does_not_hide_or_replace_a_callback_gap():
    gap = judge.healthy_o7_normal()
    gap["inputs"][0]["disposition"] = "not_entered"
    gap["inputs"][0]["callback_enter_ns"] = None
    gap["inputs"][0]["terminal_count"] = 0
    gap["inputs"][0]["receipt_received"] = False
    assert judge.judge_case(gap)["reasons"] == ["pre_callback_gap"]
    health = judge.healthy_o7_normal()
    health["health_times_s"] = [0.0, 2.0, 120.0]
    assert judge.judge_case(health)["reasons"] == ["health_gap"]


def test_legacy_stale_does_not_change_a_fresh_wait_snapshot():
    case = judge.healthy_o7_recovery()
    verdict = judge.judge_case(case)
    assert verdict["pass"] is True
    assert verdict["legacy_immediate_first"]["status"] == "STALE"
    assert verdict["legacy_changes_verdict"] is False


def test_incomplete_replay_cannot_pass():
    payload = json.loads(replay.dump_bytes(judge.healthy_o7_normal()).decode())
    del payload["publish_to_callback"][0]["publish_end_ns"]
    verdict = replay.replay_bytes(json.dumps(payload).encode())
    assert verdict["replay_possible"] is False
    assert verdict["pass"] is False
    assert verdict["reasons"] == ["replay_incomplete"]


def test_short_campaign_is_not_evaluated_and_one_failure_is_fail():
    one = judge.healthy_latency_group("only", 30000)
    short = judge.judge_case({"gate": "o6_campaign", "groups": [one]})
    assert short["status"] == "not_evaluated"
    assert short["pooled_latency"] == "not_computed"
    failed = judge.healthy_latency_group("slow", 30000)
    failed["integrate_ms"] = [600.0] * 30
    groups = [judge.healthy_latency_group("g%02d" % index, 30000) for index in range(95)]
    groups.append(failed)
    verdict = judge.judge_case({"gate": "o6_campaign", "groups": groups})
    assert verdict["status"] == "fail"
    assert verdict["failed_groups"] == ["slow"]
    assert verdict["pooled_latency"] == "not_computed"


def test_write_bundle_matches_the_rehearsal(tmp_path):
    summary = repair_s0.write_bundle(manifest.DEFAULT_FIXTURES, tmp_path)
    assert summary["groups"] == 96
    written = json.loads((tmp_path / "MANIFEST.json").read_text())
    assert written["input_bytes"] == "not_emitted"
    assert written["scoring"] == "not_run"
    discrimination = json.loads((tmp_path / "DISCRIMINATION.json").read_text())
    assert discrimination["capture_certified"] is False
    assert any(row["name"] == "pre_callback_gap" and row["reasons"] == ["pre_callback_gap"]
               for row in discrimination["cases"])
    assert "没有生成" in (tmp_path / "FINAL_REPORT.md").read_text()
    assert "pytest" in (tmp_path / "COMMANDS.sh").read_text()
