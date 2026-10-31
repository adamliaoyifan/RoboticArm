"""Offline checks for replay rejection and the O5–O8 entry.

The healthy replay runs the profile CLI. A file reread is not accepted as a replay.
"""
import json
import os
import sys
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent / "benchmark"))
import campaign_entry  # noqa: E402
import capture_replay  # noqa: E402

FIXTURES = Path("/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run01/fixtures")
REVIEW = Path("/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_review")
GEOMETRY = FIXTURES / "geometry_chamfer.yaml"
CONFIG = FIXTURES / "config_chamfer_r0p02.yaml"
CLI = os.environ.get(
    "OCC_S2_PROFILE_CLI",
    "/tmp/s2-install/uld_occ_observer/lib/uld_occ_observer/observer_profile_cli")


def test_review_counterexamples_are_not_replayable():
    bundle = capture_replay.replay_failure(REVIEW / "fabricated_failure.json", Path("/tmp/unused-replay"))
    assert bundle["replay_possible"] is False
    assert bundle["executed"] is False
    trace = capture_replay.certify_trace(REVIEW / "dropped_identity_free_trace.jsonl")
    assert trace["capture_complete"] is False
    assert trace["replay_possible"] is False
    assert "dropped" in trace["errors"] or any(item.startswith("missing_identity") for item in trace["errors"])


def test_epoch_rejection_is_observed_and_replayed(tmp_path):
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
    samples = first / "samples.yaml"
    command = capture_replay.profile_command(CLI, str(GEOMETRY), str(CONFIG), manifest, samples)
    import subprocess
    completed = subprocess.run(command, text=True, capture_output=True)
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
        "map_prefix": {"tree_nodes": row["tree_nodes"], "integrated_before": 0},
    }
    path = tmp_path / "bundle.json"
    path.write_text(json.dumps(bundle) + "\n")
    replayed = capture_replay.replay_failure(path, tmp_path / "replay")
    assert replayed["executed"] is True
    assert replayed["file_reread_is_replay"] is False
    assert replayed["replay_possible"] is True
    assert replayed["observed"]["status"] == "EPOCH_MISMATCH"
    assert replayed["observed"]["epoch"] == receipt["epoch"]
    assert receipt["status"] == "EPOCH_MISMATCH"


def test_campaign_stages_match_the_plan_and_o5_stops_later_gates():
    plan = campaign_entry.load_plan(FIXTURES)
    o5 = campaign_entry.resolve_stage(plan, "o5")
    assert o5["runner"] == "profile_cli_groups"
    assert o5["groups"] == 4
    assert o5["resolution_m"] == [0.02]
    assert o5["hulls"] == ["chamfer"]
    assert o5["boxes"] == [0, 20]
    assert o5["clouds"] == [30000, 100000]
    assert o5["outside"] == [0.5]
    assert o5["cross"] == [0.0]
    assert o5["roi_cells"] == 4096
    assert o5["scored"] == 30
    o6 = campaign_entry.resolve_stage(plan, "o6")
    assert o6["groups"] == 96
    assert o6["resolutions_m"] == [0.02, 0.05]
    assert o6["cold_starts_per_group"] == 30
    assert o6["blocks_if_o5_not_pass"] is True
    o7 = campaign_entry.resolve_stage(plan, "o7")
    assert o7["runner"] == "ros_windows"
    assert o7["normal_windows"] == 9
    assert o7["normal_inputs"] == 240
    assert o7["normal_duration_s"] == 120
    assert o7["normal_input_hz"] == 2
    assert o7["overload_inputs"] == 600
    assert o7["overload_input_hz"] == 10
    assert o7["recovery_inputs"] == 60
    assert o7["recovery_queries"] == 30
    assert o7["first_query"] == "wait_snapshot"
    assert o7["later_queries"] == "get_snapshot"
    o8 = campaign_entry.resolve_stage(plan, "o8")
    assert o8["runner"] == "ros_stability"
    assert o8["runs"] == ["o8_r0", "o8_r1", "o8_r2"]
    assert o8["duration_s"] == 180
    assert o8["no_input_s"] == 10
    assert o8["no_input_repeats"] == 3
    assert o8["saturation_frames"] == 30
    stopped = campaign_entry.continue_after({"o5": "fail"})
    assert stopped["continue_o6"] is False
    assert stopped["continue_o7"] is False
    assert stopped["stopped_at"] == "o5"
    assert campaign_entry.continue_after({"o5": "pass"})["continue_o6"] is True
    missing = campaign_entry.judge_recorded_window({"integrate_ms": [1.0] * 10, "resource_certification": "pass"})
    assert missing["pass"] is False
    assert "missing_observation" in missing["reasons"]
    resource = campaign_entry.judge_recorded_window({
        "integrate_ms": [1.0] * 30,
        "resource_certification": "NOT_EVALUATED",
    })
    assert resource["pass"] is False
    assert "resource_missing" in resource["reasons"]


def test_prepare_freezes_generation_manifests_without_scoring(tmp_path):
    index = campaign_entry.prepare_inputs(FIXTURES, tmp_path)
    assert index["scoring"] == "not_run"
    o6 = list((tmp_path / "o6").glob("*.json"))
    assert len(o6) == 96
    assert len(list((tmp_path / "o5").glob("*.json"))) == 4
    overload = json.loads((tmp_path / "o7" / "overload.json").read_text())
    assert overload["inputs"] == 600
    assert overload["input_hz"] == 10
    assert overload["point_bytes"] == "not_emitted"
    normal = list((tmp_path / "o7").glob("normal_*.json"))
    assert len(normal) == 9
    one = json.loads(normal[0].read_text())
    assert one["inputs"] == 240
    assert one["duration_s"] == 120
    o8 = json.loads((tmp_path / "o8" / "plan.json").read_text())
    assert o8["runs"] == ["o8_r0", "o8_r1", "o8_r2"]
    assert index["sha256"]
    one_cm = [path.name for path in o6 if path.name.startswith("r01_")]
    assert one_cm == []
