from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "scripts"))

from frame_migration_st09_extract import extract_acceptance_variable
from frame_migration_st09_freeze import make_frame_dump, validate_frame_dump
from frame_migration_st09_verify import compute_sample_manifest, verification_outcome
from stage_perf_probe import replay_trajectory


def _acceptance():
    return json.loads((ROOT / "st09_contracts" / "st09_acceptance_manifest.json").read_text())


def test_sample_manifest_is_computed_from_manifest_and_artifacts(tmp_path):
    (tmp_path / "samples").mkdir()
    (tmp_path / "samples" / "geometry_frame_64.json").write_text("{}\n")
    manifest = compute_sample_manifest(_acceptance(), "offline", tmp_path)
    observed = {item["sample_id"] for item in manifest["observed"]}
    missing = {item["sample_id"] for item in manifest["missing"]}
    assert "geometry_frame_64" in observed
    assert "map_cases_6_x2" in missing
    status, _ = verification_outcome([], ["missing sample: map_cases_6_x2"])
    assert status == "not_evaluated"


def test_frame_dump_v2_keeps_measured_and_gt_separate():
    measured = make_frame_dump("measured", "box-1", {"pose": [1, 2, 3]})
    gt = make_frame_dump("gt", "box-1", {"pose": [1, 2, 3]})
    assert validate_frame_dump(measured, expected_source="measured", expected_identity="box-1")
    assert validate_frame_dump(gt, expected_source="gt", expected_identity="box-1")
    try:
        validate_frame_dump({"schema": "frame_dump/2", "source": "mixed", "identity": "box-1"})
    except ValueError as error:
        assert "mixed-source" in str(error)
    else:
        raise AssertionError("mixed frame_dump must be rejected")


def test_fixed_trajectory_replay_is_deterministic_and_truncation_fails(tmp_path):
    trajectory = {
        "schema": "st09.trajectory/1",
        "mode": "fixed",
        "points": [
            {"time_from_start_ns": 0, "positions": [0, 0, 0, 0, 0, 0]},
            {"time_from_start_ns": 10, "positions": [0.1, 0, 0, 0, 0, 0]},
        ],
    }
    path = tmp_path / "trajectory.json"
    path.write_text(json.dumps(trajectory))
    first = replay_trajectory(path, None, 42)
    second = replay_trajectory(path, None, 42)
    assert first["pass"] and first["sha256"] == second["sha256"]
    bad = tmp_path / "bad.json"
    bad.write_text(json.dumps({"schema": "st09.trajectory/1", "points": [{"time_from_start_ns": 0}]}))
    assert not replay_trajectory(bad, None, 42)["pass"]


def test_extracts_three_boxes_and_two_revision_handoffs():
    evidence = {"events": []}
    for box in ("carryon-1", "standard-1", "carryon-2"):
        evidence["events"].extend([
            {"type": "pick", "box_id": box},
            {"type": "attach", "box_id": box},
            {"type": "release", "box_id": box},
            {"type": "map_publish", "box_id": box},
        ])
    evidence["events"].extend([
        {"type": "revision_handoff", "from": 1, "to": 2},
        {"type": "revision_handoff", "from": 2, "to": 3},
    ])
    result = extract_acceptance_variable(evidence, {"box_count": 3})
    assert result["status"] == "pass"
    assert result["box_count"] == 3
    assert result["revision_handoff_count"] == 2
