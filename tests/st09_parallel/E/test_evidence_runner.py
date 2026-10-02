from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "scripts"))

from frame_migration_st09_extract import extract_acceptance_variable
from frame_migration_st09_freeze import make_frame_dump, validate_frame_dump
from frame_migration_st09_verify import (
    _canonical_sha256,
    compute_sample_manifest,
    fault_replay_report,
    removal_sensitivity_matrix,
    verification_outcome,
)
from stage_perf_probe import EXPECTED_JOINT_ORDER, replay_trajectory, validate_trajectory_schema


def _acceptance():
    return json.loads((ROOT / "st09_contracts" / "st09_acceptance_manifest.json").read_text())


def _sample(sample_id, *, records=None, **overrides):
    records = records if records is not None else [{"event": "sample", "sample_id": sample_id}]
    document = {
        "schema": "st09.sample/1",
        "sample_id": sample_id,
        "datum": "world",
        "source": "measured",
        "identity": {"request_id": "req-1", "cycle_id": "cycle-1", "map_revision": 1},
        "records": records,
        "records_sha256": _canonical_sha256(records),
        "joins": [{"name": "sample", "complete": True}],
    }
    document.update(overrides)
    return document


def _write_json(path, document):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def test_sample_manifest_validates_content_before_observed(tmp_path):
    (tmp_path / "samples").mkdir()
    _write_json(tmp_path / "samples" / "geometry_frame_64.json", _sample("geometry_frame_64"))
    manifest = compute_sample_manifest(_acceptance(), "offline", tmp_path)
    observed = {item["sample_id"] for item in manifest["observed"]}
    missing = {item["sample_id"] for item in manifest["missing"]}
    assert "geometry_frame_64" in observed
    assert "map_cases_6_x2" in missing
    status, _ = verification_outcome([], ["missing sample: map_cases_6_x2"])
    assert status == "not_evaluated"


def test_sample_manifest_rejects_empty_schema_identity_datum_and_hash(tmp_path):
    cases = {
        "empty": {},
        "wrong_schema": _sample("geometry_frame_64", schema="legacy"),
        "wrong_identity": _sample("geometry_frame_64", identity={"request_id": "req-1"}),
        "wrong_datum": _sample("geometry_frame_64", datum="map"),
        "wrong_hash": _sample("geometry_frame_64", records_sha256="bad"),
    }
    for name, document in cases.items():
        root = tmp_path / name
        _write_json(root / "samples" / "geometry_frame_64.json", document)
        manifest = compute_sample_manifest(_acceptance(), "offline", root)
        assert "geometry_frame_64" in {item["sample_id"] for item in manifest["missing"]}
        rejected = manifest["missing"][0]["rejected_artifacts"]
        assert rejected, name


def test_removal_sensitivity_is_generated_for_every_required_sample():
    acceptance = _acceptance()
    matrix = removal_sensitivity_matrix(acceptance, "offline")
    required = {
        artifact
        for sample in compute_sample_manifest(acceptance, "offline", Path("/path/does/not/exist"))["required"]
        for artifact in sample["artifact_globs"]
    }
    assert {case["removed_artifact"] for case in matrix["cases"]} == required
    assert {case["expected_exit_code"] for case in matrix["cases"]} == {2}
    assert {case["expected_status"] for case in matrix["cases"]} == {"not_evaluated"}


def test_frame_dump_v2_keeps_measured_and_gt_separate():
    measured = make_frame_dump("measured", "box-1", {"pose": [1, 2, 3]})
    gt = make_frame_dump("gt", "box-1", {"pose": [1, 2, 3]})
    assert validate_frame_dump(measured, expected_source="measured", expected_identity="box-1")
    assert validate_frame_dump(gt, expected_source="gt", expected_identity="box-1")


def test_frame_dump_conflicts_use_fixed_reason_codes():
    conflicts = [
        ({"schema": "unknown", "source": "measured", "identity": "box-1"}, "UNKNOWN_SCHEMA"),
        ({"schema": "frame_dump/2", "source": "measured", "identity": "box-1", "datum": "map"}, "DATUM_CONFLICT"),
        ({"schema": "frame_dump/2", "source": "measured", "identity": "box-2"}, "IDENTITY_MISMATCH"),
        ({"schema": "frame_dump/2", "source": "mixed", "identity": "box-1"}, "MIXED_SOURCE_CONFLICT"),
        ({"schema": "frame_dump/2", "source": "measured", "identity": "box-1", "datum": "world", "overlay": "wrong"}, "OVERLAY_HASH_MISMATCH"),
    ]
    for document, reason in conflicts:
        try:
            validate_frame_dump(document, expected_source="measured", expected_identity="box-1", expected_datum="world")
        except ValueError as error:
            assert str(error) == reason
        else:
            raise AssertionError("%s must be rejected" % reason)


def test_fixed_trajectory_replay_is_deterministic_and_truncation_fails(tmp_path):
    trajectory = {
        "schema": "st09.trajectory/1",
        "mode": "fixed",
        "joint_names": list(EXPECTED_JOINT_ORDER),
        "identity": {"request_id": "req-1", "cycle_id": "cycle-1", "map_revision": 1},
        "points": [
            {"time_from_start_ns": 0, "positions": [0, 0, 0, 0, 0, 0], "velocities": [0, 0, 0, 0, 0, 0], "accelerations": [0, 0, 0, 0, 0, 0]},
            {"time_from_start_ns": 10, "positions": [0.1, 0, 0, 0, 0, 0], "velocities": [0, 0, 0, 0, 0, 0], "accelerations": [0, 0, 0, 0, 0, 0]},
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


def test_trajectory_schema_rejects_formal_corruptions():
    base = {
        "schema": "st09.trajectory/1",
        "joint_names": list(EXPECTED_JOINT_ORDER),
        "points": [
            {"time_from_start_ns": 0, "positions": [0] * 6, "velocities": [0] * 6, "accelerations": [0] * 6},
            {"time_from_start_ns": 1, "positions": [0] * 6, "velocities": [0] * 6, "accelerations": [0] * 6},
        ],
    }
    corruptions = [
        ({**base, "points": base["points"][:1]}, "TRUNCATED_TRAJECTORY"),
        ({**base, "points": [{k: v for k, v in base["points"][0].items() if k != "positions"}, base["points"][1]]}, "TRUNCATED_TRAJECTORY"),
        ({**base, "points": [{k: v for k, v in base["points"][0].items() if k != "velocities"}, base["points"][1]]}, "TRUNCATED_TRAJECTORY"),
        ({**base, "points": [{k: v for k, v in base["points"][0].items() if k != "accelerations"}, base["points"][1]]}, "TRUNCATED_TRAJECTORY"),
        ({**base, "joint_names": list(reversed(EXPECTED_JOINT_ORDER))}, "JOINT_ORDER_MISMATCH"),
        ({**base, "points": [base["points"][1], base["points"][0]]}, "NON_MONOTONIC_TIME"),
    ]
    for document, reason in corruptions:
        ok, observed_reason, _ = validate_trajectory_schema(document)
        assert not ok
        assert observed_reason == reason


def _joined_evidence(box_ids):
    evidence = {"events": []}
    for index, box in enumerate(box_ids, start=1):
        common = {
            "box_id": box,
            "request_id": "req-%d" % index,
            "cycle_id": "cycle-%d" % index,
            "causal_id": "causal-%d" % index,
            "map_revision": index,
        }
        evidence["events"].extend([
            {"type": "pick", **common},
            {"type": "place", **common},
            {"type": "release", **common},
            {"type": "integration", **common},
            {"type": "map_publish", **common},
        ])
    for revision in range(1, len(box_ids)):
        evidence["events"].append({
            "type": "revision_handoff",
            "from": revision,
            "to": revision + 1,
            "request_id": "req-%d" % revision,
        })
    return evidence


def test_extracts_two_and_three_boxes_with_strict_revision_handoffs():
    two = extract_acceptance_variable(_joined_evidence(("carryon-1", "standard-1")), {"box_count": 2})
    assert two["status"] == "pass"
    assert two["revision_handoff_count"] == 1

    evidence = _joined_evidence(("carryon-1", "standard-1", "carryon-2"))
    result = extract_acceptance_variable(evidence, {"box_count": 3})
    assert result["status"] == "pass"
    assert result["box_count"] == 3
    assert result["revision_handoff_count"] == 2


def test_extract_rejects_duplicate_crossbox_and_missing_join():
    duplicate = _joined_evidence(("carryon-1", "standard-1"))
    duplicate["events"].append(dict(duplicate["events"][0]))
    assert extract_acceptance_variable(duplicate, {"box_count": 2})["status"] == "fail"

    missing = _joined_evidence(("carryon-1", "standard-1", "carryon-2"))
    missing["events"] = [event for event in missing["events"] if event.get("type") != "revision_handoff"]
    assert extract_acceptance_variable(missing, {"box_count": 3})["status"] == "fail"

    wrong_order = _joined_evidence(("carryon-1", "standard-1"))
    wrong_order["events"][-1]["from"] = 9
    assert extract_acceptance_variable(wrong_order, {"box_count": 2})["status"] == "fail"


def test_fault_replay_reproduces_reason_and_effect_deltas():
    report = fault_replay_report([
        {
            "fault_id": "pre_pick",
            "expected": {"reason": "MAP_STALE", "effects": {"planning": 0, "execution": 0, "attachment": 0, "scene": 0, "map": 0}},
            "observed": {"reason": "MAP_STALE", "effects": {"planning": 0, "execution": 0, "attachment": 0, "scene": 0, "map": 0}},
        },
        {
            "fault_id": "pre_place",
            "expected": {"reason": "IN_FLIGHT_CONFLICT", "effects": {"planning": 0, "execution": 1, "attachment": 0, "scene": 0, "map": 0}},
            "observed": {"reason": "IN_FLIGHT_CONFLICT", "effects": {"planning": 0, "execution": 1, "attachment": 0, "scene": 0, "map": 0}},
        },
    ])
    assert report["status"] == "pass"
    assert all(not any(row["effect_deltas"].values()) for row in report["results"])
