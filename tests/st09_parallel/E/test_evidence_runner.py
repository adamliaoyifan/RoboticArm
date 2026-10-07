from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "scripts"))

from frame_migration_st09_extract import extract_acceptance_variable
from frame_migration_st09_extract import main as extract_main
from luggage_gazebo.integration_acceptance import (
    COMMIT_STAGES,
    EVENT_ORDER,
    FROZEN_HASHES,
    extract_acceptance,
)
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


def _m3_frozen():
    return {
        "box_count": 3,
        "sequence_ids": ["carryon", "standard", "carryon"],
        "hashes": {key: "a" * 64 for key in FROZEN_HASHES},
        "release_sample_interval_s": 0.05,
        "drift_sample_interval_s": 0.2,
        "event_order": list(EVENT_ORDER),
        "replay_command": ["bash", "scripts/st09_i/s5_m3_run.sh", "--n", "3"],
    }


def _m3_case(index, name):
    request = "m3-req-%d" % index
    input_revision, output_revision = index, index + 1
    events = []
    for seq, stage in enumerate(EVENT_ORDER):
        event = {
            "stage": stage,
            "request_id": request,
            "success": True,
            "acquisition_stamp_ns": 10000 * index + 10 * seq,
            "receiver_stamp_ns": 20000 * index + 10 * seq,
            "event_seq": seq,
            "scene_revision": "a" * 64,
            "geometry_hash": "a" * 64,
            "input_map_revision": input_revision,
        }
        if stage in COMMIT_STAGES:
            event["output_map_revision"] = output_revision
        events.append(event)
    return {
        "box_id": "%s@%02d" % (name, index),
        "request_id": request,
        "cycle_id": "m3-cycle-%d" % index,
        "causal_id": "m3-causal-%d" % index,
        "events": events,
        "measured": {
            "integration_success": True,
            "return_success": True,
            "payload_retained": True,
            "input_map_revision": input_revision,
            "output_map_revision": output_revision,
            "sensor_occupied_cells": 1,
            "floor_through_candidates": 0,
            "retreat_start": {"status": "ready", "max_delta": 0.008},
            # Boxes after the first must name an observed map. Box 1 stays
            # without one; the judge only requires it from the second case.
            "selection_map": (
                {"map_source": "release_integrated"} if index > 1 else {}),
        },
        "gt_reference": {
            "xy_error_m": 0.01, "z_error_m": 0.01, "drift_m": 0.005,
            "roll_deg": 0.5, "pitch_deg": -0.5, "center_inside_hull": True,
            "footprint_reference_cells": 10, "footprint_sensor_cells": 5,
        },
    }


def _m3_evidence():
    cases = [_m3_case(1, "carryon"), _m3_case(2, "standard"), _m3_case(3, "carryon")]
    handoffs = [
        {"box_id": "standard@02", "request_id": "m3-req-1", "from": 1, "to": 2,
         "from_revision": 2, "to_revision": 2},
        {"box_id": "carryon@03", "request_id": "m3-req-2", "from": 2, "to": 3,
         "from_revision": 3, "to_revision": 3},
    ]
    return {"complete": True, "hashes": _m3_frozen()["hashes"], "cases": cases,
            "handoffs": handoffs}


def test_three_box_unified_judge_requires_physical_gates():
    frozen = _m3_frozen()
    result = extract_acceptance(_m3_evidence(), frozen)
    assert result["status"] == "pass"
    assert result["box_count"] == 3
    assert result["revision_handoff_count"] == 2

    stripped = _m3_evidence()
    for case in stripped["cases"]:
        case.pop("gt_reference")
        case["measured"].pop("sensor_occupied_cells")
    result = extract_acceptance(stripped, frozen)
    assert result["status"] != "pass"
    assert any("gt_reference.xy_error_m" in item for item in result["missing"])
    assert any("sensor_occupied_cells" in item for item in result["missing"])

    disguise = _m3_evidence()
    for case in disguise["cases"]:
        for event in case["events"]:
            event["input_map_revision"] = 1
            if "output_map_revision" in event:
                event["output_map_revision"] = 1
        case["measured"]["input_map_revision"] = 1
        case["measured"]["output_map_revision"] = 1
    assert extract_acceptance(disguise, frozen)["status"] != "pass"


def _committed(evidence, confirmed, sensor_cells=0):
    """Mark every case as a ledger-committed box with the given outcome.

    Each box is committed between the previous box's output revision and its
    own integration input, so every revision after a commit moves up by one.
    """
    previous_after = None
    for index, case in enumerate(evidence["cases"]):
        measured = case["measured"]
        expected = (measured["input_map_revision"]
                    if previous_after is None else previous_after)
        before = expected + 1
        after = before + 1
        measured["input_map_revision"] = before
        measured["output_map_revision"] = after
        for event in case["events"]:
            if event.get("input_map_revision") is not None:
                event["input_map_revision"] = before
            if event.get("output_map_revision") is not None:
                event["output_map_revision"] = after
        measured["sensor_occupied_cells"] = sensor_cells
        measured["map_confirmation"] = {
            "mode": "commit_confirmed", "confirmed": confirmed,
            "cleared": [index + 1] if confirmed else [],
            "evidence_points": {str(index + 1): 9}, "min_evidence_points": 3,
            "commit_revisions": {"expected": expected, "resulting": before},
        }
        previous_after = after
    for position, row in enumerate(evidence["handoffs"]):
        revision = evidence["cases"][position]["measured"]["output_map_revision"]
        row["from_revision"] = revision
        row["to_revision"] = revision
    return evidence


def test_committed_box_is_judged_on_confirmation_not_sensor_cells():
    frozen = _m3_frozen()
    confirmed = extract_acceptance(_committed(_m3_evidence(), True), frozen)
    assert confirmed["status"] == "pass"
    unconfirmed = extract_acceptance(_committed(_m3_evidence(), False), frozen)
    assert unconfirmed["status"] == "fail"
    assert "case[0]:MAP_CONFIRMATION_REQUIRED" in unconfirmed["failures"]
    assert not any("NO_SENSOR_OCCUPANCY" in item
                   for item in unconfirmed["failures"])


def test_committed_chain_joins_through_the_commit_not_the_raw_output():
    frozen = _m3_frozen()
    evidence = _committed(_m3_evidence(), True)
    first, second = evidence["cases"][0]["measured"], evidence["cases"][1]["measured"]
    # The next box's integration input is NOT the previous output: the
    # commit sits between them.
    assert second["input_map_revision"] != first["output_map_revision"]
    assert extract_acceptance(evidence, frozen)["status"] == "pass"

    broken = _committed(_m3_evidence(), True)
    broken["cases"][1]["measured"]["map_confirmation"]["commit_revisions"][
        "expected"] += 1
    result = extract_acceptance(broken, frozen)
    assert "case[1]:STALE_MAP_REVISION" in result["failures"]

    skipped = _committed(_m3_evidence(), True)
    skipped["cases"][2]["measured"]["input_map_revision"] += 1
    assert "case[2]:STALE_MAP_REVISION" in extract_acceptance(
        skipped, frozen)["failures"]


def test_committed_box_needs_enough_evidence_points_not_sensor_coverage():
    frozen = _m3_frozen()
    evidence = _committed(_m3_evidence(), True)
    for case in evidence["cases"]:
        case["gt_reference"]["footprint_sensor_cells"] = 0
    assert extract_acceptance(evidence, frozen)["status"] == "pass"

    thin = _committed(_m3_evidence(), True)
    thin["cases"][0]["measured"]["map_confirmation"]["evidence_points"] = {"1": 2}
    assert "case[0]:MAP_EVIDENCE_POINTS" in extract_acceptance(
        thin, frozen)["failures"]

    uncommitted = _m3_evidence()
    uncommitted["cases"][0]["gt_reference"]["footprint_sensor_cells"] = 0
    assert "case[0]:GT_SENSOR_COVERAGE" in extract_acceptance(
        uncommitted, frozen)["failures"]


def test_uncommitted_box_still_needs_a_sensor_cell():
    frozen = _m3_frozen()
    evidence = _m3_evidence()
    for case in evidence["cases"]:
        case["measured"]["sensor_occupied_cells"] = 0
    result = extract_acceptance(evidence, frozen)
    assert result["status"] == "fail"
    assert "case[0]:NO_SENSOR_OCCUPANCY" in result["failures"]


def test_three_box_raw_stamps_reported_not_rewritten():
    frozen = _m3_frozen()
    evidence = _m3_evidence()
    events = evidence["cases"][0]["events"]
    events[2]["acquisition_stamp_ns"], events[3]["acquisition_stamp_ns"] = (
        events[3]["acquisition_stamp_ns"], events[2]["acquisition_stamp_ns"])
    result = extract_acceptance(evidence, frozen)
    assert result["status"] == "fail"
    assert "case[0]:EVENT_CAUSAL_ORDER" in result["failures"]
    assert events[2]["acquisition_stamp_ns"] == 10000 + 30
    assert events[3]["acquisition_stamp_ns"] == 10000 + 20


def test_main_routes_three_box_to_the_strict_judge(tmp_path, monkeypatch):
    frozen_path = tmp_path / "frozen.json"
    frozen_path.write_text(json.dumps(_m3_frozen()))
    # Shallow flat-event evidence the legacy chain check would pass: the
    # formal route must refuse it (no cases, no physical gates).
    shallow = {"events": _joined_evidence(("carryon@01", "standard@02", "carryon@03"))["events"]}
    shallow_path = tmp_path / "shallow.json"
    shallow_path.write_text(json.dumps(shallow))
    out = tmp_path / "verdict_shallow.json"
    monkeypatch.setattr(sys, "argv", [
        "extract", "--evidence", str(shallow_path),
        "--frozen", str(frozen_path), "--output", str(out)])
    assert extract_main() != 0
    verdict = json.loads(out.read_text())
    assert verdict["status"] != "pass"
    assert any("3 cases" in item for item in verdict["missing"])

    full_path = tmp_path / "full.json"
    full_path.write_text(json.dumps(_m3_evidence()))
    out_full = tmp_path / "verdict_full.json"
    monkeypatch.setattr(sys, "argv", [
        "extract", "--evidence", str(full_path),
        "--frozen", str(frozen_path), "--output", str(out_full)])
    assert extract_main() == 0
    assert json.loads(out_full.read_text())["status"] == "pass"


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


def test_replay_shell_entrypoint_refuses_live_domain(tmp_path):
    trajectory = {
        "schema": "st09.trajectory/1",
        "joint_names": list(EXPECTED_JOINT_ORDER),
        "points": [
            {"time_from_start_ns": 0, "positions": [0] * 6, "velocities": [0] * 6, "accelerations": [0] * 6},
            {"time_from_start_ns": 1, "positions": [0] * 6, "velocities": [0] * 6, "accelerations": [0] * 6},
        ],
    }
    path = tmp_path / "trajectory.json"
    path.write_text(json.dumps(trajectory), encoding="utf-8")
    script = ROOT / "scripts" / "frame_migration_st09_replay.sh"
    assert script.is_file()
    result = subprocess.run(
        ["bash", str(script), "--trajectory", str(path), "--domain", "7", "--ws", str(ROOT)],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    assert result.returncode == 2
    assert "refusing ROS_DOMAIN_ID=7" in result.stderr
