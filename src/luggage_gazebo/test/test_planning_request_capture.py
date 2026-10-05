"""Offline checks for the place-request replay record."""

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "src" / "luggage_gazebo"))
sys.path.insert(0, str(REPO / "src" / "luggage_planning"))

from luggage_gazebo.planning_request_capture import (  # noqa: E402
    audit_replay_bundle,
    build_planning_request,
    candidate_targets_from_rows,
)
from luggage_planning.atlas_prior import (  # noqa: E402
    replay_captured_candidates,
)


def _complete():
    return build_planning_request({
        "request_id": "req-1",
        "box_wdh": [0.55, 0.40, 0.25],
        "start_joints": [0.1, -0.2, 0.3, -0.4, 0.5, -0.6],
        "attachment_transform": {
            "translation": [0.0, 0.0, 0.12],
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
        },
        "container_transform": {
            "translation": [0.0, 0.0, 0.0],
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
        },
        "map_revision": 0,
        "map_source": "static_empty",
        "geometry_hash": "abc",
        "candidate_targets": [[[1.2, -0.2, 0.8], [0.0, 1.0, 0.0, 0.0]]],
        "candidates": [{
            "candidate_id": "cand-0",
            "slot_index": 0,
            "slot_pose": {
                "position": [0.2, -0.1, 0.14],
                "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            },
            "targets": [[[1.2, -0.2, 0.8], [0.0, 1.0, 0.0, 0.0]]],
            "reachability": "unreachable",
            "seed_sources": [],
            "reason": "ATLAS_UNREACHABLE",
        }, {
            "candidate_id": "cand-1",
            "slot_index": 1,
            "slot_pose": {
                "position": [0.4, -0.1, 0.14],
                "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            },
            "targets": [[[0.4, 0.2, 0.9], [0.0, 0.0, 0.0, 1.0]]],
            "reachability": "reachable",
            "seed_sources": ["atlas"],
            "reason": "",
        }],
        "map_snapshot": {
            "rows": 4, "columns": 4, "resolution_m": 0.05,
            "map_revision": 0, "geometry_hash": "abc",
        },
        "atlas_identity": {"identity": "deploy:deadbeef"},
        "stage_timings": {"search_ms": 12.0},
    })


def test_complete_fixture_is_replayable():
    audit = audit_replay_bundle(_complete())
    assert audit["status"] == "replayable"
    assert audit["missing"] == []


def test_missing_fields_are_named_and_not_called_replayable():
    document = _complete()
    del document["start_joints"]
    document["map_source"] = ""
    document["atlas_identity"] = {"enabled": True}
    audit = audit_replay_bundle(document)
    assert audit["status"] == "not_replayable"
    assert audit["replay_possible"] is False
    assert "start_joints" in audit["missing"]
    assert "map_source" in audit["missing"]
    assert "atlas_identity" in audit["missing"]


def test_complete_fixture_replays_atlas_on_and_off():
    document = _complete()

    def query(_x, y, _z, _yaw):
        # cand-0's target is the only one with a negative y.
        return 1 if y < 0.0 else 3

    rows = replay_captured_candidates(document, query)
    by_id = {row["candidate_id"]: row for row in rows}
    assert by_id["cand-0"]["atlas_on"] == "unreachable"
    assert by_id["cand-0"]["atlas_off"] == "no_atlas"
    assert by_id["cand-1"]["atlas_on"] == "reachable"
    assert by_id["cand-1"]["atlas_off"] == "no_atlas"
    assert by_id["cand-1"]["seed_sources"] == ["atlas"]


def test_character_split_json_targets_replay():
    from luggage_planning.planning_capture import normalize_targets
    encoded = "[[[0.1, 0.2, 0.8], [0.0, 0.0, 0.0, 1.0]]]"
    pairs = normalize_targets(list(encoded))
    assert pairs == [[[0.1, 0.2, 0.8], [0.0, 0.0, 0.0, 1.0]]]
    document = _complete()
    document["candidates"][0]["targets"] = list(encoded)
    rows = replay_captured_candidates(document, lambda _x, y, _z, _yaw: 1 if y < 0 else 3)
    by_id = {row["candidate_id"]: row for row in rows}
    assert by_id["cand-0"]["n_targets"] == 1
    assert by_id["cand-0"]["atlas_on"] == "reachable"


def test_stringified_targets_are_parsed_before_replay():
    from luggage_planning.planning_capture import candidates_from_rows
    rows = [{
        "candidate_id": "cand-0",
        "slot_index": 4,
        "slot_pose": {
            "position": [0.1, 0.2, 0.3],
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
        },
        "targets": "[[[0.1, 0.2, 0.8], [0.0, 0.0, 0.0, 1.0]]]",
        "reachability": "reachable",
        "seed_sources": ["atlas"],
        "reason": "PLANNING_BUDGET_EXHAUSTED",
    }]
    views = candidates_from_rows(rows)
    assert views[0]["targets"] == [[[0.1, 0.2, 0.8], [0.0, 0.0, 0.0, 1.0]]]


def test_targets_come_from_selector_rows():
    rows = [{
        "ik_refine": [{
            "target_world": [[1.0, 2.0, 3.0], [0.0, 1.0, 0.0, 0.0]],
        }],
    }]
    targets = candidate_targets_from_rows(rows)
    assert targets == [[[1.0, 2.0, 3.0], [0.0, 1.0, 0.0, 0.0]]]
