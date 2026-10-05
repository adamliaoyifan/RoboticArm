"""Offline checks for the place-request replay record."""

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "src" / "luggage_gazebo"))

from luggage_gazebo.planning_request_capture import (  # noqa: E402
    audit_replay_bundle,
    build_planning_request,
    candidate_targets_from_rows,
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
            "translation": [1.0, 0.0, 0.0],
            "quaternion_xyzw": [0.0, 0.0, 1.0, 0.0],
        },
        "map_revision": 0,
        "map_source": "static_empty",
        "geometry_hash": "abc",
        "candidate_targets": [[[1.2, -0.2, 0.8], [0.0, 1.0, 0.0, 0.0]]],
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


def test_targets_come_from_selector_rows():
    rows = [{
        "ik_refine": [{
            "target_world": [[1.0, 2.0, 3.0], [0.0, 1.0, 0.0, 0.0]],
        }],
    }]
    targets = candidate_targets_from_rows(rows)
    assert targets == [[[1.0, 2.0, 3.0], [0.0, 1.0, 0.0, 0.0]]]
