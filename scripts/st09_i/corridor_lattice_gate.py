#!/usr/bin/env python3
"""Replayability check for the corridor-lattice offline gate.

The scored search does not start unless every fixture can rebuild a
slot pose, a grasp orientation, the planning start joints, and a scene.
Run18 has no planning_request.json. This script reports that fact and
exits before MoveIt, IK, or a chain score.
"""

from __future__ import division

import json
import math
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
for _path in (
        ROOT,
        os.path.join(ROOT, "src", "luggage_planning"),
        os.path.join(ROOT, "src", "luggage_description"),
):
    if _path not in sys.path:
        sys.path.insert(0, _path)

EVIDENCE = os.path.join(
    ROOT, "docs/status/evidence/frame_migration",
    "ST09-parallel-I-s6-readiness/5db68f4/corridor_lattice")
RUN18 = os.path.join(
    ROOT, "docs/status/evidence/frame_migration/ST09-parallel-I-s5/m3",
    "g2_single_box_run18_20261004/dumps/trial_00_ok")
REQUEST = os.path.join(
    ROOT, "docs/status/evidence/frame_migration",
    "ST09-parallel-I-s6-readiness/5db68f4/single_box/dumps",
    "box_00_paths/planning_request.json")
SCENE = os.path.join(
    ROOT, "src/luggage_description/config/scene_tf_deploy.yaml")
THREE = os.path.join(
    ROOT, "docs/status/evidence/frame_migration/ST09-parallel-I-s5/m3",
    "g3_three_box_layout_20261005/frozen_layout.json")


def _load(path):
    with open(path, encoding="utf-8") as handle:
        return json.load(handle)


def _finite(values, count):
    if not isinstance(values, (list, tuple)) or len(values) != count:
        return False
    return all(isinstance(v, (int, float)) and math.isfinite(float(v))
               for v in values)


def _quat_ok(quat):
    if not _finite(quat, 4):
        return False
    norm = math.sqrt(sum(float(v) * float(v) for v in quat))
    return abs(norm - 1.0) < 1e-3


def _check_run18():
    present = {
        "slot_json": os.path.isfile(os.path.join(RUN18, "slot.json")),
        "scene_json": os.path.isfile(os.path.join(RUN18, "scene.json")),
        "trial_json": os.path.isfile(os.path.join(RUN18, "trial.json")),
        "tf_trace": os.path.isfile(os.path.join(RUN18, "tf_trace.jsonl")),
        "planning_request": os.path.isfile(
            os.path.join(RUN18, "planning_request.json")),
    }
    slot = _load(os.path.join(RUN18, "slot.json"))
    trial = _load(os.path.join(RUN18, "trial.json"))
    scene = _load(os.path.join(RUN18, "scene.json"))
    pose = (slot.get("pose_world") or {}).get("position")
    segments = []
    with open(os.path.join(RUN18, "segments.jsonl"), encoding="utf-8") as handle:
        segments = [json.loads(line) for line in handle if line.strip()]
    target = (segments[0].get("target_pose_world") or {}) if segments else {}
    missing = []
    if not _finite(pose, 3):
        missing.append("slot_pose")
    # Box centre plus yaw is not a TCP grasp. Segment targets in this
    # dump are the unset placeholder, and tf_trace stores suction xyz
    # without an orientation.
    if not _quat_ok(target.get("orientation")) or not _finite(
            target.get("position"), 3) or target.get("position") == [
                0.0, 0.0, 0.0]:
        missing.append("grasp")
    joints = (scene.get("robot_joints") or {}).get("position")
    # Those joints belong to the finished HOME state, not the planning
    # start. No other record stores the start.
    if trial.get("place_state") == "HOME" or not present["planning_request"]:
        missing.append("start_joints")
    elif not _finite(joints, 6):
        missing.append("start_joints")
    if not scene.get("world_object_ids"):
        missing.append("scene")
    return {
        "fixture": "g2_single_box_run18_20261004",
        "replayable": not missing,
        "outcome": "replayable" if not missing else "not_replayable",
        "missing": missing,
        "present_files": present,
        "slot_position": pose,
        "end_state_joints_not_used": joints,
        "detail": (
            "trial_00_ok has a box pose and a final HOME joint state. "
            "It has no grasp orientation and no planning start joints."),
    }


def _anchors_for(candidate, request, scene_config):
    from luggage_description.scene_tf_config_utils import (
        container_opening_normal_in_world,
        container_opening_target_point_in_world,
    )
    from luggage_planning.route_producer import anchor_plan

    target = candidate["targets"][0]
    plan = anchor_plan(
        tuple(float(v) for v in target[0]),
        request["container_transform"]["quaternion_xyzw"],
        float(request["box_wdh"][2]),
        opening=container_opening_target_point_in_world(scene_config),
        inward_normal=[
            -float(v) for v in
            container_opening_normal_in_world(scene_config)])
    return {
        "portal": [float(v) for v in plan.portal],
        "slot_approach": [float(v) for v in plan.slot_approach],
        "slot_upper": [float(v) for v in plan.slot_upper],
        "slot": [float(v) for v in plan.slot],
    }


def _check_slots():
    request = _load(REQUEST)
    from luggage_description.scene_tf_config_utils import load_scene_tf_config

    scene_config = load_scene_tf_config(SCENE)
    grasp = request.get("attachment_transform") or {}
    missing_common = []
    if not _quat_ok(grasp.get("quaternion_xyzw")) or not _finite(
            grasp.get("translation"), 3):
        missing_common.append("grasp")
    if not _finite(request.get("start_joints"), 6):
        missing_common.append("start_joints")
    if not os.path.isfile(SCENE):
        missing_common.append("scene")
    rows = []
    for index in (639, 656):
        candidate = next(
            (row for row in request["candidates"]
             if int(row["slot_index"]) == index), None)
        missing = list(missing_common)
        anchors = None
        if candidate is None:
            missing.append("slot_pose")
        else:
            pose = (candidate.get("slot_pose") or {}).get("position")
            if not _finite(pose, 3):
                missing.append("slot_pose")
            targets = candidate.get("targets") or []
            if not targets or not _finite(targets[0][0], 3) \
                    or not _quat_ok(targets[0][1]):
                missing.append("tcp_target")
            elif not missing:
                anchors = _anchors_for(candidate, request, scene_config)
        rows.append({
            "fixture": "5db68f4 slot %d" % index,
            "replayable": not missing,
            "outcome": "replayable" if not missing else "not_replayable",
            "missing": missing,
            "slot_index": index,
            "anchors": anchors,
        })
    return rows


def _check_three_box():
    layout = _load(THREE)
    scene_path = layout.get("scene")
    missing = []
    if not layout.get("slots") or len(layout["slots"]) < 3:
        missing.append("slot_pose")
    else:
        for slot in layout["slots"]:
            if not _finite(slot.get("center_world"), 3):
                missing.append("slot_pose")
                break
    # A nominal grasp can be synthesized by the layout scan. This
    # fixture does not record that transform or the planning start.
    if "attachment_transform" not in layout and "grasp" not in layout:
        missing.append("grasp")
    if not _finite(layout.get("start_joints"), 6):
        missing.append("start_joints")
    if not isinstance(scene_path, str) or not os.path.isfile(scene_path):
        missing.append("scene")
    return {
        "fixture": "g3_three_box_layout_20261005",
        "replayable": not missing,
        "outcome": "replayable" if not missing else "not_replayable",
        "missing": missing,
        "sequence": layout.get("sequence"),
        "n_slots": len(layout.get("slots") or []),
        "scene": scene_path,
        "detail": (
            "frozen_layout stores three box centres and a scene path. "
            "Grasp and planning start joints are not in the fixture."),
    }


def _write(name, document):
    os.makedirs(EVIDENCE, exist_ok=True)
    path = os.path.join(EVIDENCE, name)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(document, handle, indent=2, sort_keys=True)
        handle.write("\n")
    return path


def _replay_report():
    fixtures = [_check_run18()]
    fixtures.extend(_check_slots())
    fixtures.append(_check_three_box())
    blocked = [row["fixture"] for row in fixtures if not row["replayable"]]
    report = {
        "outcome": "not_evaluated" if blocked else "replayable",
        "blocked": blocked,
        "fixtures": fixtures,
        "detail": (
            "Run18 and the three-box layout stay not_evaluated. "
            "Slots 639 and 656 are the scored fixtures."),
    }
    _write("replayability.json", report)
    return report


def main():
    report = _replay_report()
    if "--score" not in sys.argv:
        print(json.dumps({
            "outcome": report["outcome"],
            "blocked": report["blocked"],
        }, indent=2))
        return 2 if report["blocked"] else 0
    slots = [
        row for row in report["fixtures"]
        if str(row["fixture"]).startswith("5db68f4")]
    if not all(row["replayable"] for row in slots):
        print(json.dumps({
            "outcome": "not_evaluated",
            "detail": "slot fixtures are not replayable",
        }, indent=2))
        return 2
    from corridor_lattice_score import score_slots

    return score_slots(EVIDENCE, REQUEST, SCENE)


if __name__ == "__main__":
    sys.exit(main())
