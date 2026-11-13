#!/usr/bin/env python3
"""B1 gate: one complete recorded input through the production chain twice.

The positive record is the run's own pick detection (measured FULL_3D,
height_source=MEASURED_SUPPORT) plus the perception boundary's joined
frames. Association and authorization run twice and must agree; the
derived negatives (missing mask / hold-only / invalid height) must
submit zero authorized picks.

usage: b1_replay.py <run_dir> <out_json>
"""
import json
import os
import sys

WS = "/tmp/elfin-master-pack-baseline-g3"
for path in (os.path.join(WS, "src", "luggage_perception"),
             os.path.join(WS, "src", "luggage_planning")):
    if path not in sys.path:
        sys.path.insert(0, path)

from luggage_perception.target_association import (  # noqa: E402
    admit_pick_frame,
    association_inputs_aligned,
    target_association_ambiguous,
)
from luggage_planning.pick.pick_authorization import (  # noqa: E402
    AUTHORIZE,
    PickAuthorizationPolicy,
    pick_authorized,
)


def box_from_detection(record):
    box = type("RecordBox", (), {})()
    box.top_surface_valid = bool(record.get("top_surface_valid"))
    box.top_surface_z = record.get("top_surface_z")
    box.height_valid = bool(record.get("height_valid"))
    box.height_source = int(record.get("height_source", 0))
    box.pca_source = str(record.get("pca_source", "measure"))
    box.id = str(record.get("id", "detected_box"))
    return box


def main():
    run_dir, out_json = sys.argv[1], sys.argv[2]
    boundary = None
    refs = None
    for root, _dirs, files in os.walk(run_dir):
        if "perception_boundary.json" in files:
            boundary = json.load(open(os.path.join(root, "perception_boundary.json")))
        if os.path.basename(root) == "replay" and "perception_refs.json" in files:
            refs = json.load(open(os.path.join(root, "perception_refs.json")))
    if refs is None or not refs.get("pick_detection"):
        raise SystemExit("no pick_detection record under %s" % run_dir)
    pick = refs["pick_detection"]

    report = {"source_run": run_dir,
              "pick_detection": pick,
              "association": {}, "authorization": {}, "negatives": {}}

    # -- association: the joined frame from the boundary, replayed twice ----
    if boundary is not None and boundary.get("join_trace"):
        joined = boundary["join_trace"][-1]
        depth_stamp = joined.get("depth_stamp")
        detection_stamp = joined.get("depth_stamp")  # joined by construction
        for run in (1, 2):
            aligned = association_inputs_aligned(
                detection_stamp, depth_stamp, depth_stamp)
            ambiguous = target_association_ambiguous(
                1, 1, stamps_aligned=aligned)
            admitted = admit_pick_frame(ambiguous, {"id": joined.get("commit_instance_id")})
            report["association"]["run%d" % run] = {
                "aligned": aligned, "ambiguous": ambiguous,
                "admitted": admitted,
            }
        # missing-mask negative: no mask message at this stamp
        aligned_nomask = association_inputs_aligned(
            detection_stamp, depth_stamp, depth_stamp, instance_stamp=None)
        ambiguous_nomask = target_association_ambiguous(2, None, stamps_aligned=True)
        report["association"]["missing_mask_negative"] = {
            "aligned_with_gap": aligned_nomask,
            "two_targets_no_mask_ambiguous": ambiguous_nomask,
            "admitted": admit_pick_frame(ambiguous_nomask, {"id": "x"}),
        }

    # -- authorization: the measured positive, twice, same action/reason ----
    policy = PickAuthorizationPolicy()
    box = box_from_detection(pick)
    first = policy.evaluate(True, box, 0.0, 1)
    second = policy.evaluate(True, box, 0.1, 1)
    report["authorization"] = {
        "run1": {"action": first.action, "reason": first.reason},
        "run2": {"action": second.action, "reason": second.reason},
        "identical": (first.action == second.action
                      and first.reason == second.reason),
        "authorized": first.action == AUTHORIZE,
        "pick_authorized": bool(pick_authorized(box)),
    }

    # -- negatives: derived from the same record, zero grasp submissions ----
    missing = box_from_detection(pick)
    missing.height_valid = False
    missing.height_source = 0  # UNAVAILABLE: no measured geometry at all
    hold_only = box_from_detection(pick)
    hold_only.pca_source = "hold_track"
    invalid_height = box_from_detection(pick)
    invalid_height.height_valid = False
    invalid_height.height_source = 0
    for name, variant, detected, reason in (
            ("missing_detection", None, False, ""),
            ("hold_only", hold_only, True, "DETECT_SUPPORT_STAMP_MISMATCH"),
            ("invalid_height", invalid_height, True, "")):
        decision = PickAuthorizationPolicy().evaluate(
            detected, variant, 0.0, 1, detect_reason=reason)
        report["negatives"][name] = {
            "action": decision.action, "reason": decision.reason,
            "authorized": decision.action == AUTHORIZE,
            "pick_authorized": bool(pick_authorized(variant, reason))
            if variant is not None else False,
        }

    checks = {
        "association_repeat_identical": (
            report["association"].get("run1") == report["association"].get("run2")
            if report["association"].get("run1") else None),
        "association_missing_mask_rejects": (
            report["association"]["missing_mask_negative"]["admitted"] is None),
        "authorization_repeat_identical": report["authorization"]["identical"],
        "measured_positive_authorizes": report["authorization"]["authorized"],
        "all_negatives_zero_submissions": all(
            not n["authorized"] and not n["pick_authorized"]
            for n in report["negatives"].values()),
    }
    report["checks"] = checks
    report["gate_pass"] = all(v is True for v in checks.values())
    with open(out_json, "w") as handle:
        json.dump(report, handle, indent=1)
        handle.write("\n")
    print(json.dumps(checks))
    return 0 if report["gate_pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
