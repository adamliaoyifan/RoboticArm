#!/usr/bin/env python3
"""Check that a finished M3 run planned on the deployment layout and branch.

For every ``dumps/box_NN_paths`` directory this reads the recorded planning
request and candidates and checks, against the scene file the run used:

* ``layout``: request ``container_transform`` equals the scene file's
  ``container_link`` static transform (1 mm, 0.001 rad);
* ``atlas``: ``atlas_identity.enabled`` and the identity names the deploy-sim
  atlas and the scene file's whole-file hash;
* ``lattice``: the winner's ``selection_id`` starts with ``route-lattice`` or,
  when there is no winner, at least one row carries a ``LATTICE_`` reason;
* ``confirmation``: every ``last_settle`` / ``pending_changes`` record found in
  the run's ``trial.json`` files (listed, not judged).

Exit 1 when any layout or atlas check fails. A missing lattice winner is
reported, not failed, because a run may legitimately fail before a winner.

  verify_layout_branch.py --run RUN_DIR --scene scene_tf_deploy.yaml
"""
import argparse
import glob
import json
import math
import os
import sys

import yaml

TRANSLATION_TOL_M = 1e-3
YAW_TOL_RAD = 1e-3
ATLAS_NAME = "reachability_atlas_deploy_sim"


def scene_container_pose(scene_path):
    with open(scene_path, "r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    for entry in config["static_transforms"]:
        if entry["child"] == "container_link":
            return list(entry["translation"]), float(entry["rotation_rpy"][2])
    raise ValueError("no container_link transform in %s" % scene_path)


def scene_file_hash(scene_path):
    import hashlib
    with open(scene_path, "rb") as handle:
        return hashlib.md5(handle.read()).hexdigest()


def request_container_pose(request):
    transform = request["container_transform"]
    x, y, z, w = transform["quaternion_xyzw"]
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return list(transform["translation"]), yaw


def yaw_gap(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def check_layout(request, expected):
    live_t, live_yaw = request_container_pose(request)
    t_err = max(abs(a - b) for a, b in zip(expected[0], live_t))
    y_err = yaw_gap(expected[1], live_yaw)
    return {"ok": t_err <= TRANSLATION_TOL_M and y_err <= YAW_TOL_RAD,
            "translation": live_t, "yaw": round(live_yaw, 6),
            "translation_error_m": round(t_err, 6),
            "yaw_error_rad": round(y_err, 6)}


def check_atlas(request, scene_hash):
    identity = request.get("atlas_identity") or {}
    text = str(identity.get("identity") or "")
    return {"ok": bool(identity.get("enabled"))
            and ATLAS_NAME in text and text.endswith(scene_hash),
            "identity": text, "enabled": bool(identity.get("enabled")),
            "loaded": identity.get("loaded"), "reason": identity.get("reason")}


def check_lattice(candidates):
    winner = candidates.get("winner") or {}
    selection_id = str(winner.get("selection_id") or "")
    reasons = [str(row.get("reason") or "") for row in candidates.get("rows", [])]
    lattice_rows = sum(1 for reason in reasons if reason.startswith("LATTICE_"))
    return {"winner_selection_id": selection_id,
            "winner_slot": winner.get("slot_index"),
            "winner_is_lattice": selection_id.startswith("route-lattice"),
            "lattice_rows": lattice_rows,
            "reason": candidates.get("reason"),
            "ok": selection_id.startswith("route-lattice") or lattice_rows > 0}


def find_key(value, key, found):
    if isinstance(value, dict):
        if key in value:
            found.append(value[key])
        for item in value.values():
            find_key(item, key, found)
    elif isinstance(value, list):
        for item in value:
            find_key(item, key, found)


def confirmation_records(run_dir):
    records = []
    for path in sorted(glob.glob(os.path.join(run_dir, "dumps", "trial_*", "trial.json"))):
        with open(path, "r", encoding="utf-8") as handle:
            trial = json.load(handle)
        settles, pendings = [], []
        find_key(trial, "last_settle", settles)
        find_key(trial, "pending_changes", pendings)
        records.append({"trial": os.path.basename(os.path.dirname(path)),
                        "last_settle": settles, "pending_changes": pendings})
    return records


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True)
    parser.add_argument("--scene", required=True)
    args = parser.parse_args(argv)
    expected = scene_container_pose(args.scene)
    scene_hash = scene_file_hash(args.scene)
    boxes = []
    ok = True
    for directory in sorted(glob.glob(os.path.join(args.run, "dumps", "box_*_paths"))):
        request_path = os.path.join(directory, "planning_request.json")
        candidates_path = os.path.join(directory, "candidates.json")
        if not (os.path.isfile(request_path) and os.path.isfile(candidates_path)):
            boxes.append({"box": os.path.basename(directory), "missing": True})
            ok = False
            continue
        with open(request_path, "r", encoding="utf-8") as handle:
            request = json.load(handle)
        with open(candidates_path, "r", encoding="utf-8") as handle:
            candidates = json.load(handle)
        entry = {"box": os.path.basename(directory),
                 "map_source": request.get("map_source"),
                 "map_revision": request.get("map_revision"),
                 "layout": check_layout(request, expected),
                 "atlas": check_atlas(request, scene_hash),
                 "lattice": check_lattice(candidates)}
        ok = ok and entry["layout"]["ok"] and entry["atlas"]["ok"]
        boxes.append(entry)
    report = {"ok": ok and bool(boxes), "scene": args.scene,
              "expected_container": {"translation": expected[0], "yaw": expected[1]},
              "scene_file_md5": scene_hash, "boxes": boxes,
              "confirmation": confirmation_records(args.run)}
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
