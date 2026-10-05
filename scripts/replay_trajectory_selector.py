#!/usr/bin/env python3
"""Replay check for the unified trajectory selector. Does not start Gazebo.

A bundle is replayable only when it stores the selector inputs: cargo
surface, map-frame portal, slot targets, and payload size. Historical
MOTION-OCC dumps store rows and a surface, not payload size, so this
script records ``not_evaluated`` instead of inventing a box.
"""

from __future__ import division

import json
import os


ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))
CASES = (
    {
        "case_id": "2026-09-24_1520_trial0_transit",
        "surface": "docs/status/evidence/motion_occ/2026-09-24_1520_atlas_a1/dumps/trial_00_PLACE_PATH_INFEASIBLE/replay/surface_2d.json",
        "rows": "docs/status/evidence/motion_occ/2026-09-24_1520_atlas_a1/dumps/box_00_paths/candidates.json",
    },
    {
        "case_id": "2026-09-20_1405_standard_traverse",
        "surface": "docs/status/evidence/cargo_map_integrate/2026-09-20_1405/dumps/box_00_integrate/surface_2d.json",
        "rows": "",
    },
)
OUT = os.path.join(
    ROOT, "docs/status/evidence/motion_occ/trajectory_selector_replay")


def _load(path):
    if not path or not os.path.isfile(os.path.join(ROOT, path)):
        return None
    with open(os.path.join(ROOT, path), "r", encoding="utf-8") as handle:
        return json.load(handle)


def _assess(case):
    missing = []
    surface = _load(case["surface"])
    rows_doc = _load(case["rows"]) if case["rows"] else None
    if not isinstance(surface, dict) or "height" not in surface:
        missing.append("surface_2d")
    elif not isinstance(surface.get("geometry_descriptor"), dict):
        missing.append("geometry_descriptor")
    rows = (rows_doc or {}).get("rows") if isinstance(rows_doc, dict) else None
    if not rows:
        missing.append("selector_rows")
    else:
        sample = rows[0]
        if not sample.get("waypoints"):
            missing.append("map_frame_waypoints")
    missing.append("payload_wdh")
    missing.append("joint_states")
    return {
        "case_id": case["case_id"],
        "outcome": "not_evaluated",
        "replay_possible": False,
        "capture_complete": False,
        "missing": missing,
        "surface": case["surface"],
        "rows": case["rows"] or None,
    }


def main():
    os.makedirs(OUT, exist_ok=True)
    report = {
        "gate": "A1_offline_replay",
        "outcome": "not_evaluated",
        "reason": "bundles omit payload size and joint state, so the selector cannot be re-run",
        "cases": [_assess(case) for case in CASES],
    }
    path = os.path.join(OUT, "replay_manifest.json")
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2)
        handle.write("\n")
    print(path)
    print(report["outcome"])


if __name__ == "__main__":
    main()
