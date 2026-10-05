#!/usr/bin/env python3
"""Replay one captured place request with atlas on and atlas off.

A fixture that lacks the request, start joints, attachment, map identity
or atlas identity exits ``not_replayable`` and names the missing fields.
It does not launch a simulation.
"""

from __future__ import division

import argparse
import json
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
GAZEBO_SRC = os.path.join(ROOT, "src", "luggage_gazebo")
if GAZEBO_SRC not in sys.path:
    sys.path.append(GAZEBO_SRC)

from luggage_gazebo.planning_request_capture import (  # noqa: E402
    audit_replay_bundle,
)


def _search(case_id):
    sys.path.insert(0, os.path.join(ROOT, "scripts", "st09_i"))
    from route_matrix_eval import (  # noqa: WPS433
        SCENE,
        Scene,
        build_cases,
        load_retired_held_out,
        load_urdf,
        run_case,
    )
    from route_matrix_eval import _starts  # noqa: WPS433

    scene = Scene(SCENE)
    urdf = load_urdf(scene_tf_config=SCENE)
    tcp = urdf.path_to("suction_contact_frame")
    starts = _starts(urdf, tcp)
    retired = load_retired_held_out(os.path.join(
        ROOT, "docs/status/evidence/frame_migration/ST09-parallel-I-s5/"
        "route_matrix_20261004/split.json"))
    cases, occupancy = build_cases(scene, starts, retired)
    case = next(item for item in cases if item["id"] == case_id)
    ctx = {
        "scene": scene, "urdf": urdf, "tcp": tcp,
        "fk": None, "starts": starts, "occupancy": occupancy,
        "atlases": None,
    }
    from route_matrix_eval import Atlases, make_offline_fk  # noqa: WPS433
    ctx["fk"] = make_offline_fk(urdf, tcp)
    ctx["atlases"] = Atlases()
    rows = []
    for atlas_on in (False, True):
        row = run_case(case, ctx, "state", atlas_on)
        rows.append({
            "atlas": atlas_on,
            "success": bool(row.get("success")),
            "reason": row.get("reason"),
            "search_outcome": row.get("search_outcome"),
            "goal_nodes": row.get("goal_nodes"),
            "goal_seed_sources": row.get("goal_seed_sources"),
            "wall_s": row.get("wall_s"),
        })
    return rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--fixture", required=True)
    parser.add_argument("--search", action="store_true",
                        help="also run the offline state search, atlas off then on")
    args = parser.parse_args()
    with open(args.fixture, encoding="utf-8") as handle:
        document = json.load(handle)
    audit = audit_replay_bundle(document)
    report = {"audit": audit, "fixture": args.fixture}
    if not audit["replay_possible"]:
        print(json.dumps(report, indent=1))
        return 2
    if args.search:
        case_id = document.get("matrix_case_id")
        if not case_id:
            report["search"] = {
                "status": "not_replayable",
                "missing": ["matrix_case_id"],
            }
            print(json.dumps(report, indent=1))
            return 2
        report["search"] = _search(case_id)
    print(json.dumps(report, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
