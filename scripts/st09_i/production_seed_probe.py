#!/usr/bin/env python3
"""Prove the production goal path attempts tool_down_j1_minus_pi.

Uses RouteProducer._goal_seed_fn, the same function the trajectory
selector installs, with the production refine tolerances (2 mm, 0.02 rad).
The matrix evaluator is used only to rebuild one dev case's goal pose.
"""

from __future__ import division

import argparse
import hashlib
import json
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "scripts", "st09_i"))

from route_matrix_eval import (  # noqa: E402
    OFFSETS,
    PAYLOADS,
    SCENE,
    Scene,
    assign_split,
    build_cases,
    load_retired_held_out,
    load_urdf,
)
from route_matrix_eval import Payload as MatrixPayload  # noqa: E402
from luggage_planning.offline_ik import refine_to_pose  # noqa: E402
from luggage_planning.routing.producer import (  # noqa: E402
    RouteProducer,
    tool_down_branch_seeds,
)
from luggage_planning.trajectory_certifier import (  # noqa: E402
    TrajectoryCertifier,
)

FREEZE = os.path.join(
    ROOT, "docs/status/evidence/frame_migration/ST09-parallel-I-s5/"
    "route_matrix_g2_20261005/freeze.json")
RETIRED = os.path.join(
    ROOT, "docs/status/evidence/frame_migration/ST09-parallel-I-s5/"
    "route_matrix_20261004/split.json")
POS_TOL = 2e-3
ROT_TOL = 2e-2


def _producer(urdf, tcp):
    def refine(position, quaternion, seed):
        return refine_to_pose(
            urdf, tcp, position, quaternion, list(seed),
            pos_tol=POS_TOL, rot_tol=ROT_TOL)

    return RouteProducer(
        graph=object(),
        edge_builder=object(),
        certifier=TrajectoryCertifier(),
        validity_adapter_factory=lambda **_kwargs: None,
        attachment_provider=object(),
        fk=lambda _q: ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        refine=refine,
        urdf=urdf,
        tcp_path=tcp,
        clock_ns=lambda: 0,
    )


def goal_pose(case, scene):
    size = PAYLOADS[case["payload"]]
    payload = MatrixPayload(size, OFFSETS[case["offset"]])
    if case["slot"] == "slot64_run02":
        from route_matrix_eval import _slot64_target
        return _slot64_target()
    from route_matrix_eval import HEADINGS
    heading = HEADINGS[case["heading"]]
    centre_w = scene.to_world(case["centre_container"])
    return payload.tcp_for_box(centre_w, scene.yaw + heading)


def _both_priors_miss(urdf, tcp, start_q, position, quaternion):
    nominal = tool_down_branch_seeds()[0][1]
    for seed in (nominal, tuple(start_q)):
        q, _info = refine_to_pose(
            urdf, tcp, position, quaternion, list(seed),
            pos_tol=POS_TOL, rot_tol=ROT_TOL)
        if q is not None:
            return False
    return True


def prove_case(case, scene, urdf, tcp, start_q):
    position, quaternion = goal_pose(case, scene)
    producer = _producer(urdf, tcp)
    goal_seeds = producer._goal_seed_fn(tuple(start_q), atlas_seeds=[])
    target = type("Target", (), {})()
    target.position = tuple(float(v) for v in position)
    target.quaternion_xyzw = tuple(float(v) for v in quaternion)
    goals = goal_seeds(target)
    record = producer.last_goal_refine[-1]
    joint1 = next(item for item in record["attempts"]
                  if item["seed"] == "tool_down_j1_minus_pi")
    families = [goal.ik_family for goal in goals]
    return {
        "case_id": case["id"],
        "attempted": joint1["seed"] == "tool_down_j1_minus_pi",
        "ok": bool(joint1["ok"]),
        "solved_by": joint1.get("solved_by"),
        "goal_families": families,
        "producer_file": os.path.abspath(
            sys.modules["luggage_planning.routing.producer"].__file__),
        "position": [float(v) for v in position],
        "quaternion_xyzw": [float(v) for v in quaternion],
        "start_joints": [float(v) for v in start_q],
    }


def find_dev_case(scene, urdf, tcp, starts, cases, dev_ids):
    by_id = {case["id"]: case for case in cases}
    for case_id in dev_ids:
        case = by_id.get(case_id)
        if case is None or case.get("kind") != "generated":
            continue
        start_q = starts[case["start"]]
        position, quaternion = goal_pose(case, scene)
        if _both_priors_miss(urdf, tcp, start_q, position, quaternion):
            return case, start_q
    return None, None


def load_context():
    from route_matrix_eval import _starts
    scene = Scene(SCENE)
    urdf = load_urdf(scene_tf_config=SCENE)
    tcp = urdf.path_to("suction_contact_frame")
    starts = _starts(urdf, tcp)
    retired = load_retired_held_out(RETIRED)
    cases, _occupancy = build_cases(scene, starts, retired)
    freeze = json.load(open(FREEZE, encoding="utf-8"))
    return scene, urdf, tcp, starts, cases, freeze


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--case-id", default="")
    parser.add_argument("--fixture-out", default="")
    args = parser.parse_args()
    scene, urdf, tcp, starts, cases, freeze = load_context()
    if args.case_id:
        case = next(item for item in cases if item["id"] == args.case_id)
        start_q = starts[case["start"]]
    else:
        case, start_q = find_dev_case(
            scene, urdf, tcp, starts, cases,
            freeze.get("dev_feasible_ids") or [])
        if case is None:
            print(json.dumps({"status": "no_dev_zero_goal_case"}))
            return 2
    result = prove_case(case, scene, urdf, tcp, start_q)
    result["split"] = assign_split(
        case["id"], set(load_retired_held_out(RETIRED)))
    result["pass"] = bool(
        result["attempted"] and result["ok"]
        and result["solved_by"] == "requested"
        and "refine-tool_down_j1_minus_pi" in result["goal_families"])
    if args.fixture_out:
        atlas_yaml = os.path.join(
            ROOT, "src/luggage_planning/data/reachability_atlas_deploy/"
            "s20_container_collision_aware_payload_%.2fx%.2fx%.2f.yaml" %
            PAYLOADS[case["payload"]])
        digest = ""
        if os.path.isfile(atlas_yaml):
            digest = hashlib.sha256(
                open(atlas_yaml, "rb").read()).hexdigest()
        fixture = {
            "request_id": "probe-%s" % case["id"],
            "matrix_case_id": case["id"],
            "box_wdh": list(PAYLOADS[case["payload"]]),
            "start_joints": result["start_joints"],
            "attachment_transform": {
                "translation": [
                    -OFFSETS[case["offset"]][0],
                    -OFFSETS[case["offset"]][1],
                    0.5 * PAYLOADS[case["payload"]][2]],
                "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            },
            "container_transform": {
                "translation": [float(v) for v in scene.t],
                "quaternion_xyzw": [float(v) for v in scene.q],
            },
            "map_revision": 0,
            "map_source": "static_empty",
            "geometry_hash": hashlib.sha256(
                json.dumps(case["centre_container"]).encode()).hexdigest(),
            "candidate_targets": [[
                result["position"], result["quaternion_xyzw"]]],
            "atlas_identity": {
                "identity": "deploy:%s" % digest[:16],
                "enabled": True,
                "atlas_yaml_sha256": digest,
            },
            "stage_timings": {},
            "provenance": {
                "solved_by": result["solved_by"],
                "producer_file": result["producer_file"],
            },
        }
        os.makedirs(os.path.dirname(args.fixture_out), exist_ok=True)
        with open(args.fixture_out, "w", encoding="utf-8") as handle:
            json.dump(fixture, handle, indent=1)
            handle.write("\n")
        result["fixture"] = args.fixture_out
    print(json.dumps(result, indent=1))
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
