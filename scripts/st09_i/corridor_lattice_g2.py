#!/usr/bin/env python3
"""Corridor lattice Generation 2: equivalent performance re-acceptance.

Per slot (639, 656), on one isolated MoveIt (domain 42, plan-only):

1. reference: the unchanged Generation 1 algorithm (uncached IK, full
   collision mask, ``search_prepared``) with stage timers. Its mask and
   chain are the equivalence reference. Timers add overhead, so this
   run's time is not a score.
2. lazy x2: exact IK cache plus lazy collision checking.

Gate: 639 is LATTICE_NO_CHAIN on both lazy runs; 656 returns the same
chain on both runs, equal to the reference and to the Generation 1
summary; every lazily checked verdict equals the reference mask; layer
and vertex counts equal Generation 1; 656 total_s < 10.0 on each run.
RouteProducer is not touched. Run18 and the three-box layout stay
not_evaluated.
"""

from __future__ import division

import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import corridor_lattice_score as score  # noqa: E402

ROOT = score.ROOT
G1 = os.path.join(
    ROOT, "docs/status/evidence/frame_migration",
    "ST09-parallel-I-s6-readiness/5db68f4/corridor_lattice/summary.json")
OUT = os.path.join(
    ROOT, "docs/status/evidence/frame_migration",
    "ST09-parallel-I-s6-readiness/5db68f4/corridor_lattice_g2")
REQUEST = os.path.join(
    ROOT, "docs/status/evidence/frame_migration",
    "ST09-parallel-I-s6-readiness/5db68f4/single_box/dumps",
    "box_00_paths/planning_request.json")
SCENE = os.path.join(
    ROOT, "src/luggage_description/config/scene_tf_deploy.yaml")
BUDGET_S = 10.0
MARGIN_TARGET_S = 8.0
G1_COUNTS = {"639": {"n_vertices": 298, "n_layers": None},
             "656": {"n_vertices": 271, "n_layers": 81}}


def _key(joints, load):
    return [[round(float(v), 9) for v in joints], load]


def _chain(result):
    return [[round(float(v), 6) for v in joints] for joints in result.chain]


def _commit():
    import subprocess

    def run(*args):
        return subprocess.check_output(
            ["git", "-C", ROOT] + list(args), text=True).strip()

    return {
        "commit": run("rev-parse", "HEAD"),
        "dirty_count": len([
            line for line in run("status", "--porcelain").splitlines()
            if line.strip()]),
    }


def _reference(document, candidate, limits, refine, client, scenes):
    from luggage_planning.routing.corridor_lattice import (
        kinematic_queries,
        prepare_corridor,
        search_prepared,
    )

    timing = {}
    started = time.perf_counter()
    request, _center, _quat = score._request_for(  # noqa: SLF001
        document, candidate, limits)
    prepared = prepare_corridor(request, refine, stats=timing)
    mark = time.perf_counter()
    queries = kinematic_queries(prepared)
    timing["graph_build_s"] = time.perf_counter() - mark
    mark = time.perf_counter()
    mask, calls, error = score._mask(  # noqa: SLF001
        client, scenes, queries, timing)
    timing["collision_s"] = time.perf_counter() - mark
    timing["service_calls"] = calls
    if error:
        return None, None, timing, error
    mark = time.perf_counter()
    result = search_prepared(
        prepared, lambda joints, load: bool(mask.get((joints, load), False)),
        BUDGET_S, started)
    timing["search_s"] = time.perf_counter() - mark
    timing["total_s"] = result.wall_time_s
    timing["n_queries"] = len(queries)
    return result, mask, timing, ""


def _lazy(document, candidate, limits, refine, client, scenes):
    from luggage_planning.routing.corridor_lattice import (
        prepare_corridor,
        search_lazy,
    )

    timing = {"service_calls": 0}
    started = time.perf_counter()
    request, _center, _quat = score._request_for(  # noqa: SLF001
        document, candidate, limits)
    prepared = prepare_corridor(request, refine, cache_ik=True, stats=timing)
    errors = []

    def validate(states):
        mask, calls, error = score._mask(  # noqa: SLF001
            client, scenes, states, timing)
        timing["service_calls"] += calls
        if error:
            errors.append(error)
            return {}
        return mask

    result, verdicts = search_lazy(
        prepared, validate, BUDGET_S, started, timing)
    timing["total_s"] = result.wall_time_s
    if errors:
        return None, verdicts, timing, errors[0]
    return result, verdicts, timing, ""


def _row(result, timing):
    row = {
        "ok": bool(result.ok),
        "reason": result.reason,
        "total_s": round(timing["total_s"], 3),
        "within_budget": timing["total_s"] < BUDGET_S,
        "n_vertices": result.n_vertices,
        "n_layers": len(result.layer_names),
        "n_states_checked": result.n_collision_calls,
        "max_step_rad": round(result.max_step_rad, 6),
        "edge_loads": list(result.edge_loads),
        "chain": _chain(result),
        "timing": {
            key: (round(value, 4) if isinstance(value, float) else value)
            for key, value in timing.items()},
    }
    return row


def _verdict_mismatches(verdicts, reference_mask):
    mismatches = []
    for (joints, load), clear in verdicts.items():
        expected = reference_mask.get((joints, load))
        if expected is None or bool(expected) != bool(clear):
            mismatches.append({
                "state": _key(joints, load),
                "lazy": bool(clear),
                "reference": expected,
            })
    return mismatches


def main():
    import rclpy
    from luggage_description.scene_tf_config_utils import (
        load_scene_tf_config,
    )
    from luggage_perception.eval.isolated_domain import isolated_replay_env
    from luggage_planning.offline_ik import load_urdf, refine_to_pose

    os.makedirs(OUT, exist_ok=True)
    manifest = {
        "generation": 2,
        "scope": "equivalent performance only",
        "fixtures": ["5db68f4 slot 639", "5db68f4 slot 656"],
        "not_evaluated": ["g2_single_box_run18_20261004",
                          "g3_three_box_layout_20261005"],
        "domain": score.DOMAIN,
        "budget_s": BUDGET_S,
        "margin_target_s": MARGIN_TARGET_S,
        "request": REQUEST,
        "scene": SCENE,
        "generation_1": G1,
        "started": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    }
    manifest.update(_commit())
    residuals = score._residuals()  # noqa: SLF001
    if residuals:
        manifest.update(outcome="not_evaluated", residuals=residuals)
        _write("manifest.json", manifest)
        print(json.dumps({"outcome": "not_evaluated",
                          "residuals": residuals}, indent=2))
        return 2

    g1 = json.load(open(G1, encoding="utf-8"))
    env = isolated_replay_env(score.DOMAIN)
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    urdf = load_urdf(scene_tf_config=SCENE)
    tcp = urdf.path_to("suction_contact_frame")
    limits = urdf.joint_limits()

    def refine(position, quaternion, seed):
        return refine_to_pose(
            urdf, tcp, position, quaternion, seed,
            pos_tol=2e-3, rot_tol=2e-2)

    document = json.load(open(REQUEST, encoding="utf-8"))
    proc = score._start_moveit(  # noqa: SLF001
        env, SCENE, os.path.join(OUT, "moveit.log"))
    rclpy.init()
    node = rclpy.create_node("corridor_lattice_g2")
    stop, thread = score._spin(node)  # noqa: SLF001
    summary = {"outcome": "incomplete", "slots": {}}
    reference_doc = {"slots": {}}
    try:
        from luggage_msgs.srv import ValidateRouteV2

        client = node.create_client(ValidateRouteV2, "/validate_route_v2")
        if not client.wait_for_service(timeout_sec=90.0):
            summary["detail"] = "validate_route_v2 did not come up"
            return 2
        scene_config = load_scene_tf_config(SCENE)
        scene_client, container_id, triangles = score._sync_world(  # noqa: SLF001
            node, scene_config)
        summary.update(container_id=container_id, mesh_triangles=triangles)
        grasp = document["attachment_transform"]
        failures = []
        for index in (639, 656):
            name = str(index)
            candidate = next(
                row for row in document["candidates"]
                if int(row["slot_index"]) == index)
            _request, center, quat = score._request_for(  # noqa: SLF001
                document, candidate, limits)
            scenes, world_ids = score._scenes(  # noqa: SLF001
                scene_client, document["start_joints"], grasp,
                center, quat, document["box_wdh"])
            if container_id not in world_ids:
                summary["detail"] = "container missing from planning scene"
                return 2
            ref, mask, ref_timing, error = _reference(
                document, candidate, limits, refine, client, scenes)
            if error:
                summary["slots"][name] = {"outcome": "incomplete",
                                          "detail": error}
                failures.append("%s reference incomplete" % name)
                continue
            ref_row = _row(ref, ref_timing)
            reference_doc["slots"][name] = {
                "result": ref_row,
                "mask": [
                    _key(joints, load) + [bool(clear)]
                    for (joints, load), clear in mask.items()],
            }
            runs = []
            checks = []
            for repeat in range(2):
                lazy, verdicts, timing, error = _lazy(
                    document, candidate, limits, refine, client, scenes)
                if error:
                    checks.append("run %d incomplete: %s" % (repeat, error))
                    break
                row = _row(lazy, timing)
                row["verdict_mismatches"] = _verdict_mismatches(
                    verdicts, mask)
                runs.append(row)
            g1_runs = (g1["slots"].get(name) or {}).get("runs") or []
            g1_chain = g1_runs[0]["chain"] if g1_runs else None
            if len(runs) == 2:
                if runs[0]["chain"] != runs[1]["chain"]:
                    checks.append("lazy runs differ")
                if runs[0]["chain"] != ref_row["chain"]:
                    checks.append("lazy chain differs from reference")
                if ref_row["chain"] != (g1_chain or []):
                    checks.append("reference differs from generation 1")
                for repeat, row in enumerate(runs):
                    if row["verdict_mismatches"]:
                        checks.append("run %d verdict mismatch" % repeat)
                    if row["n_vertices"] != G1_COUNTS[name]["n_vertices"]:
                        checks.append("run %d vertex count" % repeat)
                    expected_layers = G1_COUNTS[name]["n_layers"]
                    if expected_layers is not None \
                            and row["n_layers"] != expected_layers:
                        checks.append("run %d layer count" % repeat)
                if index == 639:
                    for repeat, row in enumerate(runs):
                        if row["reason"] != "LATTICE_NO_CHAIN":
                            checks.append(
                                "run %d not LATTICE_NO_CHAIN" % repeat)
                else:
                    for repeat, row in enumerate(runs):
                        if not row["ok"]:
                            checks.append("run %d no chain" % repeat)
                        if not row["within_budget"]:
                            checks.append(
                                "run %d total_s %.3f >= %.1f" % (
                                    repeat, row["total_s"], BUDGET_S))
                        loads = set(row["edge_loads"])
                        if loads != {"carrying", "unloaded"}:
                            checks.append("run %d load switch" % repeat)
            summary["slots"][name] = {
                "outcome": "pass" if not checks else "fail",
                "checks_failed": checks,
                "reference": {
                    key: ref_row[key] for key in (
                        "ok", "reason", "total_s", "n_vertices",
                        "n_layers", "n_states_checked", "timing")},
                "runs": runs,
            }
            failures.extend("%s: %s" % (name, check) for check in checks)
        summary["failures"] = failures
        summary["outcome"] = "pass" if not failures else "fail"
        if any(row.get("outcome") == "incomplete"
               for row in summary["slots"].values()):
            summary["outcome"] = "incomplete"
        runs_656 = (summary["slots"].get("656") or {}).get("runs") or []
        summary["slot_656_total_s"] = [row["total_s"] for row in runs_656]
        summary["slot_656_margin_s"] = [
            round(BUDGET_S - row["total_s"], 3) for row in runs_656]
        summary["margin_target_met"] = bool(runs_656) and all(
            row["total_s"] <= MARGIN_TARGET_S for row in runs_656)
        return 0 if summary["outcome"] == "pass" else 1
    finally:
        _write("reference.json", reference_doc)
        _write("summary.json", summary)
        stop.set()
        thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        score._stop_moveit(proc)  # noqa: SLF001
        manifest["ended"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        manifest["outcome"] = summary.get("outcome")
        manifest["moveit_stopped"] = proc.poll() is not None
        manifest["residuals_after"] = score._residuals()  # noqa: SLF001
        manifest["artifacts"] = sorted(os.listdir(OUT))
        _write("manifest.json", manifest)
        print(json.dumps({
            "outcome": summary.get("outcome"),
            "failures": summary.get("failures"),
            "slot_656_total_s": summary.get("slot_656_total_s"),
            "margin_target_met": summary.get("margin_target_met"),
        }, indent=2))


def _write(name, document):
    path = os.path.join(OUT, name)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(document, handle, indent=2)
        handle.write("\n")
    return path


if __name__ == "__main__":
    sys.exit(main())
