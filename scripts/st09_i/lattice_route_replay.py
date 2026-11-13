#!/usr/bin/env python3
"""RouteProducer with corridor_planner=lattice on the 5db68f4 fixture.

Isolated MoveIt (domain 42, plan-only) is started and stopped here. Slots
656 and 639 are each planned twice through ``plan_place_route``: lattice
corridor, MoveIt joint transit/return, certification, map revalidation
and leg records. Atlas is off, as in the lattice gate.

Gate (15 s route budget):
* 656: success on both runs; wall < 15 s each; lattice edges stitch to
  the Generation 2 chain (6 decimals); certification passed;
  ``route_missing_parts`` empty (the producer returns success only then).
* 639: no route on both runs; its row names ``LATTICE_NO_CHAIN``.
The joint transit/return are OMPL (unseeded), so only the lattice part
is compared across runs.
"""

from __future__ import division

import json
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import corridor_lattice_g2 as g2  # noqa: E402
import corridor_lattice_score as score  # noqa: E402
import slot64_classify_replay as classify  # noqa: E402

ROOT = score.ROOT
FIXTURE = os.path.join(
    ROOT, "docs/status/evidence/frame_migration",
    "ST09-parallel-I-s6-readiness/5db68f4")
REQUEST = os.path.join(FIXTURE, "single_box/dumps/box_00_paths",
                       "planning_request.json")
TRIAL = os.path.join(FIXTURE, "single_box/dumps",
                     "trial_00_PLANNING_BUDGET_EXHAUSTED")
SURFACE = os.path.join(TRIAL, "replay/surface_2d.json")
SCENE_JSON = os.path.join(TRIAL, "scene.json")
G2_SUMMARY = os.path.join(FIXTURE, "corridor_lattice_g2/summary.json")
OUT = os.path.join(FIXTURE, "lattice_route_replay")
SCENE_TF = os.path.join(
    ROOT, "src/luggage_description/config/scene_tf_deploy.yaml")
BUDGET_S = 15.0
SLOTS = (656, 639)
REPEATS = 2
#: The gate read the dumped (rounded) TCP targets; the producer
#: re-synthesizes them from the attachment, about 7e-7 m apart. The chain
#: is compared exactly (recorded) and within this tolerance (gated).
CHAIN_TOL_RAD = 1e-4


def _dump(name, document):
    with open(os.path.join(OUT, name), "w", encoding="utf-8") as handle:
        json.dump(document, handle, indent=2, default=str)
        handle.write("\n")


def _lattice_chain(result):
    edges = [edge for edge in result.get("edges") or ()
             if (edge.attributes or {}).get("planner") == "corridor_lattice"]
    if not edges:
        return []
    chain = list(edges[0].trajectory.positions)
    for edge in edges[1:]:
        chain.extend(edge.trajectory.positions[1:])
    return [[round(float(v), 6) for v in q] for q in chain]


def _edge_rows(result):
    rows = []
    for edge in result.get("edges") or ():
        attributes = edge.attributes or {}
        rows.append({
            "edge_id": edge.edge_id,
            "edge_class": attributes.get("edge_class"),
            "planner": attributes.get("planner"),
            "load": str(edge.load_state),
            "states": len(edge.trajectory.positions),
            "duration_s": round(
                edge.trajectory.time_from_start_ns[-1] / 1e9, 3),
        })
    return rows


def _lattice_stats(rows):
    for row in rows:
        stats = row.get("lattice") or (row.get("detail") or {}).get("lattice")
        if stats:
            return stats
    return {}


def _slim_rows(rows):
    keep = ("slot_index", "candidate_id", "reason", "detail", "lattice",
            "certified", "cert_detail", "decision", "elapsed_ms",
            "route_id", "route_build_count")
    slim = []
    for row in rows or ():
        if isinstance(row, dict):
            slim.append({key: row[key] for key in keep if key in row})
    return slim


def main():
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node

    from luggage_description.scene_tf_config_utils import (
        load_scene_tf_config,
    )
    from luggage_perception.eval.isolated_domain import isolated_replay_env
    from luggage_planning.routing.lattice import LatticeStateChecker
    from luggage_planning.motion.planning_scene_client import PlanningSceneClient
    from luggage_planning.routing.producer import register_attach_from_request

    os.makedirs(OUT, exist_ok=True)
    manifest = {
        "fixture": FIXTURE, "request": REQUEST, "surface": SURFACE,
        "scene_json": SCENE_JSON, "scene_tf": SCENE_TF,
        "slots": list(SLOTS), "repeats": REPEATS, "budget_s": BUDGET_S,
        "domain": score.DOMAIN, "atlas": "off",
        "corridor_planner": "lattice",
        "started": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    }
    manifest.update(g2._commit())  # noqa: SLF001
    residuals = score._residuals()  # noqa: SLF001
    if residuals:
        manifest.update(outcome="not_evaluated", residuals=residuals)
        _dump("manifest.json", manifest)
        print(json.dumps(manifest, indent=2))
        return 2
    env = isolated_replay_env(score.DOMAIN)
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    proc = score._start_moveit(  # noqa: SLF001
        env, SCENE_TF, os.path.join(OUT, "moveit.log"))
    rclpy.init()
    node = Node("lattice_route_replay")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    reference = json.load(open(G2_SUMMARY, encoding="utf-8"))
    reference_chain = reference["slots"]["656"]["runs"][0]["chain"]
    summary = {"outcome": "incomplete", "slots": {}}
    try:
        from luggage_msgs.srv import ValidateRouteV2

        probe = node.create_client(ValidateRouteV2, "/validate_route_v2")
        if not probe.wait_for_service(timeout_sec=90.0):
            summary["detail"] = "validate_route_v2 did not come up"
            return 2
        scene_config = load_scene_tf_config(SCENE_TF)
        manifest["world"] = classify._sync_world(  # noqa: SLF001
            node, scene_config, json.load(open(SCENE_JSON, encoding="utf-8")))
        failures = []
        for slot_index in SLOTS:
            document, surface, _scene, slot = classify._request_document(  # noqa: SLF001
                REQUEST, SURFACE, SCENE_JSON, slot_index)
            stamp_ns = int(surface.get("acquisition_stamp_ns") or 1)
            producer, _urdf, _tcp = classify._producer(  # noqa: SLF001
                node, scene_config, SCENE_TF, None, stamp_ns,
                budget_s=BUDGET_S)
            scene_client = PlanningSceneClient(node)
            producer.corridor_planner = "lattice"
            producer.lattice_checker_factory = (
                lambda scene_states, start_q: LatticeStateChecker(
                    node, scene_client, scene_states, start_q))
            runs = []
            for repeat in range(REPEATS):
                request = classify._build_request(  # noqa: SLF001
                    document, surface, slot, "lattice-route-replay",
                    stamp_ns)
                register_attach_from_request(
                    producer.attachment_provider, request,
                    box_wdh=request["box_wdh"], box_id=request["box_id"],
                    generation=1)
                started = time.perf_counter()
                result = producer.plan_place_route(request)
                wall = time.perf_counter() - started
                reset = getattr(producer.edge_builder, "reset_scene", None)
                if callable(reset):
                    reset()
                chain = _lattice_chain(result)
                run = {
                    "repeat": repeat,
                    "success": bool(result.get("success")),
                    "reason_code": result.get("reason_code"),
                    "wall_s": round(wall, 3),
                    "within_budget": wall < BUDGET_S,
                    "edges": _edge_rows(result),
                    "legs": len(result.get("legs") or ()),
                    "certified": bool(
                        result.get("certification") is not None
                        and result["certification"].passed),
                    "lattice_chain_exact_g2": chain == reference_chain
                    if chain else None,
                    "lattice_chain_max_diff_rad": (
                        max(abs(a - b) for left, right in zip(
                            chain, reference_chain)
                            for a, b in zip(left, right))
                        if chain and len(chain) == len(reference_chain)
                        else None),
                    "lattice_chain_states": len(chain),
                    "lattice_chain": chain,
                    "stage_timings": result.get("stage_timings"),
                    "rows": _slim_rows(result.get("rows")),
                }
                if not run["success"]:
                    builder = producer.edge_builder
                    run["edge_failures"] = list(
                        getattr(builder, "edge_failure_log",
                                lambda: [])())[-8:]
                    run["request_dumps"] = list(
                        getattr(builder, "request_dumps",
                                lambda: [])())[-6:]
                runs.append(run)
                _dump("slot_%d_run_%d.json" % (slot_index, repeat), run)
                node.get_logger().info(
                    "lattice replay slot %d run %d: %s %.2fs" % (
                        slot_index, repeat, run["reason_code"], wall))
            checks = []
            if slot_index == 656:
                for run in runs:
                    if not run["success"]:
                        checks.append("run %d %s" % (
                            run["repeat"], run["reason_code"]))
                        continue
                    if not run["within_budget"]:
                        checks.append("run %d wall %.2f" % (
                            run["repeat"], run["wall_s"]))
                    diff = run["lattice_chain_max_diff_rad"]
                    if diff is None or diff > CHAIN_TOL_RAD:
                        checks.append("run %d chain differs (%s)" % (
                            run["repeat"], diff))
                    if not run["certified"]:
                        checks.append("run %d not certified" % run["repeat"])
            else:
                for run in runs:
                    if run["success"]:
                        checks.append("run %d found a route" % run["repeat"])
                    named = [row.get("reason") for row in run["rows"]]
                    if "LATTICE_NO_CHAIN" not in named:
                        checks.append("run %d rows %s" % (
                            run["repeat"], named))
            summary["slots"][str(slot_index)] = {
                "outcome": "pass" if not checks else "fail",
                "checks_failed": checks,
                "wall_s": [run["wall_s"] for run in runs],
                "chain_max_diff_rad": [
                    run.get("lattice_chain_max_diff_rad") for run in runs],
                "lattice_wall_s": [
                    _lattice_stats(run["rows"]).get("lattice_wall_s")
                    for run in runs],
                "reason_codes": [run["reason_code"] for run in runs],
            }
            failures.extend("%d: %s" % (slot_index, check)
                            for check in checks)
        summary["failures"] = failures
        summary["outcome"] = "pass" if not failures else "fail"
        return 0 if not failures else 1
    finally:
        _dump("summary.json", summary)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        score._stop_moveit(proc)  # noqa: SLF001
        manifest["ended"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        manifest["outcome"] = summary.get("outcome")
        manifest["moveit_stopped"] = proc.poll() is not None
        manifest["residuals_after"] = score._residuals()  # noqa: SLF001
        manifest["artifacts"] = sorted(os.listdir(OUT))
        _dump("manifest.json", manifest)
        print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    sys.exit(main())
