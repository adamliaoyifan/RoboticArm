#!/usr/bin/env python3
"""Certify the three opening-side probes on the revision-2 scene.

Plan-only MoveIt on domain 42. Gazebo is not started. The live scene
gets the container, pedestal, ``placed_0_0_0`` from the failure
``scene.json``, and one box per occupied surface cell (the same
inflation ``scene_boxes_from_snapshot`` uses). Each probe is planned
twice through ``plan_place_route`` with ``corridor_planner=lattice``.
"""

from __future__ import division

import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
for path in (
        HERE, ROOT,
        os.path.join(ROOT, "src", "luggage_planning"),
        os.path.join(ROOT, "src", "luggage_description"),
        os.path.join(ROOT, "src", "luggage_perception"),
):
    if path not in sys.path:
        sys.path.insert(0, path)

import corridor_lattice_score as score  # noqa: E402
import slot64_classify_replay as classify  # noqa: E402

BASE = os.path.join(
    ROOT, "docs/status/evidence/frame_migration/ST09-parallel-I-s5/m3")
OUT = os.path.join(
    ROOT, "docs/status/evidence/frame_migration",
    "ST09-parallel-I-s6-readiness/placement_corridor_supergraph_g1",
    "opening_cert")
SCENE_TF = os.path.join(
    ROOT, "src/luggage_description/config/scene_tf_deploy.yaml")
BUDGET_S = 90.0
REPEATS = 2
WORLD_T = (-1.5, 0.6, 0.0)


def _dump(name, document):
    with open(os.path.join(OUT, name), "w", encoding="utf-8") as handle:
        json.dump(document, handle, indent=2, default=str)
        handle.write("\n")


def _outer_hit(request_path, candidates_path):
    request = json.load(open(request_path, encoding="utf-8"))
    document = json.load(open(candidates_path, encoding="utf-8"))
    hits = []
    for row in document["rows"]:
        if row.get("reason") != "LATTICE_NO_CHAIN":
            continue
        block = ((row.get("detail") or {}).get("lattice") or {}).get(
            "first_block") or {}
        bodies = []
        for contact in block.get("contacts") or []:
            bodies.extend((contact.get("body_1"), contact.get("body_2")))
        if "placed_0_0_0" not in bodies:
            continue
        hits.append(row)
    if not hits:
        raise SystemExit("no payload-hit row in %s" % candidates_path)
    seed = min(hits, key=lambda row: row["slot_pose"]["position"][0])
    return request, seed


def _probes():
    specs = (
        ("standard_m10", "g3_online_n3_budget180", -0.10),
        ("carryon_m10", "g3_online_n3_carryon3", -0.10),
        ("carryon_m15", "g3_online_n3_carryon3", -0.15),
    )
    built = []
    for name, run, shift in specs:
        root = os.path.join(BASE, run, "dumps")
        request, seed = _outer_hit(
            os.path.join(root, "box_01_paths/planning_request.json"),
            os.path.join(root, "box_01_paths/candidates.json"))
        center = [float(v) for v in seed["slot_pose"]["position"]]
        center[0] += shift
        built.append({
            "name": name,
            "run": run,
            "shift_m": shift,
            "source_slot": int(seed["slot_index"]),
            "center": center,
            "quaternion_xyzw": [
                float(v) for v in seed["slot_pose"]["quaternion_xyzw"]],
            "box_wdh": [float(v) for v in request["box_wdh"]],
            "request_path": os.path.join(
                root, "box_01_paths/planning_request.json"),
            "surface_path": os.path.join(
                root, "trial_01_PLANNING_BUDGET_EXHAUSTED/replay/surface_2d.json"),
            "scene_path": os.path.join(
                root, "trial_01_PLANNING_BUDGET_EXHAUSTED/scene.json"),
        })
    return built


def _equivalent(request_path, center, quaternion):
    """True when the captured selector list already has this XY and yaw."""
    import math
    document = json.load(open(request_path, encoding="utf-8"))
    wanted = _yaw(quaternion)
    for row in document.get("candidates") or []:
        pose = row.get("slot_pose") or {}
        position = pose.get("position") or []
        if len(position) < 2:
            continue
        if abs(float(position[0]) - center[0]) > 0.02:
            continue
        if abs(float(position[1]) - center[1]) > 0.02:
            continue
        other = _yaw(pose.get("quaternion_xyzw") or (0, 0, 0, 1))
        delta = abs((wanted - other + math.pi) % (2 * math.pi) - math.pi)
        if min(delta, abs(delta - math.pi)) < 0.2:
            return int(row.get("slot_index"))
    return None


def _yaw(quaternion):
    import math
    x, y, z, w = (float(v) for v in quaternion)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _clear_occupancy(node):
    from moveit_msgs.msg import PlanningSceneComponents
    from luggage_planning.planning_scene_client import PlanningSceneClient

    client = PlanningSceneClient(node)
    scene = client.get_scene(
        PlanningSceneComponents.WORLD_OBJECT_NAMES
        | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY,
        timeout=5.0)
    if scene is None:
        return
    for obj in scene.world.collision_objects:
        if str(obj.id).startswith("snapshot-cell-"):
            client.remove_object(str(obj.id))


def _add_occupancy(node, surface):
    """Occupied cells as world boxes, same size as scene_boxes_from_snapshot."""
    from luggage_planning.planning_scene_client import PlanningSceneClient

    client = PlanningSceneClient(node)
    origin = surface["origin_local"]
    resolution = float(surface["resolution"])
    size = resolution + 0.10
    added = 0
    for ix, row in enumerate(surface["state"]):
        for iy, state in enumerate(row):
            if state != "occupied":
                continue
            local_x = origin[0] + (ix + 0.5) * resolution
            local_y = origin[1] + (iy + 0.5) * resolution
            world = [
                -local_x + WORLD_T[0],
                -local_y + WORLD_T[1],
                0.15,
            ]
            ok = client.add_collision_box(
                "snapshot-cell-%d-%d" % (ix, iy), world,
                (0.0, 0.0, 0.0, 1.0), (size, size, 0.30), frame_id="world")
            if not classify._applied(ok):  # noqa: SLF001
                raise RuntimeError("occupancy cell %d,%d failed" % (ix, iy))
            added += 1
    return added


def _wrap_refine(producer, sink, original):
    def wrapped(position, quaternion, seed):
        solved, info = original(position, quaternion, seed)
        sink.append({
            "tcp": [round(float(v), 4) for v in position],
            "ok": solved is not None,
            "joints": (
                [round(float(v), 6) for v in solved] if solved is not None
                else None),
            "pos_err_m": None if not isinstance(info, dict)
            else info.get("pos_err_m"),
        })
        return solved, info

    producer.refine = wrapped


def _branch_ids(records):
    from luggage_planning.atlas.wavefront import joint_branch_distance

    grouped = {}
    for record in records:
        if not record["ok"]:
            record["branch_id"] = None
            continue
        key = tuple(record["tcp"])
        grouped.setdefault(key, [])
        assigned = None
        for index, prior in enumerate(grouped[key]):
            if joint_branch_distance(record["joints"], prior) < 0.35:
                assigned = index
                break
        if assigned is None:
            assigned = len(grouped[key])
            grouped[key].append(record["joints"])
        record["branch_id"] = assigned
    return records


def _edge_view(edge):
    attributes = edge.attributes or {}
    positions = list(getattr(edge.trajectory, "positions", ()) or ())
    return {
        "edge_id": str(edge.edge_id),
        "edge_class": attributes.get("edge_class"),
        "load": str(edge.load_state),
        "fraction": attributes.get("fraction", "not_captured"),
        "states": len(positions),
        "joints": [
            [round(float(v), 5) for v in joints] for joints in positions],
    }


def _semantic(edges):
    classes = [edge.get("edge_class") for edge in edges]
    retreats = [name for name in classes if name == "RETREAT"]
    return {
        "edge_classes": classes,
        "has_descend": "DESCEND" in classes,
        "retreat_count": len(retreats),
        "has_return": "RETURN" in classes,
        "has_entry": any(
            name in classes for name in ("TRAVERSE", "INSERT", "PORTAL_TRANSIT")),
        "complete": (
            "DESCEND" in classes and len(retreats) >= 3 and "RETURN" in classes
            and any(name in classes for name in ("TRAVERSE", "INSERT"))),
    }


def _first_failure(rows):
    for row in rows or []:
        if row.get("reason") in (None, "ATTEMPT_SUMMARY", "OK"):
            continue
        lattice = row.get("lattice") or (row.get("detail") or {}).get("lattice") or {}
        block = lattice.get("first_block") if isinstance(lattice, dict) else None
        cert = row.get("cert_detail") or {}
        return {
            "reason": row.get("reason"),
            "detail": row.get("detail"),
            "lattice_reason": (
                lattice.get("lattice_reason") if isinstance(lattice, dict) else None),
            "first_block": block,
            "cert_detail": cert,
            "moveit_failures": row.get("moveit_failures"),
        }
    return None


def _summarize(result, producer, ik_log, wall_s):
    rows = result.get("rows") or []
    graph_edges = []
    for edge in getattr(producer.graph, "edges", {}).values():
        graph_edges.append(_edge_view(edge))
    outcomes = []
    if hasattr(producer.edge_builder, "outcomes"):
        for outcome in producer.edge_builder.outcomes():
            outcomes.append({
                "edge_class": getattr(outcome, "edge_class", None),
                "ok": getattr(outcome, "ok", None),
                "reason": getattr(outcome, "reason", None),
                "fraction": getattr(outcome, "fraction", None),
                "excursion_rad_per_m": getattr(
                    outcome, "excursion_rad_per_m", None),
                "wrist_margin_rad": getattr(outcome, "wrist_margin_rad", None),
                "moveit_error_code": getattr(outcome, "moveit_error_code", None),
                "wall_s": getattr(outcome, "wall_s", None),
            })
    certification = result.get("certification")
    cert_passed = bool(
        certification is not None and getattr(certification, "passed", False))
    chain = []
    for edge in graph_edges:
        if edge["edge_class"] in ("TRAVERSE", "INSERT", "DESCEND", "RETREAT"):
            chain.extend(edge["joints"][1:] if chain else edge["joints"])
    failure = _first_failure(rows)
    return {
        "success": bool(result.get("success")),
        "reason_code": result.get("reason_code"),
        "wall_s": round(wall_s, 3),
        "certified": cert_passed,
        "semantic": _semantic(graph_edges),
        "edges": [
            {key: edge[key] for key in edge if key != "joints"}
            for edge in graph_edges
        ],
        "selected_chain_states": len(chain),
        "selected_chain": chain,
        "outcomes": outcomes,
        "first_failure": failure,
        "ik_attempts": len(ik_log),
        "ik_ok": sum(1 for row in ik_log if row["ok"]),
        "ik": ik_log,
        "stage_timings": result.get("stage_timings"),
        "min_sigma": _min_sigma(rows),
    }


def _min_sigma(rows):
    values = []
    for row in rows or []:
        lattice = row.get("lattice") or (row.get("detail") or {}).get("lattice") or {}
        if isinstance(lattice, dict) and lattice.get("min_sigma") is not None:
            values.append(lattice["min_sigma"])
    return min(values) if values else "not_captured"


def _consistent(left, right):
    if left["success"] != right["success"]:
        return False
    if left["reason_code"] != right["reason_code"]:
        return False
    if left["certified"] != right["certified"]:
        return False
    if left["semantic"] != right["semantic"]:
        return False
    chain_l = left["selected_chain"]
    chain_r = right["selected_chain"]
    if len(chain_l) != len(chain_r):
        return False
    if chain_l and max(
            abs(a - b) for one, two in zip(chain_l, chain_r)
            for a, b in zip(one, two)) > 1e-4:
        return False
    return True


def _decide(reports):
    """Apply the certification rule. None means the decision stays unsigned."""
    signed = []
    notes = []
    for report in reports:
        runs = report["runs"]
        if len(runs) != 2 or not report["consistent"]:
            notes.append("%s repeats disagree" % report["name"])
            continue
        run = runs[0]
        if (run["success"] and run["certified"] and run["semantic"]["complete"]
                and report["equivalent_slot"] is None):
            signed.append(report["name"])
            continue
        failure = run.get("first_failure") or {}
        lattice_reason = str(failure.get("lattice_reason") or "")
        if run["selected_chain_states"] and not run["certified"]:
            notes.append("%s lattice chain, certification %s" % (
                report["name"], failure.get("reason")))
        elif "LAYER_EMPTY" in lattice_reason or failure.get("reason") in (
                "LATTICE_LAYER_EMPTY", "NO_FEASIBLE_ROUTE"):
            notes.append("%s no chain through the corridor" % report["name"])
        else:
            notes.append("%s %s" % (report["name"], run["reason_code"]))
    if signed:
        return {
            "code": "CANDIDATE_GENERATION_TOO_CONSERVATIVE",
            "probes": signed,
            "notes": notes,
        }
    return {"code": None, "notes": notes}


def main():
    import rclpy
    from rclpy.node import Node

    from luggage_description.scene_tf_config_utils import load_scene_tf_config
    from luggage_perception.eval.isolated_domain import isolated_replay_env
    from luggage_planning.routing.lattice import LatticeStateChecker
    from luggage_planning.planning_scene_client import PlanningSceneClient
    from luggage_planning.routing.producer import register_attach_from_request

    os.makedirs(OUT, exist_ok=True)
    residuals = score._residuals()  # noqa: SLF001
    if residuals:
        _dump("manifest.json", {"outcome": "not_evaluated", "residuals": residuals})
        print(json.dumps({"outcome": "not_evaluated", "residuals": residuals}))
        return 2
    probes = _probes()
    for probe in probes:
        scene = json.load(open(probe["scene_path"], encoding="utf-8"))
        ids = [obj.get("id") for obj in scene.get("world_objects") or []]
        if "placed_0_0_0" not in ids:
            raise SystemExit("%s scene has no placed_0_0_0" % probe["name"])
        probe["equivalent_slot"] = _equivalent(
            probe["request_path"], probe["center"], probe["quaternion_xyzw"])
    env = isolated_replay_env(score.DOMAIN)
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    proc = score._start_moveit(  # noqa: SLF001
        env, SCENE_TF, os.path.join(OUT, "moveit.log"))
    rclpy.init()
    node = Node("probe_opening_certify")
    stop, thread = score._spin(node)  # noqa: SLF001
    reports = []
    try:
        from luggage_msgs.srv import ValidateRouteV2

        client = node.create_client(ValidateRouteV2, "/validate_route_v2")
        if not client.wait_for_service(timeout_sec=90.0):
            _dump("manifest.json", {"outcome": "validate_route_unavailable"})
            return 2
        scene_config = load_scene_tf_config(SCENE_TF)
        for probe in probes:
            document, surface, scene, _slot = classify._request_document(  # noqa: SLF001
                probe["request_path"], probe["surface_path"],
                probe["scene_path"], probe["source_slot"])
            world = classify._sync_world(node, scene_config, scene)  # noqa: SLF001
            _clear_occupancy(node)
            occupancy = _add_occupancy(node, surface)
            synthetic = {
                "slot_pose": {
                    "position": probe["center"],
                    "quaternion_xyzw": probe["quaternion_xyzw"],
                }
            }
            stamp_ns = int(surface.get("acquisition_stamp_ns") or 1)
            producer, _urdf, _tcp = classify._producer(  # noqa: SLF001
                node, scene_config, SCENE_TF, None, stamp_ns,
                budget_s=BUDGET_S)
            scene_client = PlanningSceneClient(node)
            producer.corridor_planner = "lattice"
            producer.lattice_checker_factory = (
                lambda scene_states, start_q, _node=node, _client=scene_client:
                LatticeStateChecker(_node, _client, scene_states, start_q))
            original_refine = producer.refine
            runs = []
            for repeat in range(REPEATS):
                ik_log = []
                _wrap_refine(producer, ik_log, original_refine)
                request = classify._build_request(  # noqa: SLF001
                    document, surface, synthetic, "opening-cert", stamp_ns)
                request["box_wdh"] = probe["box_wdh"]
                request["request_id"] = probe["name"]
                request["box_id"] = probe["name"]
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
                _branch_ids(ik_log)
                run = _summarize(result, producer, ik_log, wall)
                run["repeat"] = repeat
                runs.append(run)
                _dump("%s_run_%d.json" % (probe["name"], repeat), run)
                node.get_logger().info(
                    "probe %s run %d %s %.1fs chain=%d" % (
                        probe["name"], repeat, run["reason_code"],
                        wall, run["selected_chain_states"]))
            reports.append({
                "name": probe["name"],
                "center": [round(v, 4) for v in probe["center"]],
                "source_slot": probe["source_slot"],
                "equivalent_slot": probe["equivalent_slot"],
                "world": world,
                "occupancy_cells": occupancy,
                "consistent": _consistent(runs[0], runs[1]),
                "runs": [
                    {key: run[key] for key in run if key != "ik"}
                    for run in runs
                ],
            })
        decision = _decide(reports)
        _dump("summary.json", {"reports": reports, "decision": decision})
        _dump("decision.json", decision)
        print(json.dumps(decision, indent=2))
        return 0
    finally:
        stop.set()
        thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        score._stop_moveit(proc)  # noqa: SLF001
        _dump("manifest.json", {
            "domain": score.DOMAIN,
            "budget_s": BUDGET_S,
            "repeats": REPEATS,
            "scene": "revision 2 measured + placed_0_0_0 + occupancy cells",
            "gazebo": False,
            "online_planner_modified": False,
            "moveit_stopped": proc.poll() is not None,
            "residuals_after": score._residuals(),  # noqa: SLF001
        })


if __name__ == "__main__":
    sys.exit(main() or 0)
