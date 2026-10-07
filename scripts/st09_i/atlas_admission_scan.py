#!/usr/bin/env python3
"""Offline lattice scan of slots the strict atlas gate never planned.

No Gazebo. Isolated MoveIt, plan only. IK, collision, fraction, branch,
retreat and return stay on the production RouteProducer path. The lattice
wall for one candidate is the one-box budget (60 s).

The cohort is the frozen first-box list from g3_online_n3_carryon3:

* slots 639 and 656, the atlas-reachable baseline
* the 56 near-opening slots (container x from -0.395 to -0.095)
* other status-1 slots whose hard_reject_safe is false

Deep pure-unknown slots (container x >= 0.25 and both yaw queries status
0) are not planned. A second carryon, then a standard, is planned only
after a non-baseline slot returns a complete route.
"""

from __future__ import division

import copy
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import corridor_lattice_score as score  # noqa: E402
import slot64_classify_replay as classify  # noqa: E402

ROOT = score.ROOT
RUN = os.path.join(
    ROOT, "docs/status/evidence/frame_migration/ST09-parallel-I-s5/m3",
    "g3_online_n3_carryon3")
REQUEST = os.path.join(RUN, "dumps/box_00_paths/planning_request.json")
CANDIDATES = os.path.join(RUN, "dumps/box_00_paths/candidates.json")
SURFACE = os.path.join(RUN, "dumps/trial_00_ok/replay/surface_2d.json")
SCENE_JSON = os.path.join(RUN, "dumps/trial_00_ok/scene.json")
SCENE_TF = os.path.join(
    ROOT, "src/luggage_description/config/scene_tf_deploy.yaml")
OUT = os.path.join(
    ROOT, "docs/status/evidence/frame_migration/ST09-parallel-I-s5/m3",
    "g3_atlas_admission_near_20261006")
_OUT = {"dir": OUT}
NEAR_X_MAX = -0.09
DEEP_X_MIN = 0.25
BASELINE = (639, 656)
STANDARD_WDH = [0.70, 0.45, 0.28]
# TCP was 0.173 m above the box center on the frozen first-box queries.
TCP_ABOVE_CENTER_M = 0.1736


def _dump(name, document):
    os.makedirs(_OUT["dir"], exist_ok=True)
    with open(os.path.join(_OUT["dir"], name), "w", encoding="utf-8") as handle:
        json.dump(document, handle, indent=1, sort_keys=True, default=str)
        handle.write("\n")


def _append(name, document):
    os.makedirs(_OUT["dir"], exist_ok=True)
    with open(os.path.join(_OUT["dir"], name), "a", encoding="utf-8") as handle:
        handle.write(json.dumps(document, sort_keys=True, default=str))
        handle.write("\n")
        handle.flush()


def _yaw(quaternion):
    x, y, z, w = (float(v) for v in quaternion)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _footprint(box_wdh, quaternion):
    yaw = _yaw(quaternion)
    rotated = abs(abs(yaw) - math.pi / 2.0) < math.radians(10.0)
    length, width = float(box_wdh[0]), float(box_wdh[1])
    if rotated:
        return [width, length]
    return [length, width]


def _pair(queries):
    statuses = tuple(int(item.get("status") or 0) for item in queries or [])
    hard = any(bool(item.get("hard_reject_safe")) for item in queries or [])
    return statuses, hard


def cohort(document):
    """Split the frozen first-box list. Raises if the 56 are not there."""
    near_status1, near_unknown, other_status1 = [], [], []
    baseline = {}
    deep_unknown = 0
    for row in document["candidates"]:
        index = int(row["slot_index"])
        x = float(row["slot_pose"]["position"][0])
        statuses, hard = _pair(row.get("atlas_queries"))
        if index in BASELINE:
            baseline[index] = row
            continue
        if x >= DEEP_X_MIN and statuses == (0, 0):
            deep_unknown += 1
            continue
        item = (x, float(row["slot_pose"]["position"][1]), index, row)
        if -0.40 <= x <= NEAR_X_MAX:
            if 1 in statuses and not hard:
                near_status1.append(item)
            else:
                near_unknown.append(item)
        elif 1 in statuses and not hard:
            other_status1.append(item)
    near_status1.sort()
    near_unknown.sort()
    other_status1.sort()
    if len(baseline) != 2 or deep_unknown != 212:
        raise RuntimeError(
            "cohort drift baseline=%s deep_unknown=%s" % (
                sorted(baseline), deep_unknown))
    near = near_status1 + near_unknown
    if len(near) != 56:
        raise RuntimeError("near-opening count %d != 56" % len(near))
    ordered = (
        [("baseline", baseline[639]), ("baseline", baseline[656])]
        + [("near_status1", item[3]) for item in near_status1]
        + [("near_unknown", item[3]) for item in near_unknown]
        + [("other_status1", item[3]) for item in other_status1])
    return ordered, {
        "near_status1": len(near_status1),
        "near_unknown": len(near_unknown),
        "other_status1": len(other_status1),
        "deep_unknown_skipped": deep_unknown,
    }


def _edge_classes(result):
    classes = []
    for edge in result.get("edges") or ():
        classes.append(str((edge.attributes or {}).get("edge_class") or ""))
    return classes


def _plan_one(producer, document, surface, slot, box_wdh, stamp_ns, label):
    from luggage_planning.reachable_graph import RouteGraph
    from luggage_planning.route_producer import register_attach_from_request

    request = classify._build_request(  # noqa: SLF001
        document, surface, slot, "atlas-admission-scan", stamp_ns)
    request["box_wdh"] = [float(v) for v in box_wdh]
    request["request_id"] = "scan-%s" % label
    request["box_id"] = request["request_id"]
    producer.graph = RouteGraph(static_identity=request["request_id"])
    register_attach_from_request(
        producer.attachment_provider, request,
        box_wdh=request["box_wdh"], box_id=request["box_id"], generation=1)
    started = time.perf_counter()
    result = producer.plan_place_route(request)
    wall = time.perf_counter() - started
    reset = getattr(producer.edge_builder, "reset_scene", None)
    if callable(reset):
        reset()
    adapter = getattr(producer, "_last_adapter", None)
    cert_legs = []
    for item in (getattr(adapter, "requests", None) or []):
        if item.get("kind") != "validate_route":
            continue
        cert_legs = [{
            "edge": leg.get("edge"),
            "reason": leg.get("reason"),
            "n_samples": leg.get("n_samples"),
            "first_invalid": leg.get("first_invalid"),
            "leg_time_ms": round(int(leg.get("leg_time_ns") or 0) / 1e6, 1),
        } for leg in item.get("legs") or []]
    rows = result.get("rows") or []
    row = next((
        item for item in reversed(rows)
        if item.get("reason") not in ("ATTEMPT_SUMMARY", "", None)),
        rows[-1] if rows else {})
    record = {
        "label": label,
        "slot_index": int(slot["slot_index"]),
        "x": round(float(slot["slot_pose"]["position"][0]), 4),
        "y": round(float(slot["slot_pose"]["position"][1]), 4),
        "success": bool(result.get("success")),
        "reason_code": result.get("reason_code"),
        "row_reason": row.get("reason"),
        "certified": bool(row.get("certified")),
        "wall_s": round(wall, 3),
        "edge_classes": _edge_classes(result),
        "legs": len(result.get("legs") or ()),
        "cert_legs": cert_legs,
    }
    print(json.dumps({
        "slot": record["slot_index"], "label": label,
        "success": record["success"], "reason": record["row_reason"],
        "wall_s": record["wall_s"],
    }), flush=True)
    _append("attempts.jsonl", record)
    return record


def _occupy(surface, position, footprint, height):
    resolution = float(surface["resolution"])
    half_l = 0.5 * float(surface["inner_size"][0])
    half_w = 0.5 * float(surface["inner_size"][1])
    x0 = float(position[0]) - float(footprint[0]) / 2.0
    x1 = float(position[0]) + float(footprint[0]) / 2.0
    y0 = float(position[1]) - float(footprint[1]) / 2.0
    y1 = float(position[1]) + float(footprint[1]) / 2.0
    marked = 0
    for ix in range(int(surface["nx"])):
        for iy in range(int(surface["ny"])):
            x = -half_l + (ix + 0.5) * resolution
            y = -half_w + (iy + 0.5) * resolution
            if x0 <= x <= x1 and y0 <= y <= y1:
                surface["state"][ix][iy] = "occupied"
                surface["height"][ix][iy] = max(
                    float(surface["height"][ix][iy]), float(height))
                marked += 1
    return marked


def _query_status(atlas, x, y, z, yaw):
    if atlas is None:
        return None, False
    first = atlas.query(x, y, z, yaw)
    second = atlas.query(x, y, z, yaw + math.pi)
    statuses = (int(first.status), int(second.status))
    hard = bool(first.hard_reject_safe) and bool(second.hard_reject_safe)
    if 3 in statuses:
        return 3, hard
    if 2 in statuses:
        return 2, hard
    if 1 in statuses:
        return 1, hard
    return 0, hard


def _second_slots(surface, placed_position, placed_quat, placed_wdh,
                  box_wdh, atlas, floor_z):
    sys.path.append(os.path.join(
        ROOT, "src", "luggage_packing", "scripts"))
    from capacity_orientation_analysis import (  # noqa: E402
        SOLVER_PARAMS,
        SceneContext,
        make_validator,
    )
    from luggage_packing.placement_solver import (  # noqa: E402
        candidate_aabb,
        solve_placement,
    )

    occupied = copy.deepcopy(surface)
    footprint = _footprint(placed_wdh, placed_quat)
    _occupy(occupied, placed_position, footprint, placed_wdh[2])
    ctx = SceneContext(
        SCENE_TF,
        os.path.join(ROOT, "src/luggage_planning/config/atlas_runtime.yaml"))
    placed = {
        "center_container": [float(v) for v in placed_position],
        "size": [float(v) for v in placed_wdh],
        "footprint": footprint,
    }
    aabb = candidate_aabb(
        placed, ctx.inner_size[2], floor_z=ctx.floor_z)
    result = solve_placement(
        occupied, [float(v) for v in box_wdh],
        allowed_yaws=[0.0, math.pi / 2.0],
        params=dict(SOLVER_PARAMS),
        candidate_validator=make_validator(ctx, [aabb], box_wdh))
    ranked = []
    skipped_deep = 0
    for candidate in result.get("candidates") or []:
        if not candidate.get("feasible"):
            continue
        center = candidate["center_container"]
        x = float(center[0])
        yaw = float(candidate.get("box_yaw") or 0.0)
        status, hard = _query_status(
            atlas, x, float(center[1]),
            float(center[2]) + TCP_ABOVE_CENTER_M, yaw)
        if hard:
            continue
        if status in (2, 3):
            band = 0
        elif x <= NEAR_X_MAX and status == 1:
            band = 1
        elif x <= NEAR_X_MAX:
            band = 2
        elif status == 1:
            band = 3
        elif x >= DEEP_X_MIN and (status in (0, None)):
            skipped_deep += 1
            continue
        else:
            band = 4
        quat = [0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)]
        ranked.append((band, x, float(center[1]), {
            "slot_index": int(candidate.get("candidate_id") or 0)
            if str(candidate.get("candidate_id") or "").isdigit() else 0,
            "candidate_id": str(candidate.get("candidate_id")),
            "slot_pose": {
                "position": [float(v) for v in center],
                "quaternion_xyzw": quat,
            },
            "atlas_status": status,
        }))
    ranked.sort(key=lambda item: item[:3])
    return [item[3] for item in ranked], skipped_deep, floor_z


def _world_box(container_tf, position, quaternion):
    from st09_contracts.attachment import compose_transform

    return compose_transform(
        tuple(float(v) for v in container_tf["translation"]),
        tuple(float(v) for v in container_tf["quaternion_xyzw"]),
        tuple(float(v) for v in position),
        tuple(float(v) for v in quaternion))


def _load_atlas(node, scene_path):
    from luggage_planning.atlas_runtime import (
        compute_reachability_scene_hash,
        compute_scene_tf_hash,
        load_atlas_runtime,
        load_runtime_atlas,
    )
    from luggage_planning.urdf_identity import UrdfIdentity

    cfg_path = os.environ.get("SCAN_ATLAS_CONFIG") or os.path.join(
        ROOT, "src/luggage_planning/config/atlas_runtime.yaml")
    cfg = load_atlas_runtime(cfg_path)
    if os.environ.get("SCAN_ATLAS_WARN"):
        cfg = dict(cfg)
        cfg["hash_policy"] = "warn"
    identity = UrdfIdentity(node)
    identity.start()
    deadline = time.monotonic() + 20.0
    while not identity.ready and time.monotonic() < deadline:
        identity.poll()
        time.sleep(0.1)
    atlas, diag = load_runtime_atlas(
        cfg,
        scene_tf_hash=compute_scene_tf_hash(scene_path),
        urdf_hash=identity.hash,
        reachability_scene_hash=compute_reachability_scene_hash(scene_path))
    diag = dict(diag)
    diag["urdf_ready"] = bool(identity.ready)
    diag["urdf_reason"] = identity.reason
    return atlas, diag


def _search_second(producer, document, surface, placed_slot, client,
                   atlas, floor_z):
    position = placed_slot["slot_pose"]["position"]
    quaternion = placed_slot["slot_pose"]["quaternion_xyzw"]
    placed_wdh = [float(v) for v in document["box_wdh"]]
    world, world_q = _world_box(
        document["container_transform"], position, quaternion)
    added = client.add_collision_box(
        "scan_placed_0", list(world), list(world_q), tuple(placed_wdh))
    ok = added[0] if isinstance(added, tuple) else bool(added)
    if not ok:
        raise RuntimeError("placed box was not added: %r" % (added,))
    records = []
    try:
        for name, box_wdh in (
                ("carryon", placed_wdh),
                ("standard", STANDARD_WDH)):
            slots, skipped, _floor = _second_slots(
                surface, position, quaternion, placed_wdh, box_wdh,
                atlas, floor_z)
            if os.environ.get("SCAN_BOX2_NEAR_ONLY"):
                slots = [
                    item for item in slots
                    if float(item["slot_pose"]["position"][0]) <= NEAR_X_MAX
                    or item.get("atlas_status") in (2, 3)]
            limit = int(os.environ.get("SCAN_BOX2_LIMIT") or 0)
            if limit > 0:
                slots = slots[:limit]
            print(json.dumps({
                "second": name, "candidates": len(slots),
                "deep_skipped": skipped,
                "after_slot": int(placed_slot["slot_index"]),
            }), flush=True)
            for index, slot in enumerate(slots):
                slot = dict(slot)
                slot["slot_index"] = index
                label = "box2-%s-after-%s-%d" % (
                    name, placed_slot["slot_index"], index)
                record = _plan_one(
                    producer, document, surface, slot, box_wdh,
                    int(surface.get("acquisition_stamp_ns") or 1), label)
                record["second_box"] = name
                record["after_slot"] = int(placed_slot["slot_index"])
                record["atlas_status"] = slot.get("atlas_status")
                records.append(record)
                _append("box2.jsonl", record)
                if record["success"] and not os.environ.get("SCAN_COUNT_BOX2"):
                    return records
    finally:
        client.remove_objects(["scan_placed_0"])
    return records


def _static_empty_surface(surface, document):
    """Revision-1 empty grid for the first box.

    ``planning_request.json`` records the revision and geometry hash but
    not the cells. The post-run surface supplies the grid geometry; its
    occupancy is cleared so unloaded legs do not see a box that was not
    there when slot 656 was certified online.
    """
    empty = copy.deepcopy(surface)
    state = empty.get("state") or []
    empty["state"] = [["free" for _cell in row] for row in state]
    height = empty.get("height") or []
    floor = float(empty.get("floor_z") or 0.0)
    if height:
        empty["height"] = [[floor for _cell in row] for row in height]
    empty["map_revision"] = int(document.get("map_revision") or 1)
    empty["map_source"] = str(
        document.get("map_source") or "static_empty")
    empty["placed_count"] = 0
    empty["cargo_observed"] = False
    if document.get("geometry_hash"):
        empty["geometry_hash"] = document["geometry_hash"]
    return empty


def main():
    import rclpy
    from rclpy.node import Node

    from luggage_description.scene_tf_config_utils import load_scene_tf_config
    from luggage_perception.eval.isolated_domain import isolated_replay_env
    from luggage_planning.lattice_route import LatticeStateChecker
    from luggage_planning.planning_scene_client import PlanningSceneClient
    from luggage_planning.route_producer import attempt_tier  # noqa: F401

    if os.environ.get("SCAN_OUT"):
        _OUT["dir"] = os.environ["SCAN_OUT"]
    elif os.environ.get("SCAN_SLOTS"):
        name = ("g3_atlas_admission_seeded_20261006"
                if os.environ.get("SCAN_ATLAS_WARN")
                else "g3_atlas_admission_chain_retry_20261006")
        _OUT["dir"] = os.path.join(os.path.dirname(OUT), name)
    os.makedirs(_OUT["dir"], exist_ok=True)
    document = json.load(open(REQUEST, encoding="utf-8"))
    captured = json.load(open(CANDIDATES, encoding="utf-8"))
    surface = _static_empty_surface(
        json.load(open(SURFACE, encoding="utf-8")), document)
    ordered, counts = cohort({"candidates": captured["rows"]})
    manifest = {
        "request": REQUEST,
        "surface_template": SURFACE,
        "surface_identity": {
            "map_revision": int(surface["map_revision"]),
            "map_source": surface["map_source"],
            "geometry_hash": surface.get("geometry_hash"),
            "occupied_cells": 0,
        },
        "scene_json": SCENE_JSON,
        "scene_tf": SCENE_TF,
        "counts": counts,
        "planned": len(ordered),
        "deep_unknown_not_planned": counts["deep_unknown_skipped"],
        "baseline_slots": list(BASELINE),
        "lattice_budget_s": 60.0,
        "gates": "production IK, collision, fraction, branch, retreat, return",
        "started": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    }
    manifest.update(score._commit() if hasattr(score, "_commit") else {})
    # corridor_lattice_g2 owns the commit helper the other gates use.
    import corridor_lattice_g2 as g2  # noqa: E402
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
    launch_args = {}
    if os.environ.get("SCAN_GZ_URDF"):
        # Same URDF the online sim certifier loads: Gazebo plugin plus
        # the pickup_observe_sim initial joints. model_sha256 then matches
        # the online 656 capture (759bfab2...).
        launch_args = {
            "use_gz_sim": "true",
            "hardware_plugin": "gz_ros2_control/GazeboSimSystem",
            "xacro_extra": (
                "initial_joint1:=-1.250000,initial_joint2:=-1.746600,"
                "initial_joint3:=-0.997400,initial_joint4:=-1.915300,"
                "initial_joint5:=1.771400,initial_joint6:=1.861000"),
        }
    proc = score._start_moveit(  # noqa: SLF001
        env, SCENE_TF, os.path.join(_OUT["dir"], "moveit.log"),
        launch_args)
    rclpy.init()
    node = Node("atlas_admission_scan")
    stop, thread = score._spin(node)  # noqa: SLF001
    summary = {
        "outcome": "incomplete",
        "counts": counts,
        "baseline": {},
        "near_complete": [],
        "box2_complete": [],
    }
    try:
        from luggage_msgs.srv import ValidateRouteV2

        probe = node.create_client(ValidateRouteV2, "/validate_route_v2")
        if not probe.wait_for_service(timeout_sec=90.0):
            summary["detail"] = "validate_route_v2 did not come up"
            return 2
        scene_config = load_scene_tf_config(SCENE_TF)
        client, _obj, triangles = score._sync_world(node, scene_config)  # noqa: SLF001
        manifest["container_triangles"] = triangles
        atlas, atlas_diag = _load_atlas(node, SCENE_TF)
        manifest["atlas"] = {
            "loaded": bool(atlas_diag.get("loaded")),
            "reason": atlas_diag.get("reason"),
            "hash_message": atlas_diag.get("hash_message"),
        }
        _dump("manifest.json", manifest)
        stamp_ns = int(surface.get("acquisition_stamp_ns") or 1)
        producer, _urdf, _tcp = classify._producer(  # noqa: SLF001
            node, scene_config, SCENE_TF, atlas, stamp_ns, budget_s=60.0)
        scene_client = PlanningSceneClient(node)
        producer.corridor_planner = "lattice"
        producer.lattice_checker_factory = (
            lambda scene_states, start_q: LatticeStateChecker(
                node, scene_client, scene_states, start_q))
        if os.environ.get("SCAN_CERT_DUMP"):
            factory = producer.validity_adapter_factory

            def _capturing(**kwargs):
                adapter = factory(**kwargs)
                producer._last_adapter = adapter
                return adapter

            producer.validity_adapter_factory = _capturing
        floor_z = float(scene_config["container"]["inner"]["floor_z"])
        only = {
            int(item) for item in os.environ.get("SCAN_SLOTS", "").split(",")
            if item.strip()}
        repeats = max(1, int(os.environ.get("SCAN_REPEATS", "1") or 1))
        if only:
            ordered = [
                (tier, slot) for tier, slot in ordered
                if int(slot["slot_index"]) in only]
        for tier, slot in ordered:
            record = None
            for _repeat in range(repeats):
                record = _plan_one(
                    producer, document, surface, slot, document["box_wdh"],
                    stamp_ns, tier)
                record["tier"] = tier
                record["repeat"] = _repeat
                if record["success"] and not os.environ.get(
                        "SCAN_FORCE_REPEATS"):
                    break
            if record is None:
                continue
            if tier == "baseline":
                summary["baseline"][str(record["slot_index"])] = record
                _dump("summary.json", summary)
            elif record["success"]:
                summary["near_complete"].append(record)
                _dump("summary.json", summary)
            if not record["success"] or os.environ.get("SCAN_NO_BOX2"):
                continue
            box2 = _search_second(
                producer, document, surface, slot, client, atlas, floor_z)
            hits = [row for row in box2 if row.get("success")]
            summary["box2_complete"].extend(hits)
            summary["downstream_complete_route_count"] = {}
            for row in summary["box2_complete"]:
                key = "%s:%s" % (row.get("after_slot"), row.get("second_box"))
                summary["downstream_complete_route_count"][key] = (
                    summary["downstream_complete_route_count"].get(key, 0) + 1)
            _dump("summary.json", summary)
            if hits and not os.environ.get("SCAN_COUNT_BOX2"):
                summary["outcome"] = "two_box_route"
                return 0
        if summary["box2_complete"]:
            summary["outcome"] = "two_box_route"
        elif summary["near_complete"] or any(
                row.get("success") for row in
                (summary.get("baseline") or {}).values()):
            summary["outcome"] = "first_box_only"
        else:
            summary["outcome"] = "no_complete_route"
        return 0
    finally:
        summary["ended"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        _dump("summary.json", summary)
        stop.set()
        thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        score._stop_moveit(proc)  # noqa: SLF001
        manifest["ended"] = summary["ended"]
        manifest["outcome"] = summary.get("outcome")
        manifest["moveit_stopped"] = proc.poll() is not None
        manifest["residuals_after"] = score._residuals()  # noqa: SLF001
        _dump("manifest.json", manifest)
        print(json.dumps({
            "outcome": summary.get("outcome"),
            "baseline": {
                key: {
                    "success": row.get("success"),
                    "reason": row.get("row_reason"),
                    "wall_s": row.get("wall_s"),
                } for key, row in (summary.get("baseline") or {}).items()
            },
            "near_complete": [
                row.get("slot_index") for row in summary.get("near_complete") or []
            ],
            "box2_complete": [
                {
                    "after": row.get("after_slot"),
                    "box": row.get("second_box"),
                    "slot": row.get("slot_index"),
                } for row in summary.get("box2_complete") or []
            ],
        }, indent=2), flush=True)


if __name__ == "__main__":
    sys.exit(main())
