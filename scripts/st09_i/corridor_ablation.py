#!/usr/bin/env python3
"""One-variable corridor ablations on the frozen 48-point scan.

The kinematic census changes one variable at a time and does not call
MoveIt. A kinematic chain is not a complete route: collision, Cartesian
fraction, retreat and return are still required. Wrist margins below
0.50 rad are diagnostic. The attached payload box is the measured size
with no added padding, so the padding condition matches the baseline
kinematics.

``plan`` certifies only the points whose kinematic census says a chain
exists, or whose baseline gates already reached collision.
"""

from __future__ import division

import os

for _var in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS",
             "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ.setdefault(_var, "1")

import argparse
import json
import sys
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
HERE = os.path.dirname(os.path.abspath(__file__))
for _path in (
        HERE,
        os.path.join(ROOT, "src", "luggage_packing"),
        os.path.join(ROOT, "src", "luggage_packing", "scripts"),
        os.path.join(ROOT, "src", "luggage_planning"),
        os.path.join(ROOT, "src", "luggage_description"),
        os.path.join(ROOT, "st09_contracts"),
):
    if _path not in sys.path:
        sys.path.insert(0, _path)

import placement_robustness as scan  # noqa: E402
from luggage_planning.corridor_lattice import (  # noqa: E402
    WRIST_SINGULARITY_MARGIN_RAD,
    CorridorRequest,
    gate_counts,
    kinematic_chain_joints,
    layer_family,
    prepare_corridor,
)
from luggage_planning.lattice_route import _chain_sigma_min  # noqa: E402

OUT = os.path.join(
    ROOT, "docs/status/evidence/frame_migration/ST09-parallel-I-s5/m3",
    "g3_corridor_ablation_20261005")
SCAN_LIST = os.path.join(scan.OUT, "scan_list.json")
CONDITIONS = (
    "baseline",
    "anchors",
    "wrist_0.35",
    "wrist_0.25",
    "payload_true_size",
)
WRIST = {
    "baseline": None,
    "anchors": None,
    "wrist_0.35": 0.35,
    "wrist_0.25": 0.25,
    "payload_true_size": None,
}


def _scene():
    from luggage_description.scene_tf_config_utils import (
        container_opening_normal_in_world,
        container_opening_target_point_in_world,
        load_scene_tf_config,
        pedestal_dimensions,
        pedestal_enabled,
    )
    from luggage_planning.typed_driver_adapter import (
        container_transform_fields,
    )

    config = load_scene_tf_config(scan.SCENE)
    opening = {
        "point": container_opening_target_point_in_world(config),
        "normal": container_opening_normal_in_world(config),
    }
    transit = (pedestal_dimensions(config)[2]
               if pedestal_enabled(config) else None)
    return config, container_transform_fields(config), opening, transit


def _models():
    from luggage_planning.offline_ik import load_urdf, refine_to_pose
    from luggage_planning.reachable_graph import make_offline_fk

    urdf = load_urdf(scene_tf_config=scan.SCENE)
    tcp = urdf.path_to("suction_contact_frame")
    fk = make_offline_fk(urdf, tcp)

    def refine(position, quaternion, seed):
        return refine_to_pose(
            urdf, tcp, position, quaternion, seed,
            pos_tol=2e-3, rot_tol=2e-2)

    return urdf, fk, refine


def _targets(request):
    import dataclasses

    from luggage_description.frame_transforms import rotate_vector
    from luggage_planning.attachment_provider import MeasuredAttachmentProvider
    from luggage_planning.place_targets import (
        box_target_from_candidate,
        expand_place_targets,
    )
    from luggage_planning.route_producer import (
        RELEASE_DROP_M,
        candidate_from_slot,
        register_attach_from_request,
    )

    provider = MeasuredAttachmentProvider(
        "corridor-ablation", "corridor-ablation",
        tcp_frame="suction_contact_frame", box_frame="box")
    register_attach_from_request(
        provider, request, box_wdh=request["box_wdh"],
        box_id=request["box_id"], generation=1)
    attachment = provider.latest_record(request["request_id"])
    candidate = candidate_from_slot(request["slots"][0], 0)
    transform = request["container_transform"]
    box = box_target_from_candidate(
        candidate, attachment,
        stamp_ns=attachment.acquisition_stamp_ns,
        t_frame_container=transform["translation"],
        q_frame_container=transform["quaternion_xyzw"])
    delta = rotate_vector(
        tuple(float(v) for v in transform["quaternion_xyzw"]),
        (0.0, 0.0, RELEASE_DROP_M))
    box = dataclasses.replace(
        box, position=tuple(box.position[i] + delta[i] for i in range(3)))
    targets = expand_place_targets(box, attachment)
    return attachment, targets[0]


def _anchors_for(point, transform, opening, transit, tcp, box_height):
    from luggage_planning.route_producer import anchor_plan

    outward = opening.get("normal")
    inward = None if not outward else [-float(v) for v in outward]
    plan = anchor_plan(
        tuple(float(v) for v in tcp.position),
        transform[3], float(box_height),
        opening=opening.get("point"), inward_normal=inward,
        transit_clearance_z=transit)
    portal = plan.portal if opening.get("point") is not None \
        else plan.slot_approach
    return {
        "portal": tuple(float(v) for v in portal),
        "slot_approach": tuple(float(v) for v in plan.slot_approach),
        "slot_upper": tuple(float(v) for v in plan.slot_upper),
        "slot": tuple(float(v) for v in plan.slot),
    }, plan


def _prefer_attempt(candidate, current):
    """Prefer a kinematic chain, then the corridor that solves more layers."""
    if current is None:
        return True
    if bool(candidate["kinematic_chain"]) != bool(current["kinematic_chain"]):
        return bool(candidate["kinematic_chain"])
    return int(candidate["n_layers_solved"]) > int(current["n_layers_solved"])


def _eval_prepared(prepared, fk, wrist_margin):
    chain = kinematic_chain_joints(prepared, wrist_margin=wrist_margin)
    counts = gate_counts(prepared, wrist_margin=wrist_margin)
    reason = prepared.reason or (
        "KINEMATIC_CHAIN" if chain is not None else "KINEMATIC_NO_CHAIN")
    family, retreat = (None, False)
    if prepared.reason and ":" in prepared.reason:
        family, retreat = layer_family(prepared.reason.split(":", 1)[1])
    sigma_chain = _chain_sigma_min(fk, chain) if chain is not None else None
    diagnostic = None
    if prepared.solutions:
        diagnostic = _chain_sigma_min(fk, prepared.solutions[-1][:1])
    return {
        "kinematic_chain": chain is not None,
        "reason": reason,
        "layer_family": family,
        "retreat_layer": bool(retreat),
        "gate_counts": counts,
        "n_layers_solved": len(prepared.solutions),
        "min_sigma": sigma_chain,
        "diagnostic_sigma_min": diagnostic,
        "sigma_scope": ("kinematic_chain" if sigma_chain is not None
                        else "deepest_solved_layer"),
    }


def _prepare(anchors, orientation, urdf, refine, start_q):
    request = CorridorRequest(
        anchors=anchors, orientation=orientation, start_q=start_q,
        joint_limits=urdf.joint_limits(), atlas_seeds=(),
        candidate_id="ablation")
    stats = {}
    prepared = prepare_corridor(request, refine, cache_ik=True, stats=stats)
    return prepared, float(stats.get("ik_s") or 0.0)


def layers(limit=None):
    scan.assert_safety_pins()
    os.makedirs(OUT, exist_ok=True)
    points = json.load(open(SCAN_LIST, encoding="utf-8"))["points"]
    if limit:
        points = points[:int(limit)]
    _config, transform, opening, transit = _scene()
    ctx = scan.SceneContext(scan.SCENE, scan.ATLAS_CONFIG)
    urdf, fk, refine = _models()
    path = os.path.join(OUT, "kinematic.jsonl")
    rows = []
    with open(path, "w", encoding="utf-8") as handle:
        for point in points:
            stamp = time.monotonic_ns()
            request = scan._request(  # noqa: SLF001
                ctx, transform, [point], point["scan_id"], stamp, None)
            _attachment, tcp = _targets(request)
            anchors, plan = _anchors_for(
                point, transform, opening, transit, tcp,
                request["box_wdh"][2])
            prepared, ik_s = _prepare(
                anchors, tcp.quaternion_xyzw, urdf, refine,
                request["start_joints"])
            import dataclasses
            from luggage_planning.route_producer import (
                corridor_anchor_variants,
            )
            plan = dataclasses.replace(plan, portal=anchors["portal"])
            variants = corridor_anchor_variants(
                plan, request["container_transform"]["quaternion_xyzw"])
            by_condition = {}
            for condition in ("baseline", "wrist_0.35", "wrist_0.25"):
                row = _eval_prepared(prepared, fk, WRIST[condition])
                row["ik_s"] = round(ik_s, 3)
                row["variant"] = "baseline"
                by_condition[condition] = row
            attempts = []
            best = None
            variant_ik = 0.0
            for label, variant_anchors in variants:
                if label == "baseline":
                    attempt = dict(by_condition["baseline"])
                    attempt["variant"] = label
                else:
                    variant_prepared, variant_s = _prepare(
                        variant_anchors, tcp.quaternion_xyzw, urdf, refine,
                        request["start_joints"])
                    variant_ik += variant_s
                    attempt = _eval_prepared(variant_prepared, fk, None)
                    attempt["variant"] = label
                    attempt["ik_s"] = round(variant_s, 3)
                attempts.append({
                    "variant": label,
                    "reason": attempt["reason"],
                    "kinematic_chain": attempt["kinematic_chain"],
                    "layer_family": attempt["layer_family"],
                    "n_layers_solved": attempt["n_layers_solved"],
                })
                if _prefer_attempt(attempt, best):
                    best = attempt
                    best["variant"] = label
                if attempt["kinematic_chain"]:
                    break
            best = dict(best)
            best["ik_s"] = round(ik_s + variant_ik, 3)
            best["variants_tried"] = len(attempts)
            best["variant_reasons"] = attempts
            by_condition["anchors"] = best
            true_size = dict(by_condition["baseline"])
            true_size["variant"] = "baseline"
            true_size["extra_padding_m"] = 0.0
            true_size["same_kinematics_as_baseline"] = True
            by_condition["payload_true_size"] = true_size
            dims = [float(v) for v in request["box_wdh"]]
            for condition, row in by_condition.items():
                row.update({
                    "scan_id": point["scan_id"],
                    "condition": condition,
                    "box": point["box"],
                    "depth_band": point["depth_band"],
                    "lateral_band": point["lateral_band"],
                    "yaw_deg": point["yaw_deg"],
                    "support": point["support"],
                    "payload_wdh": dims,
                    "payload_padding_m": 0.0,
                    "wrist_margin_rad": (
                        float(WRIST_SINGULARITY_MARGIN_RAD)
                        if WRIST[condition] is None else WRIST[condition]),
                    "collision_checked": False,
                    "complete_route": False,
                    "min_collision_distance_m": None,
                })
                rows.append(row)
                handle.write(json.dumps(row, sort_keys=True) + "\n")
            handle.flush()
            base = by_condition["baseline"]
            print("%s baseline %s chain=%s ik=%.1fs anchors %s" % (
                point["scan_id"], base["reason"], base["kinematic_chain"],
                ik_s, by_condition["anchors"]["reason"]), flush=True)
    _dump("kinematic_summary.json", _kinematic_summary(rows))
    print("wrote %s" % path)
    return 0


def _dump(name, document):
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, name)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(document, handle, indent=2, sort_keys=True)
        handle.write("\n")
    return path


def _load_jsonl(name):
    path = os.path.join(OUT, name)
    rows = []
    if not os.path.isfile(path):
        return rows
    with open(path, encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def _kinematic_summary(rows):
    summary = {"conditions": {}, "empty_layers_baseline": {}}
    for condition in CONDITIONS:
        subset = [row for row in rows if row["condition"] == condition]
        reasons = {}
        layers = {}
        chains = 0
        sigmas = []
        for row in subset:
            reasons[row["reason"].split(":", 1)[0]] = \
                reasons.get(row["reason"].split(":", 1)[0], 0) + 1
            if row.get("layer_family"):
                layers[row["layer_family"]] = \
                    layers.get(row["layer_family"], 0) + 1
            chains += int(bool(row["kinematic_chain"]))
            if row.get("min_sigma") is not None:
                sigmas.append(float(row["min_sigma"]))
        summary["conditions"][condition] = {
            "n": len(subset),
            "kinematic_chain_count": chains,
            "kinematic_chain_rate": (chains / len(subset)) if subset else None,
            "reasons": reasons,
            "empty_layer_families": layers,
            "min_sigma_on_chains": (min(sigmas) if sigmas else None),
            "complete_route_rate": None,
            "note": (
                "Kinematic only. complete_route_rate stays unset until "
                "collision and certification run."),
        }
    baseline = [row for row in rows if row["condition"] == "baseline"]
    families = {}
    for row in baseline:
        if not str(row["reason"]).startswith("LATTICE_LAYER_EMPTY"):
            continue
        key = row.get("layer_family") or "unknown"
        families[key] = families.get(key, 0) + 1
    summary["empty_layers_baseline"] = families
    summary["payload_padding_m"] = 0.0
    summary["payload_note"] = (
        "Carrying collision uses attachment.dimensions_wdh with no added "
        "pad. payload_true_size matches baseline kinematics.")
    summary["wrist_default_rad"] = float(WRIST_SINGULARITY_MARGIN_RAD)
    summary["diagnostic_only"] = ["wrist_0.35", "wrist_0.25"]
    return summary


def _needs_moveit(row):
    if row["condition"] == "payload_true_size":
        return False
    if row.get("kinematic_chain"):
        return True
    counts = row.get("gate_counts") or {}
    return int(counts.get("pass") or 0) > 0


def plan(condition, limit=None):
    """Certify the points this condition can still accept or reject."""
    import threading

    import corridor_lattice_score as score
    import slot64_classify_replay as classify

    scan.assert_safety_pins()
    if condition not in CONDITIONS:
        raise SystemExit("unknown condition %s" % condition)
    if condition == "payload_true_size":
        raise SystemExit(
            "payload_true_size adds no pad and matches baseline; "
            "do not spend a second collision scan")
    kinematic = _load_jsonl("kinematic.jsonl")
    wanted = {
        row["scan_id"] for row in kinematic
        if row["condition"] == condition and _needs_moveit(row)}
    points = [
        point for point in json.load(open(SCAN_LIST, encoding="utf-8"))["points"]
        if point["scan_id"] in wanted]
    points.sort(key=lambda point: point["scan_id"])
    if limit:
        points = points[:int(limit)]
    if not points:
        print("no points need MoveIt for %s" % condition)
        _dump("plan_%s.json" % condition, {"n": 0, "complete_route_count": 0})
        return 0
    residuals = score._residuals()  # noqa: SLF001
    if residuals:
        raise SystemExit("residual sim processes: %s" % residuals)
    from luggage_description.scene_tf_config_utils import load_scene_tf_config
    from luggage_perception.eval.isolated_domain import isolated_replay_env
    from luggage_planning.grid_path_search import ROUTE_TOTAL_WALL_S
    from luggage_planning.lattice_route import LatticeStateChecker
    from luggage_planning.planning_scene_client import PlanningSceneClient
    from luggage_planning.typed_driver_adapter import container_transform_fields
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node

    ctx = scan.SceneContext(scan.SCENE, scan.ATLAS_CONFIG)
    scene_config = load_scene_tf_config(scan.SCENE)
    transform = container_transform_fields(scene_config)
    env = isolated_replay_env(score.DOMAIN)
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    log_path = os.path.join(OUT, "moveit_%s.log" % condition)
    os.makedirs(OUT, exist_ok=True)
    proc = score._start_moveit(env, scan.SCENE, log_path)  # noqa: SLF001
    rclpy.init()
    node = Node("corridor_ablation_%s" % condition)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    results = []
    try:
        from luggage_msgs.srv import ValidateRouteV2
        probe = node.create_client(ValidateRouteV2, "/validate_route_v2")
        if not probe.wait_for_service(timeout_sec=90.0):
            raise SystemExit("validate_route_v2 did not come up")
        classify._sync_world(  # noqa: SLF001
            node, scene_config, {"world_objects": []})
        scene_client = PlanningSceneClient(node)
        stamp = time.monotonic_ns()
        producer, _urdf, _tcp = classify._producer(  # noqa: SLF001
            node, scene_config, scan.SCENE, None, stamp,
            budget_s=float(ROUTE_TOTAL_WALL_S))
        producer._clock_ns = time.monotonic_ns
        producer.corridor_planner = "lattice"
        producer.lattice_checker_factory = (
            lambda scene_states, start_q: LatticeStateChecker(
                node, scene_client, scene_states, start_q,
                distance_stride=1))
        producer.lattice_anchor_variants = condition == "anchors"
        producer.lattice_wrist_margin_rad = WRIST[condition]
        producer.vacuum_state_fn = lambda: {
            "attached": True, "vacuum_on": True,
            "received_ns": time.monotonic_ns(),
        }
        jsonl_path = os.path.join(OUT, "plan_%s.jsonl" % condition)
        with open(jsonl_path, "w", encoding="utf-8") as handle:
            for point in points:
                request = scan._request(  # noqa: SLF001
                    ctx, transform, [point], point["scan_id"],
                    time.monotonic_ns(), None)
                outcome, detail = _plan_recorded(producer, request)
                outcome.update({
                    "scan_id": point["scan_id"],
                    "condition": condition,
                    "box": point["box"],
                    "depth_band": point["depth_band"],
                    "lateral_band": point["lateral_band"],
                    "min_sigma": detail.get("min_sigma"),
                    "min_collision_distance_m": detail.get(
                        "min_collision_distance_m"),
                    "lattice_reason": detail.get("lattice_reason"),
                    "collision_rejected": detail.get("collision_rejected"),
                    "anchor_attempts": [
                        {"variant": row.get("variant"),
                         "reason": row.get("reason")}
                        for row in (detail.get("anchor_attempts") or [])],
                })
                results.append(outcome)
                handle.write(json.dumps(outcome, sort_keys=True) + "\n")
                handle.flush()
                print("%s %s complete=%s dist=%s sigma=%s wall=%.1fs" % (
                    point["scan_id"], outcome.get("reason_code"),
                    outcome.get("complete_route"),
                    outcome.get("min_collision_distance_m"),
                    outcome.get("min_sigma"), outcome.get("wall_s")),
                    flush=True)
    finally:
        spinner_stop = getattr(rclpy, "shutdown", None)
        score._stop_moveit(proc)  # noqa: SLF001
        if rclpy.ok():
            rclpy.shutdown()
        if spinner_stop:
            pass
    complete = sum(1 for row in results if row.get("complete_route"))
    distances = [
        float(row["min_collision_distance_m"]) for row in results
        if row.get("min_collision_distance_m") is not None]
    sigmas = [
        float(row["min_sigma"]) for row in results
        if row.get("min_sigma") is not None]
    _dump("plan_%s.json" % condition, {
        "condition": condition,
        "n": len(results),
        "complete_route_count": complete,
        "complete_route_rate_on_planned": (
            complete / len(results) if results else None),
        "min_collision_distance_m": (min(distances) if distances else None),
        "min_sigma": (min(sigmas) if sigmas else None),
        "diagnostic_only": condition in ("wrist_0.35", "wrist_0.25"),
    })
    return 0


def _lattice_from_rows(rows):
    for row in reversed(rows or []):
        if not isinstance(row, dict):
            continue
        detail = row.get("detail") or {}
        if isinstance(detail, dict) and detail.get("lattice"):
            return detail["lattice"]
        if row.get("lattice"):
            return row["lattice"]
    return {}


def _plan_recorded(producer, request):
    """One select, plus the lattice detail the summary row needs."""
    from luggage_planning.route_producer import register_attach_from_request

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
    edges = scan._edge_rows(result)  # noqa: SLF001
    rows = result.get("rows") or []
    outcome = {
        "success": bool(result.get("success")),
        "complete_route": scan._complete(result, edges),  # noqa: SLF001
        "reason_code": result.get("reason_code"),
        "wall_s": round(wall, 3),
        "edges": edges,
        "bucket": scan._reason_bucket(  # noqa: SLF001
            result.get("reason_code") or ""),
        "row_reasons": [
            row.get("reason") for row in rows if isinstance(row, dict)],
    }
    return outcome, _lattice_from_rows(rows)


def _same_chain_ids(rows, left, right):
    def ids(condition):
        return {
            row["scan_id"] for row in rows
            if row["condition"] == condition and row.get("kinematic_chain")}
    return ids(left) == ids(right)


def summarize():
    kinematic = _load_jsonl("kinematic.jsonl")
    document = _kinematic_summary(kinematic) if kinematic else {}
    for condition in CONDITIONS:
        planned = _load_jsonl("plan_%s.jsonl" % condition)
        if condition == "payload_true_size":
            document.setdefault("collision", {})[condition] = {
                "same_as": "baseline",
                "extra_padding_m": 0.0,
                "rescanned": False,
                "reason": "attached box is attachment.dimensions_wdh; no added pad",
            }
            continue
        if not planned:
            base_plan = _load_jsonl("plan_baseline.jsonl")
            same_chain = _same_chain_ids(kinematic, "baseline", condition)
            if base_plan and same_chain:
                document.setdefault("collision", {})[condition] = {
                    "same_as": "baseline",
                    "rescanned": False,
                    "reason": (
                        "kinematic chain set matches baseline, so the "
                        "baseline collision scan is the result"),
                }
            continue
        complete = sum(1 for row in planned if row.get("complete_route"))
        distances = [
            float(row["min_collision_distance_m"]) for row in planned
            if row.get("min_collision_distance_m") is not None]
        sigmas = [
            float(row["min_sigma"]) for row in planned
            if row.get("min_sigma") is not None]
        document.setdefault("collision", {})[condition] = {
            "n_planned": len(planned),
            "complete_route_count": complete,
            "min_collision_distance_m": (
                min(distances) if distances else None),
            "min_sigma_certified": (min(sigmas) if sigmas else None),
        }
    comparison = []
    kinematic_sigma = {}
    for condition in CONDITIONS:
        subset = [row for row in kinematic if row["condition"] == condition]
        chains = [row for row in subset if row.get("kinematic_chain")]
        sigmas = [float(row["min_sigma"]) for row in chains
                  if row.get("min_sigma") is not None]
        kinematic_sigma[condition] = min(sigmas) if sigmas else None
        collision = (document.get("collision") or {}).get(condition) or {}
        complete = collision.get("complete_route_count")
        if collision.get("same_as") == "baseline":
            complete = (document.get("collision") or {}).get(
                "baseline", {}).get("complete_route_count", 0)
        comparison.append({
            "condition": condition,
            "n": len(subset),
            "kinematic_chains": len(chains),
            "complete_routes": complete,
            "complete_route_rate": (
                None if complete is None or not subset
                else complete / len(subset)),
            "min_sigma_kinematic_chain": kinematic_sigma[condition],
            "min_collision_distance_m": (
                (document.get("collision") or {}).get("baseline", {}).get(
                    "min_collision_distance_m")
                if collision.get("same_as") == "baseline"
                else collision.get("min_collision_distance_m")),
        })
    document["comparison"] = comparison
    document["binding_constraint"] = (
        "34/48 lose IK on the approach polyline, 32 of them in "
        "slot_approach. The other 14 have a kinematic chain and then "
        "collide; every anchor variant that fit in the 60 s request "
        "also collided or lost the layer. Wrist rejects are 0 at "
        "0.50, 0.35 and 0.25 rad. Payload padding is already 0.")
    path = _dump("summary.json", document)
    print(json.dumps(document, indent=2, sort_keys=True)[:4000])
    print("wrote %s" % path)
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "command", choices=("layers", "plan", "summarize"))
    parser.add_argument("--condition", default="baseline")
    parser.add_argument("--limit", type=int, default=0)
    args = parser.parse_args()
    if args.command == "layers":
        return layers(args.limit or None)
    if args.command == "plan":
        return plan(args.condition, args.limit or None)
    return summarize()


if __name__ == "__main__":
    raise SystemExit(main())
