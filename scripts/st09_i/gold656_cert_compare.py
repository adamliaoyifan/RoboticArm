#!/usr/bin/env python3
"""Compare slot 656's executed trajectory with the three opening probes.

One domain-42 ValidateRouteV2 process. The executed 656 joint trace is
certified as recorded; it is not re-solved. Probe routes are planned once
so the certifier sees the same edge samples as the previous run. A 30 s
call is diagnostic only.
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
import probe_opening_certify as probes  # noqa: E402
import slot64_classify_replay as classify  # noqa: E402

RUN = os.path.join(
    ROOT, "docs/status/evidence/frame_migration/ST09-parallel-I-s5/m3",
    "g3_online_n3_budget180")
BAG = os.path.join(RUN, "bag")
TRIAL = os.path.join(RUN, "dumps/trial_00_ok")
SCENE_TF = os.path.join(
    ROOT, "src/luggage_description/config/scene_tf_deploy.yaml")
OUT = os.path.join(
    ROOT, "docs/status/evidence/frame_migration",
    "ST09-parallel-I-s6-readiness/placement_corridor_supergraph_g1",
    "gold656_cert")
JOINTS = tuple("elfin_joint%d" % i for i in range(1, 7))
# State timeline of the executed place, sim time.
LEGS = (
    ("transit", 54.411, 61.817, "carrying"),
    ("transit_2", 61.817, 66.595, "carrying"),
    ("traverse", 66.595, 72.894, "carrying"),
    ("insert", 72.894, 74.072, "carrying"),
    ("descend", 74.072, 75.344, "carrying"),
    ("place_retreat", 75.939, 94.607, "unloaded"),
    ("return_home", 94.607, 96.177, "unloaded"),
)


def _dump(name, document):
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, name), "w", encoding="utf-8") as handle:
        json.dump(document, handle, indent=2, default=str)
        handle.write("\n")


def _executed_legs():
    from rclpy.serialization import deserialize_message
    from rosbag2_py import (
        ConverterOptions, SequentialReader, StorageFilter, StorageOptions,
    )
    from sensor_msgs.msg import JointState

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=BAG, storage_id="sqlite3"),
        ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr"))
    reader.set_filter(StorageFilter(topics=["/joint_states"]))
    samples = []
    while reader.has_next():
        _topic, data, _stamp = reader.read_next()
        msg = deserialize_message(data, JointState)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp < 54.3:
            continue
        if stamp > 96.3:
            break
        index = {name: i for i, name in enumerate(msg.name)}
        samples.append((
            stamp, [float(msg.position[index[name]]) for name in JOINTS]))
    legs = []
    for name, start, end, load in LEGS:
        window = [item for item in samples if start <= item[0] < end]
        kept = []
        last = None
        for stamp, joints in window:
            if last is None or stamp - last >= 0.009:
                kept.append((stamp, joints))
                last = stamp
        legs.append({
            "name": name, "load": load, "t0": start, "t1": end,
            "samples": kept,
        })
    return legs


def _trajectory(samples):
    from st09_contracts.graph import TimedTrajectory

    positions = tuple(tuple(joints) for _stamp, joints in samples)
    zeros = tuple(tuple(0.0 for _ in JOINTS) for _ in samples)
    times = []
    t0 = samples[0][0]
    for index, (stamp, _joints) in enumerate(samples):
        value = int(round((stamp - t0) * 1e9))
        if times and value <= times[-1]:
            value = times[-1] + 1_000_000
        times.append(value)
    return TimedTrajectory(
        joint_names=JOINTS, positions=positions,
        velocities=zeros, accelerations=zeros,
        time_from_start_ns=tuple(times))


def _edge(name, trajectory, edge_class):
    from types import SimpleNamespace
    return SimpleNamespace(
        edge_id="executed-%s" % name,
        trajectory=trajectory,
        attributes={"edge_class": edge_class},
    )


def _payload():
    from luggage_planning.moveit_edge_adapter import PayloadSpec
    request = json.load(open(os.path.join(
        RUN, "dumps/box_00_paths/planning_request.json"), encoding="utf-8"))
    attachment = request["attachment_transform"]
    return PayloadSpec(
        dimensions_wdh=tuple(float(v) for v in request["box_wdh"]),
        t_tcp_box_translation=tuple(
            float(v) for v in attachment["translation"]),
        t_tcp_box_quaternion_xyzw=tuple(
            float(v) for v in attachment["quaternion_xyzw"])), request


def _certify_edges(node, edges, states, budget_s):
    from luggage_planning.moveit_state_validity import (
        FrozenSceneComposer, MoveItStateValidityAdapter,
    )
    from luggage_planning.planning_scene_client import PlanningSceneClient

    client = PlanningSceneClient(node)
    call_log = []
    adapter = MoveItStateValidityAdapter(
        node, fk=None, scene_client=client,
        service_timeout_s=budget_s + 20.0,
        budget_s=budget_s, call_log=call_log)
    adapter.load_edges(edges, scene_states=states)
    started = time.perf_counter()
    error = None
    try:
        adapter._ensure_verdicts()  # noqa: SLF001
    except Exception as exc:  # noqa: BLE001 - the service refusal is the result
        error = str(exc)
    wall = time.perf_counter() - started
    record = call_log[-1] if call_log else {"result": None, "detail": error}
    record["wall_s"] = round(wall, 3)
    record["budget_s"] = budget_s
    record["waypoint_counts"] = {
        edge.edge_id: len(edge.trajectory.positions) for edge in edges}
    return record


def _states_for(edges, payload):
    from luggage_planning.moveit_edge_adapter import SceneState
    states = {}
    for edge in edges:
        carrying = edge.edge_id in (
            "executed-transit", "executed-transit_2", "executed-traverse",
            "executed-insert", "executed-descend")
        states[edge.edge_id] = SceneState(
            load_state="carrying" if carrying else "unloaded",
            payload=payload if carrying else None)
    return states


def _leg_rows(record):
    rows = []
    spent = 0.0
    first_over = None
    for leg in record.get("legs") or []:
        seconds = float(leg.get("leg_time_ns") or 0) / 1e9
        spent += seconds
        if leg.get("reason") == "BUDGET_EXCEEDED" and first_over is None:
            first_over = {
                "edge": leg.get("edge"),
                "sample": leg.get("first_invalid"),
                "n_samples": leg.get("n_samples"),
                "cumulative_s": round(spent, 3),
            }
        rows.append({
            "edge": leg.get("edge"),
            "n_samples": leg.get("n_samples"),
            "service_s": round(seconds, 3),
            "cumulative_s": round(spent, 3),
            "valid": leg.get("valid"),
            "reason": leg.get("reason"),
            "first_invalid": leg.get("first_invalid"),
            "contacts": (leg.get("contacts") or [])[:3],
        })
    return {
        "result": record.get("result"),
        "detail": record.get("detail"),
        "all_valid": record.get("all_valid"),
        "wall_s": record.get("wall_s"),
        "budget_s": record.get("budget_s"),
        "service_s": round(float(record.get("service_time_ns") or 0) / 1e9, 3),
        "first_over_budget": first_over,
        "legs": rows,
        "waypoints": record.get("waypoint_counts"),
    }


def _sync(node, scene_json, surface, with_box, with_occupancy):
    from luggage_description.scene_tf_config_utils import load_scene_tf_config
    scene_config = load_scene_tf_config(SCENE_TF)
    copied = json.loads(json.dumps(scene_json))
    if not with_box:
        copied["world_objects"] = [
            obj for obj in copied.get("world_objects") or []
            if obj.get("id") != "placed_0_0_0"]
    world = classify._sync_world(node, scene_config, copied)  # noqa: SLF001
    if not with_box:
        from luggage_planning.planning_scene_client import PlanningSceneClient
        PlanningSceneClient(node).remove_object("placed_0_0_0")
    probes._clear_occupancy(node)  # noqa: SLF001
    cells = probes._add_occupancy(node, surface) if with_occupancy else 0  # noqa: SLF001
    return {"world": world, "occupancy_cells": cells,
            "placed_box": with_box}


def _plan_probe(node, probe, budget_s):
    from luggage_description.scene_tf_config_utils import load_scene_tf_config
    from luggage_planning.lattice_route import LatticeStateChecker
    from luggage_planning.planning_scene_client import PlanningSceneClient
    from luggage_planning.route_producer import register_attach_from_request
    from luggage_planning.moveit_state_validity import MoveItStateValidityAdapter

    document, surface, _scene, _slot = classify._request_document(  # noqa: SLF001
        probe["request_path"], probe["surface_path"],
        probe["scene_path"], probe["source_slot"])
    scene_config = load_scene_tf_config(SCENE_TF)
    stamp_ns = int(surface.get("acquisition_stamp_ns") or 1)
    producer, _urdf, _tcp = classify._producer(  # noqa: SLF001
        node, scene_config, SCENE_TF, None, stamp_ns, budget_s=90.0)
    call_log = []
    scene_client = PlanningSceneClient(node)

    def validity_factory(*, scene_states, edges, _log=call_log, _budget=budget_s):
        adapter = MoveItStateValidityAdapter(
            node, fk=producer.fk, scene_client=scene_client,
            service_timeout_s=_budget + 20.0, budget_s=_budget,
            call_log=_log)
        adapter.load_edges(edges, scene_states=scene_states)
        return adapter

    producer.corridor_planner = "lattice"
    producer.validity_adapter_factory = validity_factory
    producer.lattice_checker_factory = (
        lambda scene_states, start_q: LatticeStateChecker(
            node, scene_client, scene_states, start_q))
    synthetic = {"slot_pose": {
        "position": probe["center"],
        "quaternion_xyzw": probe["quaternion_xyzw"],
    }}
    request = classify._build_request(  # noqa: SLF001
        document, surface, synthetic, "gold-compare", stamp_ns)
    request["box_wdh"] = probe["box_wdh"]
    request["request_id"] = probe["name"]
    request["box_id"] = probe["name"]
    register_attach_from_request(
        producer.attachment_provider, request,
        box_wdh=request["box_wdh"], box_id=request["box_id"], generation=1)
    started = time.perf_counter()
    result = producer.plan_place_route(request)
    wall = time.perf_counter() - started
    reset = getattr(producer.edge_builder, "reset_scene", None)
    if callable(reset):
        reset()
    cert = call_log[-1] if call_log else {"result": None, "detail": result.get("reason_code")}
    cert["wall_s"] = round(wall, 3)
    cert["budget_s"] = budget_s
    cert["plan_reason"] = result.get("reason_code")
    return cert


def _verdict(rows):
    by_name = {row["name"]: row for row in rows}
    gold = [row for row in rows if row["name"].startswith("656_empty")]
    rev2 = [row for row in rows if row["name"].startswith("656_rev2")]
    probe_rows = [row for row in rows if row["name"] in (
        "standard_m10", "carryon_m10", "carryon_m15")]
    def timed_out(row):
        return row.get("result") == 3 or "budget exceeded" in str(row.get("detail"))
    def collided(row):
        return any(
            leg.get("reason") == "COLLISION" for leg in row.get("legs") or [])
    empty_2s = [row for row in gold if row.get("budget_s") == 2.0]
    if empty_2s and all(timed_out(row) for row in empty_2s):
        return "OFFLINE_CERT_NO_FIDELITY"
    if empty_2s and all(row.get("all_valid") for row in empty_2s):
        if probe_rows and all(timed_out(row) for row in probe_rows):
            diag = [row for row in rows if row.get("budget_s") == 30.0
                    and row["name"] in (
                        "standard_m10", "carryon_m10", "carryon_m15")]
            if diag and all(row.get("all_valid") for row in diag):
                return "PROBES_PASS_ONLY_WITH_MORE_TIME"
            if diag and any(collided(row) for row in diag):
                return "PROBE_INVALID_STATE"
            return "PROBE_CERT_COST"
    if rev2 and all(collided(row) or timed_out(row) for row in rev2):
        return "SEE_EMPTY_SCENE_CONTROL"
    return "UNCLASSIFIED"


def main():
    import rclpy
    from rclpy.node import Node
    from luggage_perception.eval.isolated_domain import isolated_replay_env

    os.makedirs(OUT, exist_ok=True)
    residuals = score._residuals()  # noqa: SLF001
    if residuals:
        _dump("manifest.json", {"outcome": "not_evaluated", "residuals": residuals})
        print(json.dumps({"outcome": "not_evaluated"}))
        return 2
    executed = _executed_legs()
    _dump("executed_counts.json", [
        {"name": leg["name"], "load": leg["load"], "samples": len(leg["samples"])}
        for leg in executed])
    payload, _request = _payload()
    edges = []
    for leg in executed:
        if len(leg["samples"]) < 2:
            raise SystemExit("leg %s has %d samples" % (
                leg["name"], len(leg["samples"])))
        edges.append(_edge(leg["name"], _trajectory(leg["samples"]), leg["name"]))
    states = _states_for(edges, payload)
    env = isolated_replay_env(score.DOMAIN)
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    proc = score._start_moveit(  # noqa: SLF001
        env, SCENE_TF, os.path.join(OUT, "moveit.log"))
    rclpy.init()
    node = Node("gold656_cert_compare")
    stop, thread = score._spin(node)  # noqa: SLF001
    reports = []
    try:
        from luggage_msgs.srv import ValidateRouteV2
        client = node.create_client(ValidateRouteV2, "/validate_route_v2")
        if not client.wait_for_service(timeout_sec=90.0):
            _dump("manifest.json", {"outcome": "validate_route_unavailable"})
            return 2
        scene_json = json.load(open(os.path.join(
            TRIAL, "scene.json"), encoding="utf-8"))
        # The failure scene is box 1; 656's own trial scene is empty.
        # Use the second-box scene, which contains placed_0_0_0.
        rev2_scene = json.load(open(os.path.join(
            RUN, "dumps/trial_01_PLANNING_BUDGET_EXHAUSTED/scene.json"),
            encoding="utf-8"))
        surface = json.load(open(os.path.join(
            RUN, "dumps/trial_01_PLANNING_BUDGET_EXHAUSTED/replay/surface_2d.json"),
            encoding="utf-8"))
        def run_656(label, budget):
            record = _certify_edges(node, edges, states, budget)
            parsed = _leg_rows(record)
            parsed["name"] = label
            parsed["scene"] = scene_info
            reports.append(parsed)
            _dump(label + ".json", parsed)
            return parsed

        if os.environ.get("GOLD_ONLY") == "empty":
            scene_info = _sync(node, scene_json, surface, False, False)
            run_656("656_empty_budget2", 2.0)
            verdict = _verdict(reports)
            _dump("verdict_empty.json", {"code": verdict, "reports": reports})
            print(json.dumps({"verdict": verdict, "scene": scene_info, "rows": [
                (row["name"], row.get("result"), row.get("all_valid"), row.get("detail"))
                for row in reports]}, indent=2))
            return 0
        scene_info = _sync(node, rev2_scene, surface, True, True)
        run_656("656_rev2_before", 2.0)
        for probe in probes._probes():  # noqa: SLF001
            cert = _plan_probe(node, probe, 2.0)
            parsed = _leg_rows(cert)
            parsed["name"] = probe["name"]
            parsed["scene"] = "revision2"
            reports.append(parsed)
            _dump(probe["name"] + "_budget2.json", parsed)
        run_656("656_rev2_after", 2.0)
        scene_info = _sync(node, scene_json, surface, False, False)
        empty = run_656("656_empty_budget2", 2.0)
        if empty.get("result") == 3:
            run_656("656_empty_budget30", 30.0)
        for probe in probes._probes():  # noqa: SLF001
            prior = next(row for row in reports if row["name"] == probe["name"])
            if prior.get("result") == 3:
                # Restore the revision-2 obstacles for the probe diagnostic.
                scene_info = _sync(node, rev2_scene, surface, True, True)
                cert = _plan_probe(node, probe, 30.0)
                parsed = _leg_rows(cert)
                parsed["name"] = probe["name"]
                parsed["diagnostic"] = True
                reports.append(parsed)
                _dump(probe["name"] + "_budget30.json", parsed)
        verdict = _verdict(reports)
        _dump("verdict.json", {"code": verdict, "reports": [
            {key: row[key] for key in (
                "name", "result", "detail", "all_valid", "budget_s",
                "wall_s", "first_over_budget")}
             for row in reports]})
        print(json.dumps({"verdict": verdict, "rows": [
            (row["name"], row.get("result"), row.get("detail"), row.get("wall_s"))
            for row in reports]}, indent=2))
        return 0
    finally:
        stop.set()
        thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        score._stop_moveit(proc)  # noqa: SLF001
        _dump("manifest.json", {
            "domain": 42, "gazebo": False,
            "moveit_stopped": proc.poll() is not None,
            "residuals_after": score._residuals(),  # noqa: SLF001
            "online_planner_modified": False,
        })


if __name__ == "__main__":
    sys.exit(main() or 0)
