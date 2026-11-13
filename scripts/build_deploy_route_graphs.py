#!/usr/bin/env python3
"""Build the four deploy directed route graphs (ST09-PD).

Runs against the same no-Gazebo stack as the deploy atlas campaign
(robot_state_publisher + move_group + scene_manager). Nodes are atlas
seeds fetched through the public atlas APIs; each healthy fixture
target contributes the canonical B chain

    portal -> slot_approach -> slot_upper -> slot (carrying)
    slot(twin) -> exit(twin) -> portal(twin) (unloaded retreat/return)

and every edge is an offline joint-interpolation segment whose samples
are re-verified by /check_state_validity and retimed with the site
rest-to-rest trapezoid. The graph is a prior consumed by B's graph
route planner, not a second planning method: store-loaded edges enter
production as candidates that must be re-verified in session.

Usage (stack must be up):
  python3 scripts/build_deploy_route_graphs.py \
      --data-dir src/luggage_planning/data/reachability_atlas_deploy \
      --scene-tf src/luggage_description/config/scene_tf_deploy.yaml
"""

from __future__ import division

import argparse
import hashlib
import json
import math
import os
import sys
import time

WS = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
for _root in (
        WS,
        os.path.join(WS, "src", "luggage_planning"),
        os.path.join(WS, "src", "luggage_description")):
    if _root not in sys.path:
        sys.path.insert(0, _root)

JOINT_NAMES = ("elfin_joint1", "elfin_joint2", "elfin_joint3",
               "elfin_joint4", "elfin_joint5", "elfin_joint6")

PROFILES = (
    ("s20_deploy_empty", "s20_container_collision_aware", None),
    ("s20_deploy_payload_0.55x0.40x0.25",
     "s20_container_collision_aware_payload_0.55x0.40x0.25",
     (0.55, 0.40, 0.25)),
    ("s20_deploy_payload_0.70x0.45x0.28",
     "s20_container_collision_aware_payload_0.70x0.45x0.28",
     (0.70, 0.45, 0.28)),
    ("s20_deploy_payload_0.80x0.50x0.32",
     "s20_container_collision_aware_payload_0.80x0.50x0.32",
     (0.80, 0.50, 0.32)),
)

INTERP_STEP_RAD = 0.10
VEL_SCALE = 0.3
ACC_SCALE = 0.3
TARGETS_PER_YAW = 2

PLANNER_CONFIG = {
    "edge_builder": "offline-joint-interp",
    "interp_step_rad": INTERP_STEP_RAD,
    "validity": "check_state_validity",
    "released_box_modeled": False,
    "timing": "rest_to_rest_trapezoid",
    "vel_scale": VEL_SCALE,
    "acc_scale": ACC_SCALE,
    "targets_per_yaw": TARGETS_PER_YAW,
}


def digest8(*values):
    text = "|".join(str(v) for v in values)
    return hashlib.sha256(text.encode("utf-8")).hexdigest()[:8]


def interpolation_rows(q_from, q_to, step):
    """Joint interpolation rows inclusive of both endpoints."""
    delta = [float(b) - float(a) for a, b in zip(q_from, q_to)]
    travel = max(abs(v) for v in delta)
    count = max(1, int(math.ceil(travel / float(step))))
    rows = []
    for index in range(count + 1):
        t = float(index) / float(count)
        rows.append(tuple(float(a) + d * t for a, d in zip(q_from, delta)))
    return rows, travel


class ValidityClient(object):
    """/check_state_validity client over one rclpy node."""

    def __init__(self, node):
        import rclpy
        from moveit_msgs.srv import GetStateValidity
        self._rclpy = rclpy
        self._node = node
        self._client = node.create_client(GetStateValidity, "/check_state_validity")
        if not self._client.wait_for_service(timeout_sec=60.0):
            raise RuntimeError("/check_state_validity unavailable")
        self._get_state_validity = GetStateValidity

    def check(self, q):
        from moveit_msgs.msg import RobotState
        from sensor_msgs.msg import JointState
        request = self._get_state_validity.Request()
        request.group_name = "elfin_arm"
        state = RobotState()
        state.joint_state = JointState()
        state.joint_state.name = list(JOINT_NAMES)
        state.joint_state.position = [float(v) for v in q]
        state.is_diff = True
        request.robot_state = state
        future = self._client.call_async(request)
        self._rclpy.spin_until_future_complete(
            self._node, future, timeout_sec=30.0)
        result = future.result()
        if result is None:
            raise RuntimeError(
                "check_state_validity failed: %s" % (future.exception(),))
        return bool(result.valid)

    def segment_valid(self, rows):
        """True when every interpolated row is valid."""
        for row in rows:
            if not self.check(row):
                return False
        return True


class PayloadScene(object):
    """Attach/detach the profile payload around carrying edge checks."""

    def __init__(self, node, payload):
        import rclpy
        from moveit_msgs.srv import ApplyPlanningScene
        self._rclpy = rclpy
        self._node = node
        self._payload = payload
        self._attached = False
        self._client = node.create_client(
            ApplyPlanningScene, "/apply_planning_scene")
        if not self._client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError("/apply_planning_scene unavailable")

    def _apply(self, scene, tolerate_failure=False):
        from moveit_msgs.srv import ApplyPlanningScene
        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self._client.call_async(request)
        self._rclpy.spin_until_future_complete(
            self._node, future, timeout_sec=15.0)
        result = future.result()
        if result is None or not result.success:
            if tolerate_failure:
                # Detaching an object that is not attached reports False;
                # the next attach re-establishes a known state either way.
                print("apply_planning_scene tolerated failure (detach)",
                      flush=True)
                time.sleep(0.5)
                return
            raise RuntimeError("apply_planning_scene failed: %s" % (
                future.exception() if result is None else result.success,))
        time.sleep(0.5)

    def attach(self):
        if self._payload is None:
            return
        from luggage_planning.motion.planning_scene_client import build_attach_scene
        length, width, height = self._payload
        self._apply(build_attach_scene(
            "deploy_graph_payload", [0.0, 0.0, height * 0.5],
            (0.0, 0.0, 0.0, 1.0), [length, width, height],
            "suction_contact_frame", touch_links=["suction_panel"],
            frame_id="suction_contact_frame"))
        self._attached = True

    def detach(self):
        if not self._attached:
            return
        from luggage_planning.motion.planning_scene_client import build_detach_scene
        self._attached = False
        self._apply(build_detach_scene(
            "deploy_graph_payload", "suction_contact_frame"),
                    tolerate_failure=True)


def load_fk(urdf_path):
    from luggage_planning.motion.offline_ik import Urdf
    from luggage_planning.atlas.graph import make_offline_fk
    with open(urdf_path, "rb") as handle:
        urdf = Urdf(handle.read())
    full_path = urdf.path_to("suction_contact_frame")
    arm = JOINT_NAMES[0]
    if arm not in full_path:
        raise RuntimeError("URDF path lacks %s: %s" % (arm, full_path))
    tcp_path = full_path[full_path.index(arm):]
    return make_offline_fk(urdf, tcp_path)


def select_targets(atlas, payload, targets_per_yaw):
    """Deterministic healthy fixture targets: per directed yaw bin, the
    first ``targets_per_yaw`` contact-layer cells that are REACHABLE,
    opening-connected, and carry contact seeds (public query API only)."""
    meta = atlas.meta
    grid = meta.get("grid") or {}
    container = meta.get("container") or {}
    resolution = float(atlas.resolution)
    origin = [float(v) for v in grid.get("origin", (0.0, 0.0, 0.0))]
    # Meta stores (nx, ny, nz); the yaw axis length is the bin count.
    size = [int(v) for v in (grid.get("size") or ())]
    if len(size) == 3:
        size = size + [len(atlas.yaw_bins)]
    elif not size:
        size = list(atlas.status.shape)
    floor_z = float(container.get("floor_z", 0.0))
    # Contact-layer rule (QuerySemantics.floor_clamp, offline form): the
    # suction contact for a carried box of height h rests at floor + h,
    # but the cell CONTAINING that height has its centre below floor + h,
    # so its sampled verdict reads the box through the floor. Targets are
    # therefore taken from the lowest layer whose cell CENTRE is at or
    # above floor + h (empty-load: floor itself).
    height = 0.0 if payload is None else float(payload[2])
    center_min = floor_z + height
    iz = int(math.ceil((center_min - origin[2]) / resolution - 0.5))
    iz = max(0, iz)
    nx, ny, nz, nyaw = (int(v) for v in size)
    targets = []
    if not (0 <= iz < nz - 2):
        return targets, iz
    for iyaw in range(nyaw):
        yaw = float(atlas.yaw_bins[iyaw])
        found = 0
        for iy in range(ny):
            for ix in range(nx):
                if found >= targets_per_yaw:
                    break
                center = (
                    origin[0] + (ix + 0.5) * resolution,
                    origin[1] + (iy + 0.5) * resolution,
                    origin[2] + (iz + 0.5) * resolution,
                )
                result = atlas.query(center[0], center[1], center[2], yaw)
                if int(result.status) != 3 or not result.opening_connected:
                    continue
                if not (result.contact_seeds or result.transit_seeds):
                    continue
                targets.append({
                    "yaw_index": iyaw, "yaw": yaw,
                    "cell": [ix, iy, iz], "center": list(center)})
                found += 1
    return targets, iz


def seeds_at(atlas, x, y, z, yaw, seed_set, max_seeds=2):
    from luggage_planning.atlas.runtime import atlas_seeds_for_query
    record = atlas_seeds_for_query(
        atlas, x, y, z, yaw, seed_set=seed_set, max_seeds=max_seeds)
    return [tuple(float(v) for v in row)
            for row in record.get("seeds", ())]


def build_chain_nodes(graph, atlas, target, resolution, opening_x, fk):
    """Register the seven chain nodes for one target.

    Returns ``(nodes, missing_roles)``; ``nodes`` is None when any role
    lacks an atlas seed (recorded as a prior gap by the caller).
    """
    contracts_graph, _ = _contracts()
    carrying = contracts_graph.LoadState.CARRYING
    unloaded = contracts_graph.LoadState.UNLOADED
    cx, cy, cz = target["center"]
    roles = (
        ("portal", (opening_x, cy, cz + resolution), "transit"),
        ("slot_approach", (cx, cy, cz + 2.0 * resolution), "transit"),
        ("slot_upper", (cx, cy, cz + resolution), "transit"),
        ("slot", (cx, cy, cz), "contact"),
    )
    nodes = {}
    missing = []
    for region, (x, y, z), seed_set in roles:
        seeds = seeds_at(atlas, x, y, z, target["yaw"], seed_set)
        if not seeds:
            missing.append("%s(%s)" % (region, seed_set))
            continue
        q = seeds[0]
        position, quaternion = fk(q)
        nodes[region] = graph.register_node(
            q, position, quaternion, region, carrying,
            ik_family="atlas-%s-0" % seed_set)
    if missing:
        return None, missing
    # Unloaded twins: release at the slot, retreat exit at slot_upper,
    # return destination at the portal.
    for region, twin_of in (("slot", "slot"), ("exit", "slot_upper"),
                            ("return_portal", "portal")):
        source = nodes[twin_of]
        nodes[region + "_unloaded"] = graph.register_node(
            source.q, source.tcp_position, source.tcp_quaternion_xyzw,
            "exit" if region == "exit" else
            ("portal" if region == "return_portal" else "slot"),
            unloaded, ik_family=source.ik_family)
    return nodes, []


def _contracts():
    from luggage_planning.planning_contract import graph as contracts_graph
    from luggage_planning.planning_contract._base import digest_of
    return contracts_graph, digest_of


def make_edge(graph, edge_class, source, destination, load_state,
              validity, static_identity):
    from luggage_planning.motion.trajectory_timing import (
        time_parameterize_cartesian)
    contracts_graph, digest_of = _contracts()
    rows, travel = interpolation_rows(source.q, destination.q, INTERP_STEP_RAD)
    if not validity.segment_valid(rows):
        return None, travel
    timed = time_parameterize_cartesian(
        [list(row) for row in rows], VEL_SCALE, ACC_SCALE)
    duration_ns = int(round(timed.duration * 1e9))
    trajectory = contracts_graph.TimedTrajectory(
        joint_names=JOINT_NAMES,
        positions=tuple(tuple(row) for row in rows),
        velocities=tuple(tuple(v) for v in timed.velocities_rad),
        accelerations=tuple(tuple(a) for a in timed.accelerations_rad),
        time_from_start_ns=tuple(
            int(round(t * 1e9)) for t in timed.times_s),
    )
    edge = contracts_graph.GraphEdge(
        edge_id="e-%s-%s-%s" % (
            edge_class.lower(), digest8(source.node_id),
            digest8(destination.node_id)),
        source_node_id=source.node_id,
        destination_node_id=destination.node_id,
        load_state=load_state,
        static_identity=static_identity,
        trajectory=trajectory,
        validation_revision=None,
        attributes={
            "edge_class": edge_class,
            "edge_kind": "joint_interpolation",
            "planner": "offline-joint-interp",
            "joint_travel_rad": travel,
            "duration_ns": duration_ns,
            "cost_ns": duration_ns,
            "samples_checked": len(rows),
            "validity": "check_state_validity",
            "released_box_modeled": False,
        },
    )
    graph.add_candidate_edge(edge)
    return edge, travel


def planner_config(targets_per_yaw):
    """Frozen offline edge-builder configuration (hashed into the graph
    static identity)."""
    return dict(PLANNER_CONFIG, targets_per_yaw=int(targets_per_yaw))


def build_one_graph(node, data_dir, graph_id, basename, payload, scene_path,
                    targets_per_yaw):
    from luggage_planning.atlas.frame_contracts import geometry_hash
    from luggage_planning.atlas.graph import GraphStore, RouteGraph
    from luggage_planning.atlas.model import ReachabilityAtlas

    npz = os.path.join(data_dir, basename + ".npz")
    meta_path = os.path.join(data_dir, basename + ".yaml")
    atlas = ReachabilityAtlas.load(npz, meta_path)
    meta = atlas.meta
    deps = meta.get("dependencies") or {}
    container = meta.get("container") or {}
    grid = meta.get("grid") or {}
    resolution = float(atlas.resolution)

    urdf_path = os.path.join(data_dir, "robot_description.xml")
    with open(urdf_path, "rb") as handle:
        urdf_bytes = handle.read()
    urdf_md5 = hashlib.md5(urdf_bytes).hexdigest()
    if str(deps.get("urdf_hash") or "") != urdf_md5:
        raise RuntimeError(
            "%s: captured URDF md5 %s != atlas urdf_hash %s"
            % (basename, urdf_md5, deps.get("urdf_hash")))
    scene_md5 = hashlib.md5(open(scene_path, "rb").read()).hexdigest()
    if str(deps.get("scene_tf_hash") or "") != scene_md5:
        raise RuntimeError(
            "%s: scene md5 %s != atlas scene_tf_hash %s"
            % (basename, scene_md5, deps.get("scene_tf_hash")))

    geom_hash = geometry_hash(
        container.get("inner_dimensions") or (0.0, 0.0, 0.0),
        container.get("floor_z", 0.0),
        container.get("ceiling_z", 0.0),
        container.get("opening_side", ""))
    config = planner_config(targets_per_yaw)
    static_identity = GraphStore.static_identity(
        urdf_md5, "seven_face_hull", geom_hash,
        dict(config, payload=list(payload) if payload else None))

    graph = RouteGraph(static_identity=static_identity)
    fk = load_fk(urdf_path)
    validity = ValidityClient(node)
    scene_payload = PayloadScene(node, payload)

    opening_side = str(container.get("opening_side") or "negative_x")
    inner = [float(v) for v in (container.get("inner_dimensions") or (0, 0, 0))]
    # Opening-side innermost cell centre x (aperture column).
    sign = -1.0 if "negative" in opening_side else 1.0
    opening_x = sign * (0.5 * inner[0] - 0.5 * resolution)

    targets, iz_target = select_targets(atlas, payload, targets_per_yaw)
    summary = {
        "graph_id": graph_id,
        "basename": basename,
        "payload": list(payload) if payload else None,
        "static_identity": static_identity,
        "planner_config": config,
        "urdf_md5": urdf_md5,
        "scene_tf_md5": scene_md5,
        "geometry_hash": geom_hash,
        "target_iz": iz_target,
        "targets_selected": len(targets),
        "targets_chained": 0,
        "targets_missing_seeds": [],
        "targets_invalid_edges": [],
        "prior_gaps": [],
        "edges": 0,
        "nodes": 0,
    }

    for target in targets:
        nodes, missing = build_chain_nodes(
            graph, atlas, target, resolution, opening_x, fk)
        if nodes is None:
            gap = "ATLAS_CHAIN_UNCOVERED target=%s yaw=%.4f missing=%s" % (
                target["cell"], target["yaw"], ",".join(missing))
            graph.note_prior_gap(gap)
            summary["prior_gaps"].append(gap)
            summary["targets_missing_seeds"].append(
                {"cell": target["cell"], "yaw": target["yaw"],
                 "missing": missing})
            continue
        contracts_graph, _ = _contracts()
        carrying = contracts_graph.LoadState.CARRYING
        unloaded = contracts_graph.LoadState.UNLOADED

        # Chains sharing an exact node pair share the edge: same q
        # endpoints means the same deterministic interpolation, so the
        # edge is deduplicated on (class, source, destination).
        existing = {
            (str(edge.attributes.get("edge_class")),
             edge.source_node_id, edge.destination_node_id)
            for edge in list(graph.candidate_edges.values())}

        def build_edge(edge_class, source, destination, load):
            key = (edge_class, source.node_id, destination.node_id)
            if key in existing:
                return "shared"
            existing.add(key)
            edge, _travel = make_edge(
                graph, edge_class, source, destination, load,
                validity, static_identity)
            return edge

        scene_payload.attach()
        try:
            chain = (
                ("TRAVERSE", nodes["portal"], nodes["slot_approach"], carrying),
                ("INSERT", nodes["slot_approach"], nodes["slot_upper"], carrying),
                ("DESCEND", nodes["slot_upper"], nodes["slot"], carrying),
            )
            failed = None
            for edge_class, source, destination, load in chain:
                if build_edge(edge_class, source, destination, load) is None:
                    failed = edge_class
                    break
            if failed is None and build_edge(
                    "RETREAT", nodes["slot_unloaded"], nodes["exit_unloaded"],
                    unloaded) is None:
                failed = "RETREAT"
            if failed is None and build_edge(
                    "RETURN", nodes["exit_unloaded"],
                    nodes["return_portal_unloaded"], unloaded) is None:
                failed = "RETURN"
        finally:
            scene_payload.detach()
        if failed is not None:
            summary["targets_invalid_edges"].append(
                {"cell": target["cell"], "yaw": target["yaw"],
                 "edge_class": failed})
            gap = ("OFFLINE_EDGE_INVALID target=%s yaw=%.4f edge=%s"
                   % (target["cell"], target["yaw"], failed))
            graph.note_prior_gap(gap)
            summary["prior_gaps"].append(gap)
            continue
        summary["targets_chained"] += 1

    summary["nodes"] = len(graph.nodes)
    summary["edges"] = len(graph.candidate_edges)
    store = GraphStore(os.path.join(data_dir, "graphs"))
    path, digest = store.save(graph, graph_id)
    summary["path"] = path
    summary["digest"] = digest
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-dir", required=True)
    parser.add_argument("--scene-tf", required=True)
    parser.add_argument("--targets-per-yaw", type=int,
                        default=TARGETS_PER_YAW)
    parser.add_argument("--output", default="",
                        help="optional JSON summary path")
    args = parser.parse_args()

    import rclpy
    rclpy.init()
    node = rclpy.create_node("build_deploy_route_graphs")
    try:
        summaries = []
        for graph_id, basename, payload in PROFILES:
            print("==> graph %s (%s)" % (graph_id, basename), flush=True)
            summary = build_one_graph(
                node, args.data_dir, graph_id, basename, payload,
                args.scene_tf, args.targets_per_yaw)
            summaries.append(summary)
            print("    nodes=%d edges=%d chained=%d gaps=%d" % (
                summary["nodes"], summary["edges"],
                summary["targets_chained"], len(summary["prior_gaps"])),
                flush=True)
        payload_json = {
            "schema": "st09.deploy-graph-build/1",
            "data_dir": os.path.abspath(args.data_dir),
            "scene_tf": os.path.abspath(args.scene_tf),
            "planner_config": planner_config(args.targets_per_yaw),
            "graphs": summaries,
        }
        text = json.dumps(payload_json, indent=1, sort_keys=True)
        if args.output:
            with open(args.output, "w", encoding="utf-8") as handle:
                handle.write(text)
        print(text, flush=True)
        failed = [s for s in summaries if s["targets_chained"] == 0]
        if failed:
            print("GRAPHS EMPTY: %s" % [s["graph_id"] for s in failed],
                  flush=True)
            return 1
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
