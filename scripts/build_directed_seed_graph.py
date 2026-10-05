#!/usr/bin/env python3
"""Write a directed seed graph beside each deploy reachability atlas.

Nodes are the opening-connected joint seeds already stored in the atlas.
Edges are 6-neighbor steps within the joint-step limit, labeled inward
(deeper, away from the door), outward, lateral, or vertical. No Gazebo,
MoveIt, or new sampling.
"""

from __future__ import division

import argparse
import os
import sys

import numpy as np
import yaml

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
for pkg in ("luggage_planning", "luggage_description"):
    path = os.path.join(ROOT, "src", pkg)
    if path not in sys.path:
        sys.path.insert(0, path)

from luggage_planning.atlas_runtime import compute_scene_tf_hash  # noqa: E402
from luggage_planning.reachability_atlas import ReachabilityAtlas  # noqa: E402
from luggage_planning.reachable_graph import build_reachable_graph  # noqa: E402

KIND_INWARD = 0
KIND_OUTWARD = 1
KIND_LATERAL = 2
KIND_VERTICAL = 3
KIND_NAME = {
    KIND_INWARD: "inward",
    KIND_OUTWARD: "outward",
    KIND_LATERAL: "lateral",
    KIND_VERTICAL: "vertical",
}
# Axis and sign of an inward step. negative_x door: deeper is +X.
_INWARD_AXIS = {
    "negative_x": (0, 1.0),
    "positive_x": (0, -1.0),
    "negative_y": (1, 1.0),
    "positive_y": (1, -1.0),
    "negative_z": (2, 1.0),
    "positive_z": (2, -1.0),
}
DEFAULT_ATLAS_DIR = os.path.join(
    ROOT, "src", "luggage_planning", "data", "reachability_atlas_deploy")
DEFAULT_SCENE = os.path.join(
    ROOT, "src", "luggage_description", "config", "scene_tf_deploy.yaml")
MAX_JOINT_STEP_RAD = 1.2


def edge_kind(opening_side, src_center, dst_center):
    """Label one axis-aligned step. Inward moves away from the door."""
    delta = [float(b) - float(a) for a, b in zip(src_center, dst_center)]
    axis, sign = _INWARD_AXIS[str(opening_side or "negative_x")]
    depth = delta[axis]
    others = [abs(delta[i]) for i in range(3) if i != axis]
    if abs(depth) > 1e-9 and abs(depth) >= max(others or [0.0]):
        return KIND_INWARD if depth * sign > 0.0 else KIND_OUTWARD
    lateral_axis = 1 if axis == 0 else 0
    if abs(delta[lateral_axis]) >= abs(delta[2 if lateral_axis != 2 else 1]):
        return KIND_LATERAL
    return KIND_VERTICAL


def _merged_graph(atlas, max_joint_step_rad):
    nodes = {}
    adjacency = {}
    opening_side = "negative_x"
    for yaw in atlas.yaw_bins:
        graph = build_reachable_graph(
            atlas, yaw=float(yaw), max_joint_step_rad=max_joint_step_rad)
        if graph.disabled_reason:
            raise RuntimeError(
                "directed seed graph disabled: %s" % graph.disabled_reason)
        opening_side = getattr(graph, "opening_side", opening_side)
        nodes.update(graph.nodes)
        for index, edges in graph.adjacency.items():
            adjacency.setdefault(index, []).extend(edges)
    return nodes, adjacency, opening_side


def build_directed_records(atlas, max_joint_step_rad=MAX_JOINT_STEP_RAD):
    """Return arrays for one atlas. Raises if the graph cannot be built."""
    nodes, adjacency, opening_side = _merged_graph(atlas, max_joint_step_rad)
    order = sorted(nodes)
    row_of = {index: row for row, index in enumerate(order)}
    if order:
        node_index = np.asarray(order, dtype=np.int32)
        node_center = np.asarray(
            [nodes[index].center for index in order], dtype=np.float64)
        node_seed = np.asarray(
            [nodes[index].seed[:6] for index in order], dtype=np.float64)
        node_yaw = np.asarray([
            float(atlas.yaw_bins[index[3]]) for index in order
        ], dtype=np.float64)
    else:
        node_index = np.zeros((0, 4), dtype=np.int32)
        node_center = np.zeros((0, 3), dtype=np.float64)
        node_seed = np.zeros((0, 6), dtype=np.float64)
        node_yaw = np.zeros((0,), dtype=np.float64)
    src_rows = []
    dst_rows = []
    kinds = []
    lengths = []
    for index in order:
        src = nodes[index]
        for other, length in adjacency.get(index, []):
            dst = nodes.get(other)
            if dst is None:
                continue
            src_rows.append(row_of[index])
            dst_rows.append(row_of[other])
            kinds.append(edge_kind(opening_side, src.center, dst.center))
            lengths.append(float(length))
    order_edges = sorted(range(len(src_rows)), key=lambda i: (
        src_rows[i], dst_rows[i], kinds[i]))
    def take(values, dtype):
        if not order_edges:
            return np.zeros((0,), dtype=dtype)
        return np.asarray([values[i] for i in order_edges], dtype=dtype)
    return {
        "opening_side": opening_side,
        "node_index": node_index,
        "node_center": node_center,
        "node_seed": node_seed,
        "node_yaw": node_yaw,
        "edge_src": take(src_rows, np.int32),
        "edge_dst": take(dst_rows, np.int32),
        "edge_kind": take(kinds, np.uint8),
        "edge_length": take(lengths, np.float64),
    }


def output_stem(atlas):
    payload = atlas.meta.get("payload") or {}
    size = payload.get("size") or []
    if payload.get("enabled") and len(size) == 3:
        return "directed_seed_graph_payload_%.2fx%.2fx%.2f" % tuple(
            float(v) for v in size)
    return "directed_seed_graph"


def scene_identity(atlas, scene_path):
    """Return (scene_hash, urdf_hash). Fail if the scene file disagrees."""
    stored = str((atlas.meta.get("dependencies") or {}).get("scene_tf_hash") or "")
    urdf = str((atlas.meta.get("dependencies") or {}).get("urdf_hash") or "")
    current = compute_scene_tf_hash(scene_path) if scene_path else ""
    if not stored or not current or stored != current:
        raise RuntimeError(
            "scene hash mismatch: atlas=%s file=%s" % (
                stored[:12] or "missing", current[:12] or "missing"))
    return stored, urdf


def write_directed_seed_graph(atlas, out_dir, source_npz, scene_path,
                              max_joint_step_rad=MAX_JOINT_STEP_RAD):
    scene_hash, urdf_hash = scene_identity(atlas, scene_path)
    records = build_directed_records(atlas, max_joint_step_rad)
    stem = output_stem(atlas)
    npz_path = os.path.join(out_dir, stem + ".npz")
    yaml_path = os.path.join(out_dir, stem + ".yaml")
    np.savez_compressed(
        npz_path,
        node_index=records["node_index"],
        node_center=records["node_center"],
        node_seed=records["node_seed"],
        node_yaw=records["node_yaw"],
        edge_src=records["edge_src"],
        edge_dst=records["edge_dst"],
        edge_kind=records["edge_kind"],
        edge_length=records["edge_length"],
    )
    kind_counts = {
        name: int(np.count_nonzero(records["edge_kind"] == code))
        for code, name in KIND_NAME.items()
    }
    meta = {
        "scene_tf": os.path.basename(scene_path),
        "scene_tf_hash": scene_hash,
        "urdf_hash": urdf_hash,
        "opening_side": records["opening_side"],
        "source_npz": os.path.basename(source_npz),
        "max_joint_step_rad": float(max_joint_step_rad),
        "node_count": int(records["node_index"].shape[0]),
        "edge_count": int(records["edge_src"].shape[0]),
        "edge_counts": kind_counts,
        "edge_kind_codes": {name: code for code, name in KIND_NAME.items()},
        "seed_set": "transit_then_contact",
    }
    with open(yaml_path, "w", encoding="utf-8") as handle:
        yaml.safe_dump(meta, handle, default_flow_style=False, sort_keys=False)
    return npz_path, yaml_path, meta


def atlas_paths(atlas_dir):
    names = sorted(
        name for name in os.listdir(atlas_dir)
        if name.startswith("s20_container_collision_aware")
        and name.endswith(".npz"))
    return [os.path.join(atlas_dir, name) for name in names]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--atlas-dir", default=DEFAULT_ATLAS_DIR)
    parser.add_argument("--scene-tf", default=DEFAULT_SCENE)
    parser.add_argument("--max-joint-step-rad", type=float, default=MAX_JOINT_STEP_RAD)
    args = parser.parse_args(argv)
    written = []
    for npz_path in atlas_paths(args.atlas_dir):
        meta_path = npz_path[:-4] + ".yaml"
        atlas = ReachabilityAtlas.load(npz_path, meta_path)
        out_npz, out_yaml, meta = write_directed_seed_graph(
            atlas, args.atlas_dir, npz_path, args.scene_tf,
            max_joint_step_rad=args.max_joint_step_rad)
        written.append((out_npz, meta))
        print(
            "%s  nodes=%d edges=%d inward=%d outward=%d" % (
                os.path.basename(out_yaml),
                meta["node_count"],
                meta["edge_count"],
                meta["edge_counts"]["inward"],
                meta["edge_counts"]["outward"]),
            flush=True)
    return written


if __name__ == "__main__":
    main()
