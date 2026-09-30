#!/usr/bin/env python3
"""6-connected graph over a reachability atlas, masked by cargo occupancy.

The atlas stays a path prior: this module does not invent placement slots.
Session occupancy comes only from the caller's snapshot or ``blocked_fn``.
"""

from __future__ import division

import math

from luggage_planning.atlas_runtime import identity_verdict
from luggage_planning.joint_path import max_joint_step
from luggage_planning.reachability_atlas import MARGINAL

try:
    from luggage_planning.occupancy_place_paths import sweep_polyline
except ImportError:  # pragma: no cover - import cycle guard
    sweep_polyline = None


_NEIGHBORS = ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))


class GraphNode(object):
    def __init__(self, index, center, seed, clearance):
        self.index = tuple(index)
        self.center = tuple(float(v) for v in center)
        self.seed = [float(v) for v in seed]
        self.clearance = float(clearance)


class ReachableGraph(object):
    def __init__(self, nodes, adjacency, reject_counts, disabled_reason=""):
        self.nodes = nodes
        self.adjacency = adjacency
        self.reject_counts = dict(reject_counts)
        self.disabled_reason = str(disabled_reason or "")

    def __len__(self):
        return len(self.nodes)


def grid_disable_reason(atlas, scene_tf_hash=None, urdf_hash=None, policy="off"):
    """Return a ``GRID_DISABLED_*`` code, or ``''`` when the grid may be used.

    ``strict`` fails closed when either hash is absent. ``warn`` disables
    only on a supplied hash that differs from the atlas. ``off`` does not
    check.
    """
    if atlas is None:
        return "GRID_DISABLED_NO_ATLAS"
    mode = str(policy or "off")
    if mode == "off":
        return ""
    deps = (getattr(atlas, "meta", None) or {}).get("dependencies") or {}
    ok, _message, scene_ok, urdf_ok = identity_verdict(
        deps, scene_tf_hash=scene_tf_hash, urdf_hash=urdf_hash, policy=mode)
    if ok:
        return ""
    if not scene_ok:
        return "GRID_DISABLED_SCENE_TF"
    if not urdf_ok:
        return "GRID_DISABLED_URDF"
    return "GRID_DISABLED_URDF"


def cell_center(atlas, ix, iy, iz):
    origin = [float(v) for v in atlas.meta["grid"]["origin"]]
    resolution = float(atlas.resolution)
    return (
        origin[0] + (int(ix) + 0.5) * resolution,
        origin[1] + (int(iy) + 0.5) * resolution,
        origin[2] + (int(iz) + 0.5) * resolution,
    )


def _opening_side(atlas):
    container = (atlas.meta or {}).get("container") or {}
    return str(container.get("opening_side") or "negative_x")


def _bump(counts, key):
    counts[key] = int(counts.get(key, 0)) + 1


def _first_seed(result):
    for group in (result.transit_seeds, result.contact_seeds):
        for seed in group or []:
            if seed and len(seed) >= 6 and all(math.isfinite(float(v)) for v in seed[:6]):
                return [float(v) for v in seed[:6]]
    return None


def build_reachable_graph(atlas, yaw=0.0, max_joint_step_rad=1.2,
                          snapshot=None, payload_wdh=None, arm_radius=0.0,
                          blocked_fn=None, clearance_default=1.0):
    """Index usable cells and 6-neighbor edges that pass the joint-step limit.

    ``blocked_fn(index, center)`` returns a reason string when the cell is
    occupied, or ``''`` when it is free. A snapshot sweep is the default
    occupancy test when ``blocked_fn`` is omitted.
    """
    disabled = grid_disable_reason(atlas)
    if disabled:
        return ReachableGraph({}, {}, {}, disabled)
    contract = (getattr(atlas, "meta", None) or {}).get("contract") or {}
    period = contract.get("yaw_period_rad")
    if period is not None and abs(float(period) - math.pi) <= 1e-6:
        from luggage_description.frame_contracts import ContractError
        raise ContractError(
            "ATLAS_SEMANTICS_MISMATCH", "yaw_period_rad",
            2.0 * math.pi, period)
    nx, ny, nz, _nyaw = atlas.grid_size
    counts = {}
    nodes = {}
    for ix in range(nx):
        for iy in range(ny):
            for iz in range(nz):
                center = cell_center(atlas, ix, iy, iz)
                result = atlas.query(center[0], center[1], center[2], yaw=yaw)
                if int(result.status) < MARGINAL:
                    _bump(counts, "status")
                    continue
                if not result.opening_connected:
                    _bump(counts, "opening")
                    continue
                seed = _first_seed(result)
                if seed is None:
                    _bump(counts, "no_seed")
                    continue
                index = tuple(result.indices)
                reason = ""
                clearance = float(clearance_default)
                if blocked_fn is not None:
                    reason = str(blocked_fn(index, center) or "")
                elif snapshot is not None and payload_wdh is not None and sweep_polyline is not None:
                    swept = sweep_polyline(
                        snapshot, [center], payload_wdh, yaw=yaw,
                        arm_radius=arm_radius)
                    if swept.collides:
                        reason = swept.reason or "occupancy"
                    else:
                        clearance = float(swept.min_clearance)
                if reason:
                    _bump(counts, reason)
                    continue
                nodes[index] = GraphNode(index, center, seed, clearance)
    adjacency = {index: [] for index in nodes}
    for index, node in nodes.items():
        ix, iy, iz, iyaw = index
        for dx, dy, dz in _NEIGHBORS:
            other = (ix + dx, iy + dy, iz + dz, iyaw)
            neighbor = nodes.get(other)
            if neighbor is None:
                continue
            if max_joint_step(node.seed, neighbor.seed) > float(max_joint_step_rad) + 1e-9:
                _bump(counts, "joint_step")
                continue
            length = math.sqrt(sum(
                (a - b) ** 2 for a, b in zip(node.center, neighbor.center)))
            adjacency[index].append((other, max(length, 1e-6)))
    graph = ReachableGraph(nodes, adjacency, counts, "")
    graph.opening_side = _opening_side(atlas)
    return graph
