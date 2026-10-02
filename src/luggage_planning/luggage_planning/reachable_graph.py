#!/usr/bin/env python3
"""6-connected graph over a reachability atlas, masked by cargo occupancy.

The atlas stays a path prior: this module does not invent placement slots.
Session occupancy comes only from the caller's snapshot or ``blocked_fn``.

Section A (above the ``st09_contracts`` divider) is the legacy atlas-cell
graph consumed by ``path_candidates``/``trajectory_selector``; its API and
behaviour are frozen and must stay byte-identical. Section B adds the
ST09-B directed q-state route graph: nodes are full joint solutions with
FK pose / region / load / IK-family attributes, edges carry complete timed
trajectories, and a JSON store persists static identity while keeping
loaded trajectories candidate-only until re-verified in session.
"""

from __future__ import division

import json
import math
import os
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Sequence, Tuple

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


# -- Section B: ST09-B directed q-state route graph -----------------------
#
# Node primary state is the full joint solution q (st09_contracts
# GraphNode). Two nodes with the same TCP XYZ but different q never merge;
# the release q is registered twice (CARRYING arrival + UNLOADED retreat
# twin), which is why this store keys on (load_state, q) while the frozen
# GraphNode.same_state contract keeps comparing q alone.
#
# st09_contracts imports are lazy: Section A must stay importable from a
# plain install prefix where the repo root is not on sys.path.

ROUTE_GRAPH_SCHEMA = "st09.route-graph/1"

#: Exact-q merge tolerance for node registration (matches the frozen
#: GraphNode.same_state tolerance).
Q_MERGE_TOL = 1e-9


def _contracts():
    """Lazy (st09_contracts.graph module, digest_of) import pair."""
    from st09_contracts import graph as contracts_graph
    from st09_contracts._base import digest_of
    return contracts_graph, digest_of


def _matrix_to_quaternion_xyzw(matrix):
    """3x3 rotation matrix (nested sequence or ndarray) -> unit xyzw."""
    m = [[float(matrix[i][j]) for j in range(3)] for i in range(3)]
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quat = ((m[2][1] - m[1][2]) / scale,
                (m[0][2] - m[2][0]) / scale,
                (m[1][0] - m[0][1]) / scale,
                0.25 * scale)
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        scale = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        quat = (0.25 * scale,
                (m[0][1] + m[1][0]) / scale,
                (m[0][2] + m[2][0]) / scale,
                (m[2][1] - m[1][2]) / scale)
    elif m[1][1] > m[2][2]:
        scale = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        quat = ((m[0][1] + m[1][0]) / scale,
                0.25 * scale,
                (m[1][2] + m[2][1]) / scale,
                (m[0][2] - m[2][0]) / scale)
    else:
        scale = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        quat = ((m[0][2] + m[2][0]) / scale,
                (m[1][2] + m[2][1]) / scale,
                0.25 * scale,
                (m[1][0] - m[0][1]) / scale)
    norm = math.sqrt(sum(v * v for v in quat))
    return tuple(v / norm for v in quat)  # type: ignore[return-value]


def make_offline_fk(urdf, tcp_path):
    """q -> (tcp_position, tcp_quaternion_xyzw) in the URDF root frame.

    The returned closure is the FK injected into Section B builders; the
    caller owns which frame the URDF root expresses (world via the scene
    base patch, or base_link).
    """
    def fk(q):
        transform = urdf.tcp(list(q), tcp_path)
        position = (float(transform[0][3]), float(transform[1][3]),
                    float(transform[2][3]))
        return position, _matrix_to_quaternion_xyzw(transform)
    return fk


@dataclass(frozen=True)
class RegionAnchor:
    """One anchored graph region to seed from the atlas prior.

    ``xyz`` is the container-frame contact point and ``yaw`` the directed
    container-frame tool yaw used for atlas queries (the FK result itself
    is root-frame; the anchor only selects seeds).
    """

    region: str
    xyz: Tuple[float, float, float]
    yaw: float
    load_state: "object"  # st09_contracts.graph.LoadState

    def __post_init__(self):
        object.__setattr__(self, "region", str(self.region))
        object.__setattr__(self, "xyz", tuple(float(v) for v in self.xyz))
        object.__setattr__(self, "yaw", float(self.yaw))


class RouteGraph(object):
    """Directed q-state graph with complete timed-trajectory edges.

    ``edges`` are usable in-route (built and judged this session);
    ``candidate_edges`` hold store-loaded historical trajectories that
    must be re-verified before promotion — a cached unverified polyline
    is never a legal edge on its own.
    """

    def __init__(self, static_identity=""):
        self._nodes = {}
        self._edges = {}
        self._candidates = {}
        self._out = {}
        self._incoming = {}
        self._prior_gaps = []
        self._static_identity = str(static_identity or "")
        self._id_suffix = {}

    @property
    def static_identity(self):
        return self._static_identity

    @property
    def nodes(self):
        return dict(self._nodes)

    @property
    def edges(self):
        return dict(self._edges)

    @property
    def candidate_edges(self):
        return dict(self._candidates)

    @property
    def prior_gaps(self):
        return tuple(self._prior_gaps)

    def node(self, node_id):
        return self._nodes[str(node_id)]

    def node_for_q(self, load_state, q, tol=Q_MERGE_TOL):
        """Existing node with this load state and q within ``tol``, or None."""
        values = tuple(float(v) for v in q)
        for node in self._nodes.values():
            if str(node.load_state) != str(load_state):
                continue
            if all(abs(a - b) <= tol for a, b in zip(node.q, values)):
                return node
        return None

    def nodes_by_region(self, region):
        return tuple(node for node in self._nodes.values()
                     if node.region == str(region))

    def register_node(self, q, tcp_position, tcp_quaternion_xyzw, region,
                      load_state, ik_family=""):
        """Idempotent node registration keyed on (load_state, q) at 1e-9.

        Same TCP XYZ with a different q registers a distinct node; an
        exact-q duplicate returns the existing node unchanged.
        """
        contracts_graph, digest_of = _contracts()
        values = tuple(float(v) for v in q)
        if len(values) != 6 or not all(math.isfinite(v) for v in values):
            raise ValueError("register_node: q must be 6 finite values")
        existing = self.node_for_q(load_state, values)
        if existing is not None:
            return existing
        tag = "%s-%s-%s" % (str(load_state), str(region), digest_of(values)[:8])
        suffix = self._id_suffix.get(tag, 0)
        node_id = tag if suffix == 0 else "%s-%d" % (tag, suffix)
        while node_id in self._nodes:
            suffix += 1
            node_id = "%s-%d" % (tag, suffix)
        self._id_suffix[tag] = suffix + 1
        node = contracts_graph.GraphNode(
            node_id=node_id,
            q=values,
            tcp_position=tuple(float(v) for v in tcp_position),
            tcp_quaternion_xyzw=tuple(float(v) for v in tcp_quaternion_xyzw),
            region=str(region),
            load_state=load_state,
            ik_family=str(ik_family or ""),
        )
        self._nodes[node_id] = node
        self._out.setdefault(node_id, [])
        self._incoming.setdefault(node_id, [])
        return node

    def add_edge(self, edge):
        """Register a verified in-session edge (load-state consistent)."""
        source = self.node(edge.source_node_id)
        destination = self.node(edge.destination_node_id)
        for endpoint in (source, destination):
            if str(endpoint.load_state) != str(edge.load_state):
                raise ValueError(
                    "add_edge: %s load state %s vs endpoint %s %s" % (
                        edge.edge_id, edge.load_state,
                        endpoint.node_id, endpoint.load_state))
        if edge.edge_id in self._edges or edge.edge_id in self._candidates:
            raise ValueError("add_edge: duplicate edge id %s" % edge.edge_id)
        self._edges[edge.edge_id] = edge
        self._out.setdefault(source.node_id, []).append(edge.edge_id)
        self._incoming.setdefault(destination.node_id, []).append(edge.edge_id)
        return edge

    def add_candidate_edge(self, edge):
        """Register a store-loaded historical edge (re-verify before use)."""
        if edge.edge_id in self._edges or edge.edge_id in self._candidates:
            raise ValueError(
                "add_candidate_edge: duplicate edge id %s" % edge.edge_id)
        source = self.node(edge.source_node_id)
        destination = self.node(edge.destination_node_id)
        self._candidates[edge.edge_id] = edge
        self._out.setdefault(source.node_id, []).append(edge.edge_id)
        self._incoming.setdefault(destination.node_id, []).append(edge.edge_id)
        return edge

    def promote_candidate(self, edge_id, replacement):
        """Swap a re-verified edge in, removing the unverified candidate."""
        edge_id = str(edge_id)
        if edge_id not in self._candidates:
            raise KeyError("promote_candidate: %s is not a candidate" % edge_id)
        del self._candidates[edge_id]
        return self.add_edge(replacement)

    def drop_candidate(self, edge_id):
        self._candidates.pop(str(edge_id), None)

    def outgoing(self, node_id):
        return tuple(self._edges[eid] for eid in self._out.get(str(node_id), ())
                     if eid in self._edges)

    def outgoing_candidates(self, node_id):
        return tuple(self._candidates[eid]
                     for eid in self._out.get(str(node_id), ())
                     if eid in self._candidates)

    def incoming(self, node_id):
        return tuple(self._edges[eid]
                     for eid in self._incoming.get(str(node_id), ())
                     if eid in self._edges)

    def note_prior_gap(self, text):
        """Record an atlas prior gap (ATLAS_ORIENTATION_UNCOVERED class).

        Prior gaps steer bounded MoveIt supplementation; they are never a
        rejection reason on their own.
        """
        self._prior_gaps.append(str(text))

    def digest(self):
        """Content identity over nodes, all edges, and prior gaps.

        Verification status is session state, deliberately excluded: a
        store round-trip demotes edges to candidates and must reproduce
        the saved digest unchanged, so verified and candidate edges enter
        one order-independent union.
        """
        _, digest_of = _contracts()
        return digest_of(
            ROUTE_GRAPH_SCHEMA,
            self._static_identity,
            sorted(node.digest() for node in self._nodes.values()),
            sorted(edge.digest() for edge in
                   list(self._edges.values()) + list(self._candidates.values())),
            tuple(self._prior_gaps),
        )


def _trajectory_to_dict(trajectory):
    return {
        "joint_names": list(trajectory.joint_names),
        "positions": [list(row) for row in trajectory.positions],
        "velocities": [list(row) for row in trajectory.velocities],
        "accelerations": [list(row) for row in trajectory.accelerations],
        "time_from_start_ns": list(trajectory.time_from_start_ns),
    }


def _trajectory_from_dict(payload):
    contracts_graph, _ = _contracts()
    return contracts_graph.TimedTrajectory(
        joint_names=tuple(payload["joint_names"]),
        positions=tuple(tuple(row) for row in payload["positions"]),
        velocities=tuple(tuple(row) for row in payload["velocities"]),
        accelerations=tuple(tuple(row) for row in payload["accelerations"]),
        time_from_start_ns=tuple(int(v) for v in payload["time_from_start_ns"]),
    )


def _node_to_dict(node):
    return {
        "node_id": node.node_id,
        "q": list(node.q),
        "tcp_position": list(node.tcp_position),
        "tcp_quaternion_xyzw": list(node.tcp_quaternion_xyzw),
        "region": node.region,
        "load_state": str(node.load_state),
        "ik_family": node.ik_family,
    }


class GraphStore(object):
    """JSON persistence for RouteGraph with digest verification.

    Loaded trajectories re-enter the graph as candidate edges
    (``provenance: store``): static q-node seeds are reusable as-is, a
    cached trajectory must be re-probed before it may join a route.
    """

    def __init__(self, root):
        self.root = str(root)

    @staticmethod
    def static_identity(urdf_hash, scene_geometry_id, scene_geometry_hash,
                        planner_config):
        """Identity of everything static the graph was built against.

        Occupancy is deliberately excluded — it binds per-request through
        the snapshot digest on the PlannedRoute, not through the store.
        """
        _, digest_of = _contracts()
        return digest_of(
            ROUTE_GRAPH_SCHEMA, str(urdf_hash), str(scene_geometry_id),
            str(scene_geometry_hash),
            json.dumps(planner_config, sort_keys=True))

    def path_for(self, graph_id):
        return os.path.join(self.root, "%s.json" % str(graph_id))

    def save(self, graph, graph_id):
        """Write ``<graph_id>.json`` exclusively; returns (path, digest)."""
        os.makedirs(self.root, exist_ok=True)
        path = self.path_for(graph_id)
        digest = graph.digest()
        payload = {
            "schema": ROUTE_GRAPH_SCHEMA,
            "graph_id": str(graph_id),
            "static_identity": graph.static_identity,
            "digest": digest,
            "nodes": [_node_to_dict(node) for node in graph.nodes.values()],
            "edges": [self._edge_to_dict(edge) for edge in graph.edges.values()],
            "candidate_edges": [self._edge_to_dict(edge)
                                for edge in graph.candidate_edges.values()],
            "prior_gaps": list(graph.prior_gaps),
        }
        text = json.dumps(payload, indent=1, sort_keys=True)
        if os.path.exists(path):
            with open(path, "r") as handle:
                if handle.read() != text:
                    raise IOError("refusing to overwrite %s with new content"
                                  % path)
        else:
            with open(path, "x") as handle:
                handle.write(text)
        return path, digest

    @staticmethod
    def _edge_to_dict(edge):
        return {
            "edge_id": edge.edge_id,
            "source_node_id": edge.source_node_id,
            "destination_node_id": edge.destination_node_id,
            "load_state": str(edge.load_state),
            "static_identity": edge.static_identity,
            "trajectory": _trajectory_to_dict(edge.trajectory),
            "validation_revision": edge.validation_revision,
            "attributes": dict(edge.attributes),
        }

    def load(self, graph_id, *, expected_static_identity=None):
        """Rebuild a RouteGraph; every stored edge arrives as a candidate."""
        contracts_graph, _ = _contracts()
        with open(self.path_for(graph_id), "r") as handle:
            payload = json.load(handle)
        if payload.get("schema") != ROUTE_GRAPH_SCHEMA:
            raise ValueError("unsupported route-graph schema %r"
                             % (payload.get("schema"),))
        if expected_static_identity is not None and \
                payload.get("static_identity") != str(expected_static_identity):
            raise ValueError("static identity mismatch for %s" % graph_id)
        graph = RouteGraph(static_identity=payload.get("static_identity", ""))
        load_state_by_value = {
            str(state): state for state in (
                contracts_graph.LoadState.UNLOADED,
                contracts_graph.LoadState.CARRYING)}
        for node_payload in payload.get("nodes", []):
            graph.register_node(
                node_payload["q"],
                node_payload["tcp_position"],
                node_payload["tcp_quaternion_xyzw"],
                node_payload["region"],
                load_state_by_value[node_payload["load_state"]],
                ik_family=node_payload.get("ik_family", ""),
            )
        for edge_payload in payload.get("edges", []) + \
                payload.get("candidate_edges", []):
            graph.add_candidate_edge(contracts_graph.GraphEdge(
                edge_id=edge_payload["edge_id"],
                source_node_id=edge_payload["source_node_id"],
                destination_node_id=edge_payload["destination_node_id"],
                load_state=load_state_by_value[edge_payload["load_state"]],
                static_identity=edge_payload["static_identity"],
                trajectory=_trajectory_from_dict(edge_payload["trajectory"]),
                validation_revision=edge_payload.get("validation_revision"),
                attributes=dict(edge_payload.get("attributes", {})),
            ))
        for gap in payload.get("prior_gaps", []):
            graph.note_prior_gap(gap)
        if graph.digest() != payload.get("digest"):
            raise ValueError("digest mismatch for %s" % graph_id)
        return graph


def build_route_graph_from_atlas(atlas, anchors, *, fk, seed_set="both",
                                 max_seeds=3, graph=None):
    """Register q-nodes for anchors covered by the atlas prior.

    Uses only public atlas APIs (``directed_yaw_candidates`` over 2pi with
    both pi twins — never a two-bin whitelist — and the deterministic
    ``atlas_seeds_for_query`` neighbourhood search). A yaw with no seeds
    records an ``ATLAS_ORIENTATION_UNCOVERED`` prior gap and leaves that
    anchor for bounded MoveIt supplementation; it is never a rejection.
    """
    from luggage_planning.atlas_runtime import (
        atlas_seeds_for_query,
        directed_yaw_candidates,
    )
    if graph is None:
        graph = RouteGraph()
    seed_sets = (("transit", "contact") if seed_set == "both"
                 else (str(seed_set),))
    for anchor in anchors:
        for yaw in directed_yaw_candidates(atlas, anchor.yaw):
            found = 0
            for one_set in seed_sets:
                result = atlas_seeds_for_query(
                    atlas, anchor.xyz[0], anchor.xyz[1], anchor.xyz[2], yaw,
                    seed_set=one_set, max_seeds=max_seeds)
                for seed in result.get("seeds", ()):
                    position, quaternion = fk(seed)
                    graph.register_node(
                        seed, position, quaternion, anchor.region,
                        anchor.load_state,
                        ik_family="atlas-%s" % one_set)
                    found += 1
            if found == 0:
                graph.note_prior_gap(
                    "ATLAS_ORIENTATION_UNCOVERED region=%s yaw=%.6f xyz=(%.3f, %.3f, %.3f)"
                    % (anchor.region, yaw, anchor.xyz[0], anchor.xyz[1],
                       anchor.xyz[2]))
    return graph
