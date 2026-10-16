#!/usr/bin/env python3
"""ST09-B directed q-state route graph (Section B).

[ST09-I] The legacy Section A atlas-cell graph was deleted with the
geometric selection bypass: the atlas remains a prior through
``build_route_graph_from_atlas`` and the bounded MoveIt supplementation,
and never gains execution permission. Nodes here are full joint
solutions with FK pose / region / load / IK-family attributes, edges
carry complete timed trajectories, and a JSON store persists static
identity while keeping loaded trajectories candidate-only until
re-verified in session.
"""

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

from __future__ import division

import json
import math
import os
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Sequence, Tuple

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

    #: Regions whose nodes belong to ONE select request (the candidate's
    #: anchor ladder, the home registration, the goals). Everything else
    #: (cross-request IK caches, atlas nodes) stays.
    REQUEST_SCOPED_REGIONS = ("staging", "portal", "slot_approach",
                              "slot_upper", "exit", "home", "slot",
                              "pick_home")

    def clear_request_scoped(self):
        """[ST09-I] per-select reset of the candidate-scoped regions.

        The selector's graph lives for the node's lifetime; a previous
        request's anchors pollute the next request's hop destinations
        and the branch combinations grow unboundedly (live: the second
        box's select carried the first box's anchors — three branches x
        four regions — and burned the frozen budget before any chain
        resolved)."""
        drop = {node_id for node_id, node in self._nodes.items()
                if str(node.region) in self.REQUEST_SCOPED_REGIONS}
        if not drop:
            return 0
        self._edges = {
            edge_id: edge for edge_id, edge in self._edges.items()
            if edge.source_node_id not in drop
            and edge.destination_node_id not in drop}
        self._candidates = {
            edge_id: edge for edge_id, edge in self._candidates.items()
            if edge.source_node_id not in drop
            and edge.destination_node_id not in drop}
        self._nodes = {node_id: node for node_id, node
                       in self._nodes.items() if node_id not in drop}
        self._out = {node_id: targets for node_id, targets
                     in self._out.items() if node_id not in drop}
        self._incoming = {node_id: sources for node_id, sources
                          in self._incoming.items()
                          if node_id not in drop}
        return len(drop)

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


class GraphRoutePlanner(object):
    """ST09-B implementation of the frozen ``RoutePlanner`` protocol.

    ``plan_route`` composes the full unauthorized route for one carried
    placement: goal nodes from pure target synthesis (no tool-yaw
    pruning), the bounded entry chain, the release twin, and the
    post-release retreat chain with the placed box as a world obstacle.
    Budget exhaustion raises ``PLANNING_BUDGET_EXHAUSTED`` (never
    BIN_FULL). A certifier injected here is ADVISORY ONLY: the returned
    PlannedRoute has no certification field, and the route content is
    byte-identical with or without one (fake-certifier isolation is
    asserted by tests). Production code never imports
    st09_contracts.fixtures and never constructs a CertificationResult.
    """

    def __init__(self, graph, edge_builder, *, home_node_id,
                 goal_seeds=None, budget_policy=None,
                 entry_hops=None, retreat_hops=None,
                 t_frame_container=(0.0, 0.0, 0.0),
                 q_frame_container=(0.0, 0.0, 0.0, 1.0),
                 certifier=None, refine=None, urdf=None, tcp_path=None,
                 region_poses=None, scope_node_ids=None, scene_states=None):
        from luggage_planning.grid_path_search import RouteBudgetPolicy
        #: region -> world TCP position from the anchor plan.
        self.scene_states = None if scene_states is None else dict(scene_states)
        self.region_poses = dict(region_poses or {})
        self.continuation_diagnostics = []
        #: Node ids of the current candidate (None = whole graph). Nodes
        #: this planner registers join it; earlier candidates' do not.
        self.scope = None if scope_node_ids is None \
            else set(scope_node_ids)
        self.graph = graph
        self.edge_builder = edge_builder
        self.home_node_id = str(home_node_id)
        self.goal_seeds = goal_seeds
        self.budget_policy = budget_policy or RouteBudgetPolicy()
        self.entry_hops = entry_hops
        self.retreat_hops = retreat_hops
        self.t_frame_container = tuple(float(v) for v in t_frame_container)
        self.q_frame_container = tuple(float(v) for v in q_frame_container)
        self.certifier = certifier  # advisory only; see class docstring
        self.refine = refine  # production default: offline_ik refine
        self.urdf = urdf
        self.tcp_path = tcp_path
        self.last_entry = None
        self.last_retreat = None
        self.last_goal = None
        self.last_budget_snapshot = None
        self.last_advisory_cert = None

    # -- helpers ----------------------------------------------------------

    def _payload_identity(self, attachment):
        from st09_contracts.planning import PayloadIdentity, PayloadSymmetry
        symmetry = (PayloadSymmetry.PI if attachment.symmetry == "pi"
                    else PayloadSymmetry.NONE)
        return PayloadIdentity(
            cargo_id=attachment.cargo_id,
            box_id=attachment.box_id,
            request_id=attachment.request_id,
            cycle_id=attachment.cycle_id,
            dimensions_wdh=attachment.dimensions_wdh,
            symmetry=symmetry,
            t_tcp_box_translation=attachment.t_tcp_box_translation,
            t_tcp_box_quaternion_xyzw=attachment.t_tcp_box_quaternion_xyzw,
            attachment_incarnation=attachment.attachment_incarnation,
            acquisition_stamp_ns=attachment.acquisition_stamp_ns)

    def _goal_nodes(self, targets):
        """Register goal q-nodes (plus release twins) for every target."""
        from luggage_planning.place_targets import goal_joint_sets
        if self.goal_seeds is None:
            raise ValueError(
                "GraphRoutePlanner needs goal_seeds(target) (atlas or "
                "MoveIt supplementation supplies them)")
        load_state = _contracts()[0].LoadState
        goal_nodes = []
        refine_kwargs = ({"refine": self.refine} if self.refine is not None
                         else {"urdf": self.urdf, "tcp_path": self.tcp_path})
        for goal_set in goal_joint_sets(
                targets, seeds_for_target=self.goal_seeds, **refine_kwargs):
            for goal in goal_set.goals:
                node = self.graph.register_node(
                    goal.q, goal_set.target.position,
                    goal_set.target.quaternion_xyzw,
                    "slot", load_state.CARRYING,
                    ik_family=goal.ik_family)
                # Release twin for the post-release retreat chain.
                twin = self.graph.register_node(
                    goal.q, goal_set.target.position,
                    goal_set.target.quaternion_xyzw,
                    "slot", load_state.UNLOADED,
                    ik_family=goal.ik_family)
                if self.scope is not None:
                    self.scope.update((node.node_id, twin.node_id))
                goal_nodes.append(node)
        return goal_nodes

    def _refine_one(self, position, quaternion, seed_q):
        """Exact refine from one seed; a miss stays a miss."""
        if self.refine is None:
            return None
        try:
            q, _info = self.refine(tuple(position), tuple(quaternion),
                                   list(seed_q))
        except Exception:  # noqa: BLE001 - one seed must not abort search
            return None
        return None if q is None else tuple(float(v) for v in q)

    def _register(self, q, position, quaternion, region, load_state, family):
        node = self.graph.node_for_q(str(load_state), q)
        if node is None:
            node = self.graph.register_node(q, position, quaternion, region,
                                            load_state, ik_family=family)
        if self.scope is not None:
            self.scope.add(node.node_id)
        return node

    def _entry_table(self):
        from luggage_planning.grid_path_search import (
            DEFAULT_ENTRY_HOPS, hops_from_table)
        return self.entry_hops or hops_from_table(DEFAULT_ENTRY_HOPS,
                                                  "carrying")

    def _region_pose(self, region, goal):
        """Anchor pose for a region: the current plan, else an anchor.

        ``region_poses`` belong to the current candidate, so they win over
        graph anchors, which may be left from an earlier candidate in the
        same request. They also let a region whose anchor refine missed
        from every forward seed be back-seeded from the goal branch.
        """
        position = (self.region_poses or {}).get(region)
        if position is not None:
            return tuple(float(v) for v in position), \
                tuple(goal.tcp_quaternion_xyzw)
        anchors = [node for node in self.graph.nodes_by_region(region)
                   if str(node.load_state) == "carrying"
                   and (self.scope is None or node.node_id in self.scope)]
        if anchors:
            return (tuple(anchors[0].tcp_position),
                    tuple(anchors[0].tcp_quaternion_xyzw))
        return None

    def _register_branch_ladders(self, goal_nodes):
        """Back-seed each upstream region from every goal's branch.

        Walks the entry hops backwards from the slot, refining each
        region's pose from the next region's q, and stops at the
        destination of the last joint-space hop. A goal on another
        elbow/wrist/heading branch then has a same-branch cartesian chain,
        and the branch change happens in that joint-space transit.
        """
        if self.refine is None:
            return 0
        load_state = _contracts()[0].LoadState
        hops = list(self._entry_table())
        upstream = hops[:-1]
        registered = 0
        for goal in goal_nodes:
            q = tuple(goal.q)
            for hop in reversed(upstream):
                pose = self._region_pose(hop.to_region, goal)
                if pose is None:
                    break
                position, quaternion = pose
                q_new = self._refine_one(position, quaternion, q)
                if q_new is None:
                    break
                self._register(q_new, position, quaternion, hop.to_region,
                               load_state.CARRYING, "ladder")
                registered += 1
                q = q_new
                if hop.edge_class in ("PORTAL_TRANSIT", "RETURN"):
                    break
        return registered

    def _frontier_continuation(self, expired=lambda: False):
        """Short cartesian hops (INSERT, DESCEND, RETREAT) solved from
        the frontier's own q. A region with no registered node falls back
        to its anchor-plan pose (RETREAT holds the frontier orientation)."""
        if self.refine is None:
            return None
        load_state = _contracts()[0].LoadState

        def solve(frontier, hop, destinations):
            if hop.edge_class not in ("INSERT", "DESCEND", "RETREAT"):
                return ()
            # The position is the region's planned pose. The heading is
            # the frontier's, and the only seed is the frontier q. A
            # destination quaternion from another goal or atlas seed
            # would ask the short Cartesian hop to spin the wrist.
            planned = (self.region_poses or {}).get(hop.to_region)
            if planned is not None:
                position = tuple(float(v) for v in planned)
            elif destinations:
                position = tuple(float(v) for v in destinations[0].tcp_position)
            else:
                return ()
            quaternion = tuple(float(v) for v in frontier.tcp_quaternion_xyzw)
            q = self._refine_one(position, quaternion, frontier.q)
            diagnostic = {
                "frontier": frontier.node_id, "to_region": hop.to_region,
                "edge_class": hop.edge_class, "seed_q": list(frontier.q),
                "target_position": list(position),
                "target_quaternion_xyzw": list(quaternion),
                "result_q": None if q is None else list(q),
                "reason": "ik_failed" if q is None else "refined"}
            self.continuation_diagnostics.append(diagnostic)
            del self.continuation_diagnostics[:-64]
            if q is None:
                return ()
            from luggage_planning.grid_path_search import descend_pair_rank
            # A Newton step that leaves the wrist/elbow branch is not a
            # continuous successor. Do not hand that q to Cartesian.
            diagnostic["branch_rank"] = list(descend_pair_rank(frontier.q, q))
            if diagnostic["branch_rank"][0] != 0:
                diagnostic["reason"] = "branch_change"
                if hop.edge_class != "RETREAT":
                    return ()
                from luggage_planning.retreat_continuation import refine_retreat
                q, continuation = refine_retreat(
                    frontier.tcp_position, position, quaternion, frontier.q,
                    self._refine_one, expired)
                diagnostic["incremental"] = continuation
                if q is None:
                    return ()
                diagnostic["reason"] = "incremental_refined"
                diagnostic["result_q"] = list(q)
            node = self._register(
                q, position, quaternion, hop.to_region,
                load_state(hop.load_state), "frontier")
            if hop.edge_class == "DESCEND":
                self._register(
                    q, position, quaternion, hop.to_region,
                    load_state.UNLOADED, "frontier")
            return (node,)

        return solve

    def _released_box(self, attachment, candidate):
        """World-frame box at the candidate's placed pose, composed
        through the injected world<-container transform."""
        from luggage_planning.moveit_edge_adapter import BoxSpec
        from st09_contracts.attachment import compose_transform
        position, quaternion = compose_transform(
            self.t_frame_container, self.q_frame_container,
            candidate.position_container, candidate.quaternion_xyzw_container)
        return BoxSpec(
            object_id="released-%s" % attachment.box_id,
            center=position,
            size_whd=attachment.dimensions_wdh,
            quaternion_xyzw=quaternion)

    def _scenes(self, attachment, candidate, snapshot):
        if self.scene_states is not None:
            return self.scene_states["carrying"], self.scene_states["unloaded"]
        from luggage_planning.moveit_edge_adapter import (
            PayloadSpec,
            SceneState,
            scene_boxes_from_snapshot,
        )
        carrying = SceneState(
            load_state="carrying",
            payload=PayloadSpec(
                dimensions_wdh=attachment.dimensions_wdh,
                t_tcp_box_translation=attachment.t_tcp_box_translation,
                t_tcp_box_quaternion_xyzw=attachment.t_tcp_box_quaternion_xyzw),
            placed_boxes=scene_boxes_from_snapshot(snapshot)
            if snapshot is not None else ())
        unloaded = SceneState(
            load_state="unloaded",
            released_box=self._released_box(attachment, candidate),
            placed_boxes=scene_boxes_from_snapshot(snapshot)
            if snapshot is not None else ())
        return carrying, unloaded

    # -- protocol -----------------------------------------------------------

    def plan_route(self, snapshot, attachment, candidate, targets, budget_ns):
        from luggage_planning.grid_path_search import (
            RouteBudget,
            RouteBudgetPolicy,
            RoutePlanningError,
            RouteSearch,
        )
        contracts_graph, digest_of = _contracts()
        total_wall_s = max(0.0, float(budget_ns) * 1e-9)
        if total_wall_s <= 0.0:
            raise RoutePlanningError("PLANNING_BUDGET_EXHAUSTED", {
                "budget_ns": float(budget_ns)})
        policy = RouteBudgetPolicy(
            total_wall_s=total_wall_s,
            single_edge_wall_s=self.budget_policy.single_edge_wall_s,
            max_edge_repair_attempts=self.budget_policy.max_edge_repair_attempts,
            retreat_min_share=self.budget_policy.retreat_min_share)
        budget = RouteBudget(policy=policy)
        goal_nodes = self._goal_nodes(targets)
        if not goal_nodes:
            # A seed set that refines onto no goal is a sampling miss, not
            # proof that no configuration reaches the target.
            raise RoutePlanningError("NO_FEASIBLE_ROUTE", {
                "prior_gaps": list(self.graph.prior_gaps),
                "detail": "no IK goals refined for the given targets",
                "candidate_failure": "IK_UNAVAILABLE",
                "search_outcome": "SEARCH_INCOMPLETE"})
        carrying, unloaded = self._scenes(attachment, candidate, snapshot)
        self._register_branch_ladders(goal_nodes)
        if self.scope is not None:
            self.scope.add(self.home_node_id)
        if getattr(self, "trace", None) is not None:
            self.trace.note_regions(self.graph, "after_ladders")
        fk = None
        if self.urdf is not None and self.tcp_path is not None:
            fk = make_offline_fk(self.urdf, self.tcp_path)
        search = RouteSearch(self.graph, self.edge_builder, budget,
                             continuation=self._frontier_continuation(budget.exhausted),
                             scope=self.scope,
                             trace=getattr(self, "trace", None),
                             fk=fk)
        self.last_search = search
        entry, retreat, goal = search.best_full_route(
            self.home_node_id, goal_nodes,
            entry_hops=self.entry_hops, retreat_hops=self.retreat_hops,
            scene_carrying=carrying, scene_unloaded=unloaded)
        self.last_entry, self.last_retreat = entry, retreat
        self.last_goal = goal
        self.last_budget_snapshot = budget.snapshot()
        edges = tuple(entry.edges) + tuple(retreat.edges)
        payload = self._payload_identity(attachment)
        route = contracts_graph.PlannedRoute(
            route_id="route-b-%s" % digest_of(
                attachment.request_id, candidate.candidate_id,
                goal.node_id, entry.node_ids, retreat.node_ids)[:12],
            request_id=attachment.request_id,
            snapshot_digest=snapshot.digest(),
            payload_digest=payload.digest(),
            attachment_digest=attachment.digest(),
            candidate_id=candidate.candidate_id,
            entry_edge_ids=tuple(edge.edge_id for edge in entry.edges),
            retreat_edge_ids=tuple(edge.edge_id for edge in retreat.edges),
            edge_digests=tuple(edge.digest() for edge in edges),
            trajectory_digest=digest_of(
                *(edge.trajectory.digest() for edge in edges)),
            planning_revision="B:%s" % self.graph.digest()[:12],
        )
        if self.certifier is not None:
            # Advisory only: the result is recorded and discarded; it can
            # never enter the PlannedRoute (no certification field).
            self.last_advisory_cert = self.certifier.certify(route)
        return route

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
