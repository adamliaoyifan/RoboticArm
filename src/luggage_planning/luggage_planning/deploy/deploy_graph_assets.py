"""Offline validation of the deploy directed route graphs (ST09-PD).

ROS-free. Structural guarantees the acceptance matrix names -- edge
endpoints exist, trajectory direction matches the edge direction,
costs are finite, q-states are unique, release twins exist -- plus the
route-coverage question: from each declared start class to every
healthy fixture target, at least one directed route of offline-verified
edges must exist. Missing coverage is reported as ``not_evaluated``,
never as success. Loaded store edges are candidates by construction;
production re-verifies them in session, so this validator never claims
a live certification.
"""

from __future__ import division

import math
from collections import deque

from luggage_planning.atlas.graph import GraphStore

#: Region semantics of the canonical offline chain.
START_REGIONS = ("portal",)
TARGET_REGION = "slot"
RELEASE_TWIN_REGIONS = ("slot",)
EDGE_CLASSES = ("TRAVERSE", "INSERT", "DESCEND", "RETREAT", "RETURN")


def expected_static_identity(urdf_md5, geometry_id, geometry_hash,
                             planner_config):
    return GraphStore.static_identity(
        urdf_md5, geometry_id, geometry_hash, planner_config)


def load_deploy_graph(path, *, expected_identity=None):
    """Strict-load one graph json; identity mismatch raises ValueError."""
    import os

    graph_id = os.path.splitext(os.path.basename(path))[0]
    return GraphStore(os.path.dirname(path)).load(
        graph_id, expected_static_identity=expected_identity)


def validate_graph_structure(graph):
    """Return ``{ok, issues, ...}`` over one loaded RouteGraph."""
    issues = []
    nodes = graph.nodes
    edges = dict(graph.edges)
    edges.update(graph.candidate_edges)

    # Unique q-states per load state.
    q_tolerance = 1e-9
    seen = []
    for node in nodes.values():
        for other_id, other_q, other_load in seen:
            if other_load == str(node.load_state) and all(
                    abs(a - b) <= q_tolerance
                    for a, b in zip(node.q, other_q)):
                issues.append(
                    "duplicate q-state %s vs %s" % (node.node_id, other_id))
        seen.append((node.node_id, node.q, str(node.load_state)))

    for edge in edges.values():
        if edge.source_node_id not in nodes or edge.destination_node_id \
                not in nodes:
            issues.append("dangling edge %s" % edge.edge_id)
            continue
        source = nodes[edge.source_node_id]
        destination = nodes[edge.destination_node_id]
        if str(edge.load_state) != str(source.load_state) or \
                str(edge.load_state) != str(destination.load_state):
            issues.append("load-state inconsistent edge %s" % edge.edge_id)
        trajectory = edge.trajectory
        first, last = trajectory.positions[0], trajectory.positions[-1]
        if any(abs(a - b) > q_tolerance for a, b in zip(first, source.q)):
            issues.append(
                "edge %s trajectory starts off source q" % edge.edge_id)
        if any(abs(a - b) > q_tolerance for a, b in zip(last, destination.q)):
            issues.append(
                "edge %s trajectory ends off destination q" % edge.edge_id)
        cost = edge.attributes.get("cost_ns")
        if (not isinstance(cost, (int, float)) or isinstance(cost, bool)
                or not math.isfinite(float(cost)) or float(cost) <= 0.0):
            issues.append("edge %s cost not finite/positive (%r)"
                          % (edge.edge_id, cost))
        edge_class = str(edge.attributes.get("edge_class") or "")
        if edge_class not in EDGE_CLASSES:
            issues.append("edge %s unknown class %r"
                          % (edge.edge_id, edge_class))
        if len(trajectory.joint_names) != 6:
            issues.append("edge %s joint order lost" % edge.edge_id)

    # Release twins: every carrying slot node has an unloaded twin q.
    carrying_slots = [node for node in nodes.values()
                      if node.region == TARGET_REGION
                      and str(node.load_state) == "carrying"]
    for node in carrying_slots:
        twin = any(
            other.region == TARGET_REGION
            and str(other.load_state) == "unloaded"
            and all(abs(a - b) <= q_tolerance
                    for a, b in zip(node.q, other.q))
            for other in nodes.values())
        if not twin:
            issues.append("missing release twin for %s" % node.node_id)

    adjacency = {}
    for edge in edges.values():
        adjacency.setdefault(edge.source_node_id, []).append(
            edge.destination_node_id)

    def reachable_path(start_id, goal_id):
        queue = deque([start_id])
        parent = {start_id: None}
        while queue:
            current = queue.popleft()
            if current == goal_id:
                break
            for nxt in adjacency.get(current, ()):
                if nxt not in parent:
                    parent[nxt] = current
                    queue.append(nxt)
        if goal_id not in parent:
            return None
        path = []
        cursor = goal_id
        while cursor is not None:
            path.append(cursor)
            cursor = parent[cursor]
        return list(reversed(path))

    # Route coverage is per (start class, target): a class covers a
    # target when ANY of its start nodes has a directed edge path to it.
    # Requiring every start node to reach every target would demand a
    # mesh the offline chains never promised.
    coverage = {}
    not_evaluated = []
    for start_region in START_REGIONS:
        starts = [node.node_id for node in nodes.values()
                  if node.region == start_region
                  and str(node.load_state) == "carrying"]
        for target in carrying_slots:
            best_path = None
            for start_id in starts:
                path = reachable_path(start_id, target.node_id)
                if path is not None:
                    best_path = (start_id, path)
                    break
            coverage["%s->%s" % (start_region, target.node_id)] = (
                best_path[1] if best_path else None)
            if best_path is None:
                not_evaluated.append(
                    "%s->%s" % (start_region, target.node_id))
    start_count = len({node.node_id for node in nodes.values()
                       if node.region in START_REGIONS
                       and str(node.load_state) == "carrying"})
    target_count = len(carrying_slots)
    return {
        "ok": not issues and not not_evaluated,
        "structure_ok": not issues,
        "issues": issues,
        "node_count": len(nodes),
        "edge_count": len(edges),
        "start_class_nodes": start_count,
        "target_nodes": target_count,
        "routes": coverage,
        "not_evaluated": sorted(not_evaluated),
        "prior_gaps": list(graph.prior_gaps),
    }
