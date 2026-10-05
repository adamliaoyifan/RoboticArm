#!/usr/bin/env python3
"""Diverse shortest paths on a reachable-cell graph.

Three edge-cost profiles (short, high, clear) share one graph. A later
profile pays extra for edges an earlier path already used, and a path that
overlaps a kept path is dropped. String-pulling deletes a cell only when the
chord is free, joint-continuous, and no more expensive under that profile.
"""

from __future__ import division

import heapq
import math

from luggage_planning.joint_path import max_joint_step


PROFILES = ("short", "high", "clear")


class GridPath(object):
    def __init__(self, profile, nodes):
        self.profile = str(profile)
        self.nodes = list(nodes)

    @property
    def cells(self):
        return [node.index for node in self.nodes]

    @property
    def waypoints(self):
        return [node.center for node in self.nodes]

    @property
    def seeds(self):
        return [list(node.seed) for node in self.nodes]


def _dist(a, b):
    return math.sqrt(sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)))


def _inside(atlas, xyz):
    origin = [float(v) for v in atlas.meta["grid"]["origin"]]
    resolution = float(atlas.resolution)
    nx, ny, nz, _nyaw = atlas.grid_size
    spans = (nx, ny, nz)
    for index in range(3):
        low = origin[index] - 1e-6
        high = origin[index] + spans[index] * resolution + 1e-6
        if xyz[index] < low or xyz[index] > high:
            return False
    return True


def _entry_nodes(graph):
    if not graph.nodes:
        return []
    side = getattr(graph, "opening_side", "negative_x")
    axis = {"x": 0, "y": 1}.get(side.split("_")[-1], 0)
    sign = -1 if side.startswith("negative") else 1
    values = [node.index[axis] for node in graph.nodes.values()]
    target = min(values) if sign < 0 else max(values)
    return [node for node in graph.nodes.values() if node.index[axis] == target]


def _nearest(nodes, xyz):
    best = None
    best_dist = None
    for node in nodes:
        distance = _dist(node.center, xyz)
        if best_dist is None or distance < best_dist:
            best = node
            best_dist = distance
    return best, best_dist


def edge_cost(profile, left, right, z_max, high_z_gain):
    length = max(_dist(left.center, right.center), 1e-6)
    if profile == "high":
        z_mid = 0.5 * (left.center[2] + right.center[2])
        return length * (1.0 + float(high_z_gain) * max(0.0, z_max - z_mid))
    if profile == "clear":
        gap = min(left.clearance, right.clearance)
        return length * (1.0 + 1.0 / max(gap, 0.02))
    return length


def _dijkstra(graph, start, goal, profile, penalized, z_max, high_z_gain,
              diversity_penalty):
    start_idx = start.index
    goal_idx = goal.index
    best = {start_idx: 0.0}
    prev = {start_idx: None}
    heap = [(0.0, start_idx)]
    while heap:
        cost, current = heapq.heappop(heap)
        if cost > best.get(current, float("inf")):
            continue
        if current == goal_idx:
            break
        for other, _length in graph.adjacency.get(current, []):
            extra = edge_cost(
                profile, graph.nodes[current], graph.nodes[other],
                z_max, high_z_gain)
            if (current, other) in penalized or (other, current) in penalized:
                extra *= 1.0 + float(diversity_penalty)
            total = cost + extra
            if total + 1e-12 < best.get(other, float("inf")):
                best[other] = total
                prev[other] = current
                heapq.heappush(heap, (total, other))
    if goal_idx not in prev:
        return None
    chain = []
    cursor = goal_idx
    seen = set()
    while cursor is not None:
        if cursor in seen:
            return None
        seen.add(cursor)
        chain.append(graph.nodes[cursor])
        cursor = prev[cursor]
    chain.reverse()
    return chain


def _jaccard(left, right):
    a = set(left)
    b = set(right)
    if not a and not b:
        return 1.0
    return float(len(a & b)) / float(len(a | b))


def _shortcut(nodes, profile, z_max, high_z_gain, continuity, chord_free):
    kept = list(nodes)
    changed = True
    while changed and len(kept) > 2:
        changed = False
        index = 0
        while index < len(kept) - 2:
            left, mid, right = kept[index], kept[index + 1], kept[index + 2]
            direct = edge_cost(profile, left, right, z_max, high_z_gain)
            via = (
                edge_cost(profile, left, mid, z_max, high_z_gain)
                + edge_cost(profile, mid, right, z_max, high_z_gain))
            joint_ok = max_joint_step(left.seed, right.seed) <= float(continuity) + 1e-9
            if joint_ok and direct <= via + 1e-9 and chord_free(left, right):
                del kept[index + 1]
                changed = True
            else:
                index += 1
    return kept


def search_paths(graph, atlas, start_xyz, goal_xyz, continuity_rad=1.2,
                 high_z_gain=8.0, diversity_penalty=3.0, overlap_max=0.7,
                 chord_free=None, snap_factor=1.5):
    """Return up to three non-overlapping ``GridPath`` values.

    ``chord_free(left_node, right_node)`` is the occupancy test for a
    shortcut. It defaults to always free.
    """
    if graph is None or graph.disabled_reason or len(graph) == 0:
        return []
    if not _inside(atlas, start_xyz):
        pool = _entry_nodes(graph)
    else:
        pool = list(graph.nodes.values())
    start, _start_dist = _nearest(pool, start_xyz)
    goal, goal_dist = _nearest(graph.nodes.values(), goal_xyz)
    if start is None or goal is None:
        return []
    limit = float(snap_factor) * float(atlas.resolution)
    if goal_dist is not None and goal_dist > limit:
        return []
    z_max = max(node.center[2] for node in graph.nodes.values())
    free = chord_free or (lambda _left, _right: True)
    penalized = set()
    kept = []
    for profile in PROFILES:
        chain = _dijkstra(
            graph, start, goal, profile, penalized, z_max, high_z_gain,
            diversity_penalty)
        if not chain:
            continue
        chain = _shortcut(
            chain, profile, z_max, high_z_gain, continuity_rad, free)
        path = GridPath(profile, chain)
        if any(_jaccard(path.cells, prior.cells) > float(overlap_max) for prior in kept):
            for left, right in zip(path.cells, path.cells[1:]):
                penalized.add((left, right))
            continue
        kept.append(path)
        for left, right in zip(path.cells, path.cells[1:]):
            penalized.add((left, right))
    return kept
