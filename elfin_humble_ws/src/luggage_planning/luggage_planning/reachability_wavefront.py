"""Deterministic reachability-atlas wavefront helpers (no MoveIt)."""

from __future__ import division

import math
from collections import deque

UNKNOWN = 0
UNREACHABLE = 1
MARGINAL = 2
REACHABLE = 3


def grid_neighbors(cell, shape):
    """Return deterministic 6-connected spatial neighbors."""
    ix, iy, iz = cell
    nx, ny, nz = shape
    result = []
    for axis in range(3):
        for delta in (-1, 1):
            candidate = [ix, iy, iz]
            candidate[axis] += delta
            if (0 <= candidate[0] < nx and
                    0 <= candidate[1] < ny and
                    0 <= candidate[2] < nz):
                result.append(tuple(candidate))
    return tuple(sorted(result))


def opening_boundary_cells(shape, opening_axis, opening_sign):
    """Return sorted grid cells touching the configured opening face."""
    boundary_index = shape[opening_axis] - 1 if opening_sign > 0 else 0
    cells = []
    for ix in range(shape[0]):
        for iy in range(shape[1]):
            for iz in range(shape[2]):
                cell = (ix, iy, iz)
                if cell[opening_axis] == boundary_index:
                    cells.append(cell)
    return tuple(sorted(cells))


def deterministic_wavefront(shape, opening_axis, opening_sign, can_expand):
    """Traverse only cells connected to a successful opening boundary."""
    anchors = opening_boundary_cells(shape, opening_axis, opening_sign)
    anchor_set = set(anchors)
    queue = deque(anchors)
    queued = set(anchors)
    connected = set()
    order = []
    while queue:
        cell = queue.popleft()
        queued.discard(cell)
        predecessors = tuple(
            neighbor for neighbor in grid_neighbors(cell, shape)
            if neighbor in connected
        )
        if can_expand(cell, predecessors, cell in anchor_set):
            if cell not in connected:
                connected.add(cell)
                order.append(cell)
                for neighbor in grid_neighbors(cell, shape):
                    if neighbor not in connected and neighbor not in queued:
                        queue.append(neighbor)
                        queued.add(neighbor)
    return tuple(order)


def joint_interpolation_samples(start, goal, max_step):
    """Return deterministic interior samples for a local joint segment."""
    if len(start) != len(goal) or max_step <= 0.0:
        raise ValueError("joint vectors must match and max_step must be positive")
    max_delta = max(abs(float(b) - float(a)) for a, b in zip(start, goal))
    segments = max(1, int(math.ceil(max_delta / max_step)))
    samples = [
        tuple(float(a) + (float(b) - float(a)) * step / segments
              for a, b in zip(start, goal))
        for step in range(1, segments + 1)
    ]
    samples[-1] = tuple(float(value) for value in goal)
    return tuple(samples)


def joint_branch_distance(left, right):
    """Maximum per-joint distance, accounting for equivalent 2*pi wraps."""
    distances = []
    for a, b in zip(left, right):
        delta = float(a) - float(b)
        distances.append(abs(
            (delta + math.pi) % (2.0 * math.pi) - math.pi))
    return max(distances) if distances else 0.0


def select_distinct_branches(branches, limit, threshold):
    """Sort, deduplicate, and bound branch dictionaries deterministically."""
    ranked = sorted(
        branches,
        key=lambda branch: (
            -float(branch["margin"]),
            tuple(round(float(v), 10) for v in branch["transit"]),
            tuple(round(float(v), 10) for v in branch["contact"]),
        ),
    )
    selected = []
    for branch in ranked:
        duplicate = any(
            joint_branch_distance(branch["transit"], prior["transit"]) < threshold
            and joint_branch_distance(branch["contact"], prior["contact"]) < threshold
            for prior in selected
        )
        if not duplicate:
            selected.append(branch)
        if len(selected) >= limit:
            break
    return selected


def classify_cell(branches, indeterminate=False, marginal_margin=0.10):
    """Classify one cell conservatively from connected branch evidence."""
    if not branches:
        return UNKNOWN if indeterminate else UNREACHABLE
    if indeterminate or any(
            branch.get("repair", False) or
            float(branch["margin"]) < marginal_margin
            for branch in branches):
        return MARGINAL
    return REACHABLE
