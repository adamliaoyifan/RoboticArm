"""Deterministic reachability-atlas wavefront helpers (no MoveIt)."""

from __future__ import division

import math
from collections import deque

UNKNOWN = 0
UNREACHABLE = 1
MARGINAL = 2
REACHABLE = 3

# Same-column carry/retreat lift. Low contacts keep the nominal 0.30 m.
# The cap is the last tool-down height measured clear of the lid (>50 mm);
# 1.55–1.70 m has no sample, so the cap is not raised to the interfering pose.
NOMINAL_LIFT_CLEARANCE = 0.30
SAFE_TOOL_Z = 1.55
# Tool-down contact at this height already has the elbow in the lid.
# Same-column carry is capped lower; contacts at or above this are not sampled.
INTERFERENCE_TOOL_Z = 1.70
MIN_LIFT = 0.06

BRANCH_NONE = 0
BRANCH_VERTICAL = 1
BRANCH_INTERNAL_LIFT = 2

_AXIS_ALIGN_EPS = 1e-6


def contact_below_interference(contact_z, interference_z=INTERFERENCE_TOOL_Z):
    """False when a tool-down contact is already in the measured elbow/lid band."""
    return float(contact_z) < float(interference_z)


def adaptive_lift_clearance(contact_z, nominal=NOMINAL_LIFT_CLEARANCE,
                            safe_tool_z=SAFE_TOOL_Z, min_lift=MIN_LIFT):
    """Vertical lift that keeps the tool at or below ``safe_tool_z``.

    Returns ``None`` when the headroom above ``contact_z`` is below
    ``min_lift``: a same-column carry would either be shorter than a real
    lift or would enter the elbow/lid band. Heights are in one frame;
    callers convert ``safe_tool_z`` before calling.
    """
    headroom = float(safe_tool_z) - float(contact_z)
    if headroom < float(min_lift):
        return None
    return min(float(nominal), headroom)


def container_tool_z_in_frame(safe_tool_z, origin_z, rotation_rpy):
    """Map a container_link tool-z cap into the retreat frame.

    ``origin_z`` and ``rotation_rpy`` describe container_link in that frame.
    Roll and pitch must be zero so container z and the retreat z axis match.
    A pitched container is refused instead of treating ``safe_tool_z`` as
    already expressed in the retreat frame.
    """
    roll = float(rotation_rpy[0])
    pitch = float(rotation_rpy[1])
    if abs(roll) > _AXIS_ALIGN_EPS or abs(pitch) > _AXIS_ALIGN_EPS:
        raise ValueError(
            "container z is not aligned with the retreat frame (rpy=%s)"
            % (tuple(float(v) for v in rotation_rpy),))
    return float(origin_z) + float(safe_tool_z)


def retreat_height(contact_z, corridor_clearance, safe_tool_z):
    """Capped retreat tool z and how much of the requested lift was cut.

    The retreat never descends below ``contact_z``. A contact already above
    the cap stays put; the whole requested lift is then the truncation.
    """
    contact_z = float(contact_z)
    requested = contact_z + float(corridor_clearance)
    capped = min(requested, float(safe_tool_z))
    capped = max(capped, contact_z)
    return capped, max(0.0, requested - capped)


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
