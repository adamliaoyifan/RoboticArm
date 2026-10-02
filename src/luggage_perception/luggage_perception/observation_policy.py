#!/usr/bin/env python3
"""ST09-D: bounded observation policy — at most two prevalidated views.

ROS-free and pure: the policy consumes an ``ObservationRequirement`` (what
ST09-A wants decided), the current ``PlanningSnapshot`` and a set of view
candidates (portal aim, second visible region — the policy-level mirror of
the runtime ``cargo_view_sequence.VIEW_ORDER`` two-view sequence; no motion
requests are made here, ST09-I executes). It prevalidates each candidate
(TF freshness inside the frozen join window, target visibility through a
pinhole projection of the required cells) and predicts coverage.

The policy never imports GT or catalog geometry, and never promotes map
cells: coverage prediction is read-only; state changes belong to the
integrator (``cargo_view_integration.integrate_view_with_report``).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Sequence, Tuple

from luggage_perception.cargo_view_integration import (
    SNAPSHOT_JOIN_WINDOW_SEC,
)

# The frozen bounded-views rule: the portal aim first, then one second
# visible region — at most MAX_VIEWS prevalidated views per requirement.
VIEW_ORDER = ("portal_aim", "second_region")
MAX_VIEWS = 2

# Pinhole projection needs a target height the 2D cell grid cannot carry;
# floor-level observation is the bounded policy's default (the placement
# floor-prior band). Callers may override per requirement.
DEFAULT_ASSUME_Z_M = 0.05


@dataclass(frozen=True)
class ViewCandidate:
    """One camera viewpoint offered to the policy (container frame)."""

    name: str
    kind: str                      # "portal_aim" | "second_region"
    eye_xyz: Tuple[float, float, float]
    optical_axis: Tuple[float, float, float]   # unit, container frame
    up: Tuple[float, float, float] = (0.0, 0.0, 1.0)
    stamp_ns: int = 0
    tf_stamp_ns: int = 0           # stamp of the TF that produced the pose
    fx: float = 320.0
    fy: float = 320.0
    cx: float = 320.0
    cy: float = 180.0
    image_w: int = 640
    image_h: int = 360
    min_depth_m: float = 0.10
    max_depth_m: float = 3.00


@dataclass(frozen=True)
class PrevalidatedView:
    """A view that passed the gates, with its predicted coverage."""

    candidate: ViewCandidate
    covered_cells: Tuple[Tuple[int, int], ...]
    uncovered_cells: Tuple[Tuple[int, int], ...]

    @property
    def coverage_ratio(self) -> float:
        total = len(self.covered_cells) + len(self.uncovered_cells)
        return (float(len(self.covered_cells)) / total) if total else 0.0


@dataclass(frozen=True)
class ViewPlanResult:
    """The bounded plan: <= MAX_VIEWS views plus why the rest were dropped."""

    views: Tuple[PrevalidatedView, ...]
    rejected: Tuple[Tuple[str, str], ...]     # (view name, reason string)
    required_cells: Tuple[Tuple[int, int], ...]
    still_unknown_cells: Tuple[Tuple[int, int], ...]


def _normalize(vector):
    norm = math.sqrt(sum(v * v for v in vector))
    if norm <= 0.0:
        raise ValueError("zero-length vector")
    return tuple(v / norm for v in vector)


def _camera_basis(view):
    """Camera axes in container frame: z along the optical axis."""
    z = _normalize(view.optical_axis)
    up = _normalize(view.up)
    x = _normalize((
        up[1] * z[2] - up[2] * z[1],
        up[2] * z[0] - up[0] * z[2],
        up[0] * z[1] - up[1] * z[0]))
    y = (
        z[1] * x[2] - z[2] * x[1],
        z[2] * x[0] - z[0] * x[2],
        z[0] * x[1] - z[1] * x[0])
    return x, y, z


def project_cell(view, cell, snapshot, assume_z):
    """Pinhole-project one cell centre; None when outside the frustum.

    Returns the pixel ``(u, v)`` for cells inside the image and depth
    range. Pure geometry from the candidate's own intrinsics — no GT, no
    catalog.
    """
    resolution = float(snapshot.resolution_m)
    origin_x, origin_y = snapshot.origin_xy
    world = (
        origin_x + (cell[0] + 0.5) * resolution,
        origin_y + (cell[1] + 0.5) * resolution,
        float(assume_z),
    )
    delta = tuple(world[i] - view.eye_xyz[i] for i in range(3))
    x_axis, y_axis, z_axis = _camera_basis(view)
    z_cam = sum(delta[i] * z_axis[i] for i in range(3))
    if z_cam < view.min_depth_m or z_cam > view.max_depth_m:
        return None
    x_cam = sum(delta[i] * x_axis[i] for i in range(3))
    y_cam = sum(delta[i] * y_axis[i] for i in range(3))
    u = view.fx * (x_cam / z_cam) + view.cx
    v = -view.fy * (y_cam / z_cam) + view.cy
    if not (0.0 <= u <= float(view.image_w) and 0.0 <= v <= float(view.image_h)):
        return None
    return (u, v)


def _tf_fresh(view, join_window_s):
    skew = abs(int(view.stamp_ns) - int(view.tf_stamp_ns))
    return skew <= int(join_window_s * 1e9)


def plan_views(requirement, snapshot, candidates,
               join_window_s=SNAPSHOT_JOIN_WINDOW_SEC,
               max_views=MAX_VIEWS, assume_z=DEFAULT_ASSUME_Z_M):
    """Select at most ``max_views`` prevalidated views for the requirement.

    Selection order is the frozen two-view order: every ``portal_aim``
    candidate is considered before any ``second_region`` candidate (input
    order within a kind). A view is prevalidated when its TF is fresh and
    it sees at least one required cell; among a kind the fullest predicted
    coverage wins. Cells already decided (free/occupied on the snapshot)
    are not targets — only the requirement's still-unknown cells matter,
    plus its footprint/corridor for the coverage trace.
    """
    required = sorted(set(requirement.footprint_cells)
                      | set(requirement.corridor_cells))
    targets = [cell for cell in required
               if snapshot.state_at(cell[0], cell[1]).value == "unknown"]
    if not targets:
        targets = sorted(set(requirement.missing_cells))
    ranked = []
    rejected = []
    by_kind = {kind: [] for kind in VIEW_ORDER}
    for candidate in candidates:
        if candidate.kind not in by_kind:
            rejected.append((candidate.name, "unknown_view_kind"))
            continue
        by_kind[candidate.kind].append(candidate)
    selected = []
    for kind in VIEW_ORDER:
        best = None
        for candidate in by_kind[kind]:
            if not _tf_fresh(candidate, join_window_s):
                rejected.append((candidate.name, "VIEW_TF_STALE"))
                continue
            covered = []
            uncovered = []
            for cell in targets:
                if project_cell(candidate, cell, snapshot, assume_z) is None:
                    uncovered.append(cell)
                else:
                    covered.append(cell)
            if not covered:
                rejected.append((candidate.name, "VIEW_EMPTY"))
                continue
            prevalidated = PrevalidatedView(
                candidate=candidate,
                covered_cells=tuple(sorted(covered)),
                uncovered_cells=tuple(sorted(uncovered)))
            if best is None or (prevalidated.coverage_ratio
                                > best.coverage_ratio):
                best = prevalidated
        if best is not None and len(selected) < int(max_views):
            selected.append(best)
    combined = set()
    for view in selected:
        combined |= set(view.covered_cells)
    still_unknown = tuple(cell for cell in targets if cell not in combined)
    return ViewPlanResult(
        views=tuple(selected),
        rejected=tuple(rejected),
        required_cells=tuple(required),
        still_unknown_cells=still_unknown,
    )


def coverage_after_integration(snapshot_after, requirement):
    """Post-integration truth: which required cells are still unknown."""
    required = sorted(set(requirement.footprint_cells)
                      | set(requirement.corridor_cells))
    return tuple(
        cell for cell in required
        if snapshot_after.state_at(cell[0], cell[1]).value == "unknown")
