"""Request-scoped support/window index for structural candidate search.

G3 STRUCTURAL-CORE (plan 33ba6745 §3): build the per-request sliding
window statistics ONCE, never per candidate. The index owns:

- the exact solver window semantics for both footprint classes, via the
  solver's own ``_window_stats`` (memoized per window so structural and
  fallback paths share one evaluation);
- a vectorized windowed-maximum precompute (numpy stride tricks, no
  scipy) for the cheap top-clearance prefilter -- bit-identical to the
  cell-scan maximum (``max`` over the same floats);
- request identity (map geometry hash, box size, gate policy digest).

ROS-free, deterministic, no cross-request state. The index never mutates
the map and never quantizes heights; layer labels live in consumers.
"""

import hashlib
import json
import math

from luggage_packing.placement_solver import (
    _footprint_cells,
    _snap_yaws,
    _window_stats,
)


class SupportIndex:
    """Read-only window statistics over one surface map / one box size."""

    def __init__(self, surface_map, box_size, allowed_yaws, gate_policy):
        self._map = surface_map
        self._box_size = [float(v) for v in box_size]
        self._policy = dict(gate_policy)
        self.resolution = float(surface_map["resolution"])
        self.nx = int(surface_map["nx"])
        self.ny = int(surface_map["ny"])
        self.inner_size = [float(v) for v in surface_map["inner_size"]]
        self._stats_memo = {}
        self._wmax = {}
        self._cells = {}
        for box_yaw, rotated in _snap_yaws(allowed_yaws):
            footprint_l, footprint_w = (
                (self._box_size[1], self._box_size[0]) if rotated
                else (self._box_size[0], self._box_size[1]))
            cells = _footprint_cells(footprint_l, footprint_w, self.resolution)
            cls = 1 if rotated else 0
            self._cells[cls] = cells + (footprint_l, footprint_w, box_yaw)

    # -- identity ------------------------------------------------------

    @property
    def identity(self):
        payload = {
            "geometry_hash": str(self._map.get("geometry_hash") or ""),
            "resolution": self.resolution,
            "nx": self.nx,
            "ny": self.ny,
            "box_size": self._box_size,
            "policy": {
                key: self._policy[key]
                for key in sorted(self._policy)
                if isinstance(self._policy[key], (int, float, str, bool))
            },
            "classes": sorted(self._cells),
        }
        digest = hashlib.sha256(
            json.dumps(payload, sort_keys=True).encode("utf-8"))
        return {
            "map_geometry_hash": payload["geometry_hash"],
            "box_size": self._box_size,
            "policy_digest": digest.hexdigest()[:16],
        }

    # -- geometry ------------------------------------------------------

    def classes(self):
        return sorted(self._cells)

    def window_cells(self, cls):
        """(cells_x, cells_y) for one footprint class."""
        return self._cells[cls][0], self._cells[cls][1]

    def footprint(self, cls):
        """(footprint_l, footprint_w, box_yaw) for one class."""
        return self._cells[cls][2], self._cells[cls][3], self._cells[cls][4]

    def window_origin_range(self, cls):
        cells_x, cells_y = self.window_cells(cls)
        return range(0, self.nx - cells_x + 1), range(0, self.ny - cells_y + 1)

    def window_center(self, cls, ix0, iy0):
        cells_x, cells_y = self.window_cells(cls)
        half_l = 0.5 * self.inner_size[0]
        half_w = 0.5 * self.inner_size[1]
        return (
            -half_l + (ix0 + cells_x * 0.5) * self.resolution,
            -half_w + (iy0 + cells_y * 0.5) * self.resolution,
        )

    # -- statistics ----------------------------------------------------

    def window_peak_prefilter(self, cls):
        """Vectorized windowed max for the clearance prefilter.

        Bit-identical to the per-window ``max_h`` of ``_window_stats``
        (``max`` over the same float values). Cached per class.
        """
        if cls in self._wmax:
            return self._wmax[cls]
        import numpy as np

        cells_x, cells_y = self.window_cells(cls)
        heights = np.asarray(self._map["height"], dtype=float)
        if cells_x == 1 and cells_y == 1:
            wmax = heights
        else:
            windows = np.lib.stride_tricks.sliding_window_view(
                heights, (cells_x, cells_y))
            wmax = windows.max(axis=(2, 3))
        self._wmax[cls] = wmax
        return wmax

    def window_stats(self, cls, ix0, iy0):
        """Exact solver window stats, memoized (shared by every path)."""
        key = (cls, ix0, iy0)
        cached = self._stats_memo.get(key)
        if cached is not None:
            return cached
        cells_x, cells_y = self.window_cells(cls)
        stats = _window_stats(self._map, ix0, iy0, cells_x, cells_y)
        self._stats_memo[key] = stats
        return stats

    def stats_memo_size(self):
        return len(self._stats_memo)


def build_support_index(surface_map, box_size, allowed_yaws, gate_policy):
    """``SupportIndex.build`` entry from plan §7."""
    return SupportIndex(surface_map, box_size, allowed_yaws, gate_policy)


def plateau_components(surface_map, support_tol=0.05, min_cells=4):
    """Rectangular plateaus of the measured map, scan-order deterministic.

    A plateau is a 4-connected component of cells sharing one exact
    height above ``support_tol``. Returns list of dicts
    ``{x0, y0, x1, y1 (cell idx, inclusive), z_top_rel, cells}`` in
    deterministic scan order (ix then iy). Used offline as the neighbor
    source for structural edge candidates and for the adjacency term:
    the measured top is the plateau height; unknown gaps are not filled
    and the floor (height ~ 0) is never a plateau.
    """
    heights = surface_map["height"]
    nx, ny = int(surface_map["nx"]), int(surface_map["ny"])
    visited = [[False] * ny for _ in range(nx)]
    out = []
    for ix in range(nx):
        for iy in range(ny):
            if visited[ix][iy]:
                continue
            level = heights[ix][iy]
            visited[ix][iy] = True
            if level <= support_tol:
                continue
            stack = [(ix, iy)]
            cells = []
            while stack:
                cx, cy = stack.pop()
                cells.append((cx, cy))
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    wx, wy = cx + dx, cy + dy
                    if (0 <= wx < nx and 0 <= wy < ny and not visited[wx][wy]
                            and heights[wx][wy] == level):
                        visited[wx][wy] = True
                        stack.append((wx, wy))
            if len(cells) < min_cells:
                continue
            xs = [c[0] for c in cells]
            ys = [c[1] for c in cells]
            out.append({
                "ix0": min(xs), "iy0": min(ys),
                "ix1": max(xs), "iy1": max(ys),
                "z_top_rel": float(level),
                "n_cells": len(cells),
            })
    return out


def plateau_rect_m(plateau, surface_map):
    """Container-frame XY rect [x0, y0, x1, y1] of one plateau."""
    res = float(surface_map["resolution"])
    half_l = 0.5 * float(surface_map["inner_size"][0])
    half_w = 0.5 * float(surface_map["inner_size"][1])
    return [
        -half_l + plateau["ix0"] * res,
        -half_w + plateau["iy0"] * res,
        -half_l + (plateau["ix1"] + 1) * res,
        -half_w + (plateau["iy1"] + 1) * res,
    ]


def structural_neighbor_records(surface_map, support_tol=0.05):
    """Neighbor records for ranking/proposal sources, from the map only."""
    res = float(surface_map["resolution"])
    floor_z = float(surface_map.get("floor_z", 0.0))
    floor_container_z = (float(surface_map["center_base"][2])
                         - 0.5 * float(surface_map["inner_size"][2]))
    records = []
    for plateau in plateau_components(surface_map, support_tol=support_tol):
        rect = plateau_rect_m(plateau, surface_map)
        records.append({
            "cx": 0.5 * (rect[0] + rect[2]),
            "cy": 0.5 * (rect[1] + rect[3]),
            "half_l": 0.5 * (rect[2] - rect[0]),
            "half_w": 0.5 * (rect[3] - rect[1]),
            "yaw": 0.0,
            "top_z": floor_container_z + plateau["z_top_rel"],
            "source": "map_plateau",
        })
    return records
