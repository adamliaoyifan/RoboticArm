"""Independent Python oracle for the free-space candidate query (F gate).

Nothing here imports or calls product code. The occupancy map is modeled
analytically from the fixture's ray list:

  - row ray (x-direction, fixed y and z at cell centers): every active cell
    x in [0, x_end) on that row's layer is free; (x_end, y, z) is occupied.
  - plane drop fan (one origin above, endpoints at cell centers of one z
    layer): each endpoint cell is occupied. Fan free marks are deliberately
    NOT modeled: every scene keeps fan travel inside row-swept columns or in
    regions whose support ratio is zero, so the modeled free set and the
    product's agree on every cell that can matter to a candidate.

The chamfer hull test is reimplemented from the geometry descriptor and the
candidate enumeration follows freespace_contract's frozen semantics with its
own numpy implementation (per-z layer prefix sums, not the product's prefix
scratch). Both sides are expected to agree exactly; any difference is a
finding, never rounded away.
"""
from __future__ import annotations

import json
import math

import numpy as np

SUPPORT_TOL_M = 0.05
MIN_SUPPORT_RATIO = 0.6
CLEARANCE_MARGIN_M = 0.03
SAME_LAYER_CLEARANCE_M = 0.05
EPS = 1e-9


class GridModel:
    def __init__(self, descriptor: dict, resolution: float):
        self.length = float(descriptor["length"])
        self.width = float(descriptor["width"])
        self.floor_z = float(descriptor["floor_z"])
        self.ceiling_z = float(descriptor["ceiling_z"])
        self.chamfer = descriptor.get("chamfer")
        self.res = float(resolution)
        self.shape = (
            int(math.ceil(self.length / self.res - EPS)),
            int(math.ceil(self.width / self.res - EPS)),
            int(math.ceil((self.ceiling_z - self.floor_z) / self.res - EPS)),
        )
        self.origin = (-self.length / 2, -self.width / 2, self.floor_z)
        self.free = np.zeros(self.shape, dtype=bool)
        self.occ = np.zeros(self.shape, dtype=bool)
        self.active = self._active_cells()

    def _active_cells(self) -> np.ndarray:
        sx, sy, sz = self.shape
        xs = self.origin[0] + (np.arange(sx) + 0.5) * self.res
        ys = self.origin[1] + (np.arange(sy) + 0.5) * self.res
        zs = self.origin[2] + (np.arange(sz) + 0.5) * self.res
        inside = (
            (np.abs(xs) <= self.length / 2 + EPS)[:, None, None]
            & (np.abs(ys) <= self.width / 2 + EPS)[None, :, None]
            & ((zs >= self.floor_z - EPS) & (zs <= self.ceiling_z + EPS))[None, None, :]
        )
        if self.chamfer:
            c = self.chamfer
            slope = (c["wall_y"] - c["floor_y"]) / (c["wall_z"] - self.floor_z)
            limit = c["floor_y"] + slope * (zs - self.floor_z) + EPS       # (sz,)
            above_wall = zs >= c["wall_z"] - EPS                            # (sz,)
            chamfer_ok = (ys[None, :, None] <= limit[None, None, :]) \
                | above_wall[None, None, :]
            inside &= chamfer_ok
        return inside

    # --- scene construction ------------------------------------------------
    def add_row_block(self, z: int, y_from: int, y_to: int, x_end: int) -> None:
        """Rows y in [y_from, y_to) at layer z: free up to x_end, occupied there."""
        self.free[:x_end, y_from:y_to, z] |= self.active[:x_end, y_from:y_to, z]
        if 0 <= x_end < self.shape[0] and self.active[x_end, y_from:y_to, z].any():
            columns = self.active[x_end, y_from:y_to, z]
            self.occ[x_end, y_from:y_to, z] |= columns

    def add_occupied_strip(self, z: int, y_from: int, y_to: int, x0: int, x1: int) -> None:
        """Ray per row stopping at EVERY column of [x0, x1): free before the
        strip, occupied inside it. Occupied always wins over an earlier free
        mark (one hit flips the cell), which apply_precedence() enforces."""
        self.free[:x0, y_from:y_to, z] |= self.active[:x0, y_from:y_to, z]
        block = self.active[x0:x1, y_from:y_to, z]
        self.occ[x0:x1, y_from:y_to, z] |= block

    def apply_precedence(self) -> None:
        """A hit after a miss leaves the cell occupied; effective free state."""
        self.free &= ~self.occ

    def reset(self) -> None:
        self.free[:] = False
        self.occ[:] = False

    # --- enumeration -------------------------------------------------------
    def tops(self) -> np.ndarray:
        """Highest occupied z per column, -1 where none (the product sentinel)."""
        flipped = self.occ[:, :, ::-1]
        has = flipped.any(axis=2)
        highest = self.shape[2] - 1 - np.argmax(flipped, axis=2)
        return np.where(has, highest, -1)

    def enumerate(self, box, yaws=(0, 90)):
        """Expected feasible candidates and the five rejection counters.

        Returns (candidates, counters): candidates as a sorted list of
        (yaw, cx, cy, cz, kind); counters with the product's rejection
        priority (out_of_domain, collision, clearance, unknown, support) and
        grid_total/feasible_total.
        """
        sx, sy, sz = self.shape
        res = self.res
        fx0 = int(math.ceil(box[0] / res - EPS))
        fy0 = int(math.ceil(box[1] / res - EPS))
        fz = int(math.ceil(box[2] / res - EPS))
        slab = int(math.ceil(SUPPORT_TOL_M / res - EPS))
        floor_cells = int(math.floor(SUPPORT_TOL_M / res + EPS))
        side = int(math.ceil(SAME_LAYER_CLEARANCE_M / res - EPS))
        top = self.tops()
        occ_pre = self._prefix(self.occ.astype(np.int32))
        free_pre = self._prefix(self.free.astype(np.int32))
        act_pre = self._prefix(self.active.astype(np.int32))
        # Laterally dilated occupancy: any occupied cell within `side` columns
        # (the same_layer clearance) blocks a footprint at overlapping z.
        occ_dil = self.occ.copy()
        padded = np.pad(self.occ, ((side, side), (side, side), (0, 0)))
        for dx in range(-side, side + 1):
            for dy in range(-side, side + 1):
                if dx == 0 and dy == 0:
                    continue
                occ_dil |= padded[side + dx:side + dx + sx, side + dy:side + dy + sy, :]
        dil_pre = self._prefix(occ_dil.astype(np.int32))
        candidates = []
        counters = dict(grid_total=0, feasible_total=0, rejected_out_of_domain=0,
                        rejected_collision=0, rejected_clearance=0, rejected_unknown=0,
                        rejected_support=0)
        for yaw in yaws:
            fx, fy = (fx0, fy0) if yaw == 0 else (fy0, fx0)
            volume = fx * fy * fz
            columns = fx * fy
            for cz in range(0, sz - fz + 1):
                # Layer-wide 2D sums for every footprint at once (sliding
                # window over the prefix sums).
                act_sum = self._window(act_pre, fx, fy, cz, cz + fz)
                occ_lat = self._window(dil_pre, fx, fy, cz, cz + fz)
                free_sum = self._window(free_pre, fx, fy, cz, cz + fz)
                low = max(0, cz - slab)
                supported = ((top >= low) & (top <= cz - 1)).astype(np.int32)
                sup_pre = self._prefix2(supported)
                sup_sum = self._window2(sup_pre, fx, fy)
                world_top = self.origin[2] + (cz + fz) * res
                clearance_ok = (self.ceiling_z - world_top) >= CLEARANCE_MARGIN_M - EPS
                floor_ok = cz <= floor_cells
                ood = act_sum != volume
                collision = occ_lat > 0
                unknown = free_sum != volume
                support_bad = (sup_sum / columns < MIN_SUPPORT_RATIO - EPS) if not floor_ok \
                    else np.zeros(ood.shape, dtype=bool)
                feasible = ~ood & ~collision
                if not clearance_ok:
                    below = feasible.copy()
                    feasible = np.zeros(ood.shape, dtype=bool)
                else:
                    below = feasible.copy()
                    feasible = feasible & ~unknown & ~support_bad
                counters["grid_total"] += int(ood.size)
                counters["rejected_out_of_domain"] += int(ood.sum())
                counters["rejected_collision"] += int((collision & ~ood).sum())
                if clearance_ok:
                    counters["rejected_clearance"] += 0
                    counters["rejected_unknown"] += int((unknown & below).sum())
                    counters["rejected_support"] += int((support_bad & below & ~unknown).sum())
                else:
                    # The ceiling-margin rejection outranks unknown/support.
                    counters["rejected_clearance"] += int(below.sum())
                    counters["rejected_unknown"] += 0
                    counters["rejected_support"] += 0
                counters["feasible_total"] += int(feasible.sum())
                for cx, cy in np.argwhere(feasible):
                    kind = 1 if sup_sum[cx, cy] > 0 else 2
                    candidates.append((yaw, int(cx), int(cy), cz, kind))
        candidates.sort()
        return candidates, counters

    @staticmethod
    def _prefix(array: np.ndarray) -> np.ndarray:
        padded = np.zeros((array.shape[0] + 1, array.shape[1] + 1, array.shape[2] + 1),
                          dtype=np.int64)
        padded[1:, 1:, 1:] = array.cumsum(0).cumsum(1).cumsum(2)
        return padded

    @staticmethod
    def _prefix2(array: np.ndarray) -> np.ndarray:
        padded = np.zeros((array.shape[0] + 1, array.shape[1] + 1), dtype=np.int64)
        padded[1:, 1:] = array.cumsum(0).cumsum(1)
        return padded

    @staticmethod
    def _window(pre: np.ndarray, fx: int, fy: int, z0: int, z1: int, pad: int = 0):
        """Sliding-window sums over footprints (fx, fy) for one z slab."""
        a = pre[:, :, z1] - pre[:, :, z0]
        return (a[fx:, fy:] - a[:-fx, fy:] - a[fx:, :-fy] + a[:-fx, :-fy])

    @staticmethod
    def _window2(pre: np.ndarray, fx: int, fy: int):
        return (pre[fx:, fy:] - pre[:-fx, fy:] - pre[fx:, :-fy] + pre[:-fx, :-fy])


def model_from_manifest(manifest: dict) -> GridModel:
    descriptor = json.loads(manifest["descriptor_json"]) if "descriptor_json" in manifest \
        else manifest["descriptor"]
    model = GridModel(descriptor, manifest["resolution"])
    for ray in manifest["rays"]:
        if ray["kind"] == "row_block":
            model.add_row_block(ray["z"], ray["y_from"], ray["y_to"], ray["x_end"])
        elif ray["kind"] == "occupied_strip":
            model.add_occupied_strip(ray["z"], ray["y_from"], ray["y_to"],
                                     ray["x0"], ray["x1"])
        else:
            raise ValueError("unknown ray kind %r" % (ray["kind"],))
    model.apply_precedence()
    return model
