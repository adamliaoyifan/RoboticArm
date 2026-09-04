#!/usr/bin/env python3
"""FreeSpaceModel: dual-source free-space model + vectorized candidate gen.

P1 of the online-packing redesign (design §4, §5). Pure Python + numpy, no
ROS / scipy / cv2 (§7.5). Replaces ``placement_solver.generate_candidates`` with:

  - a *persistent* model: heightmap ``H`` + per-column ``state`` + placed-box
    AABB table + LBCP support polygons, initialized with the floor as
    known-free prior (§4.2.2 -- "floor exists" is geometric, not perceptual);
  - vectorized candidate generation via ``sliding_window_view`` (§5.1);
  - LBCP stability (CoG-in-support-polygon, §5.3) replacing ``support_ratio``;
  - the §4.2.2 peak-based gate: floor-relative ``peak≈0`` unobserved -> ``floor_prior`` allow;
    ``peak>0`` with unobserved in footprint -> ``unknown_above_floor`` reject.

The model is a drop-in source for ``placement_planner_node``: ``candidates()``
returns the same dict shape as ``placement_solver``.
"""

from __future__ import division

import math

import numpy as np
from numpy.lib.stride_tricks import sliding_window_view

# Column state codes.
STATE_UNKNOWN = 0       # unobserved column (above-floor unseen)
STATE_FREE = 1          # observed empty (sensor)
STATE_OCCUPIED = 2      # known obstacle (placed box)
STATE_FLOOR_PRIOR = 3   # a-priori floor (known-free from scene_tf)

# Support source labels (candidate field).
SRC_SENSOR = "sensor"
SRC_GEOMETRY = "geometry"
SRC_FLOOR_PRIOR = "floor_prior"

_PEAK_FLOOR_TOL = 1e-3   # peak below this counts as "on the floor"
_Z_LEVEL_TOL = 1e-3      # LBCP z-level match tolerance
_LEVEL_QUANT = 0.01      # peak quantization for candidate-pool stratification [m]


def stratified_pool(results, top_n, per_level_quota=4):
    """Fill the candidate pool level-by-level instead of by a global sort.

    Candidates on a placed-box top carry exact geometry and a contained-support
    flag, so a single global sort lets one support level consume every slot and
    the floor layer never reaches the scorer at all. Taking a quota per support
    height keeps every reachable layer represented; ranking stays the scorer's
    job.
    """
    levels = {}
    for cand in results:
        key = int(round(float(cand["peak"]) / _LEVEL_QUANT))
        levels.setdefault(key, []).append(cand)
    ordered_keys = sorted(levels)
    quota = max(1, int(per_level_quota))
    pool = []
    round_index = 0
    while len(pool) < top_n:
        added = False
        for key in ordered_keys:
            chunk = levels[key][round_index * quota:(round_index + 1) * quota]
            if not chunk:
                continue
            pool.extend(chunk)
            added = True
            if len(pool) >= top_n:
                break
        if not added:
            break
        round_index += 1
    return pool[:top_n]


# --------------------------------------------------------------------------- #
# Vectorized window ops (§5.1)
# --------------------------------------------------------------------------- #

def window_max(H, kx, ky):
    """(nx,ny) -> (nx-kx+1, ny-ky+1) per-window peak height. Separable, C-level."""
    a = sliding_window_view(H, kx, axis=0).max(axis=-1)   # (nx-kx+1, ny)
    return sliding_window_view(a, ky, axis=1).max(axis=-1)


def window_sum(mask, kx, ky):
    """Integral-image window sum for a bool/int mask. O(nx*ny)."""
    s = np.zeros((mask.shape[0] + 1, mask.shape[1] + 1), dtype=np.int64)
    s[1:, 1:] = mask.cumsum(0).cumsum(1)
    return s[kx:, ky:] - s[:-kx, ky:] - s[kx:, :-ky] + s[:-kx, :-ky]


# --------------------------------------------------------------------------- #
# LBCP geometry helpers (axis-aligned rects, pure Python) -- §5.3
# --------------------------------------------------------------------------- #

def rect_intersect_corners(ax0, ay0, ax1, ay1, bx0, by0, bx1, by1):
    """Corners of the intersection of two axis-aligned rects (empty list if none)."""
    ix0, iy0 = max(ax0, bx0), max(ay0, by0)
    ix1, iy1 = min(ax1, bx1), min(ay1, by1)
    if ix1 <= ix0 or iy1 <= iy0:
        return []
    return [(ix0, iy0), (ix1, iy0), (ix1, iy1), (ix0, iy1)]


def convex_hull(points):
    """Monotone-chain 2D convex hull. Returns CCW-ordered hull vertices."""
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def point_in_convex(hull, point):
    """Point-in-convex-polygon (hull CCW-ordered). Edge-inclusive."""
    if len(hull) < 3:
        return False
    px, py = point
    n = len(hull)
    sign = 0
    for i in range(n):
        ax, ay = hull[i]
        bx, by = hull[(i + 1) % n]
        cross = (bx - ax) * (py - ay) - (by - ay) * (px - ax)
        if cross == 0:
            continue
        cur = 1 if cross > 0 else -1
        if sign == 0:
            sign = cur
        elif sign != cur:
            return False
    return True


# --------------------------------------------------------------------------- #
# FreeSpaceModel
# --------------------------------------------------------------------------- #

class FreeSpaceModel(object):
    """Persistent free-space model: floor-prior init + vectorized candidates.

    ``center_base`` is the container interior volume center in elfin_base_link
    (i.e. ``container_interior_center_in_base_link``), matching surface_map_2d.
    Local frame: X/Y in [-inner/2, +inner/2], Z in [-inner_h/2, +inner_h/2]
    (volume-center); H is stored relative to the floor (>=0).
    """

    def __init__(self, inner_size, center_base, yaw=0.0, resolution=0.05,
                 clearance_margin=0.03, cog_margin_ratio=0.15,
                 floor_prior=True, floor_z=0.0, boundary_margin=0.05):
        self.inner_l, self.inner_w, self.inner_h = [float(v) for v in inner_size]
        self.center_base = [float(v) for v in center_base]
        self.yaw = float(yaw)
        self.resolution = float(resolution)
        self.clearance_margin = float(clearance_margin)
        self.cog_margin_ratio = float(cog_margin_ratio)
        self.floor_z = float(floor_z)
        self.boundary_margin = max(0.0, float(boundary_margin))
        self.nx = max(1, int(round(self.inner_l / self.resolution)))
        self.ny = max(1, int(round(self.inner_w / self.resolution)))

        init_state = STATE_FLOOR_PRIOR if floor_prior else STATE_UNKNOWN
        self.H = np.zeros((self.nx, self.ny), dtype=np.float32)
        self.state = np.full((self.nx, self.ny), init_state, dtype=np.uint8)
        self.boxes = []  # AABB dicts in local (volume-center) frame
        # LBCP list: (rect (x0,y0,x1,y1), z_level). Floor is LBCP_0 (z=0).
        self.lbcp = [((-self.inner_l * 0.5, -self.inner_w * 0.5,
                       self.inner_l * 0.5, self.inner_w * 0.5), 0.0)]
        self.revision = 0

    # ---- coordinate helpers ----
    def _local_to_base(self, lx, ly, lz_vol):
        """Volume-center local -> base. lz_vol is volume-center Z."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return [self.center_base[0] + c * lx - s * ly,
                self.center_base[1] + s * lx + c * ly,
                self.center_base[2] + lz_vol]

    def _cell_center(self, ix0, iy0, kx, ky):
        lx = -self.inner_l * 0.5 + (ix0 + kx * 0.5) * self.resolution
        ly = -self.inner_w * 0.5 + (iy0 + ky * 0.5) * self.resolution
        return lx, ly

    # ---- mutation (incremental, §7.3) ----
    def snapshot(self):
        """Non-mutating copy for rollout what-if (H/state arrays copied)."""
        import copy
        m = copy.copy(self)
        m.H = self.H.copy()
        m.state = self.state.copy()
        m.boxes = list(self.boxes)
        m.lbcp = list(self.lbcp)
        return m

    def add_placed_box(self, center_local, size, load_bearable=True):
        """Mark a placed box: AABB table + heightmap + LBCP top (incremental)."""
        bl, bw, bh = [float(v) for v in size]
        cx, cy, cz = [float(v) for v in center_local]
        # center_local is volume-center frame; convert z to floor-relative.
        z_floor = cz + self.inner_h * 0.5 - self.floor_z
        x0, x1 = cx - bl * 0.5, cx + bl * 0.5
        y0, y1 = cy - bw * 0.5, cy + bw * 0.5
        z0, z1 = z_floor - bh * 0.5, z_floor + bh * 0.5
        box = {"x0": x0, "x1": x1, "y0": y0, "y1": y1,
               "z0": z0, "z1": z1, "size": [bl, bw, bh],
               "load_bearable": load_bearable}
        self.boxes.append(box)
        ix0 = max(0, int((x0 + self.inner_l * 0.5) / self.resolution))
        ix1 = min(self.nx, int(math.ceil((x1 + self.inner_l * 0.5) / self.resolution)))
        iy0 = max(0, int((y0 + self.inner_w * 0.5) / self.resolution))
        iy1 = min(self.ny, int(math.ceil((y1 + self.inner_w * 0.5) / self.resolution)))
        if ix1 > ix0 and iy1 > iy0:
            self.H[ix0:ix1, iy0:iy1] = np.maximum(self.H[ix0:ix1, iy0:iy1], z1)
            self.state[ix0:ix1, iy0:iy1] = STATE_OCCUPIED
        if load_bearable:
            self.lbcp.append(((x0, y0, x1, y1), z1))
        self.revision += 1
        return box

    def merge_surface_2d(self, surface_map):
        """Merge a sensor heightmap (surface_map_2d contract) into the model.

        Observed-free columns -> STATE_FREE; occupied -> STATE_OCCUPIED at the
        observed top. Unknown columns are left as-is (floor_prior preserved).
        """
        res = float(surface_map["resolution"])
        if abs(res - self.resolution) > 1e-6:
            return  # resolution mismatch -- skip (P1: no resampling yet)
        height = surface_map["height"]
        state = surface_map["state"]
        confidence = surface_map.get("confidence")
        for ix in range(min(self.nx, len(height))):
            for iy in range(min(self.ny, len(height[ix]))):
                st = state[ix][iy]
                # surface_map_2d height is an absolute elevation above
                # container-link Z=0; FreeSpaceModel H is floor-relative.
                h = max(0.0, float(height[ix][iy]) - self.floor_z)
                if st == "free":
                    self.state[ix, iy] = STATE_FREE
                    # free column: floor-level support observed
                elif st == "occupied":
                    if (
                            confidence
                            and confidence[ix][iy] == "geometry"
                            and self.state[ix, iy] == STATE_OCCUPIED):
                        # Exact committed AABB/LBCP already populated this
                        # column. Do not replace its top with voxel-ceil height.
                        continue
                    if h > self.H[ix, iy]:
                        self.H[ix, iy] = h
                    self.state[ix, iy] = STATE_OCCUPIED
                # "unknown" -> leave current (floor_prior or unknown)
        self.revision += 1

    # ---- LBCP stability (§5.3) ----
    def is_stable(self, lx, ly, size, peak):
        """CoG (with uncertainty margin) inside the support polygon -> stable."""
        bl, bw = size[0], size[1]
        fx0, fx1 = lx - bl * 0.5, lx + bl * 0.5
        fy0, fy1 = ly - bw * 0.5, ly + bw * 0.5
        contacts = []
        for (rx0, ry0, rx1, ry1), z in self.lbcp:
            if abs(z - peak) > _Z_LEVEL_TOL:
                continue
            contacts.extend(rect_intersect_corners(fx0, fy0, fx1, fy1,
                                                   rx0, ry0, rx1, ry1))
        if not contacts:
            return False
        hull = convex_hull(contacts)
        if len(hull) < 3:
            return False
        margin = self.cog_margin_ratio * min(bl, bw)
        for dx in (-margin, margin):
            for dy in (-margin, margin):
                if not point_in_convex(hull, (lx + dx, ly + dy)):
                    return False
        return True

    def has_full_support(self, lx, ly, size, peak, tolerance=0.005):
        """Conservative hard gate: one LBCP contains the whole footprint."""
        half_l, half_w = size[0] * 0.5, size[1] * 0.5
        fx0, fx1 = lx - half_l, lx + half_l
        fy0, fy1 = ly - half_w, ly + half_w
        for (x0, y0, x1, y1), z in self.lbcp:
            if abs(z - peak) > _Z_LEVEL_TOL:
                continue
            if (
                    fx0 >= x0 - tolerance and fx1 <= x1 + tolerance
                    and fy0 >= y0 - tolerance and fy1 <= y1 + tolerance):
                return True
        return False

    # ---- vectorized candidate generation (§5.1) + §4.2.2 gate ----
    def candidates(self, box_size, allowed_yaws=None, top_n=50,
                   per_level_quota=4):
        """Return feasible candidates, vectorized, stratified by support level.

        The pool is filled level-by-level (see :func:`stratified_pool`) so the
        floor layer cannot be squeezed out by stack candidates before scoring.
        """
        results = []
        for yaw in self._snap_yaws(allowed_yaws or [0.0]):
            rotated = self._is_rotated(yaw)
            fl, fw = (box_size[1], box_size[0]) if rotated else (box_size[0], box_size[1])
            if fl > self.inner_l + 1e-6 or fw > self.inner_w + 1e-6:
                continue
            kx = max(1, int(math.ceil(fl / self.resolution - 1e-9)))
            ky = max(1, int(math.ceil(fw / self.resolution - 1e-9)))
            if kx > self.nx or ky > self.ny:
                continue
            peak = window_max(self.H, kx, ky)  # (nx-kx+1, ny-ky+1)
            unobs = np.isin(self.state, [STATE_UNKNOWN, STATE_FLOOR_PRIOR])
            has_unobs = window_sum(unobs.astype(np.int64), kx, ky) > 0
            top_z = peak + box_size[2]
            clearance_ok = (
                self.inner_h - self.floor_z - top_z
            ) >= self.clearance_margin
            floor_prior_mask = has_unobs & (peak <= _PEAK_FLOOR_TOL)
            unknown_above = has_unobs & (peak > _PEAK_FLOOR_TOL)
            feasible = (~unknown_above) & clearance_ok
            ix0s, iy0s = np.where(feasible)
            for ix0, iy0 in zip(ix0s.tolist(), iy0s.tolist()):
                p = float(peak[ix0, iy0])
                lx, ly = self._cell_center(ix0, iy0, kx, ky)
                if (
                        lx - fl * 0.5
                        < -self.inner_l * 0.5 + self.boundary_margin - 1e-9
                        or lx + fl * 0.5
                        > self.inner_l * 0.5 - self.boundary_margin + 1e-9
                        or ly - fw * 0.5
                        < -self.inner_w * 0.5 + self.boundary_margin - 1e-9
                        or ly + fw * 0.5
                        > self.inner_w * 0.5 - self.boundary_margin + 1e-9):
                    continue
                src = SRC_FLOOR_PRIOR if floor_prior_mask[ix0, iy0] else SRC_GEOMETRY
                # Floor placements (peak≈0) are stable via LBCP_0 (inner rect).
                if p > _PEAK_FLOOR_TOL:
                    if not self.is_stable(lx, ly, [fl, fw], p):
                        continue
                    if not self.has_full_support(lx, ly, [fl, fw], p):
                        continue
                absolute_z = self.floor_z + p + box_size[2] * 0.5
                lz_vol = absolute_z - self.inner_h * 0.5
                results.append({
                    "center_base": self._local_to_base(lx, ly, lz_vol),
                    "center_local": [lx, ly, lz_vol],
                    "yaw": self.yaw + yaw,
                    "box_yaw": yaw,
                    "size": [box_size[0], box_size[1], box_size[2]],
                    "footprint": [fl, fw],
                    "support_score": 1.0,
                    "clearance_score": max(
                        0.0, min(1.0, (
                            self.inner_h - self.floor_z - p - box_size[2]
                        ) / max(box_size[2], 1e-6))),
                    "support_source": src,
                    "peak": round(p, 4),
                    "clearance_top": round(float(
                        self.inner_h - self.floor_z - p - box_size[2]), 4),
                    "confidence_ratio": 0.0 if src == SRC_FLOOR_PRIOR else 1.0,
                    "reachability_score": -1.0,
                    "score": 0.0,
                    "feasible": True,
                    "reason": "ok",
                    "revision": self.revision,
                })
            # Grid quantization can make a narrow upper footprint overhang a
            # support by one cell. Add an exact LBCP-center candidate so stacks
            # use symmetric physical support rather than a nominal CoG-only fit.
            for (x0, y0, x1, y1), support_z in self.lbcp[1:]:
                if fl > (x1 - x0) + 1e-6 or fw > (y1 - y0) + 1e-6:
                    continue
                lx, ly = (x0 + x1) * 0.5, (y0 + y1) * 0.5
                footprint = (
                    lx - fl * 0.5, ly - fw * 0.5,
                    lx + fl * 0.5, ly + fw * 0.5)
                max_overlap_top = 0.0
                for box in self.boxes:
                    if (
                            footprint[2] <= box["x0"]
                            or footprint[0] >= box["x1"]
                            or footprint[3] <= box["y0"]
                            or footprint[1] >= box["y1"]):
                        continue
                    max_overlap_top = max(max_overlap_top, box["z1"])
                if max_overlap_top > support_z + _Z_LEVEL_TOL:
                    continue
                if (
                        self.inner_h - self.floor_z - support_z - box_size[2]
                        < self.clearance_margin):
                    continue
                absolute_z = (
                    self.floor_z + support_z + box_size[2] * 0.5)
                lz_vol = absolute_z - self.inner_h * 0.5
                results.append({
                    "center_base": self._local_to_base(lx, ly, lz_vol),
                    "center_local": [lx, ly, lz_vol],
                    "yaw": self.yaw + yaw,
                    "box_yaw": yaw,
                    "size": [box_size[0], box_size[1], box_size[2]],
                    "footprint": [fl, fw],
                    "support_score": 1.0,
                    "clearance_score": max(
                        0.0, min(1.0, (
                            self.inner_h - self.floor_z - support_z
                            - box_size[2]) / max(box_size[2], 1e-6))),
                    "support_source": SRC_GEOMETRY,
                    "peak": round(float(support_z), 4),
                    "clearance_top": round(float(
                        self.inner_h - self.floor_z - support_z
                        - box_size[2]), 4),
                    "confidence_ratio": 1.0,
                    "contained_support_center": True,
                    "reachability_score": -1.0,
                    "score": 0.0,
                    "feasible": True,
                    "reason": "ok",
                    "revision": self.revision,
                })
                centered_record = results[-1]
                slack_x = max(0.0, (x1 - x0 - fl) * 0.5)
                slack_y = max(0.0, (y1 - y0 - fw) * 0.5)
                for offset_x, offset_y in (
                        (slack_x, 0.0), (-slack_x, 0.0),
                        (0.0, slack_y), (0.0, -slack_y),
                        (slack_x, slack_y), (slack_x, -slack_y)):
                    if abs(offset_x) < 1e-9 and abs(offset_y) < 1e-9:
                        continue
                    aligned = dict(centered_record)
                    aligned_lx = lx + offset_x
                    aligned_ly = ly + offset_y
                    aligned["center_base"] = self._local_to_base(
                        aligned_lx, aligned_ly, lz_vol)
                    aligned["center_local"] = [
                        aligned_lx, aligned_ly, lz_vol]
                    aligned["contained_support_center"] = False
                    aligned["contained_support_aligned"] = True
                    results.append(aligned)
        # Intra-level quality order only. Support height must not be a sort key
        # here: it is the scorer's decision, and stratified_pool guarantees each
        # level reaches the scorer.
        results.sort(key=lambda c: (
            not c.get("contained_support_center", False),
            not c.get("contained_support_aligned", False),
            c["support_source"] == SRC_FLOOR_PRIOR,
        ))
        return stratified_pool(results, top_n, per_level_quota)

    @staticmethod
    def _is_rotated(yaw):
        r = yaw % math.pi
        return abs(r - math.pi * 0.5) < abs(r - 0.0)

    @staticmethod
    def _snap_yaws(allowed_yaws):
        seen, out = set(), []
        for yaw in allowed_yaws:
            # Preserve 180-degree-equivalent tool orientations: geometry is
            # identical, but the robot wrist/IK branch can be very different.
            key = round(float(yaw) % (2.0 * math.pi), 6)
            if key in seen:
                continue
            seen.add(key)
            out.append(yaw)
        return out
