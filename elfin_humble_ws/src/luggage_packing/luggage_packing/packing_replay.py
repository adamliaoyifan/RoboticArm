#!/usr/bin/env python3
"""Offline packing replay: pure-Python simulator for the online packing pipeline.

No ROS / no Gazebo. Loads the reachability atlas (.npz) + container geometry +
box catalog, replays a fixed box sequence through a placement strategy, and
computes the §9.2 metrics (reachable_fill_rate etc.). Used by
``packing_replay_eval.py`` to establish the B0 baseline and A/B future
strategies (P1+). See ``docs/plans/high_utilization_online_packing_design.md`` §9.

Design notes:
  - Atlas status codes: 0=UNKNOWN, 1=UNREACHABLE, 2=MARGINAL, 3=REACHABLE.
    REACHABLE/MARGINAL are treated as IK-feasible (§9.1).
  - BinPackerStrategy replicates bin_packer_node geometry (DBLF first-feasible)
    in floor-relative packing coordinates; atlas queries are converted to
    container_link contact-frame Z using metadata ``floor_z``.
  - B0 models the *current* system: bin_packer returns ONE first-geometry
    candidate; if the atlas says it is not reachable the box is lost (no
    no-good retry loop -- that is P3). ``retry=True`` switches to reachable-set
    DBLF (try next candidate) for comparison.
"""

from __future__ import division

import math
import os

import numpy as np
import yaml

# Atlas status codes (match reachability_atlas builder).
STATUS_UNKNOWN = 0
STATUS_UNREACHABLE = 1
STATUS_MARGINAL = 2
STATUS_REACHABLE = 3
_FEASIBLE_STATUSES = (STATUS_REACHABLE, STATUS_MARGINAL)


def unlock_floor_atlas(atlas, max_iz=1, max_ix=5):
    """Return a copy of the atlas with front-floor cells marked REACHABLE.

    What-if simulation of the §8.2 first-layer strategy (lower transit_clearance
    + lateral insertion): pretends the arm can reach the front portion of the
    floor (ix <= max_ix, iz <= max_iz). Used to measure the *algorithm* lift
    (B2/B3 vs B0) once reachability is no longer the blocker. Not a real atlas.
    """
    import copy
    clone = copy.copy(atlas)
    clone.status = atlas.status.copy()
    nx = min(max_ix + 1, clone.status.shape[0])
    nz = min(max_iz + 1, clone.status.shape[2])
    clone.status[:nx, :, :nz, :] = STATUS_REACHABLE
    return clone


class ReachabilityAtlas(object):
    """Pure-Python query over the reachability atlas .npz + .yaml metadata."""

    def __init__(self, npz_path, yaml_path=None):
        if yaml_path is None:
            yaml_path = npz_path[:-4] + ".yaml" if npz_path.endswith(".npz") else npz_path + ".yaml"
        with open(yaml_path, "r") as handle:
            meta = yaml.safe_load(handle)
        grid = meta["grid"]
        self.resolution = float(grid["resolution_xyz"])
        self.origin = [float(v) for v in grid["origin"]]
        self.size = [int(v) for v in grid["size"]]
        self.yaw_bins = [float(v) for v in grid["yaw_bins"]]
        legacy_inner = [
            float(v) for v in meta["container"]["inner_dimensions"]]
        data = np.load(npz_path, allow_pickle=True)
        self.status = np.asarray(data["status"])  # (nx,ny,nz,nyaw) uint8
        # Container pose in base (for FreeSpaceModelStrategy).
        cont = meta["container"]
        self.floor_z = float(cont.get("floor_z", 0.0))
        self.ceiling_z = float(cont.get(
            "ceiling_z", legacy_inner[2]))
        self.inner_size = [
            legacy_inner[0], legacy_inner[1],
            self.ceiling_z - self.floor_z,
        ]
        self.base_xyz = [float(v) for v in cont["base_xyz"]]
        self.base_rpy = [float(v) for v in cont["base_rpy"]]
        self.yaw = self.base_rpy[2]
        # Usable interior center in base = midpoint(floor_z, ceiling_z).
        # rpy is a pure yaw about Z, so container Z stays aligned with base Z.
        self.center_base = [
            self.base_xyz[0], self.base_xyz[1],
            self.base_xyz[2] + 0.5 * (self.floor_z + self.ceiling_z)]
        # Payload the atlas was built with (None for empty-load). §5.6.
        self.payload = meta.get("payload")

    def _cell(self, center_local, yaw):
        """Map a container_link-frame center + yaw to (ix,iy,iz,iyaw)."""
        res = self.resolution
        ix = int((center_local[0] - self.origin[0]) / res)
        iy = int((center_local[1] - self.origin[1]) / res)
        iz = int((center_local[2] - self.origin[2]) / res)
        yaw_r = yaw % math.pi
        iyaw = min(range(len(self.yaw_bins)),
                   key=lambda i: abs(yaw_r - self.yaw_bins[i] % math.pi))
        return ix, iy, iz, iyaw

    def status_at(self, center_local, yaw=0.0):
        ix, iy, iz, iyaw = self._cell(center_local, yaw)
        nx, ny, nz, _ = self.status.shape
        if not (0 <= ix < nx and 0 <= iy < ny and 0 <= iz < nz):
            return STATUS_UNREACHABLE  # outside the grid -> conservative
        return int(self.status[ix, iy, iz, iyaw])

    def is_reachable(self, center_local, yaw=0.0):
        """REACHABLE or MARGINAL -> IK-feasible (design §9.1)."""
        return self.status_at(center_local, yaw) in _FEASIBLE_STATUSES

    def reachable_volume(self):
        """Union of REACHABLE/MARGINAL cells (any yaw) x cell volume."""
        mask = np.isin(self.status, list(_FEASIBLE_STATUSES))
        any_yaw = mask.any(axis=3)
        return float(any_yaw.sum()) * (self.resolution ** 3)


class BinPackerStrategy(object):
    """Pure-Python replication of bin_packer_node geometry (DBLF first-feasible).

    Packing frame: X/Y in [-inner/2,+inner/2], floor-relative Z in
    [0, usable_height]. Boxes are axis-aligned.
    ``box_size`` is [length(X), width(Y), height(Z)] matching box_catalog.
    """

    def __init__(self, inner_size, clearance=0.02, support_overlap_ratio=0.35):
        self.inner_l, self.inner_w, self.inner_h = [float(v) for v in inner_size]
        self.clearance = float(clearance)
        self.support_overlap_ratio = float(support_overlap_ratio)

    def _intersects(self, cand, placed):
        c = self.clearance
        for occ in placed:
            if (cand["min_x"] < occ["max_x"] + c and cand["max_x"] > occ["min_x"] - c
                    and cand["min_y"] < occ["max_y"] + c and cand["max_y"] > occ["min_y"] - c
                    and cand["min_z"] < occ["max_z"] + c and cand["max_z"] > occ["min_z"] - c):
                return True
        return False

    def _has_support(self, cand, placed):
        if cand["min_z"] <= self.clearance:
            return True  # floor
        bottom_area = (cand["max_x"] - cand["min_x"]) * (cand["max_y"] - cand["min_y"])
        if bottom_area <= 0.0:
            return False
        support = 0.0
        for occ in placed:
            if abs(cand["min_z"] - occ["max_z"]) > self.clearance:
                continue
            ox = max(0.0, min(cand["max_x"], occ["max_x"]) - max(cand["min_x"], occ["min_x"]))
            oy = max(0.0, min(cand["max_y"], occ["max_y"]) - max(cand["min_y"], occ["min_y"]))
            support += ox * oy
        return support / bottom_area >= self.support_overlap_ratio

    @staticmethod
    def _unique_sorted(values, min_value):
        return [v for v in sorted(set(round(v, 4) for v in values))
                if v >= min_value - 1e-6]

    def candidates(self, box_size, placed):
        """Yield geometry-feasible candidate dicts (DBLF order: z, y, x)."""
        w, d, h = [float(v) for v in box_size]
        xs = [-self.inner_l * 0.5] + [occ["max_x"] + self.clearance for occ in placed]
        ys = [-self.inner_w * 0.5] + [occ["max_y"] + self.clearance for occ in placed]
        zs = [0.0] + [occ["max_z"] + self.clearance for occ in placed]
        for z in self._unique_sorted(zs, 0.0):
            if z + h > self.inner_h + 1e-6:
                continue
            for y in self._unique_sorted(ys, -self.inner_w * 0.5):
                if y < -self.inner_w * 0.5 - 1e-6 or y + d > self.inner_w * 0.5 + 1e-6:
                    continue
                for x in self._unique_sorted(xs, -self.inner_l * 0.5):
                    if x < -self.inner_l * 0.5 - 1e-6 or x + w > self.inner_l * 0.5 + 1e-6:
                        continue
                    cand = {"min_x": x, "max_x": x + w, "min_y": y, "max_y": y + d,
                            "min_z": z, "max_z": z + h}
                    if self._intersects(cand, placed) or not self._has_support(cand, placed):
                        continue
                    yield cand

    @staticmethod
    def center(cand):
        return (0.5 * (cand["min_x"] + cand["max_x"]),
                0.5 * (cand["min_y"] + cand["max_y"]),
                0.5 * (cand["min_z"] + cand["max_z"]))

    @staticmethod
    def volume(cand):
        return ((cand["max_x"] - cand["min_x"]) *
                (cand["max_y"] - cand["min_y"]) *
                (cand["max_z"] - cand["min_z"]))


class ReplaySimulator(object):
    """Replay a box sequence through a strategy, gated by atlas reachability."""

    def __init__(self, strategy, atlas=None, retry=False):
        self.strategy = strategy
        self.atlas = atlas
        self.retry = retry  # False=B0 (first candidate, fail if unreachable);
        #                    True=reachable-set DBLF (try next candidate)

    def run(self, sequence):
        """sequence: list of box_size [l,w,h]. Returns metrics dict (§9.2)."""
        placed = []
        V_placed = 0.0
        items_placed = 0
        first_fail_index = -1
        reject = {"atlas_unreachable": 0, "no_geometry": 0}
        for i, box_size in enumerate(sequence):
            placed_this = False
            geom_failed = True
            for cand in self.strategy.candidates(box_size, placed):
                geom_failed = False
                if self.atlas is not None:
                    center = self.strategy.center(cand)
                    contact = (
                        center[0], center[1],
                        getattr(self.atlas, "floor_z", 0.0) + cand["max_z"])
                if self.atlas is not None and not self.atlas.is_reachable(
                        contact, 0.0):
                    reject["atlas_unreachable"] += 1
                    if not self.retry:
                        break  # B0: first candidate lost
                    continue  # reachable-set DBLF: try next
                placed.append(cand)
                V_placed += self.strategy.volume(cand)
                items_placed += 1
                placed_this = True
                break
            if not placed_this:
                if first_fail_index < 0:
                    first_fail_index = i
                if geom_failed:
                    reject["no_geometry"] += 1
        inner = (self.strategy.inner_l, self.strategy.inner_w, self.strategy.inner_h)
        V_container = inner[0] * inner[1] * inner[2]
        V_reachable = self.atlas.reachable_volume() if self.atlas else V_container
        return {
            "items_placed": items_placed,
            "first_fail_index": first_fail_index,
            "V_placed": V_placed,
            "V_container": V_container,
            "V_reachable": V_reachable,
            "reachable_fill_rate": V_placed / V_reachable if V_reachable > 0 else 0.0,
            "reachable_volume_ratio": V_reachable / V_container if V_container > 0 else 0.0,
            "overall_fill_rate": V_placed / V_container if V_container > 0 else 0.0,
            "reject_histogram": reject,
        }


def summarize(runs):
    """Aggregate a list of run-metrics dicts (one per sequence)."""
    import statistics
    rfr = [r["reachable_fill_rate"] for r in runs]
    items = [r["items_placed"] for r in runs]
    floor_items = [r.get("floor_items", 0) for r in runs]
    floor_coverage = [r.get("floor_coverage", 0.0) for r in runs]
    return {
        "n_sequences": len(runs),
        "reachable_fill_rate_mean": statistics.mean(rfr) if rfr else 0.0,
        "reachable_fill_rate_stdev": statistics.stdev(rfr) if len(rfr) > 1 else 0.0,
        "reachable_fill_rate_min": min(rfr) if rfr else 0.0,
        "reachable_fill_rate_max": max(rfr) if rfr else 0.0,
        "items_placed_mean": statistics.mean(items) if items else 0.0,
        "floor_items_mean": statistics.mean(floor_items) if floor_items else 0.0,
        "floor_coverage_mean": (
            statistics.mean(floor_coverage) if floor_coverage else 0.0),
        "reachable_volume_ratio": runs[0]["reachable_volume_ratio"] if runs else 0.0,
    }


# --------------------------------------------------------------------------- #
# FreeSpaceModel-based strategies (B2/B3) -- §4/§5.7
# --------------------------------------------------------------------------- #

def _cand_box_floor(cand):
    """Candidate AABB in floor-relative coords (for EMS split)."""
    lx, ly = cand["center_local"][0], cand["center_local"][1]
    peak = cand.get("peak", 0.0)
    bl, bw, bh = cand["size"]
    return (lx - bl * 0.5, ly - bw * 0.5, peak,
            lx + bl * 0.5, ly + bw * 0.5, peak + bh)


class FreeSpaceModelStrategy(object):
    """FreeSpaceModel placement + atlas gating + proxy/value_hat scoring.

    ``mode``:
      - ``first``: first atlas-reachable candidate (FreeSpaceModel order).
      - ``b2``: reachable candidates ranked by §5.7 proxy_score.
      - ``b3``: top-K reachable candidates ranked by §5.7 rollout value_hat.
      - ``b4``: the production scorer (``placement_scoring.score_candidates``),
        i.e. proxy_score plus the floor-first counterweight and the atlas
        reachability prior. Replaying it here is what lets the online policy be
        measured before it ever runs in Gazebo.
    """

    def __init__(self, inner_size, center_base, yaw, resolution, atlas, entries,
                 mode="b2", smallest_size=None, top_n=50, top_k_rollout=5,
                 rollout_K=3, rollout_M=8, w_floor_first=None,
                 opening_side="negative_x"):
        from luggage_packing.placement_scoring import DEFAULT_W_FLOOR_FIRST
        self.inner_size = [float(v) for v in inner_size]
        self.inner_l, self.inner_w, self.inner_h = self.inner_size
        self.center_base = center_base
        self.yaw = yaw
        self.resolution = resolution
        self.atlas = atlas
        self.entries = entries
        self.mode = mode
        self.smallest_size = smallest_size or [0.55, 0.40, 0.25]
        self.top_n = top_n
        self.top_k_rollout = top_k_rollout
        self.rollout_K = rollout_K
        self.rollout_M = rollout_M
        self.w_floor_first = (
            DEFAULT_W_FLOOR_FIRST if w_floor_first is None
            else float(w_floor_first))
        self.opening_side = opening_side

    def _build_model(self, placed):
        from luggage_packing.free_space_model import FreeSpaceModel
        m = FreeSpaceModel(
            self.inner_size, self.center_base, self.yaw, self.resolution)
        for p in placed:
            m.add_placed_box(p["center_local"], p["size"])
        return m

    def _atlas_contact(self, cand):
        # Atlas targets suction_contact_frame at the box top, not box center.
        # ``cand.peak`` is relative to the usable slab-top floor.
        box_h = cand["size"][2]
        return (
            cand["center_local"][0], cand["center_local"][1],
            getattr(self.atlas, "floor_z", 0.0) + cand["peak"] + box_h)

    def _atlas_reachable(self, cand):
        return self.atlas.is_reachable(
            self._atlas_contact(cand), cand["box_yaw"])

    def _annotate_prior(self, cand):
        """Attach the atlas status as a continuous prior for the scorer."""
        from luggage_packing.placement_reachability import ATLAS_PRIOR_BY_STATUS
        status = self.atlas.status_at(
            self._atlas_contact(cand), cand["box_yaw"])
        cand["reachability_prior"] = ATLAS_PRIOR_BY_STATUS.get(status, 0.0)
        return cand

    def place(self, box_size, placed):
        from luggage_packing.ems import EMS
        from luggage_packing.insertion_corridor import proxy_score
        from luggage_packing.value_estimator import value_hat
        model = self._build_model(placed)
        ems = EMS(self.inner_size, min_useful_edge=0.1)
        for p in placed:
            ems.place(_cand_box_floor(p))
        cands = model.candidates(box_size, allowed_yaws=[0.0, 1.5707963],
                                 top_n=self.top_n)
        reachable = [c for c in cands if self._atlas_reachable(c)]
        if not reachable:
            return None
        if self.mode == "first":
            return reachable[0]
        if self.mode == "mvp":
            # The three-term proxy that shipped in placement_planner_node
            # before the floor-first work. Kept as the honest baseline: it is
            # what actually ran on the robot, and it prefers a stack over an
            # unobserved floor for any support below ~0.82 m.
            best, best_s = None, -1e18
            for c in reachable:
                s = (1.0 / (1.0 + max(0.0, float(c["peak"])))
                     + 0.25 * float(c.get("confidence_ratio", 0.0))
                     + 0.20 * float(bool(
                         c.get("contained_support_center", False)
                         or c.get("contained_support_aligned", False))))
                if s > best_s:
                    best, best_s = c, s
            return best
        if self.mode == "b4":
            from luggage_packing.placement_scoring import score_candidates
            for cand in reachable:
                self._annotate_prior(cand)
            score_candidates(
                reachable, model, ems, self.inner_size, self.smallest_size,
                opening_side=self.opening_side,
                w_floor_first=self.w_floor_first)
            return reachable[0]
        if self.mode == "b2":
            best, best_s = None, -1e18
            for c in reachable:
                s, _ = proxy_score(c, model, ems, self.inner_size, self.smallest_size)
                if s > best_s:
                    best, best_s = c, s
            return best
        if self.mode == "b3":
            pool = reachable[: self.top_k_rollout]
            best, best_s = None, -1e18
            for c in pool:
                s = value_hat(c, model, ems, self.entries,
                              K=self.rollout_K, M=self.rollout_M, seed=0)
                if s > best_s:
                    best, best_s = c, s
            return best
        return reachable[0]


class StrategySimulator(object):
    """Replay a sequence through a strategy with a ``place(box, placed)`` API."""

    def __init__(self, strategy, atlas=None):
        self.strategy = strategy
        self.atlas = atlas  # for V_reachable metrics only

    def run(self, sequence):
        placed = []
        V_placed = 0.0
        items_placed = 0
        first_fail_index = -1
        for i, box_size in enumerate(sequence):
            cand = self.strategy.place(box_size, placed)
            if cand is None:
                if first_fail_index < 0:
                    first_fail_index = i
                continue
            placed.append(cand)
            V_placed += cand["size"][0] * cand["size"][1] * cand["size"][2]
            items_placed += 1
        V_container = self.strategy.inner_l * self.strategy.inner_w * self.strategy.inner_h
        V_reachable = self.atlas.reachable_volume() if self.atlas else V_container
        floor_area = self.strategy.inner_l * self.strategy.inner_w
        # Floor-layer metrics: how much of the container bottom actually got
        # used before the policy started stacking.
        floor_items = 0
        floor_footprint = 0.0
        max_layer_peak = 0.0
        for cand in placed:
            peak = float(cand.get("peak", 0.0))
            max_layer_peak = max(max_layer_peak, peak)
            if peak <= 1e-3:
                floor_items += 1
                footprint = cand.get("footprint", cand["size"][:2])
                floor_footprint += float(footprint[0]) * float(footprint[1])
        return {
            "items_placed": items_placed,
            "first_fail_index": first_fail_index,
            "V_placed": V_placed,
            "V_container": V_container,
            "V_reachable": V_reachable,
            "reachable_fill_rate": V_placed / V_reachable if V_reachable > 0 else 0.0,
            "reachable_volume_ratio": V_reachable / V_container if V_container > 0 else 0.0,
            "overall_fill_rate": V_placed / V_container if V_container > 0 else 0.0,
            "floor_items": floor_items,
            "floor_coverage": floor_footprint / floor_area if floor_area > 0 else 0.0,
            "max_layer_peak": max_layer_peak,
        }
