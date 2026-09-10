#!/usr/bin/env python3
"""Reachability atlas: pre-computed (x,y,z,yaw) reachability grid.

Pure-Python module (no ROS). Loads a .npz data file + .yaml metadata,
provides O(1) lookup for placement candidate filtering, and stores IK
seed joint states for fast convergence.

Grid is defined in ``container_link`` coordinates (exterior bottom-face
origin): X/Y follow the semantic inner bounds and Z is sampled between the
configured slab-top floor and ceiling.
"""

from __future__ import division

import math
import os
from collections import namedtuple

import numpy as np
import yaml

from luggage_description.container_geometry import (
    contains_oriented_box,
    contains_point,
    normalize_descriptor,
    volume,
)

from .atlas_builder import PayloadProfile, payload_center
from .atlas_io import save_npz_deterministic


UNKNOWN = 0
UNREACHABLE = 1
MARGINAL = 2
REACHABLE = 3

STATUS_NAMES = {
    UNKNOWN: "unknown",
    UNREACHABLE: "unreachable",
    MARGINAL: "marginal",
    REACHABLE: "reachable",
}

GEOMETRY_IDENTITY_REQUIRED = "geometry_identity_required"
GEOMETRY_IDENTITY_MISMATCH = "geometry_identity_mismatch"
GEOMETRY_IDENTITY_CHANGED = "geometry_identity_changed"
HULL_INVALID_PAYLOAD = "hull_invalid_payload"
INACTIVE_HULL_CELL = "inactive_hull_cell"


class AtlasIdentityError(ValueError):
    """Atlas metadata cannot be bound to the requested runtime geometry."""

QueryResult = namedtuple(
    "QueryResult",
    (
        "status",
        "opening_connected",
        "contact_seeds",
        "transit_seeds",
        "solution_count",
        "joint_margin",
        "manipulability",
        "neighbor_confidence",
        "hard_reject_safe",
        "score",
        "reason",
        "indices",
        "yaw_error",
    ),
)


class ReachabilityAtlas:
    """Pre-computed reachability grid for placement candidate filtering."""

    UNKNOWN = UNKNOWN
    UNREACHABLE = UNREACHABLE
    MARGINAL = MARGINAL
    REACHABLE = REACHABLE

    def __init__(self, data, meta, expected_geometry=None, allow_legacy=False):
        """Initialize from loaded data dict and metadata dict.

        Args:
            data: v2 arrays, or legacy v1 arrays (reachable and seed_joints).
            meta: dict with grid definition and metadata.
        """
        self._meta = dict(meta)
        default_schema = int(float(meta.get("atlas_version", "1.0")))
        self._schema_version = int(meta.get("schema_version", default_schema))
        grid = meta["grid"]
        self._resolution = float(grid["resolution_xyz"])
        self._origin = [float(v) for v in grid["origin"]]
        self._size = list(grid["size"])  # [nx, ny, nz]
        self._yaw_bins = [float(y) for y in grid["yaw_bins"]]
        self._transit_clearance = float(grid.get("transit_clearance", 0.30))
        self._nx, self._ny, self._nz = self._size
        self._nyaw = len(self._yaw_bins)
        self._shape = (self._nx, self._ny, self._nz, self._nyaw)

        self._geometry = None
        self._geometry_invalid_reason = None
        self._load_geometry_identity(expected_geometry, allow_legacy)

        query_meta = meta.get("query", {})
        self._yaw_tolerance = float(
            query_meta.get("yaw_tolerance", grid.get("yaw_tolerance", 0.10)))
        self._hard_reject_min_confidence = float(
            query_meta.get(
                "hard_reject_min_neighbor_confidence",
                meta.get("hard_reject_min_neighbor_confidence", 0.90)))
        self._interior_fraction = float(
            query_meta.get("hard_reject_interior_fraction", 0.10))

        self._load_arrays(data)

    def _load_geometry_identity(self, expected_geometry, allow_legacy):
        identity = self._meta.get("geometry")
        if self._schema_version < 3:
            if expected_geometry is not None and not allow_legacy:
                raise AtlasIdentityError(GEOMETRY_IDENTITY_REQUIRED)
            return
        if not isinstance(identity, dict):
            raise AtlasIdentityError(GEOMETRY_IDENTITY_REQUIRED)
        descriptor = identity.get("descriptor")
        if not isinstance(descriptor, dict):
            raise AtlasIdentityError(GEOMETRY_IDENTITY_REQUIRED)
        try:
            geometry = normalize_descriptor(descriptor)
        except (KeyError, TypeError, ValueError) as exc:
            raise AtlasIdentityError(
                "%s: %s" % (GEOMETRY_IDENTITY_REQUIRED, exc))
        if int(identity.get("schema_version", -1)) != geometry.schema_version:
            raise AtlasIdentityError(GEOMETRY_IDENTITY_MISMATCH)
        if str(identity.get("geometry_hash", "")) != geometry.geometry_hash:
            raise AtlasIdentityError(GEOMETRY_IDENTITY_MISMATCH)
        if not str(self._meta.get("builder_revision", "")).strip():
            raise AtlasIdentityError("builder_revision_required")
        if expected_geometry is None:
            raise AtlasIdentityError(GEOMETRY_IDENTITY_REQUIRED)
        expected = normalize_descriptor(dict(expected_geometry))
        if expected.geometry_hash != geometry.geometry_hash:
            raise AtlasIdentityError(GEOMETRY_IDENTITY_MISMATCH)
        if self._meta["grid"].get("frame", geometry.frame_id) != geometry.frame_id:
            raise AtlasIdentityError(GEOMETRY_IDENTITY_MISMATCH)
        self._geometry = geometry

    def _grid_array(self, value, name, dtype, default):
        if value is None:
            return np.full(self._shape, default, dtype=dtype)
        array = np.asarray(value, dtype=dtype)
        if array.shape != self._shape:
            raise ValueError("%s has shape %r, expected %r" % (
                name, array.shape, self._shape))
        return array

    def _seed_array(self, value, name):
        if value is None:
            return np.zeros(self._shape + (0, 6), dtype=np.float64)
        array = np.asarray(value, dtype=np.float64)
        if array.shape == self._shape + (6,):
            array = np.expand_dims(array, axis=-2)
        if array.ndim != 6 or array.shape[:4] != self._shape or array.shape[-1] != 6:
            raise ValueError(
                "%s must have shape %r + (nseed, 6), got %r" % (
                    name, self._shape, array.shape))
        return array

    @staticmethod
    def _pad_seeds(array, nseed):
        if array.shape[-2] == nseed:
            return array
        padded = np.zeros(array.shape[:4] + (nseed, 6), dtype=np.float64)
        padded[..., :array.shape[-2], :] = array
        return padded

    def _load_arrays(self, data):
        """Normalize both on-disk schemas to the v2 in-memory schema."""
        legacy = "status" not in data
        reachable = self._grid_array(
            data.get("reachable"), "reachable", np.bool_, False)

        if legacy:
            status = np.where(reachable, REACHABLE, UNREACHABLE).astype(np.uint8)
            margin = data.get("joint_margin")
            marginal_threshold = float(
                self._meta.get("marginal_joint_margin", 0.10))
            if margin is not None:
                margin_array = self._grid_array(
                    margin, "joint_margin", np.float32, np.nan)
                status[np.logical_and(
                    reachable, margin_array < marginal_threshold)] = MARGINAL
            legacy_seeds = data.get("seed_joints")
            contact_seeds = self._seed_array(legacy_seeds, "seed_joints")
            transit_seeds = self._seed_array(legacy_seeds, "seed_joints")
        else:
            status = self._grid_array(
                data.get("status"), "status", np.uint8, UNKNOWN)
            if np.any(status > REACHABLE):
                raise ValueError("status contains values outside schema v2")
            contact_seeds = self._seed_array(
                data.get("contact_seeds", data.get("seed_joints")),
                "contact_seeds")
            transit_seeds = self._seed_array(
                data.get("transit_seeds", data.get("seed_joints")),
                "transit_seeds")

        nseed = max(contact_seeds.shape[-2], transit_seeds.shape[-2])
        self._contact_seeds = self._pad_seeds(contact_seeds, nseed)
        self._transit_seeds = self._pad_seeds(transit_seeds, nseed)
        self._nseed = nseed

        self._status = status.astype(np.uint8, copy=False)
        self._reachable = np.logical_or(
            self._status == MARGINAL, self._status == REACHABLE)
        self._contact_ik = self._grid_array(
            data.get("contact_ik"), "contact_ik", np.bool_, False)
        self._transit_ik = self._grid_array(
            data.get("transit_ik"), "transit_ik", np.bool_, False)
        self._opening_connected = self._grid_array(
            data.get("opening_connected"), "opening_connected", np.bool_, False)
        self._solution_count = self._grid_array(
            data.get("solution_count"), "solution_count", np.uint8, 0)
        if legacy and "solution_count" not in data:
            self._solution_count[self._reachable] = 1
        self._joint_margin = self._grid_array(
            data.get("joint_margin"), "joint_margin", np.float32, np.nan)
        self._manipulability = self._grid_array(
            data.get("manipulability"), "manipulability", np.float32, np.nan)
        self._neighbor_confidence = self._grid_array(
            data.get("neighbor_confidence"),
            "neighbor_confidence", np.float32, 0.0)

        if self._schema_version >= 3:
            if "active_mask" not in data or "cell_volumes" not in data:
                raise AtlasIdentityError("atlas_geometry_arrays_required")
            self._active_mask = self._grid_array(
                data.get("active_mask"), "active_mask", np.bool_, False)
            self._cell_volumes = np.asarray(
                data["cell_volumes"], dtype=np.float64)
            expected = (self._nx, self._ny, self._nz)
            if self._cell_volumes.shape != expected:
                raise ValueError("cell_volumes has shape %r, expected %r" % (
                    self._cell_volumes.shape, expected))
            if (not np.isfinite(self._cell_volumes).all()
                    or np.any(self._cell_volumes < 0.0)):
                raise ValueError(
                    "cell_volumes must be finite and non-negative")
            zero_volume = self._cell_volumes <= 0.0
            if np.any(np.logical_and(
                    self._active_mask,
                    np.broadcast_to(zero_volume[..., None], self._shape))):
                raise ValueError(
                    "active_mask includes zero-volume hull cells")
            self._status[~self._active_mask] = UNKNOWN
            self._reachable[~self._active_mask] = False
            self._payload = self._payload_from_meta(
                self._meta.get("payload"))
        else:
            self._active_mask = np.ones(self._shape, dtype=np.bool_)
            self._cell_volumes = np.full(
                (self._nx, self._ny, self._nz), self._resolution ** 3,
                dtype=np.float64)
            self._payload = PayloadProfile()

        # Kept for callers that used the v1 private member.
        self._seeds = (
            self._contact_seeds[..., 0, :]
            if self._nseed else np.zeros(self._shape + (6,), dtype=np.float64))

    @staticmethod
    def _payload_from_meta(raw):
        if not isinstance(raw, dict):
            raise AtlasIdentityError("payload_profile_required")
        required = {
            "enabled", "shape", "size", "center_offset",
            "yaw_convention", "center_convention",
        }
        if not required.issubset(raw):
            raise AtlasIdentityError("payload_profile_required")
        if (str(raw["yaw_convention"]) !=
                PayloadProfile().yaw_convention
                or str(raw["center_convention"]) !=
                PayloadProfile().center_convention):
            raise AtlasIdentityError("invalid_payload_profile")
        if not bool(raw.get("enabled", False)):
            if (raw["shape"] is not None or raw["size"] is not None
                    or raw["center_offset"] is not None):
                raise AtlasIdentityError("invalid_payload_profile")
            return PayloadProfile()
        if raw.get("shape") != "box":
            raise AtlasIdentityError("invalid_payload_profile")
        try:
            return PayloadProfile(
                enabled=True,
                size=tuple(raw["size"]),
                center_offset=tuple(raw["center_offset"]),
                yaw_convention=str(raw["yaw_convention"]),
                center_convention=str(raw["center_convention"]),
            ).validated()
        except (KeyError, TypeError, ValueError) as exc:
            raise AtlasIdentityError("invalid_payload_profile: %s" % exc)

    # ── Loading / Saving ─────────────────────────────────────────────

    @classmethod
    def load(cls, npz_path, meta_path, geometry_descriptor=None,
             allow_legacy=False):
        """Load an atlas and bind it to the current geometry identity.

        Schema-v1/v2 files have no authoritative hull identity and are rejected
        by default.  ``allow_legacy`` exists only for explicit offline
        inspection and migration tests; maintained runtime paths do not use it.
        """
        data = dict(np.load(npz_path, allow_pickle=False))
        with open(meta_path, "r", encoding="utf-8") as f:
            meta = yaml.safe_load(f)
        if int(meta.get("schema_version", 1)) < 3 and not allow_legacy:
            raise AtlasIdentityError(GEOMETRY_IDENTITY_REQUIRED)
        return cls(
            data, meta, expected_geometry=geometry_descriptor,
            allow_legacy=allow_legacy)

    def save(self, npz_path, meta_path):
        """Save the in-memory schema plus v1 compatibility arrays."""
        os.makedirs(os.path.dirname(npz_path) or ".", exist_ok=True)
        arrays = {
            "status": self._status,
            "opening_connected": self._opening_connected,
            "reachable": self._reachable,
            "contact_ik": self._contact_ik,
            "transit_ik": self._transit_ik,
            "contact_seeds": self._contact_seeds,
            "transit_seeds": self._transit_seeds,
            "solution_count": self._solution_count,
            "neighbor_confidence": self._neighbor_confidence,
            "seed_joints": self._seeds,
            "joint_margin": self._joint_margin,
            "manipulability": self._manipulability,
        }
        if self._schema_version >= 3:
            arrays["active_mask"] = self._active_mask
            arrays["cell_volumes"] = self._cell_volumes
        save_npz_deterministic(npz_path, arrays)
        os.makedirs(os.path.dirname(meta_path) or ".", exist_ok=True)
        saved_meta = dict(self._meta)
        saved_meta["atlas_version"] = (
            "3.0" if self._schema_version >= 3 else "2.0")
        saved_meta["schema_version"] = self._schema_version
        with open(meta_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(
                saved_meta, f, default_flow_style=False, sort_keys=False)

    @classmethod
    def from_builder(cls, *args, **kwargs):
        """Create from builder arrays using either the v1 or v2 signature.

        V1 positional signature:
          reachable, contact_ik, transit_ik, seeds, joint_margin,
          manipulability, meta

        V2 callers should use keyword arguments matching the saved array names.
        A full 11-argument v2 positional signature is also accepted.
        """
        if args:
            v1_names = (
                "reachable", "contact_ik", "transit_ik", "seed_joints",
                "joint_margin", "manipulability", "meta")
            v2_names = (
                "status", "opening_connected", "contact_ik", "transit_ik",
                "contact_seeds", "transit_seeds", "solution_count",
                "joint_margin", "manipulability", "neighbor_confidence",
                "meta")
            v2_only = {
                "status", "opening_connected", "contact_seeds",
                "transit_seeds", "solution_count", "neighbor_confidence",
            }
            names = (
                v2_names if len(args) > len(v1_names) or
                v2_only.intersection(kwargs) else v1_names)
            if len(args) > len(names):
                raise TypeError(
                    "too many positional arguments for from_builder")
            values = dict(kwargs)
            for name, value in zip(names, args):
                if name in values:
                    raise TypeError(
                        "from_builder got multiple values for %s" % name)
                values[name] = value
            kwargs = values

        values = dict(kwargs)
        if "seeds" in values and "seed_joints" not in values:
            values["seed_joints"] = values.pop("seeds")
        try:
            meta = values.pop("meta")
        except KeyError:
            raise TypeError("from_builder requires meta")
        if "status" not in values and "reachable" not in values:
            raise TypeError("from_builder requires status or reachable")
        data = values
        schema_version = int(meta.get("schema_version", 1))
        expected_geometry = None
        if schema_version >= 3:
            identity = meta.get("geometry", {})
            expected_geometry = identity.get("descriptor")
        return cls(
            data, meta, expected_geometry=expected_geometry,
            allow_legacy=schema_version < 3)

    # ── Version Verification ─────────────────────────────────────────

    def verify_version(self, scene_tf_hash=None, urdf_hash=None):
        """Check if the atlas matches the current scene/robot.

        Returns (ok: bool, message: str).
        """
        deps = self._meta.get("dependencies", {})
        if scene_tf_hash is not None:
            stored = deps.get("scene_tf_hash", "")
            if stored and stored != scene_tf_hash:
                return False, "scene_tf hash mismatch (atlas=%s, current=%s)" % (
                    stored[:8], scene_tf_hash[:8])
        if urdf_hash is not None:
            stored = deps.get("urdf_hash", "")
            if stored and stored != urdf_hash:
                return False, "urdf hash mismatch (atlas=%s, current=%s)" % (
                    stored[:8], urdf_hash[:8])
        return True, "ok"

    # ── Query ────────────────────────────────────────────────────────

    def _nearest_yaw_index(self, yaw):
        """Return (nearest-bin index, wrapped angular error)."""
        best_i, best_d = 0, float("inf")
        for i, yb in enumerate(self._yaw_bins):
            d = abs((float(yaw) - yb + math.pi) % (2.0 * math.pi) - math.pi)
            if d < best_d:
                best_d, best_i = d, i
        return best_i, best_d

    def _cell_indices(self, x, y, z):
        """Convert container_link coords to grid indices.

        Returns (ix, iy, iz) or None if out of bounds.
        """
        ix = int(math.floor((x - self._origin[0]) / self._resolution))
        iy = int(math.floor((y - self._origin[1]) / self._resolution))
        iz = int(math.floor((z - self._origin[2]) / self._resolution))
        if not (0 <= ix < self._nx and 0 <= iy < self._ny and 0 <= iz < self._nz):
            return None
        return ix, iy, iz

    def _empty_result(self, reason, yaw_error=float("nan")):
        return QueryResult(
            UNKNOWN, False, [], [], 0, float("nan"), float("nan"), 0.0,
            False, 0.0, reason, None, yaw_error)

    def _is_interior(self, x, y, z, ix, iy, iz):
        """Require both an interior grid cell and distance from cell faces."""
        if not (0 < ix < self._nx - 1 and
                0 < iy < self._ny - 1 and
                0 < iz < self._nz - 1):
            return False
        fractions = (
            (x - self._origin[0]) / self._resolution - ix,
            (y - self._origin[1]) / self._resolution - iy,
            (z - self._origin[2]) / self._resolution - iz,
        )
        margin = min(max(self._interior_fraction, 0.0), 0.499999)
        return all(margin <= fraction <= 1.0 - margin
                   for fraction in fractions)

    def _seeds_for(self, array, idx, enabled, count):
        if not enabled or self._nseed == 0:
            return []
        used = min(max(int(count), 1), self._nseed)
        return [list(seed) for seed in array[idx][:used]]

    def invalidate_geometry(self, reason=GEOMETRY_IDENTITY_CHANGED):
        """Fail all later queries after a sensed geometry identity change."""
        self._geometry_invalid_reason = str(reason)

    def bind_identity_validator(self, validator, name="reachability_atlas"):
        """Register the atlas query cache with a runtime identity tracker."""
        validator.register_invalidator(
            "atlas_queries", str(name), self.invalidate_geometry)

    def _runtime_geometry_reason(self, x, y, z, yaw, current_geometry_hash):
        if self._schema_version < 3:
            return None
        if self._geometry_invalid_reason:
            return self._geometry_invalid_reason
        if (current_geometry_hash is not None
                and str(current_geometry_hash) != self._geometry.geometry_hash):
            return GEOMETRY_IDENTITY_MISMATCH
        contact = (float(x), float(y), float(z))
        if not contains_point(self._geometry, contact):
            return HULL_INVALID_PAYLOAD
        if self._payload.enabled:
            center = payload_center(contact, yaw, self._payload)
            if not contains_oriented_box(
                    self._geometry, center, self._payload.size,
                    yaw=float(yaw)):
                return HULL_INVALID_PAYLOAD
        return None

    def query(self, x, y, z, yaw=0.0, yaw_tolerance=None,
              current_geometry_hash=None):
        """Query one contact pose and return a reachability result.

        Out-of-bounds positions and yaw misses are UNKNOWN.  Hard rejection is
        deliberately limited to high-confidence UNREACHABLE samples strictly
        inside both the atlas grid and their cell.
        """
        geometry_reason = self._runtime_geometry_reason(
            x, y, z, yaw, current_geometry_hash)
        if geometry_reason is not None:
            return self._empty_result(geometry_reason)
        idx3 = self._cell_indices(x, y, z)
        if idx3 is None:
            return self._empty_result("out_of_bounds")

        iyaw, yaw_error = self._nearest_yaw_index(yaw)
        tolerance = (
            self._yaw_tolerance if yaw_tolerance is None
            else float(yaw_tolerance))
        if yaw_error > tolerance:
            return self._empty_result("yaw_mismatch", yaw_error)

        ix, iy, iz = idx3
        idx = (ix, iy, iz, iyaw)
        if not self._active_mask[idx]:
            return self._empty_result(INACTIVE_HULL_CELL, yaw_error)
        status = int(self._status[idx])
        count = int(self._solution_count[idx])
        confidence = float(self._neighbor_confidence[idx])
        hard_reject_safe = (
            status == UNREACHABLE and
            confidence >= self._hard_reject_min_confidence and
            self._is_interior(x, y, z, ix, iy, iz))
        score = {
            UNKNOWN: 0.0,
            UNREACHABLE: -1.0,
            MARGINAL: 0.5,
            REACHABLE: 1.0,
        }[status]
        return QueryResult(
            status=status,
            opening_connected=bool(self._opening_connected[idx]),
            contact_seeds=self._seeds_for(
                self._contact_seeds, idx, self._contact_ik[idx], count),
            transit_seeds=self._seeds_for(
                self._transit_seeds, idx, self._transit_ik[idx], count),
            solution_count=count,
            joint_margin=float(self._joint_margin[idx]),
            manipulability=float(self._manipulability[idx]),
            neighbor_confidence=confidence,
            hard_reject_safe=hard_reject_safe,
            score=score,
            reason=STATUS_NAMES[status],
            indices=idx,
            yaw_error=yaw_error,
        )

    def is_reachable(self, x, y, z, yaw=0.0):
        """O(1) lookup: is (x,y,z,yaw) reachable?

        Args:
            x, y, z: position in container_link frame.
            yaw: orientation yaw (rad). Snapped to nearest bin.

        Returns:
            (reachable: bool, seed_joints: list[float] or None)
        """
        # Infinite yaw tolerance preserves the v1 nearest-bin behavior.
        result = self.query(x, y, z, yaw, yaw_tolerance=float("inf"))
        reachable = result.status in (MARGINAL, REACHABLE)
        seed = result.contact_seeds[0] if result.contact_seeds else None
        return reachable, seed

    def is_marginal(self, x, y, z, yaw=0.0, threshold=0.1):
        """Check if a cell is marginal by status or joint-limit margin."""
        idx = self._cell_indices(x, y, z)
        if idx is None:
            return False
        ix, iy, iz = idx
        iyaw, _ = self._nearest_yaw_index(yaw)
        cell = (ix, iy, iz, iyaw)
        if int(self._status[cell]) == MARGINAL:
            return True
        return bool(self._reachable[cell]) and \
            float(self._joint_margin[cell]) < threshold

    def filter_candidates(self, candidates, x_key="center_x",
                          y_key="center_y", z_key="center_z",
                          yaw_key="yaw"):
        """Batch-filter placement candidates.

        Annotates each candidate with:
            atlas_reachable: bool
            atlas_seed: list[float] or None
            atlas_reason: "ok" | "atlas_unreachable" | "out_of_bounds"

        Returns (reachable_list, unreachable_list).
        """
        reachable_list, unreachable_list = [], []
        for cand in candidates:
            x = cand.get(x_key, 0.0)
            y = cand.get(y_key, 0.0)
            z = cand.get(z_key, 0.0)
            yaw = cand.get(yaw_key, 0.0)
            result = self.query(x, y, z, yaw, yaw_tolerance=float("inf"))
            ok = result.status in (MARGINAL, REACHABLE)
            seed = result.contact_seeds[0] if result.contact_seeds else None
            cand["atlas_reachable"] = ok
            cand["atlas_seed"] = seed
            cand["atlas_status"] = result.status
            cand["atlas_status_name"] = STATUS_NAMES[result.status]
            cand["atlas_opening_connected"] = result.opening_connected
            cand["contact_atlas_seeds"] = result.contact_seeds
            cand["transit_atlas_seeds"] = result.transit_seeds
            cand["atlas_solution_count"] = result.solution_count
            cand["atlas_joint_margin"] = result.joint_margin
            cand["atlas_manipulability"] = result.manipulability
            cand["atlas_neighbor_confidence"] = result.neighbor_confidence
            cand["atlas_hard_reject_safe"] = result.hard_reject_safe
            cand["reachability_prior"] = result.score
            if ok:
                cand["atlas_reason"] = (
                    "atlas_marginal" if result.status == MARGINAL else "ok")
            elif result.status == UNREACHABLE:
                cand["atlas_reason"] = "atlas_unreachable"
            else:
                cand["atlas_reason"] = result.reason
            if ok:
                reachable_list.append(cand)
            else:
                unreachable_list.append(cand)
        return reachable_list, unreachable_list

    def annotate_candidates(self, candidates, x_key="center_x",
                            y_key="center_y", z_key="center_z",
                            yaw_key="yaw"):
        """Annotate candidates and return (accepted, safely_rejected).

        UNKNOWN, MARGINAL, and non-confident UNREACHABLE candidates remain in
        the accepted list for an online IK fallback.
        """
        accepted, rejected = [], []
        for cand in candidates:
            result = self.query(
                cand.get(x_key, 0.0), cand.get(y_key, 0.0),
                cand.get(z_key, 0.0), cand.get(yaw_key, 0.0))
            cand["atlas_status"] = result.status
            cand["atlas_status_name"] = STATUS_NAMES[result.status]
            cand["atlas_hard_reject_safe"] = result.hard_reject_safe
            cand["contact_atlas_seeds"] = result.contact_seeds
            cand["transit_atlas_seeds"] = result.transit_seeds
            cand["reachability_prior"] = result.score
            if result.hard_reject_safe:
                rejected.append(cand)
            else:
                accepted.append(cand)
        return accepted, rejected

    # ── Info ─────────────────────────────────────────────────────────

    @property
    def meta(self):
        return self._meta

    @property
    def yaw_bins(self):
        return list(self._yaw_bins)

    @property
    def resolution(self):
        return self._resolution

    @property
    def grid_size(self):
        return (self._nx, self._ny, self._nz, self._nyaw)

    @property
    def geometry_hash(self):
        return self._geometry.geometry_hash if self._geometry is not None else None

    def reachable_volume(self):
        """Clipped union volume of cells reachable in at least one yaw bin."""
        reachable_any_yaw = np.any(np.logical_and(
            self._active_mask,
            np.logical_or(self._status == MARGINAL, self._status == REACHABLE)),
            axis=3)
        return float(np.sum(self._cell_volumes[reachable_any_yaw]))

    def reachable_volume_ratio(self):
        if self._geometry is None:
            raise AtlasIdentityError(GEOMETRY_IDENTITY_REQUIRED)
        return self.reachable_volume() / volume(self._geometry)

    def stats(self):
        """Return atlas statistics."""
        total = int(np.count_nonzero(self._active_mask))
        reachable = int(np.count_nonzero(np.logical_and(
            self._active_mask, self._status == REACHABLE)))
        marginal = int(np.count_nonzero(np.logical_and(
            self._active_mask, self._status == MARGINAL)))
        unknown = int(np.count_nonzero(np.logical_and(
            self._active_mask, self._status == UNKNOWN)))
        unreachable = int(np.count_nonzero(np.logical_and(
            self._active_mask, self._status == UNREACHABLE)))
        result = {
            "total_cells": total,
            "allocated_cells": int(self._status.size),
            "inactive_cells": int(self._status.size - total),
            "reachable_cells": reachable,
            "marginal_cells": marginal,
            "unknown_cells": unknown,
            "unreachable_cells": unreachable,
            "reachability_rate": (reachable + marginal) / max(1, total),
            "grid_size": list(self.grid_size),
            "resolution": self._resolution,
            "yaw_bins": self._yaw_bins,
            "nseed": self._nseed,
        }
        if self._geometry is not None:
            result.update({
                "geometry_schema_version": self._geometry.schema_version,
                "geometry_hash": self._geometry.geometry_hash,
                "usable_hull_volume": volume(self._geometry),
                "reachable_volume": self.reachable_volume(),
                "reachable_volume_ratio": self.reachable_volume_ratio(),
            })
        return result
