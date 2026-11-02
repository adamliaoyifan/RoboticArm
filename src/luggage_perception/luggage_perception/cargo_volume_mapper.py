#!/usr/bin/env python3
"""Cargo interior voxel occupancy mapper (unknown / free / occupied)."""

from __future__ import division

import math

import numpy as np
from copy import deepcopy
from array import array

from luggage_description.container_geometry import (
    cuboid_from_inner_size,
    normalize_descriptor,
)
from luggage_perception.voxel_log_odds import LogOddsGrid
from luggage_description.cargo_grid_contract import CargoGridSpec
from luggage_perception.cargo_grid_projection import active_mask, surface_columns, occupancy_counts

UNKNOWN = 0
FREE = 1
OCCUPIED = 2

SOURCE_NONE = 0
SOURCE_SENSOR = 1
SOURCE_GEOMETRY = 2


def verify_anchor_delta_m(slot_center, verified_center):
    """Euclidean distance between an asserted slot and a measured pose.

    The AddPlacedBoxV2 anchor gate: both points are container_link box
    centers. ``None`` (unmeasured anchor) is distinguished from a distance
    by the caller — a missing measurement must not silently read as 0.
    """
    if slot_center is None or verified_center is None:
        return None
    return math.sqrt(sum(
        (float(slot_center[axis]) - float(verified_center[axis])) ** 2
        for axis in range(3)))


def _clip_half_plane(polygon, origin, normal):
    """Sutherland-Hodgman clip keeping (p - origin) . normal >= 0."""
    out = []
    count = len(polygon)
    for index in range(count):
        point = polygon[index]
        previous = polygon[index - 1]
        current = ((point[0] - origin[0]) * normal[0]
                   + (point[1] - origin[1]) * normal[1])
        before = ((previous[0] - origin[0]) * normal[0]
                  + (previous[1] - origin[1]) * normal[1])
        inside_c, inside_p = current >= -1e-12, before >= -1e-12
        if inside_c != inside_p:
            t = before / (before - current)
            out.append((previous[0] + t * (point[0] - previous[0]),
                        previous[1] + t * (point[1] - previous[1])))
        if inside_c:
            out.append(point)
    return out


def _rect_footprint_overlap(corners, xmin, ymin, xmax, ymax):
    """Positive-intersection area between a CCW footprint and a cell rect.

    Returns 0.0 for measure-zero boundary contact (half-open rule: a
    touching edge or corner alone never occupies the cell).
    """
    polygon = [(xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)]
    for index in range(4):
        a, b = corners[index], corners[(index + 1) % 4]
        edge = (b[0] - a[0], b[1] - a[1])
        polygon = _clip_half_plane(polygon, a, (-edge[1], edge[0]))
        if not polygon:
            return 0.0
    if len(polygon) < 3:
        return 0.0
    total = 0.0
    for index, point in enumerate(polygon):
        nxt = polygon[(index + 1) % len(polygon)]
        total += point[0] * nxt[1] - nxt[0] * point[1]
    return abs(total) / 2.0


def normalize_ledger_slot(center_xyz, yaw, size, source_frame="world",
                          container_yaw=0.0, container_translation=(0.0, 0.0, 0.0)):
    """Single F19 normalization shared by add/remove/readback commits.

    ``source_frame='container_link'`` (v2 slot records): the pose already
    carries container coordinates — identity, no second yaw subtraction.
    ``source_frame='world'`` is the one declared v1 shim for the existing
    orchestrator: the historical inverse container transform (translation
    subtracted, yaw un-rotated). Any other source frame is a refusal, not a
    guess. Returns ``(center_container, yaw_container, size_wdh)``.
    """
    center = [float(v) for v in center_xyz]
    size = [float(v) for v in size]
    yaw = float(yaw)
    if not all(math.isfinite(v) for v in center + size) or any(
            v <= 0.0 for v in size) or not math.isfinite(yaw):
        raise ValueError("ledger slot needs finite xyz/yaw and positive size")
    if source_frame == "container_link":
        return center, yaw, size
    if source_frame != "world":
        raise ValueError("ledger slot source frame %r is not declared"
                         % (source_frame,))
    tx, ty, tz = (float(v) for v in container_translation)
    cos_y = math.cos(-float(container_yaw))
    sin_y = math.sin(-float(container_yaw))
    dx, dy = center[0] - tx, center[1] - ty
    return ([dx * cos_y - dy * sin_y, dx * sin_y + dy * cos_y, center[2] - tz],
            yaw - float(container_yaw), size)


# ---------------------------------------------------------------------------
# Fusion kernels (module-level pure functions; @njit-ready for the B step).
# Each takes ONLY numpy arrays / scalars -- no mapper state -- so numba
# can compile them directly. The mapper's integrate_points calls these
# with its own parameters unpacked.


def _kernel_world_to_local(points, center, cos_neg_yaw, sin_neg_yaw,
                           cos_yaw, sin_yaw):
    """Batch world->local. Same formula as _world_to_local.

    The caller must pass cos(-yaw), sin(-yaw), cos(yaw), sin(yaw) each
    computed from the raw yaw exactly as the scalar path does."""
    dx = points[:, 0] - center[0]
    dy = points[:, 1] - center[1]
    dz = points[:, 2] - center[2]
    # Identical expression order to the scalar _world_to_local:
    #   local_x = cos(-yaw) * dx - sin(-yaw) * dy
    #   local_y = sin(-yaw) * dx + cos(-yaw) * dy
    lx = cos_neg_yaw * dx - sin_neg_yaw * dy
    ly = sin_neg_yaw * dx + cos_neg_yaw * dy
    return np.column_stack((lx, ly, dz))


def _kernel_local_to_voxel(local, resolution, half_l, half_w, half_h,
                           nx, ny, nz, active):
    """Batch local->voxel. Returns (valid_mask, flat_index).

    Mirrors _local_to_voxel exactly: boundary check -> int() -> clamp ->
    active check. int() on positive values == trunc.
    """
    inside = ((np.abs(local[:, 0]) <= half_l)
              & (np.abs(local[:, 1]) <= half_w)
              & (np.abs(local[:, 2]) <= half_h))
    ix = np.trunc((local[:, 0] + half_l) / resolution).astype(np.int64)
    iy = np.trunc((local[:, 1] + half_w) / resolution).astype(np.int64)
    iz = np.trunc((local[:, 2] + half_h) / resolution).astype(np.int64)
    ix = np.clip(ix, 0, nx - 1)
    iy = np.clip(iy, 0, ny - 1)
    iz = np.clip(iz, 0, nz - 1)
    flat = ix + nx * (iy + ny * iz)
    valid = inside.copy()
    if active is not None:
        if not isinstance(active, np.ndarray):
            active = np.frombuffer(bytes(active), dtype=np.uint8)
            active = active.astype(bool)
        valid &= active[flat]
    return valid, flat


def _kernel_raycast(origins, ends, resolution, center,
                    cos_neg_yaw, sin_neg_yaw, cos_yaw, sin_yaw,
                    half_l, half_w, half_h, nx, ny, nz, active):
    """Batch ray sampling. Returns concatenated flat indices (with dupes).

    Hybrid approach: per-ray AABB clipping + step-range computation
    (cheap Python loop, ~2000 iterations), then ONE batch coordinate
    transform + voxel lookup for ALL samples concatenated (the
    expensive part). Same math as the scalar _raycast_indices
    (world-space interpolation, then world->local->voxel).
    """
    if len(ends) == 0:
        return np.empty(0, dtype=np.int64), 0, 0
    o = origins
    o_local = _kernel_world_to_local(
        origins.reshape(1, 3), center, cos_neg_yaw, sin_neg_yaw,
        cos_yaw, sin_yaw)[0]
    e_local = _kernel_world_to_local(
        ends, center, cos_neg_yaw, sin_neg_yaw, cos_yaw, sin_yaw)
    halves = (half_l, half_w, half_h)
    all_t = []
    all_ray = []
    for i in range(len(ends)):
        ol = o_local
        el = e_local[i]
        dist = math.sqrt(
            (ends[i, 0] - o[0]) ** 2
            + (ends[i, 1] - o[1]) ** 2
            + (ends[i, 2] - o[2]) ** 2)
        if dist < resolution:
            continue
        steps = int(dist / (resolution * 0.5))
        if steps < 1:
            continue
        near, far = 0.0, 1.0
        clipped = True
        for axis in range(3):
            start_a, end_a, half = ol[axis], el[axis], halves[axis]
            delta = end_a - start_a
            if delta == 0:
                if abs(start_a) > half:
                    clipped = False
                    break
                continue
            enter, leave = sorted(
                ((-half - start_a) / delta, (half - start_a) / delta))
            near, far = max(near, enter), min(far, leave)
            if near > far:
                clipped = False
                break
        if not clipped:
            continue
        first = max(0, int(math.floor(near * steps)) - 1)
        last = min(steps, int(math.ceil(far * steps)) + 2)
        if last <= first:
            continue
        t = np.arange(first, last, dtype=np.float64) / steps
        all_t.append(t)
        all_ray.append(np.full(len(t), i, dtype=np.int64))
    if not all_t:
        return np.empty(0, dtype=np.int64), 0, 0
    ray_intersecting = len(all_t)  # one entry per non-clipped ray
    samples_visited = sum(len(t) for t in all_t)
    t_all = np.concatenate(all_t)
    ray_all = np.concatenate(all_ray)
    # World-space interpolation for all samples at once
    sel_ends = ends[ray_all]
    wx = o[0] + (sel_ends[:, 0] - o[0]) * t_all
    wy = o[1] + (sel_ends[:, 1] - o[1]) * t_all
    wz = o[2] + (sel_ends[:, 2] - o[2]) * t_all
    world_samples = np.column_stack((wx, wy, wz))
    local_samples = _kernel_world_to_local(
        world_samples, center, cos_neg_yaw, sin_neg_yaw,
        cos_yaw, sin_yaw)
    valid, flat = _kernel_local_to_voxel(
        local_samples, resolution, half_l, half_w, half_h, nx, ny, nz,
        active)
    return flat[valid], ray_intersecting, samples_visited


# JIT kernels (B round): used INSTEAD of the numpy versions above
# when numba is available; graceful fallback keeps the numpy path.
try:
    from luggage_perception.fusion_kernels_jit import (
        HAS_NUMBA, _jit_world_to_local, _jit_local_to_voxel)
    if HAS_NUMBA:
        _USE_JIT = True
    else:
        _USE_JIT = False
except ImportError:
    _USE_JIT = False


class CargoVolumeMapper:
    """Axis-aligned voxel grid in container-local frame (yaw-aligned with base)."""

    def __init__(
        self, inner_size, center_base, yaw, resolution, occupancy_params=None,
        max_raycast_points=None, hull_local_inside=None,
        geometry_descriptor=None, geometry_hash="",
    ):
        # Identity of the container hull this map describes. Consumers join
        # the map with their own kernel descriptor and must fail closed when
        # the hashes differ (docs/architecture/container_geometry.md).
        self.geometry_descriptor = dict(geometry_descriptor or {})
        self.geometry_hash = str(
            geometry_hash or self.geometry_descriptor.get("geometry_hash", ""))
        self._grid_spec = CargoGridSpec.create(inner_size, resolution)
        self.inner_l, self.inner_w, self.inner_h = [float(v) for v in inner_size]
        self.center = [float(v) for v in center_base]
        self.yaw = float(yaw)
        self.nx, self.ny, self.nz = self._grid_spec.dimensions_xyz
        self.occupancy_params = dict(occupancy_params or {})
        # Cap on the number of points used for free-space raycasting (the
        # per-point ray walk is the integrate bottleneck). None = no cap.
        self.max_raycast_points = (
            None if max_raycast_points is None else max(1, int(max_raycast_points))
        )
        self._hull_local_inside = hull_local_inside
        self._hull = self._load_hull()
        stamped = self._hull.descriptor()
        self.geometry_descriptor = stamped
        self.geometry_hash = str(stamped["geometry_hash"])
        self._voxel_delta = None
        self._query_cache = None
        self._query_read_owner = None
        self._surface_cache = None
        self._active = None
        self._grid = None
        self._occupancy = None
        self._revision = 0
        self._depth_ray_phase = 0
        self._placed_boxes = []
        self._build_active_mask()
        self.reset()

    @property
    def resolution(self):
        """Frozen grid spacing; only construction may establish it."""
        return self._grid_spec.resolution_m

    def _load_hull(self):
        if self.geometry_descriptor:
            try:
                return normalize_descriptor(self.geometry_descriptor)
            except (TypeError, ValueError, KeyError) as exc:
                raise ValueError(
                    "invalid geometry_descriptor: %s" % exc) from exc
        if self.geometry_hash:
            raise ValueError(
                "geometry_hash without geometry_descriptor is invalid")
        floor_z = self.center[2] - 0.5 * self.inner_h
        return cuboid_from_inner_size(
            [self.inner_l, self.inner_w, self.inner_h], floor_z=floor_z)

    def reset(self, preserve_placed=False):
        saved = list(self._placed_boxes) if preserve_placed else []
        self._placed_boxes = saved
        self._clear_grid()
        for placed in saved:
            self._rasterize_placed_box(
                placed["center"], placed["size"], placed["yaw"])
        self._revision += 1

    @property
    def map_revision(self):
        """Public read of the single revision counter (B1 snapshot identity
        reads this; nothing outside the mapper may keep a second counter)."""
        return self._revision

    def note_external_change(self):
        """Advance the revision for a state change that mutated no voxel
        (a confirmation transition: late fact event, container pose note).
        Without this, a snapshot rebuilt after such a change would carry a
        different confirmation state at an identical revision — readers
        could not distinguish it from a stale snapshot."""
        self._revision += 1
        return self._revision

    def _observe_voxels(self, indices):
        """Read-only evidence notification; never changes mutation order."""
        self._query_cache = None
        self._query_read_owner = None
        self._surface_cache = None
        if self._voxel_delta is not None:
            self._voxel_delta.record_before(indices)

    def _clear_grid(self):
        """Clear voxel state without changing the commit revision."""
        self._query_cache = None
        self._query_read_owner = None
        self._surface_cache = None
        if self._voxel_delta is not None:
            self._voxel_delta.record_reset()
        total = self.nx * self.ny * self.nz
        self._occupancy = LogOddsGrid(total, **self.occupancy_params)
        self._grid = bytearray(self._occupancy.states())
        self._source = bytearray(total)
        self._labels = array("q", [0]) * total
        self._instance_ids = array("q", [0]) * total
        # Independent sensor evidence: a per-voxel shadow log-odds value
        # the sensor's own hits/misses accumulate, plus its latest label
        # and instance. The shadow is updated unconditionally — geometry
        # locks protect the live grid, never this ledger — so retracting a
        # committed box restores the sensor's *latest* word exactly as if
        # the geometry had never been there (control-equivalence).
        self._sensor_seen = bytearray(total)
        self._sensor_values = array("d", [0.0]) * total
        self._sensor_labels = array("q", [0]) * total
        self._sensor_instance_ids = array("q", [0]) * total

    def _build_active_mask(self):
        """Index the enclosing AABB; deactivate voxels outside the 7-face hull."""
        self._active = active_mask(self)

    def _index(self, ix, iy, iz):
        return ix + self.nx * (iy + self.ny * iz)

    def _world_to_local(self, x, y, z):
        dx = x - self.center[0]
        dy = y - self.center[1]
        dz = z - self.center[2]
        local_x = math.cos(-self.yaw) * dx - math.sin(-self.yaw) * dy
        local_y = math.sin(-self.yaw) * dx + math.cos(-self.yaw) * dy
        local_z = dz
        return local_x, local_y, local_z

    def _local_to_voxel(self, local_x, local_y, local_z):
        half_l = self.inner_l * 0.5
        half_w = self.inner_w * 0.5
        half_h = self.inner_h * 0.5
        if (
            abs(local_x) > half_l
            or abs(local_y) > half_w
            or abs(local_z) > half_h
        ):
            return None
        ix = int((local_x + half_l) / self.resolution)
        iy = int((local_y + half_w) / self.resolution)
        iz = int((local_z + half_h) / self.resolution)
        ix = min(max(ix, 0), self.nx - 1)
        iy = min(max(iy, 0), self.ny - 1)
        iz = min(max(iz, 0), self.nz - 1)
        if self._active is not None and not self._active[self._index(ix, iy, iz)]:
            return None
        return ix, iy, iz

    def _voxel_center_local(self, ix, iy, iz):
        half_l = self.inner_l * 0.5
        half_w = self.inner_w * 0.5
        half_h = self.inner_h * 0.5
        return (
            -half_l + (ix + 0.5) * self.resolution,
            -half_w + (iy + 0.5) * self.resolution,
            -half_h + (iz + 0.5) * self.resolution,
        )

    def _local_to_world(self, local_x, local_y, local_z):
        wx = self.center[0] + math.cos(self.yaw) * local_x - math.sin(self.yaw) * local_y
        wy = self.center[1] + math.sin(self.yaw) * local_x + math.cos(self.yaw) * local_y
        wz = self.center[2] + local_z
        return wx, wy, wz

    def mark_occupied_world(self, x, y, z, source=SOURCE_SENSOR,
                            label=0, instance_id=0):
        local = self._world_to_local(x, y, z)
        vox = self._local_to_voxel(*local)
        if vox is None:
            return
        self.mark_occupied_voxel(
            *vox, source=source, label=label, instance_id=instance_id)

    def mark_occupied_voxel(self, ix, iy, iz, source=SOURCE_SENSOR,
                            label=0, instance_id=0):
        """Mark one exact voxel index (ST04 exact rasterization path)."""
        if not (0 <= ix < self.nx and 0 <= iy < self.ny
                and 0 <= iz < self.nz):
            return
        idx = self._index(ix, iy, iz)
        self._observe_voxels((idx,))
        if source == SOURCE_GEOMETRY:
            self._grid[idx] = self._occupancy.force_occupied(
                idx, lock_geometry=True
            )
        else:
            self._grid[idx] = self._occupancy.apply_hit(idx)
            self._record_sensor_hit(idx, label, instance_id)
        if self._source[idx] == SOURCE_NONE:
            self._source[idx] = source
        # Sensor observations overwrite geometry-inferred labels.
        if source == SOURCE_SENSOR or self._labels[idx] == 0:
            self._labels[idx] = int(label)
            self._instance_ids[idx] = int(instance_id)

    def _record_sensor_hit(self, idx, label=0, instance_id=0):
        """A sensor hit is testimony: accumulate the shadow ledger.

        The shadow accumulates the sensor-only log-odds trajectory with
        the same increments and clamps as the live grid. Because geometry
        writes never touch it, restoring it after a geometry retraction
        reproduces exactly the state the sensor alone would have built.
        """
        occupancy = self._occupancy
        self._sensor_values[idx] = min(
            occupancy.maximum, max(occupancy.minimum,
                                   self._sensor_values[idx]
                                   + occupancy.hit_increment))
        self._sensor_seen[idx] = True
        self._sensor_labels[idx] = int(label)
        self._sensor_instance_ids[idx] = int(instance_id)

    def _record_sensor_miss(self, idx):
        """A miss is testimony too: the lock protects the grid, not the
        ledger. The sensor's latest word is recorded unconditionally, so
        retracting overlaid geometry restores it (never a stale hit)."""
        occupancy = self._occupancy
        # miss_increment is negative log-odds; accumulate exactly like
        # LogOddsGrid.apply_miss (which the lock may bypass, the ledger
        # never).
        self._sensor_values[idx] = max(
            occupancy.minimum, min(occupancy.maximum,
                                   self._sensor_values[idx]
                                   + occupancy.miss_increment))
        self._sensor_seen[idx] = True
        if occupancy.classify_value(self._sensor_values[idx]) == FREE:
            self._sensor_labels[idx] = 0
            self._sensor_instance_ids[idx] = 0

    def mark_free_world(self, x, y, z):
        local = self._world_to_local(x, y, z)
        vox = self._local_to_voxel(*local)
        if vox is None:
            return
        idx = self._index(*vox)
        self._observe_voxels((idx,))
        self._grid[idx] = self._occupancy.apply_miss(idx)
        self._record_sensor_miss(idx)
        if self._grid[idx] == FREE and self._source[idx] != SOURCE_GEOMETRY:
            self._source[idx] = SOURCE_NONE
            self._labels[idx] = 0
            self._instance_ids[idx] = 0

    def _raycast_indices(self, origin, hit, diagnostics=None):
        ox, oy, oz = origin
        hx, hy, hz = hit
        dist = math.sqrt((hx - ox) ** 2 + (hy - oy) ** 2 + (hz - oz) ** 2)
        if dist < self.resolution:
            return set()
        steps = int(dist / (self.resolution * 0.5))
        if steps < 1:
            return set()
        # Clip only the loop bounds, preserving the original t=step/steps.
        # AABB is conservative for the legacy active-voxel hull semantics.
        local_origin = self._world_to_local(ox, oy, oz)
        local_end = self._world_to_local(hx, hy, hz)
        near, far = 0.0, 1.0
        for start, end, half in zip(local_origin, local_end,
                                    (self.inner_l*.5, self.inner_w*.5, self.inner_h*.5)):
            delta = end-start
            if delta == 0:
                if abs(start) > half:
                    return set()
                continue
            enter, leave = sorted(((-half-start)/delta, (half-start)/delta))
            near, far = max(near, enter), min(far, leave)
            if near > far:
                return set()
        # One sample of padding absorbs roundoff near the slab intersection;
        # the unchanged voxel predicate rejects padding outside the map.
        first = max(0, int(math.floor(near*steps))-1)
        last = min(steps, int(math.ceil(far*steps))+2)
        if diagnostics is not None:
            diagnostics["ray_intersecting"] += 1
            diagnostics["samples_visited"] += max(0, last-first)
        indices = set()
        for step in range(first, last):
            t = float(step) / float(steps)
            px = ox + (hx - ox) * t
            py = oy + (hy - oy) * t
            pz = oz + (hz - oz) * t
            local = self._world_to_local(px, py, pz)
            vox = self._local_to_voxel(*local)
            if vox is not None:
                indices.add(self._index(*vox))
        return indices

    def integrate_points(self, points, origin=None, labels=None, instance_ids=None,
                         ray_points=None, _vectorized=True):
        """Integrate observed points into the voxel grid.

        Args:
            points: list of (x, y, z) in base frame.
            origin: optional camera position for free-space raycasting.
            labels: optional parallel list of per-point semantic class labels.
            instance_ids: optional parallel list of per-point instance IDs.
            _vectorized: use the batch-numpy kernel path (oracle-tested
                to produce identical results to the original loop).
        """
        if _vectorized:
            return self._integrate_points_vectorized(
                points, origin, labels, instance_ids, ray_points)
        hit_records = {}
        diagnostics = dict(rays_selected=0, ray_intersecting=0, samples_visited=0)
        miss_indices = set()
        for i, pt in enumerate(points):
            x, y, z = pt[0], pt[1], pt[2]
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            lbl = int(labels[i]) if labels is not None else 0
            iid = int(instance_ids[i]) if instance_ids is not None else 0
            local = self._world_to_local(x, y, z)
            vox = self._local_to_voxel(*local)
            if vox is not None:
                hit_records[self._index(*vox)] = (x, y, z, lbl, iid)

        if origin is not None:
            ox, oy, oz = origin
            # The free-space ray walk is O(points * ray_steps) in pure Python;
            # subsample the rays (not the hit marks) when the cloud is large so
            # integrate stays interactive. The coarse voxel grid tolerates
            # fewer rays without losing free-space coverage.
            ray_points = points if ray_points is None else ray_points
            if self.max_raycast_points and len(ray_points) > self.max_raycast_points:
                stride = max(1, len(ray_points) // self.max_raycast_points)
                ray_points = ray_points[::stride]
            for pt in ray_points:
                x, y, z = pt[0], pt[1], pt[2]
                if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                    continue
                diagnostics["rays_selected"] += 1
                miss_indices.update(
                    self._raycast_indices((ox, oy, oz), (x, y, z), diagnostics)
                )
        miss_indices.difference_update(hit_records)
        self._observe_voxels(miss_indices)
        self._observe_voxels(hit_records)
        for idx in miss_indices:
            self._grid[idx] = self._occupancy.apply_miss(idx)
            self._record_sensor_miss(idx)
            if self._grid[idx] == FREE and self._source[idx] != SOURCE_GEOMETRY:
                self._source[idx] = SOURCE_NONE
                self._labels[idx] = 0
                self._instance_ids[idx] = 0
        for x, y, z, lbl, iid in hit_records.values():
            self.mark_occupied_world(
                x, y, z, label=lbl, instance_id=iid
            )
        if hit_records or miss_indices:
            self._revision += 1
        return diagnostics

    def _integrate_points_vectorized(self, points, origin=None,
                                     labels=None, instance_ids=None,
                                     ray_points=None):
        """Batch-numpy path: identical results to the original loop.

        Kernels are module-level pure functions (numba-ready); this
        method unpacks mapper state into scalar/array parameters,
        calls the kernels, then applies grid/ledger updates in the
        same order as the original (misses first, then hits).
        """
        diagnostics = dict(rays_selected=0, ray_intersecting=0,
                           samples_visited=0)
        resolution = self.resolution
        half_l = self.inner_l * 0.5
        half_w = self.inner_w * 0.5
        half_h = self.inner_h * 0.5
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        cos_neg = math.cos(-self.yaw)
        sin_neg = math.sin(-self.yaw)
        active = self._active
        nx, ny, nz = self.nx, self.ny, self.nz

        # --- Hit marking (batch) ---
        pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        finite = np.isfinite(pts).all(axis=1)
        # JIT path: use compiled kernels with bool-array active mask
        if _USE_JIT:
            active_bool = (np.frombuffer(bytes(active), dtype=np.uint8)
                           .astype(np.bool_) if active is not None
                           else None)
            local = _jit_world_to_local(
                pts[finite], np.asarray(self.center, dtype=np.float64),
                cos_neg, sin_neg)
            valid_hit, hit_flat = _jit_local_to_voxel(
                local, resolution, half_l, half_w, half_h,
                nx, ny, nz, active_bool)
        else:
            local = _kernel_world_to_local(
                pts[finite], self.center, cos_neg, sin_neg,
                cos_yaw, sin_yaw)
            valid_hit, hit_flat = _kernel_local_to_voxel(
                local, resolution, half_l, half_w, half_h, nx, ny, nz,
                active)

        # Build hit_records: flat_index -> (x, y, z, label, instance)
        # Later points overwrite earlier ones (same as dict assignment
        # in the original loop's insertion order).
        hit_records = {}
        valid_indices = np.flatnonzero(finite)[valid_hit]
        for i in range(len(valid_indices)):
            src = valid_indices[i]
            pt = points[src]
            lbl = int(labels[src]) if labels is not None else 0
            iid = int(instance_ids[src]) if instance_ids is not None else 0
            hit_records[int(hit_flat[i])] = (pt[0], pt[1], pt[2], lbl, iid)

        # --- Ray casting (batch) ---
        miss_indices = set()
        if origin is not None:
            ray_pts = ray_points if ray_points is not None else points
            if self.max_raycast_points and \
                    len(ray_pts) > self.max_raycast_points:
                stride = max(1, len(ray_pts) // self.max_raycast_points)
                ray_pts = ray_pts[::stride]
            ray_arr = np.asarray(ray_pts, dtype=np.float64).reshape(-1, 3)
            ray_finite = ray_arr[np.isfinite(ray_arr).all(axis=1)]
            diagnostics["rays_selected"] = len(ray_finite)
            origin_arr = np.asarray(origin, dtype=np.float64)
            miss_flat, ri, sv = _kernel_raycast(
                origin_arr, ray_finite, resolution, self.center,
                cos_neg, sin_neg, cos_yaw, sin_yaw,
                half_l, half_w, half_h, nx, ny, nz, active)
            diagnostics['ray_intersecting'] = ri
            diagnostics['samples_visited'] = sv
            miss_indices = set(int(x) for x in miss_flat)

        miss_indices.difference_update(hit_records)

        # --- Evidence notification BEFORE mutation (same contract as
        # the scalar loop, lines above: _observe_voxels precedes the
        # grid writes). The batch kernels used to notify AFTER the
        # state change, so record_before captured the NEW evidence as
        # "old" -> before==after -> empty voxel deltas -> every
        # derived index/export/replica was blind to vectorized fusion
        # writes (caught by test_500_mutations_per_seed on the merged
        # tree 2026-10-27).
        self._observe_voxels(miss_indices)
        self._observe_voxels(hit_records)

        # --- Batch grid updates: misses then hits, same order ---
        # All bytearray/array fields support the buffer protocol ->
        # np.frombuffer gives zero-copy WRITABLE views for vectorized
        # ops. Only _occupancy.values is a Python list (read with
        # fromiter, write with loop -- still far cheaper than 158k
        # per-cell function calls).
        self._batch_miss_update(miss_indices)
        self._batch_hit_update(hit_records)

        if hit_records or miss_indices:
            self._revision += 1
        return diagnostics

    def _batch_miss_update(self, miss_indices):
        """Vectorized miss update: log-odds + grid + sensor ledger.

        Produces IDENTICAL results to the per-cell loop
        (apply_miss + _record_sensor_miss + source/label reset).

        Geometry-lock semantics (gen3 I3 fix, review_three_issues): the
        scalar apply_miss BYPASSES locked cells entirely -- the live
        occupancy value is protected while the sensor shadow ledger
        still records the miss (so removing a placed box restores the
        true sensor view). The batch kernel used to decrement every
        index, so repeated free-space observations could erode a locked
        box's occupancy to FREE -- a correctness defect, not rounding
        noise (locked_miss_result.json: 3.5 -> FREE after 10 misses).
        """
        if not miss_indices:
            return
        occ = self._occupancy
        # Sorted for deterministic write order (same as original loop).
        idx_arr = np.fromiter(sorted(miss_indices), dtype=np.int64,
                              count=len(miss_indices))
        # -- geometry lock mask (protects the LIVE occupancy only).
        # geometry_locked is a Python list -> gather per index.
        locked = np.fromiter(
            map(occ.geometry_locked.__getitem__, idx_arr),
            dtype=np.bool_, count=len(idx_arr))
        # -- occupancy values (Python list: read gen, compute numpy, write loop)
        values = occ.values
        if isinstance(values, list):
            old_vals = np.fromiter(map(values.__getitem__, idx_arr),
                                    dtype=np.float64, count=len(idx_arr))
        else:
            old_vals = np.frombuffer(values, dtype=np.float64)[idx_arr]
        if occ.enabled:
            candidate = np.maximum(
                occ.minimum,
                np.minimum(occ.maximum,
                           old_vals + occ.miss_increment))
        else:
            # apply_miss disabled mode: unlocked cells go to minimum.
            candidate = np.full(len(idx_arr), occ.minimum,
                                dtype=np.float64)
        new_vals = np.where(locked, old_vals, candidate)
        if isinstance(values, list):
            for j in range(len(idx_arr)):
                values[int(idx_arr[j])] = float(new_vals[j])
        else:
            values_arr = np.frombuffer(values, dtype=np.float64)
            values_arr[idx_arr] = new_vals
        # -- grid classification (bytearray -> zero-copy view)
        grid_view = np.frombuffer(self._grid, dtype=np.uint8)
        grid_new = np.where(
            new_vals >= occ.occupied_threshold, np.uint8(OCCUPIED),
            np.where(new_vals <= occ.free_threshold, np.uint8(FREE),
                     np.uint8(UNKNOWN)))
        grid_view[idx_arr] = grid_new
        # -- sensor values (array('d') -> zero-copy view)
        sv_view = np.frombuffer(self._sensor_values, dtype=np.float64)
        sv_old = sv_view[idx_arr]
        sv_new = np.maximum(occ.minimum,
                             np.minimum(occ.maximum,
                                        sv_old + occ.miss_increment))
        sv_view[idx_arr] = sv_new
        # -- sensor seen (bytearray)
        ss_view = np.frombuffer(self._sensor_seen, dtype=np.uint8)
        ss_view[idx_arr] = 1
        # -- conditional label/instance reset (sensor FREE)
        sensor_free = sv_new <= occ.free_threshold
        reset_idx = idx_arr[sensor_free]
        if len(reset_idx):
            sl_view = np.frombuffer(self._sensor_labels, dtype=np.int64)
            si_view = np.frombuffer(self._sensor_instance_ids,
                                    dtype=np.int64)
            sl_view[reset_idx] = 0
            si_view[reset_idx] = 0
        # -- source/label/instance reset (grid FREE and not geometry)
        grid_free = grid_new == FREE
        src_view = np.frombuffer(self._source, dtype=np.uint8)
        not_geom = src_view[idx_arr] != SOURCE_GEOMETRY
        reset2 = idx_arr[grid_free & not_geom]
        if len(reset2):
            src_view[reset2] = SOURCE_NONE
            lbl_view = np.frombuffer(self._labels, dtype=np.int64)
            inst_view = np.frombuffer(self._instance_ids,
                                      dtype=np.int64)
            lbl_view[reset2] = 0
            inst_view[reset2] = 0
        # NOTE: no _observe_voxels here -- the orchestrator notifies
        # the delta builder BEFORE mutation (record_before contract).

    def _batch_hit_update(self, hit_records):
        """Vectorized hit update: log-odds + grid + sensor + labels.

        Produces IDENTICAL results to per-cell mark_occupied_world
        (apply_hit + _record_sensor_hit + source/label/instance).
        """
        if not hit_records:
            return
        occ = self._occupancy
        # hit_records: flat_idx -> (x, y, z, label, instance_id)
        # Process in dict insertion order (same as original loop).
        idx_list = list(hit_records.keys())
        idx_arr = np.array(idx_list, dtype=np.int64)
        labels_arr = np.array(
            [hit_records[i][3] for i in idx_list], dtype=np.int64)
        instances_arr = np.array(
            [hit_records[i][4] for i in idx_list], dtype=np.int64)
        # -- occupancy values (Python list)
        values = occ.values
        if isinstance(values, list):
            old_vals = np.fromiter(map(values.__getitem__, idx_arr),
                                    dtype=np.float64, count=len(idx_arr))
        else:
            old_vals = np.frombuffer(values, dtype=np.float64)[idx_arr]
        new_vals = np.maximum(occ.minimum,
                              np.minimum(occ.maximum,
                                         old_vals + occ.hit_increment))
        if isinstance(values, list):
            for j in range(len(idx_arr)):
                values[int(idx_arr[j])] = float(new_vals[j])
        else:
            values_arr = np.frombuffer(values, dtype=np.float64)
            values_arr[idx_arr] = new_vals
        # -- grid
        grid_view = np.frombuffer(self._grid, dtype=np.uint8)
        grid_new = np.where(
            new_vals >= occ.occupied_threshold, np.uint8(OCCUPIED),
            np.where(new_vals <= occ.free_threshold, np.uint8(FREE),
                     np.uint8(UNKNOWN)))
        grid_view[idx_arr] = grid_new
        # -- sensor values
        sv_view = np.frombuffer(self._sensor_values, dtype=np.float64)
        sv_old = sv_view[idx_arr]
        sv_new = np.minimum(occ.maximum,
                             np.maximum(occ.minimum,
                                        sv_old + occ.hit_increment))
        sv_view[idx_arr] = sv_new
        # -- sensor seen
        ss_view = np.frombuffer(self._sensor_seen, dtype=np.uint8)
        ss_view[idx_arr] = 1
        # -- sensor labels / instances (unconditional for hits)
        sl_view = np.frombuffer(self._sensor_labels, dtype=np.int64)
        si_view = np.frombuffer(self._sensor_instance_ids,
                                dtype=np.int64)
        sl_view[idx_arr] = labels_arr
        si_view[idx_arr] = instances_arr
        # -- source: set if currently NONE
        src_view = np.frombuffer(self._source, dtype=np.uint8)
        is_none = src_view[idx_arr] == SOURCE_NONE
        set_src = idx_arr[is_none]
        if len(set_src):
            src_view[set_src] = SOURCE_SENSOR
        # -- labels: sensor source or label==0 -> write
        lbl_view = np.frombuffer(self._labels, dtype=np.int64)
        inst_view = np.frombuffer(self._instance_ids,
                                  dtype=np.int64)
        old_labels = lbl_view[idx_arr]
        write_lbl = (old_labels == 0)  # source is SENSOR for all hits
        set_lbl = idx_arr[write_lbl]
        if len(set_lbl):
            mask_positions = np.flatnonzero(write_lbl)
            lbl_view[set_lbl] = labels_arr[mask_positions]
            inst_view[set_lbl] = instances_arr[mask_positions]
        # NOTE: no _observe_voxels here -- the orchestrator notifies
        # the delta builder BEFORE mutation (record_before contract).

    def integrate_depth_view(self, points, origin):
        """Integrate full-depth returns, keeping model walls out of cargo hits.

        This owner-side operation consumes measured points, never synthetic
        floor/geometry rays. Rejected or terminal bands remain unobserved.
        It does not stamp/confirm the map; the settled-view adapter owns that.
        """
        from luggage_perception.depth_view_rays import admit_depth_ray
        mid_z=0.5*(self._hull.floor_z+self._hull.ceiling_z)
        def hull_point(point):
            x,y,z=self._world_to_local(*point)
            return x,y,z+mid_z
        def world_point(point):
            return self._local_to_world(point[0],point[1],point[2]-mid_z)
        camera=hull_point(origin)
        rays=[];hits={};rejected=0
        for point in points:
            ray=admit_depth_ray(self._hull,camera,hull_point(point),math.sqrt(3.)*self.resolution)
            if ray is None:
                rejected+=1
                continue
            rays.append(world_point(ray.free_end))
            if ray.obstacle is not None:
                hit=world_point(ray.obstacle)
                voxel=self._local_to_voxel(*self._world_to_local(*hit))
                if voxel is not None:hits[self._index(*voxel)]=hit
        rays_admitted = len(rays)
        if self.max_raycast_points and len(rays)>self.max_raycast_points:
            stride=int(math.ceil(len(rays)/self.max_raycast_points))
            # A fixed prefix repeats the same blind spots in ordered depth
            # clouds. Rotate across fresh admitted acquisitions, retaining the
            # same per-frame ray budget and all terminal/hit safety rules.
            phase = self._depth_ray_phase % stride
            rays=rays[phase::stride]
            self._depth_ray_phase += 1
        misses=set()
        for end in rays:misses.update(self._raycast_indices(origin,end))
        misses.difference_update(hits)
        self._observe_voxels(misses)
        self._observe_voxels(hits)
        for index in misses:
            self._grid[index]=self._occupancy.apply_miss(index)
            self._record_sensor_miss(index)
            if self._grid[index]==FREE and self._source[index]!=SOURCE_GEOMETRY:
                self._source[index]=SOURCE_NONE
                self._labels[index]=0;self._instance_ids[index]=0
        for hit in hits.values():self.mark_occupied_world(*hit)
        if hits or misses:self._revision+=1
        return dict(rays=len(rays),hits=len(hits),free_voxels=len(misses),
                    rays_admitted=rays_admitted,
                    rays_sampled=(len(rays) if self.max_raycast_points
                                  else rays_admitted),
                    rejected=rejected,observed_points=list(hits.values()))

    def state_at(self, index):
        return self._occupancy.state_at(index)

    def voxel_records_world(self):
        """Return deterministic ``(center_xyz, state)`` records for scoring."""
        records = []
        for iz in range(self.nz):
            for iy in range(self.ny):
                for ix in range(self.nx):
                    idx = self._index(ix, iy, iz)
                    local = self._voxel_center_local(ix, iy, iz)
                    records.append((
                        self._local_to_world(*local),
                        self._grid[idx],
                    ))
        return records

    def state_at_world(self, x, y, z):
        voxel = self._local_to_voxel(*self._world_to_local(x, y, z))
        if voxel is None:
            return OCCUPIED
        return self._grid[self._index(*voxel)]

    def corridor_free_confidence(self, start, end, radius=0.08):
        """Estimate observed-free confidence for a conservative swept corridor.

        Unknown cells lower confidence and occupied/out-of-grid cells reject the
        corridor. Cross-section samples are axis-aligned in the base frame; the
        MoveIt validator remains the authoritative full-link collision check.
        """
        start = [float(v) for v in start]
        end = [float(v) for v in end]
        radius = max(0.0, float(radius))
        delta = [end[i] - start[i] for i in range(3)]
        length = math.sqrt(sum(v * v for v in delta))
        steps = max(1, int(math.ceil(length / (self.resolution * 0.5))))
        offsets = [(0.0, 0.0, 0.0)]
        if radius > 0.0:
            offsets.extend([
                (radius, 0.0, 0.0), (-radius, 0.0, 0.0),
                (0.0, radius, 0.0), (0.0, -radius, 0.0),
                (0.0, 0.0, radius), (0.0, 0.0, -radius),
            ])
        free = 0
        total = 0
        for step in range(steps + 1):
            t = float(step) / float(steps)
            center = [start[i] + delta[i] * t for i in range(3)]
            for offset in offsets:
                state = self.state_at_world(
                    center[0] + offset[0],
                    center[1] + offset[1],
                    center[2] + offset[2],
                )
                total += 1
                if state == OCCUPIED:
                    return 0.0
                if state == FREE:
                    free += 1
        return float(free) / float(total) if total else 0.0

    @staticmethod
    def _yaw_error(a, b):
        return abs((float(a) - float(b) + math.pi) % (2.0 * math.pi) - math.pi)

    def _matching_placed_box(self, center, size, yaw=None, tolerance=0.05):
        center = [float(v) for v in center]
        size = [float(v) for v in size]
        for index, record in enumerate(self._placed_boxes):
            position_error = math.sqrt(sum(
                (record["center"][axis] - center[axis]) ** 2
                for axis in range(3)))
            size_error = max(
                abs(record["size"][axis] - size[axis])
                for axis in range(3))
            yaw_matches = (
                yaw is None
                or self._yaw_error(record["yaw"], yaw) <= 1e-3
            )
            if (position_error <= tolerance and size_error <= tolerance
                    and yaw_matches):
                return index
        return None

    def mark_placed_box(self, center, size, yaw=0.0, tolerance=0.05,
                        measured_source=None, verify_anchor=None):
        """Mark an axis/yaw-aligned box as occupied (size: width, depth, height).

        Samples at the voxel resolution so thin boxes are not skipped. ``center``
        is in base/world coordinates; ``yaw`` rotates the footprint about Z.
        Uses SOURCE_GEOMETRY so the map can distinguish this from sensor data.
        ``measured_source`` records v2 commit provenance when supplied; it is
        absent (not defaulted) for v1 commits, so v1 ledger output is
        unchanged. ``verify_anchor`` ({request_id, delta_m, tolerance_m})
        records the accepted slot-vs-measurement agreement on the ledger so
        the commit stays auditable against the request that carried the
        anchor; absent for un-anchored commits.
        """
        center = [float(v) for v in center]
        size = [float(v) for v in size]
        if (len(center) != 3 or len(size) != 3
                or not all(math.isfinite(v) for v in center + size)
                or any(v <= 0.0 for v in size)
                or not math.isfinite(float(yaw))):
            raise ValueError("placed box needs finite xyz and positive size")
        if self._matching_placed_box(center, size, yaw, tolerance) is not None:
            return False
        if not any(self._iter_placed_box_voxels(center, size, float(yaw))):
            raise ValueError(
                "placed box rasterizes to zero voxels")
        record = {
            "center": center,
            "size": size,
            "yaw": float(yaw),
        }
        if measured_source is not None:
            record["measured_source"] = str(measured_source)
        if verify_anchor is not None:
            delta = float(verify_anchor["delta_m"])
            tol = float(verify_anchor["tolerance_m"])
            if not (math.isfinite(delta) and math.isfinite(tol)
                    and tol > 0.0 and delta <= tol):
                raise ValueError(
                    "verify anchor needs a finite delta within tolerance")
            record["verify_anchor"] = {
                "request_id": str(verify_anchor.get("request_id") or ""),
                "delta_m": delta,
                "tolerance_m": tol,
                # Declared measurement source ("sim_gt_pose" for the
                # Gazebo readback; the deploy path declares its real
                # observation source). Provenance only — never a
                # substitute for a real observation in deployment.
                "verify_source": str(verify_anchor.get("verify_source")
                                     or "undeclared"),
            }
        self._placed_boxes.append(record)
        self._rasterize_placed_box(
            record["center"], record["size"], record["yaw"])
        self._revision += 1
        return True

    def unmark_placed_box(self, center, size, yaw=None, tolerance=0.05):
        """Remove a committed geometry record and rebuild fail-closed state.

        ST04 F19: the removed box's exact voxel set is retracted — locks
        released, and every cell the sensor had its own word on is
        restored to the sensor's *latest* testimony (shadow ledger),
        never erased and never a stale hit. Cells only geometry touched
        return to unknown; removal never fabricates free space.
        """
        match = self._matching_placed_box(center, size, yaw, tolerance)
        if match is None:
            return False
        removed = self._placed_boxes[match]
        remaining = [
            record for index, record in enumerate(self._placed_boxes)
            if index != match]
        self._placed_boxes = remaining
        self._retract_box_voxels(
            self._iter_placed_box_voxels(
                removed["center"], removed["size"], removed["yaw"]))
        for record in remaining:
            self._rasterize_placed_box(
                record["center"], record["size"], record["yaw"])
        self._revision += 1
        return True

    def _retract_box_voxels(self, indices):
        """Retract one removed box's voxel set.

        Every covered cell is unlocked and cleared; the sensor's shadow
        testimony is written back as the live log-odds value (no lock, no
        force) so the cell equals the no-geometry control exactly.
        """
        occupancy = self._occupancy
        indices = tuple(indices)
        self._observe_voxels(indices)
        for idx in indices:
            occupancy.clear(idx)
            if self._sensor_seen[idx]:
                value = self._sensor_values[idx]
                state = occupancy.classify_value(value)
                occupancy.values[idx] = value
                self._grid[idx] = state
                if state == FREE:
                    # A FREE cell has no owner or label — exactly what
                    # mark_free_world's own normalization produces in the
                    # no-geometry control.
                    self._source[idx] = SOURCE_NONE
                    self._labels[idx] = 0
                    if self._instance_ids:
                        self._instance_ids[idx] = 0
                else:
                    self._source[idx] = SOURCE_SENSOR
                    self._labels[idx] = self._sensor_labels[idx]
                    if self._instance_ids:
                        self._instance_ids[idx] = \
                            self._sensor_instance_ids[idx]
                continue
            self._grid[idx] = UNKNOWN
            self._source[idx] = SOURCE_NONE
            self._labels[idx] = 0
            if self._instance_ids:
                self._instance_ids[idx] = 0

    def commit_ledger(self):
        """Return a detached, deterministic snapshot of committed boxes."""
        snapshot = []
        for record in self._placed_boxes:
            entry = {
                "center": list(record["center"]),
                "size": list(record["size"]),
                "yaw": float(record["yaw"]),
            }
            if "measured_source" in record:
                entry["measured_source"] = str(record["measured_source"])
            if "verify_anchor" in record:
                entry["verify_anchor"] = dict(record["verify_anchor"])
            snapshot.append(entry)
        return snapshot

    def _iter_placed_box_voxels(self, center, size, yaw=0.0):
        """Yield the flat indices of one committed box's exact voxel set.

        ST04 declared half-open rule: a column cell is occupied iff the
        oriented footprint rectangle intersects the cell's half-open square
        with positive area; vertical occupancy spans exactly the voxel layers
        the box's z-interval touches with positive overlap. This replaces
        the historical interior-sample lattice, which could miss large
        diagonal slivers; the occupied set is now exactly the exact
        cell-intersection oracle (never only an array-sum comparison).
        """
        w, d, h = size
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        half_l, half_w = 0.5 * self.inner_l, 0.5 * self.inner_w
        res = self.resolution
        # Grid-frame footprint corners (mapper base == container frame).
        corners = []
        for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            lx, ly = 0.5 * sx * w, 0.5 * sy * d
            corners.append((center[0] + cos_y * lx - sin_y * ly,
                            center[1] + sin_y * lx + cos_y * ly))
        extent_x = 0.5 * (abs(cos_y) * w + abs(sin_y) * d)
        extent_y = 0.5 * (abs(sin_y) * w + abs(cos_y) * d)
        ix_min = max(0, int(math.floor(
            (center[0] - extent_x + half_l) / res)))
        ix_max = min(self.nx - 1, int(math.floor(
            (center[0] + extent_x + half_l) / res)))
        iy_min = max(0, int(math.floor(
            (center[1] - extent_y + half_w) / res)))
        iy_max = min(self.ny - 1, int(math.floor(
            (center[1] + extent_y + half_w) / res)))
        z_local_min = center[2] - 0.5 * h - self.center[2]
        z_local_max = center[2] + 0.5 * h - self.center[2]
        # Half-open voxel layers [bottom, top): include a layer iff its
        # bottom is strictly below z_max and its top strictly above z_min.
        iz_min = max(0, int(math.floor(
            (z_local_min + 0.5 * self.inner_h) / res)))
        iz_max = min(self.nz - 1, int(math.ceil(
            (z_local_max + 0.5 * self.inner_h) / res)) - 1)
        for ix in range(ix_min, ix_max + 1):
            xmin = -half_l + ix * res
            for iy in range(iy_min, iy_max + 1):
                ymin = -half_w + iy * res
                if _rect_footprint_overlap(
                        corners, xmin, ymin, xmin + res, ymin + res) <= 1e-12:
                    continue
                for iz in range(iz_min, iz_max + 1):
                    yield self._index(ix, iy, iz)

    def _rasterize_placed_box(self, center, size, yaw=0.0):
        """Rasterize a committed box without changing history/revision."""
        indices = tuple(self._iter_placed_box_voxels(center, size, yaw))
        self._observe_voxels(indices)
        if self._voxel_delta is not None:
            self._voxel_delta.record_geometry(center, size, yaw, indices)
        for idx in indices:
            self._grid[idx] = self._occupancy.force_occupied(
                idx, lock_geometry=True)
            if self._source[idx] == SOURCE_NONE:
                self._source[idx] = SOURCE_GEOMETRY

    def fill_unoccupied_as_free(self):
        """Geometry GT: usable-hull empty cells are known-free.

        Mutates voxel state, so it advances the revision like every other
        grid mutation (B1: a revision-frozen snapshot contract cannot
        tolerate unmetered writes; the GT dump path does not read the
        revision afterwards)."""
        changed = False
        for i, cell in enumerate(self._grid):
            if self._active is not None and not self._active[i]:
                continue
            if cell == UNKNOWN:
                self._observe_voxels((i,))
                self._grid[i] = FREE
                changed = True
        if changed:
            self._revision += 1

    def occupancy_summary(self):
        """Counts for integration responses; does not enumerate frontiers."""
        if self._query_cache is not None:
            result = self._query_cache.as_dict()
            result.pop("frontier_count")
            result["map_revision"] = self._revision
            return result
        total, inactive, unknown, free, occupied, label_dist = occupancy_counts(self)
        voxel_vol = self.resolution ** 3
        return {
            "total_voxels": total,
            "inactive_count": inactive,
            "unknown_count": unknown,
            "free_count": free,
            "occupied_count": occupied,
            "unknown_ratio": float(unknown) / total if total else 0.0,
            "occupancy_ratio": float(occupied) / total if total else 0.0,
            "occupied_volume": float(occupied) * voxel_vol,
            "free_volume": float(free) * voxel_vol,
            "label_distribution": {str(k): v for k, v in label_dist.items()},
            "map_revision": self._revision,
            "committed_box_count": len(self._placed_boxes),
            "geometry_hash": self.geometry_hash,
        }

    def stats(self):
        return dict(self.occupancy_summary(), frontier_count=self.frontier_count())

    def frontier_count(self):
        """Count of unknown active cells with an observed 6-neighbour.

        Vectorized twin of ``_frontier_indices`` (same predicate, count
        only; oracle-tested equal). The pure-Python triple loop this
        replaces took ~6.5 s per call on the 1cm grid (4.2M voxels) and,
        by holding the GIL for that span, delayed this process's
        identity-heartbeat RECEPTION past the 2 s readiness lease --
        the PEER_LEASE_EXPIRED admission blockage (ADMISSION-DIAG
        2026-10-25, evidence placement_structural_g3_20261024/sim).
        """
        if getattr(self, "_query_cache", None) is not None:
            return self._query_cache.values["frontier_count"]
        import numpy as np
        from .cargo_grid_projection import _arrays
        grid, active = _arrays(self)
        observed = (grid == FREE) | (grid == OCCUPIED)
        unknown = (grid == UNKNOWN) & active
        padded = np.pad(observed, 1, constant_values=False)
        neighbour_observed = np.zeros(observed.shape, dtype=bool)
        neighbour_observed |= padded[:-2, 1:-1, 1:-1]
        neighbour_observed |= padded[2:, 1:-1, 1:-1]
        neighbour_observed |= padded[1:-1, :-2, 1:-1]
        neighbour_observed |= padded[1:-1, 2:, 1:-1]
        neighbour_observed |= padded[1:-1, 1:-1, :-2]
        neighbour_observed |= padded[1:-1, 1:-1, 2:]
        return int(np.count_nonzero(unknown & neighbour_observed))

    def _frontier_indices(self):
        if self._query_read_owner is not None:
            return self._query_read_owner.legacy_frontier_indices(self)
        return self._frontier_indices_full()

    def _frontier_indices_full(self):
        """Preserved scalar oracle; never used by published stats."""
        frontier = []
        for ix in range(self.nx):
            for iy in range(self.ny):
                for iz in range(self.nz):
                    idx = self._index(ix, iy, iz)
                    if self._active is not None and not self._active[idx]:
                        continue
                    if self._grid[idx] != UNKNOWN:
                        continue
                    if self._has_observed_neighbor(ix, iy, iz):
                        frontier.append((ix, iy, iz))
        return frontier

    def _has_observed_neighbor(self, ix, iy, iz):
        for dx, dy, dz in (
            (-1, 0, 0), (1, 0, 0), (0, -1, 0), (0, 1, 0), (0, 0, -1), (0, 0, 1),
        ):
            nx, ny, nz = ix + dx, iy + dy, iz + dz
            if nx < 0 or ny < 0 or nz < 0 or nx >= self.nx or ny >= self.ny or nz >= self.nz:
                continue
            if self._grid[self._index(nx, ny, nz)] in (FREE, OCCUPIED):
                return True
        return False

    def frontier_centroids_world(self, max_points=32):
        indices = self._frontier_indices()
        if not indices:
            return []
        step = max(1, len(indices) // max_points)
        sampled = indices[::step]
        points = []
        for ix, iy, iz in sampled:
            lx, ly, lz = self._voxel_center_local(ix, iy, iz)
            points.append(self._local_to_world(lx, ly, lz))
        return points

    def surface_map_2d(self):
        """Stable 2.5D map contract derived from the voxel grid.

        Returns a self-describing dict in container-local coordinates so the
        placement planner can reason on a flat top-surface model instead of the
        raw point cloud:

          - ``height``[ix][iy]: top occupied surface height above the inner floor
          - ``state``[ix][iy]:  ``occupied`` / ``free`` / ``unknown`` per column
          - ``clearance``[ix][iy]: free vertical space from the surface to the top
          - ``known_ratio``[ix][iy]: fraction of the column already observed
          - ``confidence``[ix][iy]: ``sensor`` / ``geometry`` / ``none``

        Conversion metadata (``center_base``, ``yaw``, ``inner_size``,
        ``resolution``) lets callers map a cell back into ``elfin_base_link``.
        ``geometry_hash`` / ``geometry_descriptor`` identify the hull this map
        was built for; a consumer whose own kernel hash differs must reject
        the map instead of reading ``inner_size``.
        """
        half_l = self.inner_l * 0.5
        half_w = self.inner_w * 0.5
        source = getattr(self, '_surface_dict_source', None)
        if source is not None:
            memoized = source.surface_as_dict()
            if memoized is not None:
                columns = memoized
            else:
                columns = (self._surface_cache.as_dict()
                           if self._surface_cache is not None
                           else surface_columns(self))
        else:
            columns = (self._surface_cache.as_dict() if self._surface_cache is not None
                       else surface_columns(self))

        return {
            "frame": "container_local",
            "map_revision": self._revision,
            "geometry_hash": self.geometry_hash,
            "geometry_descriptor": deepcopy(self.geometry_descriptor),
            "resolution": self.resolution,
            "nx": self.nx,
            "ny": self.ny,
            "inner_size": [self.inner_l, self.inner_w, self.inner_h],
            # Height values are relative to this mapper's usable inner floor.
            "floor_z": 0.0,
            "origin_local": [-half_l, -half_w],
            "center_base": list(self.center),
            "yaw": self.yaw,
            "height": columns["height"],
            "clearance": columns["clearance"],
            "state": columns["state"],
            "known_ratio": columns["known_ratio"],
            "confidence": columns["confidence"],
            "semantic_label": columns["semantic_label"],
        }

    def surface_contract_v2(self, scene_revision, request_id, *, projection=None):
        """F06 v2: strict container-origin surface record for this map.

        ``surface_z_container`` is measured from the container origin
        (``floor_z_container = center[2] - inner_h/2``); unknown columns are
        ``None`` with confidence ``unknown`` — never a numeric zero or the
        floor. The v1 ``surface_map_2d`` output is unchanged; this record is
        its scene-bound v2 counterpart (dual datum agrees within 1e-9 m on
        every non-null cell). Raises ContractError on any inconsistency.
        """
        from luggage_description.frame_contracts import validate_surface

        floor = self.center[2] - 0.5 * self.inner_h
        ceiling = self.center[2] + 0.5 * self.inner_h
        legacy = self.surface_map_2d() if projection is None else projection
        surface_z, state, confidence, relative = [], [], [], []
        for ix in range(self.nx):
            z_row, s_row, c_row, r_row = [], [], [], []
            for iy in range(self.ny):
                cell_state = legacy["state"][ix][iy]
                if cell_state == "unknown":
                    z_row.append(None)
                    s_row.append("unknown")
                    c_row.append("unknown")
                    r_row.append(None)
                    continue
                height = float(legacy["height"][ix][iy])
                source = legacy["confidence"][ix][iy]
                z_row.append(floor + height)
                s_row.append(cell_state)
                c_row.append(
                    "verified_geometry" if source == "geometry" else "sensor")
                r_row.append(height)
            surface_z.append(z_row)
            state.append(s_row)
            confidence.append(c_row)
            relative.append(r_row)
        record = {
            "contract_version": 2,
            "frame_id": "container_link",
            "length_unit": "m",
            # Scene-basis map snapshot: zero is an explicit valid epoch,
            # never a latest-TF request.
            "stamp_ns": 0,
            "time_basis": "scene",
            "scene_revision": str(scene_revision),
            "request_id": str(request_id),
            "kind": "surface",
            "map_revision": int(self._revision),
            "geometry_hash": self.geometry_hash,
            "height_datum": "container_origin",
            "origin_xy_container": [float(v) for v in legacy["origin_local"]],
            "resolution_m": float(self.resolution),
            "floor_z_container": float(floor),
            "ceiling_z_container": float(ceiling),
            "surface_z_container": surface_z,
            "state": state,
            "confidence": confidence,
            "height_above_floor": relative,
        }
        validate_surface(record, str(scene_revision), self.geometry_hash,
                         int(self._revision))
        return record

    def surface_cell_center_base(self, ix, iy, local_z=None):
        """Cell-center (ix,iy) in base frame; local_z defaults to inner floor."""
        half_l = self.inner_l * 0.5
        half_w = self.inner_w * 0.5
        lx = -half_l + (ix + 0.5) * self.resolution
        ly = -half_w + (iy + 0.5) * self.resolution
        lz = -self.inner_h * 0.5 if local_z is None else local_z
        return self._local_to_world(lx, ly, lz)
