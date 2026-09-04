#!/usr/bin/env python3
"""Cargo interior voxel occupancy mapper (unknown / free / occupied)."""

from __future__ import division

import math

from luggage_perception.voxel_log_odds import LogOddsGrid

try:
    import octomap
except ImportError:
    octomap = None

UNKNOWN = 0
FREE = 1
OCCUPIED = 2

SOURCE_NONE = 0
SOURCE_SENSOR = 1
SOURCE_GEOMETRY = 2

MARKER_NS = "cargo_map"
MAX_VOXEL_MARKERS = 5000
# Slightly inflate voxel/wireframe markers for RViz readability.
MARKER_VIZ_SCALE = 1.25


class CargoVolumeMapper:
    """Axis-aligned voxel grid in container-local frame (yaw-aligned with base)."""

    def __init__(
        self, inner_size, center_base, yaw, resolution, occupancy_params=None,
        max_raycast_points=None, hull_local_inside=None,
    ):
        self.resolution = float(resolution)
        self.inner_l, self.inner_w, self.inner_h = [float(v) for v in inner_size]
        self.center = [float(v) for v in center_base]
        self.yaw = float(yaw)
        self.nx = max(1, int(math.ceil(self.inner_l / self.resolution)))
        self.ny = max(1, int(math.ceil(self.inner_w / self.resolution)))
        self.nz = max(1, int(math.ceil(self.inner_h / self.resolution)))
        self.occupancy_params = dict(occupancy_params or {})
        # Cap on the number of points used for free-space raycasting (the
        # per-point ray walk is the integrate bottleneck). None = no cap.
        self.max_raycast_points = (
            None if max_raycast_points is None else max(1, int(max_raycast_points))
        )
        self._hull_local_inside = hull_local_inside
        self._active = None
        self._grid = None
        self._occupancy = None
        self._revision = 0
        self._placed_boxes = []
        self._build_active_mask()
        self.reset()

    def reset(self, preserve_placed=False):
        saved = list(self._placed_boxes) if preserve_placed else []
        if not preserve_placed:
            self._placed_boxes = []
        self._revision += 1
        total = self.nx * self.ny * self.nz
        self._occupancy = LogOddsGrid(total, **self.occupancy_params)
        self._grid = self._occupancy.states()
        self._source = [SOURCE_NONE] * total
        self._labels = [0] * total
        self._instance_ids = [0] * total
        for placed in saved:
            self._rasterize_placed_box(
                placed["center"], placed["size"], placed["yaw"])

    def _build_active_mask(self):
        total = self.nx * self.ny * self.nz
        if self._hull_local_inside is None:
            self._active = None
            return
        self._active = [True] * total
        for iz in range(self.nz):
            for iy in range(self.ny):
                for ix in range(self.nx):
                    if not self._hull_local_inside(
                            *self._voxel_center_local(ix, iy, iz)):
                        self._active[self._index(ix, iy, iz)] = False

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

    def _edge_segments_local(self):
        half_l = self.inner_l * 0.5
        half_w = self.inner_w * 0.5
        half_h = self.inner_h * 0.5
        segments = []

        for y in (-half_w, half_w):
            for z in (-half_h, half_h):
                segments.append(((-half_l, y, z), (half_l, y, z)))
        for x in (-half_l, half_l):
            for z in (-half_h, half_h):
                segments.append(((x, -half_w, z), (x, half_w, z)))
        for x in (-half_l, half_l):
            for y in (-half_w, half_w):
                segments.append(((x, y, -half_h), (x, y, half_h)))

        return segments

    def _sample_segment_local(self, start, end):
        length = math.sqrt(
            (end[0] - start[0]) ** 2
            + (end[1] - start[1]) ** 2
            + (end[2] - start[2]) ** 2
        )
        steps = max(1, int(math.ceil(length / self.resolution)))
        samples = []
        for step in range(steps + 1):
            t = float(step) / float(steps)
            samples.append((
                start[0] + (end[0] - start[0]) * t,
                start[1] + (end[1] - start[1]) * t,
                start[2] + (end[2] - start[2]) * t,
            ))
        return samples

    def edge_points_world(self):
        """Geometry-prior Cargo edge samples in world/base coordinates."""
        points = []
        seen = set()
        for start, end in self._edge_segments_local():
            for local in self._sample_segment_local(start, end):
                key = tuple(int(round(v / self.resolution)) for v in local)
                if key in seen:
                    continue
                seen.add(key)
                points.append(list(self._local_to_world(*local)))
        return points

    def edge_boxes_world(self):
        return [
            {
                "center": point,
                "size": [self.resolution] * 3,
                "source": "container_geometry_edge",
            }
            for point in self.edge_points_world()
        ]

    def _is_near_edge_local(self, local_x, local_y, local_z):
        half_l = self.inner_l * 0.5
        half_w = self.inner_w * 0.5
        half_h = self.inner_h * 0.5
        tol = self.resolution * 1.5
        near_faces = 0
        for distance in (
            abs(abs(local_x) - half_l),
            abs(abs(local_y) - half_w),
            abs(abs(local_z) - half_h),
        ):
            if distance <= tol:
                near_faces += 1
        return near_faces >= 2

    def mark_occupied_world(self, x, y, z, source=SOURCE_SENSOR,
                            label=0, instance_id=0):
        local = self._world_to_local(x, y, z)
        vox = self._local_to_voxel(*local)
        if vox is None:
            return
        idx = self._index(*vox)
        if source == SOURCE_GEOMETRY:
            self._grid[idx] = self._occupancy.force_occupied(
                idx, lock_geometry=True
            )
        else:
            self._grid[idx] = self._occupancy.apply_hit(idx)
        if self._source[idx] == SOURCE_NONE:
            self._source[idx] = source
        # Sensor observations overwrite geometry-inferred labels.
        if source == SOURCE_SENSOR or self._labels[idx] == 0:
            self._labels[idx] = int(label)
            self._instance_ids[idx] = int(instance_id)

    def mark_free_world(self, x, y, z):
        local = self._world_to_local(x, y, z)
        vox = self._local_to_voxel(*local)
        if vox is None:
            return
        idx = self._index(*vox)
        self._grid[idx] = self._occupancy.apply_miss(idx)
        if self._grid[idx] == FREE and self._source[idx] != SOURCE_GEOMETRY:
            self._source[idx] = SOURCE_NONE
            self._labels[idx] = 0
            self._instance_ids[idx] = 0

    def _raycast_indices(self, origin, hit):
        ox, oy, oz = origin
        hx, hy, hz = hit
        dist = math.sqrt((hx - ox) ** 2 + (hy - oy) ** 2 + (hz - oz) ** 2)
        if dist < self.resolution:
            return set()
        steps = int(dist / (self.resolution * 0.5))
        if steps < 1:
            return set()
        indices = set()
        for step in range(steps):
            t = float(step) / float(steps)
            px = ox + (hx - ox) * t
            py = oy + (hy - oy) * t
            pz = oz + (hz - oz) * t
            local = self._world_to_local(px, py, pz)
            vox = self._local_to_voxel(*local)
            if vox is not None:
                indices.add(self._index(*vox))
        return indices

    def integrate_points(self, points, origin=None, labels=None, instance_ids=None):
        """Integrate observed points into the voxel grid.

        Args:
            points: list of (x, y, z) in base frame.
            origin: optional camera position for free-space raycasting.
            labels: optional parallel list of per-point semantic class labels.
            instance_ids: optional parallel list of per-point instance IDs.
        """
        hit_records = {}
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
            ray_points = points
            if self.max_raycast_points and len(points) > self.max_raycast_points:
                stride = max(1, len(points) // self.max_raycast_points)
                ray_points = points[::stride]
            for pt in ray_points:
                x, y, z = pt[0], pt[1], pt[2]
                if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                    continue
                miss_indices.update(
                    self._raycast_indices((ox, oy, oz), (x, y, z))
                )
        miss_indices.difference_update(hit_records)
        for idx in miss_indices:
            self._grid[idx] = self._occupancy.apply_miss(idx)
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

    def mark_placed_box(self, center, size, yaw=0.0):
        """Mark an axis/yaw-aligned box as occupied (size: width, depth, height).

        Samples at the voxel resolution so thin boxes are not skipped. ``center``
        is in base/world coordinates; ``yaw`` rotates the footprint about Z.
        Uses SOURCE_GEOMETRY so the map can distinguish this from sensor data.
        """
        record = {
            "center": [float(v) for v in center],
            "size": [float(v) for v in size],
            "yaw": float(yaw),
        }
        self._placed_boxes.append(record)
        self._rasterize_placed_box(
            record["center"], record["size"], record["yaw"])
        self._revision += 1

    def unmark_placed_box(self, center, size, tolerance=0.05):
        """Remove a committed geometry record and rebuild fail-closed state."""
        match = None
        for index, record in enumerate(self._placed_boxes):
            position_error = math.sqrt(sum(
                (record["center"][axis] - float(center[axis])) ** 2
                for axis in range(3)))
            size_error = max(
                abs(record["size"][axis] - float(size[axis]))
                for axis in range(3))
            if position_error <= tolerance and size_error <= tolerance:
                match = index
                break
        if match is None:
            return False
        remaining = [
            record for index, record in enumerate(self._placed_boxes)
            if index != match]
        self.reset(preserve_placed=False)
        self._placed_boxes = remaining
        for record in remaining:
            self._rasterize_placed_box(
                record["center"], record["size"], record["yaw"])
        self._revision += 1
        return True

    def _rasterize_placed_box(self, center, size, yaw=0.0):
        """Rasterize a committed box without changing history/revision."""
        w, d, h = size
        sx = max(2, int(math.ceil(w / self.resolution)) + 1)
        sy = max(2, int(math.ceil(d / self.resolution)) + 1)
        sz = max(2, int(math.ceil(h / self.resolution)) + 1)
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        for ix in range(sx):
            lx = (ix / float(sx - 1) - 0.5) * w
            for iy in range(sy):
                ly = (iy / float(sy - 1) - 0.5) * d
                rx = cos_y * lx - sin_y * ly
                ry = sin_y * lx + cos_y * ly
                for iz in range(sz):
                    lz = (iz / float(sz - 1) - 0.5) * h
                    self.mark_occupied_world(
                        center[0] + rx, center[1] + ry, center[2] + lz,
                        source=SOURCE_GEOMETRY,
                    )

    def fill_unoccupied_as_free(self):
        """Geometry GT: usable-hull empty cells are known-free."""
        for i, cell in enumerate(self._grid):
            if self._active is not None and not self._active[i]:
                continue
            if cell != OCCUPIED:
                self._grid[i] = FREE

    def stats(self):
        unknown = 0
        free = 0
        occupied = 0
        total = 0
        inactive = 0
        label_dist = {}
        for i, cell in enumerate(self._grid):
            if self._active is not None and not self._active[i]:
                inactive += 1
                continue
            total += 1
            if cell == UNKNOWN:
                unknown += 1
            elif cell == FREE:
                free += 1
            elif cell == OCCUPIED:
                occupied += 1
                if self._labels[i] > 0:
                    lbl = self._labels[i]
                    label_dist[lbl] = label_dist.get(lbl, 0) + 1
        voxel_vol = self.resolution ** 3
        return {
            "total_voxels": total,
            "inactive_count": inactive,
            "unknown_count": unknown,
            "free_count": free,
            "occupied_count": occupied,
            "unknown_ratio": float(unknown) / total if total else 0.0,
            "occupancy_ratio": float(occupied) / total if total else 0.0,
            "free_volume": float(free) * voxel_vol,
            "frontier_count": len(self._frontier_indices()),
            "label_distribution": {str(k): v for k, v in label_dist.items()},
            "map_revision": self._revision,
            "committed_box_count": len(self._placed_boxes),
        }

    def _frontier_indices(self):
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
        """
        half_l = self.inner_l * 0.5
        half_w = self.inner_w * 0.5
        height = [[0.0] * self.ny for _ in range(self.nx)]
        clearance = [[self.inner_h] * self.ny for _ in range(self.nx)]
        state = [["unknown"] * self.ny for _ in range(self.nx)]
        known_ratio = [[0.0] * self.ny for _ in range(self.nx)]
        confidence = [["none"] * self.ny for _ in range(self.nx)]
        semantic_label = [[0] * self.ny for _ in range(self.nx)]

        for ix in range(self.nx):
            for iy in range(self.ny):
                top_occ = None
                top_source = SOURCE_NONE
                observed = 0
                has_unknown = False
                label_counts = {}
                active_in_col = 0
                for iz in range(self.nz):
                    idx = self._index(ix, iy, iz)
                    if self._active is not None and not self._active[idx]:
                        continue
                    active_in_col += 1
                    cell = self._grid[idx]
                    if cell == UNKNOWN:
                        has_unknown = True
                        continue
                    observed += 1
                    if cell == OCCUPIED:
                        top_occ = iz
                        top_source = self._source[idx]
                        lbl = self._labels[idx]
                        if lbl > 0:
                            label_counts[lbl] = label_counts.get(lbl, 0) + 1
                if top_occ is not None:
                    surface = (top_occ + 1) * self.resolution
                    surface = min(surface, self.inner_h)
                    height[ix][iy] = surface
                    clearance[ix][iy] = max(0.0, self.inner_h - surface)
                    state[ix][iy] = "occupied"
                    if top_source == SOURCE_SENSOR:
                        confidence[ix][iy] = "sensor"
                    elif top_source == SOURCE_GEOMETRY:
                        confidence[ix][iy] = "geometry"
                    else:
                        confidence[ix][iy] = "sensor"
                    if label_counts:
                        semantic_label[ix][iy] = max(
                            label_counts, key=label_counts.get)
                elif not has_unknown:
                    state[ix][iy] = "free"
                    confidence[ix][iy] = "sensor"
                else:
                    state[ix][iy] = "unknown"
                    confidence[ix][iy] = "none"
                known_ratio[ix][iy] = (
                    float(observed) / float(active_in_col) if active_in_col else 0.0)

        return {
            "frame": "container_local",
            "map_revision": self._revision,
            "resolution": self.resolution,
            "nx": self.nx,
            "ny": self.ny,
            "inner_size": [self.inner_l, self.inner_w, self.inner_h],
            # Height values are relative to this mapper's usable inner floor.
            "floor_z": 0.0,
            "origin_local": [-half_l, -half_w],
            "center_base": list(self.center),
            "yaw": self.yaw,
            "height": height,
            "clearance": clearance,
            "state": state,
            "known_ratio": known_ratio,
            "confidence": confidence,
            "semantic_label": semantic_label,
        }

    def surface_cell_center_base(self, ix, iy, local_z=None):
        """Cell-center (ix,iy) in base frame; local_z defaults to inner floor."""
        half_l = self.inner_l * 0.5
        half_w = self.inner_w * 0.5
        lx = -half_l + (ix + 0.5) * self.resolution
        ly = -half_w + (iy + 0.5) * self.resolution
        lz = -self.inner_h * 0.5 if local_z is None else local_z
        return self._local_to_world(lx, ly, lz)

    def occupied_clusters_world(self):
        """Return coarse occupied voxel centers for inspector packing heuristics."""
        centers = []
        step = max(1, int(0.25 / self.resolution))
        for ix in range(0, self.nx, step):
            for iy in range(0, self.ny, step):
                for iz in range(0, self.nz, step):
                    if self._grid[self._index(ix, iy, iz)] != OCCUPIED:
                        continue
                    lx, ly, lz = self._voxel_center_local(ix, iy, iz)
                    centers.append({
                        "center": list(self._local_to_world(lx, ly, lz)),
                        "size": [self.resolution * 2] * 3,
                        "source": "voxel_map",
                    })
        return centers

    def observed_edge_points_world(self, max_points=64):
        """Depth-observed occupied voxels that lie near geometry-prior edges."""
        points = []
        for ix in range(self.nx):
            for iy in range(self.ny):
                for iz in range(self.nz):
                    if self._grid[self._index(ix, iy, iz)] != OCCUPIED:
                        continue
                    lx, ly, lz = self._voxel_center_local(ix, iy, iz)
                    if not self._is_near_edge_local(lx, ly, lz):
                        continue
                    points.append(list(self._local_to_world(lx, ly, lz)))

        if len(points) <= max_points:
            return points
        step = max(1, len(points) // max_points)
        return points[::step][:max_points]

    def _iter_voxel_centers_base(self, state_filter, subsample=1, max_points=MAX_VOXEL_MARKERS):
        points = []
        step = max(1, int(subsample))
        for ix in range(0, self.nx, step):
            for iy in range(0, self.ny, step):
                for iz in range(0, self.nz, step):
                    idx = self._index(ix, iy, iz)
                    if self._active is not None and not self._active[idx]:
                        continue
                    if self._grid[idx] not in state_filter:
                        continue
                    lx, ly, lz = self._voxel_center_local(ix, iy, iz)
                    points.append(self._local_to_world(lx, ly, lz))
                    if len(points) >= max_points:
                        return points
        return points

    def to_octomap_msg(self, frame_id, stamp, include_free=True):
        """Export observed voxels as octomap_msgs/Octomap for RViz OctoMap display."""
        if octomap is None:
            return None

        from octomap_msgs.msg import Octomap

        tree = octomap.OcTree(self.resolution)
        for ix in range(self.nx):
            for iy in range(self.ny):
                for iz in range(self.nz):
                    idx = self._index(ix, iy, iz)
                    if self._active is not None and not self._active[idx]:
                        continue
                    state = self._grid[idx]
                    if state == UNKNOWN:
                        continue
                    if state == FREE and not include_free:
                        continue
                    lx, ly, lz = self._voxel_center_local(ix, iy, iz)
                    bx, by, bz = self._local_to_world(lx, ly, lz)
                    # python-octomap (the .so in this container) has no
                    # Point3d; OcTree.updateNode takes a 3-vector + occupied.
                    tree.updateNode((bx, by, bz), state == OCCUPIED)

        msg = Octomap()
        msg.header.frame_id = frame_id
        msg.header.stamp = stamp
        msg.binary = True
        msg.id = b"OcTree"
        msg.resolution = self.resolution
        binary_data = tree.writeBinary()
        if isinstance(binary_data, str):
            msg.data = [ord(ch) for ch in binary_data]
        else:
            msg.data = list(bytearray(binary_data))
        return msg

    @staticmethod
    def _box_line_points(length, width, height, z_base=0.0):
        """12 edges of an axis-aligned box in container_link (bottom-center origin)."""
        hx = length * 0.5
        hy = width * 0.5
        z0 = z_base
        z1 = z_base + height
        corners = [
            (-hx, -hy, z0), (hx, -hy, z0), (hx, hy, z0), (-hx, hy, z0),
            (-hx, -hy, z1), (hx, -hy, z1), (hx, hy, z1), (-hx, hy, z1),
        ]
        edges = (
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        )
        points = []
        for start, end in edges:
            points.append(corners[start])
            points.append(corners[end])
        return points

    @staticmethod
    def _make_color(r, g, b, a=1.0):
        from std_msgs.msg import ColorRGBA

        color = ColorRGBA()
        color.r = float(r)
        color.g = float(g)
        color.b = float(b)
        color.a = float(a)
        return color

    @staticmethod
    def _append_line_marker(markers, marker_id, frame_id, stamp, points, color, scale=0.04):
        from geometry_msgs.msg import Point, Quaternion
        from visualization_msgs.msg import Marker

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = MARKER_NS
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation = Quaternion(w=1.0)
        marker.scale.x = scale
        marker.color = color
        for x, y, z in points:
            pt = Point()
            pt.x = x
            pt.y = y
            pt.z = z
            marker.points.append(pt)
        markers.markers.append(marker)

    @staticmethod
    def _append_cube_list_marker(markers, marker_id, frame_id, stamp, points, color, cube_scale):
        from geometry_msgs.msg import Point, Quaternion
        from visualization_msgs.msg import Marker

        if not points:
            return
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = MARKER_NS
        marker.id = marker_id
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation = Quaternion(w=1.0)
        marker.scale.x = cube_scale
        marker.scale.y = cube_scale
        marker.scale.z = cube_scale
        marker.color = color
        for x, y, z in points:
            pt = Point()
            pt.x = x
            pt.y = y
            pt.z = z
            marker.points.append(pt)
        markers.markers.append(marker)

    @staticmethod
    def _append_sphere_list_marker(markers, marker_id, frame_id, stamp, points, color, diameter):
        from geometry_msgs.msg import Point, Quaternion
        from visualization_msgs.msg import Marker

        if not points:
            return
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = MARKER_NS
        marker.id = marker_id
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation = Quaternion(w=1.0)
        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = diameter
        marker.color = color
        for x, y, z in points:
            pt = Point()
            pt.x = x
            pt.y = y
            pt.z = z
            marker.points.append(pt)
        markers.markers.append(marker)

    def _container_wireframe_markers(self, scene_config, stamp):
        from visualization_msgs.msg import Marker, MarkerArray

        if scene_config is None:
            return MarkerArray()

        from luggage_description.scene_tf_config_utils import (  # noqa: WPS433
            container_aperture_edges_in_container,
            container_inner_hull_edges_in_container,
            container_opening_in_container,
            container_outer_dimensions,
        )

        markers = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        delete_all.ns = MARKER_NS
        markers.markers.append(delete_all)

        outer_l, outer_w, outer_h = container_outer_dimensions(scene_config)
        container_frame = "container_link"

        self._append_line_marker(
            markers,
            1,
            container_frame,
            stamp,
            self._box_line_points(outer_l, outer_w, outer_h),
            self._make_color(0.6, 0.6, 0.6, 1.0),
            scale=0.055,
        )
        hull_pts = []
        for start, end in container_inner_hull_edges_in_container(scene_config):
            hull_pts.append(tuple(start))
            hull_pts.append(tuple(end))
        self._append_line_marker(
            markers,
            2,
            container_frame,
            stamp,
            hull_pts,
            self._make_color(0.0, 0.9, 0.9, 1.0),
            scale=0.045,
        )
        aper_pts = []
        for start, end in container_aperture_edges_in_container(scene_config):
            aper_pts.append(tuple(start))
            aper_pts.append(tuple(end))
        if aper_pts:
            self._append_line_marker(
                markers,
                3,
                container_frame,
                stamp,
                aper_pts,
                self._make_color(1.0, 0.82, 0.25, 1.0),
                scale=0.055,
            )

        opening_xyz, _opening_rpy = container_opening_in_container(scene_config)
        ox, oy, oz = opening_xyz
        self._append_line_marker(
            markers,
            4,
            container_frame,
            stamp,
            [(ox, oy - 0.2, oz), (ox, oy + 0.2, oz)],
            self._make_color(1.0, 1.0, 0.0, 1.0),
            scale=0.065,
        )
        return markers

    def _iter_semantic_voxel_centers(self, subsample=1, max_points=MAX_VOXEL_MARKERS):
        """Return occupied voxel centers grouped by semantic label."""
        groups = {}  # label -> list of (x, y, z)
        step = max(1, int(subsample))
        total = 0
        for ix in range(0, self.nx, step):
            for iy in range(0, self.ny, step):
                for iz in range(0, self.nz, step):
                    idx = self._index(ix, iy, iz)
                    if self._grid[idx] != OCCUPIED:
                        continue
                    lbl = self._labels[idx]
                    lx, ly, lz = self._voxel_center_local(ix, iy, iz)
                    groups.setdefault(lbl, []).append(
                        self._local_to_world(lx, ly, lz))
                    total += 1
                    if total >= max_points:
                        return groups
        return groups

    def to_marker_array(
        self,
        base_frame,
        stamp,
        scene_config=None,
        show_free=False,
        show_unknown=True,
    ):
        from visualization_msgs.msg import MarkerArray

        markers = self._container_wireframe_markers(scene_config, stamp)

        semantic_groups = self._iter_semantic_voxel_centers(subsample=1)
        # Color palette per semantic label:
        # 0=background (default red), 1=container_wall (grey),
        # 2=cargo (green), 3=robot_arm (cyan), 4=unknown (orange)
        label_colors = {
            0: self._make_color(1.0, 0.2, 0.1, 0.85),
            1: self._make_color(0.6, 0.6, 0.6, 0.7),
            2: self._make_color(0.1, 0.85, 0.2, 0.9),
            3: self._make_color(0.1, 0.8, 0.9, 0.7),
            4: self._make_color(1.0, 0.55, 0.0, 0.85),
        }
        viz = MARKER_VIZ_SCALE
        occupied_pts = []
        marker_id_offset = 100
        for lbl, pts in sorted(semantic_groups.items()):
            occupied_pts.extend(pts)
            if lbl > 0:
                color = label_colors.get(lbl, self._make_color(0.8, 0.8, 0.2, 0.8))
                self._append_cube_list_marker(
                    markers, marker_id_offset + lbl, base_frame, stamp,
                    pts, color, self.resolution * 0.95 * viz,
                )

        free_pts = (
            self._iter_voxel_centers_base({FREE}, subsample=2)
            if show_free
            else []
        )
        unknown_pts = (
            self._iter_voxel_centers_base({UNKNOWN}, subsample=2)
            if show_unknown
            else []
        )
        edge_pts = self.edge_points_world()
        observed_edge_pts = self.observed_edge_points_world()
        frontier_pts = self.frontier_centroids_world()

        # Unlabeled occupied voxels (label=0) keep the original red color.
        unlabeled_occ = semantic_groups.get(0, [])
        self._append_cube_list_marker(
            markers,
            4,
            base_frame,
            stamp,
            unlabeled_occ,
            self._make_color(1.0, 0.2, 0.1, 0.85),
            self.resolution * 0.95 * viz,
        )
        self._append_cube_list_marker(
            markers,
            5,
            base_frame,
            stamp,
            free_pts,
            self._make_color(0.1, 0.9, 0.2, 0.35),
            self.resolution * 0.85 * viz,
        )
        self._append_cube_list_marker(
            markers,
            6,
            base_frame,
            stamp,
            unknown_pts,
            self._make_color(0.5, 0.5, 0.5, 0.15),
            self.resolution * 0.75 * viz,
        )
        self._append_sphere_list_marker(
            markers,
            7,
            base_frame,
            stamp,
            edge_pts,
            self._make_color(0.1, 0.4, 1.0, 0.9),
            self.resolution * 0.8 * viz,
        )
        self._append_sphere_list_marker(
            markers,
            8,
            base_frame,
            stamp,
            observed_edge_pts,
            self._make_color(1.0, 0.55, 0.0, 0.95),
            self.resolution * 0.9 * viz,
        )
        self._append_sphere_list_marker(
            markers,
            9,
            base_frame,
            stamp,
            frontier_pts,
            self._make_color(1.0, 0.0, 1.0, 0.9),
            self.resolution * viz,
        )
        return markers

    def publish_params(self, rospy_module):
        stats = self.stats()
        rospy_module.set_param("/luggage/cargo_map/stats", stats)
        rospy_module.set_param(
            "/luggage/cargo_map/frontier_points",
            self.frontier_centroids_world(),
        )
        rospy_module.set_param(
            "/luggage/cargo_map/occupied_boxes",
            self.occupied_clusters_world(),
        )
        rospy_module.set_param(
            "/luggage/cargo_map/edge_points",
            self.edge_points_world(),
        )
        rospy_module.set_param(
            "/luggage/cargo_map/edge_boxes",
            self.edge_boxes_world(),
        )
        rospy_module.set_param(
            "/luggage/cargo_map/observed_edge_points",
            self.observed_edge_points_world(),
        )
        rospy_module.set_param(
            "/luggage/cargo_map/surface_2d",
            self.surface_map_2d(),
        )
