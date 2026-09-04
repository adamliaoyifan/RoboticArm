#!/usr/bin/env python3
"""Core world-scene occupancy grid logic (ROS-independent).

Maintains a bounded voxel grid with timestamped state and stale-clearing.
Used by world_scene_mapper_node.py for the ROS integration layer.
"""

from __future__ import division

import math

from luggage_perception.voxel_log_odds import LogOddsGrid

UNKNOWN = 0
FREE = 1
OCCUPIED = 2


class WorldSceneMapper:
    """Bounded voxel grid for the robot workspace with stale-clearing."""

    def __init__(self, bounds, resolution, stale_seconds, occupancy_params=None):
        self.resolution = float(resolution)
        self.x_min, self.x_max = bounds[0]
        self.y_min, self.y_max = bounds[1]
        self.z_min, self.z_max = bounds[2]
        self.nx = max(1, int(math.ceil((self.x_max - self.x_min) / self.resolution)))
        self.ny = max(1, int(math.ceil((self.y_max - self.y_min) / self.resolution)))
        self.nz = max(1, int(math.ceil((self.z_max - self.z_min) / self.resolution)))
        self.stale_seconds = float(stale_seconds)
        self.occupancy_params = dict(occupancy_params or {})
        self._grid = None
        self._occupancy = None
        self._timestamps = None
        self.reset()

    def reset(self):
        total = self.nx * self.ny * self.nz
        self._occupancy = LogOddsGrid(total, **self.occupancy_params)
        self._grid = self._occupancy.states()
        self._timestamps = [0.0] * total
        self._labels = [0] * total

    def _index(self, ix, iy, iz):
        return ix + self.nx * (iy + self.ny * iz)

    def _world_to_voxel(self, x, y, z):
        ix = int((x - self.x_min) / self.resolution)
        iy = int((y - self.y_min) / self.resolution)
        iz = int((z - self.z_min) / self.resolution)
        if 0 <= ix < self.nx and 0 <= iy < self.ny and 0 <= iz < self.nz:
            return ix, iy, iz
        return None

    def _voxel_center(self, ix, iy, iz):
        x = self.x_min + (ix + 0.5) * self.resolution
        y = self.y_min + (iy + 0.5) * self.resolution
        z = self.z_min + (iz + 0.5) * self.resolution
        return x, y, z

    def integrate_points(self, points, origin, now, labels=None):
        """Integrate a list of (x,y,z) points with raycast free-space.

        Args:
            points: list of (x, y, z) in world/base frame.
            origin: camera position for raycasting.
            now: current timestamp (float).
            labels: optional parallel list of semantic class labels.
        """
        hit_indices = set()
        miss_indices = set()
        hit_labels = {}
        for i, pt in enumerate(points):
            x, y, z = pt[0], pt[1], pt[2]
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            vox = self._world_to_voxel(x, y, z)
            if vox is not None:
                idx = self._index(*vox)
                hit_indices.add(idx)
                if labels is not None:
                    hit_labels[idx] = int(labels[i])
            miss_indices.update(self._raycast_indices(origin, (x, y, z)))

        miss_indices.difference_update(hit_indices)
        for idx in miss_indices:
            self._grid[idx] = self._occupancy.apply_miss(idx)
            self._timestamps[idx] = now
            if self._grid[idx] == FREE:
                self._labels[idx] = 0
        for idx in hit_indices:
            self._grid[idx] = self._occupancy.apply_hit(idx)
            self._timestamps[idx] = now
            if idx in hit_labels:
                self._labels[idx] = hit_labels[idx]

    def _raycast_indices(self, origin, hit):
        ox, oy, oz = origin
        hx, hy, hz = hit
        dist = math.sqrt((hx - ox) ** 2 + (hy - oy) ** 2 + (hz - oz) ** 2)
        if dist < self.resolution:
            return set()
        steps = max(1, int(dist / (self.resolution * 0.8)))
        indices = set()
        for step in range(steps):
            t = float(step) / float(steps)
            px = ox + (hx - ox) * t
            py = oy + (hy - oy) * t
            pz = oz + (hz - oz) * t
            vox = self._world_to_voxel(px, py, pz)
            if vox is None:
                continue
            indices.add(self._index(*vox))
        return indices

    def state_at(self, index):
        return self._occupancy.state_at(index)

    def clear_stale(self, now):
        """Reset voxels that haven't been observed within stale_seconds."""
        if self.stale_seconds <= 0:
            return 0
        cutoff = now - self.stale_seconds
        cleared = 0
        for i in range(len(self._grid)):
            if self._grid[i] != UNKNOWN and self._timestamps[i] < cutoff:
                self._occupancy.clear(i)
                self._grid[i] = UNKNOWN
                self._timestamps[i] = 0.0
                self._labels[i] = 0
                cleared += 1
        return cleared

    def obstacle_clusters(self, min_cluster_size=1):
        """Return occupied voxel centers."""
        clusters = []
        for ix in range(self.nx):
            for iy in range(self.ny):
                for iz in range(self.nz):
                    if self._grid[self._index(ix, iy, iz)] != OCCUPIED:
                        continue
                    clusters.append(self._voxel_center(ix, iy, iz))
        return clusters

    def obstacle_clusters_with_labels(self):
        """Return occupied voxel centers with their semantic label: (x, y, z, label)."""
        clusters = []
        for ix in range(self.nx):
            for iy in range(self.ny):
                for iz in range(self.nz):
                    idx = self._index(ix, iy, iz)
                    if self._grid[idx] != OCCUPIED:
                        continue
                    x, y, z = self._voxel_center(ix, iy, iz)
                    clusters.append((x, y, z, self._labels[idx]))
        return clusters

    def stats(self):
        total = len(self._grid)
        unknown = self._grid.count(UNKNOWN)
        free = self._grid.count(FREE)
        occupied = self._grid.count(OCCUPIED)
        label_dist = {}
        for i in range(total):
            if self._grid[i] == OCCUPIED and self._labels[i] > 0:
                lbl = self._labels[i]
                label_dist[lbl] = label_dist.get(lbl, 0) + 1
        return {
            "total_voxels": total,
            "unknown_count": unknown,
            "free_count": free,
            "occupied_count": occupied,
            "unknown_ratio": float(unknown) / total if total else 0.0,
            "occupied_ratio": float(occupied) / total if total else 0.0,
            "label_distribution": {str(k): v for k, v in label_dist.items()},
        }
