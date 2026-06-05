#!/usr/bin/env python3
"""Cargo interior voxel occupancy mapper (unknown / free / occupied)."""

from __future__ import division

import math

try:
    import octomap
except ImportError:
    octomap = None

UNKNOWN = 0
FREE = 1
OCCUPIED = 2

MARKER_NS = "cargo_map"
MAX_VOXEL_MARKERS = 5000


class CargoVolumeMapper:
    """Axis-aligned voxel grid in container-local frame (yaw-aligned with base)."""

    def __init__(self, inner_size, center_base, yaw, resolution):
        self.resolution = float(resolution)
        self.inner_l, self.inner_w, self.inner_h = [float(v) for v in inner_size]
        self.center = [float(v) for v in center_base]
        self.yaw = float(yaw)
        self.nx = max(1, int(math.ceil(self.inner_l / self.resolution)))
        self.ny = max(1, int(math.ceil(self.inner_w / self.resolution)))
        self.nz = max(1, int(math.ceil(self.inner_h / self.resolution)))
        self._grid = None
        self.reset()

    def reset(self):
        self._grid = [UNKNOWN] * (self.nx * self.ny * self.nz)

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

    def mark_occupied_world(self, x, y, z):
        local = self._world_to_local(x, y, z)
        vox = self._local_to_voxel(*local)
        if vox is None:
            return
        self._grid[self._index(*vox)] = OCCUPIED

    def mark_free_world(self, x, y, z):
        local = self._world_to_local(x, y, z)
        vox = self._local_to_voxel(*local)
        if vox is None:
            return
        idx = self._index(*vox)
        if self._grid[idx] == UNKNOWN:
            self._grid[idx] = FREE

    def _raycast_free(self, origin, hit):
        ox, oy, oz = origin
        hx, hy, hz = hit
        dist = math.sqrt((hx - ox) ** 2 + (hy - oy) ** 2 + (hz - oz) ** 2)
        if dist < self.resolution:
            return
        steps = int(dist / (self.resolution * 0.5))
        if steps < 1:
            return
        for step in range(steps):
            t = float(step) / float(steps)
            px = ox + (hx - ox) * t
            py = oy + (hy - oy) * t
            pz = oz + (hz - oz) * t
            self.mark_free_world(px, py, pz)

    def integrate_points(self, points, origin=None):
        """points: list of (x,y,z) in base frame; origin optional camera position."""
        for x, y, z in points:
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            self.mark_occupied_world(x, y, z)

        if origin is not None:
            ox, oy, oz = origin
            for x, y, z in points:
                if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                    continue
                self._raycast_free((ox, oy, oz), (x, y, z))

    def mark_placed_box(self, center, size):
        """Mark axis-aligned box as occupied (size: width, depth, height)."""
        w, d, h = size
        steps = 3
        denom = float(max(steps - 1, 1))
        for ix in range(steps):
            for iy in range(steps):
                for iz in range(steps):
                    px = center[0] + (ix / denom - 0.5) * w
                    py = center[1] + (iy / denom - 0.5) * d
                    pz = center[2] + (iz / denom - 0.5) * h
                    self.mark_occupied_world(px, py, pz)

    def stats(self):
        total = len(self._grid)
        unknown = self._grid.count(UNKNOWN)
        free = self._grid.count(FREE)
        occupied = self._grid.count(OCCUPIED)
        voxel_vol = self.resolution ** 3
        return {
            "total_voxels": total,
            "unknown_count": unknown,
            "free_count": free,
            "occupied_count": occupied,
            "unknown_ratio": float(unknown) / total if total else 0.0,
            "occupancy_ratio": float(occupied) / total if total else 0.0,
            "free_volume": float(free) * voxel_vol,
            "frontier_count": len(self._frontier_indices()),
        }

    def _frontier_indices(self):
        frontier = []
        for ix in range(self.nx):
            for iy in range(self.ny):
                for iz in range(self.nz):
                    idx = self._index(ix, iy, iz)
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
                    if self._grid[self._index(ix, iy, iz)] not in state_filter:
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
                    state = self._grid[self._index(ix, iy, iz)]
                    if state == UNKNOWN:
                        continue
                    if state == FREE and not include_free:
                        continue
                    lx, ly, lz = self._voxel_center_local(ix, iy, iz)
                    bx, by, bz = self._local_to_world(lx, ly, lz)
                    tree.updateNode(
                        octomap.Point3d(bx, by, bz),
                        state == OCCUPIED,
                    )

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
    def _append_line_marker(markers, marker_id, frame_id, stamp, points, color, scale=0.02):
        from geometry_msgs.msg import Point
        from visualization_msgs.msg import Marker

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = MARKER_NS
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
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
        from geometry_msgs.msg import Point
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
        from geometry_msgs.msg import Point
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

    def _container_wireframe_markers(self, container_config, stamp):
        from visualization_msgs.msg import Marker, MarkerArray

        if container_config is None:
            return MarkerArray()

        from container_config_utils import (  # noqa: WPS433
            inner_dimensions,
            opening_in_container,
            outer_dimensions,
        )

        markers = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        delete_all.ns = MARKER_NS
        markers.markers.append(delete_all)

        outer_l, outer_w, outer_h = outer_dimensions(container_config)
        inner_l, inner_w, inner_h = inner_dimensions(container_config)
        container_frame = "container_link"

        self._append_line_marker(
            markers,
            1,
            container_frame,
            stamp,
            self._box_line_points(outer_l, outer_w, outer_h),
            self._make_color(0.6, 0.6, 0.6, 1.0),
            scale=0.03,
        )
        self._append_line_marker(
            markers,
            2,
            container_frame,
            stamp,
            self._box_line_points(inner_l, inner_w, inner_h),
            self._make_color(0.0, 0.9, 0.9, 1.0),
            scale=0.025,
        )

        opening_xyz, _opening_rpy = opening_in_container(container_config)
        ox, oy, oz = opening_xyz
        self._append_line_marker(
            markers,
            3,
            container_frame,
            stamp,
            [(ox, oy, oz), (ox, oy + 0.4, oz)],
            self._make_color(1.0, 1.0, 0.0, 1.0),
            scale=0.04,
        )
        return markers

    def to_marker_array(
        self,
        base_frame,
        stamp,
        container_config=None,
        show_free=False,
        show_unknown=True,
    ):
        from visualization_msgs.msg import MarkerArray

        markers = self._container_wireframe_markers(container_config, stamp)

        occupied_pts = self._iter_voxel_centers_base({OCCUPIED}, subsample=1)
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

        self._append_cube_list_marker(
            markers,
            4,
            base_frame,
            stamp,
            occupied_pts,
            self._make_color(1.0, 0.2, 0.1, 0.85),
            self.resolution * 0.95,
        )
        self._append_cube_list_marker(
            markers,
            5,
            base_frame,
            stamp,
            free_pts,
            self._make_color(0.1, 0.9, 0.2, 0.35),
            self.resolution * 0.85,
        )
        self._append_cube_list_marker(
            markers,
            6,
            base_frame,
            stamp,
            unknown_pts,
            self._make_color(0.5, 0.5, 0.5, 0.15),
            self.resolution * 0.75,
        )
        self._append_sphere_list_marker(
            markers,
            7,
            base_frame,
            stamp,
            edge_pts,
            self._make_color(0.1, 0.4, 1.0, 0.9),
            self.resolution * 0.8,
        )
        self._append_sphere_list_marker(
            markers,
            8,
            base_frame,
            stamp,
            observed_edge_pts,
            self._make_color(1.0, 0.55, 0.0, 0.95),
            self.resolution * 0.9,
        )
        self._append_sphere_list_marker(
            markers,
            9,
            base_frame,
            stamp,
            frontier_pts,
            self._make_color(1.0, 0.0, 1.0, 0.9),
            self.resolution,
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
