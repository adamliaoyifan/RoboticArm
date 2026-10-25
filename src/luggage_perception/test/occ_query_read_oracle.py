"""Independent whole-primary read oracle for recorded pressure responses.

No query index or candidate query implementation is imported. Construct a new
oracle for each replayed revision, and never mutate its mapper while verifying.
"""

import base64
import itertools
import json
import math
import numpy as np


class QueryReadOracle:

    def __init__(self, mapper, store, geometry, contains_point):
        self.mapper, self.store = (mapper, store)
        self.geometry, self.contains_point = (geometry, contains_point)
        self.nx, self.ny, self.nz = (mapper.nx, mapper.ny, mapper.nz)
        self.resolution = mapper.resolution
        self.origin = np.asarray([-mapper.inner_l / 2, -mapper.inner_w / 2, mapper._hull.floor_z])
        self._digest_cache = {}
        self._volumes_by_state = {state: None for state in (0, 2)}
        self._flat_indices_by_state = {
            state: np.flatnonzero(
                np.frombuffer(mapper._active, "u1").astype(bool)
                & (np.frombuffer(mapper._grid, "u1") == state)
            )
            for state in (0, 2)
        }

    def _cell(self, actual, index):
        assert list(actual.index) == [
            index % self.nx,
            index // self.nx % self.ny,
            index // (self.nx * self.ny),
        ], "CELL_COORDINATES"
        assert actual.flat_index == index, "CELL_FLAT_INDEX"
        assert (actual.state, actual.source, actual.semantic_label, actual.sensor_instance_id) == (
            self.mapper._grid[index],
            self.mapper._source[index],
            self.mapper._labels[index],
            self.mapper._instance_ids[index],
        ), "CELL_FACTS"
        assert list(actual.geometry_membership) == sorted(
            (key for (key, members) in self.geometry.items() if index in members)
        ), "CELL_MEMBERSHIP"

    def _frontier(self):
        grid = np.frombuffer(self.mapper._grid, "u1").reshape((self.nz, self.ny, self.nx))
        observed = (grid == 1) | (grid == 2)
        neighbors = np.zeros(grid.shape, dtype=bool)
        for axis in range(3):
            a, b = ([slice(None)] * 3, [slice(None)] * 3)
            a[axis], b[axis] = (slice(1, None), slice(None, -1))
            neighbors[tuple(a)] |= observed[tuple(b)]
            neighbors[tuple(b)] |= observed[tuple(a)]
        return np.flatnonzero(
            (grid == 0)
            & np.frombuffer(self.mapper._active, "u1").reshape(grid.shape).astype(bool)
            & neighbors
        )

    def _nearest(self, point, state, radius):
        selected = self._flat_indices_by_state[state]
        if not len(selected):
            return ("NO_MATCH", None, None)
        choices = []
        for p, o, n in zip(point, self.origin, (self.nx, self.ny, self.nz)):
            base = math.floor((p - o) / self.resolution)
            choices.append(
                [
                    i
                    for i in (base - 1, base, base + 1)
                    if 0 <= i < n and o + i * self.resolution <= p <= o + (i + 1) * self.resolution
                ]
            )
        zero = sorted(
            (x + self.nx * (y + self.ny * z) for (x, y, z) in itertools.product(*choices))
        )
        for index in zero:
            if self.mapper._active[index] and self.mapper._grid[index] == state:
                return ("MATCH", 0.0, index)
        if self._volumes_by_state[state] is None:
            xyz = np.column_stack(
                (selected % self.nx, selected // self.nx % self.ny, selected // (self.nx * self.ny))
            )
            self._volumes_by_state[state] = (
                self.origin + xyz * self.resolution,
                self.origin + (xyz + 1) * self.resolution,
            )
        low, high = self._volumes_by_state[state]
        delta = np.maximum(np.maximum(low - point, point - high), 0.0)
        squared = np.einsum("ij,ij->i", delta, delta)
        at = int(np.argmin(squared))
        distance = math.sqrt(float(squared[at]))
        if distance > radius:
            return ("RADIUS_CLEAR", None, None)
        return ("MATCH", distance, int(selected[at]))

    def verify(self, kind, request, response):
        identity = response.meta.identity
        assert (identity.epoch, identity.revision, identity.content_digest) == tuple(
            self.store.identity
        ), "RESPONSE_IDENTITY"
        from occ_query_identity_oracle import primary_voxel_digest

        schema = identity.index_schema_version
        if schema not in self._digest_cache:
            self._digest_cache[schema] = primary_voxel_digest(
                self.mapper, self.store, self.geometry, schema
            )
        assert identity.voxel_digest == self._digest_cache[schema], "VOXEL_DIGEST"
        grid = response.meta.grid
        assert (
            list(grid.shape) == [self.nx, self.ny, self.nz] and grid.resolution == self.resolution
        ), "GRID_SHAPE"
        assert [grid.origin.x, grid.origin.y, grid.origin.z] == self.origin.tolist(), "GRID_ORIGIN"
        assert (
            grid.frame_id == "container_link" and grid.geometry_hash == self.mapper.geometry_hash
        ), "GRID_BINDING"
        assert response.meta.request_id == request.query.request_id, "REQUEST_JOIN"
        method = {
            "summary": self._verify_summary,
            "roi": self._verify_roi,
            "distance": self._verify_distance,
            "surface": self._verify_surface,
        }.get(kind)
        assert method is not None, "UNSUPPORTED_READ_KIND"
        method(request, response)

    def _verify_summary(self, request, response):
        stats = self.mapper.stats()
        expected = dict(
            total_voxels=stats["total_voxels"],
            inactive_count=stats["inactive_count"],
            unknown_count=stats["unknown_count"],
            free_count=stats["free_count"],
            occupied_count=stats["occupied_count"],
            frontier_count=len(self._frontier()),
            placed_count=len(self.mapper._placed_boxes),
            unknown_ratio=stats["unknown_ratio"],
            occupancy_ratio=stats["occupancy_ratio"],
            occupied_volume=stats["occupied_volume"],
            free_volume=stats["free_volume"],
        )
        for key, value in expected.items():
            assert getattr(response, key) == value, key
        labels = sorted(((int(k), v) for (k, v) in stats["label_distribution"].items()))
        assert list(response.semantic_labels) == [k for (k, v) in labels], "LABEL_IDS"
        assert list(response.semantic_counts) == [v for (k, v) in labels], "LABEL_COUNTS"
        snapshot = self.store.v1_surface_dict()
        for key in (
            "occupancy_confirmed",
            "acquisition_stamp_ns",
            "map_source",
            "cargo_observed",
            "change_event",
        ):
            assert getattr(response, key) == snapshot[key], key
        assert response.meta.complete, "SUMMARY_COMPLETE"

    def _verify_roi(self, request, response):
        assert request.mode == 0 and request.instance.kind == 0, "UNSUPPORTED_ROI_MODE"
        selected = np.flatnonzero(np.frombuffer(self.mapper._active, "u1"))
        if request.states:
            selected = selected[
                np.isin(np.frombuffer(self.mapper._grid, "u1")[selected], request.states)
            ]
        if request.query.has_region:
            low = np.asarray(
                [request.query.region_min.x, request.query.region_min.y, request.query.region_min.z]
            )
            high = np.asarray(
                [request.query.region_max.x, request.query.region_max.y, request.query.region_max.z]
            )
            xyz = np.column_stack(
                (selected % self.nx, selected // self.nx % self.ny, selected // (self.nx * self.ny))
            )
            selected = selected[
                (
                    (self.origin + (xyz + 1) * self.resolution > low)
                    & (self.origin + xyz * self.resolution < high)
                ).all(axis=1)
            ]
        limit = request.query.limit or 1024
        expected = selected[:limit]
        assert [c.flat_index for c in response.cells] == expected.tolist(), "ROI_SET_ORDER"
        for actual, index in zip(response.cells, expected):
            self._cell(actual, int(index))
        assert response.count == response.meta.returned_count == len(expected), "ROI_COUNT"
        assert response.meta.complete == (len(selected) <= limit), "ROI_COMPLETE"
        assert bool(response.meta.next_cursor) == (len(selected) > limit), "ROI_CURSOR_PRESENCE"
        if response.meta.next_cursor:
            _, _, last = json.loads(base64.urlsafe_b64decode(response.meta.next_cursor)[32:])
            assert last == int(expected[-1]), "ROI_CURSOR_POSITION"

    def _verify_distance(self, request, response):
        assert len(response.results) == len(request.points), "DISTANCE_RESULT_COUNT"
        exhausted = any((item.status == "QUERY_BUDGET_EXHAUSTED" for item in response.results))
        assert response.meta.complete == (not exhausted), "DISTANCE_COMPLETE"
        assert response.meta.status == (
            "QUERY_BUDGET_EXHAUSTED" if exhausted else "OK"
        ), "DISTANCE_BATCH_STATUS"
        for point, item in zip(request.points, response.results):
            p = np.asarray([point.x, point.y, point.z])
            status = (
                "OUTSIDE_MAP"
                if (p < self.origin).any()
                or (
                    p >= self.origin + np.asarray([self.nx, self.ny, self.nz]) * self.resolution
                ).any()
                else "OUTSIDE_HULL" if not self.contains_point(self.mapper._hull, p) else "OK"
            )
            assert [item.point.x, item.point.y, item.point.z] == p.tolist(), "DISTANCE_POINT"
            assert item.status == (
                "QUERY_BUDGET_EXHAUSTED"
                if status == "OK"
                and any(
                    (
                        m.status == "QUERY_BUDGET_EXHAUSTED"
                        for m in (item.occupied, item.unknown)
                        if request.unknown_policy == 0 or m is item.occupied
                    )
                )
                else status
            ), "DISTANCE_POINT_STATUS"
            for state, actual in ((2, item.occupied), (0, item.unknown)):
                if state == 0 and request.unknown_policy == 1:
                    assert not item.has_unknown, "UNKNOWN_POLICY"
                    continue
                if state == 0:
                    assert item.has_unknown, "UNKNOWN_PRESENT"
                if status != "OK":
                    assert actual.status == status and (not actual.exact), "OUTSIDE_CERTIFICATION"
                    continue
                if actual.status == "QUERY_BUDGET_EXHAUSTED":
                    assert (
                        not actual.exact and (not actual.has_distance) and (not actual.has_cell)
                    ), "BUDGET_CERTIFICATION"
                    assert (
                        math.isfinite(actual.lower_bound_m) and actual.lower_bound_m >= 0.0
                    ), "BUDGET_LOWER_BOUND"
                    if actual.lower_bound_m:
                        wanted, distance, index = self._nearest(p, state, request.max_radius_m)
                        bound = distance if distance is not None else request.max_radius_m
                        assert actual.lower_bound_m <= bound + 1e-12, "BUDGET_LOWER_BOUND_TRUTH"
                    continue
                wanted, distance, index = self._nearest(p, state, request.max_radius_m)
                assert actual.status == wanted, "NEAREST_STATUS"
                assert actual.exact == (wanted in ("MATCH", "NO_MATCH")), "NEAREST_EXACTNESS"
                assert actual.has_distance == (distance is not None), "NEAREST_DISTANCE_PRESENCE"
                assert actual.has_cell == (index is not None), "NEAREST_CELL_PRESENCE"
                if distance is not None:
                    assert math.isclose(
                        actual.distance_m, distance, abs_tol=1e-12, rel_tol=0.0
                    ), "NEAREST_DISTANCE"
                    assert math.isclose(
                        actual.lower_bound_m, distance, abs_tol=1e-12, rel_tol=0.0
                    ), "NEAREST_LOWER_BOUND"
                    self._cell(actual.cell, index)
                elif wanted == "RADIUS_CLEAR":
                    assert actual.lower_bound_m == request.max_radius_m, "RADIUS_LOWER_BOUND"

    def _verify_surface(self, request, response):
        assert request.mode == 0 and (not request.query.has_region), "UNSUPPORTED_SURFACE_WINDOW"
        base = self.mapper.surface_map_2d()
        contract = self.mapper.surface_contract_v2("offline", "offline", projection=base)
        assert response.is_full and response.meta.complete, "SURFACE_COMPLETE"
        assert list(response.shape_xy) == [self.nx, self.ny] and list(response.origin_index_xy) == [
            0,
            0,
        ], "SURFACE_WINDOW"
        assert response.meta.returned_count == self.nx * self.ny, "SURFACE_COUNT"
        for key in ("height", "clearance", "known_ratio", "semantic_label"):
            assert np.array_equal(
                np.asarray(getattr(response, key)), np.asarray(base[key]).T.ravel()
            ), key
        for key, mapping in [
            ("state", {"unknown": 0, "free": 1, "occupied": 2}),
            ("confidence", {"none": 0, "sensor": 1, "geometry": 2}),
        ]:
            assert list(getattr(response, key)) == [
                mapping[v] for v in np.asarray(base[key]).T.ravel()
            ], key
        for key, flag in [
            ("surface_z_container", "has_surface_z"),
            ("height_above_floor", "has_height_above_floor"),
        ]:
            values = np.asarray(contract[key], dtype=object).T.ravel()
            assert list(getattr(response, flag)) == [v is not None for v in values], flag
            assert list(getattr(response, key)) == [
                v if v is not None else 0.0 for v in values
            ], key
        assert list(response.contract_confidence) == [
            {"unknown": 0, "sensor": 1, "verified_geometry": 2}[v]
            for v in np.asarray(contract["confidence"]).T.ravel()
        ], "CONTRACT_CONFIDENCE"
        assert (response.floor_z_container, response.ceiling_z_container) == (
            contract["floor_z_container"],
            contract["ceiling_z_container"],
        ), "SURFACE_DATUM"

    def verify_legacy(self, kind, request, response, frame_identity):
        """Validate the original full-volume wire, including admission refusals.

        This checks the old representation before its measured client-side
        conversion. A correct refusal remains a failed workload completion.
        """
        identity = self.store.identity
        if kind == "summary":
            assert response.success, "LEGACY_STATS_SUCCESS"
            stats = self.mapper.stats()
            for name in (
                "unknown_ratio",
                "occupancy_ratio",
                "occupied_volume",
                "free_volume",
                "unknown_count",
                "free_count",
                "occupied_count",
                "total_voxels",
            ):
                assert getattr(response, name) == stats[name], "LEGACY_STATS_" + name
            assert response.frontier_count == len(self._frontier()), "LEGACY_FRONTIER"
            assert response.placed_count == len(self.mapper._placed_boxes), "LEGACY_PLACED"
            surface = self.store.v1_surface_dict()
            for name in ("map_source", "cargo_observed", "occupancy_confirmed", "change_event"):
                assert getattr(response, name) == surface[name], "LEGACY_" + name
            assert response.last_acquisition_stamp_ns == surface["acquisition_stamp_ns"], (
                "LEGACY_STAMP",
                response.last_acquisition_stamp_ns,
                surface["acquisition_stamp_ns"],
            )
            assert response.message == "committed=%d" % len(
                self.mapper._placed_boxes
            ), "LEGACY_MESSAGE"
        else:
            matches = (
                request.scene_revision == frame_identity["scene_revision"]
                and request.geometry_hash == self.mapper.geometry_hash
                and request.expected_map_revision == identity.revision
                and (not request.expected_epoch or request.expected_epoch == identity.epoch)
                and (
                    not request.expected_content_digest
                    or request.expected_content_digest == identity.content_digest
                )
            )
            surface = self.store.v1_surface_dict()
            refusal = None
            if not matches:
                refusal = "SNAPSHOT_IDENTITY_MISMATCH"
            elif (
                surface["map_source"] != "measured"
                or surface["acquisition_stamp_ns"] <= 0
                or not surface["occupancy_confirmed"]
                or surface["pending_changes"]
                or (self.mapper.commit_ledger() and not self.store._pending.require_confirmation)
            ):
                refusal = "NO_CONFIRMED_MEASURED_VOLUME"
            if refusal:
                assert not response.success and response.reason == refusal, "LEGACY_REFUSAL"
                assert not response.states and not response.active, "LEGACY_REFUSAL_DATA"
                return
            assert response.success and response.reason == "OK", "LEGACY_VOLUME_SUCCESS"
            assert response.scene_revision == frame_identity["scene_revision"], "LEGACY_SCENE"
            assert response.frame_id == "container_link", "LEGACY_FRAME"
            assert (
                response.occupancy_confirmed and not response.has_pending_changes
            ), "LEGACY_CONFIRMED"
            assert response.map_source == surface["map_source"], "LEGACY_MAP_SOURCE"
            assert response.acquisition_stamp_ns == surface["acquisition_stamp_ns"], "LEGACY_STAMP"
            assert response.resolution_m == self.resolution, "LEGACY_RESOLUTION"
            assert list(response.dimensions_xyz) == [self.nx, self.ny, self.nz], "LEGACY_SHAPE"
            # Preserve copy_volume's arithmetic datum rather than substituting
            # the numerically close hull floor.
            assert list(response.origin_container) == [
                -self.mapper.inner_l / 2,
                -self.mapper.inner_w / 2,
                self.mapper.center[2] - self.mapper.inner_h / 2,
            ], "LEGACY_ORIGIN"
            assert bytes(response.states) == bytes(self.mapper._grid), "LEGACY_STATES"
            assert list(response.active) == [bool(v) for v in self.mapper._active], "LEGACY_ACTIVE"
        assert (response.epoch, response.map_revision, response.content_digest) == tuple(
            identity
        ), "LEGACY_IDENTITY"
        assert response.geometry_hash == self.mapper.geometry_hash, "LEGACY_GEOMETRY"
