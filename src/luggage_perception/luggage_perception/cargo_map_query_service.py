"""Typed ROS read boundary for the Store-owned immutable query API.

Only container_link coordinates are accepted. A first request explicitly
chooses CURRENT or a full expected identity; subsequent pages may carry its
token and that identity. Resuming never renews the five-second lifetime.
release_after_query explicitly closes a handle even for a partial response.
No write, TF-latest, motion, lease or executor authority belongs here.
"""
import numpy as np

from luggage_perception.cargo_map_query import (
    CellKey, CellPage, InstanceKey, QueryError, QueryIdentity, QueryPage, INDEX_SCHEMA_VERSION,
)


STATE = {'unknown': 0, 'free': 1, 'occupied': 2}
CONFIDENCE = {'none': 0, 'sensor': 1, 'geometry': 2}
CONTRACT_CONFIDENCE = {'unknown': 0, 'sensor': 1, 'verified_geometry': 2}


def identity_tuple(message):
    return QueryIdentity(message.epoch, message.revision, message.content_digest,
                         message.voxel_digest, message.index_schema_version)


def set_identity(message, identity):
    for name, value in zip(QueryIdentity._fields, identity):
        setattr(message, name, value)


def set_point(message, point):
    message.x, message.y, message.z = (float(v) for v in point)


def region(query):
    if not query.has_region:
        return None
    return tuple(tuple(getattr(p, axis) for axis in ('x', 'y', 'z'))
                 for p in (query.region_min, query.region_max))


def page(query):
    return QueryPage(query.limit or 1024, query.cursor)


class MapQueryService:
    def __init__(self, node, store, callback_group):
        from luggage_msgs import msg, srv
        self._messages = msg
        self._store = store
        self._logger = node.get_logger()
        self._services = tuple(node.create_service(kind, name, callback,
                                                   callback_group=callback_group)
            for kind, name, callback in (
                (srv.GetCargoMapSummaryV2, '/cargo_map/get_summary_v2', self.summary),
                (srv.QueryCargoCellsV2, '/cargo_map/query_cells_v2', self.cells),
                (srv.QueryCargoDistanceV2, '/cargo_map/query_distance_v2', self.distance),
                (srv.GetCargoSurfacePatchV2, '/cargo_map/get_surface_patch_v2', self.surface)))

    def _open(self, query):
        if (not query.request_id or len(query.request_id) > 128
                or query.frame_id != 'container_link'):
            raise QueryError('INVALID_FILTER')
        expected = identity_tuple(query.expected_identity)
        if query.use_current:
            if query.snapshot_token or any(expected):
                raise QueryError('INVALID_FILTER')
            expected = None
        elif (not expected.epoch or not expected.content_digest or not expected.voxel_digest
              or expected.index_schema_version != INDEX_SCHEMA_VERSION):
            raise QueryError('SNAPSHOT_IDENTITY_MISMATCH')
        return self._store.acquire_query_snapshot(expected, token=query.snapshot_token)

    def _meta(self, message, query, snapshot, result):
        message.request_id = query.request_id
        message.status, message.reason = result.status, result.reason
        set_identity(message.identity, result.snapshot_identity)
        message.grid.shape = list(result.grid.shape)
        set_point(message.grid.origin, result.grid.origin)
        message.grid.resolution = result.grid.resolution
        message.grid.frame_id = result.grid.frame_id
        message.grid.geometry_hash = result.grid.geometry_hash
        message.snapshot_token = '' if query.release_after_query else snapshot._token
        for name in ('complete', 'next_cursor', 'returned_count', 'examined_cells',
                     'examined_blocks', 'elapsed_ns'):
            setattr(message, name, getattr(result, name))

    def _execute(self, request, response, operation):
        snapshot = None
        query = request.query
        response.meta.request_id = query.request_id
        try:
            snapshot = self._open(query)
            result = operation(request, response, snapshot)
            self._meta(response.meta, query, snapshot, result)
        except QueryError as error:
            response.meta.status = response.meta.reason = error.reason
            response.meta.complete = False
        except (TypeError, ValueError, OverflowError) as error:
            self._logger.error('invalid typed map query: %s' % error)
            response.meta.status = response.meta.reason = 'INVALID_FILTER'
            response.meta.complete = False
        except Exception as error:
            self._logger.error('QUERY_INDEX_UNAVAILABLE in typed query: %r' % error)
            response.meta.status = response.meta.reason = 'QUERY_INDEX_UNAVAILABLE'
            response.meta.complete = False
        finally:
            if snapshot is not None and (query.release_after_query or
                    (not query.snapshot_token and response.meta.status
                     not in ('OK', 'QUERY_BUDGET_EXHAUSTED'))):
                snapshot.close()
        return response

    def summary(self, request, response):
        return self._execute(request, response, self._summary)

    def _summary(self, request, response, snapshot):
        if request.query.has_region or request.query.cursor:
            raise QueryError('INVALID_FILTER')
        result = snapshot.summary()
        for name in ('total_voxels', 'inactive_count', 'unknown_count', 'free_count',
                     'occupied_count', 'frontier_count', 'unknown_ratio', 'occupancy_ratio',
                     'occupied_volume', 'free_volume'):
            setattr(response, name, result.values[name])
        response.placed_count = result.values['committed_box_count']
        for name in ('occupancy_confirmed', 'index_status', 'acquisition_stamp_ns',
                     'map_source', 'cargo_observed', 'change_event'):
            setattr(response, name, getattr(result, name))
        labels = sorted((int(k), v) for k, v in result.values['label_distribution'].items())
        response.semantic_labels = [k for k, _ in labels]
        response.semantic_counts = [v for _, v in labels]
        return result

    def _cell(self, record):
        result = self._messages.CargoMapCellV2()
        result.index = [record.key.ix, record.key.iy, record.key.iz]
        result.flat_index, result.state, result.source = record.flat_index, record.state, record.source
        result.semantic_label, result.sensor_instance_id = record.label, record.instance_id
        result.geometry_membership = list(record.geometry_membership)
        return result

    def cells(self, request, response):
        return self._execute(request, response, self._cells)

    def _cells(self, request, response, snapshot):
        query, instance = request.query, request.instance
        if instance.kind == 0:
            instance_key = None
        elif instance.kind == 1:
            instance_key = InstanceKey('sensor_instance', instance.sensor_instance_id,
                                       instance.producer_namespace)
        elif instance.kind == 2:
            instance_key = InstanceKey('committed_box', instance.box_id)
        else:
            raise QueryError('INVALID_FILTER')
        if request.mode != 0 and request.states:
            raise QueryError('INVALID_FILTER')
        if request.mode not in (0, 2) and instance_key is not None:
            raise QueryError('INVALID_FILTER')
        if request.mode == 0:
            result = snapshot.query_cells(region(query), tuple(request.states) or (0, 1, 2),
                                           instance_key, page(query))
        elif request.mode == 1:
            result = snapshot.query_frontiers(region(query), page(query))
        elif request.mode == 2 and instance_key is not None:
            result = snapshot.query_instance(instance_key, region(query), page(query))
        elif request.mode == 3:
            if query.cursor:
                raise QueryError('INVALID_CURSOR')
            result = snapshot.count_frontiers(region(query))
            response.count = result.count
            return result
        elif request.mode == 4:
            if query.has_region or query.cursor:
                raise QueryError('INVALID_FILTER')
            record = snapshot.get_cell(CellKey(snapshot.identity.epoch,
                                               *(int(v) for v in request.cell_index)))
            result = CellPage('OK', '', snapshot.identity, snapshot.grid,
                              returned_count=1, examined_cells=1, examined_blocks=1,
                              cells=(record,))
        else:
            raise QueryError('INVALID_FILTER')
        response.cells = [self._cell(record) for record in result.cells]
        response.count = result.returned_count
        return result

    def _match(self, match):
        result = self._messages.CargoMapDistanceMatchV2()
        result.status, result.exact = match.status, match.exact
        result.has_distance = match.distance_m is not None
        result.distance_m = 0. if match.distance_m is None else match.distance_m
        result.lower_bound_m = match.lower_bound_m
        result.has_cell = match.cell is not None
        if match.cell is not None:
            result.cell = self._cell(match.cell)
        return result

    def distance(self, request, response):
        return self._execute(request, response, self._distance)

    def _distance(self, request, response, snapshot):
        if request.query.has_region or request.query.cursor:
            raise QueryError('INVALID_FILTER')
        if request.unknown_policy not in (0, 1):
            raise QueryError('INVALID_FILTER')
        policy = ('separate', 'occupied_only')[request.unknown_policy]
        points = [(p.x, p.y, p.z) for p in request.points]
        result = snapshot.nearest_obstacles(points, request.max_radius_m, policy)
        records = []
        for point in result.results:
            record = self._messages.CargoMapDistancePointV2()
            set_point(record.point, point.point)
            record.status, record.occupied = point.status, self._match(point.occupied)
            record.has_unknown = point.unknown is not None
            if point.unknown is not None:
                record.unknown = self._match(point.unknown)
            records.append(record)
        response.results = records
        return result

    def _surface_column(self, column):
        result = self._messages.CargoMapSurfaceColumnV2()
        for name in ('ix', 'iy', 'height', 'clearance', 'known_ratio', 'semantic_label'):
            setattr(result, name, getattr(column, name))
        result.state = STATE[column.state]
        result.confidence = CONFIDENCE[column.confidence]
        result.contract_confidence = CONTRACT_CONFIDENCE[column.contract_confidence]
        result.has_surface_z = column.surface_z_container is not None
        result.surface_z_container = column.surface_z_container or 0.
        result.has_height_above_floor = column.height_above_floor is not None
        result.height_above_floor = column.height_above_floor or 0.
        return result

    def surface(self, request, response):
        return self._execute(request, response, self._surface)

    def _surface(self, request, response, snapshot):
        response.floor_z_container = snapshot.grid.hull.floor_z
        response.ceiling_z_container = snapshot.grid.hull.ceiling_z
        if request.mode == 1:
            result = snapshot.surface_changes(identity_tuple(request.since_identity),
                                               region(request.query), page(request.query))
            set_identity(response.since_identity, result.since_identity)
            response.patches = [self._surface_column(c) for c in result.patches]
            return result
        if request.mode != 0 or request.query.cursor:
            raise QueryError('INVALID_FILTER')
        result = snapshot.surface(region(request.query))
        response.is_full = True
        response.floor_z_container, response.ceiling_z_container = (
            result.floor_z_container, result.ceiling_z_container)
        response.origin_index_xy, response.shape_xy = list(result.origin_index_xy), list(result.shape_xy)
        for name in ('height', 'clearance', 'known_ratio', 'semantic_label'):
            setattr(response, name, np.asarray(result.columns[name]).T.ravel().tolist())
        for name, choices in (('state', STATE), ('confidence', CONFIDENCE),
                              ('contract_confidence', CONTRACT_CONFIDENCE)):
            setattr(response, name, [choices[v] for v in np.asarray(result.columns[name]).T.ravel()])
        for name, validity in (('surface_z_container', 'has_surface_z'),
                                ('height_above_floor', 'has_height_above_floor')):
            values = np.asarray(result.columns[name], dtype=object).T.ravel()
            setattr(response, validity, [v is not None for v in values])
            setattr(response, name, [v if v is not None else 0. for v in values])
        return result
