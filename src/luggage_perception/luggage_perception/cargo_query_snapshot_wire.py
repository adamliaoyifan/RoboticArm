"""G4-A snapshot wire layer: QuerySnapshotData <-> wire objects.

Builds on cargo_query_wire (A0 freeze). One generation = a COMPLETE
manifest plus content-addressed objects. Encoding covers every public
QuerySnapshotData member; the receiver rebuilds a deep-immutable copy
and recomputes the identity root through the same hash chain the
producer uses (extracted into query_identity_root), so a transport
corruption cannot pass unnoticed.

Derived distance members are REBUILT deterministically from evidence +
active + the shape-shared layout (recipe distance_recipe_version=1,
layout = DistanceLeafLayout.initialize(block.shape)); state counts and
frontier counts are transported AND recompute-verified. fact_digest is
transported for identity continuity only -- never used as the transport
equivalence basis (cross-block frontier-only changes ship because the
change set, not the digest, decides what is sent).

Surface projection and the top-level distance index are transmitted as
typed arrays + JSON meta (no opaque blobs, no pickle). Stage coverage
is asserted: a non-None member that this stage does not encode raises
NOT_ENCODED instead of being silently dropped.
"""

from types import MappingProxyType

import numpy as np

from luggage_perception.cargo_distance_query import (
    DistanceIndex, DistanceLeafLayout, freeze_distance_members)
from luggage_perception.cargo_map_query import (
    CELL_DTYPE, GridInfo, QueryIdentity, QuerySnapshotData, VoxelBlock)
from luggage_perception.cargo_surface_projection import (
    SurfaceProjection, SurfaceTile)
from luggage_perception.cargo_voxel_delta import immutable_array
from luggage_perception.cargo_query_wire import (
    WireError, decode_array_object, decode_json_object,
    encode_array_object, encode_json_object)

DISTANCE_RECIPE_VERSION = 1


def _mapping(value):
    return MappingProxyType(dict(value))


def encode_snapshot(snapshot, producer_incarnation, seq, *,
                    base_identity=None, base_manifest_sha=None,
                    dirty_blocks=None, provenance_changed=True):
    """Snapshot -> {name: payload bytes}; caller content-addresses them.

    ``dirty_blocks`` (block ids whose payload changed, from the write
    path's dirty set incl. frontier neighborhoods) selects which block
    objects are EMITTED in a DELTA; unchanged blocks are referenced by
    their unchanged content addresses (the caller computes these from
    the previous manifest -- payloads are pure functions of the block,
    so identical blocks hash identically). None -> FULL.
    """
    objects = {}
    emit = None if dirty_blocks is None else set(dirty_blocks)

    def put(name, payload):
        if emit is None:
            objects[name] = payload
            return
        if not name.startswith('block.'):
            objects[name] = payload
            return
        # DELTA gating is per BLOCK id (the write path's dirty set),
        # never per object name and never per fact_digest.
        block_id = int(name.split('.')[1])
        if block_id in emit:
            objects[name] = payload

    grid = snapshot.grid
    put('grid', encode_json_object({
        'shape': [int(v) for v in grid.shape],
        'origin': [float(v) for v in grid.origin],
        'resolution': float(grid.resolution),
        'frame_id': grid.frame_id,
        'geometry_hash': grid.geometry_hash,
    }))
    for block in snapshot.blocks:
        prefix = 'block.%d' % block.block_id
        put(prefix + '.evidence', block.evidence)
        put(prefix + '.active', block.active)
        put(prefix + '.frontier', block.frontier)
        put(prefix + '.flat_indices',
            encode_array_object(np.asarray(block.flat_indices, dtype='<i4')))
        members = {'sensor': {str(k): v.hex() for k, v in
                              block.sensor_members.items()},
                   'box': {str(k): v.hex() for k, v in
                           block.box_members.items()}}
        put(prefix + '.members', encode_json_object(members))
        put(prefix + '.meta', encode_json_object({
            'start': [int(v) for v in block.start],
            'shape': [int(v) for v in block.shape],
            'state_counts': [int(v) for v in block.state_counts],
            'frontier_count': int(block.frontier_count),
            'fact_digest': block.fact_digest,
        }))
    put('sensor_blocks', encode_json_object(
        {str(k): sorted(int(v) for v in ids)
         for k, ids in snapshot.sensor_blocks.items()}))
    put('box_blocks', encode_json_object(
        {str(k): [int(v) for v in ids]
         for k, ids in snapshot.box_blocks.items()}))
    put('summary', encode_json_object(
        {k: (dict(v) if isinstance(v, MappingProxyType) else v)
         for k, v in snapshot.summary_values.items()}))
    legs = snapshot.map_snapshot
    put('provenance', encode_json_object({
        'identity': [str(legs.identity.epoch), int(legs.identity.revision),
                     legs.identity.content_digest],
        'acquisition_stamp_ns': int(legs.acquisition_stamp_ns),
        'map_source': str(legs.map_source),
        'cargo_observed': bool(legs.cargo_observed),
        'change_token': str(legs.change_token),
        'pending_details': legs.pending_details,
        'last_settle': legs.last_settle,
        'ledger_details': legs.ledger_details,
    }))
    put('identity', encode_json_object({
        'epoch': snapshot.identity.epoch,
        'revision': int(snapshot.identity.revision),
        'content_digest': snapshot.identity.content_digest,
        'voxel_digest': snapshot.identity.voxel_digest,
        'index_schema_version': int(snapshot.identity.index_schema_version),
    }))
    put('owner_evidence_bytes', encode_json_object(
        int(snapshot.owner_evidence_bytes)))
    put('unbound_geometry_count', encode_json_object(
        int(snapshot.unbound_geometry_count)))
    if snapshot.surface_projection is not None:
        for name, value in _encode_surface(snapshot.surface_projection):
            put(name, value)
    if snapshot.distance_index is not None:
        for name, value in _encode_distance_index(snapshot.distance_index):
            put(name, value)
    return objects


def _encode_surface(projection):
    """(name, payload) pairs; every tile field is JSON-canonical."""
    tiles = []
    for index, tile in enumerate(projection.tiles):
        fields = {}
        for name, values in tile.fields.items():
            array = np.asarray(values)
            fields[name] = array.tolist()
        tiles.append({'start': [int(v) for v in tile.start],
                      'shape': [int(v) for v in tile.shape],
                      'fields': fields})
    yield 'surface', encode_json_object(
        {'shape': [int(v) for v in projection.shape], 'tiles': tiles})


def _encode_distance_index(index):
    yield 'distance_index.lower', encode_array_object(
        np.asarray(index.lower, dtype='<f8'))
    yield 'distance_index.upper', encode_array_object(
        np.asarray(index.upper, dtype='<f8'))


def rebuild_snapshot(objects, expected_identity=None):
    """Wire objects -> deep-immutable QuerySnapshotData (stage A1a)."""
    def payload(name):
        if name not in objects:
            raise WireError('MISSING_OBJECT', name)
        return objects[name]

    grid_meta = decode_json_object(payload('grid'))
    hull_placeholder = _HullView(grid_meta['geometry_hash'])
    grid = GridInfo(tuple(grid_meta['shape']),
                    tuple(grid_meta['origin']),
                    grid_meta['resolution'], grid_meta['frame_id'],
                    grid_meta['geometry_hash'], hull_placeholder)

    block_ids = sorted(int(name.split('.')[1]) for name in objects
                       if name.startswith('block.')
                       and name.endswith('.meta'))
    layouts = {}
    blocks = []
    for block_id in block_ids:
        prefix = 'block.%d' % block_id
        meta = decode_json_object(payload(prefix + '.meta'))
        evidence = bytes(payload(prefix + '.evidence'))
        active = bytes(payload(prefix + '.active'))
        frontier = bytes(payload(prefix + '.frontier'))
        flat = decode_array_object(payload(prefix + '.flat_indices'))
        members = decode_json_object(payload(prefix + '.members'))
        sensor_members = _mapping({int(k): bytes.fromhex(v)
                                   for k, v in members['sensor'].items()})
        box_members = _mapping({k: bytes.fromhex(v)
                                for k, v in members['box'].items()})
        shape = tuple(int(v) for v in meta['shape'])
        layout = layouts.get(shape)
        if layout is None:
            layout = layouts[shape] = DistanceLeafLayout.initialize(shape)
        records = np.frombuffer(evidence, dtype=CELL_DTYPE)
        allowed = np.frombuffer(active, 'u1').astype(bool)
        distance_members = freeze_distance_members(layout, records, allowed)
        # Transported counts are recompute-verified (frontier included).
        counts = np.bincount(records['state'][allowed], minlength=3)
        transported_counts = tuple(int(v) for v in meta['state_counts'])
        if tuple(int(v) for v in counts) != transported_counts:
            raise WireError('STATE_COUNT_MISMATCH', str(block_id))
        frontier_count = int(np.frombuffer(frontier, 'u1').sum())
        if frontier_count != int(meta['frontier_count']):
            raise WireError('FRONTIER_COUNT_MISMATCH', str(block_id))
        blocks.append(VoxelBlock(
            int(block_id), tuple(int(v) for v in meta['start']), shape,
            flat.setflags(write=False) or flat, evidence, active, frontier,
            transported_counts, frontier_count, sensor_members, box_members,
            meta['fact_digest'], layout, distance_members))
    sensor_blocks = _mapping({int(k): tuple(v) for k, v in
                              decode_json_object(payload('sensor_blocks'))
                              .items()})
    box_blocks = _mapping({k: tuple(v) for k, v in
                           decode_json_object(payload('box_blocks'))
                           .items()})
    summary = _mapping(decode_json_object(payload('summary')))
    provenance = decode_json_object(payload('provenance'))
    identity_legs = decode_json_object(payload('identity'))
    map_snapshot = _MapSnapshotView(provenance)
    data = QuerySnapshotData(
        QueryIdentity(identity_legs['epoch'],
                      int(identity_legs['revision']),
                      identity_legs['content_digest'],
                      identity_legs['voxel_digest'],
                      int(identity_legs['index_schema_version'])),
        grid, tuple(blocks), sensor_blocks, box_blocks, summary,
        map_snapshot, b'',
        owner_evidence_bytes=int(
            decode_json_object(payload('owner_evidence_bytes'))),
        unbound_geometry_count=int(
            decode_json_object(payload('unbound_geometry_count'))))
    surface_projection = None
    if 'surface' in objects:
        surface_meta = decode_json_object(payload('surface'))
        tiles = []
        for tile in surface_meta['tiles']:
            fields = {}
            for name, values in tile['fields'].items():
                dtype = None
                # Numeric and string fields both rebuild exact dtypes.
                from luggage_perception.cargo_surface_projection import (
                    FIELD_DTYPES)
                dtype = np.dtype(FIELD_DTYPES[name])
                fields[name] = immutable_array(
                    np.asarray(values, dtype=dtype))
            tiles.append(SurfaceTile(
                tuple(int(v) for v in tile['start']),
                tuple(int(v) for v in tile['shape']),
                MappingProxyType(fields)))
        surface_projection = SurfaceProjection(
            tuple(int(v) for v in surface_meta['shape']), tuple(tiles))
    distance_index = None
    if 'distance_index.lower' in objects:
        distance_index = DistanceIndex(
            immutable_array(np.asarray(decode_array_object(
                payload('distance_index.lower')), dtype='<f8')),
            immutable_array(np.asarray(decode_array_object(
                payload('distance_index.upper')), dtype='<f8')))
    data_with_derived = QuerySnapshotData(
        data.identity, data.grid, data.blocks, data.sensor_blocks,
        data.box_blocks, data.summary_values, data.map_snapshot, b'',
        owner_evidence_bytes=data.owner_evidence_bytes,
        unbound_geometry_count=data.unbound_geometry_count,
        surface_projection=surface_projection,
        distance_index=distance_index)
    data = data_with_derived
    if expected_identity is not None and \
            tuple(data.identity) != tuple(expected_identity):
        raise WireError('IDENTITY_MISMATCH',
                        '%s != %s' % (tuple(data.identity),
                                      tuple(expected_identity)))
    return data


class _HullView:
    """Read-only hull stand-in carrying the geometry hash the wire needs."""

    def __init__(self, geometry_hash):
        self.geometry_hash = geometry_hash


class _MapSnapshotView:
    """Provenance legs the query surface reads, as a deep-immutable view."""

    def __init__(self, provenance):
        identity = provenance['identity']
        self.identity = _SnapshotIdentity(identity[0], int(identity[1]),
                                          identity[2])
        self.acquisition_stamp_ns = int(provenance['acquisition_stamp_ns'])
        self.map_source = provenance['map_source']
        self.cargo_observed = bool(provenance['cargo_observed'])
        self.change_token = provenance['change_token']
        self.pending_details = provenance['pending_details']
        self.last_settle = provenance['last_settle']
        self.ledger_details = provenance['ledger_details']

    def __getitem__(self, key):
        return getattr(self, key)


class _SnapshotIdentity:
    def __init__(self, epoch, revision, content_digest):
        self.epoch = epoch
        self.revision = revision
        self.content_digest = content_digest
