"""Surface grids: explicit validity, row-major shape, no unknown-to-floor coercion."""
from copy import deepcopy
import math

from luggage_msgs.msg import SurfaceContractV2
from luggage_description.frame_contracts import ContractError, validate_surface
from luggage_description.frame_diagnostics import at_boundary, validate_context
from luggage_description.frame_sequence_contracts import unsigned_integer
from .identity import (IDENTITY_FIELDS, identity_from_message, identity_to_message,
                       reject_extra_fields, require_message)

SCALARS = ('geometry_hash', 'map_revision', 'height_datum', 'resolution_m',
           'floor_z_container', 'ceiling_z_container')
FIELDS = (*IDENTITY_FIELDS, 'stamp_ns', 'kind', *SCALARS, 'origin_xy_container',
          'surface_z_container', 'state', 'confidence', 'height_above_floor',
          'epoch', 'content_digest', 'occupancy_confirmed', 'change_event')


def _validate(record, context):
    validate_surface(record, context.scene_revision, context.geometry_hash, context.map_revision)
    validate_context(record, context)
    unsigned_integer(record['map_revision'], 64, 'map_revision')
    reject_extra_fields(record, FIELDS)


def surface_to_message(record, context):
    def encode():
        _validate(record, context)
        message = SurfaceContractV2()
        message.identity = identity_to_message(record, context.scene_revision)
        for name in SCALARS:
            value = record[name]
            setattr(message, name, float(value) if name in ('resolution_m', 'floor_z_container', 'ceiling_z_container') else value)
        message.origin_xy_container = [float(v) for v in record['origin_xy_container']]
        message.rows = len(record['surface_z_container'])
        message.columns = len(record['surface_z_container'][0])
        tops = [v for row in record['surface_z_container'] for v in row]
        message.valid = [v is not None for v in tops]
        message.surface_z_container = [0.0 if v is None else float(v) for v in tops]
        message.state = [v for row in record['state'] for v in row]
        message.confidence = [v for row in record['confidence'] for v in row]
        message.has_height_above_floor = 'height_above_floor' in record
        if message.has_height_above_floor:
            message.height_above_floor = [0.0 if v is None else float(v)
                                          for row in record['height_above_floor'] for v in row]
        # [B1 snapshot identity] Optional; absent on pre-B1 records
        # (empty epoch/digest = "not declared" downstream).
        message.epoch = str(record.get('epoch', ''))
        message.content_digest = str(record.get('content_digest', ''))
        message.occupancy_confirmed = bool(record.get('occupancy_confirmed', False))
        message.change_event = str(record.get('change_event', ''))
        return message
    return at_boundary(record, context, encode)


def surface_from_message(message, context):
    record = {}
    def decode():
        require_message(message, SurfaceContractV2)
        record.update(identity_from_message(message.identity))
        record.update(kind='surface', **{name: getattr(message, name) for name in SCALARS})
        rows, columns = message.rows, message.columns
        count = rows * columns
        arrays = ('valid', 'surface_z_container', 'state', 'confidence')
        if not rows or not columns or any(len(getattr(message, n)) != count for n in arrays):
            raise ContractError('INVALID_GRID', 'wire_shape', 'nonempty matching row-major arrays', [rows, columns])
        relative = message.height_above_floor
        if ((message.has_height_above_floor and len(relative) != count)
                or (not message.has_height_above_floor and len(relative))):
            raise ContractError('INVALID_GRID', 'height_above_floor', 'presence and size agree', len(relative))
        for i, valid in enumerate(message.valid):
            values = [message.surface_z_container[i]]
            if message.has_height_above_floor:
                values.append(relative[i])
            if any(not math.isfinite(v) or (not valid and v != 0.0) for v in values):
                raise ContractError('INVALID_GRID', 'validity_payload', 'finite values; invalid cells canonical zero', values)
            if valid == (message.state[i] == 'unknown'):
                raise ContractError('UNKNOWN_SURFACE_HAS_VALUE', 'valid', 'valid iff observed', valid)
        def grid(values):
            return [list(values[i:i+columns]) for i in range(0, count, columns)]
        # rosidl fixed float arrays expose numpy.float64; the plain contract
        # intentionally accepts only built-in numbers. Convert at this typed boundary.
        record['origin_xy_container'] = [float(v) for v in message.origin_xy_container]
        record['surface_z_container'] = grid([v if ok else None for v, ok in zip(message.surface_z_container, message.valid)])
        record['state'] = grid(message.state)
        record['confidence'] = grid(message.confidence)
        if message.has_height_above_floor:
            record['height_above_floor'] = grid([v if ok else None for v, ok in zip(relative, message.valid)])
        # [B1 snapshot identity] Present only when the publisher declared
        # one (empty epoch = pre-B1 record): the 64-case serialization
        # matrix round-trips pre-B1 records byte-identically.
        if message.epoch or message.content_digest:
            record['epoch'] = str(message.epoch or '')
            record['content_digest'] = str(message.content_digest or '')
            record['occupancy_confirmed'] = bool(message.occupancy_confirmed)
            if message.change_event:
                record['change_event'] = str(message.change_event)
        _validate(record, context)
        return deepcopy(record)
    return at_boundary(record, context, decode)
