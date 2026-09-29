"""ST04 cargo map / measured commit acceptance (F06/F19/F20) on real types.

Runs after building/sourcing the isolated install; no ROS nodes, no TF.
FRAME_MIGRATION_CASE_LOG selects an append-only JSONL evidence file.
"""

import copy
from contextlib import contextmanager
import itertools
import json
import math
import os
from pathlib import Path
import time
import traceback
import unittest

from rclpy.serialization import serialize_message, deserialize_message

from luggage_description.frame_contracts import ContractError
from luggage_description.frame_diagnostics import (
    BoundaryContext, BoundaryContractError,
)
from luggage_perception.cargo_volume_mapper import (
    SOURCE_SENSOR, CargoVolumeMapper, normalize_ledger_slot,
)
from luggage_perception.frame_message_adapters import (
    measured_surface_to_message, measured_surface_from_message,
)
from luggage_planning.occupancy_place_paths import (
    OccupancyMismatch, OccupancySnapshot,
)


SCENE = 'scene-calibration-17'
INNER = (1.00, 0.60, 0.80)
RES = 0.05
WDH = (0.80, 0.50, 0.32)


def wire(message):
    return deserialize_message(serialize_message(message), type(message))


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return repr(value)
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_safe(v) for v in value]
    return value


@contextmanager
def case(case_id, inputs):
    row = dict(case_id=case_id, input=copy.deepcopy(inputs), status='pass',
               observations={})
    started = time.monotonic()
    try:
        yield row['observations']
    except BaseException:
        row.update(status='fail', traceback=traceback.format_exc())
        raise
    finally:
        row['elapsed_seconds'] = time.monotonic() - started
        output = os.environ.get('FRAME_MIGRATION_CASE_LOG')
        if output:
            with Path(output).open('a', encoding='utf-8') as handle:
                handle.write(json.dumps(json_safe(row), allow_nan=False,
                                        sort_keys=True) + '\n')
                handle.flush()
                os.fsync(handle.fileno())
        print('CASE', case_id, row['status'], flush=True)


def make_mapper(floor):
    return CargoVolumeMapper(
        INNER, (0.0, 0.0, floor + 0.5 * INNER[2]), 0.0, resolution=RES)


def surface_context(record, interface='F06'):
    return BoundaryContext(
        interface, 'cargo_volume_mapper', 'surface_v2_subscribers',
        record['request_id'], SCENE, 'container_link', '', 'scene',
        geometry_hash=record['geometry_hash'],
        map_revision=record['map_revision'])


def commit_v2(mapper, floor, support, yaw, center_xy=(0.05, -0.03),
              measured_source='verify_placed_box'):
    """Commit one non-square box through the v2 container-frame path."""
    center = [center_xy[0], center_xy[1], floor + support + 0.5 * WDH[2]]
    normalized, yaw_c, size = normalize_ledger_slot(
        center, yaw, list(WDH), source_frame='container_link')
    assert normalized == center and yaw_c == yaw
    assert mapper.mark_placed_box(normalized, size, yaw=yaw_c,
                                  measured_source=measured_source)
    return center, yaw_c


def exact_occupied_columns(center, size, yaw, mapper_center):
    """Independent oracle: cells with positive-area footprint intersection.

    Implements the declared half-open rule with its own polygon math
    (clipping each half-open cell against the four footprint edges).
    """
    nx = int(math.ceil(INNER[0] / RES))
    ny = int(math.ceil(INNER[1] / RES))
    columns = set()
    for ix in range(nx):
        for iy in range(ny):
            if footprint_cell_intersection(
                    center, size, yaw, ix, iy, mapper_center) > 1e-12:
                columns.add((ix, iy))
    return columns


def exact_top_height(center, size, mapper_center):
    """Independent z-quantization oracle for the committed box top."""
    z_local_max = (center[2] + 0.5 * size[2] - mapper_center[2]
                   + 0.5 * INNER[2])
    iz = min(int(math.ceil(z_local_max / RES)) - 1,
             int(math.ceil(INNER[2] / RES)) - 1)
    return min((iz + 1) * RES, INNER[2])


def _clip_polygon(polygon, origin, normal):
    """Sutherland-Hodgman clip against one half-plane (>=0 inside)."""
    out = []
    for index, point in enumerate(polygon):
        current = ((point[0] - origin[0]) * normal[0]
                   + (point[1] - origin[1]) * normal[1])
        previous = polygon[index - 1]
        prev = ((previous[0] - origin[0]) * normal[0]
                + (previous[1] - origin[1]) * normal[1])
        inside_c, inside_p = current >= -1e-12, prev >= -1e-12
        if inside_c != inside_p:
            t = prev / (prev - current)
            out.append((previous[0] + t * (point[0] - previous[0]),
                        previous[1] + t * (point[1] - previous[1])))
        if inside_c:
            out.append(point)
    return out


def polygon_area(polygon):
    if len(polygon) < 3:
        return 0.0
    total = 0.0
    for index, point in enumerate(polygon):
        nxt = polygon[(index + 1) % len(polygon)]
        total += point[0] * nxt[1] - nxt[0] * point[1]
    return abs(total) / 2.0


def footprint_cell_intersection(center, size, yaw, ix, iy, mapper_center):
    """Area of the oriented footprint inside one half-open grid cell."""
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    corners = []
    for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        lx, ly = 0.5 * sx * size[0], 0.5 * sy * size[1]
        corners.append((center[0] + cos_y * lx - sin_y * ly - mapper_center[0],
                        center[1] + sin_y * lx + cos_y * ly - mapper_center[1]))
    xmin = -0.5 * INNER[0] + ix * RES
    ymin = -0.5 * INNER[1] + iy * RES
    cell = [(xmin, ymin), (xmin + RES, ymin),
            (xmin + RES, ymin + RES), (xmin, ymin + RES)]
    polygon = cell
    for index in range(4):
        a, b = corners[index], corners[(index + 1) % 4]
        edge = (b[0] - a[0], b[1] - a[1])
        normal = (-edge[1], edge[0])  # inward for CCW footprint
        polygon = _clip_polygon(polygon, a, normal)
        if not polygon:
            return 0.0
    return polygon_area(polygon)


class FrameMigrationMap(unittest.TestCase):
    def assert_close(self, actual, expected, tolerance=1e-9):
        self.assertLessEqual(abs(actual - expected), tolerance)

    def reject(self, name, expected, payload, operation):
        effects = []
        with case('negative/' + name, payload) as observed:
            try:
                result = operation()
                effects.append(result)  # The only downstream dispatch point.
            except ContractError as error:
                observed.update(expected_code=expected, actual_code=error.code,
                                diagnostic=getattr(error, 'diagnostic',
                                                   error.as_dict()),
                                downstream_effects=len(effects))
                self.assertEqual(error.code, expected)
                self.assertEqual(effects, [])
            else:
                self.fail('unexpected accept: ' + name)

    def mismatch(self, name, payload, operation):
        effects = []
        with case('adapter/' + name, payload) as observed:
            try:
                effects.append(operation())
            except OccupancyMismatch as error:
                observed.update(reason=str(error), downstream_effects=0)
                self.assertIn('PLACE_PATH_INFEASIBLE', str(error))
                self.assertEqual(effects, [])
            else:
                self.fail('unexpected accept: ' + name)

    def test_64_surface_matrix_cases(self):
        # Grasp offsets are inapplicable to the map boundary (named so in
        # the plan revision); the fifth factor is the commit position, so
        # the common matrix keeps 4*2*2*2*2 = 64 cases.
        matrix = itertools.product(
            (0., math.pi / 2, math.pi, math.atan2(4, 3)),
            ((0., 0., 0.), (1.5, -.4, .2)), (.53, .71), (0., .30),
            ((0.05, -0.03), (-0.18, 0.12)))
        count = 0
        nx = int(math.ceil(INNER[0] / RES))
        ny = int(math.ceil(INNER[1] / RES))
        for index, (yaw, translation, floor, support, xy) in enumerate(matrix):
            fixture = dict(yaw=yaw, translation=translation, floor=floor,
                           support=support, commit_xy=list(xy), wdh=list(WDH))
            with self.subTest(index=index), \
                    case('surface/%02d' % index, fixture) as observed:
                mapper = make_mapper(floor)
                center, yaw_c = commit_v2(mapper, floor, support, yaw,
                                          center_xy=xy)
                # v2 record + real serialization roundtrip.
                record = mapper.surface_contract_v2(
                    SCENE, 'matrix-%02d' % index)
                ctx = surface_context(record)
                encoded = measured_surface_to_message(record, ctx)
                decoded = measured_surface_from_message(wire(encoded), ctx)
                self.assertEqual(decoded['map_revision'],
                                 record['map_revision'])
                self.assertEqual(decoded['geometry_hash'],
                                 record['geometry_hash'])
                observed.update(stage='serialized',
                                surface_wire_hex=serialize_message(
                                    encoded).hex()[:512])
                # Dual datum: v2 container-origin vs v1 floor-relative.
                legacy = mapper.surface_map_2d()
                self.assertEqual(legacy['map_revision'],
                                 record['map_revision'])
                floor_v2 = record['floor_z_container']
                self.assert_close(floor_v2,
                                  floor + 0.0)  # container floor == scene floor
                for ix in range(nx):
                    for iy in range(ny):
                        state_v1 = legacy['state'][ix][iy]
                        state_v2 = record['state'][ix][iy]
                        self.assertEqual(state_v1, state_v2,
                                         (ix, iy, state_v1, state_v2))
                        if state_v2 == 'unknown':
                            self.assertIsNone(
                                record['surface_z_container'][ix][iy])
                            self.assertIsNone(
                                record['height_above_floor'][ix][iy])
                            self.assertEqual(
                                record['confidence'][ix][iy], 'unknown')
                        else:
                            z = record['surface_z_container'][ix][iy]
                            self.assert_close(
                                z - floor_v2, legacy['height'][ix][iy])
                            self.assert_close(
                                record['height_above_floor'][ix][iy],
                                legacy['height'][ix][iy])
                            if state_v2 == 'occupied':
                                self.assertEqual(
                                    record['confidence'][ix][iy],
                                    'verified_geometry')
                # Raster oracle: the occupied set equals the exact
                # positive-area cell intersection under the declared
                # half-open rule, per cell (never only an array sum).
                occupied_columns = {
                    (ix, iy)
                    for ix in range(nx) for iy in range(ny)
                    if record['state'][ix][iy] == 'occupied'}
                expected_columns = exact_occupied_columns(
                    center, WDH, yaw_c, mapper.center)
                self.assertEqual(occupied_columns, expected_columns,
                                 'exact-intersection mismatch')
                expected_top = exact_top_height(center, WDH, mapper.center)
                for ix, iy in occupied_columns:
                    self.assert_close(
                        legacy['height'][ix][iy], expected_top)
                    self.assert_close(
                        record['surface_z_container'][ix][iy],
                        floor_v2 + expected_top)
                observed.update(stage='oracles', cells=len(occupied_columns),
                                expected_top=expected_top)
                count += 1
        self.assertEqual(count, 64)

    def test_ledger_lifecycle_and_normalization(self):
        with case('ledger/idempotent-and-provenance', {}) as observed:
            mapper = make_mapper(.53)
            center, yaw = commit_v2(mapper, .53, 0., 0.0)
            revision = mapper._revision
            # Duplicate v2 commit is idempotent: no revision bump.
            self.assertFalse(mapper.mark_placed_box(
                center, list(WDH), yaw=yaw, measured_source='verify'))
            self.assertEqual(mapper._revision, revision)
            ledger = mapper.commit_ledger()
            self.assertEqual(len(ledger), 1)
            self.assertEqual(ledger[0]['measured_source'], 'verify_placed_box')
            observed.update(revision=revision)
        with case('ledger/remove-preserves-sensor', {}) as observed:
            mapper = make_mapper(.53)
            # Independent sensor observation far from the committed box.
            sensor_local = (-0.40, 0.20, .53 + 0.10)
            mapper.mark_occupied_world(
                sensor_local[0], sensor_local[1], sensor_local[2],
                source=SOURCE_SENSOR)
            center, yaw = commit_v2(mapper, .53, 0., 0.0,
                                    center_xy=(0.05, -0.03))
            before = mapper.surface_map_2d()
            sensor_cell = (
                int(math.floor((sensor_local[0] + 0.5 * INNER[0]) / RES)),
                int(math.floor((sensor_local[1] + 0.5 * INNER[1]) / RES)))
            self.assertEqual(before['state'][sensor_cell[0]][sensor_cell[1]],
                             'occupied')
            mapper.unmark_placed_box(center, list(WDH), yaw=yaw)
            after = mapper.surface_map_2d()
            # The sensor column survives removal; the box cells go unknown.
            self.assertEqual(after['state'][sensor_cell[0]][sensor_cell[1]],
                             'occupied')
            box_cell = (
                int(math.floor((0.05 + 0.5 * INNER[0]) / RES)),
                int(math.floor((-0.03 + 0.5 * INNER[1]) / RES)))
            self.assertEqual(after['state'][box_cell[0]][box_cell[1]],
                             'unknown')
            observed.update(sensor_cell=sensor_cell, box_cell=box_cell)
        with case('ledger/v1-v2-equivalence', {}) as observed:
            # The declared v1 shim (world pose) and the v2 container path
            # produce identical voxels and one revision semantics.
            container = dict(translation=(1.5, -0.4, 0.2), yaw=0.3)
            world_center = (0.8, 0.1, .53 + .30 + 0.16)
            world_yaw = 0.9
            c1, y1, s1 = normalize_ledger_slot(
                world_center, world_yaw, list(WDH), source_frame='world',
                container_yaw=container['yaw'],
                container_translation=container['translation'])
            c2, y2, s2 = normalize_ledger_slot(
                c1, y1, list(WDH), source_frame='container_link')
            self.assertEqual(c1, c2)
            self.assert_close(y1, y2)
            m1, m2 = make_mapper(.53), make_mapper(.53)
            m1.mark_placed_box(c1, s1, yaw=y1)
            m2.mark_placed_box(c2, s2, yaw=y2)
            self.assertEqual(m1.surface_map_2d(), m2.surface_map_2d())
            observed.update(container_center=c1)

    def test_v2_surface_ingest_adapter(self):
        mapper = make_mapper(.53)
        commit_v2(mapper, .53, .30, math.atan2(4, 3))
        record = mapper.surface_contract_v2(SCENE, 'ingest-1')
        legacy = mapper.surface_map_2d()
        hull = mapper._hull
        with case('ingest/v2-equals-legacy', {'revision':
                                              record['map_revision']}) as o:
            snapshot = OccupancySnapshot.from_surface_2d(
                record, inner_size=list(INNER),
                geometry_descriptor=hull.descriptor(), inflate_m=0.0)
            reference = OccupancySnapshot.from_surface_2d(
                legacy, inflate_m=0.0)
            self.assertEqual(snapshot.obstacle, reference.obstacle)
            self.assertEqual(snapshot.map_revision, reference.map_revision)
            # Datum shift round-trips within the declared 1e-9 tolerance.
            self.assertEqual(len(snapshot.column_top),
                             len(reference.column_top))
            worst = 0.0
            for row_a, row_b in zip(snapshot.column_top,
                                    reference.column_top):
                for a, b in zip(row_a, row_b):
                    worst = max(worst, abs(a - b))
                    self.assert_close(a, b)
            self.assertLessEqual(worst, 1e-9)
            o.update(cells=snapshot.nx * snapshot.ny, worst_top_delta=worst)
        # Adapter failures: declared, fail-closed, zero effects.
        self.mismatch('v2-missing-inner-size', {'kind': 'surface'},
                      lambda: OccupancySnapshot.from_surface_2d(record))
        self.mismatch(
            'v2-missing-descriptor', {'kind': 'surface'},
            lambda: OccupancySnapshot.from_surface_2d(
                record, inner_size=list(INNER)))
        wrong_datum = dict(record, height_datum='floor_relative')
        self.mismatch(
            'v2-wrong-datum', {'height_datum': 'floor_relative'},
            lambda: OccupancySnapshot.from_surface_2d(
                wrong_datum, inner_size=list(INNER),
                geometry_descriptor=hull.descriptor()))
        wrong_frame = dict(record, frame_id='world')
        self.mismatch(
            'v2-wrong-frame', {'frame_id': 'world'},
            lambda: OccupancySnapshot.from_surface_2d(
                wrong_frame, inner_size=list(INNER),
                geometry_descriptor=hull.descriptor()))
        null_bad = copy.deepcopy(record)
        null_bad['state'][0][0] = 'occupied'
        self.mismatch(
            'v2-null-top-with-occupied-state', {},
            lambda: OccupancySnapshot.from_surface_2d(
                null_bad, inner_size=list(INNER),
                geometry_descriptor=hull.descriptor()))
        legacy_missing = copy.deepcopy(legacy)
        del legacy_missing['floor_z']
        self.mismatch(
            'legacy-missing-floor-z', {},
            lambda: OccupancySnapshot.from_surface_2d(legacy_missing))
        legacy_missing_center = copy.deepcopy(legacy)
        del legacy_missing_center['center_base']
        self.mismatch(
            'legacy-missing-center-base', {},
            lambda: OccupancySnapshot.from_surface_2d(legacy_missing_center))
        foreign = copy.deepcopy(legacy)
        foreign['frame'] = 'elfin_base_link'
        self.mismatch('legacy-foreign-frame', {'frame': 'elfin_base_link'},
                      lambda: OccupancySnapshot.from_surface_2d(foreign))
        ambiguous = {'height': [[0.0]], 'state': [['free']]}
        self.mismatch('ambiguous-schema', {},
                      lambda: OccupancySnapshot.from_surface_2d(ambiguous))

    def test_surface_record_negatives(self):
        mapper = make_mapper(.53)
        commit_v2(mapper, .53, .30, 0.0)
        record = mapper.surface_contract_v2(SCENE, 'neg-1')
        ctx = surface_context(record)
        for name, key, value, code in [
            ('wrong-frame', 'frame_id', 'world', 'FRAME_CONTRACT_MISMATCH'),
            ('missing-identity', 'request_id', '', 'MISSING_IDENTITY'),
            ('stale-scene', 'scene_revision', 'stale',
             'CALIBRATION_REVISION_MISMATCH'),
            ('stale-map', 'map_revision', record['map_revision'] - 1,
             'MAP_REVISION_MISMATCH'),
            ('wrong-hull', 'geometry_hash', 'wrong-hull',
             'GEOMETRY_IDENTITY_MISMATCH'),
            ('wrong-version', 'contract_version', 1,
             'CONTRACT_VERSION_MISMATCH'),
            ('wrong-units', 'length_unit', 'mm', 'UNIT_MISMATCH'),
            ('wrong-datum', 'height_datum', 'floor_relative',
             'HEIGHT_DATUM_MISMATCH'),
        ]:
            data = copy.deepcopy(record)
            data[key] = value
            if key == 'map_revision':
                # The context expectation stays at the true revision so the
                # record's own stale map revision is what fires.
                context = BoundaryContext(
                    'F06', 'cargo_volume_mapper', 'surface_v2_subscribers',
                    record['request_id'], SCENE, 'container_link', '',
                    'scene', geometry_hash=record['geometry_hash'],
                    map_revision=record['map_revision'])
            else:
                context = ctx
            with self.subTest(name=name):
                self.reject(name, code, {key: value},
                            lambda: measured_surface_to_message(data,
                                                                context))
        unknown_value = copy.deepcopy(record)
        ix = next(i for i, row in enumerate(record['state'])
                  if 'unknown' in row)
        iy = record['state'][ix].index('unknown')
        unknown_value['surface_z_container'][ix][iy] = \
            record['floor_z_container']
        self.reject('unknown-surface-has-value', 'UNKNOWN_SURFACE_HAS_VALUE',
                    {'cell': [ix, iy]},
                    lambda: measured_surface_to_message(unknown_value, ctx))
        unverified = copy.deepcopy(record)
        ix2 = next(i for i, row in enumerate(record['confidence'])
                   if 'verified_geometry' in row)
        iy2 = record['confidence'][ix2].index('verified_geometry')
        unverified['confidence'][ix2][iy2] = 'planned'
        self.reject('unverified-surface', 'UNVERIFIED_SURFACE',
                    {'cell': [ix2, iy2]},
                    lambda: measured_surface_to_message(unverified, ctx))
        bounds = copy.deepcopy(record)
        bounds['ceiling_z_container'] = bounds['floor_z_container']
        self.reject('height-bounds', 'INVALID_HEIGHT_BOUNDS',
                    {'ceiling': 'floor'},
                    lambda: measured_surface_to_message(bounds, ctx))

    def test_f20_unknown_not_promoted(self):
        with case('integrate/unknown-not-promoted', {}) as observed:
            mapper = make_mapper(.53)
            before = mapper.surface_contract_v2(SCENE, 'f20-1')
            unknown_before = sum(
                1 for row in before['state'] for cell in row
                if cell == 'unknown')
            # A placed-and-removed box returns unknown cells, never free.
            center, yaw = commit_v2(mapper, .53, 0., 0.0)
            mapper.unmark_placed_box(center, list(WDH), yaw=yaw)
            after = mapper.surface_contract_v2(SCENE, 'f20-2')
            free_after = sum(1 for row in after['state'] for cell in row
                             if cell == 'free')
            self.assertEqual(free_after, 0)
            observed.update(unknown_before=unknown_before,
                            free_after=free_after)


if __name__ == '__main__':
    unittest.main()
