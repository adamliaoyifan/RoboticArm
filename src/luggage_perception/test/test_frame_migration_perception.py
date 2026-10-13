"""ST03 perception frame-boundary acceptance (F02-F05) on real types.

Runs after building/sourcing the isolated install; no ROS nodes, no TF, no
YOLO. Sensor ingress cases use fixed stamped synthetic point sets.
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

import numpy as np
from rclpy.serialization import serialize_message, deserialize_message

from luggage_description.frame_contracts import ContractError, require_horizontal
from luggage_description.frame_diagnostics import (
    BoundaryContext, BoundaryContractError,
)
from luggage_description.frame_transforms import rotate_covariance3
from luggage_perception.frame_boundary_records import (
    admit_sensor_observation, contact_boundary, detection_boundary,
    opening_boundary, stamp_ns_from_seconds,
)
from luggage_perception.frame_message_adapters import (
    contact_to_message, contact_from_message,
    detection_to_message, detection_from_message,
)
from luggage_perception.container_opening_estimator import OpeningEstimate


SCENE = 'scene-calibration-17'
DEPTH_FRAME = 'camera_depth_optical_frame'
STAMP_SEC = 1234.5
STAMP_NS = 1234500000000


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


def context(record, interface, frame='', reference=''):
    return BoundaryContext(
        interface, 'synthetic-producer', 'perception-boundary-consumer',
        record.get('request_id', 'request-17'), SCENE,
        frame or record.get('frame_id', ''),
        reference or record.get('reference', ''), 'acquisition')


def observation_record(points=None, frame_id=DEPTH_FRAME,
                       stamp_ns=STAMP_NS, request='observe-1'):
    return {
        'contract_version': 2,
        'frame_id': frame_id,
        'length_unit': 'm',
        'stamp_ns': stamp_ns,
        'time_basis': 'acquisition',
        'scene_revision': SCENE,
        'request_id': request,
        'points': points,
    }


def synthetic_points(yaw, translation, count=6):
    """Fixed stamped synthetic point set carried through the transform."""
    base = [(0.1, 0.0, 1.2), (-0.2, 0.3, 1.1), (0.4, -0.1, 1.3),
            (0.0, 0.25, 1.05), (0.15, -0.35, 0.95), (-0.3, -0.2, 1.25)]
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    points = []
    for x, y, z in base[:count]:
        points.append([translation[0] + cos_y * x - sin_y * y,
                       translation[1] + sin_y * x + cos_y * y,
                       translation[2] + z])
    return points


def make_estimate(center, normal=(0.0, 0.0, 1.0), width=0.55, height=0.40,
                  stamp=STAMP_SEC):
    normal = np.asarray(normal, dtype=np.float64)
    seed = np.array([1.0, 0.0, 0.0])
    width_axis = seed - normal * float(np.dot(seed, normal))
    width_axis /= np.linalg.norm(width_axis)
    return OpeningEstimate(
        center=np.asarray(center, dtype=np.float64), normal=normal,
        width_axis=width_axis, width=width, height=height,
        source_status='tag_depth', confidence=0.9,
        pose_covariance=np.diag([0.002, 0.004, 0.006]),
        aperture_covariance=np.diag([0.001, 0.003, 0.001]),
        stamp=float(stamp), age_sec=0.0, fresh=True, accepted=True)


class FrameMigrationPerception(unittest.TestCase):
    def assert_vector(self, actual, expected, tolerance=1e-9):
        self.assertEqual(len(actual), len(expected))
        self.assertLessEqual(max(abs(a - b) for a, b in zip(actual, expected)),
                             tolerance)

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

    def test_64_perception_geometry_cases(self):
        matrix = itertools.product(
            (0., math.pi / 2, math.pi, math.atan2(4, 3)),
            ((0., 0., 0.), (1.5, -.4, .2)), (.53, .71), (0., .30),
            ((0., 0., .16), (.10, -.05, .16)))
        count = 0
        for index, (yaw, translation, floor, support, offset) in enumerate(matrix):
            center_world = [translation[0] + .41, translation[1] - .13,
                            translation[2] + floor + support + .16]
            box_yaw = .37
            fixture = dict(yaw=yaw, translation=translation, floor=floor,
                           support=support, offset=offset,
                           center_world=center_world)
            with self.subTest(index=index), \
                    case('perception/%02d' % index, fixture) as observed:
                # F03 detection: world box center, box-axis yaw, exact stamp.
                request = 'matrix-%02d' % index
                detection, yaw_source = detection_boundary(
                    center_world, box_yaw, STAMP_SEC, SCENE, request)
                self.assertEqual(yaw_source, 'measured')
                det_ctx = context(detection, 'F03')
                encoded = detection_to_message(detection, det_ctx)
                decoded = detection_from_message(wire(encoded), det_ctx)
                self.assertEqual(decoded, detection)
                self.assertEqual(decoded['stamp_ns'], STAMP_NS)
                observed.update(stage='detection',
                                detection_wire_hex=serialize_message(
                                    encoded).hex())
                # F04 contact: world surface-contact point, +Z normal.
                contact_quat = [0., 0., 0., 1.]
                contact_position = [
                    center_world[0]
                    + math.cos(box_yaw) * offset[0]
                    - math.sin(box_yaw) * offset[1],
                    center_world[1]
                    + math.sin(box_yaw) * offset[0]
                    + math.cos(box_yaw) * offset[1],
                    center_world[2] + offset[2] - .16,
                ]
                candidate = _candidate(contact_position, contact_quat)
                contact = contact_boundary(candidate, STAMP_SEC, SCENE, request)
                contact_ctx = context(contact, 'F04')
                contact_encoded = contact_to_message(contact, contact_ctx)
                contact_decoded = contact_from_message(
                    wire(contact_encoded), contact_ctx)
                self.assertEqual(contact_decoded, contact)
                observed.update(stage='contact',
                                contact_wire_hex=serialize_message(
                                    contact_encoded).hex())
                # F02 sensor admission: fixed stamped synthetic point set in
                # the truthful sensor frame; epoch zero stays valid.
                points = synthetic_points(yaw, translation)
                observation = observation_record(points)
                admit_sensor_observation(observation, context(observation,
                                                               'F02'))
                epoch = observation_record(points, stamp_ns=0)
                admit_sensor_observation(epoch, context(epoch, 'F02'))
                observed.update(stage='observation', point_count=len(points))
                # F05 opening: aperture vertices reach world only through an
                # independent transform; compare against analytic math.
                estimate = make_estimate(center_world)
                opening = opening_boundary(estimate, DEPTH_FRAME, SCENE,
                                           request)
                self.assertEqual(opening['stamp_ns'], STAMP_NS)
                cos_y, sin_y = math.cos(yaw), math.sin(yaw)
                # Path A: rotate each opening-frame vertex by the record's
                # opening orientation (hand quaternion math), add the center,
                # then apply the frame rotation+translation.
                q = opening['quaternion_xyzw']
                half_w, half_h = opening['width'] / 2.0, opening['height'] / 2.0
                for sign, vertex in zip(((-1, -1), (1, -1), (1, 1), (-1, 1)),
                                        opening['aperture_vertices_opening']):
                    rotated = _rotate_by(q, vertex)
                    local = [opening['center'][i] + rotated[i]
                             for i in range(3)]
                    world = [translation[0] + cos_y * local[0]
                             - sin_y * local[1],
                             translation[1] + sin_y * local[0]
                             + cos_y * local[1],
                             translation[2] + local[2]]
                    # Path B: the analytic vertex position for this corner,
                    # written from width/height signs and rotation math only.
                    corner = [opening['center'][0] + sign[0] * half_w,
                              opening['center'][1] + sign[1] * half_h,
                              opening['center'][2]]
                    expected = [translation[0] + cos_y * corner[0]
                                - sin_y * corner[1],
                                translation[1] + sin_y * corner[0]
                                + cos_y * corner[1],
                                translation[2] + corner[2]]
                    self.assert_vector(world, expected)
                # Covariance rotation: analytic diagonal-anisotropic case.
                world_quaternion = [0., 0., math.sin(yaw / 2),
                                    math.cos(yaw / 2)]
                covariance = rotate_covariance3(
                    world_quaternion, [1., 0., 0., 0., 2., 0., 0., 0., 3.])
                self.assert_vector(
                    covariance,
                    [cos_y * cos_y + 2 * sin_y * sin_y, -cos_y * sin_y, 0.,
                     -cos_y * sin_y, sin_y * sin_y + 2 * cos_y * cos_y, 0.,
                     0., 0., 3.])
                observed.update(stage='opening+covariance',
                                opening_stamp_ns=opening['stamp_ns'])
                count += 1
        self.assertEqual(count, 64)

    def test_yaw_fallback_declared(self):
        with case('fallback/near-square-yaw', {}) as observed:
            record, source = detection_boundary(
                (.5, -.25, .8), 1.234, STAMP_SEC, SCENE, 'fallback-1',
                yaw_valid=False, fallback_yaw=0.0)
            self.assertEqual(source, 'yaw_fallback')
            self.assert_quaternion_exact(record['quaternion_xyzw'],
                                         [0., 0., 0., 1.])
            observed.update(yaw_source=source)
        # A declared fallback is never relabelled as measured orientation.
        record, source = detection_boundary(
            (.5, -.25, .8), 1.234, STAMP_SEC, SCENE, 'fallback-2',
            yaw_valid=False, fallback_yaw=math.pi / 2)
        self.assertEqual(source, 'yaw_fallback')
        self.assert_quaternion_exact(
            record['quaternion_xyzw'],
            [0., 0., math.sin(math.pi / 4), math.cos(math.pi / 4)])

    def test_sensor_observation_negatives(self):
        good = observation_record(synthetic_points(0., (0., 0., 0.)))
        self.reject('observation-missing-frame', 'MISSING_IDENTITY',
                    dict(good, frame_id=''),
                    lambda: admit_sensor_observation(
                        dict(good, frame_id=''),
                        context(good, 'F02', frame=DEPTH_FRAME)))
        self.reject('observation-missing-stamp', 'INVALID_STAMP',
                    dict(good, stamp_ns=None),
                    lambda: admit_sensor_observation(
                        dict(good, stamp_ns=None), context(good, 'F02')))
        self.reject('observation-negative-stamp', 'INVALID_STAMP',
                    dict(good, stamp_ns=-1),
                    lambda: admit_sensor_observation(
                        dict(good, stamp_ns=-1), context(good, 'F02')))
        self.reject('observation-wrong-basis', 'TIME_BASIS_MISMATCH',
                    dict(good, time_basis='scene'),
                    lambda: admit_sensor_observation(
                        dict(good, time_basis='scene'),
                        context(dict(good, time_basis='scene'), 'F02')))
        self.reject('observation-stale-scene', 'CALIBRATION_REVISION_MISMATCH',
                    dict(good, scene_revision='stale'),
                    lambda: admit_sensor_observation(
                        dict(good, scene_revision='stale'),
                        context(good, 'F02')))
        self.reject('observation-nonfinite-point', 'INVALID_NUMERIC_PAYLOAD',
                    dict(good, points=[[math.nan, 0., 1.]]),
                    lambda: admit_sensor_observation(
                        dict(good, points=[[math.nan, 0., 1.]]),
                        context(good, 'F02')))
        self.reject('observation-malformed-point', 'INVALID_NUMERIC_PAYLOAD',
                    dict(good, points=[[1., 2.]]),
                    lambda: admit_sensor_observation(
                        dict(good, points=[[1., 2.]]),
                        context(good, 'F02')))
        # Expected frame declared by the boundary: mismatched truth rejects.
        self.reject('observation-wrong-frame', 'FRAME_CONTRACT_MISMATCH',
                    dict(good, frame_id='other_camera_frame'),
                    lambda: admit_sensor_observation(
                        dict(good, frame_id='other_camera_frame'),
                        context(good, 'F02', frame=DEPTH_FRAME)))

    def test_detection_and_contact_negatives(self):
        detection, _ = detection_boundary((.5, -.25, .8), .3, STAMP_SEC,
                                          SCENE, 'neg-1')
        for name, key, value, code in [
            ('detection-missing-frame', 'frame_id', '', 'MISSING_IDENTITY'),
            ('detection-wrong-frame', 'frame_id', 'container_link',
             'FRAME_CONTRACT_MISMATCH'),
            ('detection-wrong-reference', 'reference', 'surface_contact',
             'POSE_REFERENCE_MISMATCH'),
            ('detection-wrong-version', 'contract_version', 1,
             'CONTRACT_VERSION_MISMATCH'),
            ('detection-wrong-units', 'length_unit', 'mm', 'UNIT_MISMATCH'),
            ('detection-stale-scene', 'scene_revision', 'stale',
             'CALIBRATION_REVISION_MISMATCH'),
            ('detection-missing-stamp', 'stamp_ns', None, 'INVALID_STAMP'),
            ('detection-wrong-basis', 'time_basis', 'scene',
             'TIME_BASIS_MISMATCH'),
            ('detection-nonfinite', 'position', [math.inf, 0., 0.],
             'INVALID_NUMERIC_PAYLOAD'),
            ('detection-nonunit-quaternion', 'quaternion_xyzw',
             [0., 0., 0., 2.], 'INVALID_QUATERNION'),
        ]:
            data = copy.deepcopy(detection)
            data[key] = value
            with self.subTest(name=name):
                self.reject(name, code, data,
                            lambda: detection_to_message(
                                data, context(detection, 'F03')))
        # Wire-side: a generated message tampered after encode rejects.
        message = detection_to_message(detection, context(detection, 'F03'))
        message.identity.frame_id = 'container_link'
        self.reject('detection-wire-frame', 'FRAME_CONTRACT_MISMATCH',
                    {'field': 'frame_id', 'serialized_hex':
                     serialize_message(message).hex()},
                    lambda: detection_from_message(
                        wire(message), context(detection, 'F03')))

    def test_opening_covariance_independence(self):
        # Injected defect: covariance translated like a point (or rotated
        # about the wrong axis) must fail the analytic oracle.
        estimate = make_estimate((.6, -.1, 1.4))
        opening = opening_boundary(estimate, DEPTH_FRAME, SCENE, 'cov-1')
        with case('opening/covariance-oracle',
                  {'pose_covariance': opening['pose_covariance']}) as observed:
            good = rotate_covariance3(
                [0., 0., math.sin(math.pi / 8), math.cos(math.pi / 8)],
                [1., 0., 0., 0., 2., 0., 0., 0., 3.])
            expected = [1.5, -0.5, 0., -0.5, 1.5, 0., 0., 0., 3.]
            self.assert_vector(good, expected)
            # A position-only "rotation" (translation added) is detectably
            # wrong: covariance magnitudes would drift off the analytic row.
            translated = [good[i] + (0.01 if i in (0, 4, 8) else 0.0)
                          for i in range(9)]
            with self.assertRaises(AssertionError):
                self.assert_vector(translated, expected)
            observed.update(analytic=expected)

    def test_roll_gate_full_se3(self):
        roll = .2
        q = [math.sin(roll / 2), 0., 0., math.cos(roll / 2)]
        self.reject('container-tilt', 'UNSUPPORTED_CONTAINER_TILT',
                    {'quaternion': q}, lambda: require_horizontal(q))
        # Full SE(3) math still transforms the tilted pose (substrate rule).
        estimate = make_estimate((.3, .2, 1.0), normal=(0., math.sin(roll),
                                                        math.cos(roll)))
        with case('opening/tilted-normal',
                  {'roll': roll}) as observed:
            opening = opening_boundary(estimate, DEPTH_FRAME, SCENE, 'roll-1')
            self.assertTrue(all(
                math.isfinite(v) for v in opening['normal']))
            observed.update(normal=opening['normal'])

    def test_opening_width_axis_orientation(self):
        # R6 regression: the aperture quaternion must carry the full basis
        # (+X width axis, +Z normal), not the normal-only shortest arc — a
        # non-square aperture keeps its width direction in world.
        matrix = ((0., (0., 0., 1.), (0., 1., 0.)),
                  (math.pi / 4, (0., 0., 1.), (math.cos(math.pi / 4),
                                               math.sin(math.pi / 4), 0.)),
                  (math.pi / 2, (0., 0., 1.), (0., 1., 0.)),
                  (math.atan2(4, 3), (0., math.sin(.2), math.cos(.2)),
                   (math.cos(math.atan2(4, 3)), math.sin(math.atan2(4, 3)),
                    0.)))
        for index, (in_plane, normal, width_axis) in enumerate(matrix):
            with self.subTest(index=index), case(
                    'review/opening-width-axis/%d' % index,
                    {'in_plane': in_plane, 'normal': list(normal),
                     'width_axis': list(width_axis)}) as observed:
                estimate = make_estimate(
                    (.4, -.2, 1.1), normal=normal, width=2.0, height=1.0)
                # make_estimate seeds the width axis from +X; override with
                # the declared in-plane axis of this case.
                estimate.width_axis = np.asarray(width_axis, dtype=np.float64)
                opening = opening_boundary(estimate, 'world', SCENE,
                                           'width-%d' % index)
                quaternion = opening['quaternion_xyzw']
                # Independent oracle: reconstruct the world vertices with
                # the hand-rolled rotation and compare against the
                # estimator's own geometry (width along the declared axis,
                # height perpendicular, both in the aperture plane). A
                # declared axis not exactly perpendicular to the normal is
                # orthogonalized — the aperture plane owns the geometry.
                world = [_rotate_by(quaternion, vertex) for vertex in
                         opening['aperture_vertices_opening']]
                normal_v = np.asarray(normal, dtype=np.float64)
                normal_v = normal_v / np.linalg.norm(normal_v)
                axis_w = np.asarray(width_axis, dtype=np.float64)
                axis_w = axis_w - normal_v * float(np.dot(axis_w, normal_v))
                axis_w = axis_w / np.linalg.norm(axis_w)
                along_w = [float(np.dot(v[:3], axis_w)) for v in world]
                self.assertAlmostEqual(max(along_w) - min(along_w), 2.0,
                                       places=9)
                axis_h = np.cross(normal_v, axis_w)
                axis_h = axis_h / np.linalg.norm(axis_h)
                along_h = [float(np.dot(v[:3], axis_h)) for v in world]
                self.assertAlmostEqual(max(along_h) - min(along_h), 1.0,
                                       places=9)
                # +Z stays the outward normal; +X maps onto the width axis.
                mapped_z = _rotate_by(quaternion, (0., 0., 1.))
                for a, b in zip(mapped_z, normal):
                    self.assertAlmostEqual(a, b, places=9)
                mapped_x = _rotate_by(quaternion, (1., 0., 0.))
                dot = sum(a * b for a, b in zip(mapped_x, axis_w))
                self.assertAlmostEqual(dot, 1.0, places=9)
                observed.update(quaternion=quaternion,
                                width_extent=max(along_w) - min(along_w),
                                height_extent=max(along_h) - min(along_h))
        # The review counterexample itself: a 1x2 world opening must not
        # come back as 2x1.
        estimate = make_estimate((0., 0., 1.0), normal=(0., 0., 1.),
                                 width=2.0, height=1.0)
        estimate.width_axis = np.array([0., 1., 0.])
        opening = opening_boundary(estimate, 'world', SCENE, 'width-probe')
        world = [_rotate_by(opening['quaternion_xyzw'], v) for v in
                 opening['aperture_vertices_opening']]
        x_extent = max(v[0] for v in world) - min(v[0] for v in world)
        y_extent = max(v[1] for v in world) - min(v[1] for v in world)
        with case('review/opening-width-axis/probe-counterexample',
                  {'x_extent': x_extent, 'y_extent': y_extent}):
            self.assertAlmostEqual(x_extent, 1.0, places=9)
            self.assertAlmostEqual(y_extent, 2.0, places=9)
        # A width axis parallel to the normal is a degenerate aperture
        # plane: rejected, never silently rotated.
        degenerate = make_estimate((0., 0., 1.0), normal=(0., 0., 1.))
        degenerate.width_axis = np.array([0., 0., 1.])
        self.reject('width-axis-parallel-to-normal',
                    'INVALID_NUMERIC_PAYLOAD',
                    {'width_axis': [0., 0., 1.]},
                    lambda: opening_boundary(degenerate, DEPTH_FRAME, SCENE,
                                             'width-bad'))

    def test_acquisition_stamp_integer_identity(self):
        # R7 regression: an epoch-scale acquisition stamp survives exactly
        # in integer nanoseconds; float seconds are a transport form that
        # already lost the nanoseconds before any conversion runs.
        from luggage_perception.frame_boundary_records import (
            stamp_ns as stamp_ns_exact, stamp_ns_from_sec_nanosec,
        )

        class RosTime:
            def __init__(self, sec, nanosec):
                self.sec, self.nanosec = sec, nanosec

        sec, nanosec = 1720000000, 123456789
        expected = 1720000000123456789
        with case('review/stamp-epoch-identity',
                  {'sec': sec, 'nanosec': nanosec}) as observed:
            self.assertEqual(stamp_ns_from_sec_nanosec(sec, nanosec),
                             expected)
            self.assertEqual(stamp_ns_exact(expected), expected)
            self.assertEqual(stamp_ns_exact(RosTime(sec, nanosec)),
                             expected)
            # Adjacent nanoseconds stay distinct through every exact form.
            self.assertEqual(
                stamp_ns_exact(RosTime(sec, nanosec + 1)), expected + 1)
            self.assertNotEqual(
                stamp_ns_exact(RosTime(sec, nanosec)),
                stamp_ns_exact(RosTime(sec, nanosec + 1)))
            # The float transport form is documented lossy at this scale —
            # the exact forms above are the ones the publisher uses.
            self.assertEqual(stamp_ns_from_seconds(sec + nanosec * 1e-9),
                             expected - 21)
            observed.update(expected_ns=expected,
                            float_transport_ns=expected - 21)
        # The builder keeps whichever exact form it is given: int ns and
        # the ROS-time object produce identical records, and the record
        # survives real serialization with its nanoseconds intact.
        record_int, _ = detection_boundary(
            (.5, -.25, .8), .3, expected, SCENE, 'stamp-epoch-int')
        record_time, _ = detection_boundary(
            (.5, -.25, .8), .3, RosTime(sec, nanosec), SCENE,
            'stamp-epoch-time')
        self.assertEqual(record_int['stamp_ns'], expected)
        self.assertEqual(record_time['stamp_ns'], expected)
        with case('review/stamp-roundtrip-serialization',
                  {'stamp_ns': expected}) as observed:
            message = detection_to_message(
                record_int, context(record_int, 'F03'))
            decoded = detection_from_message(
                wire(message), context(record_int, 'F03'))
            self.assertEqual(decoded['stamp_ns'], expected)
            adjacent, _ = detection_boundary(
                (.5, -.25, .8), .3, expected + 1, SCENE, 'stamp-adjacent')
            decoded_adjacent = detection_from_message(
                wire(detection_to_message(adjacent,
                                          context(adjacent, 'F03'))),
                context(adjacent, 'F03'))
            self.assertEqual(decoded_adjacent['stamp_ns'], expected + 1)
            self.assertNotEqual(decoded['stamp_ns'],
                                decoded_adjacent['stamp_ns'])
            observed.update(adjacent_ns=expected + 1)

    def assert_quaternion_exact(self, actual, expected):
        self.assertEqual([round(v, 12) for v in actual],
                         [round(v, 12) for v in expected])


def _rotate_by(quaternion_xyzw, vector):
    """Hand-rolled quaternion rotation (independent of the substrate)."""
    x, y, z, w = quaternion_xyzw
    vx, vy, vz = vector
    # v' = v + 2*q_vec x (q_vec x v + w*v)
    cross1 = (y * vz - z * vy, z * vx - x * vz, x * vy - y * vx)
    inner = (cross1[0] + w * vx, cross1[1] + w * vy, cross1[2] + w * vz)
    cross2 = (y * inner[2] - z * inner[1], z * inner[0] - x * inner[2],
              x * inner[1] - y * inner[0])
    return (vx + 2 * cross2[0], vy + 2 * cross2[1], vz + 2 * cross2[2])


class _ContactPose:
    def __init__(self, position, quaternion):
        class Orientation:
            pass
        self.position = type('Point', (), dict(
            x=position[0], y=position[1], z=position[2]))()
        orientation = Orientation()
        orientation.x, orientation.y, orientation.z, orientation.w = quaternion
        self.orientation = orientation


def _candidate(position, quaternion, frame='world', stamp=STAMP_SEC):
    holder = type('Candidate', (), {})()
    holder.frame = frame
    holder.stamp = stamp
    holder.instance_id = 'box-1'
    holder.generation = 7
    holder.contact_pose = _ContactPose(position, quaternion)
    return holder


if __name__ == '__main__':
    unittest.main()
