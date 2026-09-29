"""ST02 acceptance on real Humble generated types, with replayable case records.

Run after building/sourcing the isolated install; no stand-ins and no ROS nodes.
FRAME_MIGRATION_CASE_LOG selects an append-only JSONL evidence file.
"""
import copy
from contextlib import contextmanager
from dataclasses import replace
import itertools
import json
import math
import os
from pathlib import Path
import time
import traceback
import unittest

from rclpy.serialization import serialize_message, deserialize_message
from luggage_msgs.msg import (
    PoseContractV2, SurfaceContractV2, WaypointArrayV2, JointTrajectoryContractV2,
    BoundaryDiagnosticV2, MotionSegment, SlotSpec, DetectedLuggage,
)
from luggage_msgs.srv import SelectPlacePath, ValidatePoseV2
from luggage_msgs.action import PlanMotion
from luggage_description.frame_contracts import ContractError, require_horizontal
from luggage_description.frame_diagnostics import BoundaryContext, BoundaryContractError
from luggage_description.frame_transforms import (
    transform_pose, box_to_tool_target, rotate_vector, rotate_covariance3,
)
from luggage_description.ros_frame_bindings.diagnostics import diagnostic_to_message
from luggage_planning.frame_message_adapters import (
    planning_pose_to_message, planning_pose_from_message,
    cartesian_path_to_message, cartesian_path_from_message,
    joint_path_to_message, joint_path_from_message,
)
from luggage_perception.frame_message_adapters import (
    detection_to_message, detection_from_message, contact_to_message, contact_from_message,
    measured_surface_to_message, measured_surface_from_message,
)


SCENE = 'scene-calibration-17'
HULL = 'sha256:measured-hull'
STAMP = 9876543210987654321  # Deliberately beyond float64's exact integer range.


def wire(message):
    return deserialize_message(serialize_message(message), type(message))


def identity(frame='container_link', basis='scene', request='request-17'):
    return dict(contract_version=2, frame_id=frame, length_unit='m', stamp_ns=STAMP,
                time_basis=basis, scene_revision=SCENE, request_id=request)


def slot_record():
    return dict(identity(), kind='pose', reference='box_center', orientation_semantics='box_axis_pi',
                position=[.41, -.13, .99], quaternion_xyzw=[0., 0., 0., 1.])


def context(record, interface='F08', reference=None):
    return BoundaryContext(interface, 'synthetic-producer', 'boundary-consumer',
                           record['request_id'], SCENE, record.get('frame_id', ''),
                           record.get('reference', '') if reference is None else reference,
                           record['time_basis'], HULL, 7)


def surface_record():
    return dict(identity(), kind='surface', geometry_hash=HULL, map_revision=7,
                height_datum='container_origin', origin_xy_container=[-.4, -.2],
                resolution_m=.05, floor_z_container=.53, ceiling_z_container=2.,
                surface_z_container=[[None, .53, .83]],
                state=[['unknown', 'free', 'occupied']],
                confidence=[['unknown', 'sensor', 'verified_geometry']],
                height_above_floor=[[None, 0., .30]])


def joint_record():
    return dict(contract_version=2, kind='joint_trajectory', request_id='request-17',
                scene_revision=SCENE, stamp_ns=STAMP, time_basis='robot_state', angle_unit='rad',
                joint_names=['joint3', 'joint1', 'joint2'], points=[
                    dict(positions=[.1, -.2, .3], velocities=[0., .1, .2],
                         accelerations=[], effort=[], time_from_start_ns=0),
                    dict(positions=[.3, .4, -.5], velocities=[], accelerations=[.1, .2, .3],
                         effort=[1., 2., 3.], time_from_start_ns=1000000001)])


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
    row = dict(case_id=case_id, input=copy.deepcopy(inputs), status='pass', observations={})
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
                handle.write(json.dumps(json_safe(row), allow_nan=False, sort_keys=True) + '\n')
                handle.flush()
                os.fsync(handle.fileno())
        print('CASE', case_id, row['status'], flush=True)


class FrameMigrationMessages(unittest.TestCase):
    def assert_vector(self, actual, expected, tolerance=1e-9):
        self.assertEqual(len(actual), len(expected))
        self.assertLessEqual(max(abs(a-b) for a, b in zip(actual, expected)), tolerance)

    def assert_quaternion(self, actual, expected, tolerance=1e-6):
        self.assertEqual(len(actual), 4)
        error = min(max(abs(a-b) for a, b in zip(actual, expected)),
                    max(abs(a+b) for a, b in zip(actual, expected)))
        self.assertLessEqual(error, tolerance)

    def test_64_geometry_serialization_cases(self):
        matrix = itertools.product((0., math.pi/2, math.pi, math.atan2(4, 3)),
                                   ((0., 0., 0.), (1.5, -.4, .2)), (.53, .71), (0., .30),
                                   ((0., 0., .16), (.10, -.05, .16)))
        count = 0
        for index, (yaw, translation, floor, support, offset) in enumerate(matrix):
            box = slot_record()
            beta = .37
            box['position'][2] = floor + support + .32/2
            box['quaternion_xyzw'] = [0., 0., math.sin(beta/2), math.cos(beta/2)]
            box['request_id'] = 'matrix-%02d' % index
            tf = dict(source_frame='container_link', target_frame='world', translation=list(translation),
                      quaternion_xyzw=[0., 0., math.sin(yaw/2), math.cos(yaw/2)],
                      scene_revision=SCENE, stamp_ns=None)
            grasp = dict(source_frame='suction_contact_frame', target_frame='box',
                         scene_revision=SCENE, translation=list(offset), quaternion_xyzw=[1., 0., 0., 0.])
            fixture = dict(box=box, transform=tf, grasp=grasp, floor=floor, support=support, wdh=[.80, .50, .32])
            with self.subTest(index=index), case('positive/%02d' % index, fixture) as observed:
                original = copy.deepcopy(box)
                bound_box = planning_pose_from_message(wire(planning_pose_to_message(box, 'slot', context(box))), 'slot', context(box))
                tool = box_to_tool_target(bound_box, grasp)
                pure = transform_pose(tool, tf)
                encoded = planning_pose_to_message(pure, 'motion', context(pure))
                actual = planning_pose_from_message(wire(encoded), 'motion', context(pure))
                c, s = math.cos(yaw), math.sin(yaw)
                cb, sb = math.cos(yaw+beta), math.sin(yaw+beta)
                expected_position = [translation[0]+c*.41-s*(-.13)+cb*offset[0]-sb*offset[1],
                                     translation[1]+s*.41+c*(-.13)+sb*offset[0]+cb*offset[1],
                                     translation[2]+floor+support+.16+offset[2]]
                expected_q = [math.cos((yaw+beta)/2), math.sin((yaw+beta)/2), 0., 0.]
                self.assert_vector(actual['position'], expected_position)
                self.assert_quaternion(pure['quaternion_xyzw'], expected_q, 1e-9)
                self.assert_quaternion(actual['quaternion_xyzw'], expected_q)
                self.assertEqual(actual, pure)
                detected = transform_pose(bound_box, tf)
                detected['time_basis'] = 'acquisition'
                det_ctx = context(detected, 'F03')
                self.assertEqual(detection_from_message(wire(detection_to_message(detected, det_ctx)), det_ctx), detected)
                contact = copy.deepcopy(detected)
                contact.update(position=expected_position, reference='surface_contact', orientation_semantics='surface_normal')
                contact_ctx = context(contact, 'F04')
                self.assertEqual(contact_from_message(wire(contact_to_message(contact, contact_ctx)), contact_ctx), contact)
                self.assertEqual(box, original)
                vector = rotate_vector(tf['quaternion_xyzw'], [.6, .8, 0.])
                self.assert_vector(vector, [c*.6-s*.8, s*.6+c*.8, 0.])
                covariance = rotate_covariance3(tf['quaternion_xyzw'], [1., 0., 0., 0., 2., 0., 0., 0., 3.])
                self.assert_vector(covariance, [c*c+2*s*s, -c*s, 0., -c*s, s*s+2*c*c, 0., 0., 0., 3.])
                require_horizontal(tf['quaternion_xyzw'])
                second = copy.deepcopy(actual)
                second['stamp_ns'] += 1
                second['time_basis'] = 'robot_state'
                second['position'][0] += .1
                path = dict(identity('world', request=box['request_id']), kind='waypoints',
                            geometry_hash=HULL, map_revision=7, waypoints=[actual, second])
                path_ctx = context(path, reference='suction_contact_frame')
                path_msg = cartesian_path_to_message(path, 'motion', path_ctx)
                decoded = cartesian_path_from_message(wire(path_msg), 'motion', path_ctx)
                self.assertEqual(decoded, path)
                path_msg.waypoints[0].pose.position.x += 10.
                self.assertEqual(decoded, path)
                decoded['waypoints'][1]['position'][0] += 7.
                self.assertNotEqual(decoded, path)
                surface = surface_record()
                surface.update(request_id=box['request_id'], floor_z_container=floor,
                               surface_z_container=[[None, floor, floor+support]],
                               height_above_floor=[[None, 0., support]])
                surface_msg = measured_surface_to_message(surface, context(surface, 'F06'))
                self.assertEqual(measured_surface_from_message(wire(surface_msg), context(surface, 'F06')), surface)
                joints = joint_record()
                joints['request_id'] = box['request_id']
                joint_ctx = context(joints, 'F15')
                joint_msg = joint_path_to_message(joints, joint_ctx)
                self.assertEqual(joint_path_from_message(wire(joint_msg), joint_ctx), joints)
                diag = wire(diagnostic_to_message(actual, context(actual)))
                self.assertEqual(diag.code, 'OK')
                self.assertEqual(diag.request_id, box['request_id'])
                self.assertEqual(diag.declared_stamp_ns, str(STAMP))
                encoded.pose.orientation.x = .5
                self.assertEqual(actual, pure)
                observed.update(expected_position=expected_position, expected_quaternion=expected_q,
                                actual=actual, covariance=covariance, rotated_vector=vector,
                                diagnostic_request=diag.request_id, identity_loss=0)
                count += 1
        self.assertEqual(count, 64)

    def reject(self, name, expected, payload, operation):
        effects = []
        with case('negative/' + name, payload) as observed:
            try:
                result = operation()
                effects.append(result)  # The only downstream dispatch point.
            except ContractError as error:
                observed.update(expected_code=expected, actual_code=error.code,
                                diagnostic=getattr(error, 'diagnostic', error.as_dict()), downstream_effects=len(effects))
                self.assertEqual(error.code, expected)
                self.assertEqual(effects, [])
                if isinstance(error, BoundaryContractError):
                    self.assertEqual(error.diagnostic['producer'], 'synthetic-producer')
                    self.assertEqual(error.diagnostic['consumer'], 'boundary-consumer')
            else:
                self.fail('unexpected accept: ' + name)

    def test_named_pose_negatives(self):
        changes = [
            ('missing-frame', 'frame_id', '', 'MISSING_IDENTITY'),
            ('wrong-frame', 'frame_id', 'world', 'FRAME_CONTRACT_MISMATCH'),
            ('wrong-reference', 'reference', 'surface_contact', 'POSE_REFERENCE_MISMATCH'),
            ('wrong-version', 'contract_version', 0, 'CONTRACT_VERSION_MISMATCH'),
            ('wrong-units', 'length_unit', 'mm', 'UNIT_MISMATCH'),
            ('stale-scene', 'scene_revision', 'stale', 'CALIBRATION_REVISION_MISMATCH'),
            ('missing-stamp', 'stamp_ns', None, 'INVALID_STAMP'),
            ('wrong-time-basis', 'time_basis', 'acquisition', 'TIME_BASIS_MISMATCH'),
            ('malformed-pose', 'position', [1., 2.], 'INVALID_NUMERIC_PAYLOAD'),
            ('nonfinite-pose', 'position', [math.nan, 0., 0.], 'INVALID_NUMERIC_PAYLOAD'),
            ('nonunit-quaternion', 'quaternion_xyzw', [0., 0., 0., 2.], 'INVALID_QUATERNION'),
            ('request-mismatch', 'request_id', 'other', 'REQUEST_IDENTITY_MISMATCH'),
            ('stamp-overflow', 'stamp_ns', 2**64, 'INTEGER_OUT_OF_RANGE'),
            ('extra-provenance-not-silently-lost', 'unbound_provenance', 'lost?', 'UNSUPPORTED_RECORD_FIELD'),
        ]
        ctx = context(slot_record())
        for name, key, value, code in changes:
            data = slot_record()
            data[key] = value
            with self.subTest(name=name):
                self.reject(name, code, data, lambda: planning_pose_to_message(data, 'slot', ctx))
        self.reject('empty-generated-pose', 'CONTRACT_VERSION_MISMATCH', {},
                    lambda: planning_pose_from_message(wire(PoseContractV2()), 'slot', ctx))
        self.reject('v1-is-not-v2', 'MESSAGE_TYPE_MISMATCH', {'type': 'SlotSpec'},
                    lambda: planning_pose_from_message(wire(SlotSpec()), 'slot', ctx))
        for name, field, value, code in [
            ('wire-frame', 'frame_id', 'world', 'FRAME_CONTRACT_MISMATCH'),
            ('wire-no-frame', 'frame_id', '', 'MISSING_IDENTITY'),
            ('wire-missing-stamp', 'stamp_valid', False, 'INVALID_STAMP'),
            ('wire-version', 'contract_version', 1, 'CONTRACT_VERSION_MISMATCH'),
        ]:
            message = planning_pose_to_message(slot_record(), 'slot', ctx)
            setattr(message.identity, field, value)
            self.reject(name, code, {'field': field, 'value': value},
                        lambda: planning_pose_from_message(wire(message), 'slot', ctx))

    def test_surface_negatives(self):
        ctx = context(surface_record(), 'F06')
        cases = [
            ('stale-map', 'map_revision', 6, 'MAP_REVISION_MISMATCH'),
            ('wrong-hull', 'geometry_hash', 'wrong-hull', 'GEOMETRY_IDENTITY_MISMATCH'),
            ('surface-units', 'length_unit', 'mm', 'UNIT_MISMATCH'),
            ('surface-unknown-value', 'surface_z_container', [[.53, .53, .83]], 'UNKNOWN_SURFACE_HAS_VALUE'),
            ('surface-unverified', 'confidence', [['unknown', 'planned', 'sensor']], 'UNVERIFIED_SURFACE'),
        ]
        for name, field, value, code in cases:
            data = surface_record()
            data[field] = value
            self.reject(name, code, data, lambda: measured_surface_to_message(data, ctx))
        mutations = [
            ('wire-stale-map', 'map_revision', 6, 'MAP_REVISION_MISMATCH'),
            ('wire-wrong-hull', 'geometry_hash', 'wrong-hull', 'GEOMETRY_IDENTITY_MISMATCH'),
            ('wire-surface-size', 'rows', 2, 'INVALID_GRID'),
            ('wire-surface-validity', 'valid', [True, True, True], 'UNKNOWN_SURFACE_HAS_VALUE'),
            ('wire-hidden-unknown-value', 'surface_z_container', [1., .53, .83], 'INVALID_GRID'),
            ('wire-unflagged-derived-height', 'has_height_above_floor', False, 'INVALID_GRID'),
        ]
        for name, field, value, code in mutations:
            msg = measured_surface_to_message(surface_record(), ctx)
            setattr(msg, field, value)
            self.reject(name, code, {'field': field, 'value': value},
                        lambda: measured_surface_from_message(wire(msg), ctx))

    def test_waypoint_negatives(self):
        pose = slot_record()
        pose.update(frame_id='world', reference='suction_contact_frame', orientation_semantics='tool_quaternion')
        path = dict(identity('world'), kind='waypoints', geometry_hash=HULL, map_revision=7, waypoints=[pose])
        ctx = context(path, reference='suction_contact_frame')
        for name, code, change in [
            ('empty-path', 'INVALID_WAYPOINTS', lambda p: p.update(waypoints=[])),
            ('path-map', 'MAP_REVISION_MISMATCH', lambda p: p.update(map_revision=6)),
            ('path-hull', 'GEOMETRY_IDENTITY_MISMATCH', lambda p: p.update(geometry_hash='wrong')),
            ('mixed-waypoint-request', 'WAYPOINT_IDENTITY_MISMATCH', lambda p: p['waypoints'][0].update(request_id='other')),
            ('waypoint-wrong-frame', 'FRAME_CONTRACT_MISMATCH', lambda p: p['waypoints'][0].update(frame_id='container_link')),
        ]:
            data = copy.deepcopy(path)
            change(data)
            self.reject(name, code, data, lambda: cartesian_path_to_message(data, 'motion', ctx))
        msg = cartesian_path_to_message(path, 'motion', ctx)
        msg.waypoints[0].identity.request_id = 'other'
        self.reject('wire-mixed-waypoint-request', 'REQUEST_IDENTITY_MISMATCH', {'path': path, 'request_override': 'other'},
                    lambda: cartesian_path_from_message(wire(msg), 'motion', ctx))

    def test_joint_negatives(self):
        ctx = context(joint_record(), 'F15')
        mutations = [
            ('duplicate-joint-names', 'INVALID_JOINT_NAMES', lambda r: r.update(joint_names=['j1', 'j1', 'j3'])),
            ('joint-vector-size', 'JOINT_VECTOR_SIZE_MISMATCH', lambda r: r['points'][0].update(positions=[1.])),
            ('joint-velocity-size', 'JOINT_VECTOR_SIZE_MISMATCH', lambda r: r['points'][0].update(velocities=[1.])),
            ('joint-nonfinite', 'INVALID_NUMERIC_PAYLOAD', lambda r: r['points'][0].update(positions=[math.inf, 0., 0.])),
            ('nonmonotonic-joint-times', 'INVALID_TRAJECTORY_TIME', lambda r: r['points'][1].update(time_from_start_ns=0)),
            ('negative-joint-time', 'INVALID_TRAJECTORY_TIME', lambda r: r['points'][0].update(time_from_start_ns=-1)),
            ('joint-cartesian-label', 'JOINT_SPACE_LABEL_MISMATCH', lambda r: r.update(frame_id='world')),
            ('joint-stamp-overflow', 'INTEGER_OUT_OF_RANGE', lambda r: r.update(stamp_ns=2**64)),
            ('joint-wrong-unit', 'UNIT_MISMATCH', lambda r: r.update(angle_unit='deg')),
        ]
        for name, code, mutation in mutations:
            data = joint_record()
            mutation(data)
            self.reject(name, code, data, lambda: joint_path_to_message(data, ctx))
        for name, code, mutation in [
            ('wire-joint-cartesian', 'JOINT_SPACE_LABEL_MISMATCH', lambda m: setattr(m.trajectory.header, 'frame_id', 'world')),
            ('wire-joint-duplicate', 'INVALID_JOINT_NAMES', lambda m: setattr(m.trajectory, 'joint_names', ['j1', 'j1', 'j3'])),
            ('wire-joint-size', 'JOINT_VECTOR_SIZE_MISMATCH', lambda m: setattr(m.trajectory.points[0], 'positions', [0.])),
            ('wire-joint-time', 'INVALID_TRAJECTORY_TIME', lambda m: setattr(m.trajectory.points[1].time_from_start, 'sec', -1)),
            ('wire-joint-nanoseconds', 'INVALID_TRAJECTORY_TIME', lambda m: setattr(m.trajectory.points[1].time_from_start, 'nanosec', 1000000000)),
        ]:
            msg = joint_path_to_message(joint_record(), ctx)
            mutation(msg)
            self.reject(name, code, {'joint_fixture': joint_record(), 'mutation': name},
                        lambda: joint_path_from_message(wire(msg), ctx))

    def test_orientation_and_full_se3(self):
        with case('orientation/q-sign-and-directed-yaw', slot_record()) as result:
            pose = slot_record()
            msg = planning_pose_to_message(pose, 'slot', context(pose))
            msg.pose.orientation.w = -1.
            decoded = planning_pose_from_message(wire(msg), 'slot', context(pose))
            self.assert_quaternion(decoded['quaternion_xyzw'], pose['quaternion_xyzw'])
            yaw0 = [1., 0., 0., 0.]
            yawpi = [0., 1., 0., 0.]
            with self.assertRaises(AssertionError):
                self.assert_quaternion(yaw0, yawpi)
            result.update(q_sign_equivalent=True, directed_tool_yaws_distinct=True)
        roll = .2
        q = [math.sin(roll/2), 0., 0., math.cos(roll/2)]
        self.reject('container-tilt', 'UNSUPPORTED_CONTAINER_TILT', {'quaternion': q}, lambda: require_horizontal(q))
        box = slot_record()
        tf = dict(source_frame='container_link', target_frame='world', translation=[1., 2., 3.],
                  quaternion_xyzw=q, scene_revision=SCENE, stamp_ns=None)
        with case('orientation/full-roll-transform', {'pose': box, 'transform': tf}) as result:
            transformed = transform_pose(box, tf)
            self.assert_vector(transformed['position'], [1.+.41, 2.+math.cos(roll)*(-.13)-math.sin(roll)*.99,
                                                        3.+math.sin(roll)*(-.13)+math.cos(roll)*.99])
            self.assert_quaternion(transformed['quaternion_xyzw'], q, 1e-9)
            transformed.update(time_basis='acquisition')
            self.assertEqual(detection_from_message(wire(detection_to_message(transformed, context(transformed, 'F03'))), context(transformed, 'F03')), transformed)
            result.update(actual=transformed)
        for name, value, code in [
            ('absent-transform-stamp', 'absent', 'INVALID_STAMP'),
            ('wrong-transform-stamp', STAMP-1, 'TRANSFORM_TIME_MISMATCH'),
        ]:
            mutated = copy.deepcopy(tf)
            if value == 'absent':
                del mutated['stamp_ns']
            else:
                mutated['stamp_ns'] = value
            self.reject(name, code, {'pose': box, 'transform': mutated}, lambda: transform_pose(box, mutated))
        self.reject('blank-transform-frame', 'MISSING_IDENTITY', {'target_frame': ' '},
                    lambda: transform_pose(box, dict(tf, target_frame=' ')))
        self.reject('invalid-transform-object', 'INVALID_RECORD', {'transform': None},
                    lambda: transform_pose(box, None))
        self.reject('invalid-grasp-object', 'INVALID_RECORD', {'grasp': None},
                    lambda: box_to_tool_target(box, None))

    def test_mutation_oracles(self):
        box = slot_record()
        box['position'] = [1., 0., 1.]
        tf = dict(source_frame='container_link', target_frame='world', translation=[0., 0., 0.],
                  quaternion_xyzw=[0., 0., math.sin(math.pi/4), math.cos(math.pi/4)], scene_revision=SCENE, stamp_ns=None)
        for mutation in ('wrong-frame-producer', 'position-only-rotation'):
            with case('mutation/' + mutation, {'pose': box, 'transform': tf}) as result:
                bad = transform_pose(box, tf)
                if mutation == 'wrong-frame-producer':
                    bad['frame_id'] = 'container_link'
                    bad['time_basis'] = 'acquisition'
                    ctx = replace(context(bad), frame_id='world')
                    with self.assertRaises(ContractError):
                        detection_to_message(bad, ctx)
                else:
                    bad['quaternion_xyzw'] = box['quaternion_xyzw'][:]
                    bad['time_basis'] = 'acquisition'
                    decoded = detection_from_message(wire(detection_to_message(bad, context(bad))), context(bad))
                    with self.assertRaises(AssertionError):
                        self.assert_quaternion(decoded['quaternion_xyzw'], [0., 0., math.sin(math.pi/4), math.cos(math.pi/4)])
                result.update(defect_detected=True, mutant=bad)

    def test_v1_v2_bindings_epoch_and_independence(self):
        with case('binding/v1-v2', {}) as result:
            types = (MotionSegment, SlotSpec, DetectedLuggage, SelectPlacePath.Request,
                     SelectPlacePath.Response, PlanMotion.Goal, ValidatePoseV2.Request,
                     ValidatePoseV2.Response, BoundaryDiagnosticV2)
            for cls in types:
                self.assertEqual(wire(cls()), cls())
            result['bindings'] = [cls.__module__ + '.' + cls.__name__ for cls in types]
        with case('binding/epoch-zero-and-independence', {}) as result:
            record = slot_record()
            record['stamp_ns'] = 0
            ctx = context(record)
            message = planning_pose_to_message(record, 'slot', ctx)
            original = copy.deepcopy(message)
            record['position'][0] += 12.
            self.assertEqual(message, original)
            output = planning_pose_from_message(wire(message), 'slot', ctx)
            output['position'][0] += 12.
            self.assertEqual(message, original)
            self.assertTrue(message.identity.stamp_valid)
            self.assertEqual(message.identity.stamp_ns, 0)
            surface = surface_record()
            surface_message = measured_surface_to_message(surface, context(surface))
            surface['state'][0][0] = 'free'
            self.assertEqual(surface_message.state[0], 'unknown')
            joint = joint_record()
            joint_message = joint_path_to_message(joint, context(joint))
            joint['points'][0]['positions'][0] += 1.
            joint['joint_names'][0] = 'changed'
            self.assertEqual(joint_message.trajectory.joint_names[0], 'joint3')
            self.assertAlmostEqual(joint_message.trajectory.points[0].positions[0], .1)
            result.update(epoch_zero_preserved=True, independent_outputs=True)
        bad = slot_record()
        bad['frame_id'] = 'world'
        with case('diagnostic/rejected-wire-payload', bad) as result:
            ctx = context(slot_record())
            try:
                planning_pose_to_message(bad, 'slot', ctx)
            except BoundaryContractError as error:
                diag = wire(diagnostic_to_message(bad, ctx, error))
                self.assertFalse(diag.accepted)
                self.assertEqual(diag.code, 'FRAME_CONTRACT_MISMATCH')
                self.assertEqual(diag.declared_frame, 'world')
                self.assertEqual(diag.expected_frame, 'container_link')
                self.assertEqual(diag.expected_scene_revision, SCENE)
                result['diagnostic'] = error.diagnostic
            else:
                self.fail('invalid input was accepted')


if __name__ == '__main__':
    unittest.main()
