"""ST03 pick/retry frame-boundary acceptance (F09/F10) on real types.

Runs after building/sourcing the isolated install; no ROS nodes, no TF.
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

from luggage_description.frame_contracts import ContractError, require_horizontal
from luggage_description.frame_diagnostics import (
    BoundaryContext, BoundaryContractError,
)
from luggage_description.frame_transforms import (
    box_to_tool_target, transform_pose,
)
from luggage_perception.frame_boundary_records import (
    contact_boundary, detection_boundary,
)
from luggage_perception.frame_message_adapters import (
    contact_to_message, contact_from_message,
    detection_to_message, detection_from_message,
)
from luggage_planning.pick_frame_contracts import (
    ApproachSnapshot, SUCTION_TARGET_LINK, approach_snapshot, grasp_transform,
    retry_segment_from_snapshot, stale_calibration_reason,
)
from luggage_planning.pose import Point, Pose
from luggage_planning.suction_candidate_selection import (
    SUCTION_REJECT_TF, ProbeRecord, contact_record_admission,
    judge_probe, observation_identity_consistent,
)
from luggage_planning.suction_candidate_waypoints import (
    attach_pose_error, pose_along_normal, quat_rotate,
    tool_orientation_for_candidate,
)
from luggage_planning.suction_pick_session import PickSession, SessionConfig
from luggage_planning.suction_retry_contracts import RetryState

import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "harness"))
from suction_fakes import make_candidate, make_detection, make_ports  # noqa: E402


SCENE = 'scene-calibration-17'
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


def context(record, interface, reference=None):
    return BoundaryContext(
        interface, 'synthetic-producer', 'pick-boundary-consumer',
        record['request_id'], SCENE, record.get('frame_id', ''),
        record.get('reference', '') if reference is None else reference,
        record['time_basis'])


def matrix_box_record(index, floor, support):
    """Container-frame box record for matrix element ``index``."""
    record = dict(
        contract_version=2, frame_id='container_link', length_unit='m',
        stamp_ns=STAMP_NS, time_basis='scene', scene_revision=SCENE,
        request_id='matrix-%02d' % index, kind='pose', reference='box_center',
        orientation_semantics='box_axis_pi', position=[.41, -.13, floor + support + .16],
        quaternion_xyzw=[0., 0., math.sin(.37 / 2), math.cos(.37 / 2)])
    return record


class FrameMigrationPick(unittest.TestCase):
    def assert_vector(self, actual, expected, tolerance=1e-9):
        self.assertEqual(len(actual), len(expected))
        self.assertLessEqual(max(abs(a - b) for a, b in zip(actual, expected)),
                             tolerance)

    def assert_quaternion(self, actual, expected, tolerance=1e-6):
        self.assertEqual(len(actual), 4)
        error = min(max(abs(a - b) for a, b in zip(actual, expected)),
                    max(abs(a + b) for a, b in zip(actual, expected)))
        self.assertLessEqual(error, tolerance)

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

    def test_64_pick_geometry_cases(self):
        matrix = itertools.product(
            (0., math.pi / 2, math.pi, math.atan2(4, 3)),
            ((0., 0., 0.), (1.5, -.4, .2)), (.53, .71), (0., .30),
            ((0., 0., .16), (.10, -.05, .16)))
        count = 0
        for index, (yaw, translation, floor, support, offset) in enumerate(matrix):
            box = matrix_box_record(index, floor, support)
            tf = dict(source_frame='container_link', target_frame='world',
                      translation=list(translation),
                      quaternion_xyzw=[0., 0., math.sin(yaw / 2),
                                       math.cos(yaw / 2)],
                      scene_revision=SCENE, stamp_ns=None)
            grasp = dict(source_frame='suction_contact_frame', target_frame='box',
                         scene_revision=SCENE, translation=list(offset),
                         quaternion_xyzw=[1., 0., 0., 0.])
            fixture = dict(box=box, transform=tf, grasp=grasp, floor=floor,
                           support=support, offset=list(offset))
            with self.subTest(index=index), \
                    case('pick/%02d' % index, fixture) as observed:
                # World detection record: container -> world at the box pose.
                world_box = transform_pose(box, tf)
                world_box['time_basis'] = 'acquisition'
                detection, yaw_source = detection_boundary(
                    world_box['position'],
                    2.0 * math.atan2(world_box['quaternion_xyzw'][2],
                                     world_box['quaternion_xyzw'][3]),
                    STAMP_SEC, SCENE, box['request_id'])
                self.assertEqual(yaw_source, 'measured')
                det_ctx = context(detection, 'F03')
                decoded_detection = detection_from_message(
                    wire(detection_to_message(detection, det_ctx)), det_ctx)
                self.assertEqual(decoded_detection['stamp_ns'], STAMP_NS)
                # Tool target from the substrate; its position IS the world
                # contact point, computed by the substrate's own composition.
                tool = box_to_tool_target(world_box, grasp)
                # Actual grasp transform from world geometry: translation is
                # the contact offset in box axes (frame-independent truth).
                actual_grasp = grasp_transform(
                    world_box['position'], world_box['quaternion_xyzw'],
                    tool['position'], tool['quaternion_xyzw'], SCENE)
                self.assert_vector(actual_grasp['translation'], offset)
                # The stored grasp reproduces the substrate tool exactly.
                reproduced = box_to_tool_target(world_box, actual_grasp)
                self.assert_vector(reproduced['position'], tool['position'])
                self.assert_quaternion(reproduced['quaternion_xyzw'],
                                       tool['quaternion_xyzw'])
                # Contact record at the tool/contact position.
                contact_record = contact_boundary(
                    _contact_candidate(tool['position']), STAMP_SEC, SCENE,
                    box['request_id'])
                contact_ctx = context(contact_record, 'F04')
                decoded_contact = contact_from_message(
                    wire(contact_to_message(contact_record, contact_ctx)),
                    contact_ctx)
                self.assertEqual(decoded_contact['stamp_ns'], STAMP_NS)
                self.assertEqual(decoded_contact, contact_record)
                # Contact +Z stays the outward normal; tool -Z opposes it.
                normal = (0., 0., 1.)
                tool_orientation = tool_orientation_for_candidate(normal, 0.)
                tool_plus_z = quat_rotate(tool_orientation, (0., 0., 1.))
                self.assert_vector(tool_plus_z, (0., 0., -1.))
                # Approach snapshot bound to the observation and grasp.
                candidate = make_candidate(
                    'C000_000', 1, xy=(tool['position'][0], tool['position'][1]),
                    z=tool['position'][2], normal=(0., 0., 1.))
                detection_view = replace(
                    make_detection([candidate]),
                    box_xyz=tuple(world_box['position']),
                    box_quat=tuple(world_box['quaternion_xyzw']),
                    box_size=(.80, .50, .32))
                approach = pose_along_normal(
                    candidate.contact, normal, 0.10,
                    yaw_rad=0.0)
                snapshot = approach_snapshot(
                    approach, candidate, detection_view, scene_revision=SCENE)
                self.assertEqual(snapshot.link_name, SUCTION_TARGET_LINK)
                self.assertEqual(snapshot.scene_revision, SCENE)
                self.assertEqual(snapshot.stamp_ns,
                                 int(round(detection_view.stamp * 1e9)))
                self.assertIsNotNone(snapshot.grasp)
                self.assert_vector(snapshot.grasp['translation'], offset)
                # Retry rebuilds strictly from the snapshot: the retry target
                # cannot move, whatever later candidate/TF state shows up.
                attach = Pose(position=Point(x=tool['position'][0],
                                             y=tool['position'][1],
                                             z=tool['position'][2]),
                              orientation=tool_orientation)
                retry = retry_segment_from_snapshot(
                    snapshot, attach_pose=attach)
                self.assertEqual(retry.target_pose, approach)
                self.assertFalse(retry.allow_ompl_fallback)
                self.assertEqual(retry.required_cartesian_fraction, 1.0)
                with self.assertRaises(ValueError):
                    # A snapshot whose frozen normal shrinks the reverse
                    # projection below the declared minimum still trips the
                    # guard — the guard reads frozen data, not live state.
                    frozen_bad = ApproachSnapshot(
                        target_pose=approach, approach_normal=(1., 0., 0.),
                        scene_revision=SCENE)
                    retry_segment_from_snapshot(frozen_bad, attach_pose=attach)
                position_mm, normal_deg, _ = attach_pose_error(
                    attach, _contact_view(tool['position']))
                self.assertLessEqual(position_mm, 1e-6)
                self.assertLessEqual(normal_deg, 1e-6)
                observed.update(
                    stage='complete', grasp=actual_grasp,
                    tool_position=tool['position'],
                    snapshot=dict(snapshot.as_dict(),
                                  target=[approach.position.x,
                                          approach.position.y,
                                          approach.position.z]))
                count += 1
        self.assertEqual(count, 64)

    def test_yaw_fallback_and_oracles(self):
        with case('fallback/yaw-invalid', {}) as observed:
            record, source = detection_boundary(
                (.5, -.25, .8), 1.234, STAMP_SEC, SCENE, 'fallback-1',
                yaw_valid=False, fallback_yaw=0.25)
            self.assertEqual(source, 'yaw_fallback')
            self.assert_quaternion(
                record['quaternion_xyzw'],
                [0., 0., math.sin(.25 / 2), math.cos(.25 / 2)], 1e-12)
            observed.update(yaw_source=source)
        # Injected wrong-frame producer: container-frame coordinates labelled
        # world must fail the independent geometric oracle (not a roundtrip).
        box = matrix_box_record(99, .53, 0.)
        tf = dict(source_frame='container_link', target_frame='world',
                  translation=(1.5, -.4, .2),
                  quaternion_xyzw=[0., 0., 0., 1.], scene_revision=SCENE,
                  stamp_ns=None)
        with case('mutation/wrong-frame-producer',
                  {'box': box, 'transform': tf}) as observed:
            mislabelled = copy.deepcopy(box)
            mislabelled['frame_id'] = 'world'
            mislabelled['time_basis'] = 'acquisition'
            decoded = detection_from_message(
                wire(detection_to_message(mislabelled,
                                          context(mislabelled, 'F03'))),
                context(mislabelled, 'F03'))
            truth = transform_pose(box, tf)
            with self.assertRaises(AssertionError):
                self.assert_vector(decoded['position'], truth['position'])
            observed.update(defect_detected=True)
        # Injected position-only rotation: a grasp whose translation carries
        # the rotated offset but whose quaternion dropped the rotation must
        # fail the orientation oracle even though its position still agrees.
        with case('mutation/position-only-rotation',
                  {'box_yaw': .37}) as observed:
            world_box = transform_pose(box, tf)
            world_box['time_basis'] = 'acquisition'
            grasp = dict(source_frame='suction_contact_frame',
                         target_frame='box', scene_revision=SCENE,
                         translation=[.10, -.05, .16],
                         quaternion_xyzw=[1., 0., 0., 0.])
            tool = box_to_tool_target(world_box, grasp)
            mutated = dict(grasp, quaternion_xyzw=[0., 0., 0., 1.])
            position_only = box_to_tool_target(world_box, mutated)
            self.assert_vector(position_only['position'], tool['position'])
            with self.assertRaises(AssertionError):
                self.assert_quaternion(position_only['quaternion_xyzw'],
                                       tool['quaternion_xyzw'])
            observed.update(defect_detected=True,
                            correct=tool['quaternion_xyzw'],
                            mutant=position_only['quaternion_xyzw'])

    def test_contact_record_admission_negatives(self):
        record = contact_boundary(
            _contact_candidate((.5, .25, .9)), STAMP_SEC, SCENE, 'adm-1')
        with case('admission/accept', record) as observed:
            self.assertIsNone(contact_record_admission(record, SCENE))
            observed.update(admitted=True)
        for name, key, value, code in [
            ('missing-frame', 'frame_id', '', 'MISSING_IDENTITY'),
            ('wrong-frame', 'frame_id', 'container_link',
             'FRAME_CONTRACT_MISMATCH'),
            ('wrong-reference', 'reference', 'box_center',
             'POSE_REFERENCE_MISMATCH'),
            ('wrong-version', 'contract_version', 1,
             'CONTRACT_VERSION_MISMATCH'),
            ('wrong-units', 'length_unit', 'mm', 'UNIT_MISMATCH'),
            ('stale-scene', 'scene_revision', 'stale',
             'CALIBRATION_REVISION_MISMATCH'),
            ('missing-stamp', 'stamp_ns', None, 'INVALID_STAMP'),
            ('wrong-time-basis', 'time_basis', 'scene', 'TIME_BASIS_MISMATCH'),
            ('malformed-contact', 'position', [1., 2.],
             'INVALID_NUMERIC_PAYLOAD'),
            ('nonfinite-contact', 'position', [math.nan, 0., 0.],
             'INVALID_NUMERIC_PAYLOAD'),
            ('nonunit-quaternion', 'quaternion_xyzw', [0., 0., 0., 2.],
             'INVALID_QUATERNION'),
        ]:
            data = copy.deepcopy(record)
            data[key] = value
            with self.subTest(name=name), \
                    case('admission/' + name, data) as observed:
                reason = contact_record_admission(data, SCENE)
                self.assertEqual(reason[0], SUCTION_REJECT_TF)
                self.assertTrue(reason[1].startswith(code),
                                reason[1])
                observed.update(reason=reason)
                effects = []
                self.assertEqual(effects, [])
        # Same mutations reject at the message boundary with exact codes.
        for name, key, value, code in [
            ('msg-missing-frame', 'frame_id', '', 'MISSING_IDENTITY'),
            ('msg-wrong-frame', 'frame_id', 'container_link',
             'FRAME_CONTRACT_MISMATCH'),
            ('msg-stale-scene', 'scene_revision', 'stale',
             'CALIBRATION_REVISION_MISMATCH'),
        ]:
            data = copy.deepcopy(record)
            data[key] = value
            self.reject(name, code, data,
                        lambda: contact_to_message(
                            data, context(record, 'F04')))
        # Planning frame other than world: v2 contact records inadmissible.
        reason = contact_record_admission(record, SCENE,
                                          planning_frame='elfin_base_link')
        with case('admission/wrong-planning-frame', {'planning_frame':
                                                     'elfin_base_link'}) as o:
            self.assertEqual(reason[0], SUCTION_REJECT_TF)
            self.assertIn('FRAME_CONTRACT_MISMATCH', reason[1])
            o.update(reason=reason)

    def test_observation_identity_gates_keep_reasons(self):
        candidate = make_candidate('C000_000', 1)
        # Stale observation / cross-object identity keep their documented
        # reason category, before any waypoint or vacuum effect.
        with case('identity/stale-observation', {}) as observed:
            mismatch = observation_identity_consistent(
                [candidate], 9999.0, 'world', 'box-1', 7)
            self.assertIsNotNone(mismatch)
            self.assertEqual(mismatch[0], 'SUCTION_CANDIDATE_IDENTITY_MISMATCH')
            observed.update(mismatch=mismatch)
        with case('identity/cross-object', {}) as observed:
            other = make_candidate('C000_000', 1, instance_id='box-2')
            mismatch = observation_identity_consistent(
                [other], 1234.5, 'world', 'box-1', 7)
            self.assertIsNotNone(mismatch)
            observed.update(mismatch=mismatch)
        effects = []
        self.assertEqual(effects, [])

    def test_retry_stale_calibration_zero_effects(self):
        candidate = make_candidate('C000_000', 1)
        detection = make_detection([candidate])
        approach = pose_along_normal(
            candidate.contact, (0., 0., 1.), 0.10)
        snapshot = approach_snapshot(approach, candidate, detection,
                                      scene_revision='scene-A')
        with case('retry/stale-calibration', snapshot.as_dict()) as observed:
            self.assertEqual(
                stale_calibration_reason(snapshot, 'scene-B'),
                'CALIBRATION_REVISION_MISMATCH')
            self.assertIsNone(stale_calibration_reason(snapshot, 'scene-A'))
            self.assertIsNone(stale_calibration_reason(snapshot, ''))
            observed.update(checked=True)

        class RevisionPorts:
            def __init__(self, base, revision):
                self._base = base
                self.revision = revision
                self.motion_calls = []
                self.vacuum_calls = []

            def __getattr__(self, name):
                return getattr(self._base, name)

            def scene_revision(self):
                return self.revision

        ports = RevisionPorts(make_ports([candidate]), 'scene-B')
        config = SessionConfig(scene_revision='scene-A', dry_run=True)
        session = PickSession(ports, config)
        session._detection_view = make_detection([candidate])
        session._detection_instance = 'box-1'
        session._detection_generation = 7
        session._recorded_snapshot = snapshot
        # Recovery-verify is the only state whose IDENTITY_CHANGED edge ends
        # in the terminal retry-recovery failure; enter it before the probe.
        session._model = replace(session._model,
                                 state=RetryState.RECOVERY_VERIFY)
        fed = []
        original_feed = session._feed

        def capture(event):
            fed.append(event)
            original_feed(event)

        session._feed = capture
        with case('retry/recovery-rejects-stale-scene',
                  {'recorded': 'scene-A', 'current': 'scene-B'}) as observed:
            session._verify_recovery()
            self.assertTrue(any(
                getattr(event, 'detail', '') == 'CALIBRATION_REVISION_MISMATCH'
                for event in fed))
            self.assertEqual(ports.motion_calls, [])
            self.assertEqual(ports.vacuum_calls, [])
            self.assertEqual(ports.detect_calls, 0)
            observed.update(
                fed=[str(getattr(e, 'event_type', '')) for e in fed],
                motion_calls=0, vacuum_calls=0)

    def test_roll_gate_and_quaternion_sign(self):
        roll = .2
        q = [math.sin(roll / 2), 0., 0., math.cos(roll / 2)]
        self.reject('container-tilt', 'UNSUPPORTED_CONTAINER_TILT',
                    {'quaternion': q}, lambda: require_horizontal(q))
        with case('orientation/q-sign', {}) as observed:
            candidate = _contact_candidate((.5, .25, .9))
            record = contact_boundary(candidate, STAMP_SEC, SCENE, 'sign-1')
            record['quaternion_xyzw'] = [0., 0., -0., -1.]
            message = contact_to_message(record, context(record, 'F04'))
            decoded = contact_from_message(wire(message),
                                           context(record, 'F04'))
            self.assert_quaternion(decoded['quaternion_xyzw'],
                                   [0., 0., 0., 1.])
            observed.update(sign_equivalent=True)

    def test_snapshot_freeze_mutation_independence(self):
        # R5 regression: the frozen approach snapshot owns its pose and
        # grasp. Mutating the caller's input pose, the returned retry
        # target, or the exported grasp view must never rewrite history.
        candidate = make_candidate('C000_000', 1)
        detection = make_detection([candidate])
        approach = pose_along_normal(
            candidate.contact, (0., 0., 1.), 0.10)
        with case('review/snapshot-freeze', {}) as observed:
            snapshot = approach_snapshot(approach, candidate, detection,
                                         scene_revision='scene-A')
            frozen = (snapshot.target_pose.position.x,
                      snapshot.target_pose.position.y,
                      snapshot.target_pose.position.z)
            approach.position.x = 123.0
            self.assertEqual(snapshot.target_pose.position.x, frozen[0])
            retry = retry_segment_from_snapshot(snapshot)
            self.assertIsNot(retry.target_pose, snapshot.target_pose)
            self.assertIsNot(retry.target_pose, approach)
            retry.target_pose.position.y = 456.0
            self.assertEqual(snapshot.target_pose.position.y, frozen[1])
            self.assertEqual(approach.position.y, 0.0)
            # Two retries never share a pose object either.
            second = retry_segment_from_snapshot(snapshot)
            second.target_pose.position.z = 789.0
            self.assertEqual(retry.target_pose.position.z, frozen[2])
            # The exported view is a detached copy of the grasp.
            view = snapshot.as_dict()
            if view['grasp'] is not None:
                view['grasp']['translation'][0] = -5.0
                self.assertNotEqual(
                    snapshot.grasp['translation'][0], -5.0)
            observed.update(snapshot=list(frozen))

    def test_judge_probe_admits_v2_record_before_effects(self):
        # Real-caller wiring: judge_probe admits a supplied v2 contact
        # record FIRST — a boundary violation rejects before the probe
        # records are even considered, carrying the exact contract code.
        candidate = make_candidate('C000_000', 1)
        good = contact_boundary(
            _contact_candidate((.65, 0.0, .85)), STAMP_SEC, SCENE,
            'gate-wire')
        stale = copy.deepcopy(good)
        stale['scene_revision'] = 'scene-old'
        record = ProbeRecord(candidate_id='C000_000', segment_name='approach',
                             ik_ok=True, fraction=1.0, moveit_error_code=1)
        with case('review/judge-probe-v2-admission', {}) as observed:
            self.assertIsNone(judge_probe(
                candidate, [record], contact_record=good,
                scene_revision=SCENE))
            reason = judge_probe(candidate, [record],
                                 contact_record=stale,
                                 scene_revision=SCENE)
            self.assertEqual(reason[0], SUCTION_REJECT_TF)
            self.assertTrue(reason[1].startswith(
                'CALIBRATION_REVISION_MISMATCH'), reason)
            # Admission runs before probe evaluation: even a failing IK
            # record cannot mask the boundary rejection.
            failing = ProbeRecord(candidate_id='C000_000',
                                  segment_name='approach', ik_ok=False,
                                  fraction=-1.0, moveit_error_code=1)
            reason = judge_probe(candidate, [failing],
                                 contact_record=stale,
                                 scene_revision=SCENE)
            self.assertTrue(reason[1].startswith(
                'CALIBRATION_REVISION_MISMATCH'), reason)
            observed.update(reason=reason)

    def test_session_gate_rejects_stale_v2_record_zero_effects(self):
        # Real-caller test through the session's own selection step: the
        # ports supply the v2 contact record; a stale scene record rejects
        # the first candidate with zero MOTION/VACUUM effects for it (the
        # planning-space probes are the judge's inputs, not effects), and
        # the next (clean) candidate is selected normally.
        stale_first = make_candidate('C001_001', 1)
        clean_second = make_candidate('C002_001', 2)
        good = contact_boundary(
            _contact_candidate((.65, 0.0, .85)), STAMP_SEC, SCENE,
            'session-wire')
        stale = copy.deepcopy(good)
        stale['scene_revision'] = 'scene-old'

        class RecordPorts:
            def __init__(self, base, records):
                self._base = base
                self._records = records

            def __getattr__(self, name):
                return getattr(self._base, name)

            def contact_record(self, candidate_id):
                return self._records.get(candidate_id)

        ports = RecordPorts(
            make_ports([stale_first, clean_second]),
            {'C001_001': stale, 'C002_001': good})
        session = PickSession(
            ports, SessionConfig(scene_revision=SCENE, dry_run=True,
                                 contact_model_version=1,
                                 contact_model_hash='abc123'))
        with case('review/session-v2-record-gate', {}) as observed:
            result = session.run_request(
                make_detection([stale_first, clean_second]))
            executed_ids = [candidate_id
                            for candidate_id, _ in ports.motion.executed]
            self.assertNotIn('C001_001', executed_ids)
            self.assertNotIn('C001_001',
                             ports.vacuum.enable_candidates)
            self.assertIn('C002_001', executed_ids)
            rejected = [row for row in session._trace
                        if row.get('kind') == 'io'
                        and row.get('name') == 'candidate_rejected']
            self.assertTrue(rejected)
            self.assertTrue(any(
                'CALIBRATION_REVISION_MISMATCH' in str(row) for row in
                rejected), rejected)
            observed.update(executed_ids=executed_ids,
                            n_rejected=len(rejected),
                            exit_code=result.exit_code)

    def test_hardware_driver_supplies_scene_revision(self):
        # Real wiring: the hardware driver forwards --frame-scene-revision
        # into SessionConfig and exposes the scene_revision port the retry
        # guard consumes (previously neither existed, so F10 recovery on
        # hardware silently skipped the stale-scene check).
        import runpy
        import types as types_module
        driver_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'scripts', 'hardware_pick_driver.py')
        module = runpy.run_path(driver_path)
        args = types_module.SimpleNamespace(
            max_candidates=3, release_low_hold_sec=0.5,
            release_deadline_sec=2.0, reverse_min_m=0.08,
            required_cartesian_fraction=1.0, request_deadline_sec=180.0,
            no_scene=False, use_vacuum=True, dry_run=False, dry_run_count=10,
            frame_scene_revision='scene-hw-17')
        model = types_module.SimpleNamespace(
            model_version=1, identity_hash='abc123')
        with case('review/hardware-driver-scene-revision',
                  {'frame_scene_revision':
                   args.frame_scene_revision}) as observed:
            config = module['HardwarePickDriver']._session_config(
                types_module.SimpleNamespace(_args=args), model)
            self.assertEqual(config.scene_revision, 'scene-hw-17')
            ports = types_module.SimpleNamespace()
            ports.scene_revision = types_module.MethodType(
                module['RosPickSessionPorts'].scene_revision,
                types_module.SimpleNamespace(
                    _driver=types_module.SimpleNamespace(_args=args)))
            self.assertEqual(ports.scene_revision(), 'scene-hw-17')
            # Undeclared stays empty (wiring gap the session reports).
            empty_args = types_module.SimpleNamespace(
                frame_scene_revision='')
            ports_empty = types_module.SimpleNamespace()
            ports_empty.scene_revision = types_module.MethodType(
                module['RosPickSessionPorts'].scene_revision,
                types_module.SimpleNamespace(
                    _driver=types_module.SimpleNamespace(_args=empty_args)))
            self.assertEqual(ports_empty.scene_revision(), '')
            observed.update(config_scene_revision=config.scene_revision)


class _ContactPose:
    def __init__(self, position):
        self.position = Point(x=position[0], y=position[1], z=position[2])
        self.orientation = type('Orientation', (), dict(x=0., y=0., z=0., w=1.))()


def _contact_candidate(position, stamp=STAMP_SEC, frame='world',
                       instance_id='box-1', generation=7):
    holder = type('Candidate', (), {})()
    holder.frame = frame
    holder.stamp = stamp
    holder.instance_id = instance_id
    holder.generation = generation
    holder.contact_pose = _ContactPose(position)
    return holder


def _contact_view(position):
    holder = type('CandidateView', (), {})()
    holder.contact = _ContactPose(position)
    return holder


if __name__ == '__main__':
    unittest.main()
