"""Carriage input frames, strict contacts and settled initialization contracts."""
from copy import deepcopy
from dataclasses import replace
from pathlib import Path
import unittest
import numpy as np
from scipy.spatial.transform import Rotation

from execution_policy import ExecutionPolicy
from geometry import pose
from planning_types import UpperConfig
from release_fixture import ReleaseFixturePolicy
from release_observation import ReleaseObservationReader
from release_readiness import inspect_release_start, ReleaseReadyWindow, CalibrationReadyWindow
from release_trajectory import make_release_withdrawal


def state(position):
    return dict(position=dict(zip('xyz', position)), orientation=dict(x=0., y=0., z=0., w=1.))


def native(stamp=1.):
    return dict(ready=True, attached=True, joint_positions_complete=True,
        contacts_component_available=True, attachment_joint_component_present=True,
        payload_pose_command_component_present=False, joint_names=['release_x', 'release_z'],
        q=[.01, -.02], sim_time_s=stamp, pose_command_count=0, attachment_create_count=1,
        attachment_detach_count=0, processed_commands=1, payload_pose_world=state([0., 0., 1.125]),
        link_pose_world=state([0., 0., 1.]), link_T_payload_xyz=[0., 0., .125],
        link_T_payload_quat_xyzw=[0., 0., 0., 1.],
        observed_model_poses_world=dict(lower0=dict(valid=True, pose=state([.2, 0., .5]))),
        payload_contacts=dict(contact=[dict(collision1=dict(name='payload::body::collision'),
            collision2=dict(name='lower0::body::collision'), position=[dict(x=0., y=.2, z=1.)],
            normal=[dict(z=1.)], depth=[0.])]))


class ReleaseInputContracts(unittest.TestCase):
    def setUp(self):
        self.reader = ReleaseObservationReader(np.eye(4), {'lower0': (.6, 1., 1.)}, 'payload')
        self.config = UpperConfig.load(); self.policy = ExecutionPolicy.load()

    def test_native_same_step_maps_to_immutable_metric_carriage_input(self):
        row = native(); sample = self.reader.read(row)
        row['q'][0] = 99.; row['observed_model_poses_world']['lower0']['pose']['position']['x'] = 99.
        self.assertEqual(sample.joint_displacements_xz_m, (.01, -.02))
        self.assertAlmostEqual(np.array(sample.lower[0].container_T_body).reshape(4, 4)[0, 3], .2)
        self.assertEqual(sample.lower[0].stamp_s, sample.stamp_s)
        result = inspect_release_start(sample, (1., 1., .25), (0., 0., 0.), self.config, self.policy)
        self.assertTrue(result['accepted'], result)
        self.assertFalse(result['execution_admitted'])
        self.assertAlmostEqual(result['support']['near_contact_area_ratio'], .6)
        self.assertIn('unavailable', result['support']['pressure_area_status'])

    def test_nonidentity_world_container_frame_applies_to_all_geometry(self):
        frame = pose([2., -1., .53], Rotation.from_euler('z', 90, degrees=True).as_matrix())
        reader = ReleaseObservationReader(frame, {'lower0': (.6, 1., 1.)}, 'payload')
        row = native()
        pose_rows = [row['payload_pose_world'], row['link_pose_world'], row['observed_model_poses_world']['lower0']['pose']]
        for value in pose_rows:
            xyz = frame @ np.r_[[value['position'][k] for k in 'xyz'], 1.]
            value['position'] = dict(zip('xyz', xyz[:3]))
            value['orientation'] = dict(zip(('x', 'y', 'z', 'w'), Rotation.from_matrix(frame[:3, :3]).as_quat()))
        contact = row['payload_contacts']['contact'][0]
        point = frame @ np.array([0., .2, 1., 1.])
        contact['position'] = [dict(zip('xyz', point[:3]))]
        transformed = reader.read(row); expected = self.reader.read(native())
        np.testing.assert_allclose(transformed.container_T_box, expected.container_T_box, atol=1e-12)
        np.testing.assert_allclose(transformed.container_T_tool, expected.container_T_tool, atol=1e-12)
        np.testing.assert_allclose(transformed.contacts[0].point_container, expected.contacts[0].point_container, atol=1e-12)

    def test_incomplete_invalid_or_commanded_native_inputs_reject(self):
        changes = [lambda r: r.update(joint_names=['elfin_joint1', 'elfin_joint2']),
            lambda r: r.update(q=[0.] * 6), lambda r: r.update(q=[float('nan'), 0.]),
            lambda r: r.update(ready='true'), lambda r: r.update(pose_command_count=1),
            lambda r: r.update(payload_pose_command_component_present=True),
            lambda r: r.update(contacts_component_available=False),
            lambda r: r['observed_model_poses_world'].clear(),
            lambda r: r['link_pose_world']['orientation'].update(w=2.),
            lambda r: r['payload_contacts']['contact'][0].update(normal=[])]
        for change in changes:
            row = native(); change(row)
            with self.subTest(row=row):
                with self.assertRaises(ValueError):
                    self.reader.read(row)

    def test_readiness_retains_strict_normal_and_runtime_support_limits(self):
        row = native(); row['payload_contacts']['contact'][0]['normal'] = [dict(x=1.)]
        sample = self.reader.read(row)
        rejected = inspect_release_start(sample, (1., 1., .25), (0., 0., 0.), self.config, self.policy)
        self.assertIn('RELEASE_INVALID_NATIVE_SUPPORT', [r['reason'] for r in rejected['failures']])
        sample = self.reader.read(native())
        result = inspect_release_start(sample, (1., 1., .25), (0., 0., 0.),
            replace(self.config, support_com_margin_m=.11), self.policy)
        self.assertFalse(result['accepted'])
        self.assertIn('RELEASE_COM_MARGIN_BELOW_POLICY', [r['reason'] for r in result['failures']])

    def test_dwell_requires_uninterrupted_settled_supported_samples(self):
        policy = replace(self.policy, stability_dwell_s=.04)
        window = ReleaseReadyWindow((1., 1., .25), (0., 0., 0.), self.config, policy)
        for stamp in (1., 1.01, 1.02, 1.03):
            self.assertFalse(window.observe(self.reader.read(native(stamp)))['ready'])
        self.assertTrue(window.observe(self.reader.read(native(1.04)))['ready'])
        row = native(1.05); row['payload_contacts'] = {}
        self.assertFalse(window.observe(self.reader.read(row))['ready'])
        self.assertIsNone(window.baseline)
        result = window.observe(self.reader.read(native(1.10)))
        self.assertEqual(result['failures'][0]['reason'], 'RELEASE_READY_OBSERVATION_GAP')
        self.assertFalse(result['ready'])
        with self.assertRaisesRegex(ValueError, 'STAMP_NOT_INCREASING'):
            window.observe(self.reader.read(native(1.10)))

    def test_lower_movement_cannot_be_hidden_by_box_stability(self):
        baseline = self.reader.read(native())
        row = native(1.01); row['observed_model_poses_world']['lower0']['pose']['position']['y'] += .004
        result = inspect_release_start(self.reader.read(row), (1., 1., .25), (0., 0., 0.), self.config, self.policy, baseline)
        self.assertIn('RELEASE_INITIAL_LOWER_NOT_SETTLED', [r['reason'] for r in result['failures']])

    def test_calibration_keeps_failed_policy_visible_without_execution_readiness(self):
        policy = replace(self.policy, stability_dwell_s=.02)
        config = replace(self.config, support_area_ratio_min=.7)
        window = CalibrationReadyWindow((1., 1., .25), (0., 0., 0.), config, policy)
        window.observe(self.reader.read(native(1.)))
        window.observe(self.reader.read(native(1.01)))
        result = window.observe(self.reader.read(native(1.02)))
        self.assertTrue(result['calibration_ready'])
        self.assertFalse(result['accepted'])
        self.assertFalse(result['ready'])
        self.assertFalse(result['execution_admitted'])
        self.assertFalse(result['execution_policy_satisfied'])
        row = native(1.03); row['payload_contacts']['contact'][0]['normal'] = [dict(x=1.)]
        result = window.observe(self.reader.read(row))
        self.assertFalse(result['calibration_ready'])
        self.assertIsNone(window.baseline)

    def test_withdrawal_uses_measured_start_and_runtime_geometry_and_timing(self):
        sample = self.reader.read(native()); fixture = ReleaseFixturePolicy.load()
        down, outward = make_release_withdrawal(sample, self.config, self.policy, fixture)
        np.testing.assert_allclose(down.joint_displacements_xz_m[0], sample.joint_displacements_xz_m)
        np.testing.assert_allclose(down.joint_displacements_xz_m[-1], [.01, -.02 - self.config.release_drop_m])
        np.testing.assert_allclose(outward.joint_displacements_xz_m[-1], [.01 - self.config.withdraw_m, -.02 - self.config.release_drop_m])
        self.assertEqual(down.joint_velocities_xz_mps[0], (0., 0.))
        np.testing.assert_allclose(outward.joint_velocities_xz_mps[-1], [0., 0.], atol=1e-12)
        self.assertLessEqual(down.velocity_bound_mps, self.config.contact_speed_mps + 1e-12)
        self.assertLessEqual(down.acceleration_bound_mps2, self.config.push_acceleration_mps2 + 1e-12)
        slower, _ = make_release_withdrawal(sample, self.config, self.policy,
            replace(fixture, velocity_limit_mps=self.config.contact_speed_mps / 2))
        self.assertGreater(slower.time_from_start_s[-1], down.time_from_start_s[-1])
        changed, _ = make_release_withdrawal(sample, replace(self.config, release_drop_m=.07), self.policy, fixture)
        self.assertAlmostEqual(changed.joint_displacements_xz_m[-1][1], -.09)
        self.assertFalse(outward.execution_admitted)
        with self.assertRaisesRegex(ValueError, 'TRAVEL_LIMIT'):
            make_release_withdrawal(sample, self.config, self.policy, replace(fixture, travel_limit_m=.03))


if __name__ == '__main__':
    unittest.main()
