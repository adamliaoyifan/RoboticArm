"""Native-step safeguards and tracking interpolation, no transport effects."""
from dataclasses import replace
import unittest
import numpy as np

from execution_policy import ExecutionPolicy
from planning_types import UpperConfig
from release_execution import ReleaseExecutionMonitor, ReleaseMotionPolicy, motion_reference
from release_fixture import ReleaseFixturePolicy
from release_observation import ReleaseObservationReader
from release_trajectory import make_release_withdrawal
import test_release_observation as fixtures


class GeometryStub:
    def evaluate_observation(self, sample, *, separating_interface):
        return dict(minimum_m=.01)


class ReleaseExecutionContracts(unittest.TestCase):
    def setUp(self):
        self.config = UpperConfig.load(); self.policy = ExecutionPolicy.load()
        self.motion_policy = ReleaseMotionPolicy.load()
        sample = ReleaseObservationReader(np.eye(4), {'lower0': (.6, 1., 1.)}, 'payload').read(fixtures.native())
        self.sample = replace(sample, tool_contact_components_complete=True)

    def monitor(self):
        return ReleaseExecutionMonitor(self.sample, (1., 1., .25), self.config,
            self.policy, self.motion_policy, GeometryStub())

    def observe(self, monitor, sample):
        return monitor.observe(sample, detached=False, reference_tool=self.sample.container_T_tool,
                               separating_interface=True)

    def test_every_native_pose_checked_between_geometry_updates(self):
        monitor = self.monitor()
        self.assertIsNone(self.observe(monitor, self.sample))
        body = self.sample.lower[0]; pose = np.array(body.container_T_body).reshape(4, 4)
        pose[0, 3] += self.policy.lower_translation_m*2
        bad = replace(self.sample, stamp_s=self.sample.stamp_s+.001,
                      lower=(replace(body, container_T_body=tuple(pose.ravel())),))
        self.assertEqual(self.observe(monitor, bad)['reason'], 'RELEASE_LOWER_MOVED')
        self.assertEqual(monitor.metrics()['geometry_samples'], 1)

    def test_side_contact_signed_normal_and_excess_depth_rejected(self):
        for contact in (replace(self.sample.contacts[0], normal_toward_payload_container=(1., 0., 0.)),
                        replace(self.sample.contacts[0], depth_m=self.config.contact_penetration_m*2)):
            with self.subTest(contact=contact):
                error = self.observe(self.monitor(), replace(self.sample, contacts=(contact,)))
                self.assertEqual(error['reason'], 'RELEASE_INVALID_SUPPORT_CONTACT')

    def test_tool_contact_completeness_and_actual_contacts(self):
        self.assertEqual(self.observe(self.monitor(), replace(self.sample, tool_contact_components_complete=False))['reason'],
                         'RELEASE_TOOL_CONTACTS_INCOMPLETE')
        self.assertEqual(self.observe(self.monitor(), replace(self.sample, tool_contacts=('contact-sentinel',)))['reason'],
                         'RELEASE_UNEXPECTED_TOOL_CONTACT')

    def test_constraint_still_present_cannot_confirm_detach(self):
        error = self.monitor().observe(self.sample, detached=True,
            reference_tool=self.sample.container_T_tool, separating_interface=True)
        self.assertEqual(error['reason'], 'RELEASE_ATTACHMENT_STATE_INVALID')

    def test_tracking_native_gap_and_persistent_first_failure(self):
        tool = np.array(self.sample.container_T_tool).reshape(4, 4); tool[0, 3] += .004
        monitor = self.monitor()
        error = self.observe(monitor, replace(self.sample, container_T_tool=tuple(tool.ravel())))
        self.assertEqual(error['reason'], 'RELEASE_TRACKING_ERROR')
        self.assertEqual(self.observe(monitor, self.sample), error)
        monitor = self.monitor(); self.assertIsNone(self.observe(monitor, self.sample))
        self.assertEqual(self.observe(monitor, replace(self.sample, stamp_s=self.sample.stamp_s+.002))['reason'], 'RELEASE_NATIVE_GAP')

    def test_missing_support_and_box_displacement_rejected(self):
        self.assertEqual(self.observe(self.monitor(), replace(self.sample, contacts=()))['reason'], 'RELEASE_SUPPORT_MISSING')
        box = np.array(self.sample.container_T_box).reshape(4, 4); box[2, 3] += .003
        self.assertEqual(self.observe(self.monitor(), replace(self.sample, container_T_box=tuple(box.ravel()), contacts=()))['reason'], 'RELEASE_BOX_UNSTABLE')

    def test_reference_tracks_sampled_joint_path_in_container_xz(self):
        motions = make_release_withdrawal(self.sample, self.config, self.policy, ReleaseFixturePolicy.load())
        for m in motions:
            for t, expected in ((-1., m.joint_displacements_xz_m[0]),
                                (m.time_from_start_s[-1]+1, m.joint_displacements_xz_m[-1])):
                q, tool = motion_reference(m, t)
                np.testing.assert_allclose(q, expected)
                start = np.array(m.container_T_tool_start).reshape(4, 4)
                delta = np.array(q)-m.joint_displacements_xz_m[0]
                start[[0, 2], 3] += delta
                np.testing.assert_allclose(np.array(tool).reshape(4, 4), start)
        with self.assertRaisesRegex(ValueError, 'REFERENCE_TIME'):
            motion_reference(motions[0], float('nan'))

    def test_motion_limits_external_and_validated(self):
        for key in self.motion_policy.__dataclass_fields__:
            with self.assertRaisesRegex(ValueError, 'INVALID_RELEASE_MOTION_POLICY'):
                replace(self.motion_policy, **{key: 0.})


if __name__ == '__main__':
    unittest.main()
