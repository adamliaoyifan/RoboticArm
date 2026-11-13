from pathlib import Path
import sys
import time
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from payload_tube import bound_envelope_path, PayloadEnvelopeSampler, certify_payload_noncontact_tube


class EnvelopeIntervalContracts(unittest.TestCase):
    def test_undeclared_obstacle_cannot_disappear_into_contact_exclusions(self):
        from types import SimpleNamespace
        import numpy as np
        from full_arm import FullArmScene
        scene = SimpleNamespace(payload=dict(size=(.5, .4, .3), allow_support=True),
            payload_transform=lambda: np.eye(4), robot_parts={}, world_parts={'mystery': None},
            contact_mode=FullArmScene.contact_mode, push_geometry=SimpleNamespace(collision_key='panel'))
        sampler = PayloadEnvelopeSampler(scene, 1e-6, .005, .01)
        with self.assertRaisesRegex(ValueError, 'UNDECLARED_NONCONTACT_PAIR_MISSING'):
            sampler.enclose_current_distances([])
        scene.world_parts = {}; scene.robot_parts = {'panel': None}
        with self.assertRaisesRegex(ValueError, 'UNDECLARED_NONCONTACT_PAIR_MISSING'):
            sampler.enclose_current_distances([], False)
        self.assertEqual(sampler.enclose_current_distances([], 'push_plate'), ())
        self.assertIn(('payload', 'panel'), sampler.excluded_pairs)

    def test_stale_planning_input_is_rejected_before_geometry_queries(self):
        from types import SimpleNamespace
        from planning_types import UpperConfig
        config = UpperConfig.load()
        scene = SimpleNamespace(config=config, snapshot=SimpleNamespace(snapshot_id='current'))
        plan = dict(stages=[dict(name='push')], action=dict(placement=dict(
            snapshot_id='stale', config_revision=config.revision)))
        with self.assertRaisesRegex(ValueError, 'INPUT_IDENTITY_MISMATCH'):
            certify_payload_noncontact_tube(scene, None, plan, config, None, None,
                                           time.monotonic() + 3, stages=('push',))

    def test_invalid_error_bounds_cannot_skip_validation_on_distant_geometry(self):
        for translation, angle in ((-.01, .01), (.01, -.01), (.01, 4.), (.01, float('nan'))):
            with self.assertRaises(ValueError):
                PayloadEnvelopeSampler(None, 1e-6, translation, angle)
        with self.assertRaises(ValueError):
            PayloadEnvelopeSampler(None, 1e-6, .005, .01, transverse_to=[2, 0, 0])

    def test_moving_obstacle_requires_continuous_subdivision(self):
        def evaluate(i, u, q):
            return ((.006 + (q[0] - .23) ** 2, ('payload', 'obstacle')),), None
        result = bound_envelope_path([[-.5], [.5]], evaluate,
            lambda pair, delta, *args: 2 * delta[0], .005, time.monotonic() + 3)
        self.assertTrue(result['noncontact_clearance_certified'])
        self.assertGreater(result['checks'], 1)
        self.assertGreaterEqual(result['lower_m'], .005)

    def test_hidden_violation_is_not_certified_from_midpoint_alone(self):
        result = bound_envelope_path([[0.], [1.]],
            lambda i, u, q: (((.012 - .012 * u, ('payload', 'obstacle')),), None),
            lambda pair, delta, i, low, high: .012 * (high - low) / 2,
            .005, time.monotonic() + 3)
        self.assertFalse(result['noncontact_clearance_certified'])
        self.assertEqual(result['reason'], 'ENVELOPE_SAMPLE_UNPROVED')

    def test_deadline_missing_evidence_and_invalid_bound_fail_closed(self):
        result = bound_envelope_path([[0.], [1.]], lambda *a: self.fail('expired'),
            lambda *a: 0., .005, time.monotonic() - 1)
        self.assertEqual(result['reason'], 'ENVELOPE_DEADLINE')
        result = bound_envelope_path([[0.], [1.]], lambda *a: ((), None),
            lambda *a: 0., .005, time.monotonic() + 3)
        self.assertEqual(result['reason'], 'ENVELOPE_MISSING_DISTANCE_EVIDENCE')
        result = bound_envelope_path([[0.], [1.]],
            lambda *a: (((.1, ('payload', 'obstacle')),), None),
            lambda *a: -1., .005, time.monotonic() + 3)
        self.assertEqual(result['reason'], 'INVALID_ENVELOPE_DISTANCE_BOUND')


if __name__ == '__main__':
    unittest.main()
