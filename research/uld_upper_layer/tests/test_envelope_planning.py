from pathlib import Path
import sys
import json
import tempfile
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from assets import load
from envelope_planning import FreeBoxEnvelopeScene
from full_arm import FullArmScene
from planning_types import UpperConfig
from ideal_snapshot import capture_fixture
from scene import fixture


class EnvelopeRoutingContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.assets = load(Path(__file__).resolve().parents[3]); cls.config = UpperConfig.load()
        cls.temp = tempfile.TemporaryDirectory()
        data = fixture(cls.assets)
        data['pickup_payload'] = dict(name='upper_layer_payload', pose_world=[-1, 0, .986, 0, 0, 0])
        (Path(cls.temp.name) / 'fixture.json').write_text(json.dumps(data))
        cls.snapshot, _ = capture_fixture(cls.temp.name, cls.assets, cls.config)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def scene(self, error):
        scene = FreeBoxEnvelopeScene(self.assets, self.snapshot, self.config, 1e-6, error, 0.)
        # Derive a six-millimetre gap from the actual source robot extremum.
        vertices = np.concatenate([bounds for bounds in scene.current_bounds.values()])
        vertex = vertices[np.argmax(vertices[:, 0])].copy()
        pose = np.eye(4); pose[:3, 3] = vertex + [.016, 0, 0]
        scene.set_payload((.02, .02, .02), 'world', pose)
        return scene

    def test_nominal_gap_cannot_bypass_uncertain_box_constraint(self):
        scene = self.scene(.01)
        self.assertIsNone(FullArmScene.check_state(scene, [0.] * 6, .005))
        failure = scene.check_state([0.] * 6, .005)
        self.assertEqual(failure['reason'], 'PAYLOAD_ENVELOPE_MARGIN_UNPROVED')
        self.assertIn('payload', failure['pair'])

    def test_zero_error_retains_nominal_gap_and_interval_rows(self):
        scene = self.scene(0.)
        self.assertIsNone(scene.check_state([0.] * 6, .005))
        self.assertTrue(any('payload' in pair for _, pair in scene.sample_distances))
        self.assertGreaterEqual(min(d for d, p in scene.sample_distances if 'payload' in p), .005)

    def test_attached_box_does_not_receive_fixed_world_motion_proof(self):
        scene = self.scene(.005)
        scene.set_payload((.02, .02, .02), 'attached', np.eye(4), np.eye(4))
        with self.assertRaisesRegex(ValueError, 'FIXED_WORLD_BOX_REQUIRED'):
            scene.check_state([0.] * 6, .005)

    def test_mutated_world_box_cannot_reuse_fixed_obstacle_motion_bound(self):
        scene = self.scene(.005)
        scene.payload['world_pose'][0, 3] += .001
        with self.assertRaisesRegex(ValueError, 'MOVING_WORLD_BOX_INVALIDATES'):
            scene.check_state([0.] * 6, .005)


if __name__ == '__main__':
    unittest.main()
