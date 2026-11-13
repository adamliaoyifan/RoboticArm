"""The physics payload override changes only the placed box, never the OCC."""
from pathlib import Path
import json
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scene import write_scene

REPO = Path(__file__).resolve().parents[3]


class PickupOverrideContracts(unittest.TestCase):
    def test_override_spawns_requested_payload_and_keeps_lower_layers(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / 'scene600'
            write_scene(REPO, out, pickup_override=dict(size_m=(.6, .4, .25), mass_kg=8.0))
            spec = json.loads((out / 'fixture.json').read_text())
        self.assertEqual(spec['pickup_payload']['size_m'], [.6, .4, .25])
        self.assertEqual(spec['pickup_payload']['mass_kg'], 8.0)
        self.assertEqual(spec['payload']['id'], 'carryon')
        self.assertEqual(spec['payload']['size_m'], [.55, .4, .25])
        self.assertEqual(len(spec['boxes']), 24)
        self.assertEqual(spec['pickup_payload_override']['source'],
                         'upper_prior_physics_case_input')
        # Same height as the catalog payload: identical platform rest pose.
        pose = spec['pickup_payload']['pose_world']
        self.assertAlmostEqual(pose[2], 0.986, delta=1e-6)

    def test_override_rejects_invalid_boxes(self):
        with tempfile.TemporaryDirectory() as tmp:
            for bad in (dict(size_m=(.4, .6, .25), mass_kg=8.),
                        dict(size_m=(.6, .4, 0.), mass_kg=8.),
                        dict(size_m=(.6, .4, .25), mass_kg=0.)):
                with self.assertRaises(ValueError):
                    write_scene(REPO, Path(tmp) / 'bad', pickup_override=bad)


if __name__ == '__main__':
    unittest.main()
