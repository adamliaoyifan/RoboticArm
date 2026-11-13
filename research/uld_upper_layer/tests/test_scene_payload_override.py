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

    def test_real_time_factor_paces_world_and_records_clock(self):
        import xml.etree.ElementTree as ET
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / 'paced'
            write_scene(REPO, out, real_time_factor=.4)
            spec = json.loads((out / 'fixture.json').read_text())
            factor = ET.parse(out / 'overlay.sdf').getroot().find('world/physics/real_time_factor')
        self.assertEqual(spec['validation_clock']['real_time_factor'], .4)
        self.assertIn('CPU pacing only', spec['validation_clock']['scope'])
        self.assertAlmostEqual(float(factor.text), .4, delta=1e-12)
        with tempfile.TemporaryDirectory() as tmp:
            for bad in (0., 1.5, True):
                with self.assertRaises(ValueError):
                    write_scene(REPO, Path(tmp) / 'bad', real_time_factor=bad)


if __name__ == '__main__':
    unittest.main()
