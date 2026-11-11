"""World construction must retain source physics and reject changed inputs."""
from dataclasses import replace
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from assets import load
from ideal_snapshot import capture_fixture
from planning_types import UpperConfig
from release_fixture import ReleaseFixturePolicy
from release_geometry import release_geometry
from release_scene import write_release_scene
from scene import write_scene


def content(element):
    return (element.tag, element.attrib, (element.text or '').strip(),
            [content(child) for child in element])


class ReleaseWorldContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.repo = Path(__file__).resolve().parents[3]
        cls.temporary = tempfile.TemporaryDirectory()
        cls.directory = Path(cls.temporary.name)
        cls.source = cls.directory / 'source_world'
        write_scene(cls.repo, cls.source)
        cls.assets = load(cls.repo)
        cls.config = UpperConfig.load(); cls.policy = ReleaseFixturePolicy.load()
        cls.snapshot, cls.state = capture_fixture(cls.source, cls.assets, cls.config)
        cls.case = release_geometry(cls.assets, cls.snapshot, cls.state, cls.config, .6, 0., 0)
        # This is an opaque identity fixture for a builder test, never loaded.
        cls.plugin = cls.directory / 'plugin_identity_fixture'
        cls.plugin.write_bytes(b'opaque-builder-test-identity')

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def test_world_preserves_all_source_models_and_physics(self):
        with tempfile.TemporaryDirectory() as temporary:
            out = Path(temporary) / 'new'
            manifest = write_release_scene(self.assets, self.snapshot, self.state, self.config,
                self.case, self.policy, self.source, self.plugin, out)
            original = ET.parse(self.source / 'standalone.sdf').getroot().find('world')
            result = ET.parse(out / 'release.sdf').getroot().find('world')
            for model in original.findall('model'):
                target = result.find(f"model[@name='{model.get('name')}']")
                if model.get('name') == 'upper_layer_payload':
                    model.remove(model.find('pose')); target.remove(target.find('pose'))
                self.assertEqual(content(model), content(target))
            self.assertEqual(content(original.find('physics')), content(result.find('physics')))
            native = result.find("plugin[@name='uld_upper::IdealVacuum']")
            self.assertEqual(native.findtext('initially_attached'), 'true')
            self.assertEqual([n.text for n in native.findall('joint_name')], ['release_x', 'release_z'])
            self.assertEqual(len(native.findall('observed_model')), 24)
            self.assertEqual(manifest['physical_stability'], 'not_evaluated')
            with self.assertRaisesRegex(ValueError, 'ALREADY_EXISTS'):
                write_release_scene(self.assets, self.snapshot, self.state, self.config,
                    self.case, self.policy, self.source, self.plugin, out)

    def test_changed_case_is_rejected_before_writing(self):
        with tempfile.TemporaryDirectory() as temporary:
            out = Path(temporary) / 'new'
            with self.assertRaisesRegex(ValueError, 'CASE_INPUT_CHANGED'):
                write_release_scene(self.assets, self.snapshot, self.state, self.config,
                    replace(self.case, requested_support_ratio=.55), self.policy, self.source, self.plugin, out)
            self.assertFalse(out.exists())


if __name__ == '__main__':
    unittest.main()
