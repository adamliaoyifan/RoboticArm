"""Check fixture geometry, inertia equivalence and physical joint directions."""
from dataclasses import replace
from pathlib import Path
import tempfile
import json
import subprocess
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from assets import load, origin, source_mesh
from candidates import matrix
from ideal_snapshot import capture_fixture
from planning_types import UpperConfig
from release_geometry import release_geometry
from release_fixture import ReleaseFixturePolicy, release_carriage_model, source_tool_link
from scene import fixture


def sdf_pose(node):
    xyzrpy = np.fromstring(node.findtext('pose', '0 0 0 0 0 0'), sep=' ')
    result = np.eye(4)
    result[:3, 3] = xyzrpy[:3]
    result[:3, :3] = Rotation.from_euler('xyz', xyzrpy[3:]).as_matrix()
    return result


class ReleaseFixtureContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.assets = load(Path(__file__).resolve().parents[3])
        cls.config = UpperConfig.load(); cls.policy = ReleaseFixturePolicy.load()
        cls.temp = tempfile.TemporaryDirectory()
        spec = fixture(cls.assets)
        spec['pickup_payload'] = dict(name='upper_layer_payload', pose_world=[-1, 0, .986, 0, 0, 0])
        (Path(cls.temp.name) / 'fixture.json').write_text(json.dumps(spec))
        cls.snapshot, cls.state = capture_fixture(cls.temp.name, cls.assets, cls.config)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def case(self, ratio=.6, yaw=0):
        return release_geometry(self.assets, self.snapshot, self.state, self.config, ratio, yaw, 0)

    def test_source_collision_geometry_retains_all_material_transforms(self):
        tool, ledger = source_tool_link(self.assets)
        urdf = self.assets['urdf']
        base_tcp = urdf.tcp([0.] * 6, urdf.path_to('suction_contact_frame'))
        self.assertEqual(len(ledger['parts']), len(self.assets['tool']))
        for item in ledger['parts']:
            name, index = item['source_part'].split(':')
            source = urdf.root.find(f"link[@name='{name}']").findall('collision')[int(index)]
            converted = tool.find(f"collision[@name='{item['sdf_collision']}']")
            expected = urdf.tcp([0.] * 6, urdf.path_to(name)) @ origin(source)
            np.testing.assert_allclose(base_tcp @ sdf_pose(converted), expected, atol=2e-9)
            mesh = source.find('geometry/mesh')
            if mesh is not None:
                self.assertEqual(Path(converted.findtext('geometry/mesh/uri')),
                                 source_mesh(self.assets['repo'], mesh.get('filename')).resolve())
                np.testing.assert_allclose(np.fromstring(converted.findtext('geometry/mesh/scale'), sep=' '),
                    np.fromstring(mesh.get('scale', '1 1 1'), sep=' '))
            else:
                self.assertEqual(converted.findtext('geometry/box/size'), source.find('geometry/box').get('size'))

    def test_welded_body_preserves_source_rigid_kinetic_energy(self):
        _, ledger = source_tool_link(self.assets)
        urdf = self.assets['urdf']; tcp = urdf.tcp([0.] * 6, urdf.path_to('suction_contact_frame'))
        source_names = {'suction_panel', 'eef_mount_adapter', 'camera_link', 'mid360_mount_frame'}
        components = []
        for name in source_names:
            inertial = urdf.root.find(f"link[@name='{name}']/inertial")
            mass = float(inertial.find('mass').get('value'))
            frame = np.linalg.inv(tcp) @ urdf.tcp([0.] * 6, urdf.path_to(name)) @ origin(inertial)
            a = {k: float(v) for k, v in inertial.find('inertia').attrib.items()}
            inertia = np.array([[a['ixx'], a['ixy'], a['ixz']], [a['ixy'], a['iyy'], a['iyz']],
                                [a['ixz'], a['iyz'], a['izz']]])
            components.append((mass, frame[:3, 3], frame[:3, :3] @ inertia @ frame[:3, :3].T))
        self.assertAlmostEqual(ledger['mass_kg'], sum(row[0] for row in components))
        for omega in (np.array([1., 0, 0]), np.array([0, 1., 0]), np.array([.3, -.4, .9])):
            velocity = np.array([.2, -.1, .7])
            expected = sum(.5 * m * np.linalg.norm(velocity + np.cross(omega, c)) ** 2
                           + .5 * omega @ inertia @ omega for m, c, inertia in components)
            actual = .5 * ledger['mass_kg'] * np.linalg.norm(velocity + np.cross(
                omega, ledger['com_tool_m'])) ** 2 + .5 * omega @ np.array(ledger['inertia_com_tool_kg_m2']) @ omega
            self.assertAlmostEqual(actual, expected, places=12)

    def test_carriage_axes_follow_container_at_both_tool_yaws(self):
        for yaw, ratio in ((0, .6), (90, .45)):
            case = self.case(ratio, yaw)
            model, _ = release_carriage_model(self.assets, case, self.policy)
            world = sdf_pose(model)
            np.testing.assert_allclose(world, self.assets['world_container'] @ matrix(case.container_T_tool), atol=2e-9)
            for label, container_axis in [('x', [1, 0, 0]), ('z', [0, 0, 1])]:
                axis = np.fromstring(model.findtext(f"joint[@name='release_{label}']/axis/xyz"), sep=' ')
                np.testing.assert_allclose(world[:3, :3] @ axis,
                    self.assets['world_container'][:3, :3] @ container_axis, atol=2e-9)
            self.assertEqual(model.findtext("joint[@name='world_anchor']/parent"), 'world')
            self.assertEqual(model.findtext('static'), 'false')

    def test_unavailable_geometry_is_not_a_physics_fixture(self):
        with self.assertRaisesRegex(ValueError, 'GEOMETRY_UNAVAILABLE'):
            release_carriage_model(self.assets, self.case(.8), self.policy)

    def test_exported_model_is_accepted_by_installed_sdf_parser(self):
        model, _ = release_carriage_model(self.assets, self.case(), self.policy)
        root = ET.Element('sdf', version='1.8')
        world = ET.SubElement(root, 'world', name='carriage_contract')
        world.append(model)
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'model.sdf'
            ET.ElementTree(root).write(path)
            result = subprocess.run(['ign', 'sdf', '-k', str(path)], capture_output=True,
                                    text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('Valid', result.stdout)

    def test_runtime_policy_changes_actual_joint_limits_and_controller(self):
        policy = replace(self.policy, effort_limit_n=321., position_p_gain=1234., travel_limit_m=.3)
        model, _ = release_carriage_model(self.assets, self.case(), policy)
        self.assertEqual(float(model.findtext("joint[@name='release_x']/axis/limit/effort")), 321.)
        self.assertEqual(float(model.findtext('plugin/p_gain')), 1234.)
        self.assertEqual(float(model.findtext("joint[@name='release_z']/axis/limit/lower")), -.3)
        with self.assertRaises(ValueError):
            replace(policy, velocity_limit_mps=0)


if __name__ == '__main__':
    unittest.main()
