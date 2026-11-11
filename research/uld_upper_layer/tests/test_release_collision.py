"""Continuous withdrawal, actual source geometry and adversarial references."""
from dataclasses import replace
from pathlib import Path
import tempfile
import time
import unittest
import xml.etree.ElementTree as ET

import fcl
import numpy as np

from assets import load
from execution_policy import ExecutionPolicy
from geometry import pose
from ideal_snapshot import capture_fixture
from physical_observation import ObservedBody
from planning_types import UpperConfig
from release_collision import ReleaseCollisionScene, _Shape
from release_fixture import ReleaseFixturePolicy
from release_geometry import release_geometry
from release_observation import ReleaseObservation
from release_scene import write_release_scene
from release_trajectory import make_release_withdrawal
from scene import write_scene


class WithdrawalCertificates(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.root = Path(cls.temp.name)
        cls.assets = load(Path(__file__).resolve().parents[3])
        cls.config = UpperConfig.load()
        cls.fixture_policy = ReleaseFixturePolicy.load()
        source = cls.root/'source'; write_scene(cls.assets['repo'], source)
        snapshot, state = capture_fixture(source, cls.assets, cls.config)
        case = release_geometry(cls.assets, snapshot, state, cls.config, .6, 0., 0)
        plugin = cls.root/'opaque_plugin'; plugin.write_bytes(b'not-loaded-in-offline-tests')
        output = cls.root/'release'
        write_release_scene(cls.assets, snapshot, state, cls.config, case,
                            cls.fixture_policy, source, plugin, output)
        cls.world = output/'release.sdf'; cls.dimensions = state.dimensions
        lower = tuple(ObservedBody(b.name, b.size, tuple(pose(b.center).ravel()), 1.) for b in snapshot.occupied)
        cls.sample = ReleaseObservation(1., (0., 0.), case.container_T_box,
            case.container_T_tool, lower, (), True, True, 1, 0, 0, tuple(np.eye(4).ravel()))
        cls.motions = make_release_withdrawal(cls.sample, cls.config, ExecutionPolicy.load(), cls.fixture_policy)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def scene(self, world=None, sample=None):
        return ReleaseCollisionScene(self.assets, world or self.world, sample or self.sample,
                                     self.dimensions, self.config)

    def test_full_source_geometry_continuous_certificate_and_interface(self):
        scene = self.scene()
        tool = np.array(self.sample.container_T_tool).reshape(4, 4)
        self.assertTrue(scene.initial_seal_valid(tool))
        ordinary = scene.evaluate(tool, separating_interface=False)
        self.assertEqual(ordinary['failure'], 'RELEASE_TOOL_COLLISION_OR_CLEARANCE')
        self.assertEqual(ordinary['pair'], ['tool/suction_panel_3', 'payload'])
        result = scene.certify(self.motions, time.monotonic()+10)
        self.assertTrue(result['valid'], result)
        self.assertEqual(result['source_tool_parts'], 8)
        self.assertEqual(result['world_parts'], 28)
        self.assertEqual(result['planes'], 1)
        self.assertGreaterEqual(result['noncontact_lower_m'], self.config.collision_margin_m)
        self.assertGreater(result['interface_lower_m'], 0)
        self.assertFalse(result['execution_admitted'])

    def test_empty_missing_stage_and_forged_waypoints_rejected(self):
        scene = self.scene()
        for motions in ((), self.motions[:1]):
            with self.assertRaisesRegex(ValueError, 'STAGES_INVALID'):
                scene.certify(motions, time.monotonic()+5)
        first = self.motions[0]
        q = np.array(first.joint_displacements_xz_m); q[len(q)//2, 0] += .1
        forged = replace(first, joint_displacements_xz_m=tuple(map(tuple, q)))
        with self.assertRaisesRegex(ValueError, 'NOT_MONOTONE_LINE'):
            scene.certify((forged, self.motions[1]), time.monotonic()+5)
        changed_start = replace(self.sample, joint_displacements_xz_m=(.1, 0.))
        with self.assertRaisesRegex(ValueError, 'NOT_CONTINUOUS_TRANSLATION'):
            self.scene(sample=changed_start).certify(self.motions, time.monotonic()+5)

    def test_deadline_rejects_without_admission(self):
        result = self.scene().certify(self.motions, time.monotonic()-1)
        self.assertEqual(result['reason'], 'RELEASE_CERTIFICATE_DEADLINE')
        self.assertFalse(result['valid']); self.assertFalse(result['execution_admitted'])

    def test_clear_endpoints_do_not_hide_interior_obstacle(self):
        scene = self.scene()
        # A small synthetic probe isolates an interior hit from the original
        # tool's earlier downward sweep; all source tool/world geometry stays.
        motion = self.motions[1]
        start = np.array(motion.container_T_tool_start).reshape(4, 4)
        end = np.array(motion.container_T_tool_end).reshape(4, 4)
        geometry = ET.fromstring('<geometry><box><size>.001 .001 .001</size></box></geometry>')
        center_local = start[:3, :3].T @ np.array([0., 3., 0.])
        scene._tool.append(_Shape('test_probe', geometry, pose(center_local), {}))
        middle = (start[:3, 3]+end[:3, 3])/2 + start[:3, :3] @ center_local
        scene._world.append(_Shape('test_interior_obstacle', geometry, pose(middle), {}))
        self.assertNotIn('failure', scene.evaluate(start, separating_interface=False))
        self.assertNotIn('failure', scene.evaluate(end, separating_interface=False))
        result = scene.certify(self.motions, time.monotonic()+10)
        self.assertFalse(result['valid']); self.assertIn('COLLISION', result['reason'])
        self.assertEqual(result['stage'], 1)

    def test_omitted_tool_and_changed_lower_geometry_fail_closed(self):
        root = ET.parse(self.world)
        tool = root.find("world/model[@name='release_carriage']/link[@name='tool']")
        tool.remove(tool.find("collision[@name='camera_link_0']"))
        changed = self.root/'omitted.sdf'; root.write(changed)
        with self.assertRaisesRegex(ValueError, 'SOURCE_TOOL_SET_CHANGED'):
            self.scene(world=changed)
        b = self.sample.lower[0]
        modified = replace(b, dimensions=(b.dimensions[0]+.001, *b.dimensions[1:]))
        with self.assertRaisesRegex(ValueError, 'LOWER_GEOMETRY_CHANGED'):
            self.scene(sample=replace(self.sample, lower=(modified, *self.sample.lower[1:])))

    def test_changed_joint_axis_and_unobserved_dynamic_body_rejected(self):
        root = ET.parse(self.world)
        root.find("world/model[@name='release_carriage']/joint[@name='release_x']/axis/xyz").text = '0 0 1'
        changed = self.root/'axis.sdf'; root.write(changed)
        with self.assertRaisesRegex(ValueError, 'AXES_CHANGED'):
            self.scene(world=changed)
        root = ET.parse(self.world)
        root.find("world/model[@name='pickup_platform']/static").text = 'false'
        changed = self.root/'dynamic.sdf'; root.write(changed)
        with self.assertRaisesRegex(ValueError, 'UNOBSERVED_DYNAMIC'):
            self.scene(world=changed)

    def test_fcl_detects_contained_solids(self):
        outer = fcl.CollisionObject(fcl.Box(1., 1., 1.))
        inner = fcl.CollisionObject(fcl.Box(.01, .01, .01))
        self.assertGreater(fcl.collide(outer, inner, fcl.CollisionRequest(), fcl.CollisionResult()), 0)

    def test_nonrigid_observed_pose_rejected(self):
        box = np.array(self.sample.container_T_box).reshape(4, 4)
        box[0, 0] = 2.
        with self.assertRaisesRegex(ValueError, 'POSE_NOT_RIGID'):
            self.scene(sample=replace(self.sample, container_T_box=tuple(box.ravel())))


if __name__ == '__main__':
    unittest.main()
