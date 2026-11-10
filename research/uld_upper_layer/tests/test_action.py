import sys
from pathlib import Path
import unittest
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'src/luggage_description'))
from geometry import axis_box, clipped_at_x, handoff_interval, pose, separated
from action import boundary_failure, build, cases, check_state


def fixture():
    return dict(tool=[axis_box('suction_panel:0',[-.09,-.09,-.286],[.09,.09,-.003])],
                hull=SimpleNamespace(ceiling_z=2.01, floor_z=.53, half_x=.745,
                                     half_y=.985, chamfer=None),
                scene={'container':{'opening':{'aperture':{'corners':[
                    [-.745,-.928,.597],[-.745,.398,.597],[-.745,.398,2.009],[-.745,-.928,2.009]]}}}},
                contact={'footprint_size_xy_m':[.18,.18],'boundary_margin_m':.015},
                catalog={'box_catalog':[{'id':n,'size':[.55,.4,.25]}
                                       for n in ('carryon','standard','large')]})


class GeometryTests(unittest.TestCase):
    def test_centered_grasp_has_no_stable_handoff(self):
        window=handoff_interval(.6,.5,.3,.09,.02,.005)
        self.assertFalse(window['feasible'])

    def test_rear_grasp_has_window_but_rear_heavy_box_may_not(self):
        self.assertTrue(handoff_interval(.6,.5,.495,.09,.02,.005)['feasible'])
        self.assertFalse(handoff_interval(.6,.7,.495,.09,.02,.005)['feasible'])

    def test_exact_half_is_not_stability_margin(self):
        self.assertFalse(handoff_interval(.6,.5,.3,0,.02,0)['feasible'])

    def test_touch_is_allowed_but_clearance_is_not(self):
        a=axis_box('a',[0,0,0],[1,1,1]); b=axis_box('b',[1,0,0],[2,1,1])
        self.assertTrue(separated(a,b,0));self.assertFalse(separated(a,b,.005))
        b.center[0]-=.01
        self.assertFalse(separated(a,b,0))

    def test_rotated_separation_and_rigid_invariance(self):
        a=axis_box('a',[0,0,0],[1,1,1]);b=axis_box('b',[2,0,0],[3,1,1])
        transform=pose([3,-1,7],Rotation.from_euler('xyz',[.2,.3,.9]).as_matrix())
        self.assertTrue(separated(a.moved(transform),b.moved(transform),.1))
        np.testing.assert_allclose(a.moved(transform).moved(np.linalg.inv(transform)).corners(),a.corners(),atol=1e-12)

    def test_aperture_clipping_adds_crossing_vertices(self):
        a=axis_box('a',[-1,-1,-1],[1,1,1])
        vertices=clipped_at_x(a,0)
        self.assertEqual(len(vertices),8)
        self.assertAlmostEqual(vertices[:,0].min(),0)

    def test_matrix_cardinality_and_stages(self):
        assets=fixture(); matrix=list(cases(assets));self.assertEqual(len(matrix),864)
        case=next(c for c in matrix if c['grasp']=='rear' and c['com_from_front_fraction']==.5)
        result=build(assets,case)
        self.assertEqual(result['outcome'],'candidate')
        stage={s['name']:s for s in result['stages']}
        self.assertEqual(stage['handoff']['contact_mode'],'tool_and_support')
        self.assertEqual(stage['release_down']['contact_mode'],'support_only')
        self.assertEqual(stage['push_end']['contact_mode'],'support_and_pusher')
        np.testing.assert_allclose(stage['release_down']['payload'].center,stage['handoff']['payload'].center)
        self.assertLess(stage['release_down']['tcp'][2,3],stage['handoff']['tcp'][2,3])

    def test_front_protrusion_blocks_handoff(self):
        assets=fixture();assets['tool'].append(axis_box('blocked_sensor',[.2,0,-.3],[.5,.1,-.2]))
        case=next(c for c in cases(assets) if c['grasp']=='rear' and c['com_from_front_fraction']==.5)
        self.assertEqual(build(assets,case)['reason'],'handoff_window_empty')

    def test_ceiling_rejects_payload_even_when_center_fits(self):
        assets=fixture()
        fits=axis_box('payload',[-.5,-.4,1.7],[0,-.1,2.0])
        self.assertIsNone(boundary_failure(fits,assets))
        fits.center[2]+=.02
        self.assertEqual(boundary_failure(fits,assets)['reason'],'uld_hull_clearance')

    def test_support_collision_returns_body_pair(self):
        assets=fixture()
        case=next(c for c in cases(assets) if c['grasp']=='rear' and c['com_from_front_fraction']==.5)
        result=build(assets,case)
        stage=next(s for s in result['stages'] if s['name']=='handoff')
        self.assertIsNone(check_state(assets,result,stage))
        blocked=dict(stage,tcp=stage['tcp'].copy())
        blocked['tcp'][0,3]+=.3
        failure=check_state(assets,result,blocked)
        self.assertEqual(failure['reason'],'tool_support_clearance')
        self.assertEqual(failure['obstacle']['name'],'support')


if __name__=='__main__':
    unittest.main()
