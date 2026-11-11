"""Analytic minima and hidden interior collisions test interval scoring."""
from pathlib import Path
import sys
import time
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from clearance_bounds import DistanceSample,bound_joint_path
from plan_clearance import FullArmDistanceSampler
from types import SimpleNamespace
from unittest.mock import patch


class ClearanceBoundsContracts(unittest.TestCase):
    def test_off_center_analytic_minimum_is_bracketed_and_search_history_is_unused(self):
        def evaluate(i,u,q):
            distance=.006+(q[0]-.23)**2
            return DistanceSample(((distance,('arm','world'),'robot'),),distance,('arm','world'))
        result=bound_joint_path([[-.5],[.5]],evaluate,lambda *a:2*a[1][0],.005,.00025,time.monotonic()+5)
        self.assertTrue(result['valid'])
        self.assertLessEqual(result['robot_lower_m'],.006)
        self.assertGreaterEqual(result['robot_upper_m'],.006)
        self.assertLessEqual(result['robot_upper_m']-result['robot_lower_m'],.00025+1e-10)
        self.assertGreater(result['checks'],1)

    def test_static_robot_with_independently_moving_box_does_not_lose_motion_bound(self):
        # The robot does not move, but an approaching free payload collides near
        # the end. A q-only bound would falsely accept the midpoint clearance.
        scene=SimpleNamespace(relative_motion_bound=lambda pair,delta:0.)
        def box_motion(i,u):
            pose=np.eye(4);pose[0,3]=.02*u;return pose
        sampler=FullArmDistanceSampler(scene,1e-6,box_motion)
        def evaluate(i,u,q):
            distance=.02-.02*u
            return DistanceSample(((distance,('arm','payload'),'robot'),),distance,('arm','payload'),
                                  dict(reason='COLLISION') if distance<.005 else None)
        result=bound_joint_path([[0.],[0.]],evaluate,sampler.motion_bound,.005,.00025,time.monotonic()+5)
        self.assertFalse(result['valid'])
        self.assertEqual(result['diagnostic']['reason'],'COLLISION')

    def test_missing_evidence_and_deadline_cannot_yield_partial_score(self):
        result=bound_joint_path([[0.],[1.]],lambda *a:DistanceSample((),float('inf'),()),
            lambda *a:0.,.005,.00025,time.monotonic()+5)
        self.assertFalse(result['valid'])
        result=bound_joint_path([[0.],[1.]],lambda *a:self.fail('expired deadline queried geometry'),
            lambda *a:0.,.005,.00025,time.monotonic()-1)
        self.assertEqual(result['reason'],'CLEARANCE_SCORE_BUDGET_EXHAUSTED')

    def test_runtime_numeric_enclosure_bounds_fcl_discrepancy_without_relaxing_margin(self):
        import tempfile,yaml
        from selection_policy import SelectionPolicy
        actual=.01-30e-9
        scene=SimpleNamespace(robot_parts={'arm':None},sample_distances=[(.01,('arm','world'))],
            config=SimpleNamespace(collision_margin_m=.005),check_state=lambda *a:None)
        for error,valid in ((1e-9,False),(1e-6,True)):
            with tempfile.TemporaryDirectory() as directory:
                path=Path(directory)/'selection.yaml'
                from dataclasses import asdict
                values=asdict(SelectionPolicy.load());values['distance_numeric_error_m']=error
                path.write_text(yaml.safe_dump(values))
                policy=SelectionPolicy.load(path)
            sampler=FullArmDistanceSampler(scene,policy.distance_numeric_error_m)
            with patch.object(sampler,'_exact',return_value=actual):
                sample=sampler.evaluate(0,0.,[0.],False)
            self.assertEqual(sample.failure is None,valid)
            if valid:
                self.assertLessEqual(sample.distances[0][0],actual)
                self.assertGreaterEqual(sample.robot_minimum_m,actual)


if __name__=='__main__':unittest.main()
