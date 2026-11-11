"""Process-boundary admission: a completed nominal path alone is insufficient."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from planning_worker import _worker
from planning_types import UpperConfig
from selection_policy import SelectionPolicy


class WorkerScoreAdmission(unittest.TestCase):
    def run_worker(self,directory,score=None,error=None):
        class Scene:
            minimum_clearance_m=.04
            clearance_witness=('search_only','obstacle')
            def __init__(self,*args):pass
        with patch('assets.load',return_value={}),patch('full_arm.FullArmScene',Scene),\
             patch('stage_planner.plan_action',return_value=dict(valid=True,stages=[])),\
             patch('plan_clearance.score_plan',return_value=score,side_effect=error):
            _worker('.',None,None,None,[],[],UpperConfig.load(),SelectionPolicy.load(),directory,5.,0)

    def test_failed_score_preserves_path_but_denies_admission(self):
        with tempfile.TemporaryDirectory() as directory:
            self.run_worker(directory,dict(valid=False,reason='CLEARANCE_SCORE_BUDGET_EXHAUSTED'))
            output=Path(directory)
            self.assertTrue(json.loads((output/'nominal_result.json').read_text())['valid'])
            result=json.loads((output/'result.json').read_text())
            self.assertFalse(result['valid'])
            self.assertEqual(result['reason'],'COMPLETE_PATH_CLEARANCE_REJECTED')
            self.assertEqual(result['clearance_score_status'],'not_accepted')

    def test_completed_score_is_attached_without_using_search_accumulator_as_score(self):
        with tempfile.TemporaryDirectory() as directory:
            self.run_worker(directory,dict(valid=True,robot_lower_m=.006))
            result=json.loads((Path(directory)/'result.json').read_text())
            self.assertTrue(result['valid'])
            self.assertEqual(result['clearance_score']['robot_lower_m'],.006)
            self.assertEqual(result['search_observed_clearance_lower_bound_m'],.04)

    def test_scoring_crash_leaves_replayable_nominal_input_and_no_accepted_result(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(RuntimeError,'diagnostic scoring failure'):
                self.run_worker(directory,error=RuntimeError('diagnostic scoring failure'))
            output=Path(directory)
            self.assertTrue((output/'nominal_result.json').exists())
            self.assertTrue((output/'error.txt').exists())
            self.assertFalse((output/'result.json').exists())


if __name__=='__main__':unittest.main()
