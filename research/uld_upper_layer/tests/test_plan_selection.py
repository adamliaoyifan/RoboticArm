"""Selection contract tests: collision priority, stale proofs and action diversity."""
from pathlib import Path
from dataclasses import asdict
import copy
import itertools
import sys
import tempfile
import unittest
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from geometry import pose
from planning_types import UpperConfig
from selection_policy import SelectionPolicy
from execution_policy import ExecutionPolicy
from noncontact_admission import FREE_BOX_STAGES
from physical_executor import STAGES
from plan_clearance import plan_digest
from plan_selection import rank_nominal_plans


class PlanSelectionContracts(unittest.TestCase):
    def setUp(self):
        self.config=UpperConfig.load();self.policy=SelectionPolicy.load();self.execution=ExecutionPolicy.load()

    def plan(self,name,clearance,margin=.04,area=.8,grasp_x=0.):
        flat=lambda xyz:tuple(pose(xyz).ravel())
        action=dict(action_id=name,placement=dict(final_box_pose=flat([0,0,1.]),
            snapshot_id='scene',config_revision=self.config.revision),
            box_T_tool=flat([grasp_x,0,0]),handoff_box_pose=flat([0,0,1.]),
            push_start_tool_pose=flat([-.3,0,1.]),tool_T_push_surface=flat([0,0,0]),
            push_contact_geometry_id='source_plate')
        stages=[dict(name=name,trajectory=dict(positions=[[0.]*6,[0.]*6]),
            certificate=dict(valid=True),payload_mode='world',contact_tool=False) for name in STAGES]
        plan=dict(valid=True,action=action,stages=stages)
        plan['clearance_score']=dict(valid=True,action_id=action['action_id'],plan_digest=plan_digest(plan),
            snapshot_id='scene',config_revision=self.config.revision,clearance_policy_revision=self.policy.clearance_revision,
            model_identity='models',robot_lower_m=clearance,robot_upper_m=clearance+.00005,
            payload_lower_m=.01,min_com_margin_m=margin,min_support_area_ratio=area,
            stages=[dict(stage=s,valid=True) for s in STAGES],robot_witness=dict(stage='push',pair=['arm','world']))
        plan['noncontact_tube']=dict(noncontact_clearance_certified=True,plan_digest=plan_digest(plan),
            snapshot_id='scene',config_revision=self.config.revision,model_identity='models',
            execution_policy_revision=self.execution.revision,clearance_policy_revision=self.policy.clearance_revision,
            distance_numeric_error_m=self.policy.distance_numeric_error_m,
            translation_error_m=self.execution.push_lateral_error_m,rotation_error_deg=self.execution.push_rotation_error_deg,
            error_frame='world',rotation_model='SO3_geodesic_ball_about_geometric_box_centre',
            requested_stages=list(FREE_BOX_STAGES),stages=[dict(stage=s,noncontact_clearance_certified=True,
                lower_m=.01,checks=1,certified_intervals=1,translation_model='perpendicular_ball' if s=='push' else 'isotropic_ball',
                transverse_direction_world=[1.,0.,0.] if s=='push' else None) for s in FREE_BOX_STAGES],
            execution_admitted=False,intended_contact_uncertainty='not_evaluated')
        return plan

    def rank(self,plans,policy=None):
        return rank_nominal_plans(plans,'scene',self.config,policy or self.policy,'models',self.execution)

    def test_collision_margin_beats_support_and_hard_support_failure_is_ineligible(self):
        a=self.plan('more_support',.01,.2,1.)
        b=self.plan('more_collision_space',.015,.025,.6,grasp_x=.02)
        bad=self.plan('unsupported',.05,.2,.59,grasp_x=.04)
        result=self.rank([a,b,bad])
        self.assertEqual(result['ordered_indices'],[1,0])
        self.assertEqual(result['rejected'][0]['reason'],'SUPPORT_BELOW_POLICY')

    def test_runtime_tie_band_changes_selection_without_recomputing_geometry(self):
        a=self.plan('more_support',.0145,.2,.9)
        b=self.plan('more_collision_space',.015,.025,.6,grasp_x=.02)
        self.assertEqual(self.rank([a,b])['selected_index'],0)
        with tempfile.TemporaryDirectory() as directory:
            values=asdict(self.policy);values['collision_tie_m']=.002
            path=Path(directory)/'selection.yaml';path.write_text(yaml.safe_dump(values))
            wider=SelectionPolicy.load(path)
        a=self.plan('more_support',.0135,.2,.9)
        self.assertEqual(self.rank([a,b])['selected_index'],1)
        self.assertEqual(self.rank([a,b],wider)['selected_index'],0)
        self.assertEqual(wider.clearance_revision,self.policy.clearance_revision)

    def test_anchored_tiers_are_deterministic_for_nontransitive_pairwise_ties(self):
        plans=[self.plan('a',.01,.3,grasp_x=0.),self.plan('b',.0107,.2,grasp_x=.02),
               self.plan('c',.0114,.1,grasp_x=.04)]
        for sequence in itertools.permutations(plans):
            result=self.rank(sequence)
            self.assertEqual([sequence[i]['action']['action_id'] for i in result['ordered_indices']],['b','c','a'])

    def test_same_action_with_different_joint_path_does_not_inflate_count(self):
        a=self.plan('a',.01)
        duplicate=copy.deepcopy(a);duplicate['action']['action_id']='another_ik'
        duplicate['stages'][0]['trajectory']['positions'][1][0]=.1
        duplicate['clearance_score']['action_id']='another_ik'
        duplicate['clearance_score']['plan_digest']=plan_digest(duplicate)
        duplicate['noncontact_tube']['plan_digest']=plan_digest(duplicate)
        distinct=self.plan('different_grasp',.01,grasp_x=.02)
        result=self.rank([a,duplicate,distinct])
        self.assertEqual(result['achieved'],2)
        self.assertTrue(any(r['reason']=='EQUIVALENT_GEOMETRIC_ACTION' for r in result['rejected']))

    def test_missing_stale_or_modified_proofs_are_not_ranked(self):
        for change in (lambda p:p.pop('clearance_score'),
                       lambda p:p['clearance_score'].update(model_identity='old_model'),
                       lambda p:p['clearance_score'].update(snapshot_id='old_scene'),
                       lambda p:p['stages'][0]['trajectory']['positions'][1].__setitem__(0,.1),
                       lambda p:p['clearance_score'].update(robot_lower_m=.004),
                       lambda p:p['clearance_score']['stages'].pop()):
            plan=self.plan('a',.01);change(plan);result=self.rank([plan])
            self.assertIsNone(result['selected_index'])
            self.assertFalse(result['nominal_minimum_met'])

    def test_missing_stale_partial_or_undersized_envelope_cannot_enter_selection(self):
        changes=(lambda p:p.pop('noncontact_tube'),
                 lambda p:p['noncontact_tube'].update(execution_policy_revision='stale'),
                 lambda p:p['noncontact_tube'].update(translation_error_m=.001),
                 lambda p:p['noncontact_tube']['stages'].pop(),
                 lambda p:p['noncontact_tube']['stages'][0].update(lower_m=.004),
                 lambda p:p['noncontact_tube'].update(execution_admitted=True))
        for change in changes:
            with self.subTest(change=change):
                plan=self.plan('a',.01);change(plan);result=self.rank([plan])
                self.assertIsNone(result['selected_index'])
                self.assertFalse(result['nominal_minimum_met'])


if __name__=='__main__':unittest.main()
