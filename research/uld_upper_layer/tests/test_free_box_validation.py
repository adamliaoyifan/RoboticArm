from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import sys
import time
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from free_box_validation import prepare_free_box_motion
from planning_types import UpperConfig
from selection_policy import SelectionPolicy
from execution_policy import ExecutionPolicy
from physical_executor import STAGES


class FreeBoxPreparationContracts(unittest.TestCase):
    def setUp(self):
        self.config = UpperConfig.load(); self.selection = SelectionPolicy.load(); self.execution = ExecutionPolicy.load()
        self.pose = np.eye(4); self.pose[:3, 3] = [2., 3., 4.]
        self.plan = dict(valid=True, action=dict(box_T_tool=np.eye(4).ravel().tolist()), stages=[dict(
            name=name, motion_kind='joint' if name == 'push_pre' else 'cartesian',
            payload_mode='world', box_motion_model='fixed_world', box_reference_container=self.pose.ravel().tolist(),
            contact_tool=False, certificate=dict(valid=True),
            trajectory=dict(positions=[[0.] * 6, [1.] * 6])) for name in STAGES])
        self.failed = dict(noncontact_clearance_certified=False, stages=[dict(
            stage='push_pre', reason='ENVELOPE_SAMPLE_UNPROVED')])
        self.passed = dict(noncontact_clearance_certified=True, stages=[])
        self.new_path = [[0.] * 6, [.6] * 6, [1.] * 6]
        self.source_scene = SimpleNamespace(world_container=np.eye(4), set_payload=lambda *a, **k: None)
        self.events = []

    def prepare(self, proofs, path=None, expired=False):
        with patch('free_box_validation.FullArmScene', return_value=SimpleNamespace()),\
                patch('free_box_validation.FreeBoxEnvelopeScene', return_value=self.source_scene),\
                patch('free_box_validation.certify_payload_noncontact_tube', side_effect=proofs) as validate,\
                patch('free_box_validation.joint_path', return_value=(self.new_path if path is None else path,
                    dict(certificate=dict(valid=True)))) as route,\
                patch('free_box_validation.time_path', side_effect=lambda scene, path, config: dict(positions=path)):
            result = prepare_free_box_motion({}, None, SimpleNamespace(dimensions=(.55, .4, .25)),
                self.plan, self.config, self.selection, self.execution,
                time.monotonic() + (-1 if expired else 3), self.events.append)
        return result, validate, route

    def test_repair_is_independently_rechecked_without_changing_input_or_endpoints(self):
        original = deepcopy(self.plan)
        result, validate, route = self.prepare([self.failed, self.passed])
        self.assertTrue(result['valid']); self.assertEqual(validate.call_count, 2)
        self.assertEqual(route.call_count, 1); self.assertEqual(self.plan, original)
        before = next(s for s in original['stages'] if s['name'] == 'push_pre')
        after = next(s for s in result['plan']['stages'] if s['name'] == 'push_pre')
        self.assertEqual(after['trajectory']['positions'], self.new_path)
        self.assertEqual(before['trajectory']['positions'][0], after['trajectory']['positions'][0])
        self.assertEqual(before['trajectory']['positions'][-1], after['trajectory']['positions'][-1])
        self.assertFalse(result['plan']['execution_admitted'])

    def test_contact_cartesian_and_moving_box_semantics_are_not_rerouted(self):
        for change in (dict(motion_kind='cartesian'), dict(contact_tool=True),
                       dict(box_motion_model='straight_push'), dict(box_reference_container=None)):
            with self.subTest(change=change):
                stage = next(s for s in self.plan['stages'] if s['name'] == 'push_pre')
                old = deepcopy(stage); stage.update(change)
                result, _, route = self.prepare([self.failed])
                self.assertFalse(result['valid']); route.assert_not_called()
                self.assertFalse(result['plan']['valid'])
                stage.clear(); stage.update(old)

    def test_changed_endpoint_and_repeated_unproved_repair_fail_closed(self):
        result, _, _ = self.prepare([self.failed], path=[[.1] * 6, [1.] * 6])
        self.assertEqual(result['reason'], 'ENVELOPE_ROUTE_ENDPOINT_CHANGED')
        result, validate, route = self.prepare([self.failed, self.failed])
        self.assertFalse(result['valid']); self.assertEqual(validate.call_count, 2)
        self.assertEqual(route.call_count, 1)

    def test_deadline_and_already_proved_case_do_not_start_routing(self):
        result, validate, route = self.prepare([], expired=True)
        self.assertFalse(result['valid']); validate.assert_not_called(); route.assert_not_called()
        result, validate, route = self.prepare([self.passed])
        self.assertTrue(result['valid']); route.assert_not_called()


if __name__ == '__main__':
    unittest.main()
