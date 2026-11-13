"""Demote-only adaptive family scheduling from endpoint-barren failures."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from planning_types import UpperConfig
from request_planner import AdaptiveFamilyQueue, action_family


class Stub:
    def __init__(self, yaw, spin):
        self.action_id = f'yaw{yaw}spin{spin}'
        self.tool_yaw_deg = float(yaw)
        self.tool_T_push_surface = tuple([float(spin)] * 16)


BARREN = 'NO_SAMPLED_HANDOFF_ENDPOINT_SOLUTION'


class FamilyDemoteContracts(unittest.TestCase):
    def three_families(self):
        # Interleaved static order A,B,C,A,B,C,A,B,A (A = yaw0 spin1 family).
        return ([Stub(0, 1), Stub(90, 1), Stub(180, 1),
                 Stub(0, 1), Stub(90, 1), Stub(180, 1),
                 Stub(0, 1), Stub(90, 1), Stub(0, 1)])

    def test_barren_family_moves_back_and_is_never_dropped(self):
        queue = AdaptiveFamilyQueue(self.three_families(), 3)
        order = []
        demotions = []
        for _ in range(9):
            action = queue.pop()
            self.assertIsNotNone(action)
            order.append(action.action_id)
            # Family A (yaw 0) is endpoint-barren; everything else fails later
            # at path level, which must not demote.
            reason = BARREN if action.tool_yaw_deg == 0 else 'SWEEP_REJECTED'
            demotion = queue.record(action, reason)
            if demotion:
                demotions.append(demotion)
        self.assertIsNone(queue.pop())
        self.assertEqual(len(demotions), 1)
        self.assertEqual(demotions[0]['tool_yaw_deg'], 0)
        self.assertEqual(demotions[0]['candidates_moved'], 1)  # last A left when demoted
        # A candidates tried first three (limit), the rest only after others.
        self.assertEqual(order[:3], ['yaw0spin1', 'yaw90spin1', 'yaw180spin1'])
        self.assertEqual(order[3], 'yaw0spin1')          # 2nd barren
        self.assertEqual(order[4], 'yaw90spin1')
        self.assertEqual(order[5], 'yaw180spin1')
        self.assertEqual(order[6], 'yaw0spin1')          # 3rd barren -> demoted
        self.assertEqual(order[7], 'yaw90spin1')         # remaining families first
        self.assertEqual(order[8], 'yaw0spin1')          # demoted A member last
        # Nothing dropped: every candidate appears exactly once.
        self.assertEqual(sorted(order), sorted(a.action_id for a in self.three_families()))

    def test_non_barren_failure_resets_streak(self):
        actions = [Stub(0, 1)] * 5
        queue = AdaptiveFamilyQueue(actions, 3)
        queue.record(actions[0], BARREN)
        queue.record(actions[1], BARREN)
        queue.record(actions[2], 'STAGE_PATH_FAILED')  # endpoints exist: reset
        demotion = queue.record(actions[3], BARREN)
        self.assertIsNone(demotion)
        demotion = queue.record(actions[3 - 1], BARREN)
        self.assertIsNone(demotion)  # streak 2 after reset, not yet 3

    def test_limit_zero_keeps_static_order(self):
        queue = AdaptiveFamilyQueue(self.three_families(), 0)
        first = queue.pop()
        queue.record(first, BARREN)
        queue.record(queue.pop(), BARREN)
        queue.record(queue.pop(), BARREN)
        # Static order continues regardless of failures.
        self.assertEqual([queue.pop().action_id for _ in range(6)],
                         ['yaw0spin1', 'yaw90spin1', 'yaw180spin1',
                          'yaw0spin1', 'yaw90spin1', 'yaw0spin1'])

    def test_family_key_separates_spins_and_config_carries_limit(self):
        self.assertNotEqual(action_family(Stub(0, 1)), action_family(Stub(0, 2)))
        self.assertNotEqual(action_family(Stub(0, 1)), action_family(Stub(90, 1)))
        self.assertEqual(action_family(Stub(0, 1)), action_family(Stub(0.0, 1)))
        config = UpperConfig.load()
        self.assertEqual(config.family_demote_barren_limit, 3)
        with self.assertRaises(ValueError):
            UpperConfig.load(**{'family_demote_barren_limit': -1})
        with self.assertRaises(ValueError):
            UpperConfig.load(**{'family_demote_barren_limit': True})


if __name__ == '__main__':
    unittest.main()
