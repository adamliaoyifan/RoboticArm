#!/usr/bin/env python3
"""Retreat may start only from descend's final knot, after detach.

Run17c/d aborted QP_START_STATE_MISMATCH because descend had already
reported success while joint 2 was still ~0.011 rad short of the certified
knot. The gate stays 0.01 rad.
"""

import os
import sys
import unittest

_SCRIPTS = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from place_smoke_driver import (  # noqa: E402
    CANONICAL_ARM_JOINTS,
    QP_START_TOLERANCE_RAD,
    detach_joint_motion,
    retreat_start_decision,
    trajectory_endpoint,
)


def _q(joint2):
    return [0.1, float(joint2), 0.2, 0.3, 0.4, 0.5]


def _sample(stamp, joint2, names=None, positions=None):
    names = list(names or CANONICAL_ARM_JOINTS)
    if positions is None:
        values = _q(joint2)
        if list(names) != list(CANONICAL_ARM_JOINTS):
            by_name = dict(zip(CANONICAL_ARM_JOINTS, values))
            values = [by_name[name] for name in names]
    else:
        values = list(positions)
    return {"stamp": float(stamp), "name": names, "position": values}


class TestRetreatStartContract(unittest.TestCase):
    def test_tolerance_is_the_executor_gate(self):
        self.assertEqual(QP_START_TOLERANCE_RAD, 0.01)

    def test_retreat_start_equal_to_descend_final_is_ready(self):
        decision = retreat_start_decision(
            [_sample(1.0, -2.70), _sample(2.0, -2.705)],
            detach_stamp=1.5,
            retreat_names=CANONICAL_ARM_JOINTS,
            retreat_positions=_q(-2.705),
            descend_names=CANONICAL_ARM_JOINTS,
            descend_positions=_q(-2.705))
        self.assertEqual(decision["status"], "ready")
        self.assertEqual(decision["retreat_first_joints"],
                         decision["descend_final_joints"])
        self.assertLessEqual(decision["max_delta"], 0.01)
        self.assertTrue(decision["stamp_after_detach"])
        self.assertGreater(decision["joint_state_stamp"], 1.5)

    def test_stale_joint_state_after_detach_is_rejected(self):
        decision = retreat_start_decision(
            [_sample(1.0, -2.705)],
            detach_stamp=1.0,
            retreat_names=CANONICAL_ARM_JOINTS,
            retreat_positions=_q(-2.705),
            descend_names=CANONICAL_ARM_JOINTS,
            descend_positions=_q(-2.705))
        self.assertEqual(decision["status"], "stale_joint_state")
        self.assertFalse(decision["stamp_after_detach"])
        self.assertEqual(decision["joint_state_stamp"], 1.0)
        later = retreat_start_decision(
            [_sample(1.0, -2.694), _sample(2.2, -2.705)],
            detach_stamp=1.5,
            retreat_names=CANONICAL_ARM_JOINTS,
            retreat_positions=_q(-2.705),
            descend_names=CANONICAL_ARM_JOINTS,
            descend_positions=_q(-2.705))
        self.assertEqual(later["status"], "ready")
        self.assertEqual(later["joint_state_stamp"], 2.2)

    def test_permuted_joint_order_normalizes(self):
        gazebo = [
            "elfin_joint1", "elfin_joint2", "elfin_joint5",
            "elfin_joint3", "elfin_joint4", "elfin_joint6"]
        decision = retreat_start_decision(
            [_sample(2.0, -2.705, names=gazebo)],
            detach_stamp=1.0,
            retreat_names=list(reversed(CANONICAL_ARM_JOINTS)),
            retreat_positions=list(reversed(_q(-2.705))),
            descend_names=gazebo,
            descend_positions=_sample(0.0, -2.705, names=gazebo)["position"])
        self.assertEqual(decision["status"], "ready")
        self.assertEqual(
            decision["joint_names"], list(CANONICAL_ARM_JOINTS))

    def test_unknown_joint_order_is_rejected(self):
        decision = retreat_start_decision(
            [_sample(2.0, -2.705, names=[
                "elfin_joint1", "elfin_joint2", "elfin_joint3",
                "elfin_joint4", "elfin_joint5", "not_a_joint"],
                positions=_q(-2.705))],
            detach_stamp=1.0,
            retreat_names=CANONICAL_ARM_JOINTS,
            retreat_positions=_q(-2.705),
            descend_names=CANONICAL_ARM_JOINTS,
            descend_positions=_q(-2.705))
        self.assertEqual(decision["status"], "joint_order")

    def test_over_threshold_is_rejected(self):
        # run17d: live joint2 about 0.0104 rad short of the certified knot.
        decision = retreat_start_decision(
            [_sample(2.0, -2.694)],
            detach_stamp=1.0,
            retreat_names=CANONICAL_ARM_JOINTS,
            retreat_positions=_q(-2.705),
            descend_names=CANONICAL_ARM_JOINTS,
            descend_positions=_q(-2.705))
        self.assertEqual(decision["status"], "mismatch")
        self.assertGreater(decision["max_delta"], 0.01)
        self.assertEqual(decision["tolerance_rad"], 0.01)
        self.assertEqual(len(decision["delta_per_joint"]), 6)

    def test_exact_tolerance_is_not_widened(self):
        on_gate = retreat_start_decision(
            [_sample(2.0, -2.705 + 0.01)],
            detach_stamp=1.0,
            retreat_names=CANONICAL_ARM_JOINTS,
            retreat_positions=_q(-2.705),
            descend_names=CANONICAL_ARM_JOINTS,
            descend_positions=_q(-2.705))
        over_gate = retreat_start_decision(
            [_sample(2.0, -2.705 + 0.0101)],
            detach_stamp=1.0,
            retreat_names=CANONICAL_ARM_JOINTS,
            retreat_positions=_q(-2.705),
            descend_names=CANONICAL_ARM_JOINTS,
            descend_positions=_q(-2.705))
        self.assertEqual(on_gate["status"], "ready")
        self.assertEqual(over_gate["status"], "mismatch")

    def test_retreat_start_disagreement_is_rejected(self):
        decision = retreat_start_decision(
            [_sample(2.0, -2.705)],
            detach_stamp=1.0,
            retreat_names=CANONICAL_ARM_JOINTS,
            retreat_positions=_q(-2.690),
            descend_names=CANONICAL_ARM_JOINTS,
            descend_positions=_q(-2.705))
        self.assertEqual(decision["status"], "contract_mismatch")
        self.assertGreater(decision["max_delta"], 0.01)

    def test_endpoint_and_detach_motion(self):
        segment = {
            "joint_trajectory": {
                "joint_names": list(CANONICAL_ARM_JOINTS),
                "points": [
                    {"positions": _q(-2.0)},
                    {"positions": _q(-2.705)},
                ],
            }
        }
        names, first = trajectory_endpoint(segment, "first")
        _names, last = trajectory_endpoint(segment, "last")
        self.assertEqual(names, list(CANONICAL_ARM_JOINTS))
        self.assertAlmostEqual(first[1], -2.0)
        self.assertAlmostEqual(last[1], -2.705)
        motion = detach_joint_motion(
            _sample(1.0, -2.694), _sample(3.0, -2.705))
        self.assertTrue(motion["moved"])
        self.assertGreater(motion["max_delta"], 0.01)


if __name__ == "__main__":
    unittest.main()
