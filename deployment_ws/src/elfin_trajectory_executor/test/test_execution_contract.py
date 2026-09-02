import unittest

from elfin_trajectory_executor.execution_contract import (
    DEFAULT_ACTION_NAME,
    EVENT_SUCCEEDED,
    SCHEMA,
    event_to_json,
    make_event,
    parse_event,
)


class ExecutionContractTest(unittest.TestCase):
    def test_action_name_matches_planner(self):
        self.assertEqual(
            DEFAULT_ACTION_NAME,
            "/elfin_arm_controller/follow_joint_trajectory",
        )

    def test_succeeded_is_the_only_ready_for_next(self):
        ok = make_event(EVENT_SUCCEEDED, goal_id="abc")
        self.assertTrue(ok["ready_for_next"])
        idle = make_event("idle")
        self.assertFalse(idle["ready_for_next"])
        aborted = make_event("aborted", error_code=-2, error_string="hw")
        self.assertFalse(aborted["ready_for_next"])

    def test_roundtrip_json(self):
        payload = make_event(
            EVENT_SUCCEEDED, goal_id="deadbeef", stamp_ns=42
        )
        parsed = parse_event(event_to_json(payload))
        self.assertEqual(parsed["schema"], SCHEMA)
        self.assertEqual(parsed["event"], EVENT_SUCCEEDED)
        self.assertTrue(parsed["ready_for_next"])
        self.assertEqual(parsed["goal_id"], "deadbeef")

    def test_parse_rejects_garbage(self):
        self.assertIsNone(parse_event("{"))
        self.assertIsNone(parse_event('{"event":"succeeded"}'))


if __name__ == "__main__":
    unittest.main()
