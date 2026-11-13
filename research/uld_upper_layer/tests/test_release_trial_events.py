"""The real phase recorder must accept named motions and flush their evidence."""
import io
import json
import unittest

from release_trial import record_event


class TrialEventContracts(unittest.TestCase):
    def test_motion_name_payload_does_not_collide_with_event_type(self):
        trace = io.StringIO()
        for phase in ('separate_down', 'withdraw_outward'):
            for event in ('motion_started', 'motion_feedback', 'motion_completed'):
                record_event(trace, event, name=phase, stamp_s=7.154)
        rows = [json.loads(line) for line in trace.getvalue().splitlines()]
        self.assertEqual(len(rows), 6)
        self.assertEqual(rows[0]['event'], 'motion_started')
        self.assertEqual(rows[0]['name'], 'separate_down')
        self.assertEqual(rows[-1]['name'], 'withdraw_outward')
        self.assertEqual(rows[-1]['event'], 'motion_completed')
        self.assertTrue(all(row['stamp_s'] == 7.154 for row in rows))


if __name__ == '__main__':
    unittest.main()
