"""Concurrency tests for exclusive trajectory goal ownership."""

import threading
import unittest

from elfin_trajectory_executor.goal_ownership import SingleGoalOwner


class SingleGoalOwnerTest(unittest.TestCase):
    def test_exactly_one_of_eight_contenders_wins_100_rounds(self):
        for round_index in range(100):
            owner = SingleGoalOwner()
            barrier = threading.Barrier(8)
            winners = []
            winners_lock = threading.Lock()

            def contend(index):
                barrier.wait()
                if owner.try_reserve():
                    with winners_lock:
                        winners.append(index)

            threads = [
                threading.Thread(target=contend, args=(i,)) for i in range(8)
            ]
            for thread in threads:
                thread.start()
            for thread in threads:
                thread.join(timeout=2.0)

            self.assertTrue(
                all(not thread.is_alive() for thread in threads), round_index)
            self.assertEqual(len(winners), 1, (round_index, winners))
            goal_id = "round-%s-owner-%s" % (round_index, winners[0])
            self.assertTrue(owner.bind(goal_id))
            self.assertFalse(owner.cancel("not-the-owner"))
            cancel_event = owner.cancel_event(goal_id)
            self.assertIsNotNone(cancel_event)
            self.assertFalse(cancel_event.is_set())
            self.assertTrue(owner.cancel(goal_id))
            self.assertTrue(cancel_event.is_set())
            self.assertTrue(owner.release(goal_id))

            self.assertTrue(owner.try_reserve())
            next_goal_id = goal_id + "-next"
            self.assertTrue(owner.bind(next_goal_id))
            next_event = owner.cancel_event(next_goal_id)
            self.assertIsNotNone(next_event)
            self.assertFalse(next_event.is_set())
            self.assertTrue(owner.release(next_goal_id))

    def test_pending_reservation_releases_only_before_binding(self):
        owner = SingleGoalOwner()
        self.assertTrue(owner.try_reserve())
        self.assertTrue(owner.release_pending())
        self.assertFalse(owner.busy)

        self.assertTrue(owner.try_reserve())
        self.assertTrue(owner.bind("active"))
        self.assertFalse(owner.release_pending())
        self.assertTrue(owner.busy)
        self.assertTrue(owner.release("active"))

    def test_cancel_can_bind_before_handle_callback_runs(self):
        owner = SingleGoalOwner()
        self.assertTrue(owner.try_reserve())
        self.assertTrue(owner.cancel("accepted-before-bind"))
        self.assertTrue(owner.bind("accepted-before-bind"))
        event = owner.cancel_event("accepted-before-bind")
        self.assertIsNotNone(event)
        self.assertTrue(event.is_set())
        self.assertTrue(owner.release("accepted-before-bind"))


if __name__ == "__main__":
    unittest.main()
