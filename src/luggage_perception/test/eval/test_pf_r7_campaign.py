#!/usr/bin/env python3
"""PF-R7-H1 watchdog, reset, budget, and ledger tests. No ROS."""

from __future__ import division

import json
import os
import tempfile
import unittest

from luggage_perception.eval.pf_r7_campaign import (
    CampaignDriver,
    FakeClock,
    ScriptedBackend,
    default_seed_matrix,
    pidfile_live,
)
from luggage_perception.eval.pf_r7_classifier import (
    CLASS_ELIGIBLE_FAIL,
    CLASS_ELIGIBLE_PASS,
    CLASS_INFRA,
    CLASS_KNOWN_MISS,
)

from pf_r7_fixtures import miss_record, pass_record


def _by_size(record_fn, n_per_size=6):
    return {
        "carryon": [record_fn() for _ in range(n_per_size)],
        "standard": [record_fn() for _ in range(n_per_size)],
        "large": [record_fn() for _ in range(n_per_size)],
    }


def _read_jsonl(path):
    if not os.path.isfile(path):
        return []
    rows = []
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def _run(backend, **kwargs):
    clock = kwargs.pop("clock", FakeClock())
    budgets = kwargs.pop("budgets", None)
    deadlines = kwargs.pop("deadlines", None)
    tmp = kwargs.pop("tmp", None)
    owned_tmp = tmp is None
    if owned_tmp:
        tmp = tempfile.mkdtemp(prefix="pfr7h1_")
    driver = CampaignDriver(
        out_dir=tmp,
        backend=backend,
        budgets=budgets,
        deadlines=deadlines,
        seed_matrix=default_seed_matrix(),
        clock=clock,
        poll_dt=1.0,
        git_commit="60dafb7deee50a6f3a76d48076b743bf3e3e1bc8",
        install_signals=False,
    )
    verdict = driver.run()
    return driver, verdict


class TestThreeSlotPass(unittest.TestCase):
    def test_eighteen_eligible_passes(self):
        backend = ScriptedBackend(records_by_size=_by_size(pass_record, 6))
        driver, verdict = _run(backend)
        self.assertEqual(verdict["outcome"], "pass", verdict)
        self.assertEqual(verdict["n_attempts"], 18)
        rows = _read_jsonl(driver.attempts_path)
        self.assertEqual(len(rows), 18)
        self.assertTrue(all(
            row["attempt_class"] == CLASS_ELIGIBLE_PASS for row in rows))
        self.assertEqual(backend.teardown_calls, 1)
        self.assertEqual(backend.case_reset_calls, 18)
        self.assertFalse(os.path.isfile(driver.miss_path))


class TestMissDoesNotAdvanceOrReset(unittest.TestCase):
    def test_miss_uses_next_same_size_seed(self):
        by_size = _by_size(pass_record, 6)
        by_size["carryon"] = [miss_record()] + by_size["carryon"]
        backend = ScriptedBackend(records_by_size=by_size)
        driver, verdict = _run(backend)
        self.assertEqual(verdict["outcome"], "pass", verdict)
        rows = _read_jsonl(driver.attempts_path)
        self.assertEqual(rows[0]["attempt_class"], CLASS_KNOWN_MISS)
        self.assertEqual(rows[0]["size"], "carryon")
        self.assertEqual(rows[0]["seed_id"], "carryon_00")
        carryon_passes = [
            row for row in rows
            if row["size"] == "carryon"
            and row["attempt_class"] == CLASS_ELIGIBLE_PASS]
        self.assertEqual(len(carryon_passes), 6)
        self.assertEqual(carryon_passes[0]["seed_id"], "carryon_01")
        misses = _read_jsonl(driver.miss_path)
        self.assertEqual(len(misses), 1)
        self.assertEqual(misses[0]["seed_id"], "carryon_00")
        self.assertIn("dump", misses[0])


class TestEligibleFailStops(unittest.TestCase):
    def test_first_fail_stops_without_rerun(self):
        fail = pass_record(mask_join_failed=True)
        by_size = _by_size(pass_record, 6)
        by_size["carryon"] = [fail] + by_size["carryon"]
        backend = ScriptedBackend(records_by_size=by_size)
        driver, verdict = _run(backend)
        self.assertEqual(verdict["outcome"], "fail", verdict)
        self.assertEqual(verdict["reason"], "eligible_fail")
        rows = _read_jsonl(driver.attempts_path)
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["attempt_class"], CLASS_ELIGIBLE_FAIL)
        self.assertEqual(backend.teardown_calls, 1)


class TestBudgets(unittest.TestCase):
    def test_three_consecutive_size_exclusions_stop(self):
        by_size = {
            "carryon": [miss_record(), miss_record(), miss_record()],
            "standard": [pass_record()],
            "large": [pass_record()],
        }
        backend = ScriptedBackend(records_by_size=by_size)
        driver, verdict = _run(backend)
        self.assertEqual(verdict["outcome"], "inconclusive", verdict)
        self.assertEqual(
            verdict["reason"],
            "inconclusive/perception_availability_blocked")
        rows = _read_jsonl(driver.attempts_path)
        self.assertEqual(len(rows), 3)
        self.assertTrue(all(
            row["attempt_class"] == CLASS_KNOWN_MISS for row in rows))

    def test_slot_exclusion_budget(self):
        by_size = {
            "carryon": [miss_record() for _ in range(6)],
            "standard": [],
            "large": [],
        }
        backend = ScriptedBackend(records_by_size=by_size)
        driver, verdict = _run(backend, budgets={
            "slots": 1,
            "eligible_per_size": 2,
            "max_attempts_per_slot": 12,
            "max_exclusions_per_slot": 6,
            "max_consecutive_size_exclusions": 10,
            "max_stack_resets": 2,
            "max_attempts_campaign": 36,
        })
        self.assertEqual(verdict["outcome"], "inconclusive")
        self.assertEqual(len(_read_jsonl(driver.attempts_path)), 6)

    def test_campaign_attempt_budget(self):
        by_size = {
            "carryon": [miss_record() for _ in range(4)],
            "standard": [miss_record() for _ in range(4)],
            "large": [miss_record() for _ in range(4)],
        }
        backend = ScriptedBackend(records_by_size=by_size)
        driver, verdict = _run(backend, budgets={
            "slots": 3,
            "eligible_per_size": 2,
            "max_attempts_per_slot": 12,
            "max_exclusions_per_slot": 12,
            "max_consecutive_size_exclusions": 10,
            "max_stack_resets": 2,
            "max_attempts_campaign": 3,
        })
        self.assertEqual(verdict["outcome"], "inconclusive")
        self.assertLessEqual(verdict["n_attempts"], 3)

    def test_time_budget(self):
        backend = ScriptedBackend(records_by_size=_by_size(pass_record, 6))
        driver, verdict = _run(
            backend,
            clock=FakeClock(start=0.0),
            deadlines={"campaign_sec": 0.0},
        )
        self.assertEqual(verdict["outcome"], "inconclusive")
        self.assertEqual(verdict["reason"], "inconclusive/time_budget")


class TestResetAndWatchdog(unittest.TestCase):
    def test_case_reset_increments_epoch_and_logs(self):
        backend = ScriptedBackend(records_by_size=_by_size(pass_record, 6))
        driver, verdict = _run(backend)
        self.assertEqual(verdict["outcome"], "pass")
        resets = _read_jsonl(driver.reset_path)
        case_resets = [row for row in resets if row.get("kind") == "case"]
        self.assertEqual(len(case_resets), 18)
        self.assertEqual(case_resets[-1]["epoch"], 18)
        self.assertEqual(case_resets[-1]["stale_frames"], 0)
        self.assertTrue(case_resets[-1]["box_absent"])

    def test_duplicate_clock_stack_reset_budget(self):
        infra = miss_record(n_clock_publishers=2, duplicate_clock=True)
        by_size = {
            "carryon": [infra, infra, infra],
            "standard": [pass_record()],
            "large": [pass_record()],
        }
        backend = ScriptedBackend(records_by_size=by_size)
        driver, verdict = _run(backend, budgets={
            "slots": 1,
            "eligible_per_size": 2,
            "max_attempts_per_slot": 12,
            "max_exclusions_per_slot": 12,
            "max_consecutive_size_exclusions": 10,
            "max_stack_resets": 2,
            "max_attempts_campaign": 36,
        })
        self.assertEqual(verdict["outcome"], "inconclusive")
        self.assertEqual(
            verdict["reason"], "inconclusive/infrastructure_blocked")
        self.assertEqual(backend.stack_reset_calls, 2)
        self.assertGreaterEqual(backend.stop_sim_calls, 2)
        resets = _read_jsonl(driver.reset_path)
        self.assertEqual(
            sum(1 for row in resets if row.get("kind") == "stack"), 2)

    def test_no_progress_watchdog_classifies_snapshot(self):
        backend = ScriptedBackend(
            hang_observe=True,
            snapshot=miss_record(n_clock_publishers=2, duplicate_clock=True),
        )
        driver, verdict = _run(
            backend,
            clock=FakeClock(),
            deadlines={"no_progress_sec": 5.0, "observe_sec": 8.0,
                       "campaign_sec": 2700.0, "precheck_sec": 60.0,
                       "case_reset_sec": 20.0, "slot_sec": 600.0},
            budgets={
                "slots": 1,
                "eligible_per_size": 2,
                "max_attempts_per_slot": 3,
                "max_exclusions_per_slot": 3,
                "max_consecutive_size_exclusions": 3,
                "max_stack_resets": 2,
                "max_attempts_campaign": 3,
            },
        )
        rows = _read_jsonl(driver.attempts_path)
        self.assertGreaterEqual(len(rows), 1)
        self.assertEqual(rows[0]["attempt_class"], CLASS_INFRA)

    def test_signal_abort_flushes_unscored(self):
        tmp = tempfile.mkdtemp(prefix="pfr7h1_abort_")
        holder = []

        class AbortBackend(ScriptedBackend):
            def poll_observe(self, case):
                holder[0].request_abort()
                return {"record": pass_record(), "progress": True}

        backend = AbortBackend(records_by_size=_by_size(pass_record, 6))
        driver = CampaignDriver(
            out_dir=tmp,
            backend=backend,
            clock=FakeClock(),
            poll_dt=1.0,
            git_commit="60dafb7deee50a6f3a76d48076b743bf3e3e1bc8",
        )
        holder.append(driver)
        verdict = driver.run()
        self.assertEqual(verdict["outcome"], "inconclusive")
        self.assertEqual(verdict["reason"], "inconclusive/signal_abort")
        rows = _read_jsonl(driver.attempts_path)
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["attempt_class"], CLASS_INFRA)
        self.assertIn("signal_abort", rows[0]["reasons"])
        manifest = os.path.join(rows[0]["dump"], "attempt_manifest.json")
        self.assertTrue(os.path.isfile(manifest))
        self.assertNotEqual(rows[0]["attempt_class"], CLASS_ELIGIBLE_PASS)
        self.assertNotEqual(rows[0]["attempt_class"], CLASS_ELIGIBLE_FAIL)

    def test_missing_dump_flush_is_fail_closed(self):
        backend = ScriptedBackend(
            records_by_size=_by_size(pass_record, 6),
            flush_writes=False,
        )
        driver, verdict = _run(backend, budgets={
            "slots": 1,
            "eligible_per_size": 2,
            "max_attempts_per_slot": 4,
            "max_exclusions_per_slot": 4,
            "max_consecutive_size_exclusions": 4,
            "max_stack_resets": 2,
            "max_attempts_campaign": 4,
        })
        self.assertEqual(verdict["outcome"], "inconclusive")
        rows = _read_jsonl(driver.attempts_path)
        self.assertGreaterEqual(len(rows), 1)
        self.assertEqual(rows[0]["attempt_class"], "evidence_invalid")

    def test_occupied_sim_does_not_launch(self):
        backend = ScriptedBackend(
            occupied=True, records_by_size=_by_size(pass_record, 1))
        driver, verdict = _run(backend)
        self.assertEqual(verdict["outcome"], "inconclusive")
        self.assertEqual(verdict["reason"], "inconclusive/sim_slot_busy")
        self.assertEqual(verdict["n_attempts"], 0)
        self.assertEqual(backend.teardown_calls, 0)
        self.assertEqual(backend.spawned, [])

    def test_stale_reset_triggers_stack_reset_not_pass_rewrite(self):
        backend = ScriptedBackend(
            records_by_size={"carryon": [pass_record()],
                             "standard": [pass_record()],
                             "large": [pass_record()]},
            stale_after_reset=3,
        )
        driver, verdict = _run(backend, budgets={
            "slots": 1,
            "eligible_per_size": 1,
            "max_attempts_per_slot": 6,
            "max_exclusions_per_slot": 6,
            "max_consecutive_size_exclusions": 6,
            "max_stack_resets": 2,
            "max_attempts_campaign": 6,
        })
        rows = _read_jsonl(driver.attempts_path)
        self.assertEqual(rows[0]["attempt_class"], CLASS_ELIGIBLE_PASS)
        self.assertGreaterEqual(backend.stack_reset_calls, 1)


class TestSimSlotHelper(unittest.TestCase):
    def test_missing_pidfile_is_not_live(self):
        self.assertFalse(pidfile_live("/tmp/pfr7h1_missing.pid"))


if __name__ == "__main__":
    unittest.main()
