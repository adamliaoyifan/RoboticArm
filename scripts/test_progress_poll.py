#!/usr/bin/env python3
"""Tests for the deterministic project progress poller."""

from __future__ import annotations

import os
import subprocess
import tempfile
import unittest
from datetime import date
from pathlib import Path

import progress_poll


OPEN_HEADER = """# Open cross-agent work

| id | kind | parent | subtask | depends_on | revision | to_role | to_agent | to_model | from_role | from_agent | from_model | cli | thread | request |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
"""


class ProgressPollTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        (self.root / "docs/agents/discuss").mkdir(parents=True)
        (self.root / "docs/agents/reviews").mkdir(parents=True)
        (self.root / "docs/status").mkdir(parents=True)
        self._git("init", "-q")
        self._git("config", "user.email", "test@example.com")
        self._git("config", "user.name", "Progress Test")

    def tearDown(self):
        self.temp.cleanup()

    def _git(self, *args, env=None):
        subprocess.run(
            ["git", "-C", str(self.root), *args],
            check=True,
            capture_output=True,
            env=env,
        )

    def _commit(self, subject, timestamp):
        marker = self.root / "marker.txt"
        marker.write_text(subject + "\n", encoding="utf-8")
        self._git("add", "marker.txt")
        env = os.environ.copy()
        env["GIT_AUTHOR_DATE"] = timestamp
        env["GIT_COMMITTER_DATE"] = timestamp
        self._git("commit", "-q", "-m", subject, env=env)

    def _write_fixtures(self):
        completed = self.root / "docs/agents/discuss/completed.md"
        completed.write_text(
            """# Completed

- status: done
- parent: DEMO
- subtask: DONE-1

## Result -- eng/codex -- 2026-09-06 10:00 -- codex/gpt-5

- outcome: pass
- completed_at: 2026-09-06T10:00:00+08:00
- revision: abc123
- tests: 4 passed
- summary: Finished the deterministic fixture.
""",
            encoding="utf-8",
        )
        readiness = self.root / "docs/agents/discuss/readiness.md"
        readiness.write_text(
            """# Readiness audit

- status: done
- parent: DEMO
- subtask: AUDIT-1

## Result -- test/cursor -- 2026-09-06 10:30 -- cursor/grok-4.6

- outcome: pass
- completed_at: 2026-09-06T10:30:00+08:00
- revision: def456
- tests: probe passed
- summary: audit_outcome=pass; gt_readiness=blocked by unsafe fallback.
""",
            encoding="utf-8",
        )
        waiting = self.root / "docs/agents/discuss/waiting.md"
        waiting.write_text(
            """# Waiting

- status: open
- parent: DEMO
- subtask: NEXT-2

## Open

- Blocked until DONE-2 passes.
""",
            encoding="utf-8",
        )
        active = self.root / "docs/agents/discuss/active.md"
        active.write_text(
            """# Active

- status: open
- parent: DEMO
- subtask: ACTIVE-1

## Open

- Execute the assigned audit.

## Claim -- test/cursor -- 2026-09-06 11:00 -- cursor/grok-4.6

- started_at: 2026-09-06T11:00:00+08:00
""",
            encoding="utf-8",
        )
        (self.root / "docs/agents/discuss/OPEN.md").write_text(
            OPEN_HEADER
            + "| Q-20260906-1 | subtask | DEMO | ACTIVE-1 | none | abc | test | cursor | grok-4.6 | reviews | codex | gpt-5 | codex | active.md | Run active work. |\n"
            + "| Q-20260906-2 | subtask | DEMO | NEXT-2 | DONE-2 | abc | eng | codex | gpt-5 | reviews | codex | gpt-5 | codex | waiting.md | Run after dependency. |\n",
            encoding="utf-8",
        )
        (self.root / "docs/agents/reviews/2026-09-06_1200_decision.md").write_text(
            """# Decision

- role: reviews
- status: done

## Summary

Approved the fixture boundary.
""",
            encoding="utf-8",
        )
        (self.root / "docs/plans").mkdir(parents=True)
        (self.root / "docs/plans/demo.md").write_text(
            """# Demo Plan

Parent task: `DEMO`

| ID | Owner | Depends on | Scope |
|---|---|---|---|
| DONE-1 | `codex/gpt-5` | none | completed |
| NEXT-2 | `codex/gpt-5` | DONE-2 | open |
| FUTURE-3 | `cursor/grok-4.6` | DONE-1 | later |
""",
            encoding="utf-8",
        )

    def test_sunday_generates_daily_weekly_and_is_idempotent(self):
        self._write_fixtures()
        self._commit("Sunday progress", "2026-09-06T09:00:00+08:00")
        output = self.root / "docs/status/progress"

        first = progress_poll.poll_once(self.root, output, date(2026, 9, 6))
        self.assertTrue(first.daily_changed)
        self.assertTrue(first.weekly_changed)
        daily = (output / "daily/2026-09-06.md").read_text(encoding="utf-8")
        weekly = (output / "weekly/2026-W36.md").read_text(encoding="utf-8")
        self.assertIn("`DONE-1` (pass, `abc123`)", daily)
        self.assertIn("`ACTIVE-1`", daily)
        self.assertIn("Blocked until DONE-2 passes.", daily)
        self.assertIn("`AUDIT-1` readiness", daily)
        self.assertIn("Planned, not dispatched: `FUTURE-3`", daily)
        self.assertIn("Approved the fixture boundary.", daily)
        self.assertIn("Sunday progress", weekly)

        second = progress_poll.poll_once(self.root, output, date(2026, 9, 6))
        self.assertFalse(second.daily_changed)
        self.assertFalse(second.weekly_changed)
        self.assertFalse(second.index_changed)

    def test_non_sunday_skips_weekly_unless_forced(self):
        (self.root / "docs/agents/discuss/OPEN.md").write_text(
            OPEN_HEADER, encoding="utf-8"
        )
        self._commit("Monday progress", "2026-09-07T09:00:00+08:00")
        output = self.root / "docs/status/progress"

        normal = progress_poll.poll_once(self.root, output, date(2026, 9, 7))
        self.assertIsNone(normal.weekly_path)
        forced = progress_poll.poll_once(
            self.root, output, date(2026, 9, 7), force_weekly=True
        )
        self.assertEqual(forced.weekly_path, "docs/status/progress/weekly/2026-W37.md")

    def test_period_bounds_use_monday_through_next_monday(self):
        start, end = progress_poll.period_bounds(date(2026, 9, 6), weekly=True)
        self.assertEqual(start.date(), date(2026, 8, 31))
        self.assertEqual(end.date(), date(2026, 9, 7))


if __name__ == "__main__":
    unittest.main()
