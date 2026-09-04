# Daily and Weekly Progress Polling

- role: eng
- agent: codex
- model: gpt-5
- cli: codex
- status: done
- parent: PROGRESS-POLL-20260904
- subtask: PP-1
- base_revision: 1e8ea460aa7abdce6c7c66dc4020317e777d8747
- revision: working-tree
- started_at: 2026-09-04T20:34:42+08:00
- completed_at: 2026-09-04T20:40:00+08:00

## Requirement

Poll repository progress into a daily document, generate a weekly summary on
Sundays, and include future plans and current blockers without relying on chat
history.

## Summary

Added an idempotent repository progress poller. It generates a daily summary on
every run, generates an ISO weekly summary on Sundays, and reports commits,
agent Results, review decisions, active work, undispatched approved plan rows,
future dependency order, readiness failures, and current blockers. A user
systemd timer installer provides persistent periodic execution.

## Changed

- `scripts/progress_poll.py`
- `scripts/test_progress_poll.py`
- `scripts/install_progress_poll_timer.sh`
- `docs/status/progress_polling.md`
- `docs/status/README.md`
- `docs/status/progress/README.md`
- `docs/status/progress/daily/2026-09-04.md`

## Tests

- `python3 -m py_compile scripts/progress_poll.py scripts/test_progress_poll.py`
- `PYTHONPATH=scripts python3 -m unittest -v scripts/test_progress_poll.py`
- `python3 scripts/progress_poll.py --json`
- `scripts/install_progress_poll_timer.sh --dry-run`
- `git diff --check`
- `scripts/check_agent_contract.sh`

## Result

- pass: daily output generated; Sunday and forced-weekly behavior, plan/blocker
  extraction, and repeat-run idempotency are covered by isolated tests.

## Pointers

- `docs/status/progress_polling.md`
- `docs/status/progress/README.md`
- `docs/status/progress/daily/2026-09-04.md`

## Open

- The systemd unit is provided but is not installed without user approval,
  because installation writes to the user's home configuration and starts a
  background timer.
