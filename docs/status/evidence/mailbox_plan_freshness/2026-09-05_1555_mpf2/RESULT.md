# MPF-2 Evidence

- task: MPF-20260905 / MPF-2
- role: eng
- agent: codex
- model: gpt-5
- cli: codex
- started_at: 2026-09-05T15:55:13+08:00
- outcome: pass

## Summary

Implemented scheduler and poller freshness behavior for stopped generations
and highest-generation dependency selection. Polling now reports stopped
claimed work before active or ready work, so `--claim-next` cannot claim a
replacement while the old stopped generation is unacknowledged. Scheduler
planning now emits live Codex stop actions, file-only stop visibility, stop
notice lease idempotency, replacement blocking until acknowledgement, and
dependency readiness based on the highest generation in a lineage.

## Verification

- pass: `scripts/test_agent_scheduler_poller_freshness.sh`
- pass: `scripts/test_agent_mailbox_freshness.sh`
- pass: `scripts/test_agent_lifecycle_smoke.sh`
- pass: `python3 -m py_compile scripts/agent_mailbox.py scripts/agent_scheduler.py scripts/agent_poll_self.py scripts/progress_poll.py`
- pass: `bash -n scripts/agent_notify.sh scripts/agent_start.sh scripts/agent_complete.sh scripts/check_agent_contract.sh scripts/test_agent_lifecycle_smoke.sh scripts/test_agent_mailbox_freshness.sh scripts/test_agent_scheduler_poller_freshness.sh`
- pass: `git diff --check -- scripts/agent_scheduler.py scripts/agent_poll_self.py scripts/check_agent_contract.sh scripts/test_agent_scheduler_poller_freshness.sh`
- pass: `scripts/check_agent_contract.sh`

## Logs

- `test_agent_scheduler_poller_freshness.log`
- `test_agent_mailbox_freshness.log`
- `test_agent_lifecycle_smoke.log`
- `py_compile.log`
- `bash_syntax.log`
- `diff_check.log`
- `check_agent_contract.log`
