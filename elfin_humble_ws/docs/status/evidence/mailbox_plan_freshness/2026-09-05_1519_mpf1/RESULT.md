# MPF-1 Evidence

- task: MPF-20260905 / MPF-1
- role: eng
- agent: codex
- model: gpt-5
- cli: codex
- started_at: 2026-09-05T15:19:10+08:00
- outcome: pass

## Summary

Implemented generation-aware mailbox lifecycle helpers for notify, start,
complete, supersede/cancel, stop acknowledgement, and migration. Focused
temp-repo tests cover normal owner flow, missing generation rejection,
row/thread mismatch, claimed supersede, cancellation, stale dependency
completion rejection, serialized concurrent transitions, positive migration,
idempotent migration, and ambiguous migration fail-closed behavior.

## Verification

- pass: `scripts/test_agent_mailbox_freshness.sh`
- pass: `scripts/test_agent_lifecycle_smoke.sh`
- pass: `python3 -m py_compile scripts/agent_mailbox.py scripts/agent_scheduler.py scripts/agent_poll_self.py scripts/progress_poll.py`
- pass: `bash -n scripts/agent_notify.sh scripts/agent_start.sh scripts/agent_complete.sh scripts/check_agent_contract.sh scripts/test_agent_lifecycle_smoke.sh scripts/test_agent_mailbox_freshness.sh`
- pass: `git diff --check -- scripts/agent_mailbox.py scripts/agent_notify.sh scripts/agent_start.sh scripts/agent_complete.sh scripts/check_agent_contract.sh scripts/test_agent_lifecycle_smoke.sh scripts/test_agent_mailbox_freshness.sh`
- known pre-existing failure: `scripts/check_agent_contract.sh` still reports the two previously exposed eng note formatting defects.

## Logs

- `test_agent_mailbox_freshness.log`
- `test_agent_lifecycle_smoke.log`
- `py_compile.log`
- `bash_syntax.log`
- `diff_check.log`
- `check_agent_contract.log`
