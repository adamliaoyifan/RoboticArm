# Dispatch Ready Gate

- date: 2026-09-05 16:13
- role: eng
- agent: codex
- model: gpt-5
- cli: codex
- task: dispatch-ready-gate
- status: done

## Summary

Added a reviewer-owned `dispatch_ready` thread metadata gate for runnable
mailbox rows. New runnable notifications default to `dispatch_ready: no`;
`agent_start.sh`, `agent_poll_self.py`, and `agent_scheduler.py` keep those
rows waiting until reviewers explicitly update the thread to
`dispatch_ready: yes`. Legacy rows without the field remain compatible until
the live mailbox migration is complete.

## Pointers

- `scripts/agent_notify.sh`
- `scripts/agent_mailbox.py`
- `scripts/agent_poll_self.py`
- `scripts/agent_scheduler.py`
- `scripts/test_agent_mailbox_freshness.sh`
- `scripts/test_agent_scheduler_poller_freshness.sh`
