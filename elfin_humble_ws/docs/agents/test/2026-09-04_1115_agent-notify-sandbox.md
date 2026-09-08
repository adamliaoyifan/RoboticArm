# 2026-09-04 — agent_notify.sh sandbox

- role: test
- agent: cursor
- model: unknown
- cli: cursor
- status: done

## Summary

This Cursor session is the test agent. Session-start mailbox was empty.
Sandbox-ran `scripts/agent_notify.sh` against the documented CLI (multiline
`--question`, two `--pointer`s, `AGENT_COORD_ROOT`). First notify matches the
spec. Follow-up `--thread` on an already-open row appends the discuss post
then exits 1 and leaves `OPEN.md` stale. A leftover exported
`AGENT_COORD_ROOT` sent a later notify into `/tmp` instead of the primary
mailbox.

## Commands

- `AGENT_COORD_ROOT=/tmp/elfin_agent_notify_sandbox_* scripts/agent_notify.sh --to eng --from test --cli codex --slug packing-eval-regression --question "..." --pointer ... --pointer ...`
- `scripts/agent_notify.sh --thread 2026-09-04_1115_packing-eval-regression.md --question "follow-up: still YOLO_NOT_READY"`
- `scripts/check_agent_contract.sh`

## Evidence

- Primary mailbox `docs/agents/discuss/OPEN.md` was empty at session start; sandbox copy only.

## Result

- pass: first notify creates `docs/agents/discuss/YYYY-MM-DD_HHMM_<slug>.md` and one `OPEN.md` row (`to=eng`, flattened question).
- pass: `AGENT_COORD_ROOT` writes the mailbox in that root, not the caller cwd.
- pass: `scripts/check_agent_contract.sh`.
- fail: `--thread` on a listed thread is not a clean append; post is written, then `exit 1`, `OPEN.md` question is not updated.
- fail: exported `AGENT_COORD_ROOT` is sticky; a live notify after the sandbox hit `/tmp` (`Q-20260904-3` there), not `docs/agents/discuss/OPEN.md`.

## Pointers

- `scripts/agent_notify.sh`
- `docs/agents/README.md`
- `docs/agents/eng/2026-09-04_1108_agent-notify-helper.md`

## Open

- Eng: follow-up notify should append the thread and refresh or keep the existing `OPEN.md` row without failing after a partial write.
- Eng: refuse or warn if `AGENT_COORD_ROOT` is set but is not the primary workspace; do not silently write a `/tmp` mailbox.
