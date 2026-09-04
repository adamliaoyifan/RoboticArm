# 2026-09-04 — agent role-log workspace

- role: discuss
- status: done

## Summary

Multiple agents share `elfin_humble_ws`. Chat stays in Cursor. Session notes
are split under `docs/agents/{reviews,eng,test,discuss}`. Test evidence stays
in `docs/status/evidence/`; this tree only stores pointers. Agent-started sims
still stop via `scripts/stop_sim.sh`.

## Pointers

- `docs/agents/README.md`
- `.cursor/rules/agent-logs.mdc`
- `.cursor/rules/sim-lifecycle.mdc`
