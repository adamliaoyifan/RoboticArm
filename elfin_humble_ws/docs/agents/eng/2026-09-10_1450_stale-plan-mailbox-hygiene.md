# 2026-09-10 -- Stale plan index and mailbox hygiene

- role: eng
- agent: cursor
- model: grok-4.6
- cli: cursor
- status: done

## Summary

Did not delete superseded threads, evidence, or the misdirect D455 plan
directory: they are still pointed at by lineage and comparisons. Closed two
stale mailbox rows instead, and marked outdated plan headers so they cannot
be read as live dispatch gates. `Q-20260908-2` is `done` (D555, not D455;
work transferred to DSIM/HE). `PF-R6-VERIFY` is `cancelled` with owner
stop-ack because PF-R6 generation 3 already passed. PF-R10 generation 2
remains on `OPEN.md` until `claude/glm-5.3` stop-acks the existing
supersede.

## Pointers

- `docs/agents/discuss/OPEN.md`
- `docs/agents/discuss/2026-09-08_1643_d455-sim-parity-and-min-z-conflict.md`
- `docs/agents/discuss/2026-09-05_1840_pf-r6-grok-validation.md`
- `docs/plans/README.md`
- `docs/plans/pf_f3_depth_primary_contract.md`
- `docs/plans/d455_replace_d435/README.md`
- `docs/plans/d555_replace_d435/README.md`
- `docs/plans/d555_sim_depth_pipeline_execution.md`
