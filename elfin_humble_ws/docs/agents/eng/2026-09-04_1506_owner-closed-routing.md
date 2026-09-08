# 2026-09-04 -- Owner-closed routing implementation

- role: eng
- agent: codex
- model: gpt-5
- cli: codex
- status: done
- parent: AGENT-WORKFLOW-20260904
- subtask: WF-2-WF-3
- base_revision: 0674f84-working-tree
- started_at: 2026-09-04T14:47:00+08:00
- completed_at: 2026-09-04T15:13:56+08:00

## Summary

Replaced checkpoint role handoffs with machine-readable, owner-closed subtask
routing. Added concrete ownership and dependency validation, Codex consensus
routing, standardized Claim/Result events, and a timing metrics summarizer.

## Changed

- `AGENTS.md`
- `.cursor/rules/agent-logs.mdc`
- `docs/agents/README.md`
- `docs/agents/discuss/OPEN.md`
- `docs/agents/{reviews,eng,test}/README.md`
- `docs/plans/agent_subtask_efficiency_experiment.md`
- `scripts/agent_complete.sh`
- `scripts/agent_notify.sh`
- `scripts/agent_flow_metrics.py`
- `scripts/check_agent_contract.sh`

## Verification

- `bash -n scripts/agent_notify.sh scripts/check_agent_contract.sh`: pass.
- `python3 -m py_compile scripts/agent_flow_metrics.py`: pass.
- `scripts/check_agent_contract.sh`: pass.
- Consensus/subtask routing smoke: valid routes created expected metadata;
  self-consensus and non-concrete subtask owners were rejected.
- Synthetic metrics smoke: two parallel subtasks produced queue waits 2/1
  minutes, cycles 10/20 minutes, lead time 21 minutes, parallel factor 1.429.
- Completion smoke: verified owner identity, resolved a real Git commit,
  appended a passing Result, marked the thread done, and removed its row.

## Requirement

- Reviews only reaches consensus, decomposes, and assigns work.
- A concrete owner handles requirement understanding, implementation, tests,
  failure repair, evidence, and closure for each subtask.
- Final whole-chain verification is an explicit dependent integration subtask.
- Efficiency conclusions use measured queue/lead/quality data.

## Result

- pass: workflow contract, routing checks, and metrics calculations pass.

## Pointers

- `docs/agents/reviews/2026-09-04_1452_owner-closed-subtask-flow.md`
- `docs/plans/agent_subtask_efficiency_experiment.md`
