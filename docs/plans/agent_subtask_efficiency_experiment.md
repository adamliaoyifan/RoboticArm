# Owner-Closed Subtask Efficiency Experiment

Date: 2026-09-04

## Hypothesis

Assigning each bounded subtask to one concrete agent for requirement reading,
implementation, testing, repair, and closure will reduce queue and context
handoff time without increasing escaped defects or weakening test coverage.

## Old-Flow Baseline

The platform-free-height run is the first baseline observation:

- reviews dispatched eng at 11:43;
- eng reported implementation ready at 12:17 (34 minutes);
- test reported the G0-G6 result at 14:23 (160 minutes from dispatch);
- the evidence run started around 14:12, so approximately 115 minutes elapsed
  between eng-ready and test execution starting;
- G0, G4, and G6 failed and work returned to eng.

These timestamps demonstrate handoff latency but are only one sample. They do
not establish a statistically reliable baseline by themselves.

## New Flow

Reviews reaches requirement consensus with a distinct Codex agent, creates
bounded subtasks, and assigns each to a concrete agent/model. Each owner closes
its own implementation and tests. A final integration subtask depends on all
implementation subtasks and owns the whole-chain regression.

Every owner note records:

- `parent`, `subtask`, `agent`, `model`, and `base_revision`;
- `started_at` and `completed_at` in ISO-8601 local time;
- commands/tests run and pass/fail result;
- blocker time and reason, if any;
- evidence paths and final revision.

Runnable owners must call `scripts/agent_start.sh` before work and
`scripts/agent_complete.sh` to close. The start helper appends a machine-readable
`Claim` event with `started_at`; the complete helper requires an exact Git
commit plus tests for passing work, appends a `Result` event, marks the thread
done, and removes the mailbox row. This gives a scheduler/listener a stable
lifecycle protocol instead of inferring state from prose.

Summarize a parent task at any time with:

```bash
scripts/agent_flow_metrics.py --parent <TASK-ID>
scripts/agent_flow_metrics.py --parent <TASK-ID> --json
```

Incomplete work remains `pending`; unified lead time is emitted only when all
runnable subtask threads are `done` with `outcome: pass`.

## Metrics

| Metric | Definition |
|---|---|
| Unified lead time | first subtask dispatch to integration completion |
| Queue wait | dispatch to owner `started_at` |
| Owner cycle time | owner `started_at` to passing completion |
| Handoff count | ownership changes required to close one subtask |
| Rework | failed owner test runs before pass |
| First-pass integration | integration passes without returning work |
| Escaped defect count | failures first discovered outside owner tests |
| Parallel efficiency | sum of owner cycle times / unified lead time |

## Decision Rule

Run at least three unified tasks with two or more implementation subtasks. The
new flow is considered more efficient when:

- median queue wait decreases by at least 30%;
- median unified lead time decreases by at least 20% for comparable scope;
- routine ownership handoffs are at most one per subtask;
- required-test completion rate does not decrease;
- escaped defects and integration regressions do not increase.

After the first task, report only a directional pilot result. Do not claim an
efficiency improvement until the three-task minimum is met.

## Current Pilot

The in-flight platform-free-height work began under the old workflow, so it is
retained as the baseline rather than counted as a clean new-flow sample. PF-R7
is an explicit dependent integration subtask. The next new unified task is
pilot sample 1.

Interim owner-closed observations from the remediation transition:

- PF-R1 was dispatched at 14:53 and the owner reported implementation plus
  focused/package tests at 14:59: about 6 minutes;
- PF-R2 was dispatched at 14:53 and the same owner reported implementation
  plus perception regression at 15:03: about 10 minutes;
- PF-R3 was dispatched at 14:53 and the owner reported implementation plus
  focused/perception regression at 15:18: about 25 minutes;
- PF-R1, PF-R2, PF-R3, and PF-R4 are formally closed through standard
  `Claim`/`Result` events at revision
  `a001be7855373473b52cd2115cb12954e6c38abf`;
- these subtasks did not wait for a separate test owner;
- current recorded owner cycle times are PF-R1 41.73 min, PF-R2 35.8 min,
  PF-R3 28.8 min, and PF-R4 13.8 min;
- PF-R5 is now dependency-ready; PF-R6 waits for PF-R5;
- PF-A1 and PF-A2 were added as owner-closed audit/readiness subtasks and have
  been claimed but not completed;
- PF-R7 integration is still pending, so no unified lead-time or escaped-defect
  comparison is available yet.

This is a positive directional signal for handoff latency, not evidence that
the overall workflow is more efficient. Apply the three-task decision rule.
