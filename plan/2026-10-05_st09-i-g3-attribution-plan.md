# ST09-I: generation 2 failure and generation 3 plan

Date: 2026-10-05. Worktree: `/tmp/elfin-st09-converge-i`, branch
`integration/st09-converge-i-g3-fix`, last commit `82a6bc7`.

## Current problem

Generation 2 held-out acceptance (`route_matrix_g2_20261005`, 123
reference-feasible cases) failed the rate gate:

- state/atlas_on: 112/123 (0.9106, Wilson LB 0.8469)
- state/atlas_off: 116/123 (0.9431, Wilson LB 0.8872)
- baseline: 64/123 (0.5203)
- Gate: rate >= 0.95 and Wilson LB >= 0.90. Both state variants fail.
- All other gates were green: slot64 first DESCEND tier 0 with fraction 1.0,
  zero false successes, branch flips, proxy collisions or runs over 10 s.

The joint1 seed (`tool_down_j1_minus_pi`) solved all 63 dev zero-goal
misses. The remaining failures are therefore not missing-goal problems. Two
questions are unresolved:

1. Why do cases that already have a feasible goal still end in
   `SEARCH_INCOMPLETE`?
2. Why does atlas-on solve 4 fewer cases than atlas-off?

Two whole-chain `PLANNING_BUDGET_EXHAUSTED` failures already had an IK goal.
Whether they share a mechanism with the matrix `SEARCH_INCOMPLETE` cases is
unknown, and whether their T2 evidence can be replayed has not been audited.

G3 Run03 stays blocked. No exploratory live sim is scheduled.

## Principles for this generation

- Record objective search state first; derive any classification from the
  data. Do not design instrumentation around a mechanism you have already
  assumed.
- A bounded search that never reaches a goal proves only
  `goal_not_reached`. Do not report "graph disconnected" unless the search
  was exhaustive (the frontier emptied with no cap or budget hit).
- Atlas guarantees are structural: the identity, order and minimum quota of
  the baseline seeds do not change when atlas is turned on. A per-case
  monotonicity gate is required only if a reserved baseline lane is actually
  implemented.
- Replayability is audited, not assumed. Missing fields give
  `not_replayable`, together with the list of missing fields.

## To-do

1. **Freeze generation 2**
   - Mark acceptance as fail on the thread
     `docs/agents/discuss/2026-10-04_1710_st09-i-g3-fix-g2.md`.
   - Add the generation 2 held-out IDs to the retired set.
   - Keep the existing freeze, trace and summary read-only.
   - Never use these held-out cases to choose a fix.

2. **Audit diagnostics before changing behaviour**
   - Inventory the fields that `route_producer.py` and the search modules
     already expose.
   - Add only the missing boundary statistics. Per case:
     `goal_count_by_source`, `attempted_goal_ids`, `frontier_remaining`,
     `unexpanded_count`, `stop_condition`, `failure_cap_state`,
     `elapsed`/`budget`, `edge_rejections_by_reason`.
   - Do not dump individual internal nodes; keep the evidence bounded.

3. **Paired attribution, dev split only**
   - Run each case with atlas off and with atlas on, and locate the first
     state divergence before attaching any label.
   - Answer for each case:
     - Are the baseline seeds identical?
     - Was any baseline goal removed by deduplication?
     - Did the order of baseline goal attempts change?
     - Did atlas seeds consume the candidate cap or the failure cap?
     - Were baseline candidates still untried when the search exited?

4. **Replayability audit of the two whole-chain failures**
   - Check that the T2 bundle contains the full request, map, attachment,
     atlas identity and start joints.
   - If it is complete, replay offline. If not, record `not_replayable` with
     the missing fields.
   - Do not launch G3 to fill evidence gaps. Add the missing boundary capture
     to the next legitimate live run.

5. **One bounded fix, chosen by the measured mechanism**

   | Mechanism | Fix |
   |---|---|
   | Seed crowding | Reserve a baseline lane: keep its identity, order and minimum quota |
   | Deduplication | Fix seed provenance and the equivalence check |
   | Failure cap | Separate the quota by source (baseline vs atlas) |
   | Frontier not finished | Fix scheduling fairness |

   Unchanged: total budget, IK/collision tolerances, 0.01 rad retreat gate.

6. **Dev acceptance**
   - All 63 zero-goal regressions still succeed.
   - No case in the atlas-off success set is lost under atlas-on because of
     atlas quota consumption.
   - slot64 still starts at tier 0 with fraction 1.0.
   - The focused route tests are green.
   - Run `scripts/st09_i/run_bi_route_tests.sh` once, as the final regression
     only, not in every diagnostic loop.

7. **New generation (g3) held-out**
   - New salt.
   - Exclude every held-out ID from generations 1 and 2.
   - At least 53 reference-feasible cases; the case set does not change after
     the freeze.
   - Score once. Pass only if rate >= 0.95, Wilson LB >= 0.90 and every other
     existing gate is green.

8. **G3 Run03** starts only after every gate above passes.

## Considered and not adopted now

- Exploratory three-box live run in parallel with this work. A live run
  cannot settle the search attribution, it adds sim, controller and timing
  variables, and a lucky pass would not overturn the matrix failure. The
  separate three-box completion plan (which would replace the matrix gate
  with two consecutive full live sessions) is held as an alternative. It
  needs an explicit user decision to replace this plan.
