- role: eng
- agent: claude
- model: glm-5.3
- cli: claude-code
- status: done
- parent: PLACEMENT-GRID-LAYOUT-20261023
- subtask: COST-CORE
- base_revision: fd8f57a1
- started_at: 2026-10-23T18:33:46+08:00
- completed_at: 2026-10-23T21:30:00+08:00

## Summary

Implemented the G2 pure placement layout cost/ranking core (plan b976fe53,
review docs/agents/reviews/2026-10-23_1834 applied before the DTO freeze):
deep-immutable DTOs consuming the full st09_contracts AttachmentGeometry
and one duck-typed OccupancyMapStore.MapSnapshot, adjacency against the
candidate predicted nominal top, policy-gated future-support/fragmentation
window evaluation bit-identical to the solver oracle (including its frozen
crossed-corner overlap convention), the layer_layout_depth_v1 and verbatim
baseline rankings, and an offline fixtures/benchmark CLI with
pass/fail/not_evaluated exits.  Two real defects (convex window corners;
unrotated-AABB prefilter under-covering rotated boxes) were caught by the
dual independent oracles and fixed; fixtures matrix 540 cases with 201/201
brute-oracle agreement.  Benchmark: 0.05 groups pass the component bounds
(p95 ~2s); 0.02/0.01 single requests 65-2248s exceed the 20s max gate and
stay in offline optimization per G2 §6 with per-candidate cost analysis
recorded -- no enumeration was reduced to pass.

## Requirement

- Plan: /tmp/elfin-placement-cost-dispatch/docs/plans/PLACEMENT_COST_PARALLEL_G2_20261023.md
  at b976fe53 (parent fc7c704b).  Pure offline delivery: DTO contracts first
  (reported for integration consumption), then cost/ranking/fixtures/benchmark;
  no OCC edits, no ROS wiring, no simulation; solver helpers read-only.
- Review: docs/agents/reviews/2026-10-23_1834_cost-core-implementation-review.md
  findings 1-7 all applied before the freeze (full AttachmentGeometry
  consumption, adjacency vs predicted nominal top, full-enumeration
  benchmark with pre/post counts and finite max_candidates, per-layer
  before caches with exposed-plane K, consistent height+state+clearance
  patch, policy-driven gates, pipefail + durable logs, full-format note).
- Base: fd8f57a1 by user instruction (recorded in the thread per the
  generation rules); plan_revision stays b976fe53.

## Changed

- `src/luggage_packing/luggage_packing/placement_layout_contracts.py` (DTO freeze, commit fbe8e3a6)
- `src/luggage_packing/luggage_packing/placement_layout_cost.py`
- `src/luggage_packing/luggage_packing/placement_ranking.py`
- `src/luggage_packing/test/placement_layout_fixtures.py`
- `src/luggage_packing/test/test_placement_layout_contracts.py`
- `src/luggage_packing/test/test_placement_layout_cost.py`
- `src/luggage_packing/test/test_placement_ranking.py`
- `scripts/g3/placement_cost_core_eval.py` (new g3 entry; no package-config change needed)

All on branch agent/eng/placement-cost-g2 (worktree /tmp/elfin-placement-cost-g2),
implementation commit 92455834, clean tree, no other agents' changes carried.

## Verification

- `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q
  src/luggage_packing/test/test_placement_layout_contracts.py
  src/luggage_packing/test/test_placement_layout_cost.py
  src/luggage_packing/test/test_placement_ranking.py`: 41 passed
  (log: evidence pytest_layout.log; launch_testing's collection hook would
  import in-progress S2 files on this base, hence plugin autoload off).
- Baseline: test_placement_solver.py + test_placement_atlas_hook.py +
  test_occ_crop_isolation.py: 45 passed, new failures 0 (pytest_baseline.log).
- `python3 scripts/g3/placement_cost_core_eval.py fixtures --resolutions
  0.05,0.02,0.01 --out .../fixtures`: exit 0; 540/540 cases, brute-oracle
  agreement 201/201.
- `python3 scripts/g3/placement_cost_core_eval.py benchmark --resolutions
  0.01,0.02,0.05 --boxes 0,5,20 --warmup 3 --repeat 30 --out .../benchmark`:
  exit 3; 0.05 groups pass (p95 1.96-2.13s, RSS 0.25GiB), 0.02/0.01 groups
  fail the 20s component max (first requests 64.8-2247.5s, recorded with
  full inputs; sample-count and budget-probe protocol in group JSONs).
- Disabled-mode ranking equals the solver's ranked_feasible_candidates on
  identical inputs (208/208 ids) including constructed band-0/1 ties.
- not run: real snapshot service, ComputePlacementV2 wiring, online
  behavior, installation identity -- integration-owned, not dispatched.

## Result

- pass at `92455834` (DTO freeze fbe8e3a6), plan b976fe53.
- Benchmark component bounds: 3 groups pass at 0.05; 6 groups fail at
  0.02/0.01 and remain in offline optimization per G2 §6 (analysis and
  directions in the evidence REPORT); this task's delivery is complete and
  no gate was satisfied by reducing enumeration.

## Pointers

- Evidence: `docs/status/evidence/occ_owner_b1/placement_cost_core_g2_20261023/`
- Thread: `docs/agents/discuss/2026-10-23_1813_placement-cost-core-g2.md`
- Worktree: `/tmp/elfin-placement-cost-g2` (branch agent/eng/placement-cost-g2)
