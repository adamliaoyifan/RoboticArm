# 2026-10-23 -- Placement cost core G2: immutable contracts and offline ranking

- status: done
- to_role: eng
- to_agent: claude
- to_model: glm-5.3
- kind: subtask
- parent: PLACEMENT-GRID-LAYOUT-20261023
- subtask: COST-CORE
- depends_on: none
- revision: b976fe53
- generation: 2
- plan_revision: b976fe53
- dispatch_ready: yes

## Post -- reviews/codex -- 2026-10-23 18:13 -- codex/gpt-5

User authorized dispatch to Claude. Exact approved scope and acceptance: docs/plans/PLACEMENT_COST_PARALLEL_G2_20261023.md at b976fe53 (parent plan fc7c704b). Create independent worktree /tmp/elfin-placement-cost-g2 on agent/eng/placement-cost-g2 from the dispatch base. Claim via primary workspace scripts; first commit DTO contract/tests and report that hash, then cost/ranking/fixtures/benchmark. Preserve OCC-CROP ownership. No writable public fields, no mutable input aliases, no cross-request cache in G2. Final integration remains Codex-owned and not dispatched. Read the plan before implementing, complete all specified offline gates and commit evidence; no simulation authorization. Existing registry route is claude/glm-5.3; a different model must obtain an explicit routing update rather than claim under another identity.

## Pointers

- `/tmp/elfin-placement-cost-dispatch/docs/plans/PLACEMENT_COST_PARALLEL_G2_20261023.md`
- `docs/plans/PLACEMENT_GRID_LAYOUT_COST_20261023.md`

## Open

- Implement pure placement cost/ranking and immutable DTOs under G2; own tests and benchmarks; no OCC edits, ROS wiring, deployment or simulation.

## Claim -- eng/claude -- 2026-10-23 18:33 -- claude-code/glm-5.3

- started_at: 2026-10-23T18:33:46+08:00
- claimed_generation: 2
- claimed_plan_revision: b976fe53
- claimed_dependencies: none


## Reply -- eng/claude -- 2026-10-23 18:43 -- claude-code/glm-5.3

- base_change: implementation worktree /tmp/elfin-placement-cost-g2 on branch agent/eng/placement-cost-g2 is built from fd8f57a1 per user instruction after review docs/agents/reviews/2026-10-23_1834_cost-core-implementation-review.md. Dispatch plan_revision stays b976fe53; product base adds the S1 GRID + OCC-CROP lineage (fc7c704b..fd8f57a1) beyond the dispatch base product line b7ac4371. Task scope unchanged.
- review_accepted: findings 1-7 will be applied before DTO freeze: consume full st09_contracts AttachmentGeometry (binding digest + both transforms), adjacency against candidate predicted nominal top, full-enumeration benchmark with pre/post truncation counts and finite max_candidates, per-layer before caches with K as exposed usable plane, consistent local height+state+spacing patch, policy-driven support/clearance gates, pipefail + durable full pytest logs, full-format role note.

## Reply -- eng/claude -- 2026-10-23 19:02 -- claude-code/glm-5.3

- dto_commit: fbe8e3a6 on agent/eng/placement-cost-g2 (review rev2 applied before freeze). PlacementPayload consumes the full st09_contracts AttachmentGeometry with a deterministic freeze_geometry reconstruction check; predicted_nominal_pose delegates to box_pose_at_conservative. Scene snapshots freeze one upstream MapSnapshot duck-typed with expected-identity cross-checks; gate mirrors live in PlacementLayoutPolicy (no hardcoded 0.6/0.05 at call sites). New tests: 17 contracts cases green; baseline solver/atlas/occ-crop 45 green, new failures 0. Integration consumers: build DTOs only via these create/from_* entries; report needed changes through this owner.
## Result -- eng/claude -- 2026-10-23 21:20 -- claude-code/glm-5.3

- outcome: pass
- completed_at: 2026-10-23T21:20:07+08:00
- revision: 92455834626a44a7b1ce760183ca9b19b4ed1bc7
- tests: layout contracts/cost/ranking 41 passed; solver+atlas+occ-crop baseline 45 passed, new failures 0; fixtures 540 cases, brute-oracle agreement 201/201 (exit 0); benchmark exit 3: 0.05 groups pass p95 1.96-2.13s RSS 0.25GiB, 0.02/0.01 groups exceed the 20s component max (first requests 64.8-2247.5s) and remain in offline optimization per G2 section 6, no enumeration reduced
- summary: COST-CORE delivered at 92455834 (DTO freeze fbe8e3a6, base fd8f57a1 per recorded base change): immutable DTOs over full AttachmentGeometry + one duck-typed MapSnapshot, adjacency against the candidate predicted nominal top, policy-gated F/P window evaluation bit-identical to the solver oracle (frozen crossed-corner convention; two real defects caught by dual independent oracles and fixed), layer_layout_depth_v1 plus verbatim baseline ranking, offline fixtures/benchmark CLI with pass/fail/not_evaluated exits. Integration consumers build DTOs only via the create/from_* entries.
- evidence: docs/status/evidence/occ_owner_b1/placement_cost_core_g2_20261023


## Rework Claim -- eng/claude -- 2026-10-24 00:24 -- claude-code/glm-5.3

- started_at: 2026-10-24T00:24:00+08:00
- reason: delivery verdict docs/agents/reviews/2026-10-24_0000_cost-core-delivery-verdict.md rejected the acceptance; user authorized a bounded correction round for the same owner. Per user instruction this round records per-stage timings ONLY (snapshot acquisition -> enumeration -> hard gates -> adjacency A -> after-map future windows/fragmentation F/P (shared segment) -> layer books -> ranking, with input/rejection/window-scan/cache-hit counters) on real 0/5/20-box loads; no functional optimization (deferred until root cause is located); chamfer hull acceptance stays open and is not attempted this round.

## Reply -- eng/claude -- 2026-10-24 19:40 -- claude-code/glm-5.3

- rev3_complete: measurement-only round delivered at f3029889 (books-split 865fdb8b and accounting fix 5135ba7f are subsequent-run code, explicitly not used to interpret this batch). Timing table 9/9 groups on REAL 0/5/20-box loads (placed==requested asserted), one cold run per group; fixtures re-verified 540 cases / brute-oracle 201/201 with instrumentation active; cost_digest equality pre/post instrumentation verified on three input classes only (empty, 5-degree neighbour, platform) -- not extrapolated.
- closure_finding: the sum-vs-total check exposed an instrumentation double count (repaint interval in both books_s and other_s); evidence ANALYSIS.md corrects attribution analytically (unattributed <=2.7%, other is a residual bucket), code fixed for future runs.
- root_cause_data: adjacency <=0.5s everywhere (not a bottleneck); after-window rescan 60-72% and coverage-books 28-40% at 0.01; books_s tracks paint_blocks at a constant ~2.0-2.5us/block across all resolutions and loads (locating clue, not causal proof); per-candidate after cost ~constant per resolution, total driven by valid-candidate count; interior memo hit rate 1.1% at 0.01x0, 0 with boxes.
- standing_acceptance: implementation delivered, executed tests pass (43), component performance FAILED (0.02/0.01 exceed the 20s max gate: 52-2277s), chamfer hull acceptance NOT done, load matrix NOW measured (timing table; sampled benchmark gates still not re-run), online enable FORBIDDEN. Optimization candidates ranked in ANALYSIS.md section 4 (coverage-book rectangular add/sub -> 2D difference equivalent replacement first); actual gains require a pre/post same-input comparison in an authorized optimization round. No optimization, no solver change, no scoring-semantics change performed this round.
