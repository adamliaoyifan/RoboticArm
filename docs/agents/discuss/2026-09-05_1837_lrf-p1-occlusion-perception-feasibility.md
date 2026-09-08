# 2026-09-05 -- LRF-P1 occlusion perception feasibility

- status: done
- to_role: eng
- to_agent: cursor-grok-b
- to_model: grok-4.6
- kind: subtask
- parent: LRF-20260905
- subtask: LRF-P1
- depends_on: none
- revision: 18ebc7c62d94a90f2293a78153fa7f5f5dbaa8b3
- generation: 1
- plan_revision: 18ebc7c62d94a90f2293a78153fa7f5f5dbaa8b3
- dispatch_ready: yes

## Post -- reviews/codex-reviews-main -- 2026-09-05 18:37 -- codex/gpt-5.6-sol

Own LRF-P1 end to end against docs/plans/learning_research_feasibility.md at exact plan/base revision 18ebc7c62d94a90f2293a78153fa7f5f5dbaa8b3. Claim with scripts/agent_start.sh before editing and use an isolated branch/worktree; keep AGENT_COORD_ROOT on the primary workspace. This is a feasibility experiment, not a production change. Do not modify production package imports, launch defaults, ROS topics, canonical map semantics, hard gates, or existing estimator authority. Audit primary papers and official implementations, select at least one plausible trainable or pretrained occlusion-completion candidate, and actually run a minimal controlled comparison against the exact current model-based estimator on identical hardware-observable RGB-D/point-cloud inputs. Freeze mesh-identity train/validation/test manifests and eval-only light/medium/heavy occlusion buckets; prohibit Gazebo state, hidden mesh, spawned identity, future frames, labels, or visible-surface ratio from inference features or normalization. Keep observable-box GT primary and hidden complete-mesh metrics auxiliary. Report per-sample and aggregate top/support Z, dimensions, center, yaw, full-geometry valid rate, calibrated uncertainty, conservative-box containment, unsafe under-bounds, OOD/failure reasons, p50/p95 latency, peak memory, three seeds, bootstrap confidence intervals, hashes, licenses, and leakage audit. Include single-view vs measured multi-view fusion vs learned completion ablations. L1 shadow eligibility requires every common gate in the plan, heavy-occlusion valid rate at least +10 percentage points, composite normalized p95 error at least 20 percent better with no critical p95 metric more than 10 percent worse, conservative envelope containment at least 99 percent, zero face under-bounds over 30 mm, calibrated 90 percent intervals covering 85-95 percent, nominal regression no more than 10 percent, and p95 no more than 200 ms for per-frame use. If a gate is impossible or a dependency/weight/license is unavailable, record it as fail or not-measurable; never fabricate a pass or silently substitute the baseline. Store machine-readable evidence under docs/status/evidence/learning_research/LRF-P1/<revision>/, keep large datasets/weights/raw imagery out of Git, add focused no-privilege/OOD/import-isolation tests, run git diff --check and scripts/check_agent_contract.sh, commit bounded research artifacts, write the required eng note, and close with agent_complete.sh. Return exactly one recommendation: stop, continue-research, or shadow-candidate, with each L1 gate mapped to pass/fail/not-measurable. Completion of LRF-P1 never authorizes shadow wiring or production replacement. Do not start LRF-A1 or LRF-PL1.

## Pointers

- `docs/plans/learning_research_feasibility.md`
- `docs/agents/discuss/2026-09-05_1827_learning-research-feasibility-consensus.md`
- `docs/architecture/perception_architecture.md`
- `docs/architecture/sensor_data_pipeline.md`
- `docs/architecture/production_orchestration.md`
- `docs/plans/sim_r1_production_orchestrator_exploration.md`

## Open

- Run the research-only occlusion-aware 3D perception feasibility spike and return reproducible baseline/candidate metrics plus an L0/L1 recommendation; do not replace production modules.
## Claim -- eng/cursor-grok-b -- 2026-09-05 18:40 -- cursor/grok-4.6

- started_at: 2026-09-05T18:40:33+08:00
- claimed_generation: 1
- claimed_plan_revision: 18ebc7c62d94a90f2293a78153fa7f5f5dbaa8b3
- claimed_dependencies: none

## Result -- eng/cursor-grok-b -- 2026-09-05 18:53 -- cursor/grok-4.6

- outcome: pass
- completed_at: 2026-09-05T18:53:21+08:00
- revision: 5c08312192f65a1bd75a301c927b2a557faa0ab4
- tests: python3 -m unittest discover -s research/lrf_p1/tests -p test_*.py: 13 passed; run_spike.py --tests-result pass
- summary: LRF-P1 L0 harness is reproducible; recommendation continue-research; learned residual is not L1 or shadow eligible.
- evidence: docs/status/evidence/learning_research/LRF-P1/c09ec7009c5d58fd724373fceb43ecfc6c0bcc35

