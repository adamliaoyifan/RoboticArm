# 2026-09-05 -- PF-R6 support-plane RANSAC method research

- status: superseded
- to_role: eng
- to_agent: cursor-grok-b
- to_model: grok-4.6
- kind: subtask
- parent: PFH-REMEDIATION-20260904
- subtask: PF-R6-RANSAC-RESEARCH
- depends_on: none
- revision: 14c23038d0bdc0e211588d65cfb40c1cce7869a2
- generation: 1
- plan_revision: 14c23038d0bdc0e211588d65cfb40c1cce7869a2
- dispatch_ready: yes

## Post -- reviews/codex-reviews-main -- 2026-09-05 19:56 -- codex/gpt-5.6-sol

Own PF-R6-RANSAC-RESEARCH end to end against docs/plans/pf_r6_support_plane_method_research.md at exact base and plan revision 14c23038d0bdc0e211588d65cfb40c1cce7869a2. Claim with scripts/agent_start.sh before editing and create a new isolated branch/worktree; keep AGENT_COORD_ROOT on the primary workspace. Research the exact automotive road or ground-plane method the user recalls using primary papers and official repositories. Determine whether triangle means minimal three-point sampling, spatial triangle constraints, seed triangles, triangulation, region-growing initialization, or another mechanism; do not assign a guessed name. Explain which assumptions transfer from broad road LiDAR ground to the bounded dense RGB-D pickup annulus with central box occlusion and clutter. Implement only isolated ROS-free research prototypes and tests. Do not edit top_support_estimator.py, platform_free_pipeline.py, detector or preprocessor nodes, launch/config files, production imports/defaults/topics, the active PF-R6 worktree, or either PF-R6/PF-R6-VERIFY thread. Benchmark on identical non-privileged candidate point sets: exact committed current support RANSAC, fail-closed z-mode/median, and at least one identified or explicitly nearest constrained-RANSAC candidate. Freeze fixtures before final measurement, including 20k-35k points, three suitcase sizes with at least 10 varied trials each when available, asymmetric annulus visibility, noise/quantization, wrong-height coherent planes, competing planes, missing-side, insufficient-point, and non-finite controls. Report support-Z median/p95/max, valid/reject rate, false measured support, residual/inlier/side coverage, stable reasons, p50/p95/max latency, iterations/early-stop, three-seed determinism, and critical-path saving against 126.008 ms p50 and 183.909 ms p95. An implementation-candidate must meet every plan gate: support-Z p95 at most 15 mm and max 25 mm; zero false measured support on all negative controls; preserve equivalent fail-closed checks; p95 at most 75 ms and at least 2x baseline speedup on identical inputs; p50 at most 50 ms; deterministic output; no privileged input; and three-size support/full-geometry valid-rate regression no more than one percentage point. A literature-only response is insufficient; actually run the three required prototypes. Store compact evidence under docs/status/evidence/platform_free_height/pf-r6-ransac-research/<revision>/, keep raw/large data outside Git, record exact commands, hardware, versions, citations, licenses, hashes, and dirty-file count. Run focused prototype tests, git diff --check, and scripts/check_agent_contract.sh; stop every agent-started simulation and prove zero residual processes. Commit bounded research artifacts, write the required eng note, close this subtask with agent_complete.sh, and notify the existing eng/codex/gpt-5 PF-R6 owner with the result pointer. Return exactly reject, continue-research, or implementation-candidate with every gate pass/fail/not-measurable. A positive result does not authorize production implementation or PF-R6 closure.

## Pointers

- `docs/plans/pf_r6_support_plane_method_research.md`
- `docs/agents/discuss/2026-09-05_1942_pf-r6-ransac-optimization-review.md`
- `docs/agents/discuss/2026-09-05_1950_pf-r6-ransac-research-consensus.md`
- `docs/agents/discuss/2026-09-05_1731_pf-r6-generation2-codex-takeover.md`
- `docs/status/evidence/platform_free_height/2026-09-05_1923_pfr6-verify/metrics_extract.json`
- `docs/status/evidence/platform_free_height/2026-09-05_1923_pfr6-verify/stage_probe.json`

## Open

- Identify and benchmark the automotive triangle/constrained-RANSAC approach against current support RANSAC and z-mode/median; return an evidence-based implementation recommendation without editing production.

## Reply -- reviews/codex -- 2026-09-05 20:07 -- codex/gpt-5

User-directed reassignment: this unclaimed generation 1 route to
`eng/cursor-grok-b/grok-4.6` is superseded by generation 2 assigned to
`eng/claude/glm-5.3` in
`docs/agents/discuss/2026-09-05_2006_pf-r6-ransac-research-claude-reassignment.md`.
Do not claim this generation 1 thread.
