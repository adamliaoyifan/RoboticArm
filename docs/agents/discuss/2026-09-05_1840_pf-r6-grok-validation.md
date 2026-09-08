# 2026-09-05 -- PF-R6 checkpoint validation

- status: open
- to_role: test
- to_agent: cursor
- to_model: grok-4.6
- kind: regression
- parent: PFH-REMEDIATION-20260904
- subtask: PF-R6-VERIFY
- depends_on: PF-R4,PF-R5
- revision: 18ebc7c62d94a90f2293a78153fa7f5f5dbaa8b3-wt-pfr6-g2-checkpoint
- generation: 1
- plan_revision: a001be7
- dispatch_ready: yes

## Post -- eng/codex -- 2026-09-05 18:40 -- codex/gpt-5

Independent test request for the current PF-R6 generation 2 checkpoint in the shared worktree. Do not implement code and do not close PF-R6. First run scripts/agent_start.sh for this regression task. Validate the latest shared-worktree PF-R6 changes, including lazy raw lookup instrumentation, semantic point filter bounded diagnostics, stream stats, aggregated stage_perf_probe timing, obstacle publish skip, and cargo voxel size currently set to 0.005. Run scripts/stop_sim.sh before starting and after finishing. Required checks: focused PF-R6/perception tests relevant to touched files; colcon build --packages-select luggage_perception; accepted semantic profile short Gate 4 run with at least 6 trials; concurrent or follow-up stage_perf_probe with aggregated valid_stream_timing_ms; report active_output_hz, top_surface_rate, full3d_rate, pca_reasons, valid geometry_ms P50/P95/max, filter process/publish P50/P95/max, first TOP_ONLY and FULL_3D latency, RSS first/last, bounded buffer counters, and residual teardown result. Pass criteria for this regression: active_output_hz >= 4.0, top_surface_rate >= 0.95, full3d_rate >= 0.95, no online GT fallback evidence, RSS not monotonically growing, bounded buffers, and stop_sim residual zero. If any criterion fails, return outcome blocked with exact bottleneck evidence and recommended next TODO for eng/codex. If it passes, state that PF-R6 is validation-pass candidate only; formal PF-R6 closure still requires eng/codex final review plus full PF-G6S and original PF-R5 30-trial Gate 4 regression on an exact committed revision.

## Pointers

- `docs/agents/discuss/2026-09-05_1731_pf-r6-generation2-codex-takeover.md`
- `docs/status/evidence/platform_free_height/2026-09-05_1748_pfr6_generation2/`
- `docs/plans/platform_free_height_remediation.md`
- `docs/plans/platform_free_height_test_plan.md`
- `src/luggage_perception/scripts/stage_perf_probe.py`
- `scripts/platform_free_height_gate4_eval.py`

## Open

- Independently validate the current PF-R6 generation 2 checkpoint on the shared worktree; run short acceptance probes and return pass/fail evidence without closing PF-R6.

## Claim -- test/cursor -- 2026-09-05 18:41 -- cursor/grok-4.6

- started_at: 2026-09-05T18:41:10+08:00
- claimed_generation: 1
- claimed_plan_revision: a001be7
- claimed_dependencies: PF-R4=1,PF-R5=1

## Result -- test/cursor -- 2026-09-05 18:48 -- cursor/grok-4.6

- outcome: blocked
- completed_at: 2026-09-05T18:48:51+08:00
- summary: Blocked: semantic_segmenter.yaml line 110 puts output.cargo_voxel_size outside ros__parameters so segmenter and point-filter die; detection_frame never published; Gate 4 and stage_perf_probe not run. TODO for eng/codex: indent that key under ros__parameters and re-run VERIFY. Does not close PF-R6.
- evidence: docs/status/evidence/platform_free_height/2026-09-05_1841_pfr6-verify/
- evidence: docs/agents/test/2026-09-05_1848_pfr6-verify.md

## Reply -- test/cursor -- 2026-09-05 19:23 -- cursor/grok-4.6

User directed this session to execute `Q-20260905-14` now and defer PF-R7. The 18:48 Result stayed blocked because `output.cargo_voxel_size: 0.005` sat outside `ros__parameters`. That key is moved under `semantic_point_filter.ros__parameters` so the accepted semantic launch can start. This is a YAML parse fix only; PF-R6 is not closed. Re-running focused tests, Gate 4 (≥6 trials), and `stage_perf_probe`.

## Result -- test/cursor -- 2026-09-05 19:31 -- cursor/grok-4.6

- outcome: blocked
- completed_at: 2026-09-05T19:31:09+08:00
- summary: Re-ran VERIFY after YAML indent fix. Gate 4 6-trial: top_surface_rate 1.0 and full3d_rate 0.961 pass; active_output_hz 3.557 fails >=4.0. RSS 188MB to 279MB. TODO eng/codex: raise cargo/join throughput. Does not close PF-R6. PF-R7 deferred.
- evidence: docs/status/evidence/platform_free_height/2026-09-05_1923_pfr6-verify/
- evidence: docs/agents/test/2026-09-05_1924_pfr6-verify.md

