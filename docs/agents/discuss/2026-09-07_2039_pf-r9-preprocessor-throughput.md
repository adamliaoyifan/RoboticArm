# 2026-09-07 -- PF-R9 preprocessor throughput and cloud-wait semantics

- status: superseded
- to_role: eng
- to_agent: claude
- to_model: glm-5.3
- kind: subtask
- parent: PFH-REMEDIATION-20260904
- subtask: PF-R9
- depends_on: PF-R6
- revision: f34d9171f8448d7bf92ce84897dc188a110216e2
- generation: 1
- plan_revision: 3460bff538af177217ce71c64e31d1d208cb2e67
- dispatch_ready: yes

## Post -- reviews/cursor -- 2026-09-07 20:39 -- claude-code/opus5

Implement PF-R9 to plan acceptance B1-B6: record the stamp-vs-arrival period measurement first, then split camera_slop_sec into a pairing tolerance and a wait deadline and restore preprocessor emission throughput. Keep tolerance pairing inside the preprocessor only.

## Pointers

- `docs/plans/pf_r8_r9_perception_acceptance.md`
- `docs/agents/reviews/2026-09-07_1956_pf-r6-gen3-current-issues-review.md`
- `docs/agents/discuss/2026-09-07_2010_pf-r8-r9-perception-acceptance-consensus.md`
- `docs/status/evidence/platform_free_height/2026-09-07_pfr6_gen3/stage_probe.json`

## Open

- Implement PF-R9 to plan acceptance B1-B6: record the stamp-vs-arrival period measurement first, then split camera_slop_sec into a pairing tolerance and a wait deadline and restore preprocessor emission throughput. Keep tolerance pairing inside the preprocessor only.

## Post -- reviews/cursor -- 2026-09-07 20:52 -- cursor/opus5

`dispatch_ready` reverted to `no`. This row was issued at plan revision
`76e5307` on a premature `consensus: reached`. The authoritative consensus
thread is `docs/agents/discuss/2026-09-07_2026_pf-r8-r9-plan-consistency.md`,
which rejects the amended plan with open findings. Do not claim this row until
that thread records `consensus: reached` and a new plan revision is bound here.

## Post -- reviews/cursor -- 2026-09-07 21:15 -- cursor/opus5

`dispatch_ready: yes`, bound to plan revision `3460bff538af177217ce71c64e31d1d208cb2e67`. The authoritative consensus
thread `docs/agents/discuss/2026-09-07_2026_pf-r8-r9-plan-consistency.md`
records `consensus: reached` at that revision, so the earlier park is lifted.

`depends_on: PF-R6` still gates the claim: `agent_start.sh` will refuse until
PF-R6 generation 3 has a passing `Result`. Record PF-R6's passing commit as the
implementation base in your Claim, not `f34d917`, which is only this plan's
analysis base.

## Claim -- eng/claude -- 2026-09-08 11:22 -- claude/glm-5.3

- started_at: 2026-09-08T11:22:13+08:00
- claimed_generation: 1
- claimed_plan_revision: 3460bff538af177217ce71c64e31d1d208cb2e67
- claimed_dependencies: PF-R6=3

## Result -- eng/claude -- 2026-09-08 13:09 -- claude/glm-5.3

- outcome: blocked
- completed_at: 2026-09-08T13:09:50+08:00
- revision: bebaa7c
- tests: full suite 510 passed; B6 8 new split/isolation tests + 19 existing preprocessor tests (1 updated to split semantics); B1/B2/B4/B5/B6 acceptance met; B3 blocked (0.462x vs 0.8, p50 256 ms vs 60 ms) with measured cause; whole-chain sanity active_output_hz 12.30, top 1.000, false 0
- summary: Blocked on B3 only, by measurement. B1 done first (receipt lag p50 2.8/p95 22.9/max 39.3 ms, 15.24% cloudless RGB stamps, BEST_EFFORT loses 31-37% -> params tolerance 5 ms, deadline 60 ms, RGB-clock driven). B2 split implemented+tested. B4 passes on the final stride-2 config (cloud_ok 0.966, join 0.989, stale 0.012). B5: BLAS-free transform (node was 435% CPU), publisher-thread decoupling (publish p95 was 233 ms in-callback), float32+fast decode, decimation stride 2 (density 29.5k->7.4k cargo points), filter stats throttle. B3 misses with cause: DDS transport ~24 MB/s for 1.8 MB/emission, Python decode 16-59 ms/frame, pairing wait floor ~40 ms; stride 3 reaches 0.615x but breaks B4 at the filter. The plan's stated trigger fired: raise the out-of-scope pixel-space masking change as a new consensus item (removes the 307k cloud publish entirely; only path to both B3 bars). Whole chain at final config: active_output_hz 12.30 vs 4.0 bar, top_surface_rate 1.000, support z sub-micron, false 0.
- evidence: docs/status/evidence/platform_free_height/2026-09-08_pfr9_throughput/

## Superseded -- reviews/codex-reviews-main -- 2026-09-09 11:47 -- codex/gpt-5.6-sol

- transitioned_at: 2026-09-09T11:47:29+08:00
- old_generation: 1
- replacement: 2026-09-09_1147_pf-r9-g2-payload-depth-primary.md
- reason: User directly approved PF-R9 generation 2 payload-backed depth-primary execution at plan revision 5fe74ed72135e569efd1e01f47047f664234fb6b; generation 1 remains blocked on B3 and must stop before generation 2 claim.

