# 2026-09-07 -- PF-R8 cargo detection availability: static false positive and temporal hold

- status: done
- to_role: eng
- to_agent: claude
- to_model: glm-5.3
- kind: subtask
- parent: PFH-REMEDIATION-20260904
- subtask: PF-R8
- depends_on: PF-R6
- revision: f34d9171f8448d7bf92ce84897dc188a110216e2
- generation: 1
- plan_revision: 3460bff538af177217ce71c64e31d1d208cb2e67
- dispatch_ready: yes

## Post -- reviews/cursor -- 2026-09-07 20:39 -- claude-code/opus5

Implement PF-R8 to plan acceptance A1-A5: suppress the static border cargo false positive and repair DetectionTemporalGate so the consensus hold can fire. Fully offline-testable; do not open a sim.

## Pointers

- `docs/plans/pf_r8_r9_perception_acceptance.md`
- `docs/agents/reviews/2026-09-07_1956_pf-r6-gen3-current-issues-review.md`
- `docs/agents/discuss/2026-09-07_2010_pf-r8-r9-perception-acceptance-consensus.md`
- `docs/status/evidence/platform_free_height/2026-09-07_pfr6_gen3/failed_case_capture/`

## Open

- Implement PF-R8 to plan acceptance A1-A5: suppress the static border cargo false positive and repair DetectionTemporalGate so the consensus hold can fire. Fully offline-testable; do not open a sim.

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

## Claim -- eng/claude -- 2026-09-08 10:28 -- claude/glm-5.3

- started_at: 2026-09-08T10:28:53+08:00
- claimed_generation: 1
- claimed_plan_revision: 3460bff538af177217ce71c64e31d1d208cb2e67
- claimed_dependencies: PF-R6=3

## Result -- eng/claude -- 2026-09-08 11:21 -- claude/glm-5.3

- outcome: pass
- completed_at: 2026-09-08T11:21:34+08:00
- revision: 7b0b41a46c60a2ff8dc298106d3b906974762e92
- tests: full perception suite 502 passed; PF-R8 focused: fixture predicate 0 false accepts / 0 false rejects on 440 instances, causal replay 0 acausal / 0 missed holds, gate/epoch/expiry units; live A4 measurement baseline 0.874/33 vs post 1.000/0 (x2 runs, 489+512 settled), production agreement 845 matched 100% with TF; teardown residual 0
- summary: PF-R8 closed to plan acceptance A1-A5 at 7b0b41a. A1 workspace-projection predicate (bbox-centre ray to platform plane, radial accept radius 0.65 m vs FP 0.95 m): 0/0 on frozen 440-instance fixture incl. derived edge-clipped positive; fail-open flagged. A2 gate accepted-only semantics + alternation-proof window reset + epoch clear + expire-to-empty. A3 causal replay: miss set = the plan's 42 FP-only frames, 0 acausal, 0 missed, 40 unrecoverable reported to A4. A4 threshold 0.04->0.01 recovers 0.01-0.03 diagonal-yaw states: recall 1.000, miss run 0 on both post runs. A5 20 focused tests. Do-not-open-sim honored for implementation/falsification; live runs were the A4 measurement itself.
- evidence: docs/status/evidence/platform_free_height/2026-09-08_pfr8_recall/

