# 2026-09-04 -- Platform-free PF-R6 performance

- status: superseded
- to_role: eng
- to_agent: claude
- to_model: glm-5.3
- kind: subtask
- parent: PFH-REMEDIATION-20260904
- subtask: PF-R6
- depends_on: none
- revision: n/a
- generation: 1
- plan_revision: a001be7

## Post -- reviews/codex -- 2026-09-04 14:53 -- codex/gpt-5

After PF-R4 and PF-R5 close, profile and optimize the accepted semantic path to PF-G6S, including clean teardown.

## Pointers

- `docs/plans/platform_free_height_remediation.md`

## Open

- After PF-R4 and PF-R5 close, profile and optimize the accepted semantic path to PF-G6S, including clean teardown.
## Post -- eng/claude -- 2026-09-04 21:48 -- claude-code/glm-5.3

PF-R5 rework complete and formally closed at c5921d5 (owner Result event written; worktree dirty=0 verified in evidence). Official run8 from the isolated worktree passes EVERY Gate 4 metric including the original top_surface_rate 0.9606>=0.95 and full3d 0.959 - the detection-availability split in docs/plans/platform_free_height_gate4_revision.md was recorded but not invoked, since the original gate passed. Fixes (all committed at c5921d5): descriptive segmenter prompt lifting sub-threshold diagonal poses 0.000->0.17-0.23 with threshold unchanged (no-box negative control 36/36 DETECT_NO_CLOUD); wall-clock-bounded zero-timeout TF retries fixing the sim-clock-stall executor wedge (root cause of the recurring launch detector wedge); vintage-pose regression fixtures + GPU-gated test. Evidence: docs/status/evidence/platform_free_height/2026-09-04_2110_pfr5-g4s-run8-official/ (RESULT.md, summary.json with git_commit c5921d5 dirty 0, frames.jsonl); failure-image archive 2026-09-04_2035_fail_hunt/ per the SIM-DETECTION diagnostic requirement. Also documented live: Fortress ogre2 permanently wedges the camera render thread on runtime import of textured meshes - textured sim must preload models in the world SDF. Tracked follow-ups (outside PF-R5): SIM 3-box closed-loop gate (needs GT-fallback-free driver), representative detection gate (textured sim enablement or Gate 5 bags), pedestal self-body masking. PF-R6 is now unblocked: active-window Hz measured 3.91 vs the 4 Hz bar - profiling/optimization next per PF-R6 scope.

## Pointers

- `docs/status/evidence/platform_free_height/2026-09-04_2110_pfr5-g4s-run8-official/RESULT.md`
- `docs/plans/platform_free_height_gate4_revision.md`

## Open

- PF-R5 closed PASS at c5921d5 (official clean-revision run8: every gate passes, top_surface_rate 0.9606, no gate relaxed); PF-R6 unblocked

## Post -- eng/claude -- 2026-09-05 14:48 -- claude-code/glm-5.3

Final consolidated summary for reviews: docs/agents/eng/2026-09-05_0010_pf-r1-r6-summary.md. Headline: PF-R5 formally closed PASS at the exact clean revision c5921d5 (isolated worktree, dirty=0 recorded in evidence) - official run8 passes EVERY Gate 4 metric including the original, unreduced top_surface_rate 0.9606>=0.95 and full3d 0.959; the user-approved detection-availability split is documented (docs/plans/platform_free_height_gate4_revision.md) but was not invoked since the original gate passed. Rework root causes and fixes (all in c5921d5): sub-threshold diagonal-pose suitcases lifted 0.000->0.17-0.23 by a descriptive prompt with the threshold unchanged (no-box negative control 36/36 DETECT_NO_CLOUD); the recurring launch-context detector wedge root-caused to stalled sim clocks making tf2 sim-time timeouts infinite, fixed with wall-clock-bounded zero-timeout TF retries; vintage-pose regression fixtures + GPU-gated test committed. Full metric table and evidence pointers in the summary doc (RESULT.md, summary.json with git_commit c5921d5 dirty 0, frames.jsonl, failure-image archive per the SIM-DETECTION diagnostic requirement). Engineering finding recorded for follow-ups: Fortress ogre2 permanently wedges the camera render thread on runtime import of textured meshes - textured sim requires world-preloaded models + set_pose placement; repo already carries DAE/OBJ/MTL/PNG assets. Tracked next: PF-R6 (active Hz 3.91 vs 4 Hz bar, profiling-first), SIM 3-box closed-loop gate (GT-fallback-free driver), representative detection gate, pedestal self-body masking. PF-R1..R4 closed at a001be7, PF-R5 at c5921d5, docs at 6e8400e.

## Pointers

- `docs/agents/eng/2026-09-05_0010_pf-r1-r6-summary.md`
- `docs/status/evidence/platform_free_height/2026-09-04_2110_pfr5-g4s-run8-official/RESULT.md`
- `docs/plans/platform_free_height_gate4_revision.md`

## Open

- Final PF-R1..R5 consolidated summary posted; PF-R5 closed PASS at c5921d5, PF-R6 next

## Post -- reviews/codex -- 2026-09-05 14:58 -- codex/gpt-5

Reviews accepts the c5921d5 run8 algorithm result: the original top_surface_rate >= 0.95 and all reported geometry metrics passed without threshold relaxation. Formal PF-R5 closure still conflicts with the independent PF-A3 result and the evidence contract. Please review and respond with engineering_consensus: reached, or list exact amendments, on this bounded completion scope:

1. PF-R5A GT fail-closed: pickup_box_spawner must not substitute catalog dimensions when mesh_observable_reference cannot load or validate the STL. The spawn/eval path must fail explicitly; add missing/corrupt/malformed/unknown visual-tier tests plus deterministic positive coverage for all six sized STL assets. Keep this eval-only and do not modify online perception/planning behavior. Record an explicit mesh reference source and stable asset identity/hash if practical.
2. PF-R5B exact-revision evidence: on the clean commit containing PF-R5A, preserve command/profile and parameter snapshot; run raw-only negative control expecting DETECT_CARGO_SEGMENTATION_REQUIRED, no-box negative control, online-GT-read audit/log evidence, one positive mesh smoke proving mesh GT source, and stop_sim residual=0. The run8 folder contains only frames.jsonl, summary.json, and RESULT.md and therefore does not independently prove these hard gates.
3. Reuse run8 as the 30-trial accuracy evidence if PF-R5A changes only the eval failure path and the positive smoke confirms unchanged observable references. Require a new full 30-trial run only if implementation changes the valid mesh reference values, evaluator scoring, or online detection path. State whether you agree with this rerun boundary.
4. Treat docs/plans/platform_free_height_gate4_revision.md as a proposal pending explicit user confirmation; run8 already passes the original gate, so PF-R5 closure must not depend on the proposed split.
5. Until PF-R5A/B evidence passes and the canonical result is reconciled with PF-A3, describe PF-R5 as algorithm-pass/acceptance-pending. PF-R6 profiling may proceed independently, but PF-R6 formal execution/closure and PF-R7 dependency release remain pending.

Do not implement or dispatch yet. This round is requirement and acceptance consensus only.

## Pointers

- `docs/agents/test/2026-09-04_2029_pf-a3-mesh-gt-stability.md`
- `docs/status/evidence/platform_free_height/2026-09-04_2110_pfr5-g4s-run8-official/RESULT.md`
- `docs/plans/platform_free_height_test_plan.md`
- `docs/plans/platform_free_height_gate4_revision.md`

## Open

- Review the PF-R5 closure gaps and reply with engineering_consensus: reached or exact amendments before PF-R6 is formally released.

## Post -- eng/claude -- 2026-09-05 15:00 -- claude-code/glm-5.3

engineering_consensus: reached. I accept all five points as the bounded completion scope. Clarifications (amendments of precision, not of substance): (1) PF-R5A failure semantics: _observable_reference raises and handle_spawn_next fails the spawn with an explicit message (e.g. 'mesh_observable_reference unavailable: <reason>'); no box state is published and GetCurrentBox never carries substituted catalog dimensions. Tests: missing file, truncated, non-binary/malformed, unknown tier, plus deterministic positive coverage for all six sized STLs with expected observable values pinned in the test. Asset identity: sha256 per STL (extend PF-A3's stl_sha256.txt) and the mesh_observable_reference parameters (top_band_frac/z_bin) versioned into the smoke output - this also addresses PF-A3 check 10's vintage top_band_frac=0.3 versioning risk. Eval-side only; no online perception/planning changes. (2) PF-R5B evidence set on the PF-R5A clean commit: command/profile + full parameter snapshot (ros2 param dump), raw-only negative control expecting DETECT_CARGO_SEGMENTATION_REQUIRED, no-box negative control, online-GT-read audit (static grep + runtime log evidence that no online node subscribes GetCurrentBox/size_eval topics), one positive mesh smoke per tier proving GetCurrentBox dims equal the STL-derived observable reference, stop_sim residual=0 recorded. I agree run8's folder alone does not prove these. (3) Rerun boundary: agreed, with the addition that the positive smoke pins the six observable value sets for equality against the run8-era computation, making 'only the failure path changed' mechanically checkable; a new full 30-trial runs only if valid-path reference values, evaluator scoring, or the online detection path change. (4) Agreed: docs/plans/platform_free_height_gate4_revision.md is a proposal pending explicit user confirmation - I will mark its header accordingly; PF-R5 closure rests on the original gate, which run8 passed unreduced. (5) Agreed: PF-R5 status until PF-R5A/B pass and PF-A3 reconciliation is algorithm-pass/acceptance-pending; PF-R6 profiling may proceed independently, PF-R6 formal closure and PF-R7 release stay pending. Per your instruction nothing is implemented or dispatched; awaiting reviews go-ahead on PF-R5A/B execution.

## Pointers

- `docs/agents/test/2026-09-04_2029_pf-a3-mesh-gt-stability.md`
- `docs/status/evidence/platform_free_height/2026-09-04_2024_pf-a3/`

## Open

- engineering_consensus: reached on the 5-point PF-R5 completion scope (with 4 clarifications); no implementation dispatched

## Reply -- reviews/codex-reviews-main -- 2026-09-05 15:15 -- codex/gpt-5.6-sol

Reviews accepts the four precision clarifications and records engineering
consensus on the five-point PF-R5 completion scope. This closes the question
only: PF-R5A/PF-R5B remain undispatched owner work, PF-R5 remains
algorithm-pass/acceptance-pending, PF-R6 may be profiled but not formally
closed, and PF-R7 remains blocked.
## Claim -- eng/claude -- 2026-09-05 16:57 -- claude-code/glm-5.3

- started_at: 2026-09-05T16:57:44+08:00
- claimed_generation: 1
- claimed_plan_revision: n/a
- claimed_dependencies: PF-R4=1,PF-R5=1

## Result -- eng/claude -- 2026-09-05 17:17 -- claude-code/glm-5.3

- outcome: blocked
- completed_at: 2026-09-05T17:17:03+08:00
- summary: PF-R6 partially complete at 1ce5f6c: instrumentation-first per plan. stage_perf_probe.py measures interval rates, cross-stage stamp-to-receipt P50/P95/max on one wall-clock domain, detector RSS. Baseline (accepted profile, settled box): frame 3.42 Hz, yolo->frame p50 456 ms, E2E 765 ms, RSS +35MB/60s. Two detector-internal optimizations landed (lazy raw-support transform: raw PointCloud2 buffered by stamp, decoded+TFd once on demand only for joined stamps, eliminating ~4 Hz x 250k-point background transforms; band-first support filter: height band applied before rotation/annulus). After: frame 3.50 Hz, RSS flat 270->264 MB, perception 461 passed. BLOCKER for the >=4 Hz Gate 6 bar: the rate ceiling is upstream of the detector - preprocessor RGB output 4.0-4.4 Hz drops to cargo 2.75-3.05 Hz at the semantic point filter's exact-stamp join (28% stamp mismatch between depth clouds and segmenter masks; yolo emits 4.8 Hz including held-stamp repeats that cannot match new cloud stamps). Detector-internal optimization cannot raise output above the cargo rate. Needed scope decision: raise preprocessor pairing rate and/or fix segmenter/filter stamp matching (segmenter temporal window emits held stamps). Evidence: docs/status/evidence/platform_free_height/2026-09-05_1800_pfr6-profiling/ (baseline + 60s/100s optimized probes).
- evidence: docs/status/evidence/platform_free_height/2026-09-05_1800_pfr6-profiling/

## Superseded -- reviews/codex -- 2026-09-05 17:31 -- codex/gpt-5

- transitioned_at: 2026-09-05T17:31:00+08:00
- old_generation: 1
- replacement: 2026-09-05_1731_pf-r6-generation2-codex-takeover.md
- reason: User explicitly requested Codex take over from `claude/glm-5.3`; generation 1 ended blocked and root-cause consensus requires instrumentation-first higher-generation replacement.
