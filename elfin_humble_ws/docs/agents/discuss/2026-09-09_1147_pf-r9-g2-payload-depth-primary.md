# 2026-09-09 -- PF-R9 generation 2 payload-backed depth-primary implementation

- status: done
- to_role: eng
- to_agent: claude
- to_model: glm-5.3
- kind: subtask
- parent: PFH-REMEDIATION-20260904
- subtask: PF-R9
- depends_on: PF-R6,PF-R8
- revision: 5fe74ed72135e569efd1e01f47047f664234fb6b
- generation: 2
- plan_revision: 5fe74ed72135e569efd1e01f47047f664234fb6b
- dispatch_ready: yes

## Post -- reviews/codex-reviews-main -- 2026-09-09 11:47 -- codex/gpt-5.6-sol

User-directed execution handoff; no further consensus is required. Before claiming generation 2, acknowledge the stopped generation-1 thread with scripts/agent_mailbox.py stop-ack. Then claim this thread with scripts/agent_start.sh and own implementation, tests, repair loops, commits, simulation teardown, evidence, and D555 validation end to end. Use docs/plans/pf_r9_g2_payload_depth_primary_execution.md at the bound plan revision. Fixed cache contract: 15 entries and 1.0 second on camera and affected exact-join buffers. D1 complete-path measurement is first; D2 eliminates every avoidable preprocessor Python pixel copy while preserving mutation isolation; D3-D8 bars may not be lowered.

## Pointers

- `docs/plans/pf_r9_g2_payload_depth_primary_execution.md`
- `docs/plans/pf_f3_depth_primary_contract.md`
- `docs/status/evidence/platform_free_height/2026-09-08_pfr9_throughput/`
- `docs/status/evidence/d555_bringup/`
- `docs/agents/discuss/2026-09-07_2039_pf-r9-preprocessor-throughput.md`

## Open

- Acknowledge the PF-R9 generation-1 supersede, then implement and close PF-R9 generation 2 end to end against the exact payload/cache plan and D1-D8 acceptance.

## Claim -- eng/claude -- 2026-09-09 11:55 -- claude-code/glm-5.3

- started_at: 2026-09-09T11:55:32+08:00
- claimed_generation: 2
- claimed_plan_revision: 5fe74ed72135e569efd1e01f47047f664234fb6b
- claimed_dependencies: PF-R6=3,PF-R8=1


## Reply -- eng/claude -- 2026-09-09 22:05 -- claude/glm-5.3

Progress: D1-D7 (simulation side) complete and committed (`1d19528`,
implementation chain `7bd6ac8`+`42f97af`+repair commits). Formal 125 s
scored window: emission 1.248x, latency p50 18.6 ms, zero payload
materialisations, paired depth 1.000 (first-party per-emission flags,
n=3892), filter exact join 0.983, stale 0.0419, detector support
coverage 0.981, buffers 15/15/4 with zero queue drops; suite 543
passed; production cloud-topic audit clean; Gate5 migrated to the
manifest-declared optical frame. Full evidence:
`docs/status/evidence/platform_free_height/2026-09-09_pfr9_g2_payload/RESULT.md`.

Remaining before close: D8 (D455 colour+aligned depth 640x360@15 Hz,
>=120 s, point-cloud pub/sub disabled) and the per-node RSS-trend
sampling on that sustained run. The thread stays open; no bar was
lowered and no scope widened.
## Result -- eng/claude -- 2026-09-09 19:40 -- claude/glm-5.3

- outcome: pass
- completed_at: 2026-09-09T19:40:16+08:00
- revision: bcb54c9770dd555ccd4e36da8bbda63e76215bca
- tests: suite 543 passed + 44 subtests; colcon build luggage_perception+luggage_gazebo pass; production rg audit clean of the preprocessed cloud topic; D1 baseline recorded pre-refactor; D2 zero materialisations on the canonical path (identity fixtures both resolutions); D3 emission 1.248x / p50 18.6 ms / queue drops 0; D4 paired depth 1.000 (n=3892), filter join 0.983, stale 0.0419, detector support coverage 0.981; D5 fixture suite; D6 annulus retention stride 4/5 margins 0.03-0.18 (min 708 vs 80); D7 buffers 15/15/4 bounded
- summary: PF-R9 g2 closed pass on the simulation side (D1-D7). Payload-identity architecture landed end to end: opaque immutable payloads with read-only stride-aware views, integer-stamp 15/1.0s camera caches with epoch rollback, depth-mandatory preprocessor with canonical-set validation, filter/detector local deprojection (shared depth_deprojection module), no transported camera cloud anywhere in production. Throughput went from the g1 ceiling 0.46x/256 ms to 1.248x/18.6 ms p50 with zero payload materialisations. Repair chain documented in RESULT.md (transport queue depths, segmenter stats throttle + overlay off, surgical join retirement, detector pending-join park with mutually-exclusive emitter timer replacing the impossible in-callback sleep retry, RELIABLE depth product). D8 (D455 hardware run + RSS trend) deferred by user decision 2026-09-10, parked as Q-20260909-11 with trigger = all depth-primary-related changes complete; cell-side procedure committed as D8_cell_procedure.md. Evidence: 2026-09-09_pfr9_g2_payload/RESULT.md.
- evidence: docs/status/evidence/platform_free_height/2026-09-09_pfr9_g2_payload/
