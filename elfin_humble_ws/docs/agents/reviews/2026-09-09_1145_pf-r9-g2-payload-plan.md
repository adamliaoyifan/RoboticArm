# 2026-09-09 -- PF-R9 generation 2 payload execution plan

- role: reviews
- agent: codex-reviews-main
- model: gpt-5.6-sol
- cli: codex
- status: done

## Summary

Converted the directly agreed payload-reference design into an executable
PF-R9 generation-2 plan for `claude/glm-5.3`. The plan fixes camera and
downstream exact-join caches at 15 entries and one second, specifies opaque
payload ownership and read-only views across the full receive-to-publish path,
and defines D1-D8 simulation, hardware, mutation, boundedness, geometry, and
Gate 5 acceptance. Per the user's instruction, no further consensus round is
an execution prerequisite.

## Acceptance

- The implementation removes all avoidable preprocessor-owned RGB/depth
  payload copies and proves application-layer source/output payload identity.
- Camera and affected downstream buffers retain at most 15 entries over at
  most one second of canonical camera time, with bounded in-flight references.
- The fixed D1-D8 bars and required evidence in the execution plan all pass.

## Risks

- Python payload identity removes application-layer materialisation but does
  not remove DDS/CDR serialization.
- Exact stamp is the current wire-visible acquisition identity. Local epoch
  reset clears rollback state, but a cross-process epoch token remains a
  possible later architecture upgrade.

## Pointers

- `docs/plans/pf_r9_g2_payload_depth_primary_execution.md`
- `docs/plans/pf_f3_depth_primary_contract.md`
- `docs/agents/discuss/2026-09-07_2039_pf-r9-preprocessor-throughput.md`
- `docs/agents/discuss/2026-09-09_1053_pf-r9-gen1-stop-ack.md`
