# 2026-09-10 -- PF-R9 g2 payload-backed depth-primary closed (D1-D7 pass, D8 deferred)

- role: eng
- agent: claude
- model: glm-5.3
- cli: claude
- status: done

## Summary

PF-R9 generation 2 closed **pass** at `bcb54c9` (implementation chain
`7bd6ac8` → `42f97af` → repair commits → `1d19528`). The camera point
cloud is gone from the pipeline: opaque immutable payload references
with read-only stride-aware views carry colour and aligned depth through
receive -> buffer -> pair -> copy-out -> queue -> identity republish with
**zero Python full-frame materialisations** (measured, first-party
counter); camera caches are 15 entries / 1.0 s on integer stamps with a
rollback epoch; aligned depth is the mandatory pair gate with
canonical-set validation; the filter and detector deproject locally via
the shared `depth_deprojection` module. Simulation acceptance: emission
**1.248×** colour rate (g1 ceiling 0.46×), latency p50 **18.6 ms**
(was 256), paired depth 1.000, filter exact join 0.983, stale 0.0419,
detector support coverage 0.981, buffers 15/15/4 with zero queue drops,
suite 543 passed, production cloud-topic audit clean. The D1 measurement
corrected g1's attribution: the 233 ms "publish" was output construction
(`tobytes`, ~40 ms/MB); `publish()` itself is 0.2-0.5 ms. D8 (D455
hardware 120 s + RSS trend) deferred by user decision, parked as
`Q-20260909-11` with trigger = all depth-primary-related changes
complete; the cell-side run pack is
`D8_cell_procedure.md`. No bar was lowered; D8 deferral is recorded in
the Result summary and the evidence.

## Pointers

- `docs/agents/discuss/2026-09-09_1147_pf-r9-g2-payload-depth-primary.md`
- `docs/agents/discuss/2026-09-09_1939_d455-hardware-validation-deferred.md`
- `docs/status/evidence/platform_free_height/2026-09-09_pfr9_g2_payload/RESULT.md`
- `docs/plans/pf_r9_g2_payload_depth_primary_execution.md` @ `5fe74ed`

## Open

- Q-20260909-11 (deferred D455 validation) awaits reviews formalisation;
  PF-R10 remains the next runnable dependency of the chain.
