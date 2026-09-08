# 2026-09-07 -- PF-R6 generation 3 zmode implementation

- role: eng
- agent: claude
- model: glm-5.3
- cli: claude
- status: done

## Summary

Implemented the adopted `zmode_median` support estimator in production
(`top_support_estimator.py`) on the gen-2 checkpoint, added 14 focused
tests (69 passed in the focused suite), and verified live: support-stage
latency 126.008 -> 0.604 ms p50 (~208x), valid geometry 210.8 -> 65.3 ms
p50, support-Z error sub-micron, false measured height 0. PF-R6's 4.0 Hz
acceptance is now blocked upstream (preprocessor 3.6 Hz ceiling and
observation-completeness variance), not by the detector; evidence and
decision options recorded. PF-R6 remains open.

## Status

- Production change is bounded to the support-fitting stage; top RANSAC,
  join semantics, config surface, and every fail-closed gate unchanged.
- One refinement over the research prototype (documented in the
  estimator docstring and evidence RESULT.md): cluster selection is
  dominant-window-count-first with a 5% tie tolerance to the higher
  plane, replacing the prototype's first-from-top rule that let thin
  noise tails bias support_z by ~5 mm.
- Formal generation 3 dispatch is pending reviews (`Q-20260907-1`,
  requested 2026-09-07 17:55); implementation proceeded on explicit user
  direction in the claude session. Claim bookkeeping will be attached to
  the generation 3 thread when it exists.
- `gate4_pass` remains false: active_output_hz 3.51-3.76 (bar 4.0).
  Bottleneck moved out of detector scope; three decision options are in
  the evidence RESULT.md. Do not close PF-R6 or Q-20260905-10.

## Verification

- `PYTHONPATH=src/luggage_perception python3 -m pytest -q` over the six
  focused files: 69 passed.
- `colcon build --packages-select luggage_perception --symlink-install`:
  pass.
- Production estimator on the research 156-frame capture: valid rate
  1.000, height err med 3.68 mm, fit p50 0.102 ms.
- Live probes: see
  `docs/status/evidence/platform_free_height/2026-09-07_pfr6_gen3/`
  (`stage_probe.json`, `gate4_short6*`). Teardown residual 0;
  `check_agent_contract.sh` pass.

## Pointers

- `docs/status/evidence/platform_free_height/2026-09-07_pfr6_gen3/RESULT.md`
- `src/luggage_perception/luggage_perception/top_support_estimator.py`
- `src/luggage_perception/test/test_pf_r6_zmode_support.py`
- `docs/agents/discuss/2026-09-07_1755_2026-09-07_pf-r6-gen3-reassignment-request.md`
- `docs/agents/discuss/2026-09-05_2040_2026-09-05_2145_pf-r6-ransac-research-result-for-owner.md`
