# 2026-09-09 -- HB-1 IR datum review

- role: reviews
- agent: codex
- model: gpt-5
- cli: codex
- status: done

## Summary

HB-1 does not provide a housing-to-left-IR mechanical dimension, but that
dimension is unnecessary for ChArUco hand-eye calibration. The solve observes
the colour optical frame directly, and HB-1 supplies the live static transform
needed to compose the result into `d555_link`. The missing datum limits the CAD
seed; it must not block the calibration.

The generation-2 HE-1 blocked outcome is valid as a CAD-only outcome because
its accepted requirement explicitly demanded an absolute mechanical-to-
`d555_link` transform. The requirement itself is too strong for the unified
calibration objective. A replacement generation should retain the mechanical
fit as an orientation/collision sanity check, drop absolute CAD translation as
a prerequisite, and use hand-eye residuals, multi-method spread, holdout
error, and the independent HB-3 re-measurement as the acceptance evidence.

## Pointers

- `docs/status/evidence/d555_bringup/20260908_1951_hb1/SUMMARY.md`
- `docs/status/evidence/d555_bringup/20260908_1951_hb1/tf/lookups.txt`
- `docs/agents/discuss/2026-09-09_1218_d555-handeye-he-1-generation2.md`
- `docs/plans/d555_handeye_calibration.md`
- `src/luggage_perception/scripts/handeye_solve.py`
