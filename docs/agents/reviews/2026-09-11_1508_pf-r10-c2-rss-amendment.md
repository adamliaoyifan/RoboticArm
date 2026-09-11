# 2026-09-11 -- PF-R10 C2 RSS amendment

- role: reviews
- agent: codex
- model: gpt-5
- cli: codex
- status: done

## Summary

By user decision, C2 no longer treats the raw least-squares RSS slope or
time-window minima over a mixed luggage-size sequence as a leak gate. Raw
measurements remain mandatory. The `2 MiB/min` risk guard now applies to a
workload-adjusted elapsed-time coefficient after controlling for luggage size;
bounded allocator and native-cache residency is explicitly non-failing.
Existing bd70df5 measurements are reported without retroactive rescoring
because the old probe lacks per-RSS-sample workload labels.

## Pointers

- `docs/plans/pf_r8_r9_perception_acceptance.md`
- `docs/status/evidence/platform_free_height/2026-09-11_pfr10_g4/sha_bd70df5/C2_RSS_AMENDMENT.md`
- `docs/status/evidence/platform_free_height/2026-09-11_pfr10_g4/sha_bd70df5/RESULT.md`
