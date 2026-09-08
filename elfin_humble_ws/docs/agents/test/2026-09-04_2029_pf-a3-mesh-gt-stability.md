# 2026-09-04 -- PF-A3 mesh-observable GT stability audit

- role: test
- agent: cursor
- model: grok-4.6
- cli: cursor
- status: done
- parent: PFH-REMEDIATION-20260904
- subtask: PF-A3
- revision: c5921d5f29ae5252747c7430ba2724214d1cbfc4

## Summary

Completed the immutable-snapshot PF-A3 audit at `c5921d5` (GT sources identical
at `408f6d5` and `1e8ea46`). Dual-seed probes compared equal. `audit_outcome`
is pass; `gt_readiness` is blocked because the spawner substitutes catalog
size into GetCurrentBox when the STL cannot be loaded. PF-R5 was not rerun,
accepted, or closed.

## Commands

- `git archive c5921d5f29ae5252747c7430ba2724214d1cbfc4` into `/tmp/elfin_pf_a3_c5921d5.DYsO97`
- `PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:$AUDIT_ROOT/src/luggage_description:$AUDIT_ROOT/src/luggage_perception python3 -m pytest -q src/luggage_description/test/test_suitcase_visual.py`
- `PYTHONHASHSEED=1` and `PYTHONHASHSEED=987654` runs of `pf_a3_gt_probe.py` with `cmp`

## Evidence

- `docs/status/evidence/platform_free_height/2026-09-04_2024_pf-a3/`
- `docs/status/evidence/platform_free_height/2026-09-04_2024_pf-a3/probe_seed1.json`
- `docs/status/evidence/platform_free_height/2026-09-04_2024_pf-a3/stl_sha256.txt`

## Result

- pass for the audit task
- audit_outcome: pass
- gt_readiness: blocked
- Checks 1-7 and 9 PASS; check 8 BLOCKED (catalog fallback); check 10 PASS with vintage `top_band_frac=0.3` versioning risks

## Pointers

- `docs/status/evidence/platform_free_height/2026-09-04_2024_pf-a3/`
- `docs/agents/reviews/2026-09-04_2022_pf-a3-mesh-gt-stability-dispatch.md`
- `docs/agents/discuss/2026-09-04_2022_pf-a3-mesh-gt-stability-audit.md`

## Open

- Historical check 8 at `c5921d5` stays blocked in this note. Independent
  reconciliation at `a3dba5e`: `docs/agents/test/2026-09-05_1639_pf-a3-gt-readiness-reconcile.md`.
