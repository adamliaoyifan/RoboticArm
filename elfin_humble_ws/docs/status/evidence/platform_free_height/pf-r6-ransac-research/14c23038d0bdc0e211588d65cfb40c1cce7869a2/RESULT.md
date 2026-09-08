# PF-R6-RANSAC-RESEARCH result (generation 2)

- owner: `eng/claude/glm-5.3/claude`
- plan/task revision: `14c23038d0bdc0e211588d65cfb40c1cce7869a2`
- date: 2026-09-05
- machine: AMD Ryzen 9 9950X3D, RTX 5090 (idle), Linux 6.8.0-138,
  Python 3.10.12, numpy 2.2.6
- dirty-file isolation: production PF-R6 files untouched; baseline is the
  committed snapshot (see research/pf_r6_ransac/PROVENANCE.md sha256s)

## Recommendation: implementation-candidate = `zmode_median`

All eight plan gates pass for `zmode_median` and
`one_point_constrained_ransac`; `grid_seed_robust_z` also passes but is
slower and carries a known min-z bias (median err 5 mm vs 0 mm).

## Aggregate (synthetic frozen fixtures, 18 fixtures)

| method | pos valid | err med/p95/max (mm) | false measured (neg) | plane-fit p50/p95 @31k (ms) | speedup |
|---|---|---|---|---|---|
| baseline_ransac (committed) | 13/13 | 0.0/0.0/0.0 | 0 | 89.2/89.3 | 1x |
| zmode_median | 13/13 | 0.0/5.4/6.0 | 0 | 0.18/0.20 | ~450x |
| one_point_constrained_ransac | 13/13 | 0.0/0.0/0.0 | 0 | 0.19/0.21 | ~430x |
| grid_seed_robust_z | 13/13 | 5.0/7.6/10.0 | 0 | 1.54/1.55 | ~57x |

Full `estimate_local_support` (crop+band+annulus+fit), synthetic flat
fixtures: baseline p50 90.8-120.9 ms; zmode 2.2-3.6 ms; one_point
3.0-3.5 ms; grid 4.2-5.1 ms (`latency_full_ms.json`).

## Simulation gate 8 (accepted sim profile, ROS_DOMAIN_ID=7)

156 usable same-stamp (raw world cloud + live detector top) frames over
30 spawn trials, 10 per size (carryon/standard/large), XY jitter 0.12 m,
yaw ±0.6 rad. Support-valid rate = 1.000 for every comparator on every
size; max delta vs the exact committed baseline = 0.0 pp. Measured-height
error medians 3.3/9.4/3.8 mm (carryon/standard/large), identical across
methods — the error is dominated by the top estimate, not support Z.
Raw clouds: `/home/adamliao/work/pf_r6_ransac_data/sim_capture/`
(outside Git). Sim teardown residual: 0.

## Gate evaluation (summary.json)

1. Support-Z p95 ≤ 15 mm, max ≤ 25 mm — pass (all comparators)
2. Zero false measured support on all negative controls — pass
3. Fail-closed semantics — pass (identical committed validation wrapper)
4. p95 ≤ 75 ms and ≥ 2x baseline — pass (57x-450x)
5. p50 ≤ 50 ms — pass
6. Determinism (fixed input+seed; rng-free for zmode/grid) — pass
7. No privileged input / production import in the estimator — pass
8. Three-size valid-rate regression ≤ 1 pp — pass (0.0 pp, measured)

## Estimated critical-path effect

Production PF-R6 `support_ransac_ms` p50 126.008 / p95 183.909. Same
machine, same point counts: baseline re-measured 89.2/89.3 ms here;
`zmode_median` 0.18/0.20 ms. Scaling the production numbers by the
measured ratio replaces the ~126 ms stage with ≈0.3 ms and the whole
support stage (measured `support_total_ms` p50 135.5) with ≈3-4 ms —
an estimated ≈130 ms p50 reduction on the detector critical path.
PF-R6 closure remains the PF-R6 owner's to verify (PF-G6S + PF-R5
30-trial Gate 4 after production implementation).

## Method identification summary (see eng note for full report)

No primary automotive source names a "triangle-based RANSAC" ground-plane
method. The recalled family is most consistent with: (a) standard plane
RANSAC itself, whose 3-point minimal sample is a triangle — exactly what
the committed baseline already is; (b) Himmelsbach et al. IV-2008
(polar-grid seed + line fitting — line-based, not triangle); (c)
Axelsson 2000 progressive TIN densification (triangle-based ground
filtering, airborne LiDAR, not automotive RANSAC). Per the plan's
fallback rule, the benchmarked constrained candidate is the 1-point
normal-constrained RANSAC with adaptive early stopping, explicitly
labelled as the nearest well-supported constrained-RANSAC family, not as
the recalled method.

## Files

- `fixture_manifest.json` — frozen before final measurement (18 fixtures)
- `per_sample_metrics.json` — per fixture x method rows
- `determinism.json` — repeat + cross-seed checks
- `latency_ms.json`, `latency_full_ms.json` — plane-fit and full-stage
- `sim_gate8.json` — simulation three-size matrix
- `capture_rows.json` — capture index (157 files, 376 MB raw kept
  outside Git)
- reproduction commands: `research/pf_r6_ransac/README.md`
