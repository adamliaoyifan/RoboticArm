# 2026-09-05 -- PF-R6 RANSAC research (generation 2)

- role: eng
- agent: claude
- model: glm-5.3
- cli: claude
- status: done

## Summary

Completed `PF-R6-RANSAC-RESEARCH` generation 2 (`Q-20260905-18`): benchmarked
the committed support-plane RANSAC against a fail-closed z-mode/median
estimator, a 1-point normal-constrained RANSAC with adaptive early stopping,
and a grid-seed robust-z estimator on identical non-privileged candidate
sets (18 frozen synthetic fixtures plus 156 same-stamp simulation frames
across carryon/standard/large). All eight plan gates pass for
`zmode_median` and `one_point_constrained_ransac`. Recommendation returned:
`implementation-candidate` (`zmode_median` primary). Production files were
not edited; no result here authorizes production replacement or PF-R6
closure.

## Method Identification Report

Required method study per
`docs/plans/pf_r6_support_plane_method_research.md`.

### Candidate identification

No primary automotive source found in the searched literature (web search
2026-09-05: "triangle-based RANSAC ground plane automotive", survey
cross-checks) names a "triangle-based RANSAC" ground-plane method. The
recall is most consistent with one of three real families:

1. Standard plane RANSAC itself. Its minimal sample of three
   non-collinear points is a triangle; surveys and tutorials describe it
   as "select three points, compute the plane" without triangle
   terminology. The committed baseline `_ransac_horizontal_plane`
   already is this method (3-point minimal sampling, 200 iterations,
   normal gate near vertical, highest-supported-plane preference).
2. Himmelsbach, Luerssen, Mueller, Manz, Franke, "Fast Segmentation of
   3D Point Clouds for Ground Vehicles", IEEE IV 2008 — the canonical
   fast automotive ground method: 2D polar grid, lowest point per cell
   as seed, line-segment fitting along beams with slope thresholds. It
   is line-based, not triangle-based.
3. Axelsson, "DEM Generation from Laser Scanner Data Using Adaptive TIN
   Models", ISPRS 2000 — progressive TIN densification: seed triangles,
   point-in-triangle angle/distance thresholds. Triangle-based ground
   filtering, but airborne LiDAR DEM generation, not automotive RANSAC.

Confidence that the exact recalled method was identified: low. Per the
plan's fallback rule, the benchmarked constrained candidate is the
nearest well-supported constrained-RANSAC family for this problem and is
labelled as such, not as the recalled method.

### "Triangle" meaning (plan question 2)

Most plausibly the three-point minimal sample of plane RANSAC
(`one_point_constrained_ransac` in this study shows the same idea pushed
to its constrained limit: a horizontal-prior plane needs a 1-point
minimal set). Spatial triangle constraints (Axelsson TIN) and polar-grid
seeds (Himmelsbach) are the alternatives; neither is RANSAC.

### Assumption transfer (plan question 4)

Himmelsbach assumes an unbounded, continuous, roughly planar road with
sparse LiDAR (10^4-10^5 pts, multi-beam rings), slope-based line fitting,
and no central occluder. This pickup problem is the inverse: a bounded
~0.3 m^2 annulus, dense noisy RGB-D (21k-31k candidates), one expected
horizontal support plane, central box occlusion, and possible
wrong-height clutter. What transfers: per-cell lowest-point seeding
(flyer suppression) and the horizontal prior. What does not: polar-beam
line models, long-range continuity, slope propagation. Axelsson's TIN
transfer is weaker still (no triangulation is needed for one plane).
This is why the grid-seed adaptation is the weakest of the three
comparators here.

### Complexity, stopping, licensing (plan questions 3, 5)

Baseline RANSAC: O(iterations x N) = 200 x 31k distance evaluations,
fixed iteration count, no early stop. 1-point constrained RANSAC: m=1
minimal sample, classical confidence bound
k >= log(1-0.99)/log(1-w), typically 5-15 iterations here, each O(N).
z-mode/median: histogram over z at 8 mm bins, O(bins x N) with ~57 bins,
rng-free. grid-seed: O(N log N) lexsort + histogram over ~120 seeds.
All are trivially ROS-free, numpy-only, no third-party implementation
needed; no licensing constraint was introduced (no external code was
copied; the Himmelsbach/Axelsson ideas are re-expressed in ~60 lines).

## Comparators and Isolation

- Baseline: verbatim committed `_ransac_horizontal_plane` at revision
  `14c23038d0bdc0e211588d65cfb40c1cce7869a2` (snapshot sha256s in
  `research/pf_r6_ransac/PROVENANCE.md`; the dirty PF-R6 working
  checkpoint was not copied).
- Every comparator is a drop-in `_ransac_horizontal_plane` replacement
  executed inside the unchanged committed `estimate_local_support`
  (band, annulus, min points, side coverage, reason codes), so identical
  fail-closed semantics and identical candidate sets hold by
  construction.
- Simulation GT (spawn response) and synthetic GT are eval-only; no
  estimator saw them.

## Results

See `docs/status/evidence/platform_free_height/pf-r6-ransac-research/14c23038d0bdc0e211588d65cfb40c1cce7869a2/RESULT.md`
for the aggregate table. Headlines (31k-point plane fit, this machine):

- baseline: p50 89.2 ms / p95 89.3 ms; full stage 90.8-120.9 ms
- zmode_median: 0.18/0.20 ms (~450x); err p95 5.4 mm; rng-free
- one_point_constrained_ransac: 0.19/0.21 ms (~430x); err p95 0.0 mm
- grid_seed_robust_z: 1.54/1.55 ms (~57x); err p95 7.6 mm (min-z bias)

Gates 1-8: pass for zmode_median and one_point_constrained_ransac
(gate 8 measured on 156 sim frames, 0.0 pp valid-rate delta);
grid_seed_robust_z passes all gates too but is dominated.

## Recommendation

`implementation-candidate`: `zmode_median` (dominant-z-bin cluster +
median, fail-closed). It is rng-free (trivially deterministic), the
fastest, has no external assumptions beyond the existing v1
horizontal-support contract, and its 5.4 mm synthetic p95 error is an
artifact of quantized noise at the 8 mm bin scale — real sim height
errors were identical to the baseline's. Secondary candidate:
`one_point_constrained_ransac` if the PF-R6 owner prefers keeping a
RANSAC-shaped estimator; it matched baseline accuracy exactly.

Next step for the PF-R6 owner (not authorized by this note): implement
the selected estimator in production scope, rerun focused tests, PF-G6S,
and the original PF-R5 30-trial Gate 4 regression.

## Verification

- `PYTHONPATH=research/pf_r6_ransac python3 -m pytest -q
  research/pf_r6_ransac/tests/test_prototypes.py`: 31 passed.
- `git diff --check`: clean on staged research artifacts.
- `scripts/check_agent_contract.sh`: pass.
- Sim teardown: `scripts/stop_sim.sh`, residual 0.

## Pointers

- `docs/status/evidence/platform_free_height/pf-r6-ransac-research/14c23038d0bdc0e211588d65cfb40c1cce7869a2/RESULT.md`
- `research/pf_r6_ransac/` (prototypes, fixtures, harness, README)
- `docs/agents/discuss/2026-09-05_2006_pf-r6-ransac-research-claude-reassignment.md`
- `docs/plans/pf_r6_support_plane_method_research.md`
