# PF-R6-RANSAC-RESEARCH (isolated, ROS-free prototypes)

Research comparison of support-plane estimators for PF-R6, per
`docs/plans/pf_r6_support_plane_method_research.md` at plan revision
`14c23038d0bdc0e211588d65cfb40c1cce7869a2`. Nothing here edits
production PF-R6 files; the committed production estimators are snapshotted
read-only into `luggage_perception/` (see `PROVENANCE.md`).

## Comparators (`pfr6bench/methods.py`)

| key | what it is |
|---|---|
| `baseline_ransac` | verbatim committed 3-point minimal-sample RANSAC (200 fixed iterations, highest-supported-plane preference) |
| `zmode_median` | dominant-z-bin cluster + median (fail-closed robust z estimator from the PF-R6 handoff "Proposed Direction") |
| `one_point_constrained_ransac` | horizontal-prior 1-point RANSAC + classical confidence-based early stopping (nearest constrained-RANSAC candidate; *not* claimed to be the recalled automotive method) |
| `grid_seed_robust_z` | per-cell lowest-point seeds + robust-z over seeds (Himmelsbach-2008/Axelsson-2000 lineage adaptation) |

Every comparator is a drop-in replacement for the committed
`_ransac_horizontal_plane` and is run *inside* the unchanged committed
`estimate_local_support` (band, annulus, side coverage, reason codes), so
identical non-privileged candidate sets and identical fail-closed semantics
are guaranteed by construction (`pfr6bench/harness.py`).

## Reproduction

```bash
cd research/pf_r6_ransac
PYTHONPATH=. python3 -m pytest -q tests/test_prototypes.py
PYTHONPATH=. python3 pfr6bench/bench.py \
  --out <evidence_dir> --raw-dir <dir-outside-git>
```

Simulation capture (gate 8 inputs; requires the accepted sim profile from
the PF-R6 handoff running under `ROS_DOMAIN_ID=7`):

```bash
PYTHONPATH=research/pf_r6_ransac:$PYTHONPATH \
python3 research/pf_r6_ransac/pfr6bench/capture_sim.py \
  --out /home/adamliao/work/pf_r6_ransac_data/sim_capture \
  --trials 30 --frames-per-trial 6 --frames-window 6.0 --settle 8.0

PYTHONPATH=research/pf_r6_ransac python3 \
  research/pf_r6_ransac/pfr6bench/bench_offline.py \
  --capture-dir /home/adamliao/work/pf_r6_ransac_data/sim_capture \
  --out <evidence_dir>/sim_gate8.json
```

Raw dumps go to `/home/adamliao/work/pf_r6_ransac_data/` (outside Git).
Compact machine-readable metrics land in
`docs/status/evidence/platform_free_height/pf-r6-ransac-research/14c23038d0bdc0e211588d65cfb40c1cce7869a2/`.
