# PF-R10 g3 experiment — vectorized refine, no dump, accepted-only mask

Scored `gate4_short6` (`--warmup-frames 30`, `--settle-sec 8.0`, `--min-trials-per-size 2`),
`ROS_DOMAIN_ID=7`, `gui:=false use_rviz:=false`. Residual 0 after every completed
run. Tree is dirty (not a C1 identity revision).

## Code (uncommitted)

- Vectorized `_refine_rectangle` (same 161-step search / trim / first-min).
  Tests: `test_luggage_box_estimator.py` 25 passed.
- Published cargo mask keeps only accepted YOLO bboxes
  (`restrict_cargo_mask_to_accepted`). Tests: segmenter + PF-R8 70 passed
  with the estimator suite.

## Runs

| Folder | Code | C1 | Notes |
|---|---|---|---|
| `exp_nodump_refine/` | vectorized refine, old mask | pass | `t_first_full3d` 0.26–0.62 s; width p95 0.021 m; `failed=0` |
| `exp_nodump_r2/` | same | fail | trials 1 and 5 whole-window `DETECT_TOP_UNOBSERVABLE`; `failed=365`; top rate 0.572 |
| `exp_mask_accepted/` | + accepted-only mask | boot fail | `controller_manager` not up before spawner 60 s timeout |
| `exp_mask_accepted2/` | + accepted-only mask | pass | all 6 `pca_reason=ok`; `t_first_full3d` 0.78–1.05 s; width p95 0.041 m; `failed=0` |
| `exp_mask_r3/` | same | boot fail | same controller_manager timeout after a tight relaunch |

## Reading

- Dump_run2 recovery (>1.4 s) did not reproduce without `--dump-dir`. Unique
  geometry is no longer the C1 miss when detection works.
- Whole-window UNOBSERVABLE with ~14–16 k cargo returned on r2. r2 cargo sat
  on rejected YOLO AABBs still painted as label 2. Mask change is the
  reviews-allowed recall lever; one scored pass followed, not three
  consecutive.
- PF-G6S RSS slopes on ~69 s eval windows are not a C2 identity measurement.
- C1–C3 still need one clean commit and three consecutive stored runs.

## Pointers

- `exp_nodump_refine/gate4/summary.json`
- `exp_nodump_r2/gate4/summary.json`
- `exp_mask_accepted2/gate4/summary.json`
