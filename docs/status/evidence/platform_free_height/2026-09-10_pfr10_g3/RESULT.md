# PF-R10 generation 3 progress (cursor/grok-4.6)

PF-R10 generation 3 remains **open**. C1 is not met. Dirty-tree Gate-4
passes are not identity evidence. This file is a session reconstruction:
the original dump/identity directories are not in this tree.

## Identity revision

- Code: `df9c7a27f8ad81102b6f5462b08b724910b042cd`
  Keep cargo masks on accepted YOLO boxes and vectorize rectangle refine.
- Clean worktree used for identity eval: `/tmp/pfr10_g3` at that SHA,
  `git status --porcelain` empty.
- Placement closed-loop and static-hold already on master
  (`0d24f18`, `60ad37e`, `a5c5e29`).
- Eval-only Gate-4 dump harness: `7918a78` (off unless `--dump-dir`).
- This status record: `66aa423`.

## What C1 requires

`gate4_short6` x3 consecutive on **one committed revision** with
`git_dirty_files=0`: `active_output_hz>=4`, `top_surface_rate>=0.95`,
FULL_3D `>=0.95`, `false_measured_height==0`, `failed==0`, untrimmed
recovery `<=1.4 s`, `n_settled>=30` per trial. Then PF-G6S (C2) and
zero-residual teardown (C3).

## Diagnostic / dirty-tree scored runs

Same launch profile: `gui:=false use_rviz:=false`, semantic on, mesh,
catalog `carryon,standard,large`, `--warmup-frames 30`, `--settle-sec 8`,
`--min-trials-per-size 2`, `ROS_DOMAIN_ID=7`. Residual 0 after completed
runs.

| Run | Tree | C1 numbers | Notes |
|---|---|---|---|
| dump_run2 | dirty | fail | `t_first_full3d` 2.39 s / 1.66 s; width p95 0.062 m. Hz 19.8 passed. Not a missing lid after ~0.5 s. |
| exp_nodump_refine | dirty, vectorized refine | pass | recovery 0.26-0.62 s; width p95 0.021 m; `failed=0` |
| exp_nodump_r2 | same | fail | trials 1 and 5 whole-window `DETECT_TOP_UNOBSERVABLE`; `failed=365`; top 0.572 |
| exp_mask_accepted2 | dirty, accepted-only mask | pass | recovery 0.78-1.05 s; width p95 0.041 m; Hz 25.18; top 1.0; FULL_3D 0.983; `failed=0` |
| identity/run1 | `df9c7a2` dirty=0 | fail | trial 2 large vintage `DETECT_LOW_CONFIDENCE`, 2946 cargo points; top 0.818; `failed=184`. Streak stopped. Other trials recovered 0.59-0.81 s. |

Two other launches died when `controller_manager` missed the 60 s spawner
window. That is boot flake, not a Gate-4 score.

## Working diagnosis

- Dump CPU made unique geometry ~1.5 s/frame. No-dump recovery is under 1.4 s
  when the top plane is found. Not a 4 Hz miss.
- Unaccepted YOLO AABBs were still painted as cargo. That class is addressed
  in `df9c7a2` (`restrict_cargo_mask_to_accepted`). Geometry RANSAC
  thresholds were not loosened.
- Remaining C1 miss on the clean commit: a too-small accepted box
  (`DETECT_LOW_CONFIDENCE` / ~3k cargo points), a recall problem, not
  placement tip-over (closed-loop place already fail-closes).

## Next

Repair the small-box / low-confidence miss without changing detector
geometry thresholds, then re-run three consecutive `gate4_short6` plus
PF-G6S on one clean exact commit.
