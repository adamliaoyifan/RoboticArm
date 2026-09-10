# PF-R10 g3 identity run1 — C1 not met

Identity attempt on `df9c7a27f8ad81102b6f5462b08b724910b042cd` from
worktree `/tmp/pfr10_g3` (`dirty=0`). Stopped after run1; run2/run3 were
not started.

## Verdict

- C1: fail. Trial 2 (`pickup_box_0003_large`, vintage, yaw 0.376) whole-window
  `DETECT_LOW_CONFIDENCE`, 2946 cargo points, `t_first_valid=None`.
  `top_surface_rate=0.818`, `failed=184`. Other trials recovered in 0.59–0.81 s.
- C2: fail on this window (`executor_lag` ratio, filter/detector RSS slope).
- C3: pass for this attempt (`residual_count=0`, sim stopped).

Existing dirty-tree passes (`exp_nodump_refine`, `exp_mask_accepted2`) are
not this revision's C1 evidence.

## Pointers

- `run1/gate4/summary.json`
- `run_streak.sh`
