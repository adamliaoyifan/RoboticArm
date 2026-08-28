# Phase 5 process log

Chronological record of what was tried and the numbers that decided each step.
Rejected directions are kept deliberately: they are the expensive part to
re-derive. Machine-readable sweep results live in `ablations.yaml`
(regenerate with `packing_score_ablation.py`).

---

## 1. Hypothesis

Utilization is low and E16R failed for the same reason: space decisions depend
on a hand-tuned near-ROI rectangle, the online scorer ties on an empty
container, and a placement is chosen without regard for whether the next box
can still be reached.

## 2. Why the first box could not be followed by a second on the floor

Reconstructed from the E16 committed pose `center_base=[-0.335,-1.155,-0.170]`,
converted to container-local with `container_link` at `[0,-1.5,-0.86]`,
yaw `-1.5708`:

| quantity | value |
|---|---|
| local center | (-0.345, -0.335) |
| footprint (large, yaw 0) | 0.80 x 0.50 |
| local X edges | [-0.745, +0.055] -- flush against the opening wall |
| local Y edges | [-0.585, -0.085] |
| container local Y range | [-0.985, +0.985] |
| E16 `near_roi_y_min` | **-0.59** |

The box edge sits 5 mm inside the ROI bound and 0.40 m away from the real
container wall. After it, the ROI leaves strips of 0.045 m and 0.145 m --
nothing else fits on the floor, so box two *had* to stack. The stacking was an
artifact of the constant, not a policy decision.

ROI cost, measured: usable floor area 1.120 / 2.935 m2 = 38% at the launch
default, 0.549 / 2.935 = 19% at the E16 configuration.

## 3. Two independent mechanisms found in the ranking path

**(a) The scorer paid a structural bonus for stacking.** With the shipped
three-term proxy `1/(1+peak) + 0.25*conf + 0.20*contained`, an unobserved floor
column scores exactly 1.000 while a stack on a 0.32 m box scores
`1/1.32 + 0.25 + 0.20 = 1.208`. Break-even is at peak ~0.82 m, so *any* stack
below that beat an open floor. `confidence_ratio` and `contained_support` are
only ever set for stack candidates, so both terms are one-sided.

**(b) The floor never reached the scorer at all.** `FreeSpaceModel.candidates()`
sorted `SRC_FLOOR_PRIOR` last and then truncated with `results[:top_n]`.
Changing scoring weights alone would not have helped, because the floor
candidates were already gone. Fixed with `stratified_pool()` -- a per-support-
level quota -- before touching any weight.

## 4. Ablation: how much floor-first weight

Full grid in `ablations.yaml` under `w_floor_first_sweep`. The curve plateaus
at w >= 0.6 (identical results through w = 2.0), so 0.60 was taken as the first
point on the plateau rather than the largest value that still passes.

## 5. Rejected: grading MARGINAL below REACHABLE in the atlas prior

First implementation mapped atlas status to prior as
`REACHABLE 1.0 / MARGINAL 0.5 / UNKNOWN 0.0 / UNREACHABLE -1.0`. Result:

| prior mapping | rfr | items | floor items |
|---|---:|---:|---:|
| graded, MARGINAL 0.50 | 0.8432 | 8.85 | 3.20 |
| graded, MARGINAL 0.85 | 0.8532 | 8.90 | 3.20 |
| graded, MARGINAL 0.95 | 0.8674 | 8.90 | 3.20 |
| **flat top (shipped)** | **0.9201** | **9.60** | **3.30** |

Candidates that reach the scorer have already passed the atlas gate, so only
REACHABLE and MARGINAL remain; a 0.5 gap on a 0.15 weight was enough to pull
placements toward the opening and cost 0.75 items. Both statuses mean "IK
exists"; MARGINAL only adds "joint margin is tight", which the MoveIt filter
and the tilt/retention gates judge far more precisely downstream. Shipped
mapping therefore flattens the top two and keeps the penalty only for UNKNOWN
and UNREACHABLE.

## 6. Isolation: prior or tie-break?

b4 at w=0 initially differed from b2 (0.8432 vs 0.9201). Forcing a constant
prior of 0.5 reproduced b2 *exactly* (0.9201 / 9.6 / 3.30), proving the
deterministic tie-break was neutral and the regression was entirely the prior.
Without this step the tie-break would have been a plausible suspect and the
wrong thing might have been reverted.

After the flat mapping shipped, all three rows in
`prior_vs_tiebreak_isolation` coincide -- that is the regression guard passing,
not a lost signal.

## 7. Rejected by inspection: proxy_score Z coordinate

`insertion_corridor.proxy_score` built the candidate AABB with
`lz_floor = peak + box_h*0.5 + inner_h*0.5`. `peak` is already floor-relative,
so the extra `inner_h/2` lifted a floor placement to mid-container and
`blocks_deep_space` had been scoring the wrong corridor. Fixed. Historical B2
numbers were computed with the bug.

## 8. Offline gate

20 sequences x 60 boxes on the real slab-top atlas:

| strategy | rfr | items | floor items |
|---|---:|---:|---:|
| MVP three-term (what actually ran on the robot) | 0.9156 | 9.6 | 3.30 |
| B2 proxy_score | 0.9201 | 9.6 | 3.30 |
| B4 production scorer, w=0.60 | 0.9181 | 9.9 | 3.50 |

Items up, floor items up, rfr within noise (stdev ~0.09). Gate met, proceeded
to Gazebo.

## 9. Gazebo result: policy works, execution does not

Every seed offered `8 candidates (8 feasible, 8 on floor, atlas_rejected=0)`
for the first box. In the smoke run the second box still saw 8/8 floor
candidates at `map_revision=2, placed=1` -- no premature stacking. The first
box committed at `container_y=-0.635`, a position the old `near_roi_y_min`
forbade outright.

The matrix nonetheless produced zero commits, on three execution-layer gates
that had never been exercised this far out in the workspace:

- seeds 0/1: `release settle failed after hold 3.01s/0.0313` and `/0.0302`
  against a 0.03 rad/s tolerance
- seed 2: `DETECT_SIZE_MISMATCH errors=[0.1,0.05,0.04]`
- smoke run box 2: transit `sampler_or_collision_disconnected`,
  stage_mid `goal_ik_unavailable`

## 10. Partial fix applied, and what was deliberately not done

The backup-candidate path triggered only on `transit`/`traverse`; a
`stage_mid` failure aborted the whole loading loop. Extended to
`stage`/`stage_mid`/`stage_late` -- all run before insertion, so retrying
another candidate cannot leave a partially committed placement.
`insert`/`descend`/`retreat` still must not retry.

Not done, on purpose: relaxing the 0.03 rad/s settle tolerance or the 5 degree
suction gate would have produced commits immediately. Both are payload-safety
properties and E16R already records them as do-not-repeat. Guessing between
"3 s is too short" and "there is a structural residual" without a velocity
trace is exactly the kind of unfounded fix this log exists to prevent.

## 11. Next

Capture a settle-window velocity trace at a far placement pose. Decaying means
raise `release_settle_timeout`; flat means the residual is structural and needs
a controller-side fix. Then re-run this matrix for real utilization numbers.
