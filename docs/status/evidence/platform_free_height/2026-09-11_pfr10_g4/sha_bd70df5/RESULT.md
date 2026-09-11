# PF-R10 g4 — semantic filter RSS and executor-lag repair

- Revision: `bd70df563c5567bcf36298c0aa083abeeed44af3`
- Worktree: `/tmp/pfr10_g4`, `dirty=0` at every run start
- Workload: unchanged `gate4_short6`, six trials, `--warmup-frames 30`
- Teardown: `scripts/stop_sim.sh`, residual process count `0` after every run

## Root cause

This was not an unbounded exact-stamp join or a retained history of old sensor
messages. In the failing g3 evidence the depth and mask buffers peaked at two
entries and joined entries were popped. RSS instead stepped upward when the
semantic cargo cloud changed size: in g3 run3 the last trial increased the
cargo cloud from about 26,000 to 120,270 points at the same time that the RSS
floor rose by roughly 5 MiB.

The hot path promoted the fixed float32 deprojection view to a variable-sized
float64 camera cloud, allocated another transformed world cloud, retained an
exact-sized tracker cloud, copied it even on the normal measurement path, then
converted it back to float32 for publication. The varying mask area caused
native allocator high-water retention and fragmentation. It was allocator
retention from avoidable variable-sized arrays, not evidence of unreachable
Python objects accumulating without bound.

## Repair

- Reserve the tracker's maximum decimated camera capacity once from live
  camera-info and reuse that buffer across measurements and epoch resets.
- Use geometric buffer growth for non-ROS tracker callers.
- Transform the fixed float32 camera view directly instead of first promoting
  it to another float64 array.
- Preserve float32 through the normal publication path and copy a tracked
  world cloud only on hold/reject paths that consume it after releasing the
  lock.

## Measured values under the superseded raw-slope rule

| run | filter RSS first→last MiB | slope MiB/min | lag Q4/Q1 | frame Hz | residual |
|---|---:|---:|---:|---:|---:|
| 1 | 98.69→99.73 | 0.326 | 1.108 | 23.323 | 0 |
| 2 | 95.34→96.29 | 0.793 | 1.118 | 23.958 | 0 |
| 3 | 98.53→99.88 | 0.320 | 1.080 | 21.961 | 0 |

All three filter raw slopes are below `2 MiB/min`; all lag Q4 means are below
`0.20 s` and ratios below `1.25`. Filter pending-work occupancy remained
bounded. These measurements show that the filter repair removed the g3
symptom, but they are not a PF-R10 full-acceptance result.

The 2026-09-11 C2 amendment no longer scores a least-squares fit over the raw
mixed-size RSS sequence. This probe version did not label each RSS sample with
box size and scored occurrence, so these runs cannot be retroactively scored
against the amended workload-adjusted memory-growth gate. They remain the
reported measurements that motivated the amendment.

## Guard results and caveats

- Run2 and run3 also passed Gate-4. Run1 had a stochastic geometry miss in
  trial 2 (`full3d_rate=0.817`) despite no geometry or threshold changes; C1
  remains established by the three clean g3 runs at `65821cf` and was not the
  user-directed collection target for g4.
- The detector's unadjusted RSS slope was `8.47 MiB/min` in run2; runs 1 and 3
  were `0.75` and `-0.22`. The current evidence cannot separate its bounded
  box-size working-set effect from a time effect because the required sample
  labels were not recorded. It is therefore reported without a pass claim.
- One run2 launch attempt aborted before scoring on `CONTROLLER_RACE`; teardown
  completed and the successful run2 replaced it.

## Verification

- Focused ROS/Python regression: `72 passed`.
- Full `luggage_perception` suite: `628 passed, 1 skipped`; three unrelated
  vintage tests could not obtain the missing `yolov8s-world.pt` in the
  restricted test process.
- Full clean satellite build: 11 packages passed.

## Acceptance status

- `bd70df5` demonstrates the semantic-filter allocation repair.
- It is not a same-clean-SHA `C1 AND C2` three-run pass: run1 failed C1, and
  the amended C2 memory gate needs a labelled rerun.
