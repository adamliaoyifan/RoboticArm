# PF-R7 generation 10 acceptance redefinition and bounded startup recovery

Date: 2026-09-16

Parent: `PFH-REMEDIATION-20260904`

Subtask: `PF-R7`

Generation: `10`

Owner: `test/cursor/grok-4.6`

Base revision: `e686808bb7af83fdfa1333ff6e66e849568a3273`

Dependencies: `PF-R1,PF-R2,PF-R3,PF-R4,PF-R5,PF-R6,PF-A1,PF-R8,PF-R9,PF-R10`

Authority: user-directed on 2026-09-16. Generations 3 through 9 froze
`0.20` / `0.95` / `1.4 s` / `8.000 s` and the campaign shape as immutable.
This generation is explicitly authorised to change the aggregation, seed
supply, C2 sampling and infrastructure-budget rules. It does **not** change
the per-frame product thresholds.

## Decision and objective

Nine generations produced zero product defects. The record:

| Gen | Stop | Class |
|---|---|---|
| 3 run 1 | `eligible_fail` rate 0.9247 | scoring cursor set before the post-spawn drain |
| 3 run 2 | `eligible_fail` rate 0.903 | window started inside the designed `hold_track` transition |
| 4 run 1 | `eligible_fail` rate 0.903 | leftover support occupancy selected the wrong `t_steady` |
| 4 run 2 | `perception_availability_blocked` | three consecutive valid known detector misses |
| 5 | `time_budget` | Ultralytics CLIP auto-install, 540 s and 426 s |
| 6 | `seed_exhausted` | six imported standard seeds could not supply six eligible cases |
| 7 | `infrastructure_invalid` | controller manager never constructed |
| 8 | `infrastructure_invalid` | `event.action is target` never matched; S20 create never scheduled |
| 9 | `infrastructure_invalid` | spawn deadline measured against a probe the gate did not own |

All three scored failures were defects in the measuring instrument, confirmed
by review at the time. The remaining six were sample supply, environment
contamination and infrastructure. The gate has never once evaluated its own C2
bars.

Generation 10 keeps every per-frame product threshold and fixes the rules that
made the gate unable to return a product verdict.

## Unchanged product contract

These are byte-for-byte unchanged and must not be retuned:

- Production anchor `60dafb7deee50a6f3a76d48076b743bf3e3e1bc8`.
- Proposal confidence floor `0.20`.
- Conditional valid-top and `FULL_3D` rate floors `0.95` each.
- Output rate `>=4 Hz`.
- Support-ready starts at an admitted `support_window_count == 5`.
- Scored interval is the half-open `[t_steady, t_steady + 8.000s)`.
- `t_first_valid_top`, `t_first_FULL_3D` and `t_steady` are each
  `<= t_proposal + 1.4s`.
- All G4 geometry P95/max thresholds.

The generation-3 offline replay justifies keeping `0.95`: once the window
starts at the admitted support-ready boundary the settled tail scores
140/141 = 0.993. The bar was never the problem; the denominator was.

## Change 1: declare the per-case reliability target

The current rule is three slots by three sizes by two eligible cases, with one
`eligible_fail` aborting. That is an AND over 18 cases. At a true per-case pass
probability `p` the campaign passes with probability `p^18`:

| p | campaign pass |
|---|---|
| 0.99 | 0.84 |
| 0.97 | 0.58 |
| 0.95 | 0.40 |

The gate cannot distinguish "the product is 97 % reliable" from "the product is
broken"; both return fail. `AGENTS.md` requires a plan to state its aggregation
rule and thresholds, and the aggregation rule was stated while the product
requirement it encodes never was.

Generation 10 must state both:

- **Target**: at least 95 % of eligible cases meet every per-frame and geometry
  bar.
- **Campaign shape**: 18 eligible cases, three slots, two per size per slot,
  unchanged.
- **Accept** when at most 1 of 18 eligible cases fails and no failure belongs
  to a correctness class listed below.
- **Zero-tolerance classes**, where a single occurrence fails the campaign
  regardless of count: false cargo detection, geometry outside the G4 P95/max
  bars, a safety or collision violation, `stale_scored_or_fused` data, and any
  attempt whose required evidence is incomplete.

Rationale for "at most 1": at `p = 0.95` the 18-case campaign accepts with
about 0.77 probability and at `p = 0.85` with about 0.16, so the gate now
discriminates between the reliability levels it claims to care about instead
of returning fail for both.

The campaign still stops on the first `eligible_fail` to preserve T2 and the
exclusive slot. Stopping is a capture decision; the verdict is computed over
every completed case.

## Change 2: separate instrument failures from product failures

A scored `eligible_fail` whose root cause is proven to lie in the evaluator
(scoring window, classifier, quarantine accounting, seed bookkeeping) is an
**instrument defect**. It must:

- preserve its full T2 bundle;
- be repaired, regression-tested and committed inside the same generation;
- not consume a generation, and not require a superseding plan.

Generation 3 already did exactly this once under review direction when the
drain-cursor defect was found. Generations 4 through 9 instead spent a full
review, plan revision, mailbox row, worktree and claim cycle on each discovery.

A failure may only be classified as an instrument defect with a named,
evidenced mechanism and a regression fixture that reproduces it offline. A
threshold that is merely inconvenient is not an instrument defect.

## Change 3: seed supply gets a confidence margin and ordered substitutes

Measured standard-class detector availability over `standard_00..15`:
ten available, six missed (`03, 04, 05, 08, 11, 15`), a 37.5 % miss rate.

Two independent defects in the current rule:

1. **No margin band.** `standard_02` was scanned `proposal_available` at
   accepted confidence 0.256, only 0.056 above the unchanged 0.20 floor, and
   then became a valid known detector miss in the generation-6 live. Scan
   availability at marginal confidence does not transfer to the scored run.
2. **No substitution.** A valid known miss is unscored but still consumes a
   frozen seed, and the frozen list may not wrap, reuse or append. Generation 6
   died precisely here, one seed short.

Generation 10 requires:

- A seed counts as detector-available only when the accepted proposal
  confidence is `>= 0.30`, that is the unchanged `0.20` floor plus a `0.10`
  margin. Seeds between `0.20` and `0.30` are recorded as
  `available_marginal` and may enter the ordered substitute list but never the
  primary workload.
- The frozen artefact is an **ordered substitute list**, not an exact count. On
  a valid known detector miss the campaign draws the next substitute and
  continues; it does not stop.
- The list must hold at least the six primary standard cases plus six
  substitutes, sized from the measured 37.5 % miss rate rather than assumed.
- Exhausting the substitute list is reported as
  `perception_availability_blocked` and recorded as a **PF-R8 detection
  availability finding**, not as a PF-R7 stop. PF-R7 certifies platform-free
  height and geometry; PF-R8 owns detection availability and is already closed
  as a separate subtask.

## Change 4: C2 gets a minimum-sample rule

`executor_lag` Q4 mean `<= 0.20 s` and `<= 1.25 x Q1`, and workload-adjusted
RSS `<= 2 MiB/min`, have produced no samples in nine campaigns. Every result
reads "C2 unscorable because the campaign did not finish". A gate that has
never yielded a sample provides no assurance.

- Compute C2 over the completed prefix.
- Declare a minimum sample requirement: at least two completed slots and at
  least 200 executor-lag samples.
- Below the minimum, record `c2: not_evaluated` explicitly. That is neither a
  pass nor a fail, and it must appear in the verdict rather than being implied.
- C2 no longer blocks PF-R7 acceptance. Move the resource bars to a dedicated
  short soak that runs long enough to produce its own samples.

## Change 5: pre-scoring startup failures are not a product budget

Measured on this workspace, the `gz_ros2_control` URDF fetch loses its single
non-retrying parameter reply on roughly one launch in ten and then blocks
permanently. Evidence and mechanism:
`docs/status/evidence/platform_free_height/2026-09-16_startup_rehearsal/RESULT.md`.

A startup that fails before any case is attempted moves no scoring cursor,
consumes no seed and changes no hash. It cannot bias a result, so bounding it
to `restart_count <= 1` buys nothing and cost generations 8 and 9 outright.

- Pre-scoring startup attempts are unlimited within a declared wall-clock
  budget, default 900 s.
- Each attempt records its reason code, stage timings and teardown residuals.
- The per-launch startup success rate is reported as campaign evidence.
- The conditions stay strict: zero cases attempted, unchanged revision, config
  and seed hashes, complete T2 for the failed startup, exactly one `/clock`
  publisher after launch, and zero residuals before the next attempt.
- Once any case starts, no restart is permitted. That rule is unchanged, and it
  is the rule that actually protects the result.
- The launch now aborts with `startup_failed: plugin_urdf_not_received` within
  `startup_watchdog_sec` instead of hanging, so a relaunch costs seconds.

## Offline verification before the simulator

- Three consecutive runs of the focused classifier, window, campaign, seed
  substitution and C2-sampling suites with zero failures, errors, skips or
  retries.
- `src/luggage_gazebo/test/test_startup_probe.py` three consecutive times.
- Gate4 37 and PF-R10 RSS 10 once each.
- New regressions must cover: the at-most-one-failure aggregation, each
  zero-tolerance class failing on a single occurrence, substitute draw on a
  valid known miss, the `0.30` margin band rejecting a `0.256` seed, the C2
  minimum-sample rule emitting `not_evaluated`, and unlimited pre-scoring
  restarts refusing to restart after the first case starts.
- Evidence must record exact commands, exit codes, revision and dirty counts.

## Live procedure

Exclusive Gazebo, `gui:=false use_rviz:=false`, `ROS_DOMAIN_ID=7`,
`/tmp/elfin_humble_sim.pid`, `YOLO_OFFLINE=1`, `ULTRALYTICS_OFFLINE=1`,
`YOLO_AUTOINSTALL=false` asserted in the child before Ultralytics import.
Zero `/clock` publishers before launch, exactly one after. `stop_sim.sh` and a
zero-residual check between every attempt and at the end.

## Acceptance

Pass when the clean revision passes every offline check, the campaign completes
18 eligible cases with at most one non-zero-tolerance failure, no zero-tolerance
class occurs, seed substitution stayed inside the frozen ordered list, C2 is
either evaluated and within bars or explicitly `not_evaluated`, every attempt
has complete replayable evidence, and teardown residuals are zero.

Any zero-tolerance occurrence, a second ordinary failure, incomplete evidence,
a hash mismatch, contamination or a nonzero residual is non-pass.
