# PF-R7 nine-generation audit: why it never passed, and whether the metrics are sound

- role: reviews
- agent: cursor
- model: opus-5
- cli: cursor
- status: done
- date: 2026-09-16
- parent: PFH-REMEDIATION-20260904
- subtask: PF-R7
- thread: `docs/agents/discuss/2026-09-16_0930_pf-r7-generation10-acceptance-redefinition.md`
- commits: `0ff903b` (launch fix and rehearsal tooling), `cbcdfe9` (generation 10 plan)
- evidence: `docs/status/evidence/platform_free_height/2026-09-16_startup_rehearsal/RESULT.md`

## Summary

Nine PF-R7 generations found zero product defects. All three scored failures
were defects in the measuring instrument; the other six stops were sample
supply, environment contamination, or infrastructure. The three infrastructure
generations shared one never-diagnosed fault: `gz_ros2_control` 0.7.21 loses
its single non-retrying URDF parameter reply on roughly one launch in ten and
then blocks forever. A ten-launch rehearsal locates the failure at
`plugin_urdf_not_received` and shows `robot_state_publisher` answering within
1.21 s every time, which refutes the premise all three generations were built
on. The launch now bounds that race instead of reordering around it, and
generation 10 redefines the acceptance aggregation, seed supply, C2 sampling
and infrastructure budget while keeping every per-frame product threshold.

## Pointers

- `docs/plans/pf_r7_generation10_acceptance_redefinition.md`
- `docs/agents/discuss/2026-09-16_0930_pf-r7-generation10-acceptance-redefinition.md`
- `docs/status/evidence/platform_free_height/2026-09-16_startup_rehearsal/RESULT.md`
- `scripts/sim_startup_rehearsal.sh`
- `src/luggage_gazebo/test/test_startup_probe.py`
- `.cursor/rules/sim-lifecycle.mdc`

## Finding 1: nine generations, zero product defects

Between 2026-09-14 18:28 and 2026-09-16 09:13, PF-R7 ran nine generations and
at least nine live campaigns and found no defect in the product under test.

| Gen | Stop | Actual cause |
|---|---|---|
| 3 run 1 | `eligible_fail` 0.9247 | scoring cursor set before the post-spawn drain |
| 3 run 2 | `eligible_fail` 0.903 | window started inside the designed `hold_track` transition; settled tail 140/141 = 0.993 |
| 4 run 1 | `eligible_fail` 0.903 | leftover support occupancy chose the wrong `t_steady` |
| 4 run 2 | `perception_availability_blocked` | three consecutive valid known detector misses |
| 5 | `time_budget` | Ultralytics CLIP auto-install, 540 s and 426 s |
| 6 | `seed_exhausted` | six imported standard seeds could not supply six eligible cases |
| 7 | `infrastructure_invalid` | controller manager never constructed |
| 8 | `infrastructure_invalid` | `event.action is target` never matched; S20 create never scheduled |
| 9 | `infrastructure_invalid` | spawn deadline measured against a probe the gate did not own |

All three scored failures were defects in the measuring instrument, each
confirmed by review at the time. Sixteen `eligible_pass` cases in generation 6
alone, and zero `eligible_fail` across generations 4 through 9.

## Finding 2: the infrastructure root cause was never diagnosed, only worked around

`gz_ros2_control` 0.7.21 fetches the model URDF with one
`AsyncParametersClient::get_parameters` call whose `wait_for(5s)` return value
is discarded before `get()` is called. One lost ~22 KB reply blocks the Gazebo
system-load thread permanently; the outer retry loop is unreachable. The
installed shared object offers no other URDF source.

Measured on stock master with `scripts/sim_startup_rehearsal.sh`, N=10: nine
ready stacks, one `plugin_urdf_not_received`. The failing log shows
`connected to service!!` followed by neither the success line nor the plugin's
own retry line, which only the blocked `get()` can produce.

`robot_state_publisher` served the parameter in at most 1.21 s in every
iteration including the failing one. Generations 7, 8 and 9 all attempted to
make the robot spawn wait longer for RSP. **RSP was never the late party, so
none of those three generations could have worked**, and each of them burned a
scored live slot proving nothing. G7 was diagnosed by inspection as a
"startup/RMW service or relaunch-isolation defect"; G8 and G9 then built on
that unverified reading and each introduced a fresh launch defect.

## Finding 3: the process converted a 10% flake into three dead generations

Three structural rules interacted badly:

1. The eval task was allowed to modify `sim_world.launch.py`, the artifact
   under test, under a one-scored-live-per-generation budget.
2. Any stop superseded the generation, so each launch-wiring experiment cost a
   review round, a plan commit, a mailbox row, a worktree and a claim cycle.
3. G9's offline gate asserted that the launch source "contains all four stock
   `OnProcessExit` edges and contains neither `_on_exit_zero`". That suite
   passed three consecutive times and the live run still stopped at the same
   stage, because grepping the spelling of a fix cannot catch a blocked-call
   race.

Startup wiring is deterministic and cheap to reproduce: the rehearsal loop
completes an iteration in about 27 s. Twenty iterations cost nine minutes and
answered what three generations could not.

## Finding 4: metric review

The per-frame thresholds are sound and were kept. Four rules were not.

- **`0.95` is not the problem; the denominator was.** The system has a designed
  transition (`hold_track` plus `SupportStabilityFilter(window=5)`) producing
  roughly 19 non-`FULL_3D` frames. Once the window starts at the admitted
  support-ready boundary the tail scores 140/141 = 0.993. G4 fixed the start
  correctly, after two repairs. Note that the same transition is already bound
  by `t_steady <= t_proposal + 1.4s`, so counting transition frames in the rate
  denominator gated one physical behaviour twice.
- **The aggregation encodes an undeclared requirement.** Three slots by three
  sizes by two cases with one failure aborting is an AND over 18: `p=0.99`
  gives 0.84, `p=0.97` gives 0.58, `p=0.95` gives 0.40. The gate returns the
  same verdict for a 97 %-reliable system and a broken one. `AGENTS.md` asks
  for the aggregation rule, which was stated; the per-case reliability it
  encodes never was.
- **Seed supply was the binding constraint, not geometry.** Standard-class miss
  rate is 37.5 % (6 of 16). A valid known miss is unscored yet consumes a
  frozen seed with no substitution, and `standard_02` was frozen as available
  at confidence 0.256, just 0.056 above the floor, then missed live. PF-R8
  already owns detection availability as a closed subtask; PF-R7 certifies
  platform-free height and geometry.
- **C2 has never been evaluated.** `executor_lag` and RSS bars produced no
  samples in nine campaigns; every result reads "unscorable". A bar that has
  never yielded a sample provides no assurance.

## Actions taken

Launch fix committed at `0ff903b`: an observable `spawn_robot: create S20`
marker, a controller-manager watchdog that aborts with
`startup_failed: plugin_urdf_not_received` instead of leaving both spawners on
60 s timeouts, a persistent-client staged probe under one clock, and a
non-scored rehearsal loop. Behavioural regressions replay the captured G9 and
baseline logs: 27 passed on three consecutive runs, `luggage_gazebo` 109
passed.

The fix bounds the race; it does not remove it. Recovery depends on relaunch
being free, which generation 10 makes explicit. Verification ran 10/10 with the
watchdog never firing, so the relaunch path is unexercised in that sample; the
observed raw rate across both runs is one failure in twenty launches.

Generation 10 plan committed at `cbcdfe9` with the five acceptance changes.
`AGENTS.md` and `.cursor/rules/sim-lifecycle.mdc` gained rules against
certifying an artifact you are editing under a single-run budget, against
source-text-only offline gates, and for declaring the reliability an
aggregation encodes. Those two files were already dirty with an unrelated
in-flight documentation restructure, so the edits are left uncommitted rather
than folded into this lineage.

## Teardown

`scripts/stop_sim.sh` after every rehearsal attempt. Residual sim and bridge
processes 0, PID file absent, `/clock` publishers 0 before each launch and 1
after. Sim slot released.
