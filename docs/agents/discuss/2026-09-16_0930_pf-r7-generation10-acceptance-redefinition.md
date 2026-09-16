# 2026-09-16 -- PF-R7 generation 10 acceptance redefinition and bounded startup recovery

- status: open
- to_role: test
- to_agent: cursor
- to_model: grok-4.6
- kind: integration
- parent: PFH-REMEDIATION-20260904
- subtask: PF-R7
- depends_on: PF-R1,PF-R2,PF-R3,PF-R4,PF-R5,PF-R6,PF-A1,PF-R8,PF-R9,PF-R10
- revision: 0ff903b
- generation: 10
- plan_revision: cbcdfe93671733658a034ee3b5ed4d6dc02edd5e
- dispatch_ready: yes

## Post -- reviews/cursor -- 2026-09-16 09:30 -- cursor/opus-5

Generation 10 supersedes blocked G9 under direct user authority. It is the
first generation permitted to change acceptance aggregation, seed supply, C2
sampling and the infrastructure budget. Per-frame product thresholds are
unchanged: `0.20`, `0.95`, `>=4 Hz`, admitted 5/5 support-ready `t_steady`,
`[t_steady, t_steady + 8.000s)`, `t_* <= t_proposal + 1.4s`, and every G4
geometry P95/max bar.

The infrastructure root cause is now measured rather than inferred, and the
launch fix is committed at `0ff903b`. Implement the plan's five acceptance
changes, prove them offline, then run the campaign.

## Pointers

- `docs/plans/pf_r7_generation10_acceptance_redefinition.md`
- `docs/status/evidence/platform_free_height/2026-09-16_startup_rehearsal/RESULT.md`
- `docs/agents/reviews/2026-09-16_0930_pf-r7-nine-generation-audit.md`
- `docs/agents/discuss/2026-09-15_2210_pf-r7-generation9-robot-spawn-wiring.md`
- `.cursor/rules/sim-lifecycle.mdc`
- `.cursor/rules/debug-evidence.mdc`

## Open

- Implement the five acceptance changes, prove them with behavioural offline
  regressions, then run the G10 campaign end to end.

## Reply -- reviews/cursor -- 2026-09-16 09:30 -- cursor/opus-5

Root cause of the three infrastructure generations, established by
measurement, not inspection. `scripts/sim_startup_rehearsal.sh` ran the stock
master launch ten times: nine ready stacks, one failure, and the failing stage
was `plugin_urdf_not_received`. In that iteration the plugin logged
`connected to service!!` and then neither `Received URDF from param server`
nor its own retry line, which is only reachable by blocking inside the `get()`
whose preceding `wait_for(5s)` result is discarded. `gz_ros2_control` 0.7.21
has no other URDF source.

`robot_state_publisher` answered in at most 1.21 s in every iteration
including the failing one. G7, G8 and G9 all tried to make the spawn wait
longer for RSP, so none of them could ever have worked. G8 additionally never
scheduled the S20 create at all, and G9 measured a 2 s spawn deadline against
an eval-side probe it did not own.

The committed fix does not remove the race; it bounds it. The launch now emits
`spawn_robot: create S20` and aborts with
`startup_failed: plugin_urdf_not_received` within `startup_watchdog_sec`
instead of leaving both spawners on their 60 s timeouts. Recovery comes from
relaunching, which the plan now makes free before the first scored case.

Verification ran ten more iterations on the fixed launch: 10/10 ready, the
watchdog never fired, `plugin_urdf` max 3.21 s. Stated plainly so it is not
over-read: the race did not recur in that sample, so the relaunch path is
unexercised there; across both runs the observed raw rate is one failure in
twenty launches. The abort path is covered offline instead, by the watchdog's
exit code against an empty graph and by unit tests of the shutdown handler.

Offline: `src/luggage_gazebo/test/test_startup_probe.py` 27 passed on three
consecutive runs; full `luggage_gazebo` suite 109 passed. Two of those tests
replay the captured G9 `ready.json` and the captured baseline failure and
reproduce their exact classifications, which is the level G9's source-text
assertions could not reach.

Teardown: `stop_sim.sh` between every attempt, residuals 0, PID file absent,
`/clock` publishers 0 before launch and 1 after.
