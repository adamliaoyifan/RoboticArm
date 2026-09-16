# 2026-09-16 -- Current deployment readiness review

- role: reviews
- agent: codex
- model: gpt-5
- cli: codex
- status: done

## Summary

At assessed `master` revision
`64a57367ba15f8cc6fa7f16165ef12b559986a6f`, the repository supports
supervised sensor, detection, plan-only, single-pick, vacuum, simulation, and
offline/place-only engineering workflows, but it is not production-ready for
an autonomous real loading cycle. The pure production state contract is sound,
while the current runtime remains a ROS 1 catkin orchestrator excluded from
colcon; accepted ROS 2 orchestrator and exploration-policy revisions are not
ancestors of `master`. Real trajectory safety repairs are present and verified
offline, but every H0--H5 hardware qualification row remains TODO, stop-start
continuity is unresolved, and no real placement or complete orchestrated cycle
has passed. The assessed worktree had 10 dirty entries, so it is not a release
artifact.

The single-box pickup procedure is explicit but not a general autonomous pick
selection policy: it uses the first detector result and its valid top surface.
Its diagnostic driver releases vacuum on a later segment failure or process
exit, which is incompatible with the production carry-fault invariant and must
not be treated as the production orchestrator. Placement candidate generation
and hard constraints are substantially clear and have place-only simulation
evidence, but the policy is not singular: the ROS 2 service calls
`placement_solver.solve_placement`, while the offline path described as the
production scorer calls `placement_scoring.score_candidates`. Real sensor/map
integration, failure semantics, commit/verification, and hardware execution
also remain unqualified.

## Risks

- Immediate physical blocker: complete H0--H5, including a buffered controller
  backend if the measured internal stop gaps remain at or above 80 ms.
- Immediate system blocker: integrate the accepted ROS 2 orchestrator and
  exploration policy, then implement the missing stamped depth/map action,
  backend parity, and end-to-end SIM-R1 stages.
- The current hardware pick driver is commissioning tooling, not a safe
  carrying-fault controller; automatic vacuum release can drop a payload.
- Placement currently conflates invalid input/no feasible candidate with
  `BIN_FULL`; the online and offline scoring paths differ; and full
  geometry-hash/map-revision orchestration is incomplete.
- Real Gate 5 remains unaccepted; the last recorded cell attempt failed at
  attach before vacuum enable.

## Pointers

- `docs/architecture/production_orchestration.md`
- `docs/plans/sim_r1_production_orchestrator_exploration.md`
- `docs/plans/elfin_real_trajectory_hardware_acceptance.md`
- `docs/status/20260915_gate5_night_summary.md`
- `docs/agents/eng/2026-09-10_1624_sim-r1-5-ros2-orchestrator.md`
- `docs/agents/eng/2026-09-14_1035_sim-r1-2-exploration-policy.md`
- `docs/agents/eng/2026-09-14_2134_place-only-perfect-geometry.md`
- `src/luggage_planning/scripts/hardware_pick_driver.py`
- `src/luggage_packing/luggage_packing/placement_solver.py`
- `src/luggage_bringup/scripts/orchestrator_node.py`
