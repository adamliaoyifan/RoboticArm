# Placement prior and grasp-offset integration G1

## Objective and constraints

Implement preselected box placements and read-only background priors, propagate
selected grasp offsets into real-cell pickup and certified place planning, first
prove functional recovery of a sensor-colliding target and then measure planning
efficiency. User authorization 2026-10-16. No physical hardware motion is included
in development commands; hardware planning interfaces must be exercised without
execution. Simulation execution is used for whole-chain functional validation.
Preserve measured-geometry/attachment provenance, safety thresholds, occupancy
ownership, full-route certification, vacuum and recovery gates.

Prior worker reads immutable versioned OCC snapshots, never updates OCC/shared
PlanningScene/ACM/attachment. Hypothetical dimensions are explicitly speculative,
never fabricated detection or measured attachment. Adopt only after measured
geometry, chosen suction candidate, scene/model identity and actual start-state
checks. Cache miss/stale/error falls back to ordinary certified planning. Select
grasp before pickup; keep actual attachment authoritative after pickup. Fixed
box orientation representation must persist across pick and place.

Base a920f4bd; isolated worktree /tmp/elfin-placement-prior-g1, independent
build-prior/install-prior. Other agents' dirty primary files are not inputs or
edit targets. Coordination stays in primary docs/agents. Missing deploy inputs
must be recorded, not silently imported from mutable other worktrees.

## Owners and units

| Unit | Owner | Dependencies | Scope |
|---|---|---|---|
| CORE | eng/codex/gpt-6 | none | Snapshot/prior contracts, rigid-tool collision screening, grasp candidate selection, bounded async worker, real-cell caller/selector integration, focused tests |
| INTEGRATION | eng/codex/gpt-6 | CORE | Native saved-scene recovery, real-cell plan-only boundary replay, simulation execution, paired efficiency and resource evaluation |

Sequential ownership; no child agents. Exact plan commit is pinned by dispatch.
No changes to the bounded-Cartesian G1 acceptance or other ongoing work.

## Functional acceptance

F0 -- automated behavioural tests: python3 -m pytest focused new prior tests plus
existing pick-frame, suction session, attachment, typed-driver and selector tests.
Every test must pass. Cover rotated pickup/place boxes, nonzero lateral grasp,
pi representative consistency, real contact-candidate identity, full tool rather
than TCP-only collision, unchanged target box pose, stale map/model/attachment,
new snapshot superseding work, cancellation/deadline, concurrent requests,
background exception, cache miss fallback, and zero scene/occupancy mutations.
Test fake ports record calls; source-string checks do not establish behaviour.

F1 -- native exact fixture: original req-1a/edge-0000 and full scene/model from
bounded_cartesian/g1/acquisition_20261016_1025. Reproduce original partial path
and radar/container collision, then select a supported offset that removes it
without changing box target. Capture full candidate list, transform chain,
source provenance and every rejected contact. All final robot/payload/tool
collision, endpoint and timing constraints must pass; partial fraction is not
success. An inability to recover is a failed requirement, not a passing filter.
Additional measured suction-patch evidence is required to authorize an actual
pick; synthetic patches can exercise transforms but cannot claim measured pick.

F2 -- hardware plan-only integration: real hardware launch/caller configuration
with execution disabled, actual service boundaries and generated v2 messages.
Selected candidate contact must reproduce G exactly (1e-9 m numerical compose
oracle), propagate through pickup waypoint creation and measured attachment
admission to requested box placement (existing endpoint tolerances unchanged).
Exercise cache hit and miss, plus stale snapshot and candidate-retry cases;
no stale result may reach execution admission. Save request/response CDR,
scene/model/calibration identity and emitted executable route. ROS-free tests
alone cannot satisfy this gate.

F3 -- simulation full functional chain: serial exclusive stack; rehearse readiness
before scoring. Freeze two measured cases before scoring: the original geometry
regression with a noncentral valid grasp, and a second box at a rotated pickup
pose with occupied placement scene. Three independent cycles per case (6 total),
all must complete pick/place/release/verified occupancy, maintain requested box
pose within existing verifier tolerance, and have zero contact/endpoint/safety
violations. Report each unit and empirical rate; 6/6 is a regression gate, not a
statistical proof of fleet reliability. If the collision path never occurs in
baseline, label recovery not_evaluated and acquire required evidence first.

## Performance evaluation (after F0-F2; final acceptance also requires F3)

Freeze six exact measured planning workloads spanning two map snapshots, three
placement/grasp situations (central feasible, offset required, infeasible) and
at least two pickup orientations. Preserve inputs before timing. For each case
run five paired repetitions, alternate baseline/prior arm order, same unchanged
runtime commit, hardware, isolated environment and policy budgets: 60 s Select,
existing 2 s certification/5 s client watchdog; bound prior preparation to 10 s
and 32 candidates per request, report exhaustion. Thirty pairs are a case matrix,
not an AND reliability claim. Never omit infeasible/time-out cases from tables.

Measure prior preparation wall/CPU time, resident memory, cache hit/fallback,
post-pick exposed planning latency, full request latency, collision/IK/service
counts, success/complete-route equivalence and certification duration. Separate
cold prior preparation from warmed adoption; do not claim savings by hiding
preparation cost. Simulate overlap using measured readiness/pick timing and also
report zero-overlap cost. Case-wise median and p95 across pooled samples plus
raw samples; compare paired differences. Successful warm-prior workloads target
>=10% lower median exposed latency and <=10% p95 regression, no loss of feasible
cases or safety regressions. If unavailable samples (<5 per arm/case) report
not_evaluated. If benefit misses target, report no demonstrated improvement and
profile/revise rather than claim speedup. Report CPU/memory overhead even if
latency improves. Scope statistics to these frozen workloads.

## Commands, artifacts and revision discipline

Implement reproducible scripts under scripts/placement_prior/:
- functional_replay.py: F1/F2 exact fixture and hardware-plan-only callback chain.
- benchmark.py: frozen workload manifest, paired trial procedure above.
- simulation_eval.py: F3 orchestration using existing exclusive lifecycle tools.
Each script supports --help and explicit --manifest/--output; no implicit motion.
Simulation execution requires its explicit flag/profile; no hardware execution.
Build with colcon --build-base build-prior --install-base install-prior, required
packages only; run behavioural tests before native/service/whole-chain stages.
Commit runtime before scored runs; require clean isolated runtime and matching
installed-module hashes. Evidence scripts may instrument but not mutate scored
artifacts. Rehearsal failures consume no scored sample but are retained.

Evidence root docs/status/evidence/placement_prior/g1 in primary workspace.
T0 manifest includes revision, dirty state, versions, commands, start/end,
workload/seed/sample identity, timings and teardown. T1 trace each OCC snapshot,
prior job, selected grasp/slot, pick/attachment/place boundary. Failed T2 contains
exact request/responses, robot model, frozen scene/OCC, candidate list, FK/IK,
contacts, controller and verification outputs. Persist before rerun/teardown.
Missing boundaries require instrumentation before another scored attempt.
Final report must separate functional recovery, hardware planning integration,
simulation placement and efficiency; none may stand in for another.

## Post-release observation rehearsal repair

Exact saved-frame replay in `mapper_replay01` showed that saturated free voxels
need at least three consistent fresh hits under the existing log-odds settings.
Each executed observation pose now admits at most four distinct depth acquisitions,
waiting past its cached frame and chaining the owner's map revision. A 15-second
budget gates starting each acquisition/integration; an in-flight call retains its
existing service watchdog. Stop at the existing confirmation criterion or refuse
on unavailable/duplicate input, failed integration, or exhaustion. Explicit depth
crop failure cannot fall back to an unrelated semantic frame. Each frame retains
its own raw depth and integration trace. This is an unscored rehearsal repair;
confirmation thresholds, OCC ownership and F3 acceptance remain unchanged.

The fresh_views_cycle02 rehearsal exposed a 117.295207-second certified transit
submitted with a 60-second result wait. Certified place execution now uses the
same duration-plus-existing-response-overhead rule as observation execution,
without changing planning or certification budgets or controller tolerances.
Missing/invalid certified times refuse before submission. Submitted-leg evidence
records response_wait_s. This addresses client premature cancellation only;
long-path selection cost and full-cycle success still require measurement.

Unscored single-cycle reporting uses `scripts/placement_prior/simulation_eval.py
--trials <cycle/trials.jsonl> --output <new-report.json>`. It preserves original
legacy two-box rehearsal output, recognizes typed route verification, and reports
motion/measured-OCC coverage separately. `f3_campaign` and `performance` remain
`not_evaluated`; a one-case report cannot pass the frozen six-cycle acceptance.
