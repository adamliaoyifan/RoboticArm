# Direct portal first, optional staging fallback

User goal: first direct portal, then consider staging; run two closed-loop
validations. Base b97b489c, integration worktree agent/eng/integration-prior-g1.
Owner eng/codex/gpt-6; parent PHASE2-20261022, subtask DIRECT-FIRST generation1.

## Behavior and acceptance

Direct home→portal and portal→home topology is attempted first. Do not generate
staging IK or issue staging validity calls before that attempt fails. When a
staging baseline exists, allocate60% of remaining candidate time to direct
search; staging preparation and fallback share the residual of the original
candidate deadline. No new overall Select budget. Without staging available,
direct receives all remaining time. Failed or unevaluated staging endpoints
cannot enter fallback; failure does not retroactively invalidate direct success.
Both graph and lattice paths must respect this order. Preserve frozen payload,
scene construction, full-route certification, expiry and collision allowances.
Record topology attempts, budgets, failure reason and selected topology in rows.
Existing scene-pose and nominal-height issues remain independently documented;
this change isolates route topology policy.

## Verification

Behavioral tests: direct success produces no staging generation/check; direct
failure tries staging only with remaining budget; invalid/missing endpoints
never connected; exhausted budget never starts fallback; graph and lattice
factory selection receives appropriate staging scope. Existing route/certifier
suite must retain pass status, baseline failures separately reproduced if any.
Run pytest for new module, staging candidates and tests/st09_parallel/I route
producer; build luggage_planning, confirm installed/source byte identity.

Then freeze implementation revision and run two fresh n=3 suites, seed0,
A-prime scene,180s diagnostic Select budget, prior off, unchanged margins/gain.
Use PHASE2_RUN_OUT=<evidence>/live_01 (then live_02) bash
scripts/g3/n3_functional_run.sh. Each suite stops at its first product failure.
Exactly two product attempts; startup-only failures consume no attempt and
are bounded by20minutes total rehearsal time. Single Gazebo, clean teardown
between runs; preserve manifests/configs/URDF/selection request+response+scene,
trial/suite/trace and failure artifacts before retry. No code changes between
product attempts. Record per-suite attempted/picked/packed/certified counts,
reason, topology traces and latency. Functional pass is3 packed and verified
with occupancy commits per suite,2/2 suites pass to claim repeated closure.
This is6 prescribed box units if both complete, not a calibrated reliability
claim; report per-box results and n, do not infer general success probability.
If both block, report task validation outcome as failed; do not silently run a
third or change thresholds. Code delivery plus two faithfully reported runs
completes the requested experiment even if closed-loop acceptance is blocked.
