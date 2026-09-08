# EXP-A1 NBV Baseline Readiness Dispatch

- role: reviews
- agent: codex
- model: gpt-5
- cli: codex
- status: done
- parent: SIM-R1-20260904
- subtask: EXP-A1
- revision: 408f6d5dd9aeb8536377f0f14d06155be7a904d5

## Summary

`EXP-A1` is an independent, non-production audit assigned to
`codex/gpt-5`. It determines which existing ROS-free exploration components
can be reused by `SIM-R1-2`, and records reproducible blockers before that
implementation checkpoint starts. It must not repair the components it audits
or edit the active `SIM-R1-1` files.

## Scope

Audit these production modules and their focused tests at the exact base
revision:

- `luggage_planning/cargo_nbv_planner.py`
- `luggage_planning/geometry_view_generator.py`
- `luggage_planning/interior_view_scorer.py`
- `luggage_planning/smart_explore_termination.py`
- `luggage_planning/interior_probe_planner.py`
- `luggage_planning/reachability_atlas.py`
- `luggage_planning/layout_atlas.py`

Read the normative policy boundary in:

- `docs/architecture/production_orchestration.md`
- `docs/plans/sim_r1_production_orchestrator_exploration.md`
- `docs/plans/true_container_inner_geometry.md`

Do not edit production source, existing tests, launch files, interfaces, or
configuration. New scripts used only for this audit stay inside the evidence
directory. Do not start ROS, Gazebo, MoveIt, or a simulator.

## Execution Commands

Create an immutable source snapshot so concurrent work in the primary tree
cannot contaminate results:

```bash
set -euo pipefail
export SOURCE_ROOT=/home/adamliao/work/elfin_humble_ws
export BASE_REV=408f6d5dd9aeb8536377f0f14d06155be7a904d5
export RUN_ID="$(date +%Y-%m-%d_%H%M%S)_exp-a1"
export EVIDENCE="$SOURCE_ROOT/docs/status/evidence/sim_r1/$RUN_ID"
export AUDIT_ROOT="$(mktemp -d /tmp/elfin_exp_a1_408f6d5.XXXXXX)"
mkdir -p "$EVIDENCE"
git -C "$SOURCE_ROOT" archive "$BASE_REV" | tar -x -C "$AUDIT_ROOT"
test "$(git -C "$SOURCE_ROOT" rev-parse "$BASE_REV")" = "$BASE_REV"
printf '%s\n' "$BASE_REV" > "$EVIDENCE/base_revision.txt"
printf '%s\n' "$AUDIT_ROOT" > "$EVIDENCE/audit_root.txt"
cd "$AUDIT_ROOT"
```

Run the existing ROS-free baseline suite and retain full output:

```bash
export PYTHONPATH="$AUDIT_ROOT/src/luggage_planning:$AUDIT_ROOT/src/luggage_description"
set -o pipefail
python3 -m pytest -q \
  src/luggage_planning/test/test_geometry_view_generator.py \
  src/luggage_planning/test/test_interior_view_scorer.py \
  src/luggage_planning/test/test_floor_coverage_metrics.py \
  src/luggage_planning/test/test_interior_probe_planner.py \
  src/luggage_planning/test/test_reachability_atlas.py \
  src/luggage_planning/test/test_layout_atlas.py \
  src/luggage_planning/test/test_smart_explore_termination.py \
  2>&1 | tee "$EVIDENCE/focused_pytest.log"
```

Record the static dependency boundary. Every hit must be classified; comments
or eval-only references are not automatically failures:

```bash
rg -n \
  'rclpy|rospy|tf2_ros|gazebo|model_states|GetCurrentBox|SpawnNextBox|ground[_ -]?truth|spawner' \
  src/luggage_planning/luggage_planning/{cargo_nbv_planner,geometry_view_generator,interior_view_scorer,smart_explore_termination,interior_probe_planner,reachability_atlas,layout_atlas}.py \
  > "$EVIDENCE/privileged_dependency_scan.txt" || true

rg -n \
  'point_inside_container_inner_box|container_usable_dimensions|container_opening|hull|geometry_hash|map_revision|stamp|candidate' \
  src/luggage_planning/luggage_planning/{cargo_nbv_planner,geometry_view_generator,interior_view_scorer,smart_explore_termination,interior_probe_planner,reachability_atlas,layout_atlas}.py \
  > "$EVIDENCE/geometry_contract_scan.txt" || true
```

Create an evidence-local ROS-free probe, for example
`$EVIDENCE/exp_a1_probe.py`, and run it against the immutable snapshot:

```bash
python3 "$EVIDENCE/exp_a1_probe.py" \
  --repo "$AUDIT_ROOT" \
  --scene src/luggage_description/config/scene_tf.yaml \
  --json-out "$EVIDENCE/probe_results.json" \
  2>&1 | tee "$EVIDENCE/probe.log"
```

The probe must exercise all of the following with synthetic, deterministic
plain data:

1. Same inputs in two fresh Python processes produce byte-identical candidate
   IDs, poses, scores, selected index, termination reason, and diagnostics.
2. Two candidates with different visible unknown cells receive different
   information-gain values and the higher-gain feasible candidate ranks first.
   Exercise `interior_view_scorer` and separately expose whether
   `CargoNBVPlanner._coverage_score()` is candidate-specific.
3. Invalid candidate dimensions, NaN/Inf pose values, malformed quaternions,
   repeated candidate IDs, exhausted views, unknown-threshold completion, and
   stagnation are classified with stable outcomes.
4. Generated camera poses are finite; quaternion norm is within `1e-6` of one;
   tilt and opening-aperture constraints are checked for each candidate.
5. A synthetic point inside the AABB but outside the true seven-face hull is
   rejected by the exploration containment path. If no such path exists, mark
   the requirement `BLOCKED`, not `PASS`.
6. Reachability/layout atlas lookup behavior is deterministic for valid,
   missing, stale, and geometry-mismatched descriptors. If the current schema
   lacks hash/revision correlation, record the exact missing fields.
7. Map every existing component to `ExplorationPolicy.reset`, `propose`, and
   `observe`, and list the adapter work still required by Gate R2. Do not claim
   that a function exists unless its call site and data contract are shown.

Finish with repository hygiene checks from the primary workspace:

```bash
cd "$SOURCE_ROOT"
git diff --check
scripts/check_agent_contract.sh
git status --short > "$EVIDENCE/primary_status_after.txt"
```

## Acceptance

- `probe_results.json` records `base_revision`, Python version, test command,
  and one row for every mandatory probe with `PASS`, `FAIL`, `BLOCKED`, or
  `NOT_APPLICABLE`, plus a concrete evidence pointer.
- The report contains a module-by-module reuse matrix: reusable as-is, needs a
  `SIM-R1-2` adapter, superseded by TCIG, or unsafe for production.
- The report explicitly decides whether information gain is candidate-specific,
  whether true seven-face containment exists, and whether hash/revision/stamp
  correlation is present. Unknown evidence fails closed as `BLOCKED`.
- Existing focused tests pass, or every failure is reproduced and retained in
  evidence. A pre-existing failure does not permit omission of later probes.
- No ROS graph or simulator is started and no production file is modified.
- A test role note points to the evidence directory and separately reports
  `audit_outcome` and `nbv_readiness`. `audit_outcome=pass` means the audit is
  complete; it does not mean `nbv_readiness=ready`.
- The canonical thread is closed by `scripts/agent_complete.sh` with an exact
  evidence revision. Any readiness blocker is returned to `reviews` in the
  same thread and becomes input to `SIM-R1-2`; this owner must not fix it.

## Risks

- The current greedy planner appears to use the same frontier count for every
  candidate. The audit must reproduce or refute this from executable evidence.
- Existing geometry helpers may enforce an axis-aligned inner box rather than
  the approved seven-face interior. Do not treat later TCIG intent as current
  implementation evidence.
- `smart_explore_termination` may intentionally fail open when a metric is
  absent. Record where the coordinator must own the corresponding hard gate.

## Consensus

- Codex agent: existing `SIM-R1-20260904` formal consensus
- Thread: `docs/agents/discuss/2026-09-04_1801_sim-r1-gpt55-consensus.md`
- Result: reached; this separately requested audit does not amend production
  behavior or the approved subtask ownership

## Pointers

- `docs/architecture/production_orchestration.md`
- `docs/plans/sim_r1_production_orchestrator_exploration.md`
- `docs/plans/true_container_inner_geometry.md`
- `docs/agents/discuss/2026-09-04_2021_exp-a1-nbv-readiness-audit.md`

## Open

- None. Dispatch is authorized by the user.
