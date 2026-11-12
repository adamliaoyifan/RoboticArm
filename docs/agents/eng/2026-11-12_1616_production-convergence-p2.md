# 2026-11-12 -- Shared observed-volume placement and four directed targets

- role: eng
- agent: codex
- model: gpt-6
- cli: codex
- status: done
- parent: MOVEIT-ATLAS-JOINT-PLAN-20261111
- subtask: ST-4-PRODUCTION-P2
- base_revision: ea19c48c6f9cd9e0fa049a6ed3b3f2b6ba02dfff
- started_at: 2026-11-12T15:44:16+08:00
- completed_at: 2026-11-12T16:12:32+08:00

## Summary

ComputePlacement and complete prepare share a ROS-free geometry/observed-volume
query. Four actual quarter-turn targets survive generation, pool retention and
wire serialization. Corrected O 3D absolute origin; the earlier callback
reproducer and complementary occupied/unknown-layer cases now pass. No Gazebo,
controller, native endpoint process or real robot was started.

## Requirement

Plan fb4a0e4f generation1. User permits final face contact without penetration
or pushing, and explicitly chose corridor ranking with whole-trajectory motion
authority. Preserve motion uncertainty/release policies independently. This
bounded unit is not the full-model baseline or production-client switchover.

## Changed

- Added packing placement_query and placement_volume_query. Same-source complete
  box volume is required even above a floor prior. Unknown/inactive/uncovered/
  short data refuse; occupied cells are not eroded in the shared query.
- Integral volumes build once per snapshot. Default nominal side gap is zero;
  existing support tolerance, top/wall clearance, native collision inflation,
  payload envelope and release height remain separate and unchanged.
- Requested planned-box copies cannot establish occupancy before a measured
  ledger. Existing ledger agreement and commit-time overlap recheck retained.
- Straight corridor and box-center atlas vetoes become ranking diagnostics;
  actual bound-grasp atlas seeds and native trajectory gates retain authority.
- Shared retention reserves one deep candidate and one shallow fallback per
  directed heading before filling the remainder. Compute output retention uses
  the same rule. Prepare no longer invents half-turn variants. Requested directed
  targets match their own admitted quaternion (quaternion sign equivalence only).
- Updated normative placement documentation and focused callback fixtures.
  Preserved unrelated dirty scene-path cleanup in placed-copy test unstaged;
  its already-dirty original is frozen in evidence, not committed with P2.

## Verification

`scripts/production/run.sh python3 scripts/production/test_source.py -q
src/luggage_packing/test/test_placement_query.py
src/luggage_packing/test/test_placement_query_consumers.py
src/luggage_packing/test/test_placement_planner_validity_wiring.py
src/luggage_packing/test/test_placement_v2_placed_copy.py
src/luggage_planning/test/test_map_owner_launch.py
src/luggage_packing/test/test_placement_solver.py
src/luggage_packing/test/test_placement_volume_evidence.py
src/luggage_planning/test/test_placement_prior_requests.py
src/luggage_planning/test/test_placement_prior_prefetch.py
src/luggage_planning/test/test_joint_operation_observer.py
src/luggage_planning/test/test_joint_operation_candidates.py
src/luggage_planning/test/test_joint_operation_planner.py
--junitxml=<root>/tests-final.xml`: 162 pass, no skips. The existing volume
input fixture constructs/destroys in-process rclpy nodes/executor; this is more
than the plan's literal no-node statement, but no external stack ran. Other
consumer tests call real methods on explicit fixture state.

1200 deterministic zero-erosion scalar comparison cases: 714 accepted, zero
admission mismatches. All failure attempts/logs and exact fixture sources retained.
Synthetic 1cm149x197x148 all-free timing only: prefix0.0696s/35,402,400bytes;
61,480 candidate enumeration/admission4.033s; 2000 retained with four headings;
process max RSS262,736KiB. No measured-scene/performance-rate or motion claim.

`scripts/production/build.sh --evidence <root>/build-closed` passes13 packages;
source before/after hashes unchanged. `scripts/production/run.sh python3
scripts/production/check_install.py --output <root>/install-closed.json`:
37/37 installed checks pass. Source/code diff whitespace checks pass.

## Result

Pass at fee50590. P3 regional-access contracts, P4 default bound-operation
client/budget, and P5 full-model simulated cycle remain open. During this unit
user confirmed first obtain a complete successful baseline and stage timings,
then define production latency thresholds; do not preserve an unexplained10s
hard maximum as the baseline success criterion.

## Pointers

- docs/plans/PRODUCTION_CONVERGENCE_P2_EXECUTION_20261112.md
- docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-PRODUCTION-P2/run_01/
- docs/agents/discuss/2026-11-12_1544_production-convergence-p2.md
