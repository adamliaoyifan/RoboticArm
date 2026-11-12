# 2026-11-12 -- Shared O snapshot and selected owner launch

- role: eng
- agent: codex
- model: gpt-6
- cli: codex
- status: done
- parent: MOVEIT-ATLAS-JOINT-PLAN-20261111
- subtask: ST-4-PRODUCTION-P1
- base_revision: 7db985cb
- started_at: 2026-11-12T15:10:17+08:00

## Summary

Extracted shared read-only map contracts/transport into luggage_mapping. Both
ComputePlacement and complete prepare now use the same O reader, exact owner
configuration and dimension-based paging. Integrated hardware placement selects
one O/G producer, default O. No nodes, maps, simulation or controllers started.

## Requirement

Plan e4f75646, generation2. Generation1 was superseded solely to replace mutable
HEAD with this exact plan hash. This bounded P1 prerequisite does not close G2
whole-chain integration or establish full O/G route/execution equivalence.

## Changed

- Shared immutable surface/source, common admission and O reader package, with
  old planning imports retained as explicit exports of identical class objects.
  Preserved pre-existing dirty map-refresh behavior; original inputs/hashes saved.
- ComputePlacement uses the same admitted reader/config bytes as prepare; removed
  its independent2cm default and64-page cap. Positive legacy resolution only
  asserts consistency; timeouts retain late lease cleanup and one deadline.
- One selected producer; immutable effective source geometry/configuration;
  source optical subscription requires explicit topic/frame. O service mode
  awaits real observations and does not claim the ULD is empty. G volume evidence
  is enabled, with its existing native transport. O frame graph omits nonexistent
  G peer/endpoints. Existing deployment-profile edits were retained.
- Test-only source runner prepends source paths without deleting ROS paths.

## Verification

85 behavioral tests pass: real launch callbacks/default O, O/G at1/2cm,
configuration tamper refusal, actual compute callback using shared capture,
same class identity,67-page4,344,244-cell fixture, bad-page refusal, admission,
runtime map refresh/journal regression and previous observer cleanup tests.
Initial collection failures are preserved: one nonexistent planned filename,
then an incorrectly replaced ROS Python path. Actual replacement tests/procedure
are in the retained logs; neither attempt ran a scored product case.

Fresh13-package production build10.07s, zero input changes;35/35 installed
checks and2/2 installed launch parses pass. Native67-RPC, actual O/G sensor input,
old selector compatibility, all-direction candidate parity and whole-cycle are
not_evaluated. These remain in later units; no success-rate/performance claim.

## Result

Bounded P1 selected-owner/shared-O-reader acceptance passes. Continue P2 shared
placement policy/four directed headings; full convergence remains open.

## Pointers

- docs/plans/PRODUCTION_CONVERGENCE_P1_EXECUTION_20261112.md
- docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-PRODUCTION-P1/run_01/manifest.json
- docs/agents/discuss/2026-11-12_1509_production-convergence-p1-g2.md
- src/luggage_mapping/README.md
