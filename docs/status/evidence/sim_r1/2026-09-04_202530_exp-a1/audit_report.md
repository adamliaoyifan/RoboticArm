# EXP-A1 NBV Baseline Readiness Audit

- parent: SIM-R1-20260904
- subtask: EXP-A1
- base_revision: 408f6d5dd9aeb8536377f0f14d06155be7a904d5
- audit_outcome: pass
- nbv_readiness: blocked
- completed_at: 2026-09-04T20:33:36+08:00

## Snapshot

The audit ran against an immutable `git archive` snapshot:

- audit root: recorded in `audit_root.txt`
- base revision: recorded in `base_revision.txt`
- no ROS, Gazebo, MoveIt, or simulator was started
- production source was not modified by the audit

## Existing Focused Tests

The requested focused pytest command was run and retained in:

- `focused_pytest.log`
- `focused_pytest_rosenv.log`

Both attempts failed during collection because the audit environment cannot
import `ament_index_python` from `luggage_description._share`. This was treated
as retained evidence and did not stop the mandatory probes.

## Static Boundary

- `privileged_dependency_scan.txt`: 0 matches for ROS/Gazebo/GT/spawner terms
  in the audited planning modules.
- `geometry_contract_scan.txt`: 99 geometry/candidate/stamp-related matches
  used to classify current contract coverage.

## Mandatory Probe Summary

Probe results are machine-readable in `probe_results.json`:

- PASS: fresh process determinism.
- FAIL: `interior_view_scorer` is candidate-specific, but
  `CargoNBVPlanner._coverage_score()` ignores candidate values.
- FAIL: NaN/Inf poses and termination cases are stable, but duplicate
  candidate IDs, candidate dimensions, and quaternion validation are not
  enforced by the baseline contracts.
- PASS: synthetic corridor candidate poses are finite and carry normalized
  orientation plus feasibility fields.
- PASS: seven-face hull containment rejects a point inside AABB but outside the
  chamfered usable hull.
- BLOCKED: reachability/layout atlas checks are deterministic for existing
  scene/URDF hashes, but schemas lack production `geometry_hash`,
  `map_revision`, and source acquisition stamp correlation.
- BLOCKED: no audited component directly implements
  `ExplorationPolicy.reset/propose/observe`; SIM-R1-2 needs an adapter and
  coordinator layer.

## Reuse Matrix

| Module | Classification | Evidence |
|---|---|---|
| `cargo_nbv_planner.py` | unsafe for production as-is | deterministic, but `_coverage_score()` is not candidate-specific and `observe()` is absent |
| `geometry_view_generator.py` | needs SIM-R1-2 adapter | emits plain candidates; coordinator must enforce TCIG and correlation gates |
| `interior_view_scorer.py` | reusable behind hard gates | candidate-specific scoring works in the probe |
| `smart_explore_termination.py` | needs SIM-R1-2 adapter | termination helpers are stable, but metric absence can fail open |
| `interior_probe_planner.py` | needs SIM-R1-2 adapter | deterministic ranking; duplicate-ID and immutable snapshot gates are caller-owned |
| `reachability_atlas.py` | needs SIM-R1-2 adapter | deterministic lookup/version checks; lacks geometry hash/map revision/stamp contract |
| `layout_atlas.py` | offline/coordinator input only | deterministic grid compatibility and layout scoring helpers |

## Readiness Decision

`audit_outcome=pass`: the requested audit was completed with retained evidence.

`nbv_readiness=blocked`: SIM-R1-2 may reuse parts of the baseline, but it must
add a coordinator/policy adapter, candidate-specific CargoNBV gain repair,
duplicate-ID/quaternion/dimension hard gates, and geometry hash/map
revision/stamp correlation before claiming production readiness.

## Hygiene

- `git diff --check`: pass
- `scripts/check_agent_contract.sh`: pass
- primary workspace status saved to `primary_status_after.txt`
