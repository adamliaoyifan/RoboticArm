# 2026-11-13 -- Tolerant geometry and G/O consumer review

- role: reviews
- agent: codex
- model: gpt-6
- cli: codex
- status: done

## Summary

User directs impact-based tolerance and whole-chain completion before generic
perception improvements. Reviewed current sources and the existing containment
boundary evidence; changed no perception, planning, mapper or runtime policy.
G/O have shared placement geometry/evidence semantics but are not interchangeable
whole-operation backends. Eight scripted G snapshot assembly tests pass; no live
G execution, new simulation, actual-error calibration or performance campaign ran.

## Findings

- Source review point: local master `6bf14c53`, with unrelated concurrent dirty
  changes retained. This is a source inspection, not a certified installed build.
- Existing sampled simulation containment failure remains valid as a model
  coverage observation, not a measured hardware error bound or physical collision.
  Boxes 2/3 maximum envelope excess: 6.630/6.143 mm. Actual collision dimensions
  are 550x400x250 mm; detected widths/depths underestimate those dimensions by
  47.339--54.451 / 30.232--40.699 mm across all three boxes. Do not call these
  different quantities the same error. The observable lid reference is separately
  515.886x390.038x237.463 mm. Missing raw synchronized perception windows prevent
  decomposition into visibility, masks, fit, pose and history contributions.
- Current estimator fits a robust rectangle to top-plane inliers. This is a
  source-established reason that top support and whole-body extent need separate
  meanings; it is not proof of the sole root cause or of hardware incidence.
- Impact proposal: retain the original goal/path when refreshed geometry still
  satisfies current placement and full-path checks. A measured/modelled error
  should consume available clearance, not trigger a universal millimetre deadband.
  Prefer another measured grasp, orientation, branch, slot or targeted observation
  when clearance is affected. No new threshold or observation requirement enabled.

## G/O source audit

| Boundary | Current code | Remaining difference |
|---|---|---|
| Owner selection | Hardware placement uses `occupancy_backend=observer|grid_g`, exactly one producer; same resolved deployment resolution | Gazebo whole-operation wiring explicitly constructs O; hardware placement switch is not a whole-chain switch |
| Input | Both accept PointCloud2-derived geometry and acquisition-stamped TF | O has optical + separate physical LiDAR ingress; G buffers semantic cargo cloud and integrates through a settled-view service/Store transaction; it is not the same raw sensor stream |
| Integration | O owns C++ OctoMap; G owns grid log odds + geometry/instance/confirmation state through Store | Filtering, ray selection, source/confirmation policies are distinct; shared resolution does not imply equal maps |
| Floor filter | G `integrate_min_z_above_floor` defaults to 0.05 m | Existing point-input filter, distinct from obstacle inflation or inter-box spacing; requires separate review before equivalent ingress |
| Placement | Both adapt native source grids into the common surface and `KnownFreeVolume` predicate | G standalone default `volume_grid_g_cells=False` returns surface-only; integrated hardware placement explicitly enables True |
| G dense transport | `query_cells_dense_v2` exists in current main-tree producer and consumer | Earlier typed-page expiry is not a current absence of dense export. Stale comments and live test default `/tmp/elfin-occ-grid-consumer-compat-g1` must not serve as current install qualification |
| Snapshot binding | G surface and dense responses must have equal full identity | First dense request opens `use_current=True` rather than using the retained surface token. Concurrent update can cause explicit mismatch/refusal; not proof of silent map mixing |
| Prepare | O single capture; G compatibility path uses confirmed surface topics + exact-identity `GetCargoVolumeV2` | G is a separate compatibility capture path, with possible surface/current-volume races; no shared source lifecycle yet |
| Execution | `runtime_ports.py` constructs `ObserverSnapshotPort` directly | G cannot yet provide all refresh/release/adoption functions merely by changing map_backend |
| Provenance wire | `OperationMapSourceV2` / source conversion requires producer observer and uint64 epoch | G UUID epoch cannot be losslessly represented by that field; needs versioned backend-neutral identity, not hashing/truncation/coercion |
| Regional reads | Shared immutable source-grid regions/blocks exist; O reader can page an explicit region | G dense query rejects regions; legacy full volume is full transfer. Slicing after transfer saves no network. Neither enables candidate-scoped whole-chain certification yet |
| Initial condition | O source-bound explicit initially-empty transaction exists | Equivalent G session initialization/provenance has not been demonstrated; do not use planned-box commits or floor prior as its substitute |

Priority for G is consumer/identity/lifecycle alignment, then input equivalence,
then measured transport/fusion optimization. It need not reproduce OctoMap's
internal representation or byte-identical probabilistic estimates.

## Verification

- Sourced `/opt/ros/humble/setup.bash` and `install-production/setup.bash`;
  prepended source mapping/packing/perception/planning packages; disabled pytest
  plugin autoload; ran:
  `python3 -m pytest -q src/luggage_packing/test/test_placement_grid_g_snapshot_compute.py`.
- Result: 8 passed in 0.21 s. Scripted services cover layout/known-free evidence,
  wrong epoch/hull, page identity drift, short data and explicit surface-only mode.
  This does not validate native services, update races or complete G execution.
- Source/containment review only; no runtime behaviour changed, no Gazebo started.

## Pointers

- docs/status/evidence/tolerant_geometry_go_review/20261113/grid_g_snapshot_pytest.log
- docs/plans/ATLAS_ROUTE_TOLERANCE_EVALUATION_20261113.md
- docs/status/evidence/geometry_plan_reuse/20261113/REPORT.md
- docs/status/evidence/geometry_plan_reuse/20261113/simulation_01/payload_containment_eval_v5.json
- src/luggage_planning/luggage_planning/deploy/hardware_placement_launch.py
- src/luggage_planning/luggage_planning/joint_operation/prepare_node.py
- src/luggage_planning/luggage_planning/joint_operation/runtime_ports.py
- src/luggage_mapping/luggage_mapping/source.py
- src/luggage_packing/scripts/placement_planner_node.py
- src/luggage_perception/scripts/cargo_volume_mapper_node.py
- docs/architecture/cargo_grid.md
