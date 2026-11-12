# 2026-11-12 -- O initial empty source and independent Livox input

- role: eng
- agent: codex
- model: gpt-6
- cli: codex
- status: open
- parent: O-INITIAL-LIVOX-20261112
- subtask: SOURCE-INPUT
- base_revision: 36d9e62c4fe94ca5bf14d92752c01a14dc2a46df
- started_at: 2026-11-12T22:32:31+08:00
- completed_at: n/a

## Summary

Implemented source-owned initial-empty declaration/provenance and independent
stationary LiDAR XYZ ingestion into the same O owner. Required source/API checks
pass. No Gazebo/controller/hardware cycle was run; whole-operation baseline stays
open. Validation uses the shared dirty working tree and exact build hashes, not
an isolated clean checkout.

## Requirement

User explicitly requests initialization with source records and a separate
Livox input using correct frame transformation. Plan5febcde5 defines the gates.

## Changed

- Native initialize_empty service atomically starts a new epoch and materializes
  exact active hull cells as initial free, with declaration ID/authority/time.
  Sensor acquisition time/revision remain zero. Initial free is separate from
  sensor probabilities: a first hit overrides it. Reset never reinitializes it;
  repeated declaration is idempotent and cannot erase acquired sensor changes;
  a second initialization within this owner lifetime refuses. Acquisitions before
  declaration time cannot cross the initialization boundary.
- Snapshot/operation-source identities, checkpoint audit data and per-cell
  evidence_sources retain provenance. Initialized source consumers distinguish
  initial-condition time from acquisition freshness. Normal post-observation
  age checks, epoch/generation/config/content binding remain. Common placement
  labels initial support explicitly and still applies same-source volume checks.
- LiDAR physical-frame XYZ cloud input uses acquisition TF into container_link
  and the same ray fusion/store/grid as RGB-D. No optical-Z filter is applied to
  LiDAR. Invalid/zero returns provide no rays; missing TF/wrong frame refuses.
  LiDAR-only and mixed subscriptions both work, with distinct sensor receipts.
  A separate existing-joint-gate adapter forwards stationary scans unchanged.
- Operation composition launches that adapter; topic/frame are configurable.
  Standalone O exposes LiDAR-only transport. Explicit-start sim driver declares
  initial empty and no longer defaults to portal observation; measured pickup
  observation and configured complete static ULD/pedestal admission remain.
- No physical collision clearance, release gap, atlas or placement pose constant
  was changed. New source behavior follows explicit user empty-ULD authorization.

## Verification

- 199 required Python cases pass,60 frame-contract subtests pass; zero skips.
- 9 CTest executables/checks pass in Release,130.30s. The added initial-input
  checker covers first-hit override, old acquisition rejection, source audit,
  checkpoint round trip, reset/lease invalidation and separate validity.
- 2 actual O tests pass,2.63s: LiDAR-only domain178 and mixed domain179. Real
  PointCloud2 subscription + stamped TF yaw90/translation reaches cell[1,2,1].
  Missing TF/wrong frame leaves revision unchanged; mixed camera/LiDAR uses one
  writer. Full request/response CDR and frozen harness retained; both owned
  processes exit0 and both executors join. No simulator used.
- Production rebuild13 packages passes,6.91s,zero concurrent source changes;
 191 installed-origin/generated-interface checks pass.
- Exploratory older frame-migration suite has3 stale solve_placement monkeypatch
  failures. Replaying those exact cases against the base-revision node reproduces
  all3. They are recorded separately, not claimed repaired or counted as passes.
- Earlier fixture mistakes and unoptimized CTest attempt are preserved in the
  summary. Final native evidence adds frozen harness capture; no failed data was
  overwritten. Scope is deterministic correctness, not a cycle success-rate/SLA.

## Result

Source/interface implementation verified. Next integration must exercise the
actual full S20 observation/pick/carry/release/adoption/exit chain, including the
post-release initial-free/actual-cargo boundary. This work does not qualify
moving-scan deskew, new self segmentation, historical CustomMsg conversion or
physical sensor calibration. Current Mid-360S launch defaults xfer_format=0
PointCloud2/livox_frame; a historical bag uses CustomMsg and requires conversion
or a PointCloud2 driver configuration rather than blind topic relabelling.

## Pointers

- docs/plans/O_INITIAL_EMPTY_LIVOX_20261112.md
- docs/agents/discuss/2026-11-12_2231_o-initial-livox.md
- docs/status/evidence/o_initial_livox/20261112/summary.json
- src/uld_occ_observer/README.md
- docs/architecture/cargo_grid.md

Code commit: `f03b27366137b4ad875630b4bd708909a157d597`.
