# OctoMap observer implementation contract and delegated acceptance

- date: 2026-10-26
- parent: OCC-QUERY-INDEX-20261025
- subtask: OCTOMAP-OBSERVATION
- generation: 1
- base_revision: b33dda919521ef043fa9e30b07dcc79c87f76fce
- normative_plan: d7af0b70fc92c30dcde50e5386f935d36447e1f8:docs/plans/OCC_OCTOMAP_OBSERVER_G1_20261026.md
- owner: eng/codex/gpt-6
- status: implementation-contract

## User-directed division of work

Codex implements the independent package and performs build/link/basic operational checks. The user explicitly delegates tests and simulation to Cursor, overriding the original same-owner test execution rule. O0--O9 remain the acceptance scope; implementation delivery alone is not acceptance. Cursor receives a separate concrete task with exact implementation and plan revisions, runnable procedures, sample counts, thresholds and failure evidence requirements. This document does not claim that Cursor has started. No production backend migration or packing motion is included; simulation means isolated sensor-observation validation, not n=3 packing.

## O0 decisions

The following are implementation choices resolving the open design items, with explicit public contracts:

- Health metadata publishes at 2 Hz. O7 retains strict measured max received gap <1 s, including scheduler jitter. Normal input remains 2 Hz.
- Snapshot handles have a 5 s steady-clock TTL, maximum 8 active handles and 2 distinct pinned map revisions. ReleaseSnapshot is explicit and idempotent; reads take immutable ownership before releasing the registry lock, so expiry cannot invalidate an in-flight read. New acquisitions under pressure return SNAPSHOT_PRESSURE. Fusion never waits for readers to release. Current immutable block state is accounted separately from pinned generations; unchanged blocks are shared. Reset invalidates handles for future calls, while an already linearized read can finish with its original epoch.
- Surface means highest occupied voxel top in each active XY column, clipped at ceiling, with observed/active count and unknown-present flag. No occupied cell yields no height, even for an entirely known-free column. This is an observation projection, not a certified support surface. Overhangs retain unknown below; no solid fill. Surface cache is lazy, shared by same map revision and bounded by pinned/current views.
- Grid origin is (-length/2,-width/2,floor_z). Grid index=floor((p-origin)/resolution), upper bound half-open. Shape=ceil(extent/resolution); a cell is active only when its nominal center lies in the authoritative hull (EPS=1e-9), matching b33 active_mask. Partial cells whose center lies outside are inactive. Reported surface top is clipped. OctoMap coordinates=p-origin; lowest-level key offset=32768, preserving grid centers. Origin/endpoint outside representable OctoMap coordinate range rejects the whole frame; no partial fusion.
- Geometry DTO comes only from the existing Python kernel's normalized descriptor/hash. An offline exporter produces the immutable descriptor artifact; the standalone runtime consumes that artifact without importing mapper/planning/lease. A content digest protects the exported artifact; no second geometry-hash algorithm is introduced.
- Acquisition timestamps are integer nanoseconds in a declared clock domain. Positive same-stamp rigid TF is mandatory; receipt/age uses the same domain, while queue/handle/timing uses steady clock. A backward input timestamp is rejected; reset is explicit. All-filtered frames leave observed/map revisions and last integrated timestamp unchanged.
- DepthZFilterConfig is immutable, finite, 0<min<max, default [0.52,6.0] m inclusive. Input XYZ is optical-frame meters. Raw depth image deprojection belongs upstream; the new node subscribes PointCloud2. Input cloud is never edited. Node/CLI call the same preprocessing and transform functions.
- Rejected/duplicate frames do not update the tree. Same identity with different payload is CONFLICT; bounded cache 4096 plus per-sensor last-stamp monotonicity prevents replay after eviction. Explicit epoch and geometry/config identity accompany every snapshot. No TTL changes tree probabilities.

## Implementation sequence

1. Immutable types, optical-Z filtering, same-stamp rigid transform, authoritative geometry exporter, bounded domain policy; isolated CMake/ament package with pinned ROS OctoMap 1.9.8.
2. Persistent single-writer tree, frame-level hit/free aggregation, probability changes and incremental immutable block views, receipts and counters.
3. Bounded handles, ROI paging, surface, reset, complete-probability checkpoint/restore with sidecar and fail-closed validation.
4. Thin standalone ROS node/IDL, CLI replay, configuration and launch; explicit unsupported business operations.
5. Freeze implementation revision, document successful build/basic checks and limitations, then publish Cursor acceptance task. Leave full observer acceptance open until independent evidence is returned.

## Acceptance elaboration to carry into Cursor handoff

O6 uses 36 basic groups (3 resolutions x 3 box counts x 2 hulls x 2 point counts). Each basic group has all 4 outside/cross-ray combinations, each with seeds 0..9 and 3 repeats per seed: 30 scored samples per subgroup, 144 subgroups total. Three warmups precede each subgroup. All subgroup P95/max thresholds apply separately; no pooling to hide failures. Input manifest fixes raw count and accepted count. Cold observer startup and first surface build each have their own 30-sample campaign; cached calls are separately labeled. Test harness implementation belongs to Cursor; no generated references may enter online fusion as truth.

O9 qualification precedes certified performance runs. O5 failure stops O6/O7; O8 remains required. O1/O2 correctness is deterministic per case and requires zero mismatches across the finite matrix, not a claimed field reliability. Timing campaign passes only if every subgroup meets the frozen bound; missing samples are not_evaluated. O7 each 120s window must complete, have >=30 samples per query type, no normal-profile drop/reject, queue age <=1s and health gap <1s. This AND gate detects observed violations; it does not establish a numerical long-term success rate.

Simulation extension must freeze camera topic/frame/stamps, authoritative geometry, independent reference inputs, episode count, stable-input windows, acquisition/TF availability and coverage metrics before dispatch. It may not treat missing camera views as a free-volume proof. Shared Gazebo exclusivity, teardown, bounded failure bundles and separate performance lock remain mandatory. Full detailed commands and acceptance artifacts will be tied to the delivered implementation, rather than dispatching commands before their interfaces exist.
