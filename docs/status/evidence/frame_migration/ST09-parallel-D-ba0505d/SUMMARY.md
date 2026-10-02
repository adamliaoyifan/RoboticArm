# ST09-D — observation policy, async map integration, attachment provider (2026-10-02)

- owner: eng/claude-map, glm-5.3 (ST09-D generation 2, thread
  `2026-10-02_1517_st09-d-map-attachment-claude-g2.md`, depends_on ST09-A =
  passed @2cb46f9)
- base: `ba0505dc0d5c180310ee778a5888771f5a8d9cde` (C0_HEAD), branch
  `implementation/st09-converge-d`, worktree `/tmp/elfin-st09-converge-d`
- verdict: **implemented + green** — 8 map cases × sim/deploy, 3 traceable
  immutable revisions + idempotent duplicate, bounded two-view policy,
  measured attachment provider with the full negative matrix, and a real
  saved-sensor replay (rehearsal-52 aim depth/mask/intrinsics).

## What was delivered

1. **`observation_policy.py`** (new, `luggage_perception`, ROS-free): at
   most **2 prevalidated views** (`VIEW_ORDER = portal_aim →
   second_region`, the policy-level mirror of the runtime
   `cargo_view_sequence` two-view sequence — no motion requests, ST09-I
   executes). Prevalidation: TF freshness inside the frozen 0.2 s join
   window (`VIEW_TF_STALE`), pinhole visibility of the requirement's
   still-unknown cells (`VIEW_EMPTY` when a view sees none), best-coverage
   selection within a kind. Coverage prediction is **read-only**: never
   imports GT/catalog geometry, never mutates the snapshot.
2. **`cargo_view_integration.py`** (additive; every pre-existing function
   untouched — mapper node, drivers and their tests keep behavior):
   - `planning_snapshot_from_surface` — the missing
     `surface_map_2d → PlanningSnapshot` producer (sparse non-unknown
     cells, row-major sorted, C0 fixture conventions).
   - `integrate_view_with_report` + `SnapshotViewIntegrator` (frozen
     `ViewIntegrator` protocol): pure, immutable — new snapshot per view
     (map_revision+1, view acquisition stamp, recomputed digest);
     **stateless idempotency**: a view whose acquisition stamp equals the
     snapshot's own is the same observation already integrated → snapshot
     returned unchanged, `VIEW_DUPLICATE`, no double-counted revision.
     Named failures: `MAP_GEOMETRY_HASH_MISMATCH`, `VIEW_TF_STALE`,
     `VIEW_DEPTH_MISSING`, `VIEW_EMPTY`; partial coverage reported
     (`VIEW_PARTIAL_COVERAGE` + still-unknown list). **Missing sensors
     never promote**: only miss-testimony cells (`observed_cells`) turn
     UNKNOWN→FREE; hits turn cells OCCUPIED; everything else is untouched.
   - The async story is structural: the integrator is a pure function of
     `(snapshot, view)`; ST09-I's adapter queues pairs and never holds a
     mapper lock. No driver edits.
3. **`attachment_provider.py`** (new, `luggage_planning`, ROS-free core +
   injectable sources): `MeasuredAttachmentProvider` implements the frozen
   `AttachmentProvider` protocol — `T_tcp_box = inverse(T_world_tcp) *
   T_world_box` from same-stamp measured poses (skew ≤ 0 ns enforced;
   `t_tcp_box` cross-checked against independent scipy ≤1e-9, reproduce
   oracle ≤1e-9 m); publish-once per valid observation + attach
   confirmation (duplicate events idempotent); new `attachment_incarnation`
   per confirmed re-attach with the old record invalidated
   (`ATTACHMENT_REATTACH`); `slip_recheck` composes the frozen record
   forward and invalidates beyond 0.01 m / 0.10 rad (`ATTACHMENT_SLIP`);
   frozen 10 s expiry (`ATTACHMENT_EXPIRED`); GT / GetCurrentBox pull /
   planned-slot / PCA-yaw-only / gazebo-physics sources refused
   (`ATTACHMENT_GT_AS_MEASURED` / `ATTACHMENT_SOURCE_UNQUALIFIED`);
   unconfirmed attach never publishes a route-grade record
   (`ATTACHMENT_UNCONFIRMED`); pre-attach estimates only as
   `MEASURED_PRE_ATTACH_ESTIMATE` (never route-grade). All policy values
   quoted from the frozen contract — no widening.

## Traces (this directory)

- `revision_trace.json` — 3 valid observations → revisions 1→2→3→4, digest
  chain, acquisition joins; duplicate re-integration of the last stamp
  returns the same snapshot (no 5th revision).
- `coverage_trace.json` — bounded plan `[aim, lookback]` with the stale
  portal rejected `VIEW_TF_STALE`; per-view covered cells and the
  still-unknown residual.
- `attachment_trace.json` — publish-once, re-attach incarnation 0→1 with
   invalidation, slip recheck beyond tolerance → `ATTACHMENT_SLIP`.
- `traces_manifest.json` — the above + exact test node ids + environment.

## Tests

- `tests/st09_parallel/D`: **40 passed** (map cases 8×sim/deploy +
  never-promote + protocol signature; revision trace 2 + saved replay 3;
  attachment provider 11; observation policy 5).
- Regression vs pinned baseline (branch base `ba0505d`):
  - luggage_perception 1068 passed + 207 subtests, 1 pre-existing
    (`test_pf_a1_static_audit` sim-backend classifier node, identical on
    base);
  - luggage_planning 791 + 171 subtests, 1 pre-existing
    (`test_j6_carry_unwind_is_not_refused`, identical on base);
  - `tests/st09_parallel` (C0 69 + D 40): 109 passed.
- Saved-data replay uses rehearsal-52's real aim view (`fixtures/`:
  depth npy 640×360, camera_info K, mask, integrate request, and the
  production-integrated `surface_2d_aim.json` as the independent
  expectation: 30 free / 73 occupied / 1097 unknown at revision 2 — the
  producer reproduces exactly those counts). The camera→container TF at
  capture time was not preserved in the dump, so the replay validates the
  real depth/mask deprojection + the integrator's real-stamp path, not a
  bit-exact cell reproduction (recorded honestly here).

## Environment

- plain `colcon build --build-base build-st09 --install-base install-st09`,
  packages incl. `luggage_perception` + `luggage_planning`; thread pinning
  at every entry; unified entry `python3 -m pytest tests/st09_parallel/D -q`
  → 40 passed. No Gazebo, no live runs (per dispatch: saved data only; live
  view-provider wiring belongs to ST09-I).

## Notes for ST09-I

- Wire `SnapshotViewIntegrator` + a queued `(snapshot, view)` adapter
  behind the existing `/cargo_map/integrate_cargo_view` path; the mapper
  node itself is untouched by D.
- Wire `MeasuredAttachmentProvider` at the F18 attach boundary
  (`typed_driver_adapter.grasp_from_attach` measured pair +
  `vacuum_backend` seal confirmation); `latest_record` / `slip_recheck`
  feed C's certification and the carry-fault entry.
- `observation_policy.plan_views` consumes ST09-A's
  `ObservationRequirement` (the solver now emits it on every solve);
  execute the returned ≤2 views through the unified route entry.
