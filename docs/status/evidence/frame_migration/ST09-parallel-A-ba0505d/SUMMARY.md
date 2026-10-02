# ST09-A — placement geometry & ordering (2026-10-02)

- owner: eng/claude-map, glm-5.3 (ST09-A generation 2, thread
  `2026-10-02_1516_st09-a-placement-claude-g2.md`)
- base: `ba0505dc0d5c180310ee778a5888771f5a8d9cde` (C0_HEAD), branch
  `implementation/st09-converge-a`, worktree `/tmp/elfin-st09-converge-a`
- verdict: **implemented + green** — 24/24 saved slanted-wall candidates
  rejected by the exact seven-face kernel, 16-case deep matrix green,
  centers/corners vs independent scipy ≤1e-9, named observation outcomes,
  unsupported tilt rejected, protocol adapter delivered.

## What changed

1. **Exact hull legality before truncation** (`placement_solver.py`): the
   solver rehydrates the chamfered hull from the map's
   `geometry_descriptor` and evaluates every enumerated window with
   `contains_oriented_box_through_aperture` (floor-absolute container
   frame, margin = `hull_margin`). The node's injected validator keeps
   historical precedence (reason order unchanged); the internal gate closes
   the standalone/replay path. Legality is now a property of the candidate
   itself (`verdict`, `capacity_feasible`), not of the caller's closures.
2. **Deterministic deep-first ordering** (frozen
   `PlacementCandidate.sorting_key`): `support_layer_z` asc (resolution
   buckets, floor-first preserved) → `depth_m` desc (opening-plane datum
   from the new `container_geometry.opening_inward_datum`, aperture-derived
   inward normal, no hardcoded axis) → route clearance desc (0.0 +
   `route_not_evaluated` until ST09-B) → score desc → stable
   `candidate_id`. Selection replaces the historical max-score pick.
3. **Named outcomes**: `OBSERVATION_REQUIRED` (new) — zero feasible AND the
   unknown region could host a legal landing once observed (hypothetical
   floor windows, occupied testimony + injected validator kept so placed
   cargo cannot be observed away). Evidence-absent rejections
   (`unknown_above_floor`) outrank policy exhaustion; `BIN_FULL` remains a
   capacity-only claim; corridor-only failure stays
   `PLACE_CANDIDATE_EXHAUSTED`. Every solve emits an
   `ObservationRequirement` (footprint + aperture→slot corridor cells,
   still-unknown cells with reasons) for ST09-D's bounded observation.
4. **Unsupported tilt** (`support_tilt_unsupported`): a contact plane that
   ramps within `support_tol` across the footprint reads as full support by
   ratio; the contact-cell height span over the footprint axis exposes it.
   `max_support_tilt_ratio = 0.05` (~2.9°) is **defined_by_A** (no contract
   precedent; voxel-level steps never fire because only one level stays
   within tol; not widened after live failures).
5. **Footprint classes / heading**: `footprint_class` 0/1 from the frozen
   `PayloadIdentity.footprint_classes` semantics; `heading_theta` θ and θ+π
   emitted only under declared `payload_symmetry: "pi"` (distinct ids, same
   footprint); π/−π and q/−q equivalence never rejected (tested).
6. **Named margins** per candidate detail: hull static wall incl. chamfer /
   top clearance / support contact — separate parameters, recorded per
   name.
7. **Kernel** (`container_geometry.py`, additive only):
   `opening_inward_datum(geometry, aperture_corners=None)` — aperture-plane
   centroid + axis-aligned inward normal derived toward the hull centre
   (no scene-drawing hardcoding); door-face fallback is the kernel's own
   −X convention. No existing signature touched.
8. **Node** (`placement_planner_node.py`): `OBSERVATION_REQUIRED` passes
   through the existing wire string; latched `last_result` gains
   `observation_requirement` + `hypothetical_legal_windows` + the new
   per-candidate diagnostics flow automatically. No message changes.
9. **Protocol adapter** (`SnapshotCandidateGenerator` in
   `placement_solver.py`): frozen `CandidateGenerator` signature
   `(PlanningSnapshot, payload_digest) → (PlacementCandidate…,
   ObservationRequirement?)`; payload + geometry ride construction (the
   coarse snapshot carries neither); digest mismatch refused; pure, no
   motion requests.

## Replay of the saved failure (candidate_replay.json)

- input: rehearsal-52 revision-2 surface (fixtures/
  `rev2_starved_surface_2d.json`) + the 24 saved slanted-wall poses
  (`slot_hull_violation_saved24.json`), box 0.6462×0.4092×0.2768.
- **24/24 rejected** by the exact kernel; bottom-corner chamfer violations
  recompute to the recorded 444.6–463.1 mm within 0.02 mm (rounding only).
- solver outcome on the starved map: rim windows (footprint touching the
  top grid rows) all `outside_hull`/`hull_violation`; **zero false-legal**
  candidates (every feasible re-checked against the kernel); the winner is
  a legal deep interior **floor-prior** slot (`c021_000_f1_h0`, depth
  1.275 m, layer 0, `support_source=floor_prior`, confidence 0) with a
  317-cell `ObservationRequirement` — the map-starvation root cause now
  surfaces as an observation request instead of an illegal slot.
- hull-valid centre limits respected (y ≤ 0.345 yaw 0 / 0.227 yaw 90).

## Tests (exact node ids in candidate_replay.json)

- `tests/st09_parallel/A`: **29 passed** (24-replay 6, matrix+named 5,
  geometry exactness 6, tilt/margins 5, protocol adapter 7... per-file:
  replay 6, matrix 5, exactness 6, tilt 5, adapter 7).
- Regression vs pinned baseline (branch base `ba0505d`):
  - luggage_packing 127 passed + 81 subtests, 1 failed — the standing
    exempt live node (`ATLAS_IDENTITY_UNVERIFIED: urdf hash missing`,
    identical on base).
  - luggage_description 197 + 59 subtests, 5 pre-existing failures
    (identical on base).
  - luggage_planning 791 + 171 subtests, 1 pre-existing
    (`test_j6_carry_unwind_is_not_refused`, identical on base).
  - `tests/st09_parallel` (C0 + A): 98 passed.
- Intentional behaviour updates in A-owned focused tests, each tied to a
  named outcome: `test_hook_prior_reorders_selection` (atlas prior now
  reorders through the score term within the frozen deep-first key; y-keyed
  stub), `test_unobserved_support_is_observation_required` (was
  `…is_candidate_exhaustion`; S1 4.3 names the observation deficit).

## Environment

- plain `colcon build --build-base build-st09 --install-base install-st09`
  (no `--symlink-install`), packages: elfin_control elfin_description
  elfin_moveit_config luggage_{msgs,description,packing,planning,perception,
  gazebo}; thread pinning (OPENBLAS/OMP/MKL=1) at every entry; unified
  entry `python3 -m pytest tests/st09_parallel/A -q` → 29 passed.

## Notes for reviewers / downstream

- ST09-B consumes `detail.route_clearance_m` (0.0 +
  `route_not_evaluated` today) and must apply the same frozen sorting_key
  to route-bearing candidates.
- ST09-D consumes `observation_requirement` (dict form here; frozen
  dataclass via the adapter).
- The saved 24 poses live on the HULL-frame window grid while the solver
  enumerates the MAP-frame grid (inner_size 1.50×2.00 vs hull 1.49×1.97 —
  the 3 cm phantom strip); both grids' rim windows are rejected, and the
  grid overshoot itself is recorded here as contract-mismatch evidence.
