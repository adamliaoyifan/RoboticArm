# P3 fix evidence (2026-10-03, commits on top of 36d1157)

## Fix 1 — atlas prior queries the MEASURED TCP targets

`atlas_prior(targets, request)` now receives the targets
`_expand_targets()` synthesized through the measured attachment
(T_world_box x inverse(T_tcp_box): grasp yaw deviation, eccentric XY,
actual tool rotation composed in) and queries each at its OWN yaw in
container frame. A candidate is ATLAS_UNREACHABLE only when EVERY
allowed TCP target is sampled-unreachable; the box-centre + h/2 +
release-gap hand-offset query is deleted. Verified by the live suite
below (mock grasp is centred, so the synthetic query agreed — the fix
matters for eccentric/rotated grasps where the old query could
mis-reject).

## Fix 2 — explicit ordering contract (no reverse())

- `seeded_order.reverse()` REMOVED. Ordering is an explicit frozen
  contract: atlas buckets (seeded -> reachable -> unknown), each
  ordered by distance-to-opening ASCENDING with ties broken by
  DESCENDING placement index (`EXECUTABLE_FIRST_ORDER` constant with
  the audit basis and the three-box capacity cost documented inline).
- Per-candidate budget slicing was evaluated and REJECTED with data: a
  complete route build costs ~6-10 s of the frozen 10 s budget
  (staging OMPL dominates); 6 s and 4.6 s slices each starved the only
  viable attempt (runs p3run16-p3run18). Fairness against doomed-edge
  re-attempts comes from the search-level failure memo instead:
- Failure memo (grid_path_search._edge_for): an edge failing twice
  this session is not retried for the next candidate — shared staging
  anchors made every candidate re-attempt the same failed OMPL
  transit (live: 42 attempts, zero completed routes before the memo).

## Fix 3 — atlas resample record

See `atlas_resample_record.md` in this directory: SHA-256 of all four
NPZ (BYTE-IDENTICAL to pre-resample — deterministic sampling; the URDF
diff is ros2_control plugin/initial values only) and all four YAML
(identity rebinding), build commands + per-variant connected/elapsed,
scene/URDF/payload identity, source/install paths.

## Fix 4 — narrowed occupancy-avoidance claim + causal artifacts

The loop test now captures, per landed box: the occupied surface cells
under its footprint (`landed_footprint_cells`), the grid shape and
total occupied count, and — for the second box — asserts ZERO overlap
with every previous box's footprint cells. full_loop.json (this
directory, from the first counted suite run): box1 12 footprint cells,
box2 10, overlap []. Claim scope: "box 2 selected a non-overlapping
candidate on a revision containing box 1's observed occupancy" — this
is exactly what the evidence shows, no more.

## Verification at this revision

    bash scripts/st09_i/kill_stack.sh
    source install-st09/setup.bash
    export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=/tmp/elfin-st09-converge-i/src: ROS_LOCALHOST_ONLY=1
    python3 -m pytest tests/st09_parallel/I -q

I: 34 passed / 0 skipped, runs p3run27 (204.4 s) and p3run28 (183.5 s)
consecutive, plus standalone loop runs (footprint capture). B 140,
PD 42p/2s, A 29, C 19, C0 69, D 40, E 11, planning 648+171 — green.

## Known limits recorded

- EXECUTABLE_FIRST ordering may occupy opening-adjacent cells; the
  three-box capacity acceptance at M3 (S6) is the check.
- One loop run observed 33 vs 24+10 occupied cells (a box-2 footprint
  cell coincided with the pre-place observation band, not with box 1's
  footprint) — the overlap assertion is scoped to previous boxes'
  footprints exactly as claimed.
