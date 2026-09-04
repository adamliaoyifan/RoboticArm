# 2026-09-04 - True container inner geometry draft

- role: reviews
- agent: codex
- model: gpt-5
- cli: codex
- status: done

## Summary

Drafted unified task `TCIG-20260904` so all usable-space semantics use the
physical seven-face inner hull. AABB storage remains allowed, but candidate
retention, map volumes, EMS/replay, corridor checks, reachability, floor
coverage, and metrics must clip to the canonical hull. Cursor technical and
distinct Codex consensus are reached. This note and the approved plan are
committed together; that commit is the reproducible dispatch revision.

## Acceptance

- One ROS-free geometry kernel is authoritative for calibrated and sensed
  geometry in simulation and hardware.
- Checked-in hull resolves to `4.22433625 m^3` usable volume and `2.28715 m^2`
  usable horizontal floor area.
- Complete boxes and swept payloads, not centers alone, remain inside the hull.
- Hull-invalid candidates are removed before ranking or retention.
- Every volume, coverage, reachability, and blocked-space metric uses the same
  geometry hash as its input map/atlas.
- Cuboid configurations retain supported behavior and invalid descriptors fail
  closed.

## Consensus

- Cursor technical review: `cursor_consensus: reached` in
  `docs/agents/discuss/2026-09-04_1601_true-container-geometry-cursor-review.md`
- Required distinct Codex consensus: reached in
  `docs/agents/discuss/2026-09-04_1642_tcig-formal-codex-consensus.md`
- Result: consensus reached; the commit containing this note and plan is the
  reproducible approved revision

## Subtasks

| ID | Proposed owner agent/model | Depends on | Scope | Acceptance | Required tests |
|---|---|---|---|---|---|
| TCIG-1 | `codex/gpt-5` | none | Canonical seven-face geometry kernel and architecture contract | Exact area/volume/containment/clipping | G1 geometry tests |
| TCIG-2 | `codex/gpt-5` | TCIG-1 | Hull-weighted cargo map and surface contract | Exact weighted stats and floor semantics | G2 mapper tests |
| TCIG-3 | `codex/gpt-5` | TCIG-1,TCIG-2 | Online placement | Hull gate before `top_n`; no false BIN_FULL | G3 placement tests |
| TCIG-4 | `cursor/grok-4.6` | TCIG-1 | Corridor, ROS-free waypoint algorithm, swept payload | Payload-yaw hull erosion shared by plan/audit | G4 corridor tests |
| TCIG-5 | `cursor/grok-4.6` | TCIG-1 | Geometry metric helper and eval entry points, excluding replay core | No hard-coded volume/floor denominators | G5 volume/floor metric tests |
| TCIG-6 | `cursor/grok-4.6` | TCIG-1,TCIG-2,TCIG-4,TCIG-5,TCIG-7 | EMS, free-space model, replay | Hull-clipped offline policy semantics | G6 offline tests |
| TCIG-7 | `cursor/grok-4.6` | TCIG-1 | Atlas, floor coverage, versioned runtime geometry interface | Hull-active reachability and reachable-volume ratio | G7 planning/interface tests |
| TCIG-INTEGRATION | `codex/gpt-5` | TCIG-1..TCIG-7 | Integrated closeout | All focused gates and headless E2E pass | G1-G7 plus E2E |

Assignments are proposed, not dispatched. Cursor must confirm the amended
ownership and file isolation before TCIG-4 through TCIG-7 become runnable.
The single `cursor/grok-4.6` session receives only one runnable row at a time,
after its queued PF-R7 or after registration of a second Cursor session.

## Risks

- Center-sampled voxel masks are resolution-dependent; physical volume requires
  clipped boundary-cell weights.
- A late hull gate can still produce false `BIN_FULL` after `top_n` truncation.
- EMS cuboids are only bounding structures; useful volume means their exact
  intersection with the hull.
- `ContainerOpeningEstimate.inner_size` cannot represent a chamfer and needs a
  versioned geometry contract before sensed geometry can be authoritative.
- Existing unported ROS 1 scripts must not be mistaken for production ROS 2
  implementations.

## Cursor Amendments Incorporated

- `packing_replay.py` is TCIG-6-only; TCIG-5 owns a ROS-free metric helper and
  eval entry points.
- `insertion_corridor.py` and `blocks_deep_space()` are TCIG-4-only.
- Reachable-volume ratio moved from G5 to G7.
- TCIG-4 owns a new ROS-free waypoint algorithm while the ROS 2 node stays thin.
- TCIG-1 exports payload-yaw hull erosion consumed by TCIG-3 and TCIG-4.
- TCIG-7 versions messages/adapters without editing the TCIG-1 kernel.
- Eval-only identity may provide metric numerators but never usable-space
  denominators or online inputs.
- Cursor work is dispatched serially, respecting PF-R7 priority.
- Unsupported opening sides fail closed and corridor erosion uses actual yaw.
- Generate-time hull filtering remains before `top_n`; the node gate is
  invariant diagnostics only.

## Pointers

- `docs/plans/true_container_inner_geometry.md`
- `docs/architecture/README.md`
- `docs/architecture/perception_architecture.md`
- `docs/agents/discuss/OPEN.md`

## Open

- None. Dispatch remains subject to the dependency and serial-owner rules in
  the approved plan.
