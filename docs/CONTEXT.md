# Elfin Humble Workspace Context

Last updated: 2026-11-13.

This repository is a ROS 2 Humble colcon workspace for an Elfin S20 luggage
loading cell. The active production route is complete-operation planning and
execution on measured occupancy, not the 2026-09 YOLO closed-loop migration
path.

```text
operator START
  -> initialize O map owner (explicit initially-empty declaration or observation)
  -> integrate measured ULD map
  -> observe pickup box
  -> shared PlacementQuery (directed headings)
  -> joint-operation prepare (atlas / aperture routing)
  -> execute certified joint plan
  -> release / verify / return observe
```

Authoritative contracts:

| Need | Open |
|---|---|
| Architecture index | `docs/architecture/README.md` |
| Production cycle | `docs/architecture/production_orchestration.md` |
| Placement / map | `docs/architecture/placement.md`, `cargo_grid.md` |
| Agent mailbox | `docs/agents/discuss/OPEN.md` |
| Active plans | `docs/plans/README.md` |

Historical 2026-09 closed-loop / calibration / migration context is archived
under `docs/archive/` and is not the current product route.

Current verification checkpoint: continuous selected-pose admission, stationary
pre-pick geometry history, calibrated LiDAR structure-terminal support and
successful-path proposal persistence are implemented. One full-model RGB-D/O
floor/floor/stack execution passed functionally; sampled physical payload
containment failed for boxes 2/3. Hardware geometry accuracy, actual calibrated
LiDAR qualification and real-cell baseline are not established. See the
[implementation and evidence report](status/evidence/geometry_plan_reuse/20261113/REPORT.md).

G is selectable as an alternative owner at the hardware placement boundary;
whole-operation execution still binds O and is not qualified for backend
switching. The [G/O review](agents/reviews/2026-11-13_1222_tolerant-geometry-go-review.md)
records the remaining interface, input and lifecycle gaps. Next work prioritizes
complete operation and impact-based tolerance; simulation discrepancies alone
do not authorize simulator-specific perception tuning. Atlas/cache benefits
await the [independent evaluation design](plans/ATLAS_ROUTE_TOLERANCE_EVALUATION_20261113.md),
which is not yet dispatched or executed.

Package map, build commands, and launch details: see
`docs/architecture/README.md` and the package READMEs under `src/`. Do not
treat archived hardware YOLO dual-machine notes as the default startup path.
