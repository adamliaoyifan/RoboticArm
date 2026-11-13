# Elfin Humble Workspace Context

Last updated: 2026-11-12.

This repository is a ROS 2 Humble colcon workspace for an Elfin S20 luggage
loading cell. The active production route is complete-operation planning and
execution on measured occupancy, not the 2026-09 YOLO closed-loop migration
path.

```text
operator START
  -> reset selected map owner (O default; G explicit alternative)
  -> observe ULD / integrate measured map
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

Package map, build commands, and launch details: see
`docs/architecture/README.md` and the package READMEs under `src/`. Do not
treat archived hardware YOLO dual-machine notes as the default startup path.
