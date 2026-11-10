# Size semantics: catalog vs placement (normative note)

Established 2026-10-15 (G3 wave 3 geometry chain audit). Two conventions
describe one physical box; they are NOT interchangeable and neither is
"wrong" — but every record must say which one it uses.

## The two conventions

| Convention | Fields | Meaning | Where used |
|---|---|---|---|
| catalog | `catalog_H`, `catalog_W`, `catalog_D` | standing suitcase: H long edge, W width, D thin edge | `box_catalog.yaml.example` entries (`size:`), GT spawn records (`command_size`) |
| placement | `footprint_x`, `footprint_y`, `vertical_extent` | the box as it rests: two horizontal top-face extents and the vertical thickness | perception measurement, `box_wdh`, placement candidates, manifest `size`/`primitive_collision_size` |

A lying suitcase (largest face up) with catalog H×W×D has top face
W×H (plus handle bands) and `vertical_extent` = catalog_D. The legacy
field names map as: `width`/`depth` = the two top-face extents (NOT the
catalog thin edge), `height` = `top_z − support_z` = the vertical
extent (`top_support_estimator.compose_box_geometry`).

## Sources of truth

- **Sim object contract**: the tier registry (`iter_size_tiers`) plus
  the pipeline manifest `sized_suitcases.json` (`size`,
  `measure_size`, `primitive_collision_size`, asset hashes). One
  pipeline (`generate_sized_suitcases.py generate | sync-manifest |
  check`) owns every asset; hand-committed model dirs are defects.
- **Online planning contract**: ComputePlacement consumes the
  perception-measured `box_wdh` (footprint_x, footprint_y,
  vertical_extent). Catalog/GT sizes never substitute for it
  (`real_scenario.md`, `privilege_boundary.md`).

## The 2026-10-15 audit in one line

Assets were consistent all along; the first functional divergence was
the perception support plane (annulus overhang on a tight platform
picked a container rim, doubling `vertical_extent`) — fixed with
near-ring priority in `estimate_local_support`.
