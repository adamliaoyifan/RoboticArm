# 2026-09-10 -- TCIG-2 weighted cargo map

- role: eng
- agent: codex-tcig-map-eng
- model: gpt-5.6-sol
- cli: codex
- status: done
- parent: TCIG-20260904
- subtask: TCIG-2
- base_revision: ebdf4be468ed1efb7419391ca99a2930b1ff0b30
- started_at: 2026-09-10T15:36:11+08:00
- completed_at: 2026-09-10T16:24:36+08:00

## Summary

Implemented exact hull-intersection voxel weights, geometry-bearing surface maps, typed physical-volume stats, and fail-closed surface identity validation.

## Requirement

- Preserve AABB indexing while making physical statistics and floor support use the normalized TCIG-1 hull, including fractional boundary cells and a zero-weight inactive wedge.
- Expose explicit geometry identity and physical volumes without wiring TCIG-3 placement consumption.

## Changed

- `src/luggage_perception/luggage_perception/cargo_volume_mapper.py`
- `src/luggage_perception/luggage_perception/cargo_surface_schema.py`
- `src/luggage_perception/scripts/cargo_volume_mapper_node.py`
- `src/luggage_msgs/srv/GetCargoMapStats.srv`
- focused perception and packing tests

## Verification

- Focused G2 suites: 23 passed.
- Perception suite excluding the unrelated broken CUDA-only vintage fixture: 568 passed, 44 subtests passed.
- Description suite: 141 passed, 2 skipped; mapper packing contract: 3 passed.
- Required four-package `colcon build`: pass; package tests pass except the existing perception CTest 60-second timeout at 95%.
- `git diff --check`: pass.

## Result

- pass at `a7df9f95cb36b1c532dafceea35b365cb85c8a44`.

## Pointers

- `docs/agents/discuss/2026-09-10_1534_tcig-2-weighted-map-g2.md`
- `docs/status/evidence/true_container_inner_geometry/0a0a7d5/g2/summary.md`
