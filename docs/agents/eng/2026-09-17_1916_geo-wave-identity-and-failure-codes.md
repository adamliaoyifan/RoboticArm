# 2026-09-17 -- GEO wave: cargo map geometry identity and honest placement failure codes

- role: eng
- agent: cursor
- model: opus-5
- cli: cursor
- status: done
- parent: GEO-WAVE-20260917
- subtask: GEO-1-3
- base_revision: b720976
- started_at: 2026-09-17T19:14:22+08:00
- completed_at: 2026-09-17T19:16:00+08:00

## Summary

Defects 8 and 9 of the `12dd654` dirty-tree audit are fixed. The cargo map now
carries the kernel `geometry_hash` everywhere it describes the container
interior, and `ComputePlacement` returns a reason code in which `BIN_FULL` is a
capacity claim that the independent eval arbiter can falsify.

## Requirement

GEO-8: mapper and planner each load their own `scene_tf.yaml` and were joined
with no cross-check, so a map built for a different hull was read for its
`inner_size`. Every artifact describing the interior must name its hull, and a
mismatch must fail closed rather than fall back to the floor prior.

GEO-9: `solve_placement` labelled both invalid size and no-candidate
`BIN_FULL`, so a box lost entirely to aperture, corridor, or unobserved support
read as a full container and terminated a pack run. Capacity exhaustion and
candidate exhaustion must be separate answers, with every tried candidate and
its reason retained.

## Changed

- `src/luggage_perception/luggage_perception/cargo_volume_mapper.py`
- `src/luggage_perception/scripts/cargo_volume_mapper_node.py`
- `src/luggage_packing/luggage_packing/placement_solver.py`
- `src/luggage_packing/scripts/placement_planner_node.py`
- `src/luggage_msgs/srv/ComputePlacement.srv`
- `src/luggage_msgs/srv/GetCargoMapStats.srv`
- `src/luggage_gazebo/scripts/pack_eval_driver.py`
- `src/luggage_gazebo/scripts/place_only_eval_driver.py`
- `src/luggage_gazebo/luggage_gazebo/place_only_fixture.py`
- `src/luggage_packing/test/test_placement_reason_codes.py` (new)
- `src/luggage_packing/test/test_placement_geometry_identity_live.py` (new)
- `src/luggage_packing/test/test_compute_placement_fixture.py`
- `src/luggage_gazebo/test/test_place_only_fixture.py`
- `docs/architecture/container_geometry.md`
- `.cursor/rules/container-geometry.mdc`

## Design notes

Capacity gates are overlap, top clearance, and hull containment; aperture,
insertion corridor, unobserved support, and insufficient support ratio are
policy or observation gates. The split matches the independent enumerator in
`place_only_fixture.enumerate_footprints`, so the product's claim and the
arbiter's verdict are comparable and a disagreement is evidence rather than a
definitional artifact.

Two ordering details were load-bearing. Capacity gates are evaluated
independently of the gate that fires first, otherwise the class would depend on
report order. The capacity count is taken over every enumerated candidate, not
the `top_n` / `keep_rejected` subset, otherwise a retention cap could turn
candidate exhaustion into a capacity claim.

## Verification

- `colcon build --packages-select luggage_msgs luggage_description
  luggage_perception luggage_packing luggage_planning luggage_gazebo`: pass.
- `python3 -m pytest src/luggage_packing/test src/luggage_perception/test
  src/luggage_gazebo/test src/luggage_description/test`: 1308 passed,
  2 skipped.
- `python3 -m pytest test` in `src/luggage_planning`: 388 passed. Run from the
  package directory; collecting it together with the other roots hits a
  pre-existing `harness` module name collision unrelated to this change.
- `src/luggage_packing/test/test_placement_geometry_identity_live.py`: 3
  passed. Launch-free two-node behavioural gate. With a mutated chamfer in the
  mapper's scene config, `ComputePlacement` returns
  `CARGO_MAP_GEOMETRY_MISMATCH` with an empty slot and the planner refuses the
  map; the matching-config control returns a slot and the kernel hash.
- `bash scripts/place_only_run.sh --mode dry-run --out
  docs/status/evidence/geo_wave/2026-09-17_dryrun2`: 5/5 cases pass, one
  `/clock` publisher, `rtf_min` 0.9998, zero residuals at clean room and
  teardown.

## Measured results

- `geometry_hash` identical across fixture, cargo map, and response in 5/5
  `ComputePlacement` calls.
- P3 product `reason_code` `BIN_FULL` with `product_agrees: true` from the
  independent capacity test; 1/1 placement-failure samples agree.
- Zero responses claimed `BIN_FULL` while `geometric_capacity` found a fitting
  footprint.
- P3's histogram is `insufficient_clearance=140 outside_aperture=60` with
  `n_capacity_feasible=0`. Those 60 aperture rejections are exactly the
  ambiguity GEO-9 was about: counting capacity independently shows none of them
  had room, so the capacity claim stands.

## Coverage not claimed

The dry-run exercised `BIN_FULL` only. `BOX_EXCEEDS_CONTAINER`,
`INVALID_BOX_SIZE`, `PLACE_CANDIDATE_EXHAUSTED`, and the live
`CARGO_MAP_GEOMETRY_MISMATCH` path are covered offline, not in Gazebo. No
claim is made about pack-to-full run length or placement success rate; the
`pack_eval_driver` termination change was not exercised in sim.

## Result

- pass at `3b50f8411c0604fd7ff87fa853832e993eeecb8c`.

## Pointers

- `docs/status/evidence/geo_wave/2026-09-17_dryrun2/`
- `docs/agents/discuss/2026-09-17_1914_geo-wave-identity-and-failure-codes.md`
- `docs/agents/reviews/2026-09-16_1227_dirty-tree-defect-audit.md`
- `docs/architecture/container_geometry.md`
