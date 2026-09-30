# ST09 generation 4: completion breakdown (claude takeover)

- owner: eng/claude/glm-5.3
- base_revision: b6b6336 (generation 3 implementation checkpoint, same branch)
- generation: 4
- plan_revision: ca8426f (obligations unchanged; this document only breaks
  the remaining work into ordered phases)
- status: implementation in progress

## Obligations

Unchanged from `docs/plans/frame_migration_st09_g3.md`. The generation 3
checkpoint is an implementation baseline, not release evidence: migration
checker exit 2 with exactly F01/F21/F22/F23/F24 pending and
`runtime_execution: not_evaluated`.

## Phase breakdown

- P0 takeover: claim, this breakdown, eng role note, fresh install build and
  a reproduced baseline verification record (offline `not_evaluated`,
  migration exit 2) under `ST09-g4-baseline/`.
- P1 production v2 adoption and enforcement: enable the v2 map/placement
  chain in sim and hardware launches (resolved identity fields reach
  `cargo_volume_mapper` and `placement_planner`); a node runtime bundle
  (identity report + 4 Hz steady-clock heartbeat + readiness gate with
  recorded transitions) wired into every endpoint-owning production node;
  admission at request acceptance and immediately before motion, attachment
  and scene/map effects, v1 handlers included; shared typed driver adapter;
  seven caller migrations (hardware driver and the six eval drivers, the
  motion_occ driver switching to `SelectPlacePathV2`); F01 residuals (scene
  identity, horizontal-container gate, shared full-pose transforms replacing
  hand conversions). Closes the code half of F01/F21/F22.
- P2 dump/replay/viz wiring: GT writers, replay readers, visualizers and the
  packing surface reader on `luggage.frame_dump/2`; legacy only through the
  named adapter; measured and GT records separate. Closes F23.
- P3 graph campaign and F24 isolation: production graphs on mock controllers
  without Gazebo, three healthy startups per profile and every fault once
  within 180 s startup / 90 s teardown budgets; graph, process identity, gate
  transition and counter evidence; legacy absence proven on installed
  executables and tested graphs. Runtime proofs for F01/F21/F22; F24 isolated.
- P4 exclusive non-scored rehearsal: event-producing rehearsal mode for the
  standard driver (separate namespace, no scoring writes), frozen manifest
  builder, domain-7 exclusive runner within 20 minutes, diagnostic carryon
  then standard cycle, pre-pick/pre-place fault rehearsal, live extractor
  pass.
- P5 capture and replay: measured capture windows, three paired replays with
  median throughput degradation at most 10 percent, data budget respected,
  frozen handoff manifest with hash-change invalidation.
- P6 release: final matched regressions, all five runtime proofs re-emitted
  against final sources, both checkers exit 0, verification runner pass in
  all modes, evidence bundle and lifecycle records. Scored INTEGRATION
  remains a separate release against this exact candidate and frozen
  configuration.

## Evidence naming

`docs/status/evidence/frame_migration/ST09-g4-<stage>/` for every generation
4 run; g3 evidence directories are immutable history.
