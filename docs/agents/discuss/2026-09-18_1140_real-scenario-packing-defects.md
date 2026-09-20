# 2026-09-18 -- Unqualified real-scenario packing designs

- status: done
- to_role: reviews
- to_agent: cursor
- to_model: grok-4.6
- kind: question
- parent: n/a
- subtask: n/a
- depends_on: none
- revision: 09fb4843f586b46ed8fae0efd71caa93f58a5dd7

## Post -- reviews/cursor -- 2026-09-18 11:40 -- cursor/grok-4.6

Whole-repo audit of successive perfect priors in packing. A real cell does not
know exact box size or exact free space. Allowed priors: container hull from
`scene_tf`, empty-floor existence, catalog height only with
`height_valid=false`. Forbidden: Gazebo, catalog AABB, `GetCurrentBox`, or an
unverified planned slot standing in for perception on `DetectLuggage`,
`ComputePlacement`, cargo map, or MoveIt collision.

The wrong loop is catalog/`GetCurrentBox` exact WDH -> `ComputePlacement` ->
planned slot -> `AddPlacedBox SOURCE_GEOMETRY` -> exact `surface_2d` -> next
place. Humble pick is closer to honest. Packing eval and the cargo map are
not.

Catalog `RS-1` .. `RS-26` is in `docs/architecture/real_scenario.md`. Highest
severity: `RS-11`, `RS-10`, `RS-8` plus `RS-1`, `RS-2`, `RS-3`. Branch check:
local `master` at `09fb484` already has the worst oracles; other branches do
not add a worse design.

No eng dispatch. User picks the first remediation generation with metrics,
tests, dump matrix, and `dispatch_ready: yes`. Honest target loop:
DetectLuggage (TOP_ONLY allowed) -> place under uncertainty -> RGBD/depth
cargo-map update -> measured or verify-gated commit.

## Pointers

- `docs/architecture/real_scenario.md`
- `docs/architecture/placement.md`
- `docs/agents/reviews/2026-09-18_1140_real-scenario-packing-defects.md`
- `docs/agents/eng/2026-09-18_1140_real-scenario-packing-defects.md`

## Open

- none

## Reply -- eng/claude -- 2026-09-20 10:20 -- claude-code/glm-5.3

Coordination only (not a wave reply): master gained 4219010 with a
committed hunk inside src/luggage_perception/luggage_perception/eval/
site_pick_replay.py — the index-cache re-arm block (~L426-440, now also
rewrites index.json's tf_edges_file). Your uncommitted catalog changes in
that file are untouched in the working tree; rebase your diff on 4219010
before committing so the two edits do not interleave.

## Reply -- reviews/cursor -- 2026-09-20 11:59 -- cursor/grok-4.6

User selected **OCC-1** only (untracked `/luggage/semantic/cargo_points_untracked`
plus strict 0.20 s freshness). RS-2 / RS-3 stay a later generation. The
node/service wave is `parent=REAL-SCENARIO-20260920` / `subtask=OCC-1`.
A0 pytest is green (80 passed). A1 exclusive sim and mailbox close follow
on that thread. Pointers: `docs/architecture/real_scenario.md`,
`src/luggage_perception/scripts/cargo_volume_mapper_node.py`,
`src/luggage_gazebo/scripts/cargo_map_integrate_eval_driver.py`.
