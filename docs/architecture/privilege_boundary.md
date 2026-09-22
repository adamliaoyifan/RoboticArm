# Privilege boundary: GT vs measured information

Normative. Violations are defects. Established 2026-09-21 (PAYLOAD-GEOM;
`docs/agents/discuss/2026-09-21_1830_payload-measured-geometry.md`).

## Rule

Chain modules — perception, placement planning, motion planning, motion
execution, vacuum attach as a *planning* artifact — must not consume
privileged (GT/spawn) information. Privileged information is allowed only
for:

1. Sim eval scoring (drivers' GT comparison, coverage, accuracy gates);
2. Placement-point computation inputs defined as fixtures (container hull
   geometry comes from `scene_tf` config, not runtime GT);
3. GT generation (the spawner itself, `/luggage/perception/size_eval/*`);
4. Sim physics backends (vacuum contact gate, retention, gz kinematic
   follow) — these simulate physical truth;
5. Eval fixtures (place-only perfect descriptors, catalog sizes);
6. Visualization (`scene_viz` GT wireframe).

The boundary is **structurally enforced**, not convention-enforced:
`/luggage/current_box` (schema 2) physically carries no GT fields.

## Channels

| Channel | Carries | Allowed consumers |
|---|---|---|
| `/luggage/current_box` (latched, schema 2) | `id`, `generation`, `measured{width,depth,height,height_source,yaw_valid,stamp_sec}` | any chain module |
| `/pickup_box_spawner/sync_detected_pickup_box` | request `DetectedLuggage` + `expected_generation` (CAS); writes the `measured` record | pick drivers, eval fixtures |
| `/pickup_box_spawner/get_current_box` (pull) | GT `DetectedLuggage` + `generation`, `mass_kg`, `model_name` | sim physics, eval, fixtures, viz — never chain planning |
| `/luggage/perception/size_eval/spawned` | GT spawn record | eval scoring only |

Accessors (`luggage_planning/luggage_planning/current_box_payload.py`):
`identity_from_current_box_payload` / `measured_from_current_box_payload`
are the only sanctioned reads of the topic. There is deliberately no
accessor that returns GT off the topic.

## Invariants

1. The MoveIt attached `pickup_box` collision geometry is the
   perception-measured size; vacuum attach refuses without a synced
   `measured` record (`ATTACH_REFUSED`).
2. The motion occupancy FK re-sweep payload resolves
   measured-first (`resolve_payload_wdh`); the static
   `payload_width/depth/height` node params are a conservative fallback
   envelope only, and the boundary record names the source
   (`payload_source: current_box_measured | static_default`).
3. `sync_detected_pickup_box` is CAS on `generation`; the spawner clears
   `measured` on spawn/clear/finalize, so a measurement can never leak
   onto a newer instance.
4. `sync` rejects `height_valid=false` (catalog priors never enter
   collision geometry — `DetectedLuggage` geometry contract).
5. Physics-vs-planning split inside the vacuum controller: the pulled GT
   record feeds the contact gate/retention/follow offset; the MoveIt
   scene geometry comes from the measured record. The two never mix.

## Audit snapshot (2026-09-21)

GT consumers at rework time: vacuum controller (fixed → measured for
scene, pull for physics), scene_viz (fixed → pull), place-smoke/pack
drivers' `model_name`/`mass_kg` (fixed → pull), eval scoring (allowed),
spawner GT generation (allowed). Placement solver inputs were already
measured/fixture-only.
