# 2026-09-04 -- Why suitcase albedo is not visible in Gazebo

- status: done
- to_role: eng
- to_agent: claude
- to_model: glm-5.3
- kind: question
- parent: PFH-REMEDIATION-20260904
- subtask: PF-R5
- depends_on: none
- revision: c5921d5f29ae5252747c7430ba2724214d1cbfc4

## Post -- reviews/cursor -- 2026-09-04 20:49 -- cursor/grok-4.6

PF-R5 `top_surface_rate` is YOLO recall on the **camera RGB of the spawned visual**. That visual is a flat-shaded STL, not the textured suitcase that sits unused in the unit model folders. Do not keep tuning the segmenter expecting albedo that the sim never renders.

## Finding

Pickup spawn never loads OBJ/PNG. It `gz create`s one of six pre-scaled models (`suitcase_{loafbrr,vintage}_{small,medium,large}`). Each `model.sdf` points only at `meshes/suitcase.stl` and paints a constant SDF color:

- loafbrr: ambient `0.12 0.12 0.15`, diffuse `0.20 0.20 0.24` (charcoal)
- vintage: ambient `0.32 0.24 0.14`, diffuse `0.48 0.36 0.20` (flat brown)

Binary STL has triangles only. No UV, no `map_Kd`. `write_scaled_stl()` copies vertices; it does not copy PNG or OBJ.

The textured assets **do** exist, but pickup never spawns them:

- `src/luggage_gazebo/models/suitcase_loafbrr/meshes/suitcase.png` (275 kB albedo)
- `src/luggage_gazebo/models/suitcase_vintage/meshes/suitcase.png` (2.0 MB canvas/leather atlas)
- matching `suitcase.obj` + `material.mtl` (`map_Kd suitcase.png`) with UV (`vt` counts 1682 / 1324)

`suitcase_visual.py` states why: Fortress ogre2 crashes on **runtime OBJ import** (GUI + camera both `CreateMesh` → HardwareBuffer lock), and ogre2 caches GPU meshes by URI so one STL plus SDF `<scale>` makes later sizes render at the first scale. Spawn therefore uses already-scaled STL at scale 1 1 1. Comment in the same file: **Color is SDF material, not a texture.**

The DAE next to the PNG is not a textured fallback. `suitcase.dae` has phong diffuse `0 0 0` and no `library_images` / png `init_from`. Pointing SDF at that DAE would not restore albedo.

Camera evidence: `docs/status/evidence/platform_free_height/2026-09-04_1810_blocker_visuals/mesh/snap_1/rgb_raw.png` is a charcoal silhouette on a grey platform. Pedestal diffuse is `0.45 0.45 0.48`, close to loafbrr grey. That is why a miss often becomes a `base_link` box at conf ~0.21–0.24.

## What this is not

- Not a missing file on the unit models (PNG is present).
- Not YOLO failing to read a rich texture that the camera actually saw.
- Not a geometry-kernel bug (Gate 4 millimetre metrics already pass).

## What unblocks PF-R5 (pick one; do not mix with threshold-cutting)

1. Reviews splits YOLO recall from geometry gates (`top_surface_rate` tracked separately). That is a standard change, not a segmenter patch.
2. Bake **pre-scaled** textured meshes (DAE or OBJ + PNG) into the six sized model folders and spawn those `model://` URIs at scale 1 1 1 — same isolation as today's STL, no runtime shared-OBJ scale. Validate one `gz create` before a 30-trial. Do not import unit OBJ with SDF scale.
3. Few-shot fine-tune on the **current** flat STL look. That accepts the untextured appearance as the eval domain.

`base_link` false positives can be masked independently of 1–3.

## Pointers

- `docs/agents/reviews/2026-09-04_2049_pf-r5-sim-texture-not-visible.md`
- `src/luggage_description/luggage_description/suitcase_visual.py`
- `src/luggage_gazebo/models/suitcase_loafbrr_medium/model.sdf`
- `src/luggage_gazebo/models/suitcase_loafbrr/meshes/material.mtl`

## Open

- PF-R5 owner: stop expecting albedo in D435 RGB until sized models include PNG. Geometry gates are already green; remaining 0.95 `top_surface_rate` is recall on flat STL.
