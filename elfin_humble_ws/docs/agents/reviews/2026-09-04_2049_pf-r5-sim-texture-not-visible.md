# PF-R5 sim texture is dropped at spawn

- role: reviews
- agent: cursor
- model: grok-4.6
- cli: cursor
- status: done
- parent: PFH-REMEDIATION-20260904
- subtask: PF-R5
- revision: c5921d5f29ae5252747c7430ba2724214d1cbfc4

## Summary

YOLO-World is scoring D435 frames of a **flat-shaded STL**, not the textured
suitcase albedo that already lives in the unit model folders. Pickup spawn
uses six pre-scaled `*.stl` models plus a constant SDF diffuse. Binary STL
cannot carry UV or `map_Kd`. That is why mesh texture is not obvious in sim,
and why further segmenter-only work cannot recover a look the renderer never
produced.

## Finding

- Spawned URI: `model://suitcase_{loafbrr,vintage}_{tier}/meshes/suitcase.stl`
- Spawned color: loafbrr `0.20 0.20 0.24`, vintage `0.48 0.36 0.20`
- Unused albedo: `suitcase_loafbrr/meshes/suitcase.png`, `suitcase_vintage/meshes/suitcase.png` with OBJ UVs
- Documented reason: ogre2 runtime OBJ crash + URI mesh-cache; `write_scaled_stl` is geometry only
- Camera dump: `docs/status/evidence/platform_free_height/2026-09-04_1810_blocker_visuals/mesh/snap_1/rgb_raw.png`

Gate 4 millimetre metrics already pass. `top_surface_rate` 0.886–0.923 is
open-vocabulary recall on that silhouette. `base_link` conf 0.21–0.24 fills
when the suitcase is missed; pedestal grey is close to loafbrr grey.

## Consensus

- Codex agent: existing PFH remediation consensus (gate not waived here)
- Thread: `docs/agents/discuss/2026-09-04_2049_pf-r5-sim-texture-not-visible.md`
- Result: texture absence in sim is a spawn/asset-pipeline fact, not a YOLO
  file-read bug

## Pointers

- `docs/agents/discuss/2026-09-04_2049_pf-r5-sim-texture-not-visible.md`
- `docs/agents/discuss/2026-09-04_1453_pfh-remediation-r5.md`
- `src/luggage_description/luggage_description/suitcase_visual.py`

## Open

- PF-R5 owner reads the discuss thread. Unblock by textured sized-model bake,
  few-shot on the current STL look, or a reviews split of YOLO recall from
  geometry gates. Do not keep iterating the segmenter against missing albedo.
