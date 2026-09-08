# 2026-09-08 -- Atlas HTML shows container hull

- role: eng
- agent: cursor-grok-b
- model: grok-4.6
- cli: cursor
- status: done

## Summary

The reachability atlas HTML now overlays the scene_tf 7-face inner hull, outer AABB, and door aperture in `container_link`, using the same geometry kernel as packing rather than the atlas YAML cuboid.

## Pointers

- `src/luggage_planning/scripts/reachability_atlas_html.py`
- `src/luggage_description/config/scene_tf.yaml`
- `/tmp/reachability_atlas.html`
