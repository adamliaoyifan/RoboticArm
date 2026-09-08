# 2026-09-08 -- Reachability atlas files and HTML viewer

- role: eng
- agent: cursor-grok-b
- model: grok-4.6
- cli: cursor
- status: done

## Summary

S20 collision-aware reachability grids from 2026-08-11 are on disk as `.npz` plus YAML under `src/luggage_planning/data/reachability_atlas/`. Added a ROS-free HTML dump so Humble can inspect them without the rospy RViz node.

## Pointers

- `src/luggage_planning/data/reachability_atlas/`
- `src/luggage_planning/scripts/reachability_atlas_html.py`
- `src/luggage_planning/scripts/reachability_atlas_viz.py`
- `/tmp/reachability_atlas.html`
