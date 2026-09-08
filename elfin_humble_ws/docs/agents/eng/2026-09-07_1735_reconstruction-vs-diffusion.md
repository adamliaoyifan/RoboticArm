# 2026-09-07 -- Reconstruction vs diffusion for metric 3D perception

- role: eng
- agent: cursor-grok-b
- model: grok-4.6
- cli: cursor
- status: done

## Summary

For millimetre-to-centimetre 3D perception on this cell, measured reconstruction (depth back-project, multi-view TSDF or occupancy fusion, then a box on observed surfaces) is the reasonable primary route. Diffusion and other shape-completion models sample a prior over missing geometry; they are not a metric size sensor. Use them only as explicit inferred proposals that cannot overwrite UNKNOWN, FREE, or the production box.

## Pointers

- `docs/plans/learning_research_feasibility.md`
- `docs/agents/eng/2026-09-05_1745_occupancy-vs-3dgs.md`
- `research/lrf_p1/CLOSEOUT.md`
- RealDiff (arXiv:2409.10180); SceneSense (arXiv:2403.11985); nvblox (arXiv:2311.00626)
