# 2026-09-07 -- Tesla occupancy flow vs this cell

- role: eng
- agent: cursor-grok-b
- model: grok-4.6
- cli: cursor
- status: done

## Summary

Tesla occupancy is a camera-to-voxel predictor trained with supervised occupancy labels that were themselves reconstructed offline from the fleet (multi-view, multi-pass, time series), not a diffusion generator. Occupancy flow is voxel velocity for moving traffic. Similar representation (OCCUPIED volume) to cargo mapping; different sensors, scale, motion, and whether unseen voxels may be filled by a learned prior.

## Pointers

- Tesla AI Day 2022 occupancy talk; US20240185445A1
- Waymo Occupancy Flow Fields (IEEE RA-L 2022) is a related but BEV forecasting paper
- `docs/agents/eng/2026-09-05_1745_occupancy-vs-3dgs.md`
- `docs/agents/eng/2026-09-07_1735_reconstruction-vs-diffusion.md`
