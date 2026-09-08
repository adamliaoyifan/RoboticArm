# LRF-P1 closeout -- stop

- stopped_at: 2026-09-05T20:13:17+08:00
- started_at: 2026-09-05T18:40:33+08:00
- budget: 5 h research cap; stop and summarize regardless of remaining ideas
- recommendation: stop this spike (not shadow-candidate)

Further LRF-P1 / MuJoCo / residual experiments in this session are finished.
LRF-A1 and LRF-PL1 were not started. Production packages were not changed.

## What actually ran

Two sensors, one detector, one weak learner.

| Phase | Input | Methods | Result |
|---|---|---|---|
| A. Offline STL dropout | ~900 surface samples, fake viewpoint dropout, single frame XYZ | `estimate_box` vs extra-view concat vs Ridge residual | L0 harness pass; learned **not** L1 |
| B. MuJoCo D455 | one 848x480 pinhole depth frame -> world XYZ | same `estimate_box`; residual then pose-locked | Baseline seats the suitcase; unconstrained residual **drifts** |

Not run: Isaac Lab, real D455, lidar, ROS bag, PoinTr weights, time-series.

## Phase A (mailbox LRF-P1, commit `5c08312`)

Worktree `/home/adamliao/work/elfin_humble_ws_eng_lrfp1` branch `agent/eng/lrf-p1`.

- Train: loafbrr all sizes (162). Test: vintage medium+large (108). No mesh overlap.
- Baseline heavy composite p95 1.868; fusion 1.721; learned 1.423 but unoccluded 0.090 -> 0.470.
- L1: leakage/OOD/isolation/latency pass. Heavy +10 pp valid, no-critical-p95, containment 99%, under-bound 0, interval coverage, nominal 10% all **fail**.
- Ablation: measured extra views help heavy occlusion without wrecking easy cases. Hallucinated completion does not.

Evidence: `docs/status/evidence/learning_research/LRF-P1/c09ec7009c5d58fd724373fceb43ecfc6c0bcc35/`

## Phase B (user D455 sim, uncommitted on same branch)

- Scene: one suitcase on a table, MuJoCo 3 + EGL, D455-like depth VFOV 58 deg.
- Baseline on vintage medium clear: composite 0.182, XY ~1.7 cm, yaw ~0.3 deg.
- Ridge residual trained on STL dropout, tested on D455: composite ~2, unusable.
- Residual retrained on D455 loafbrr: composite looked better but **red box drifted** (center ~+1.8 cm, yaw ~+2 deg). Composite was misleading.
- Final D455 policy: if `estimate_box` succeeds, learned copies that box. Overlay red = blue, seated on the suitcase.

Evidence: `docs/status/evidence/learning_research/LRF-P1/mujoco_d455/index.html`

## Method (executed)

- Detector: `luggage_perception.luggage_box_estimator.estimate_box`
- Learner: `research.lrf_p1.candidate.ResidualEnsemble` (sklearn Ridge, 5 bootstrap models)
- Code: `research/lrf_p1/` and `research/lrf_p1/mujoco_d455/`
- Audited not executed: PoinTr/AdaPoinTr, ConvONet, 3DGS occupancy

## Do not take from this spike

- Do not shadow-wire the residual into production.
- Do not treat STL-dropout numbers as camera evidence.
- Do not treat MuJoCo pinhole depth as a real D455 or a hardware bag.
- Active sensing / extra views remain more plausible than single-view box residual.

## Codebase

```
/home/adamliao/work/elfin_humble_ws_eng_lrfp1
  research/lrf_p1/          # Phase A harness (committed at 5c08312)
  research/lrf_p1/mujoco_d455/  # Phase B camera sim (worktree dirty)
  src/luggage_perception/luggage_perception/luggage_box_estimator.py
```
