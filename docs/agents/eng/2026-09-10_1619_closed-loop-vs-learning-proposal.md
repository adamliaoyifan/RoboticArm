# 2026-09-10 -- Closed-loop state vs robot-learning formulation

- role: eng
- agent: cursor-grok-b
- model: grok-4.6
- cli: cursor
- status: done

## Summary

Most current packing failures are open-loop prior use, not missing neural
policies. Keep CAD, kinematics, calibration, and safety as versioned priors,
but demote them from final fact to initializer, constraint, and check.
Learning is useful later as residual ranking, view scoring, or pose-hypothesis
scoring on top of that closed loop. End-to-end visuo-motor RL, occupancy
hallucination, and joint-space policies are the wrong formulation for this
cell. Restored during the 2026-09-10 workspace archive because the original
note was missing from the shared tree.

## File review

The 2026-09-10 reviews already state the same diagnosis: the ROS 2 place chain
has static prior awareness, not online container awareness. The matching code
is:

- `scene_tf` publishes a calibrated `base_link -> container_link`. MoveIt,
  waypoints, packing, and metrics all consume that prior. A physically moved
  container keeps software internally consistent and globally wrong.
- `container_opening_estimator.py` already fuses tag, depth, and prior, with
  confidence and stamp. ROS 2 place waypoints still use the configured
  aperture rather than a gated live estimate.
- `cargo_volume_mapper_node.py` is `SOURCE_GEOMETRY` only: it commits planned
  AABBs and never integrates sensor points. `integrate_points()` exists in the
  algorithm class and is unused online.
- `placement_planner_node.py` returns one geometry winner. Atlas, corridor,
  and Cartesian dry-run are not an active ranker. `value_hat` / CEM already
  estimate remaining fill, but the inner rollout ignores IK and corridor.
- Corridor architecture already reserved the learning slot: replace candidate
  generation, never the hard filters.
- `learning_research_feasibility.md` already forbids learned modules from
  executing motion, mutating the canonical map, or bypassing hull / corridor /
  collision / IK / freshness / OOD gates.
- LRF-P1 ran. Measured extra views helped heavy occlusion. A residual box
  completer was not L1: it wrecked easy cases and was uncalibrated.

Valuable problems, in order, are identity (P0), online container and aperture
(P1), measured post-place commit (P1), then live occupancy, candidate
feasibility, and uncertainty margins (P2). Learned ranking and NBV are P2/P3
research, not the first production delta.

## Literature

Four literatures matter. Only the last is robot learning in the usual sense.

1. Metric scene state: AprilTag / TagSLAM, point-to-plane ICP, CAD
   registration, FoundationPose as a CAD-conditioned 6D hypothesis generator.
   OctoMap / Voxblox / nvblox keep FREE / OCCUPIED / UNKNOWN. Occupancy
   Networks and 3DGS do not.
2. Closed-loop packing execution: stop-and-look, gated scene snapshots,
   post-place verification, EKF pose, covariance-inflated collision.
3. Online 3D bin packing as a constrained MDP: Zhao et al., PackerBot, GOPT.
   Transfer the finite feasible-set ranker, not a CNN over a known empty
   cuboid.
4. Visuo-motor learning: Residual Policy Learning can stack on a classical
   controller. Diffusion Policy / VLA / joint-space RL are a poor fit for
   Elfin corridor insertion.

## Problem formulation

Formulate a constrained POMDP with a factored belief. Priors initialize,
constrain, and check; they are not today's measured site pose. Learning may
only score already-gated proposals and must fail closed.

## Proposal

Phase 0 is geometric closed loop with no new learner. Phase 1, after that
snapshot exists, may add FoundationPose/ICP hypotheses, LRF-A1 NBV scoring,
and LRF-PL1 / GOPT-style ranking of already-safe slots. Do not resurrect the
LRF-P1 box residual.

## Pointers

- `docs/agents/reviews/2026-09-10_1558_deployment-place-awareness-review.md`
- `docs/plans/learning_research_feasibility.md`
- `docs/plans/corridor_constraints.md`
- `docs/status/evidence/learning_research/LRF-P1/CLOSEOUT.md`
- `docs/agents/eng/2026-09-10_2126_cursor-grok-b-workspace-archive.md`
