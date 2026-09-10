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
cell.

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
  completer was not L1: it wrecked easy cases and was uncalibrated. Recommendation
  was stop/not-shadow, with active sensing more plausible than hallucinated
  completion.

Valuable problems, in order, are therefore identity (P0), online container
and aperture (P1), measured post-place commit (P1), then live occupancy,
candidate feasibility, and uncertainty margins (P2). Learned ranking and NBV
are P2/P3 research, not the first production delta.

## Literature

Four literatures matter. Only the last is "robot learning" in the usual
sense.

1. **Metric scene state.** AprilTag / TagSLAM localize a known rigid body.
   Point-to-plane ICP or CAD registration refines RGB-D. FoundationPose (Wen
   et al., CVPR 2024) is a CAD-conditioned 6D hypothesis generator for novel
   objects, not a replacement for stamped TF or collision authority. OctoMap
   (Hornung 2013), Voxblox, and nvblox (Millane et al., 2023) keep FREE /
   OCCUPIED / UNKNOWN or TSDF/ESDF. That matches the cargo-map contract.
   Occupancy Networks, ConvONet, and 3DGS do not: they paint unseen volume or
   store photometry. Splat-Nav still converts Gaussians into conservative
   collision geometry before planning.

2. **Closed-loop packing execution.** Industrial practice is stop-and-look,
   gated scene updates, and post-place verification. Visual-servoing plus EKF
   pose (Janabi-Sharifi 2010) and chance-constrained / covariance-inflated
   collision checks are the uncertainty path. Insertion here is a corridor
   polyline plus Cartesian IK, not peg-in-hole force search.

3. **Online 3D bin packing as constrained MDP.** Zhao et al. (AAAI 2021;
   Sci. China Inf. Sci. 2022) pack with an action mask and stability
   analysis. PackerBot (IROS 2021) mixes heuristics and DRL. GOPT (RA-L 2024)
   is the closest "more general" packing policy: a heuristic Placement
   Generator emits Empty Maximal Spaces, a transformer ranks them, and bin
   size is not baked into the action grid. All of these assume a true bin
   occupancy image. They do not estimate container pose, UNKNOWN space, or
   arm corridors. Transferable idea: rank a finite feasible set. Not
   transferable: CNN over a known empty cuboid.

4. **Visuo-motor robot learning.** Residual Policy Learning (Johannink /
   Silver 2018) improves a classical controller. Transporter / CLIPort emit
   spatial pick-place maps for tabletop rearrangement. Diffusion Policy, ACT,
   RT-2, OpenVLA, and pi0 target contact-rich or open-world skills. They are
   a poor fit for Elfin joint control inside a ULD corridor: the failure mode
   is an infeasible or colliding trajectory, not a missing style of reaching.

LRF-P1 already rejected PoinTr/AdaPoinTr (ShapeNet domain) and occupancy
completion (UNKNOWN violation) as production candidates.

## Problem formulation

Do not formulate "make packing more general" as `pixels -> joints`.

Formulate a **constrained POMDP with a factored belief**, priors as
init/constraint/check, and learning only on residual scores.

Belief, in `container_link` after a gated scene snapshot:

- `T_base_container` and covariance
- aperture `(center, normal, width, height, confidence, stamp)`
- occupancy `{FREE, OCCUPIED, UNKNOWN}` plus instance ledger
- current payload box
- `scene_revision` / `geometry_hash`
- robot state and calibration covariance

Priors that stay:

- seven-face hull CAD
- Elfin kinematics, tool, suction
- camera extrinsics/intrinsics
- hard motion limits and corridor template

They initialize the belief, clip every volume query, reject inconsistent
updates, and remain the collision mesh. They are not the measured pose of
today's site.

Observation: `SyncedObservation` RGB-D, stamped TF, joint state. No Gazebo
truth, no hidden mesh.

Actions, layered:

1. Perception: stop-and-look views; accept or reject a scene snapshot.
2. Discrete packing: choose among candidates that already passed hull,
   support, aperture, corridor, collision, and IK.
3. Motion: corridor polyline + MoveIt verify; bounded retry on the next
   candidate.
4. Commit: re-observe after release; write measured occupancy.

Reward / objective already in this repo: expected remaining
`reachable_fill_rate`, not myopic packing score. Completion and hard-gate
violations are constraints, not soft penalties.

Generalization axes that are real for this cell:

- container pose and small aperture deformation at a new site
- unknown future box sequence
- occlusion and missing depth
- calibration covariance

Axes that are not the next problem:

- arbitrary non-convex container meshes (out of the geometry contract)
- deleting CAD and learning occupancy from scratch
- weaving between boxes
- joint-space insertion skills

A method is suitable only if it emits a scored proposal plus uncertainty,
fails closed on OOD, and cannot mutate TF, the cargo map, or motion without
the existing gates.

## Proposal

Phase 0, geometric closed loop, no new learner. This is the generalization
that actually unblocks deployment.

- P0: one deployment profile; every consumer checks the same
  `scene_revision` / `geometry_hash`.
- P1: estimate `base_link -> container_link` from tag + RGB-D plane/edge,
  fuse against the CAD prior with jump/freshness/confidence gates, then
  publish a versioned scene snapshot that MoveIt and waypoints both read.
- P1: consume the existing opening estimator for portal/staging, still
  clipped by the hull.
- P1: after release, estimate the real box pose and commit that occupancy.
- P2: integrate settled depth into FREE/OCCUPIED/UNKNOWN; never mark unseen
  as free.
- P2: rank a candidate queue by packing value + reachability + motion cost;
  retry a bounded number of already-gated slots.
- P2/P3: inflate aperture/clearance from pose and calibration covariance;
  generate observe poses from FOV and reachability.

Phase 1, learning only after Phase 0 has a stable snapshot and measured
commits. Reuse LRF promotion levels. Do not resurrect LRF-P1 box residual.

- Pose hypotheses: FoundationPose or a small ICP residual as a *scored
  hypothesis*, fused with tags/planes in an EKF. The fused pose still cannot
  move collision geometry unless gates pass.
- LRF-A1: learned NBV / `ExplorationPolicy` scoring under a fixed view and
  time budget. Coordinator rejection stays 100%.
- LRF-PL1 / GOPT-style ranker: transformer or value net over already-safe
  EMS/slot tokens, trained on full-horizon `reachable_fill_rate` with an
  action mask from hull/corridor/IK. Inner rollout must include atlas and
  corridor, which the current `value_hat` still skips.
- Residual Policy Learning only as a later SE(3) waypoint residual
  `(dh, dy, dyaw)` on the corridor template, still filtered by Cartesian
  fraction and FCL.

Do not start: end-to-end Diffusion Policy / VLA, occupancy completion that
fills UNKNOWN, joint-space RL, or replacing MoveIt with a motion-policy net.

Success is not "a network packs well in a perfect occupancy image". Success
is: a moved container is localized, occupancy matches the released boxes,
infeasible slots fail before motion, and any learner can be unplugged
without changing safety.

## Pointers

- `docs/agents/reviews/2026-09-10_1558_deployment-place-awareness-review.md`
- `docs/agents/reviews/2026-09-10_1558_container-awareness-clarification.md`
- `docs/plans/learning_research_feasibility.md`
- `docs/plans/corridor_constraints.md`
- `docs/architecture/container_geometry.md`
- `docs/architecture/production_orchestration.md`
- `docs/status/evidence/learning_research/LRF-P1/CLOSEOUT.md`
- `docs/agents/eng/2026-09-08_1119_rl-for-placement-vs-control.md`
- `docs/agents/eng/2026-09-08_1135_global-packing-value.md`
- `docs/agents/eng/2026-09-05_1745_occupancy-vs-3dgs.md`
- `src/luggage_perception/scripts/cargo_volume_mapper_node.py`
- `src/luggage_perception/luggage_perception/container_opening_estimator.py`
- `src/luggage_packing/scripts/placement_planner_node.py`
