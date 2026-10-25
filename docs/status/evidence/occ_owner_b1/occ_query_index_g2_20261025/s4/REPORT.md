# S4 exact distance implementation checkpoint

Parent implementation ddb82d94 (S3 product 7a77256f); paired baseline remains b33dda91. No overall G2 PASS.

New nearest_obstacles(points, max_radius_m=.30, unknown_policy="separate", budget=None) reads immutable cell blocks. Distances are to full voxel AABB volumes, not centers. A deterministic branch-and-bound heap orders candidate blocks by point-to-block lower bound; blocks tied at best are visited to preserve smallest-flat-index tie breaking. Geometry bounds are immutable/reused and budgeted; no mutable nearest-parent field is retained across obstacle removal.

Policies are explicit: separate returns occupied and unknown matches independently; occupied_only leaves unknown unqueried, never classified free. MATCH has exact distance/cell; NO_MATCH proves the target class empty; RADIUS_CLEAR has no distance, exact=false and a proved radius lower bound. OUTSIDE_MAP/HULL is per-point and cannot masquerade as safety. Default limits are <=256 points, <=2,000,000 examined cells, <=50,000 blocks and 200ms; an explicit DistanceBudget can only reduce these. QUERY_BUDGET_EXHAUSTED has exact=false/no purported nearest distance and incomplete batch. This is a read query, not payload/hull collision certification.

## Verification

- 11 distance tests pass. For each of .01/.02/.05, 1,000 fixed points match independent brute voxel-AABB oracle; a second mixed occupied/unknown 1,000-point fixture at each resolution also matches. Deletion, restored sensor shadow, cross-block zero-distance tie, empty/free/radius, chamfer/outside, limits and deep immutability covered.
- Combined focused/regression: 138 passed +2 subtests (regression.log). Includes S1 20x500 net changes, S2 concurrent readers/writer, S3 and existing Store/projection/commit/confirmation/anchor plus b33 fallback.
- First new free-fixture test incorrectly expected a single hit after one miss to be occupied; the unchanged fusion correctly left UNKNOWN. Raw failure and before/after evidence retained; corrected by a second public hit, with no fusion change.
- Unscored development smoke was run using the legacy upper-Z dimension 2.01 rather than usable span 1.48 (149x197x201, not deployment grid). Its ~35ms batch and RSS are not formal deployment evidence. No formal query/write/RSS or heartbeat gate evaluated here.

S5 typed ROS, independent build, full shape/resolution/actual-box matrix and all formal paired gates continue. No planner, lease, write protocol/fusion, worker, simulation, shared install or production merge changes.
