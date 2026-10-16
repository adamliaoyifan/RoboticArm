# Placement prior ownership and adoption

A prior is a read-only speculative hint, never an execution certificate. The
nominal orchestration cycle and operator/motion/vacuum authority are unchanged.
The prior worker owns only private copies and bounded temporary computation.
It must not call ApplyPlanningScene, modify OCC, commit cargo or issue motion.

## Snapshot boundary

OCC's v2 surface is the authoritative typed grid. Its existing v1 publication
carries acquisition and confirmation metadata absent from v2; join both by map
revision, geometry identity and matching grid data. Require measured source,
positive acquisition stamp, confirmed occupancy and no pending changes. For
committed cargo, also require the owner's require_confirmed_map gate to be on;
its default-off "confirmed" flag is insufficient evidence. Read that setting
through a read-only parameter query. Never fabricate a measured next box from
catalog dimensions. Old revisions must not replace newer accepted input.

GetPlanningScene provides a baseline world for a private copy. Append columns
from the frozen OCC grid using its explicit container-to-world transform.
Unknown cells remain blocked for prior screening. Retain unrelated world
obstacles; only replace the prior's own reserved column IDs. Never publish the
composed scene. Missing/full-scene identity or invalid occupied heights reject
adoption rather than falling back to an empty world. Final route checking still
uses the actual current scene and authoritative attachment.

## Grasp and tool boundary

Use T_box_tcp, shared with existing pick/place contracts. Measured suction
contact orientation is an outward surface normal, not a TCP quaternion. Derive
the grasp from the attach target produced by build_candidate_pick_segments.
Choose a validated candidate ID and retain its observation/contact-model identity;
do not shift an accepted suction patch without re-evaluation. Keep the selected
directed box axes, including the pi heading branch, across pick and place.

Rigid tool screening positions the fixed component containing suction_contact_frame
in a PRIVATE RobotState and explicitly refreshes collision-body transforms.
Check all its robot collision shapes and supplied box against the frozen world.
Do not infer clearance from TCP clearance, skip the radar/camera, or exempt a
colliding tool/world pair. The arm outside this rigid component is intentionally
not evaluated by this screening query; whole-arm IK and collision certification
remain mandatory. Critical transit/entry endpoints matter as well as final place.

The CheckPlacementToolV2 service is bounded and returns contacts plus model and
snapshot identities. It has no live scene subscription or scene-update clients.
Its successful verdict means only rigid-tool/world screening, not executable
trajectory feasibility. Native pointer/geometry caches must be updated before
const collision APIs; the historical radar collision is a required regression.

## Background and consumer boundary

One active job, one replaceable pending job, one published result. New map/input
invalidates stale work. Cancellation/deadline discards partial results. An
abandoned native service future remains tracked until completion so subsequent
jobs cannot create an unbounded service backlog. Templates and measured requests
have distinct provenance and cannot be adopted interchangeably.

Before pick, bind the selected slot, candidate and grasp. Picking another candidate
requires a new matching binding; a prior for candidate A cannot follow a retry
that picked B. After attach, measured attachment and actual joints remain
canonical. Reconnect/replan as required and pass the existing complete route
certifier. On prior miss, mismatch or failure use ordinary planning; do not
relax limits or replay a cached trajectory blindly.
