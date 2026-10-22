# Diagnostic planning envelope — 2026-10-22

Status: implementation and offline coverage/rejection verified; full functional
acceptance NOT achieved. New physical route, inter-knot physical validation and
n=3 remain not_evaluated. No simulation or carry execution was started.

## Configuration and semantics

User bounds: yaw +/-5deg around contact-frame origin/Z; expanded box-axis
faces50mm each; tool-axis translation +/-100mm each after rotation. All in
src/luggage_planning/config/payload_envelope.yaml. Set launch argument
payload_envelope_config:=/absolute/path/to/file.yaml to use a hand-edited file;
restart selector to load changes. No calibrated probability is claimed.
Roll/pitch exact and nominal box Z parallel/antiparallel to tool Z. Nominal
measured dimensions and eccentric T_tool_box remain unchanged. Continuous yaw
bounds enumerate endpoints and all derivative-zero angles of each corner's
XY sinusoids; convexity extends corner bounds to the whole expanded cuboid.
The envelope is tool-axis aligned with an explicit center offset. No GT input
enters this construction. Invalid limits/tilt/attachment fail closed.

Envelope is frozen in AttachmentRecord and included in its digest; graph
payload geometry, slot containment, corridor footprint, carrying scenes,
released obstacle and certification share it. Nominal target synthesis still
uses original attachment. Existing allowed contact links unchanged, link2 not
allowed. Existing slip recheck invalidation remains; no new slip sensing is
claimed. Closed-hull slot check deliberately preserves floor collision.

## Observed evidence

Input: executor_root_cause_20261022/live_20261022_151711 exact saved measured
request and original selected response, robot_expanded.urdf, physical spawn.sdf
and constant physical tool/box offset from follow_trace. Physical references
are consumed only by offline scripts.

- Envelope dimensions: 0.8496864101 x 0.6844225183 x 0.5175408646m.
- All40 vertices of all5 physical collision cuboids lie inside. Since each
  cuboid and enclosing envelope are convex, this proves containment of all
  those collision solids, not just sampled faces. Minimum per-axis slack
  0.1474723255m; actual eccentric transform included.
- Native FCL0.7 actual link2 mesh: old measured model0/337 colliding knots;
  physical model15/337; envelope165/337. All15 physical collisions are rejected.
  This is a knot check, not continuous-trajectory certification.
- Saved selected slot16 DESCEND endpoint FK: floorZ0.530m, envelope minimum
  Z0.429986677m; penetration into excluded floor halfspace0.100013323m.
  This uses the actual saved endpoint, not an assumed ideal floor-touch pose.
- Independently, an ideal nominal floor-touch slot has envelope bottom150mm
  below floor under the requested size/position bounds. Production route
  entry test returns PAYLOAD_ENVELOPE_OUTSIDE_SLOT_HULL before any edge build.
- Focused suite237 tests pass (pytest.txt); one additional positive integration
  test passes through real graph/certifier with fake edge/validity reports,
  proving identity/scene propagation only, not a physical new-route success.
- Three packages built: st09_contracts, luggage_planning, luggage_gazebo.
  Installed-module/config import and launch --show-args succeeded.
- Two stale test fixtures reproduced on baseline849cec0d: expiry assumed11s
  despite60s contract, and object.__new__ graph omitted occupancy_boxes. Tests
  updated to current contracts; runtime expiry/cargo geometry unchanged.

## Acceptance limitation

Positive vertical bounds conflict with floor-supported placement. A larger
search budget cannot make the demonstrated goal legal. Do not remove floor
checks, shrink limits to fit old collision points, or report hovering as
completed placement. Await user choice: preserve bounds and record infeasible;
separately design controlled contact/release; or explicitly revise Z bounds.
No new-route or n=3 launch is justified before this geometric conflict is
resolved. This result does not explain all historical executor failures.

## Reproduction

Source /opt/ros/humble/setup.bash and install/setup.bash in the integration
worktree; prepend repo root and src/luggage_planning,src/luggage_description
to PYTHONPATH. Native probe is compiled from scripts/g3/mesh_box_collision_probe.cpp
with g++ -O2 -std=c++17 and pkg-config fcl.

```
python3 scripts/g3/replay_box_arm_collision.py docs/status/evidence/occ_owner_b1/executor_root_cause_20261022/live_20261022_151711 --urdf docs/status/evidence/occ_owner_b1/executor_root_cause_20261022/robot_expanded.urdf --envelope-config src/luggage_planning/config/payload_envelope.yaml --output-dir docs/status/evidence/occ_owner_b1/payload_envelope_20261022/offline_original
python3 scripts/g3/replay_payload_envelope_goal.py docs/status/evidence/occ_owner_b1/executor_root_cause_20261022/live_20261022_151711 --urdf docs/status/evidence/occ_owner_b1/executor_root_cause_20261022/robot_expanded.urdf --config src/luggage_planning/config/payload_envelope.yaml --output docs/status/evidence/occ_owner_b1/payload_envelope_20261022/original_goal.json
```
