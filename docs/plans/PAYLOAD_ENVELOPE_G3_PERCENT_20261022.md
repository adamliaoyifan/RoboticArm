# Payload envelope G3: percent growth and downward-only height

- date: 2026-10-22
- owner: eng/codex/gpt-5
- base_revision: 51b11d12
- parent: PHASE2-20261022
- subtask: PAYLOAD-ENVELOPE
- generation: 3

## Requirement

Replace the oversized fixed margins with a diagnostic relative-size model.
Nominal measured geometry and the eccentric `T_tool_box` remain unchanged.
Width and depth each grow by 10 percent total, symmetrically. Height grows by
10 percent only on the face away from the suction contact; under the supported
tool-down attachment this is world `-Z`. The retained 5 mm Z translation error
is also one-sided away from contact. No extra XY translation range is applied.
Continuous yaw `+/-5deg` remains and may make the final XY AABB exceed 1.10 of
the nominal dimensions.

The contract must reject attachments whose contact-facing height face cannot
be identified unambiguously. It must state that the world-down interpretation
is valid only for the existing tool-down/box-vertical carrying contract.

## Acceptance

- Unit geometry: with yaw zero and a contact-at-top attachment, output width
  and depth are exactly `1.10 * nominal`; the contact-facing Z face is exactly
  unchanged; the opposite face is exactly `0.10 * nominal_height + 0.005m`
  farther away. Equivalent antiparallel box-Z attachment behaves identically.
- Continuous-yaw corner coverage is checked at endpoints and interior extrema.
- Invalid percentages, tilt and ambiguous contact geometry are named refusals.
- Attachment identity and all existing planning consumers continue to bind the
  same generated envelope; no collision whitelist or slot exception is added.
- Replay `live_03` saved states against links1--4 and record nominal, old G2,
  and new G3 collision counts and dimensions. This is offline attribution, not
  a route-success claim.
- Run focused envelope/route tests and build the three touched ROS packages.
- Then run one non-scored headless closed-loop diagnostic with the existing
  180 s selector budget, slot-envelope diagnostic bypass, domain 7, inch20,
  seed 0 and gain 0.8. Preserve all failure-stage artifacts before teardown.
  Startup failures before a case may be retried within 20 minutes with the
  frozen revision. One attempted product case ends the run.

## Evidence boundaries

Preserve frozen config/hash, source/install identity, request including grasp
and slots, exact selector response/CDR and rows, full-precision staging states,
runtime URDF, lifecycle, motion/follow/plugin streams, suite/result and process
teardown state. The failed-case bundle must remain replayable and below 1 GiB.
No result may claim continuous trajectory validity unless certification emits
and checks a route.
