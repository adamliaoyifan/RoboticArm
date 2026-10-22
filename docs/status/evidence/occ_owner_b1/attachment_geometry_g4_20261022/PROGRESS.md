# G4 implementation progress

Base 702e3c44; plan 6d9ee6dc. Implementation is incomplete and no closed-loop evaluation has run.

## Source observations

The actual v2 path is attachment_frame_adapters.handle_vacuum_command_v2 -> admit_attachment -> SimFollowBackend.attach. The backend uses record[size_wdh] and record[box_pose] directly for scene attachment. VacuumCommandV2 returns only nominal attached_box_wdh and measured_grasp. PickRetreatEvalDriver retains only the grasp, discarding a unified geometry object. PlaceSmokeDriver and hardware_place_planning then call ComputePlacementV2 with nominal sizes. Selector constructs its own envelope through MeasuredAttachmentProvider and planning-owned configuration. Thus v2 independently confirms the representation split previously inferred from legacy v1 code.

## Completed foundation

Added immutable AttachmentGeometry with one freeze constructor and reversible nominal/conservative pose conversion. Source measurement lists are copied to immutable tuples. Four behavioural tests pass: source mutation isolation/repeated consumer reads, downward-offset target conversion for both box-Z signs, eccentric/yaw pose roundtrip. This proves only the new pure geometry foundation, not ROS wiring or full-chain consistency.

Command: python3 -m pytest -q tests/st09_parallel/B/test_attachment_geometry.py
Result: 4 passed in 0.02s.

## Remaining

Wire single Attachment policy and response transport, preserve this object in drivers and hardware adapter, consume it before candidate generation and through selector/MoveIt, remove planning-local construction/config/late slot filter, add boundary tests and architecture updates, build/freeze then at most two blocked closed-loop suites under the plan. No simulation launched; evaluation attempt count is zero.
