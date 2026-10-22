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


## G4 boundary wiring checkpoint

Attachment controller now loads attachment_geometry.yaml, freezes and returns
AttachmentGeometryV2 before physical effect, and supplies the same conservative
pose/size to the sim scene backend. Driver and hardware attachment adapters retain
this output. ComputePlacement client supplies conservative sizes and converts
returned envelope-centre poses to nominal-box targets without mutating input.
Select transports frozen geometry; admission refuses missing output, provider
binds its measured relation and adopts it without rebuilding. Selector tolerance
configuration, planning-local constructor, late slot hull filter and diagnostic
bypass are removed. Historical replay utilities now load the Attachment policy.
Runner launch arguments and architecture contract updated.

A new test exposed shared slot/candidates aliasing: deepcopy of the whole
response preserved shared references and transformed one object twice. Fixed by
converting each output independently from its original source. Failed log retained.

Latest focused result: 161 passed in 1.33s. Includes real ROS serialization,
geometry tamper/missing refusal, conservative capacity input/target conversion,
controller/physical follow, release/hardware adapters and route certification
identity/collision refusal. Earlier 162 included the obsolete late-slot-filter
unit, now replaced by upstream capacity and missing-output admission coverage.
Source imports use current worktree and generated messages from initial build;
this is NOT a final installed-artifact result.

Command environment: source /opt/ros/humble/setup.bash; source install/setup.bash;
PYTHONPATH=$PWD:$PWD/src/luggage_planning:$PWD/src/luggage_description:$PYTHONPATH.
Command: python3 -m pytest -q tests/st09_parallel/B/test_attachment_geometry.py
src/luggage_planning/test/test_attachment_geometry_transport.py
src/luggage_planning/test/test_attachment_revision_propagation.py
src/luggage_planning/test/test_sim_physical_attachment.py
src/luggage_planning/test/test_st09_release.py
src/luggage_planning/test/test_hardware_place_planning.py
src/luggage_planning/test/test_hardware_place_replay.py
src/luggage_planning/test/test_payload_envelope.py
tests/st09_parallel/I/test_route_producer.py

Remaining before evaluation: audit pre-pick/prior and capture/replay callers for
new required geometry, complete service/driver regression and geometry provenance
coverage, rebuild all affected/dependent packages in plain-install mode, verify
installed identities and capture health, freeze then execute at most two blocked
functional suites. No simulation has been launched; attempted suites remain zero.
