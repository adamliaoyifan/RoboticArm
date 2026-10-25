# S5 typed boundary checkpoint (formal campaign pending)

S4 product bb4cb175, adopted baseline b33dda91, plan b9f4b2dc. No overall G2 PASS.

Four typed read-only services and nine typed DTO messages are registered. They wrap immutable query handles in the mapper's existing reentrant callback group; primary Store writer, heartbeat parameters, lease, executor and callback-group topology remain unchanged. Python schema-1 meanings are retained. The registry supports strict same-token/full-identity resume without renewing expiry; invalid requests preserve an existing token unless explicit release was requested. Protocol details are in src/luggage_perception/QUERY_API.md.

## Preflight verification

- Independent /tmp/occ-query-g2-build and install initially built luggage_msgs/description/packing/perception/planning, including generated type-support libraries, in 80s. No shared overlay rebuilt. The missing YOLO checkpoint warning is unrelated to these read-only services; no semantic detector launched.
- Six real generated-message serialization tests pass, including summary, frozen old-generation paging, explicit release/expiry/invalid-cursor retry, enums/errors, nullable full surface and patches, and distance policies. No new contract test skipped.
- Initial typed summary test exposed adapter KeyError(placed_count). Fixed mapping to existing committed_box_count; old summary DTO unchanged. Raw response/logger/summary and replay pointer retained in preflight/failures/typed_summary/.
- Additional existing mapper/grid/confidence/occ-input/view/snapshot and adapter regression: 114 passed, four existing skips. This includes existing no-motion live mapper tests. Their environment used the default ROS domain (unset); this is rehearsal, not isolated formal timing evidence. Subsequent standalone runtime campaigns use dedicated domains.
- Existing skips, all the parked prior-node semantics in test_cargo_map_integrate_live.py: test_full_depth_endpoint_isolated_from_empty_cargo_at_same_stamp; test_depth_observer_real_ros_client_reaches_owner; test_volume_service_exact_revision_read_only_and_stale_refusal; test_volume_read_waits_for_inflight_map_mutation. They do not cover the replaced prior depth observer/volume-wait behavior; not silently counted as covered.
- A source-side read-only ROS wiring fixture is added for installed-module provenance, unique domain, per-request serialized CDR artifacts and bounded shutdown. Its actual installed rehearsal is still pending at this checkpoint. The pure frozen st09_contracts package must also be independently built before that rehearsal.

## Outstanding mandatory work

Full CLI oracle/benchmark/concurrency/planning/replay implementation, full 20x500 shape/resolution/actual-box matrix, clean-artifact paired 3+30 groups, write P95 regression, 120sx3 heartbeat and real placement hit/fallback service checks are not_evaluated. Existing offline unit/concurrency tests and a fast development number do not pass these gates. No simulation/motion or production integration performed.
