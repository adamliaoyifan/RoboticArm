# Release pose observation format

`pose_text.py` decodes `ign topic -e -t /world/airport_loading/pose/info`
stdout, using a private protobuf descriptor that mirrors ignition-msgs8's
Pose_V, Pose, Header, Time, Vector3d and Quaternion. It is for simulation
observation and scoring only, never online planning geometry. Dependency:
python3-protobuf. The live release cache and offline replay share this decoder.

The CLI emits top-level header and pose blocks on separate lines, with nested
fields possibly on multiple lines. Header.stamp applies to subsequent poses;
every new header resets sec/nsec, including omitted zero scalar components.
Repeated entities, omitted scalar zeros, field ordering and protobuf numeric
syntax are decoded by protobuf. Position and orientation submessages and a
header stamp are required; scalar absence inside them means zero. Quaternion
squared norm must differ from1 by at most1e-3 (permits rounded text, rejects
zero/nonunit values); all seven coordinates must be finite. Values are never
silently normalized. One block is bounded to1MiB; recent rejection details to32.
The diagnostic descriptor field numbers/types mirror the installed msgs8
schemas; changes to Gazebo's schema require explicit review and fixture tests.
Unknown fields fail visibly rather than silently changing pose semantics.

Cache samples are (simulation_stamp_ns, [x,y,z,qx,qy,qz,qw], host_receive_monotonic).
Units are metres and a world-frame quaternion in xyzw order. Existing cache
identity, freshness and epoch checks remain in the driver. Presence evidence
adds decoder counters/recent rejection details; a rejected target pose without
an accepted cache sample reports its named rejection rather than no_message.
Reader process errors and truncated EOF are retained in diagnostics.

Capture poses.raw contains unmodified CLI text bytes, not protobuf binary.
chunks.jsonl indexes byte ranges, not message boundaries. Reassemble lines
across arbitrary chunks before decoder.feed_line; call finish at EOF. Keep raw
inputs immutable, check capture coverage/limits in manifest, and store derived
outputs separately. ready.json proves readers started, not publisher readiness.
trials.jsonl contains derived evidence rather than a complete message archive.

Verification: test_pose_text.py covers text conventions and bad inputs;
test_release_pose_stream.py verifies live/offline equivalence and unchanged
freshness refusal gates. Raw diagnostic evidence is under
`docs/status/evidence/occ_owner_b1/release_pose_publication_20261023/`.

Kinematic release position acceptance (user directive2026-10-23): planar XY
Euclidean error <=5mm. Z error, including >50mm, is observation-only at this
boundary; final settle/verification retain their own gates. The record keeps
exact declared pose, observed xyzw7, signed XYZ delta, XY/3D norms and set_pose
request/response simulation/host timestamps. Existing set_pose, valid identity
and fresh-message requirements remain. This does not add a post-command sample
barrier or change the planned release-gap or payload envelope.
