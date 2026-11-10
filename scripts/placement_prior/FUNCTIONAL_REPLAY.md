# Frozen functional replay entry point

`functional_replay.py --manifest FILE --output NEW_DIR` verifies a clean pinned
runtime revision and all captured input hashes, then writes a reviewed command.
Add `--run` under the installed ROS environment and `ROS_DOMAIN_ID=8` to run the
existing bounded isolated MoveIt replay. It starts neither Gazebo nor execution.
The delegated replay owns shutdown and preserves native failure artifacts.

The manifest schema is `placement-prior-functional/1`; the currently supported
mode is `captured_consumer_plan_only`. This mode exercises the production hardware
consumer with captured inputs. It does not satisfy actual deployment F2 or
original-target recovery F1, and always reports those gates as `not_evaluated`.
Success requires a successful native response, identical consumer request,
completed callback shutdown and zero native process exit code.

Required fields: `revision`, `capture`, `scene_tf`, `robot_poses`, `observe_pose`,
`select_index`, `surface`, `scene`, `attachment_request`, `attachment_response`,
`prior_response`, `choice_index`, and `artifacts` (path to SHA256 mapping).
Paths resolve relative to the manifest file, or may be absolute. Indices select
the exact captured Select call and prior choice. No implicit first surface or
first scene fallback is permitted at this entry point.

Hash all seven explicit file inputs, plus the capture's
`dumps/typed_service_boundaries/boundaries.jsonl`, selected request CDR, preceding
map-stats response CDR, `dumps/runtime_identity.json`, and
`dumps/box_NN_paths/candidates.json`. The entry point derives those implicit reads
from the verified ledger and refuses missing or changed files before launch.
Model identity is subsequently checked by the native replay against live MoveIt.
This is not a snapshot lock: do not concurrently modify frozen input artifacts.

The output retains the manifest, exact argv, client log and native evidence under
`native/`. Without `--run`, it reports `native_replay=not_run`, never a functional
pass. Full F1/F2 acceptance still needs its separate measurements and deployment
profile; unsupported modes fail before launch.
