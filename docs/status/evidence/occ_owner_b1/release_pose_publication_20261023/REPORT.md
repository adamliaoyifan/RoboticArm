# Gazebo release-pose publication investigation — 2026-10-23

Base: `f1a5898d6f7940314a4f2dccfcbba739af617f05`, product `b7ebea8c`; only pre-existing untracked `pydeps-frozen` before investigation. Same integration worktree and startup profile; scene MD5 `603f9b6523e104f675cbeea77805e1ff`. No product modification, no scored box attempt, no arm command.

## Direct observations

- Both historical failed runs logged publication startup for `/world/airport_loading/pose/info` (live_01 launch.log:288; live_02:294). Those logs alone do not prove target-model reception.
- First diagnostic startup passed gain/readiness; `ign topic -i` found an `ignition.msgs.Pose_V` publisher; `ign topic -e --num 3` received three actual messages (both commands exit 0). See topic_info.txt, raw_pose.txt, probe_result.json.
- Captured messages include `_preload_suitcase_loafbrr_small` three times. Its position contains only z=-100; orientation only w=1. The runtime pickup box was not created in this startup-only run.
- Offline replay of unchanged `ReleasePoseStream` class with the exact captured lines leaves this published suitcase absent from the cache, while several arm links are cached. See parser_replay.json and replay_parser.py.
- A second startup-only attempt intended to create the historical pickup box aborted at the startup gain gate, before create/request/capture. No target-box publication claim is made. Full startup failure logs retained in box_probe/. This was not a packing retry.
- Both owned stacks have been torn down; final process audit found no Gazebo or parameter_bridge processes.

## Source-established mechanism and limits

`place_smoke_driver.py:620` collects only explicit xyz/w fields, and caches only when exactly seven numbers were collected. Captured text omits zero-valued fields; the parser therefore discards valid published poses. `no_message` means missing cache entry, not proof of no Gazebo publication.

Confirmed: publisher and actual topic traffic; a consumer parsing defect on real published suitcase samples. Unconfirmed: runtime pickup-box publication at the historical release instant, and whether this defect alone explains either historical release failure. No fix applied.
