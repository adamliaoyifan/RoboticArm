# 2026-09-10 -- Pendant bag replay YOLO offline evaluation

- role: eng
- agent: claude
- model: glm-5.3
- cli: claude-code
- status: done

## Summary

Built the open-loop replay pipeline for the teach-pendant bags
(`~/work/robotarm_bags`): Humble's rosbag2 cannot open them (newer-rosbag2
embedded metadata; `ros2 bag play` included), so `eval/bag_mcap_source.py`
reads the mcap records directly via `pip install mcap mcap-ros2-support`
plus `rclpy.serialization`. `eval/bag_frame_join.py` joins colour↔aligned
depth by exact header stamp (30 ms rescue tolerance, orphans reported) and
`eval/replay_evaluate.py` + `scripts/pendant_bag_replay_eval.py` run the
ROS-free YOLO-World segmenter per frame and write the timestamp-keyed
evidence tree (camera_info once per bag). All 4496 joined frames across
the three raw bags replayed at 5–7 ms/frame on the RTX 5090; every frame
has a cargo detection, conf max 0.975–0.997, dominant prompt `luggage on
a platform viewed from directly above`. 124 unit tests (three new modules,
synthetic mcap fixture) pass. The compressed bag
(`pendant_jog_compressed`) is out of scope by decision; cargo points stay
in the optical frame because `d555_color_optical_frame` is missing from
the recorded TF tree (driver TFs were not recorded) — world-frame 3D needs
a follow-up extrinsics bridge.

## Pointers

- `src/luggage_perception/luggage_perception/eval/bag_mcap_source.py`
- `src/luggage_perception/luggage_perception/eval/bag_frame_join.py`
- `src/luggage_perception/luggage_perception/eval/replay_evaluate.py`
- `src/luggage_perception/scripts/pendant_bag_replay_eval.py`
- `src/luggage_perception/test/eval/test_bag_mcap_source.py`
- `src/luggage_perception/test/eval/test_bag_frame_join.py`
- `src/luggage_perception/test/eval/test_replay_evaluate.py`
- `src/luggage_perception/test/fixtures/make_tiny_replay_bag.py`
- Evidence: `docs/status/evidence/pendant_replay/RESULT.md` (full-fidelity
  tree at `~/work/pendant_replay_out/`, 12.5 GB, outside the repo)
