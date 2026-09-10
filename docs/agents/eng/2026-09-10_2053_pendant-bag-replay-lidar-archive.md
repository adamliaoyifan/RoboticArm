# 2026-09-10 -- Pendant replay full /livox/lidar archive

- role: eng
- agent: claude
- model: glm-5.3
- cli: claude-code
- status: done

## Summary

Added the full-volume Mid-360 archive to the pendant bag replay
evaluator (user request: parse ALL livox scans and archive them with the
rest, including the deskew fields). `decode_lidar_scan` in
`eval/bag_mcap_source.py` reads the Mid360 PointXYZRTLT layout
offset-aware (FLOAT32 x/y/z/intensity at 0/4/8/12, UINT8 tag/line at
16/17, FLOAT64 timestamp at 18, point_step 26) as a structured (N,)
array; `evaluate_bag` archives EVERY `/livox/lidar` message during the
index pass under `lidar/<sec>_<nsec>/{points.npy, lidar.ply}` plus a
per-scan `lidar_index.jsonl` row (stamp/log_time/n_points/frame_id/
ts_min_ns/ts_max_ns/scan_span_ms/dir). Measured timestamp semantics:
absolute unix ns as FLOAT64, starting at the scan header stamp +0.000 ms,
span p5/p50/p95 = 97/100/103 ms — i.e. the per-point deskew input is
complete. Full rerun on the three raw bags: 2574/2574 scans (counts
equal the bag metadata), 47.46 M points, 0 decode failures. Camera
frames always carry `meta.json lidar.nearest_{stamp_ns,dir}` + `dt_sec`
pointers into the archive (`--with-lidar` copies payloads per frame on
top, same structured format). Default on; `--no-lidar-archive` opts out
(the sampled repo evidence copy opts out — the archive lives in the
full tree only). 132 unit tests pass (decode/archive/pointer cases; the
tiny fixture packs the exact 26-byte wire layout with a duplicate stamp
and a far-from-frame scan).

## Pointers

- `src/luggage_perception/luggage_perception/eval/bag_mcap_source.py`
  (`decode_lidar_scan`)
- `src/luggage_perception/luggage_perception/eval/replay_evaluate.py`
  (lidar archive pass, `lidar_index.jsonl`, nearest pointers)
- `src/luggage_perception/scripts/pendant_bag_replay_eval.py`
  (`--no-lidar-archive`)
- `src/luggage_perception/test/fixtures/make_tiny_replay_bag.py`
- `src/luggage_perception/test/eval/test_bag_mcap_source.py`,
  `src/luggage_perception/test/eval/test_replay_evaluate.py`
- Evidence: `docs/status/evidence/pendant_replay/RESULT.md` (full tree at
  `~/work/pendant_replay_out/`, 14 GB, outside the repo)
