# 2026-09-10 -- Pendant replay real-site YOLO tuning (threshold, prompts, one-box selection)

- role: eng
- agent: claude
- model: glm-5.3
- cli: claude-code
- status: done

## Summary

Retuned the pendant bag replay evaluator from the first-pass review
ground truth: one luggage per frame, near the image centre; low-conf
boxes are mis-detections and the bbox_fill masks were polluted by them.
Data analysis of the first pass (`rank-1 conf p10 0.56-0.92`, rank-2
mostly <0.5 and peripheral, true box central in 97-100% of frames) plus a
4-set prompt ablation picked: confidence floor 0.3, prompts
`luggage on a platform viewed from directly above / suitcase / luggage`
(all cargo; `box/floor/container/robot arm` dropped), and a new
`select_cargo_detection` one-box rule (central max-conf within 0.35×width,
`off_center` fallback, honest miss below floor) with `repaint_label_map`
so the mask/overlay/points are rebuilt from the kept box alone. Tuned
full rerun on all three raw bags: exactly one cargo box per frame, kept
conf p50 0.91-0.97, 99.9% central selection (25 off_center, 8 no-cargo
frames reported honestly), inference p95 5.3-6.2 ms. Artifact chain
verified byte-identical on frame `1788943637_390859231`: 1 det (0.974) in
`detections.json`, single-region mask, overlay ==
`draw_detections_overlay(rgb, [kept])`. 129 unit tests pass (new
selection/repaint cases). Sim config (`config/semantic_segmenter.yaml`)
untouched — tuning is replay-CLI defaults only (`--cargo-select none`
/ `--config` restores raw behaviour).

## Pointers

- `src/luggage_perception/luggage_perception/eval/replay_evaluate.py`
  (`REAL_SITE_PROMPTS`, `select_cargo_detection`, `repaint_label_map`)
- `src/luggage_perception/scripts/pendant_bag_replay_eval.py`
- `src/luggage_perception/test/eval/test_replay_evaluate.py`
- Evidence: `docs/status/evidence/pendant_replay/RESULT.md` (full-fidelity
  tree at `~/work/pendant_replay_out/`, outside the repo)
- Prior note: `docs/agents/eng/2026-09-10_1919_pendant-bag-replay-yolo-eval.md`
