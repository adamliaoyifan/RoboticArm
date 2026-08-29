"""Offline sampling and gate helpers. Not part of the live detect pipeline.

Live pickup is ``sensor_preprocessor`` → ``semantic_segmenter`` →
``semantic_point_filter`` → ``luggage_detector``. Modules here are only
imported by N-trial drivers and their pytest files:

- ``detection_accuracy`` — meas vs GetCurrentBox
- ``detection_gate_sampling`` — stamp join, failure dumps, summaries
- ``yolo_window_stats`` — YOLO hit-rate window summaries

Drivers: ``scripts/detection_gt_gate_run.py``, ``scripts/yolo_two_class_window.py``.
"""
