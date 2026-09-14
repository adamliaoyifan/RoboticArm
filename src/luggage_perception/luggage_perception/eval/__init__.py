"""Offline sampling and gate helpers. Not part of the live detect pipeline.

Live pickup is ``sensor_preprocessor`` → ``semantic_segmenter`` →
``semantic_point_filter`` → ``luggage_detector``. Modules here are only
imported by N-trial drivers and their pytest files:

- ``detection_accuracy`` — meas vs GetCurrentBox
- ``detection_gate_sampling`` — stamp join, failure dumps, summaries
- ``yolo_window_stats`` — YOLO hit-rate window summaries
- ``gate5_bag_readiness`` — PF-A2 Gate 5 manifest checker (no accuracy claim)
- ``pf_a1_static_audit`` — PF-A1 privileged-input scan of online nodes
- ``site_pick_replay`` — offline mcap pick vs recorded TCP/joints HTML viz
- ``open3d_rgbd_export`` — offline RGB-D + FK pose dump for Open3D TSDF samples
- ``lidar_camera_calib_export`` — offline Mid-360 CustomMsg vs D555 dump for mount TF
- ``sim_texture`` — PF-R10 generation-6 eval-only texture waiver / spawn-flip / C1-G6

Drivers: ``scripts/detection_gt_gate_run.py``, ``scripts/yolo_two_class_window.py``.
"""
