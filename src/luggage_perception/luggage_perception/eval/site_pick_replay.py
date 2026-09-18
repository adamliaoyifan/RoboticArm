"""Eval-only site-bag pick replay. Not imported by online nodes.

Reads a pendant/real mcap (Humble rosbag2 cannot open the site bags),
joins colour↔depth, runs the current YOLO + platform-free top estimator,
and compares the estimated pick / Cartesian waypoints against recorded
``/joint_states`` and ``/elfin/tcp_pose``. Perception is fully offline:
no ROS graph, so it cannot share the live sim's ``ROS_DOMAIN_ID``.

Optional MoveIt planning is a separate subprocess on an isolated domain
(default 42, never 7) and never executes a trajectory.
"""
from __future__ import division

import base64
import json
import math
import os
from dataclasses import asdict, dataclass, field
from types import SimpleNamespace

import numpy as np

from luggage_perception.eval.bag_frame_join import (
    dedupe_stamped_entries,
    nearest_stamp,
    plan_frame_join,
)
from luggage_perception.eval.bag_mcap_source import (
    COLOR_INFO_TOPIC,
    DEPTH_INFO_TOPIC,
    JOINT_TOPIC,
    TCP_TOPIC,
    TF_STATIC_TOPIC,
    TF_TOPIC,
    decode_color_message,
    decode_depth_message,
    find_mcap_file,
    iter_bag_messages,
    scan_bag,
    select_image_topics,
)
from luggage_perception.eval.bag_replay_index import (
    camera_info_frame_from_payload,
    camera_info_payload as _index_camera_info_payload,
    load_index,
    sidecar_path,
    write_index,
)
from luggage_perception.eval.bag_tf import BagTfBuffer
from luggage_perception.eval.pickup_xy_candidates import (
    strategies_for_frame,
    write_candidates_file,
)
from luggage_perception.eval.isolated_domain import (
    DEFAULT_REPLAY_DOMAIN_ID,
    IsolatedDomainError,
    LIVE_SIM_DOMAIN_ID,
    assert_isolated_domain,
)
from luggage_perception.eval.replay_evaluate import (
    REAL_SITE_CLASS_MAPPING_LABELS,
    REAL_SITE_PROMPTS,
    ReplayEvalConfig,
    _joint_payload,
    _planned_pairs,
    _tcp_payload,
    build_replay_segmenter,
    repaint_label_map,
    select_cargo_detection,
)
from luggage_perception.eval.run_provenance import collect_provenance
from luggage_perception.eval.site_pick_viz import write_viz_html
from luggage_perception.platform_free_pipeline import PlatformFreeDetector
from luggage_perception.ros_message_adapters import camera_info_frame_from_msg
from luggage_perception.semantic_segmenter import draw_detections_overlay
from luggage_perception.top_support_estimator import TopSupportConfig

# Keep in lockstep with luggage_planning.waypoint_generator.DEFAULT_PICK_CLEARANCES
# (perception eval must not import luggage_planning: that package already
# depends on this one).
_PICK_CLEARANCES = {
    "pre_grasp": 0.30,
    "approach": 0.25,
    "attach": 0.0,
    "pick_retreat": 0.35,
    "pre_grasp_min": 0.20,
    "approach_min": 0.08,
}

NS_PER_MS = 1_000_000
LABEL_CARGO = 2
JOINT_NAMES = [
    "elfin_joint1", "elfin_joint2", "elfin_joint3",
    "elfin_joint4", "elfin_joint5", "elfin_joint6",
]


@dataclass
class SitePickConfig(object):
    backend: str = "yolo_world"
    device: str = "cuda"
    model_path: str = ""
    config_yaml: str = ""
    prompts: object = None
    class_mapping_labels: object = None
    confidence: float = 0.3
    min_top_confidence: float = 0.70
    join_tolerance_ms: float = 30.0
    aux_tolerance_ms: float = 50.0
    stride: int = 10
    max_frames: int = 24
    pixel_stride: int = 2
    cargo_select: str = "center_conf"
    center_radius_frac: float = 0.35
    require_backend: bool = True
    world_frame: str = "world"
    scene_tf_config: str = ""
    support_mode: str = "top_only"
    roi_margin: float = 0.5
    auto_workspace: bool = True
    max_cloud_points: int = 1800
    jpeg_quality: int = 70
    plan_moveit: bool = False
    ros_domain_id: int = DEFAULT_REPLAY_DOMAIN_ID
    moveit_timeout_sec: float = 45.0
    # "cargo": the detected box as a MoveIt collision object in the
    # isolated plan-only scene (closes the known "no suitcase collision
    # object" deviation); "cargo_ground" adds a ground slab; "none"
    # keeps the empty scene (explicit, not silent).
    moveit_scene: str = "cargo"
    # Pickup XY benchmark candidates output path (empty = do not emit).
    emit_candidates: str = ""
    # Label mode: frames carry frame_id and the HTML viz accepts clicks
    # that record suction_safe_lid_center_pixel rows (the human judgment
    # lives in pixel space; backfill_pickup_labels converts to world XY).
    label_viz: bool = False
    index_cache_dir: str = ""            # "" = XDG default
    use_index_cache: bool = True
    # TF interpolation (EVAL-wave defect: nearest-stamp lookup carries up
    # to ~20 ms of motion error at 50 Hz). Default OFF — scored
    # comparisons keep the old semantics until re-baselined; the config
    # hash records the state either way.
    tf_interpolate: bool = False
    tf_max_gap_ms: float = 50.0          # one 50 Hz TF period


def _replay_eval_cfg(cfg):
    return ReplayEvalConfig(
        backend=cfg.backend,
        device=cfg.device,
        model_path=cfg.model_path,
        config_yaml=cfg.config_yaml,
        prompts=cfg.prompts if cfg.prompts is not None else list(
            REAL_SITE_PROMPTS),
        class_mapping_labels=(
            cfg.class_mapping_labels if cfg.class_mapping_labels is not None
            else list(REAL_SITE_CLASS_MAPPING_LABELS)),
        confidence=cfg.confidence,
        join_tolerance_ms=cfg.join_tolerance_ms,
        aux_tolerance_ms=cfg.aux_tolerance_ms,
        stride=cfg.stride,
        max_frames=cfg.max_frames,
        require_backend=cfg.require_backend,
        with_points=True,
        cargo_select=cfg.cargo_select,
        center_radius_frac=cfg.center_radius_frac,
    )


def workspace_for_cloud(points, center_xy, half_extents, min_points=50,
                        auto=True):
    """Keep the scene crop when it still sees the cargo; otherwise recenter.

    Site bags are not the sim pickup at ``(-1, 0)``. Eval-only: never
    imported by the live detector.
    """
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[0] == 0:
        return list(center_xy), list(half_extents), "empty"
    cx, cy = float(center_xy[0]), float(center_xy[1])
    hx, hy = float(half_extents[0]), float(half_extents[1])
    inside = (
        (pts[:, 0] >= cx - hx) & (pts[:, 0] <= cx + hx)
        & (pts[:, 1] >= cy - hy) & (pts[:, 1] <= cy + hy)
    )
    if int(inside.sum()) >= int(min_points) or not auto:
        return [cx, cy], [hx, hy], "scene"
    xy = np.median(pts[:, :2], axis=0)
    span = np.ptp(pts[:, :2], axis=0)
    hx = max(0.6, float(span[0]) * 0.5 + 0.15)
    hy = max(0.6, float(span[1]) * 0.5 + 0.15)
    return [float(xy[0]), float(xy[1])], [hx, hy], "cloud"


def _jpeg_b64_bgr(bgr, quality=70):
    import cv2
    ok, buf = cv2.imencode(
        ".jpg", bgr,
        [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        return ""
    return base64.b64encode(buf.tobytes()).decode("ascii")


def _downsample_xyz(points, n, rng):
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[0] == 0:
        return []
    if pts.shape[0] <= n:
        return pts[:, :3].tolist()
    idx = rng.choice(pts.shape[0], int(n), replace=False)
    return pts[idx, :3].tolist()


def _ordered_joints(payload):
    if not payload:
        return None
    by_name = dict(zip(payload.get("joint_names") or [],
                       payload.get("position") or []))
    if not all(name in by_name for name in JOINT_NAMES):
        pos = payload.get("position") or []
        if len(pos) >= 6:
            return [float(v) for v in pos[:6]]
        return None
    return [float(by_name[name]) for name in JOINT_NAMES]


def pick_from_pipeline_result(result):
    """Detector-contract pick fields as a namespace (no ROS messages)."""
    if result is None or not result.top_valid or result.box is None:
        return None
    box = result.box
    top = box.top
    if top is None:
        return None
    if result.height_valid and box.center_xyz is not None:
        center_z = float(box.center_xyz[2])
    else:
        center_z = float(top.top_z)
    return SimpleNamespace(
        x=float(top.center_xy[0]),
        y=float(top.center_xy[1]),
        z=center_z,
        top_z=float(top.top_z),
        width=float(box.width),
        depth=float(box.depth),
        height=float(box.height),
        yaw=float(top.yaw),
        yaw_valid=bool(top.yaw_valid),
        top_surface_valid=True,
        height_valid=bool(result.height_valid),
        height_source=int(result.height_source),
        confidence=float(top.confidence),
    )


def pick_waypoints(pick, suction_z=None):
    """Cartesian pick segments matching ``waypoint_generator`` pick phase."""
    top_z = float(pick.top_z)
    x, y = float(pick.x), float(pick.y)
    pre = _PICK_CLEARANCES["pre_grasp"]
    approach = _PICK_CLEARANCES["approach"]
    attach = _PICK_CLEARANCES["attach"]
    retreat = _PICK_CLEARANCES["pick_retreat"]
    if suction_z is not None:
        gap = max(0.05, float(suction_z) - top_z)
        pre = max(_PICK_CLEARANCES["pre_grasp_min"], 0.6 * gap)
        approach = max(_PICK_CLEARANCES["approach_min"], 0.3 * gap)
    return [
        {"name": "pre_grasp", "type": "pose_target",
         "xyz": [x, y, top_z + pre]},
        {"name": "approach", "type": "cartesian",
         "xyz": [x, y, top_z + approach]},
        {"name": "attach", "type": "cartesian",
         "xyz": [x, y, top_z + attach]},
        {"name": "pick_retreat", "type": "cartesian",
         "xyz": [x, y, top_z + retreat]},
    ]


def _project_uv(xyz_optical, fx, fy, cx, cy):
    z = float(xyz_optical[2])
    if z <= 1e-6 or not math.isfinite(z):
        return None
    u = fx * float(xyz_optical[0]) / z + cx
    v = fy * float(xyz_optical[1]) / z + cy
    return u, v


def _draw_pick(overlay_bgr, uv, waypoints_uv=None):
    import cv2
    if uv is None:
        return overlay_bgr
    u, v = int(round(uv[0])), int(round(uv[1]))
    cv2.drawMarker(overlay_bgr, (u, v), (0, 0, 255),
                   cv2.MARKER_TILTED_CROSS, 28, 2)
    cv2.circle(overlay_bgr, (u, v), 10, (0, 0, 255), 2)
    if waypoints_uv:
        pts = [(int(round(p[0])), int(round(p[1]))) for p in waypoints_uv
               if p is not None]
        for a, b in zip(pts, pts[1:]):
            cv2.line(overlay_bgr, a, b, (0, 255, 0), 2)
    return overlay_bgr


def _workspace_and_catalog(scene_tf_config, roi_margin):
    from luggage_description.box_catalog_utils import (
        box_catalog_entries, load_box_catalog)
    from luggage_description.scene_tf_config_utils import (
        load_scene_tf_config, pickup_source_in_world,
        resolve_scene_tf_config_path)
    path = resolve_scene_tf_config_path(scene_tf_config or None)
    scene = load_scene_tf_config(path)
    source_xyz, _rpy = pickup_source_in_world(scene)
    catalog = box_catalog_entries(load_box_catalog(scene_config=scene))
    return ([float(source_xyz[0]), float(source_xyz[1])],
            [float(roi_margin), float(roi_margin)],
            catalog, path)


def _tcp_in_world(tf_buffer, tcp_row, world_frame, stamp_ns,
                  interpolate=False, max_gap_ns=None):
    if not tcp_row:
        return None
    xyz = np.asarray(tcp_row["position"], dtype=np.float64).reshape(1, 3)
    src = str(tcp_row.get("frame_id") or "")
    if not src or src == world_frame:
        return [float(v) for v in xyz[0]]
    pts = tf_buffer.transform_points(
        xyz, world_frame, src, stamp_ns, interpolate=interpolate,
        max_gap_ns=max_gap_ns)
    if pts is None:
        return [float(v) for v in xyz[0]]
    return [float(v) for v in pts[0]]


def _moveit_scene_objects(mode, pick_dict):
    """BOX collision objects for the isolated plan-only MoveIt scene.

    Cargo center sits at (x, y, top_z - margin - h/2): the pick dict's
    xyz is the top-surface point, the box extends downward from it. The
    1 cm top margin keeps the attach goal (exactly at top_z) numerically
    OFF the collision surface — a goal lying on the box surface is in
    collision and the planner refuses, which measures nothing. Height
    falls back to 0.30 m when the measured height is invalid — a guess
    at the geometry is still better than planning through empty space,
    and the applied object is recorded verbatim so the report says so.
    """
    if mode == "none" or not pick_dict:
        return []
    top_z = float(pick_dict["top_z"])
    width = max(0.05, float(pick_dict["width"]))
    depth = max(0.05, float(pick_dict["depth"]))
    height = (float(pick_dict["height"])
              if pick_dict.get("height_valid") else 0.30)
    x, y = float(pick_dict["xyz"][0]), float(pick_dict["xyz"][1])
    objects = [{
        "id": "replay_cargo",
        "xyz": [x, y, top_z - 0.01 - height / 2.0],
        "dimensions": [width, depth, height],
    }]
    if mode == "cargo_ground":
        objects.append({
            "id": "replay_ground",
            "xyz": [x, y, -0.025],
            "dimensions": [2.0, 2.0, 0.05],
        })
    return objects


def replay_site_pick(bag_path, out_dir, cfg, segmenter=None,
                     source_iter=None, moveit_plan_fn=None):
    """Run one bag. Returns the summary dict (also written to disk)."""
    if cfg.plan_moveit:
        assert_isolated_domain(cfg.ros_domain_id)

    scan = scan_bag(bag_path)
    color_topic, depth_topic = select_image_topics(scan)
    mcap_path = find_mcap_file(bag_path)
    stream = source_iter or (lambda topics: iter_bag_messages(
        mcap_path, topics=topics))
    bag_name = os.path.basename(os.path.normpath(
        scan.bag_path if os.path.isdir(bag_path) else bag_path))
    if bag_name.endswith(".mcap"):
        bag_name = bag_name[:-len(".mcap")]
    os.makedirs(out_dir, exist_ok=True)

    eval_cfg = _replay_eval_cfg(cfg)
    index_topics = [color_topic, depth_topic, COLOR_INFO_TOPIC,
                    DEPTH_INFO_TOPIC, JOINT_TOPIC, TCP_TOPIC,
                    TF_TOPIC, TF_STATIC_TOPIC]
    # Sidecar index (bag_replay_index): on a hit, only /tf and /tf_static
    # still stream (the TF edge series joins the sidecar in the bag_tf
    # array-cache step); injected test streams bypass the cache.
    cached = None
    if cfg.use_index_cache and source_iter is None:
        cached = load_index(mcap_path, scan, color_topic, depth_topic,
                            cfg.index_cache_dir,
                            producer="site_pick_replay")
    color_entries, depth_entries = [], []
    joint_by_stamp, tcp_by_stamp = {}, {}
    info_first = {}
    tf_static_msgs = 0
    tf_buffer = BagTfBuffer()
    if cached is not None:
        joint_by_stamp = {int(s): dict(p)
                          for s, p in cached["joint_payloads"]}
        tcp_by_stamp = {int(s): dict(p)
                        for s, p in cached["tcp_payloads"]}
        info_first = {
            topic: camera_info_frame_from_payload(payload)
            for topic, payload in cached["camera_info"].items()}
        tf_npz = sidecar_path(mcap_path, "tf_edges.npz",
                              cfg.index_cache_dir)
        tf_loaded = False
        if os.path.isfile(tf_npz):
            try:
                tf_buffer.load_edges_npz(tf_npz)
                tf_loaded = True
            except (OSError, ValueError, KeyError):
                tf_loaded = False
        if not tf_loaded:
            # Fall back to streaming /tf (+/tf_static): a missing or
            # corrupt edge file only costs time, never correctness.
            for rec in iter_bag_messages(mcap_path, topics=[
                    TF_TOPIC, TF_STATIC_TOPIC]):
                tf_buffer.add_tf_message(
                    rec.message, static=rec.topic == TF_STATIC_TOPIC)
            # Upgrade a sidecar that predates TF edges (e.g. written by
            # pendant_bag_replay_eval): persist what was just streamed
            # so the next warm run skips the /tf walk entirely.
            if cfg.use_index_cache:
                try:
                    os.makedirs(os.path.dirname(tf_npz), exist_ok=True)
                    tf_buffer.save_edges_npz(tf_npz)
                except OSError:
                    pass
    else:
        pass_a_stream = (
            iter_bag_messages(mcap_path, topics=index_topics,
                              header_only_topics=[color_topic,
                                                  depth_topic])
            if source_iter is None else stream(index_topics))
        for rec in pass_a_stream:
            stamp = rec.header_stamp_ns
            if rec.topic == color_topic:
                color_entries.append((stamp, rec.log_time_ns, None))
            elif rec.topic == depth_topic:
                depth_entries.append((stamp, rec.log_time_ns, None))
            elif rec.topic == JOINT_TOPIC:
                payload = _joint_payload(rec.message)
                if payload is not None:
                    joint_by_stamp[stamp] = payload
            elif rec.topic == TCP_TOPIC:
                tcp_by_stamp[stamp] = _tcp_payload(rec.message)
            elif rec.topic in (COLOR_INFO_TOPIC, DEPTH_INFO_TOPIC):
                if rec.topic not in info_first:
                    info_first[rec.topic] = camera_info_frame_from_msg(
                        rec.message)
            elif rec.topic == TF_STATIC_TOPIC:
                tf_static_msgs += 1
                tf_buffer.add_tf_message(rec.message, static=True)
            elif rec.topic == TF_TOPIC:
                tf_buffer.add_tf_message(rec.message, static=False)

    if cached is not None:
        # Sidecar entries are pre-deduped; duplicates counts ride along.
        color_sorted = [(int(s), int(log), None)
                        for s, log in cached["color_entries"]]
        depth_sorted = [(int(s), int(log), None)
                        for s, log in cached["depth_entries"]]
    else:
        color_sorted, color_dup = dedupe_stamped_entries(color_entries)
        depth_sorted, depth_dup = dedupe_stamped_entries(depth_entries)
    if (cached is None and cfg.use_index_cache and source_iter is None):
        tf_edges_file = None
        try:
            tf_path = sidecar_path(mcap_path, "tf_edges.npz",
                                   cfg.index_cache_dir)
            os.makedirs(os.path.dirname(tf_path), exist_ok=True)
            tf_buffer.save_edges_npz(tf_path)
            tf_edges_file = "tf_edges.npz"
        except OSError:
            tf_edges_file = None
        write_index(mcap_path, {
            "color_topic": color_topic, "depth_topic": depth_topic,
            "color_entries": [[int(s), int(log)]
                              for s, log, _p in color_sorted],
            "color_duplicates": [[int(s), int(n)]
                                 for s, n in sorted(color_dup.items())],
            "depth_entries": [[int(s), int(log)]
                              for s, log, _p in depth_sorted],
            "depth_duplicates": [[int(s), int(n)]
                                 for s, n in sorted(depth_dup.items())],
            "joint_payloads": [[int(s), p] for s, p in
                               sorted(joint_by_stamp.items())],
            "tcp_payloads": [[int(s), p] for s, p in
                             sorted(tcp_by_stamp.items())],
            "camera_info": {topic: _index_camera_info_payload(frame)
                            for topic, frame in info_first.items()},
            "camera_k_variants": {},
            "tf_static_message_count": int(tf_static_msgs),
            # This replay never indexes lidar: an empty list matches the
            # "lidar not indexed" semantics, and replay_evaluate's
            # archive guard forces its own miss whenever it needs more.
            "lidar_stamps": [],
            "tf_edges_file": tf_edges_file,
        }, cfg.index_cache_dir, producer="site_pick_replay")
    plan = plan_frame_join(
        [s for s, _log, _payload in color_sorted],
        [s for s, _log, _payload in depth_sorted],
        tolerance_ns=int(cfg.join_tolerance_ms * NS_PER_MS))
    pairs = _planned_pairs(plan, eval_cfg)
    planned_by_color = {pair.stamp_ns: pair for pair in pairs}

    color_info = info_first.get(COLOR_INFO_TOPIC)
    optical_frame = ""
    # info_first holds CameraInfoFrame objects (converted at ingest, so
    # the sidecar cache round-trips the same shape).
    if color_info is not None:
        optical_frame = str(getattr(color_info, "frame_id", "") or "")
        frame = color_info
    else:
        frame = None

    from luggage_perception.semantic_point_filter import (
        CameraIntrinsics, DepthToColorExtrinsics, SemanticPointFilter)
    point_filter = None
    if frame is not None:
        intrinsics = CameraIntrinsics(
            frame.fx, frame.fy, frame.cx, frame.cy,
            frame.width, frame.height,
            distortion_coeffs=list(frame.distortion_coeffs),
            distortion_model=frame.distortion_model)
        point_filter = SemanticPointFilter(
            intrinsics, intrinsics,
            DepthToColorExtrinsics.identity(), [LABEL_CARGO], [])

    ws_xy, ws_half, catalog, scene_path = _workspace_and_catalog(
        cfg.scene_tf_config, cfg.roi_margin)
    detector = PlatformFreeDetector(
        config=TopSupportConfig(
            workspace_center_xy=ws_xy,
            workspace_half_extents=ws_half,
            min_top_points=50,
            voxel_size=0.01,
        ),
        support_mode=cfg.support_mode,
        catalog_entries=catalog,
        catalog_tolerance=0.08,
    )

    if segmenter is None:
        segmenter, backend, _config_dict = build_replay_segmenter(eval_cfg)
    else:
        backend = str(getattr(segmenter, "last_stats", {}).get(
            "backend", cfg.backend))

    joint_stamps = sorted(joint_by_stamp)
    tcp_stamps = sorted(tcp_by_stamp)
    rng = np.random.RandomState(0)
    pending = {"color": {}, "depth": {}}
    frame_rows = []

    def _aux(stamp_ns, table, stamps):
        hit = nearest_stamp(
            stamps, stamp_ns, int(cfg.aux_tolerance_ms * NS_PER_MS))
        if hit is None:
            return None, None
        idx, dt = hit
        return dict(table[stamps[idx]]), dt / 1e6

    def _flush_pair(color_stamp):
        pair = planned_by_color.pop(color_stamp)
        rgb = pending["color"].pop(color_stamp)
        depth = pending["depth"].pop(pair.depth_stamp_ns)
        frame_rows.append(_process_frame(
            pair, rgb, depth, segmenter, point_filter, cfg, detector,
            tf_buffer, optical_frame, color_info, joint_by_stamp,
            joint_stamps, tcp_by_stamp, tcp_stamps, rng, _aux))

    def _flush_ready():
        while True:
            for color_stamp in sorted(pending["color"]):
                pair = planned_by_color.get(color_stamp)
                if pair is None:
                    pending["color"].pop(color_stamp, None)
                    continue
                if pair.depth_stamp_ns in pending["depth"]:
                    _flush_pair(color_stamp)
                    break
            else:
                return

    planned_depths = {pair.depth_stamp_ns for pair in pairs}
    for rec in stream([color_topic, depth_topic]):
        stamp = rec.header_stamp_ns
        if rec.topic == color_topic:
            if stamp not in planned_by_color:
                continue
            rgb = decode_color_message(rec.message)
            if rgb is None:
                continue
            pending["color"][stamp] = rgb
        elif rec.topic == depth_topic:
            if stamp not in planned_depths:
                continue
            depth = decode_depth_message(rec.message)
            if depth is None:
                continue
            pending["depth"][stamp] = depth
        _flush_ready()
    _flush_ready()

    selected = 0
    best = (-1.0, -1)
    for i, row in enumerate(frame_rows):
        pick = row.get("pick")
        if not pick:
            continue
        score = float(pick.get("confidence") or 0.0)
        if score > best[0]:
            best = (score, i)
            selected = i
    if best[1] < 0:
        for i, row in enumerate(frame_rows):
            if row.get("n_cargo_points", 0) > 0:
                selected = i
                break

    tcp_series = []
    tcp_t = []
    if cfg.label_viz:
        for row in frame_rows:
            # Join key shared with the candidates file: <bag>@<stamp>.
            row["frame_id"] = "%s@%.9f" % (bag_name, row["stamp_sec"])
    for stamp in tcp_stamps:
        row = tcp_by_stamp[stamp]
        xyz = _tcp_in_world(
            tf_buffer, row, cfg.world_frame, stamp,
            interpolate=bool(cfg.tf_interpolate),
            max_gap_ns=int(cfg.tf_max_gap_ms * NS_PER_MS))
        if xyz is None:
            continue
        tcp_t.append(stamp / 1e9)
        tcp_series.append(xyz)
    joint_t, joint_q = [], []
    for stamp in joint_stamps:
        ordered = _ordered_joints(joint_by_stamp[stamp])
        if ordered is None:
            continue
        joint_t.append(stamp / 1e9)
        joint_q.append(ordered)

    planned = None
    scene_objects = []
    if cfg.plan_moveit:
        planner = moveit_plan_fn
        if planner is None:
            from luggage_planning.eval.isolated_moveit_plan import (
                plan_pick_segments)
            planner = plan_pick_segments
        sel = frame_rows[selected] if frame_rows else {}
        start = sel.get("joints")
        segs = sel.get("waypoints") or []
        scene_objects = _moveit_scene_objects(
            cfg.moveit_scene, sel.get("pick"))
        try:
            planned = planner(
                start_joints=start,
                waypoints=segs,
                domain_id=cfg.ros_domain_id,
                timeout_sec=cfg.moveit_timeout_sec,
                collision_objects=scene_objects,
            )
        except IsolatedDomainError:
            raise
        except Exception as exc:  # noqa: BLE001 isolated helper is optional
            planned = {"message": "MoveIt skipped: %s" % exc,
                       "t_sec": [], "q": [], "xyz": []}

    payload = {
        "bag": scan.bag_path,
        "bag_name": bag_name,
        "color_topic": color_topic,
        "depth_topic": depth_topic,
        "scene_tf_config": scene_path,
        "backend": backend,
        "workspace_center_xy": ws_xy,
        "label_mode": bool(cfg.label_viz),
        "domain": {
            "perception": "offline-mcap",
            "moveit": (int(cfg.ros_domain_id) if cfg.plan_moveit else None),
            "reserved": LIVE_SIM_DOMAIN_ID,
        },
        "join": plan.stats,
        "frames": frame_rows,
        "selected": selected,
        "tcp": {"t_sec": tcp_t, "xyz": tcp_series,
                "frame_id": cfg.world_frame},
        "joints": {"t_sec": joint_t, "names": list(JOINT_NAMES),
                   "q": joint_q},
        "planned": planned,
        "moveit_scene": {"mode": cfg.moveit_scene,
                         "objects": scene_objects},
        "tf_frames": sorted(tf_buffer.frames()),
    }
    with open(os.path.join(out_dir, "replay.json"), "w",
              encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)
        handle.write("\n")
    html_path = write_viz_html(
        payload, os.path.join(out_dir, "replay.html"))
    n_candidates = None
    if getattr(cfg, "emit_candidates", ""):
        n_candidates = write_candidates_file(
            cfg.emit_candidates, bag_name, scan.bag_path, frame_rows,
            collect_provenance(asdict(cfg)))
    summary = {
        "bag": scan.bag_path,
        "out_dir": out_dir,
        "html": html_path,
        "backend": backend,
        "frames_processed": len(frame_rows),
        "frames_with_pick": sum(1 for row in frame_rows if row.get("pick")),
        "selected": selected,
        "join": plan.stats,
        "color_topic": color_topic,
        "depth_topic": depth_topic,
        "plan_moveit": bool(cfg.plan_moveit),
        "moveit_scene": cfg.moveit_scene,
        "ros_domain_id": (
            int(cfg.ros_domain_id) if cfg.plan_moveit else None),
        # TF-semantics disclosure: which mode ran and how many frames
        # carried interpolated edges (never silently cross-compare runs
        # with different tf_interpolate — the config hash differs too).
        "tf_interpolate": bool(cfg.tf_interpolate),
        "tf_max_gap_ms": float(cfg.tf_max_gap_ms),
        "n_frames_with_interpolated_tf": sum(
            1 for row in frame_rows if row.get("tf_mode") == "interpolated"),
        "max_tf_gap_ms": max(
            (row["tf_gap_ms"] for row in frame_rows
             if row.get("tf_gap_ms") is not None), default=None),
        # Attribution: revision + the full effective site-pick config
        # (asdict keeps it honest through any future field additions).
        "provenance": collect_provenance(asdict(cfg)),
        "pickup_candidates": (cfg.emit_candidates or None),
        "pickup_candidate_frames": n_candidates,
    }
    with open(os.path.join(out_dir, "summary.json"), "w",
              encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)
        handle.write("\n")
    return summary


def _process_frame(pair, rgb, depth, segmenter, point_filter, cfg, detector,
                   tf_buffer, optical_frame, color_info, joint_by_stamp,
                   joint_stamps, tcp_by_stamp, tcp_stamps, rng, aux_fn):
    stamp_ns = int(pair.stamp_ns)
    stamp_sec = stamp_ns / 1e9
    tf_interp = bool(getattr(cfg, "tf_interpolate", False))
    tf_gap_ns = int(getattr(cfg, "tf_max_gap_ms", 50.0) * NS_PER_MS)
    tf_buffer.reset_stats()
    label_map, detections = segmenter.segment(rgb)
    detections = list(detections or [])
    if label_map is None:
        label_map = np.zeros(rgb.shape[:2], dtype=np.uint8)
    kept = None
    if cfg.cargo_select == "center_conf":
        kept, _info = select_cargo_detection(
            detections, (rgb.shape[1], rgb.shape[0]),
            min_conf=float(cfg.confidence),
            center_radius_frac=cfg.center_radius_frac)
        detections = ([dict(kept)] if kept is not None else [])
        label_map = repaint_label_map(rgb.shape[:2], detections)
    yolo_conf = (float(kept["confidence"]) if kept is not None else None)

    cargo_pts = np.zeros((0, 3), dtype=np.float32)
    if point_filter is not None and depth is not None:
        cargo_pts, _obs = point_filter.filter_depth(
            depth, label_map, pixel_stride=int(cfg.pixel_stride))

    src_frame = optical_frame
    if color_info is not None and not src_frame:
        src_frame = str(color_info.frame_id or "")
    pts_world = None
    tf_ok = False
    if cargo_pts is not None and len(cargo_pts) and src_frame:
        pts_world = tf_buffer.transform_points(
            cargo_pts, cfg.world_frame, src_frame, stamp_ns,
            interpolate=tf_interp, max_gap_ns=tf_gap_ns)
        tf_ok = pts_world is not None
    elif cargo_pts is not None and len(cargo_pts) and not src_frame:
        pts_world = None

    result = None
    reason = "no_cargo"
    ws_src = "none"
    if pts_world is None and len(cargo_pts):
        reason = "DETECT_TF_FAILED"
    elif pts_world is not None and len(pts_world) >= 50:
        ws_xy, ws_half, ws_src = workspace_for_cloud(
            pts_world,
            detector.config.workspace_center_xy,
            detector.config.workspace_half_extents,
            min_points=int(detector.config.min_top_points),
            auto=bool(cfg.auto_workspace))
        detector.config.workspace_center_xy = ws_xy
        detector.config.workspace_half_extents = ws_half
        result = detector.update(
            pts_world, None,
            source="measure",
            geometry_ok=False,
            geometry_gate_reason="status_missing",
            stamp_sec=stamp_sec,
            cargo_segmented=True)
        reason = result.top_reason
        if result.top_valid and float(result.box.top.confidence) < float(
                cfg.min_top_confidence):
            reason = "DETECT_LOW_CONFIDENCE"
            result.top_valid = False
    elif pts_world is not None:
        reason = "DETECT_TOO_FEW_POINTS"

    pick_ns = pick_from_pipeline_result(result)
    joint_row, _jdt = aux_fn(stamp_ns, joint_by_stamp, joint_stamps)
    tcp_row, _tdt = aux_fn(stamp_ns, tcp_by_stamp, tcp_stamps)
    joints = _ordered_joints(joint_row)
    tcp_xyz = _tcp_in_world(tf_buffer, tcp_row, cfg.world_frame, stamp_ns,
                            interpolate=tf_interp, max_gap_ns=tf_gap_ns)
    waypoints = []
    if pick_ns is not None:
        suction_z = None if tcp_xyz is None else float(tcp_xyz[2])
        waypoints = pick_waypoints(pick_ns, suction_z=suction_z)

    overlay = draw_detections_overlay(rgb, detections)
    uv = None
    pick_xyz_world = None
    if pick_ns is not None:
        pick_xyz_world = [float(pick_ns.x), float(pick_ns.y),
                          float(pick_ns.top_z)]
    if pick_xyz_world is not None and color_info is not None and src_frame:
        cam = tf_buffer.transform_points(
            np.asarray(pick_xyz_world).reshape(1, 3),
            src_frame, cfg.world_frame, stamp_ns,
            interpolate=tf_interp, max_gap_ns=tf_gap_ns)
        if cam is not None:
            uv = _project_uv(cam[0], color_info.fx, color_info.fy,
                             color_info.cx, color_info.cy)
    wps_uv = []
    if waypoints and color_info is not None and src_frame:
        for wp in waypoints:
            cam = tf_buffer.transform_points(
                np.asarray(wp["xyz"]).reshape(1, 3),
                src_frame, cfg.world_frame, stamp_ns,
                interpolate=tf_interp, max_gap_ns=tf_gap_ns)
            wps_uv.append(
                None if cam is None else _project_uv(
                    cam[0], color_info.fx, color_info.fy,
                    color_info.cx, color_info.cy))
    overlay = _draw_pick(overlay, uv, wps_uv)

    pick_dict = None
    if pick_ns is not None:
        pick_dict = {
            "xyz": pick_xyz_world,
            "top_z": float(pick_ns.top_z),
            "width": float(pick_ns.width),
            "depth": float(pick_ns.depth),
            "height": float(pick_ns.height),
            "height_valid": bool(pick_ns.height_valid),
            "top_surface_valid": True,
            "yaw_valid": bool(pick_ns.yaw_valid),
            "confidence": float(pick_ns.confidence),
        }

    tf_stats = tf_buffer.stats
    pickup_candidates = None
    if getattr(cfg, "emit_candidates", ""):
        pickup_candidates = strategies_for_frame(
            kept, depth, color_info, pts_world, result,
            float(detector.config.ransac_dist_thresh),
            tf_points_fn=lambda pts: tf_buffer.transform_points(
                pts, cfg.world_frame, src_frame, stamp_ns,
                interpolate=tf_interp, max_gap_ns=tf_gap_ns))
    return {
        "stamp_sec": stamp_sec,
        "join": pair.source,
        "n_detections": len(detections),
        "n_cargo_points": int(len(cargo_pts)),
        "yolo_conf": yolo_conf,
        "reason": reason,
        "workspace_source": ws_src,
        "tf_ok": bool(tf_ok),
        "pickup_candidates": pickup_candidates,
        "tf_mode": ("interpolated"
                    if tf_stats["interpolated_edges"]
                    else ("nearest" if tf_stats["lookups"] else "none")),
        "tf_gap_ms": (tf_stats["max_edge_gap_ns"] / 1e6
                      if tf_stats["interpolated_edges"] else None),
        "optical_frame": src_frame,
        "pick": pick_dict,
        "waypoints": waypoints,
        "joints": joints,
        "tcp_xyz": tcp_xyz,
        "cloud_xyz": _downsample_xyz(pts_world if pts_world is not None
                                     else cargo_pts, cfg.max_cloud_points, rng),
        "overlay_jpeg_b64": _jpeg_b64_bgr(overlay, cfg.jpeg_quality),
    }
