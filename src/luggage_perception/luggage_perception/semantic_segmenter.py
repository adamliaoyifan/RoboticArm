#!/usr/bin/env python3
"""RGB semantic segmentation core (no ROS, no rospy).

Produces a per-pixel label map from an RGB image. Designed so the heavy ML
dependencies (torch, ultralytics, sam2) are imported lazily inside the
backend constructor; the module itself imports cleanly without them, letting
the unit tests run in a bare Python environment and letting the ROS node
fall back to a stub backend when the deps are missing.

Label convention (matches the plan in
.cursor/plans/semantic_perception_pipeline_4ade087b.plan.md):

    0 = background       (floor / wall / ceiling / unrelated)
    1 = container_wall
    2 = cargo            (suitcases / luggage / boxes of interest)
    3 = robot_arm
    4 = unknown_object   (unidentified but possibly an obstacle)

Backend implementations map free-text YOLO-World class prompts onto these
integer labels via a ``class_mapping`` dict provided in the config.
"""

from __future__ import division

import warnings
from dataclasses import dataclass

import numpy as np


LABEL_BACKGROUND = 0
LABEL_CONTAINER_WALL = 1
LABEL_CARGO = 2
LABEL_ROBOT_ARM = 3
LABEL_UNKNOWN = 4

DEFAULT_LABEL_NAMES = {
    LABEL_BACKGROUND: "background",
    LABEL_CONTAINER_WALL: "container_wall",
    LABEL_CARGO: "cargo",
    LABEL_ROBOT_ARM: "robot_arm",
    LABEL_UNKNOWN: "unknown_object",
}

# YOLO-World ``predict(conf=)`` floor. Boxes below this never leave the
# model. Hardware-tunable on ``/semantic_segmenter``.
DEFAULT_YOLO_CONFIDENCE_THRESHOLD = 0.2
# YOLO robot_arm boxes below this do not stamp class 3. Geometric
# self-body (panel mesh + row-band) is separate.
DEFAULT_ROBOT_ARM_CONFIDENCE_THRESHOLD = 0.5
# After workspace acceptance, keep one cargo AABB: the highest-confidence
# compact box at or above this floor. Raise independently of the YOLO
# floor if overlay should still show weaker proposals.
DEFAULT_CARGO_MIN_CONFIDENCE = 0.2
LIVE_CONFIDENCE_PARAMS = (
    "confidence_threshold",
    "cargo_min_confidence",
    "robot_arm_confidence_threshold",
)
_LIVE_CONFIDENCE_ATTR = {
    "confidence_threshold": "confidence_threshold",
    "cargo_min_confidence": "cargo_min_confidence",
    "robot_arm_confidence_threshold": "robot_arm_confidence_threshold",
}


def set_live_confidence_param(segmenter, name, value):
    """Apply a hardware-tunable floor. Returns ``(ok, reason)``."""
    attr = _LIVE_CONFIDENCE_ATTR.get(str(name))
    if attr is None:
        return False, "not a live confidence param: %s" % name
    try:
        val = float(value)
    except (TypeError, ValueError):
        return False, "%s must be a number" % name
    if not 0.0 <= val <= 1.0:
        return False, "%s must be in [0, 1]" % name
    setattr(segmenter, attr, val)
    return True, ""


def confidence_floors(segmenter):
    """Current YOLO / cargo / robot-arm floors for stats and logs."""
    return {
        "confidence_threshold": float(segmenter.confidence_threshold),
        "cargo_min_confidence": float(segmenter.cargo_min_confidence),
        "robot_arm_confidence_threshold": float(
            segmenter.robot_arm_confidence_threshold),
    }

# BGR palette shared by colorize_label_map and the detection overlay so the
# debug viz matches the published colorized mask. Cargo is red so the primary
# task object stands out.
LABEL_PALETTE_BGR = {
    LABEL_BACKGROUND: (40, 40, 40),
    LABEL_CONTAINER_WALL: (180, 180, 180),
    LABEL_CARGO: (0, 0, 220),
    LABEL_ROBOT_ARM: (220, 220, 0),
    LABEL_UNKNOWN: (0, 140, 255),
}


def _resolve_class_mapping(prompts, class_mapping):
    """Build prompt_index -> label_id map from config.

    ``class_mapping`` may map either prompt text or label name to a label id.
    Missing entries default to LABEL_UNKNOWN so detections are kept rather
    than silently dropped.
    """
    if class_mapping is None:
        class_mapping = {}
    name_to_label = {v: k for k, v in DEFAULT_LABEL_NAMES.items()}
    mapping = {}
    for prompt in prompts:
        label_id = class_mapping.get(prompt)
        if label_id is None:
            label_id = name_to_label.get(prompt)
        if label_id is None:
            label_id = class_mapping.get(prompt.lower())
        if label_id is None:
            label_id = LABEL_UNKNOWN
        mapping[prompt] = int(label_id)
    return mapping


@dataclass(frozen=True)
class WorkspaceAcceptanceContext:
    """Per-frame geometry for the accepted-detection predicate.

    All fields are plain numbers so the context can be frozen per frame and
    replayed offline in tests. ``optical_to_world`` is a row-major 3x4 (R|t)
    transform of the camera optical frame in the world frame at the image
    stamp; ``fx/fy/cx/cy`` are the live pinhole intrinsics of the same
    stamp. ``plane_z``/``center_xy``/``half_xy``/``margin`` describe the
    pickup workspace: a ground plane at the platform top and the workspace
    XY region (measured static geometry, the same kind the detector already
    crops its point cloud with).
    """

    fx: float
    fy: float
    cx: float
    cy: float
    plane_z: float
    center_xy: tuple
    half_xy: tuple
    margin: float
    optical_to_world: tuple = None  # row-major 3x4 (R|t), optical -> world

    def available(self):
        return self.optical_to_world is not None and self.fx > 0.0


def bbox_center_on_plane(bbox, ctx):
    """Intersect the bbox-center pixel ray with the workspace plane.

    Returns (x, y) in the world frame, or None when the ray never descends
    to the plane (optical axis parallel to it) or the context is missing.
    """
    if not ctx.available() or bbox is None or len(bbox) < 4:
        return None
    u = 0.5 * (float(bbox[0]) + float(bbox[2]))
    v = 0.5 * (float(bbox[1]) + float(bbox[3]))
    m = ctx.optical_to_world  # row-major 3x4 (R|t)
    r11, r12, r13, tx = m[0]
    r21, r22, r23, ty = m[1]
    r31, r32, r33, tz = m[2]
    d_opt = ((u - ctx.cx) / ctx.fx, (v - ctx.cy) / ctx.fy, 1.0)
    dx = r11 * d_opt[0] + r12 * d_opt[1] + r13 * d_opt[2]
    dy = r21 * d_opt[0] + r22 * d_opt[1] + r23 * d_opt[2]
    dz = r31 * d_opt[0] + r32 * d_opt[1] + r33 * d_opt[2]
    if dz > -1e-6:
        # Ray does not descend toward the workspace plane (nadir-ish views
        # only); no decision rather than an invented intersection.
        return None
    s = (ctx.plane_z - tz) / dz
    return (tx + s * dx, ty + s * dy)


def evaluate_detection_acceptance(bbox, ctx, image_shape=None):
    """Accepted-detection predicate for one bbox (PF-R8 A1).

    A detection is accepted when its bbox-center ray lands on the workspace
    ground plane within ``max(half_xy) + margin`` of the workspace centre.
    The radial distance is measured in the world plane, so the rule needs
    no camera-yaw knowledge: a nadir fixture geometry decides it exactly.
    An edge-clipped suitcase whose visible centre still projects onto the
    platform is accepted; static structures outside the platform (the robot
    pedestal at the image border, the container wall) are rejected.

    Returns ``(accepted, reason)``. Compact in-workspace cargo still fails
    open when camera_info or stamped TF is missing (flagged, not faked).
    An edge-strip bbox may not ride that fail-open path: without a
    workspace decision it is rejected as ``predicate_unavailable_edge_strip``.
    """
    if ctx is None or not ctx.available():
        if image_shape is not None and len(image_shape) >= 2:
            height, width = int(image_shape[0]), int(image_shape[1])
            if bbox_is_edge_strip(bbox, width, height):
                return False, "predicate_unavailable_edge_strip"
        return True, "predicate_unavailable"
    point = bbox_center_on_plane(bbox, ctx)
    if point is None:
        return False, "ray_off_plane"
    dx = point[0] - float(ctx.center_xy[0])
    dy = point[1] - float(ctx.center_xy[1])
    accept_radius = max(float(ctx.half_xy[0]), float(ctx.half_xy[1])) \
        + float(ctx.margin)
    if (dx * dx + dy * dy) <= accept_radius * accept_radius:
        return True, "in_workspace"
    return False, "outside_workspace"


def restrict_cargo_mask_to_accepted(label_map, detections, instance_map=None,
                                    cargo_label=LABEL_CARGO):
    """Drop cargo pixels that are not covered by an accepted cargo bbox.

    ``bbox_fill`` paints every YOLO AABB, including outside-workspace false
    positives. The accepted-detection predicate already annotates those
    boxes; this makes the published mask match that verdict so the
    detector does not RANSAC a pedestal/platform cloud.

    Detections with no ``accepted`` key keep historic accept-all behaviour.
    """
    labels = np.array(label_map, copy=True)
    inst_out = (None if instance_map is None
                else np.array(instance_map, copy=True))
    cargo = labels == int(cargo_label)
    if not cargo.any():
        return labels, inst_out, 0
    keep = np.zeros(labels.shape, dtype=bool)
    saw_verdict = False
    h, w = labels.shape[:2]
    for det in detections or ():
        if int(det.get("label", -1)) != int(cargo_label):
            continue
        if "accepted" not in det:
            continue
        saw_verdict = True
        if not det.get("accepted"):
            continue
        bbox = det.get("bbox") or ()
        if len(bbox) < 4:
            continue
        x1 = max(0, min(w, int(bbox[0])))
        y1 = max(0, min(h, int(bbox[1])))
        x2 = max(0, min(w, int(bbox[2])))
        y2 = max(0, min(h, int(bbox[3])))
        if x2 > x1 and y2 > y1:
            keep[y1:y2, x1:x2] = True
    if not saw_verdict:
        return labels, inst_out, 0
    drop = cargo & ~keep
    n_drop = int(drop.sum())
    labels[drop] = LABEL_BACKGROUND
    if inst_out is not None and n_drop:
        inst_out[drop] = 0
    return labels, inst_out, n_drop


def bbox_touches_border(bbox, width, height, margin=12):
    if bbox is None or len(bbox) < 4:
        return True
    x1, y1, x2, y2 = (int(v) for v in bbox[:4])
    m = int(margin)
    return (x1 <= m or y1 <= m or x2 >= int(width) - m
            or y2 >= int(height) - m)


def bbox_is_edge_strip(bbox, width, height, margin=12, max_strip_px=80):
    """True when a border-clipped box is a thin pedestal/wall strip.

    A suitcase that merely touches the image edge still has a compact
    AABB (one side well above ``max_strip_px``). The documented base_link
    false positive is a ~30 px strip glued to the right edge. Only the
    strip should lose to an inner suitcase; dropping every border-touching
    box also drops a clipped suitcase when a small inner false positive
    exists.
    """
    if bbox is None or len(bbox) < 4:
        return False
    x1, y1, x2, y2 = (int(v) for v in bbox[:4])
    m = int(margin)
    cap = int(max_strip_px)
    width = int(width)
    height = int(height)
    if width < 160 or height < 120:
        # Fixture / cropped images are smaller than a pedestal strip on
        # the 640x480 pickup_observe frame; do not classify them.
        return False
    bw = max(0, x2 - x1)
    bh = max(0, y2 - y1)
    if x1 <= m and bw <= cap:
        return True
    if x2 >= int(width) - m and bw <= cap:
        return True
    if y1 <= m and bh <= cap:
        return True
    if y2 >= int(height) - m and bh <= cap:
        return True
    return False


def unaccept_border_cargo_when_inner_exists(detections, image_shape,
                                           cargo_label=LABEL_CARGO,
                                           margin=12, max_strip_px=80,
                                           min_conf=DEFAULT_CARGO_MIN_CONFIDENCE):
    """Keep the highest-confidence suitcase; drop the rest.

    Pedestal/container AABBs sit on the image edge as thin strips. A
    full-frame 0.015 cargo box can also pass the workspace predicate and
    used to win by area (``max(compact, key=area)``), erasing the 0.589
    suitcase. The live rule matches the replay selector: among accepted
    cargo with ``confidence >= min_conf``, prefer a compact (non-strip)
    box and keep the max-confidence one. Nothing at or above the floor
    means an honest miss — a sub-threshold box is never the suitcase.
    """
    dets = list(detections or [])
    h, w = (int(image_shape[0]), int(image_shape[1])) if image_shape else (0, 0)
    if w <= 0 or h <= 0:
        return dets, 0

    def _conf(det):
        return float(det.get("confidence") or 0.0)

    accepted = [d for d in dets
                if int(d.get("label", -1)) == int(cargo_label)
                and d.get("accepted")]
    if not accepted:
        return dets, 0
    floor = float(min_conf)
    above = [d for d in accepted if _conf(d) >= floor]
    compact_above = [
        d for d in above
        if not bbox_is_edge_strip(d.get("bbox"), w, h, margin, max_strip_px)]
    # Never promote a pedestal/wall strip just because it is the only box
    # above the cargo floor. A clipped suitcase is compact, not a strip.
    pool = compact_above
    if not pool:
        n_drop = 0
        for det in accepted:
            det["accepted"] = False
            if bbox_is_edge_strip(
                    det.get("bbox"), w, h, margin, max_strip_px):
                det["accept_reason"] = "edge_strip_rejected"
            else:
                det["accept_reason"] = "cargo_below_min_conf"
            n_drop += 1
        return dets, n_drop
    primary = max(pool, key=_conf)
    n_drop = 0
    for det in accepted:
        if det is primary:
            continue
        det["accepted"] = False
        if _conf(det) < floor:
            det["accept_reason"] = "cargo_below_min_conf"
        else:
            det["accept_reason"] = "border_cargo_with_inner"
        n_drop += 1
    return dets, n_drop


def _setup_clip_vendor():
    """Make the vendored CLIP package + deps importable offline.

    YOLO-World backends need OpenAI CLIP (``import clip``) to embed the text
    prompts. Ultralytics auto-installs it via ``pip install
    git+https://github.com/ultralytics/CLIP.git`` on first ``set_classes()``,
    which is slow / fragile behind proxies and unreachable offline. Instead we
    vendor CLIP + its ``ftfy``/``regex`` deps + the ViT-B/32 weights under
    ``luggage_perception/vendor/`` and wire them up here so ``import clip``
    resolves to the vendored copy and ``clip.load("ViT-B/32")`` finds the
    cached weights without a 338 MB download.

    Looks for ``vendor/`` next to this module's parent dir; override the
    location with ``$LUGGAGE_CLIP_VENDOR_DIR`` (useful when the repo is mounted
    at a different path inside a container). Silently no-ops when the folder is
    absent, falling back to ultralytics' default auto-install behavior.
    """
    import os
    import sys

    module_dir = os.path.dirname(os.path.abspath(__file__))
    candidates = []
    env_base = os.environ.get("LUGGAGE_CLIP_VENDOR_DIR")
    if env_base:
        candidates.append(env_base)
    # Source tree: <pkg>/luggage_perception/../vendor
    candidates.append(os.path.join(module_dir, "..", "vendor"))
    # Installed tree: the ament python package sits anywhere inside
    # <prefix>/.../luggage_perception, and the vendor is installed at
    # <prefix>/share/luggage_perception/vendor. The install layout depth
    # differs per distro (lib/site-packages vs local/lib/dist-packages),
    # so probe upward instead of hard-coding the hop count.
    probe = module_dir
    for _ in range(8):
        probe = os.path.dirname(probe)
        candidates.append(os.path.join(
            probe, "share", "luggage_perception", "vendor"))
    base = next((c for c in candidates if os.path.isdir(
        os.path.join(c, "clip_pkg"))), candidates[0])
    base = os.path.normpath(base)
    for sub in ("clip_pkg",):
        path = os.path.join(base, sub)
        if os.path.isdir(path) and path not in sys.path:
            sys.path.insert(0, path)
    # CLIP deps (ftfy, regex): the vendored regex ships a CPython-specific C
    # extension, so a deps dir built for one ABI (e.g. cp310 on the host) will
    # not import under another (e.g. cp38 in the Noetic container). Add it only
    # when the deps actually load; otherwise leave ftfy/regex to the
    # interpreter's own site-packages (each environment pip-installs them).
    deps = os.path.join(base, "deps")
    if os.path.isdir(deps) and deps not in sys.path:
        sys.path.insert(0, deps)
        try:
            import regex  # noqa: F401
            import ftfy  # noqa: F401
        except ImportError:
            sys.path.remove(deps)
            sys.modules.pop("regex", None)
            sys.modules.pop("_regex", None)
            sys.modules.pop("ftfy", None)
    # clip.load("ViT-B/32") reads ~/.cache/clip/ViT-B-32.pt and skips the
    # download when the sha matches. Symlink the vendored checkpoint there.
    weights = os.path.join(base, "clip_models", "ViT-B-32.pt")
    if os.path.isfile(weights):
        cache_dir = os.path.expanduser(os.path.join("~", ".cache", "clip"))
        target = os.path.join(cache_dir, "ViT-B-32.pt")
        try:
            os.makedirs(cache_dir, exist_ok=True)
            if os.path.islink(target) and not os.path.exists(target):
                os.remove(target)
            if not os.path.exists(target):
                os.symlink(weights, target)
        except OSError:
            # Read-only home or cross-device symlink refused - leave it; clip
            # will fall back to downloading (still works online).
            pass
    return base


@dataclass(frozen=True)
class SegmenterOutput:
    """Frozen snapshot of one segmentation result.

    stamp/frame_id come from the input message header (never ``now()``).
    ``copy()`` re-copies every mutable field so a consumer can never alias
    internal buffers across frames.
    """

    stamp: float
    frame_id: str
    label_map: np.ndarray        # HxW uint8
    detections: tuple            # tuple of dict, already copied
    instance_map: np.ndarray     # HxW uint16 or None
    stats: dict
    generation: int = 0
    instance_id: str = ""

    def copy(self):
        return SegmenterOutput(
            stamp=self.stamp,
            frame_id=self.frame_id,
            label_map=np.copy(self.label_map),
            detections=tuple(_copy_detection(d) for d in self.detections),
            instance_map=(None if self.instance_map is None
                          else np.copy(self.instance_map)),
            stats=dict(self.stats),
            generation=int(self.generation or 0),
            instance_id=str(self.instance_id or ""),
        )


def _copy_detection(det):
    """Copy one detection dict, including its (optional) HxW bool mask."""
    out = dict(det)
    mask = out.get("mask")
    if mask is not None:
        out["mask"] = np.copy(mask)
    return out


def cargo_detection_count(detections, include_held=False):
    """Number of cargo boxes. Held (temporal) boxes are excluded by default."""
    n = 0
    for det in detections or []:
        if int(det.get("label", -1)) != LABEL_CARGO:
            continue
        if det.get("held") and not include_held:
            continue
        n += 1
    return n


def compact_detections(detections):
    """JSON-safe detection list (no masks) for per-frame stats."""
    out = []
    for det in detections or []:
        item = {
            "label": int(det.get("label", -1)),
            "prompt": str(det.get("prompt", "")),
            "confidence": float(det.get("confidence", 0.0) or 0.0),
            "held": bool(det.get("held", False)),
        }
        bbox = det.get("bbox")
        if bbox is not None and len(bbox) >= 4:
            item["bbox"] = [int(round(float(v))) for v in bbox[:4]]
        if "accepted" in det:
            item["accepted"] = bool(det["accepted"])
            item["accept_reason"] = str(det.get("accept_reason", ""))
        if "self_body_overlap" in det:
            try:
                item["self_body_overlap"] = float(det["self_body_overlap"])
            except (TypeError, ValueError):
                item["self_body_overlap"] = None
        if det.get("dropped"):
            item["dropped"] = str(det.get("dropped"))
        out.append(item)
    return out


def bbox_mask_overlap(bbox, mask):
    """Fraction of the bbox rectangle that sits on a boolean mask."""
    if bbox is None or mask is None or len(bbox) < 4:
        return None
    body = np.asarray(mask)
    if body.ndim != 2 or body.size == 0:
        return None
    height, width = body.shape[:2]
    x1, y1, x2, y2 = (int(v) for v in bbox[:4])
    x1, x2 = max(0, min(width, x1)), max(0, min(width, x2))
    y1, y2 = max(0, min(height, y1)), max(0, min(height, y2))
    if x2 <= x1 or y2 <= y1:
        return None
    patch = body[y1:y2, x1:x2]
    if patch.size == 0:
        return None
    return float(patch.mean())


def detections_dropped_by_self_body(before, after, body_mask):
    """Cargo boxes present in *before* but removed by ``apply_self_body_mask``."""
    after_keys = set()
    for det in after or []:
        bbox = det.get("bbox")
        if bbox is not None and len(bbox) >= 4:
            after_keys.add(tuple(int(round(float(v))) for v in bbox[:4]))
    dropped = []
    for det in before or []:
        try:
            label = int(det.get("label", -1))
        except (TypeError, ValueError):
            continue
        if label != LABEL_CARGO:
            continue
        bbox = det.get("bbox")
        key = None
        if bbox is not None and len(bbox) >= 4:
            key = tuple(int(round(float(v))) for v in bbox[:4])
        if key is not None and key in after_keys:
            continue
        item = compact_detections([det])[0]
        item["dropped"] = "self_body"
        item["self_body_overlap"] = bbox_mask_overlap(bbox, body_mask)
        dropped.append(item)
    return dropped


def clip_bbox_xyxy(bbox, width, height):
    """Integer xyxy clipped to an image. None if empty after clip."""
    if bbox is None or len(bbox) < 4:
        return None
    x1 = max(0, min(int(width), int(bbox[0])))
    y1 = max(0, min(int(height), int(bbox[1])))
    x2 = max(0, min(int(width), int(bbox[2])))
    y2 = max(0, min(int(height), int(bbox[3])))
    if x2 <= x1 or y2 <= y1:
        return None
    return x1, y1, x2, y2


def robot_arm_below_conf_floor(label_id, confidence, min_conf):
    """True when a YOLO robot_arm box is below the class-3 paint floor."""
    return (int(label_id) == LABEL_ROBOT_ARM
            and float(min_conf) > 0.0
            and float(confidence) < float(min_conf))


def paint_detection_on_label_map(label_map, det):
    """Fill *det* onto *label_map* (instance mask if present, else bbox)."""
    label_id = int(det.get("label", LABEL_BACKGROUND))
    mask = det.get("mask")
    if mask is not None:
        m = np.asarray(mask)
        if m.shape[:2] == label_map.shape[:2] and m.any():
            label_map[np.asarray(m, dtype=bool)] = label_id
            return
    box = clip_bbox_xyxy(det.get("bbox"), label_map.shape[1], label_map.shape[0])
    if box is None:
        return
    x1, y1, x2, y2 = box
    label_map[y1:y2, x1:x2] = label_id


def suppress_low_conf_robot_arm(label_map, detections, min_conf,
                                instance_map=None):
    """Drop YOLO robot_arm boxes below *min_conf* and restore overwritten pixels.

    ``bbox_fill`` paints later detections over earlier ones, so a 0.006
    ``robot arm`` AABB can erase an accepted cargo box. Geometric self-body
    stamping happens after this and is not affected. Returns
    ``(label_map, detections, instance_map, n_dropped)``.
    """
    dets = list(detections or [])
    labels = np.asarray(label_map)
    if float(min_conf) <= 0.0 or labels.size == 0:
        return labels, dets, instance_map, 0
    kept = []
    n_dropped = 0
    for det in dets:
        if robot_arm_below_conf_floor(
                det.get("label", -1), det.get("confidence", 0.0), min_conf):
            n_dropped += 1
            continue
        kept.append(det)
    if n_dropped == 0:
        return labels, dets, instance_map, 0
    out = np.zeros(labels.shape[:2], dtype=np.uint8)
    inst_out = None
    if instance_map is not None:
        inst_out = np.zeros(
            labels.shape[:2], dtype=np.asarray(instance_map).dtype)
    for i, det in enumerate(kept, start=1):
        paint_detection_on_label_map(out, det)
        if inst_out is None:
            continue
        inst_id = int(det.get("instance_id") or i)
        mask = det.get("mask")
        if mask is not None:
            m = np.asarray(mask)
            if m.shape[:2] == inst_out.shape[:2] and m.any():
                inst_out[np.asarray(m, dtype=bool)] = inst_id
                continue
        box = clip_bbox_xyxy(
            det.get("bbox"), inst_out.shape[1], inst_out.shape[0])
        if box is None:
            continue
        x1, y1, x2, y2 = box
        inst_out[y1:y2, x1:x2] = inst_id
    return out, kept, inst_out, n_dropped


def apply_self_body_mask(label_map, detections, body_mask,
                         label_id=LABEL_ROBOT_ARM, instance_map=None,
                         overlap_drop=0.5):
    """Paint *body_mask* as *label_id* and drop detections that sit on it.

    *body_mask* is HxW bool in the same frame as *label_map*. Detections
    whose bbox overlaps the mask by >= *overlap_drop* are dropped (the
    panel YOLO box, not a suitcase that merely grazes the arc). Inputs
    are not mutated. Returns (label_map, detections, instance_map, n_self).
    """
    labels = np.asarray(label_map)
    dets = list(detections or [])
    inst = None if instance_map is None else np.asarray(instance_map)
    if body_mask is None or labels.size == 0:
        return labels, dets, inst, 0
    mask = np.asarray(body_mask, dtype=bool)
    if mask.shape != labels.shape[:2] or not mask.any():
        return labels, dets, inst, 0
    out = np.array(labels, copy=True, dtype=np.uint8)
    out[mask] = int(label_id)
    n_self = int(mask.sum())
    if inst is not None:
        inst = np.array(inst, copy=True)
        inst[mask] = 0
    kept = []
    height, width = mask.shape
    for det in dets:
        item = dict(det)
        bbox = item.get("bbox")
        if bbox is None or len(bbox) < 4:
            kept.append(item)
            continue
        x1, y1, x2, y2 = (int(v) for v in bbox[:4])
        x1, x2 = max(0, min(width, x1)), max(0, min(width, x2))
        y1, y2 = max(0, min(height, y1)), max(0, min(height, y2))
        if x2 <= x1 or y2 <= y1:
            continue
        patch = mask[y1:y2, x1:x2]
        if patch.size and float(patch.mean()) >= float(overlap_drop):
            continue
        det_mask = item.get("mask")
        if det_mask is not None:
            m = np.array(det_mask, copy=True)
            if m.shape[:2] == mask.shape:
                m[mask] = False
            item["mask"] = m
        kept.append(item)
    return out, kept, inst, n_self


def row_band_mask(shape, row_start_frac):
    """Bottom row-band as HxW bool. All-False when frac is outside (0, 1)."""
    height, width = int(shape[0]), int(shape[1])
    band = np.zeros((max(0, height), max(0, width)), dtype=bool)
    frac = float(row_start_frac)
    if frac <= 0.0 or frac >= 1.0 or band.size == 0:
        return band
    y0 = max(0, min(height, int(round(height * frac))))
    band[y0:, :] = True
    return band


def combined_self_body_mask(mesh_mask, row_start_frac, shape):
    """Union of the projected panel silhouette and the bottom row band.

    The mesh silhouette only covers the panel centre (measured x in
    [202, 481] of 640). YOLO keeps scoring the flanks that stick out past it
    as cargo, and those boxes overlap the silhouette too little to be
    dropped, so every frame carried a false cargo blob. Both masks together,
    not one or the other.
    """
    band = row_band_mask(shape, row_start_frac)
    if mesh_mask is None:
        return band
    mesh = np.asarray(mesh_mask, dtype=bool)
    if mesh.shape != band.shape:
        return band
    return mesh | band


def apply_wrist_self_body(label_map, detections, row_start_frac,
                          label_id=LABEL_ROBOT_ARM, instance_map=None):
    """Fallback: paint a bottom row-band. Prefer a projected mesh mask."""
    labels = np.asarray(label_map)
    if labels.size == 0:
        inst = None if instance_map is None else np.asarray(instance_map)
        return labels, list(detections or []), inst, 0
    band = row_band_mask(labels.shape[:2], row_start_frac)
    if not band.any():
        inst = None if instance_map is None else np.asarray(instance_map)
        return labels, list(detections or []), inst, 0
    return apply_self_body_mask(
        labels, detections, band, label_id=label_id,
        instance_map=instance_map)



class SemanticSegmenter:
    """Base interface. Subclasses implement ``segment``."""

    def __init__(self, prompts, class_mapping=None, confidence_threshold=DEFAULT_YOLO_CONFIDENCE_THRESHOLD):
        self.prompts = list(prompts)
        self.class_mapping = _resolve_class_mapping(prompts, class_mapping)
        self.confidence_threshold = float(confidence_threshold)
        self._last_stats = {
            "backend": "base",
            "inference_ms": 0.0,
            "detection_count": 0,
            "label_counts": {label: 0 for label in DEFAULT_LABEL_NAMES},
        }
        self._instance_map = None
        self._output = None
        # 0 disables the row-band fallback. A projected mesh mask, when set,
        # takes priority (see apply_self_body_mask).
        self.self_body_row_start_frac = 0.0
        self.self_body_mask = None
        self.temporal_gate = None
        self.robot_arm_confidence_threshold = (
            DEFAULT_ROBOT_ARM_CONFIDENCE_THRESHOLD)
        self.cargo_min_confidence = DEFAULT_CARGO_MIN_CONFIDENCE
        # Per-frame WorkspaceAcceptanceContext set by the ROS node right
        # before update(); None disables the accepted-detection predicate.
        self.workspace_ctx = None

    @property
    def last_stats(self):
        return dict(self._last_stats)

    @property
    def instance_map(self):
        """Deprecated: read the last result via ``copy_output()`` instead."""
        warnings.warn(
            "SemanticSegmenter.instance_map is deprecated; use copy_output()",
            DeprecationWarning,
            stacklevel=2,
        )
        return None if self._instance_map is None else np.copy(self._instance_map)

    def update(self, rgb_uint8, stamp, frame_id, generation=0, instance_id=""):
        """Run ``segment()`` and store a deep-copied snapshot.

        The backends may reuse internal buffers between calls, so every array
        is copied here; a previously returned ``SegmenterOutput`` is never
        mutated by a later ``update``. No publishing, no I/O. ``stamp`` /
        ``frame_id`` must come from the input header, not from wall clock.
        ``generation`` / ``instance_id`` are the task epoch from ingest
        (optional ``/luggage/current_box``); they are copied onto the output
        and never used as geometry.

        Panel pixels are painted letterbox-grey *before* ``segment()`` so
        YOLO never proposes the suction panel as cargo. ``apply_self_body_mask``
        still runs afterwards as a fallback.
        """
        t_body = self._time.perf_counter() if hasattr(self, "_time") else None
        body = combined_self_body_mask(
            self.self_body_mask, self.self_body_row_start_frac,
            rgb_uint8.shape[:2])
        rgb_in = rgb_uint8
        if body is not None and np.any(body):
            rgb_in = np.array(rgb_uint8, copy=True)
            rgb_in[body] = 114
        t_seg = self._time.perf_counter() if t_body is not None else None
        label_map, detections = self.segment(rgb_in)
        t_after_seg = self._time.perf_counter() if t_body is not None else None
        label_map, detections, instance_map, n_arm_drop = (
            suppress_low_conf_robot_arm(
                label_map, detections, self.robot_arm_confidence_threshold,
                instance_map=self._instance_map))
        if n_arm_drop:
            self._instance_map = instance_map
        detections_before = list(detections or [])
        n_before = cargo_detection_count(detections_before)
        label_map, detections, instance_map, n_self = apply_self_body_mask(
            label_map, detections, body, instance_map=self._instance_map)
        n_after = cargo_detection_count(detections)
        stats = dict(self._last_stats)
        if t_after_seg is not None:
            stage = dict(stats.get("stage_ms") or {})
            stage["self_body_prep"] = round((t_seg - t_body) * 1000.0, 2)
            stage["after_segment"] = round(
                (self._time.perf_counter() - t_after_seg) * 1000.0, 2)
            stats["stage_ms"] = stage
        stats["n_dropped_low_conf_robot_arm"] = int(n_arm_drop)
        stats.update(confidence_floors(self))
        stats["n_yolo_cargo_before_self_body"] = int(n_before)
        stats["n_dropped_self_body"] = int(max(0, n_before - n_after))
        stats["detections_dropped_self_body"] = detections_dropped_by_self_body(
            detections_before, detections, body)
        stats["raw_cargo"] = bool(n_after > 0)
        stats["held"] = False
        # Accepted-detection predicate (PF-R8 A1): annotate every cargo
        # detection with the workspace-projection verdict before anything
        # downstream (the temporal vote) treats it as a positive sample.
        accepted_reasons = {}
        n_accepted = 0
        for det in detections:
            if int(det.get("label", -1)) != LABEL_CARGO:
                continue
            accepted, reason = evaluate_detection_acceptance(
                det.get("bbox"), self.workspace_ctx,
                image_shape=rgb_uint8.shape[:2])
            det["accepted"] = bool(accepted)
            det["accept_reason"] = str(reason)
            accepted_reasons[str(reason)] = (
                accepted_reasons.get(str(reason), 0) + 1)
            if accepted:
                n_accepted += 1
        detections, n_border = unaccept_border_cargo_when_inner_exists(
            detections, rgb_uint8.shape[:2],
            min_conf=self.cargo_min_confidence)
        accepted_reasons = {}
        n_accepted = 0
        for det in detections:
            if int(det.get("label", -1)) != LABEL_CARGO:
                continue
            reason = str(det.get("accept_reason") or "")
            accepted_reasons[reason] = accepted_reasons.get(reason, 0) + 1
            if det.get("accepted"):
                n_accepted += 1
        stats["accepted_cargo_count"] = int(n_accepted)
        stats["accept_reasons"] = accepted_reasons
        stats["border_cargo_unaccepted"] = int(n_border)
        stats["workspace_predicate"] = bool(
            self.workspace_ctx is not None and self.workspace_ctx.available())
        label_map, instance_map, n_cleared = restrict_cargo_mask_to_accepted(
            label_map, detections, instance_map=instance_map)
        stats["unaccepted_cargo_pixels_cleared"] = int(n_cleared)
        # Self-body first, then the vote. A panel flank scored as cargo on
        # every frame made the window think it always saw cargo, so a real
        # miss never reached the majority test.
        t_temporal = self._time.perf_counter() if t_after_seg is not None else None
        if self.temporal_gate is not None:
            label_map, detections, tstats = self.temporal_gate.apply(
                label_map, detections, rgb_uint8)
            stats["temporal"] = dict(tstats)
            stats["held"] = bool(tstats.get("held"))
            if tstats.get("held") and n_self:
                # A held bbox may reach into the panel; never hand the arm
                # back to the cargo cloud.
                label_map = np.array(label_map, copy=True)
                label_map[body] = LABEL_ROBOT_ARM
        t_counts = self._time.perf_counter() if t_after_seg is not None else None
        stats["detections"] = compact_detections(detections)
        stats["label_counts"] = {
            int(label): int((label_map == label).sum())
            for label in DEFAULT_LABEL_NAMES
        }
        stats["detection_count"] = len(detections)
        if n_self:
            stats["self_body_pixels"] = n_self
        generation = int(generation or 0)
        instance_id = str(instance_id or "")
        stats["generation"] = generation
        stats["instance_id"] = instance_id
        if t_after_seg is not None:
            now = self._time.perf_counter()
            stage = dict(stats.get("stage_ms") or {})
            stage["before_temporal"] = round(
                (t_temporal - t_after_seg) * 1000.0, 2)
            stage["temporal"] = round((t_counts - t_temporal) * 1000.0, 2)
            stage["label_counts_and_copy"] = round((now - t_counts) * 1000.0, 2)
            stats["stage_ms"] = stage
        self._last_stats = dict(stats)
        self._output = SegmenterOutput(
            stamp=float(stamp),
            frame_id=str(frame_id),
            label_map=np.copy(label_map),
            detections=tuple(_copy_detection(d) for d in detections),
            instance_map=(None if instance_map is None
                          else np.copy(instance_map)),
            stats=stats,
            generation=generation,
            instance_id=instance_id,
        )

    def copy_output(self):
        """The only external read path; returns a copy, or None before the
        first ``update``."""
        if self._output is None:
            return None
        return self._output.copy()

    def segment(self, rgb_image):
        """Return (label_map, detections).

        ``rgb_image`` is an HxWx3 uint8 numpy array (RGB order).
        ``label_map`` is an HxW uint8 array of label ids.
        ``detections`` is a list of dicts: {label, prompt, confidence, bbox}.
        Backends that produce instance masks also populate ``self._instance_map``
        (HxW uint16, 0 = background, 1..N = instance IDs) and add
        ``instance_id`` and ``mask`` keys to each detection dict.
        """
        raise NotImplementedError


class StubSegmenter(SemanticSegmenter):
    """Always-background segmenter used when ML deps are unavailable.

    Lets the ROS node boot and publish a valid (all-zero) mask so downstream
    consumers stay alive. The point-filter node treats all pixels as
    background, which is the safe default — nothing reaches the cargo occ
    map until a real segmenter is wired in.
    """

    def __init__(self, prompts, class_mapping=None, confidence_threshold=DEFAULT_YOLO_CONFIDENCE_THRESHOLD):
        super().__init__(prompts, class_mapping, confidence_threshold)
        self._last_stats["backend"] = "stub"

    def segment(self, rgb_image):
        h, w = rgb_image.shape[:2]
        label_map = np.zeros((h, w), dtype=np.uint8)
        counts = {label: 0 for label in DEFAULT_LABEL_NAMES}
        counts[LABEL_BACKGROUND] = int(h * w)
        self._last_stats = {
            "backend": "stub",
            "inference_ms": 0.0,
            "detection_count": 0,
            "label_counts": counts,
        }
        return label_map, []


class BboxFillSegmenter(SemanticSegmenter):
    """YOLO-World bbox-only segmenter (no SAM2).

    Uses ``ultralytics.YOLOWorld`` for open-vocabulary detection, then fills
    each detected bbox with its label id. Cheaper than full SAM2 mask
    refinement and is sufficient when the camera ROI is dominated by the
    target objects (true for the container-interior explore views).
    """

    def __init__(self, prompts, class_mapping=None,
                 confidence_threshold=DEFAULT_YOLO_CONFIDENCE_THRESHOLD,
                 model_name="yolov8s-world.pt", device="cpu"):
        super().__init__(prompts, class_mapping, confidence_threshold)
        import time

        self._time = time
        self._device = str(device)
        self._model_name = str(model_name)
        if self._device.startswith("cuda"):
            import torch
            if not torch.cuda.is_available():
                raise RuntimeError(
                    "YOLO-World device=%s but torch.cuda is unavailable"
                    % self._device)
        clip_vendor = _setup_clip_vendor()
        from ultralytics import YOLOWorld  # noqa: WPS433

        self._model = YOLOWorld(self._model_name)
        to_fn = getattr(self._model, "to", None)
        if callable(to_fn):
            to_fn(self._device)
        self._model.set_classes(list(self.prompts))
        self._last_stats["backend"] = "bbox_fill:%s" % self._model_name
        if clip_vendor:
            self._last_stats["clip_vendor"] = clip_vendor

    def segment(self, rgb_image):
        import numpy as _np

        t0 = self._time.perf_counter()
        results = self._model.predict(
            rgb_image, conf=self.confidence_threshold, device=self._device,
            verbose=False,
        )
        t_predict = self._time.perf_counter()
        inference_ms = (t_predict - t0) * 1000.0

        h, w = rgb_image.shape[:2]
        label_map = _np.zeros((h, w), dtype=_np.uint8)
        detections = []
        counts = {label: 0 for label in DEFAULT_LABEL_NAMES}

        def _finish(sync_ms, fill_ms):
            self._last_stats = {
                "backend": self._last_stats["backend"],
                "inference_ms": inference_ms,
                "detection_count": len(detections),
                "label_counts": counts,
                "stage_ms": {
                    "predict": round(inference_ms, 2),
                    "gpu_sync": round(sync_ms, 2),
                    "mask_fill": round(fill_ms, 2),
                },
            }
            return label_map, detections

        if not results:
            return _finish(0.0, 0.0)

        result = results[0]
        boxes = getattr(result.boxes, "xyxy", None)
        classes = getattr(result.boxes, "cls", None)
        confs = getattr(result.boxes, "conf", None)
        if boxes is None or classes is None or len(boxes) == 0:
            return _finish(0.0, 0.0)

        t_sync = self._time.perf_counter()
        boxes = boxes.cpu().numpy()
        classes = classes.cpu().numpy()
        confs = confs.cpu().numpy() if confs is not None else _np.zeros(len(boxes))
        sync_ms = (self._time.perf_counter() - t_sync) * 1000.0
        t_fill = self._time.perf_counter()

        for idx in range(len(boxes)):
            cls_idx = int(classes[idx])
            if cls_idx < 0 or cls_idx >= len(self.prompts):
                continue
            prompt = self.prompts[cls_idx]
            label_id = self.class_mapping.get(prompt, LABEL_UNKNOWN)
            if robot_arm_below_conf_floor(
                    label_id, confs[idx], self.robot_arm_confidence_threshold):
                continue
            x1, y1, x2, y2 = boxes[idx]
            ix1 = max(0, int(round(x1)))
            iy1 = max(0, int(round(y1)))
            ix2 = min(w, int(round(x2)))
            iy2 = min(h, int(round(y2)))
            if ix2 <= ix1 or iy2 <= iy1:
                continue
            # Write label; later detections overwrite earlier ones on overlap.
            label_map[iy1:iy2, ix1:ix2] = label_id
            detections.append({
                "label": label_id,
                "prompt": prompt,
                "confidence": float(confs[idx]),
                "bbox": [ix1, iy1, ix2, iy2],
            })
            counts[label_id] += int((ix2 - ix1) * (iy2 - iy1))

        counts[LABEL_BACKGROUND] = int((label_map == LABEL_BACKGROUND).sum())
        fill_ms = (self._time.perf_counter() - t_fill) * 1000.0
        return _finish(sync_ms, fill_ms)


class YoloWorldSam2Segmenter(SemanticSegmenter):
    """YOLO-World detection + SAM2 mask refinement.

    Uses YOLO-World for open-vocabulary bbox detection, then refines each
    bbox into a pixel-accurate mask via SAM2's box-prompt interface. Produces
    both a class-level label_map (mono8, backward compatible) and a per-instance
    instance_map (uint16, 0 = background, 1..N per detection).
    """

    def __init__(self, prompts, class_mapping=None,
                 confidence_threshold=DEFAULT_YOLO_CONFIDENCE_THRESHOLD,
                 model_name="yolov8s-world.pt", device="cuda",
                 sam2_checkpoint="facebook/sam2-hiera-small",
                 sam2_model_type="sam2_hiera_s"):
        super().__init__(prompts, class_mapping, confidence_threshold)
        import time
        self._time = time
        self._device = str(device)
        self._model_name = str(model_name)
        clip_vendor = _setup_clip_vendor()

        from ultralytics import YOLOWorld  # noqa: WPS433
        self._yolo = YOLOWorld(self._model_name)
        self._yolo.set_classes(list(self.prompts))

        import torch
        from sam2.build_sam import build_sam2
        from sam2.sam2_image_predictor import SAM2ImagePredictor

        sam2_model = build_sam2(
            sam2_model_type,
            sam2_checkpoint,
            device=self._device,
        )
        self._sam2_predictor = SAM2ImagePredictor(sam2_model)
        self._torch = torch
        self._last_stats["backend"] = "yolo_world_sam2:%s" % self._model_name
        if clip_vendor:
            self._last_stats["clip_vendor"] = clip_vendor

    def segment(self, rgb_image):
        import numpy as _np

        t0 = self._time.time()
        results = self._yolo.predict(
            rgb_image, conf=self.confidence_threshold, device=self._device,
            verbose=False,
        )
        yolo_ms = (self._time.time() - t0) * 1000.0

        h, w = rgb_image.shape[:2]
        label_map = _np.zeros((h, w), dtype=_np.uint8)
        instance_map = _np.zeros((h, w), dtype=_np.uint16)
        detections = []
        counts = {label: 0 for label in DEFAULT_LABEL_NAMES}

        if not results:
            self._finish_stats(yolo_ms, 0, counts, label_map, instance_map)
            return label_map, detections

        result = results[0]
        boxes = getattr(result.boxes, "xyxy", None)
        classes = getattr(result.boxes, "cls", None)
        confs = getattr(result.boxes, "conf", None)
        if boxes is None or classes is None or len(boxes) == 0:
            self._finish_stats(yolo_ms, 0, counts, label_map, instance_map)
            return label_map, detections

        boxes_np = boxes.cpu().numpy()
        classes_np = classes.cpu().numpy()
        confs_np = confs.cpu().numpy() if confs is not None else _np.zeros(len(boxes_np))

        # Sort by confidence descending so higher-confidence masks take priority.
        order = _np.argsort(-confs_np)

        t1 = self._time.time()
        self._sam2_predictor.set_image(rgb_image)
        instance_id = 0

        for rank_idx in order:
            cls_idx = int(classes_np[rank_idx])
            if cls_idx < 0 or cls_idx >= len(self.prompts):
                continue
            prompt = self.prompts[cls_idx]
            label_id = self.class_mapping.get(prompt, LABEL_UNKNOWN)
            if robot_arm_below_conf_floor(
                    label_id, confs_np[rank_idx],
                    self.robot_arm_confidence_threshold):
                continue
            box = boxes_np[rank_idx]

            masks, scores, _logits = self._sam2_predictor.predict(
                box=box[None, :],
                multimask_output=False,
            )
            # masks: (1, H, W) bool
            mask_2d = masks[0].astype(bool)

            instance_id += 1
            # Higher-confidence detections were painted first; do NOT overwrite.
            fresh = mask_2d & (instance_map == 0)
            label_map[fresh] = label_id
            instance_map[fresh] = instance_id
            pixel_count = int(fresh.sum())
            counts[label_id] = counts.get(label_id, 0) + pixel_count

            ix1, iy1, ix2, iy2 = (
                max(0, int(round(box[0]))),
                max(0, int(round(box[1]))),
                min(w, int(round(box[2]))),
                min(h, int(round(box[3]))),
            )
            detections.append({
                "label": label_id,
                "prompt": prompt,
                "confidence": float(confs_np[rank_idx]),
                "bbox": [ix1, iy1, ix2, iy2],
                "instance_id": instance_id,
                "mask": mask_2d,
            })

        sam2_ms = (self._time.time() - t1) * 1000.0
        total_ms = yolo_ms + sam2_ms
        counts[LABEL_BACKGROUND] = int((label_map == LABEL_BACKGROUND).sum())
        self._finish_stats(total_ms, len(detections), counts, label_map, instance_map)
        return label_map, detections

    def _finish_stats(self, inference_ms, det_count, counts, label_map, instance_map):
        self._instance_map = instance_map
        self._last_stats = {
            "backend": self._last_stats["backend"],
            "inference_ms": inference_ms,
            "detection_count": det_count,
            "label_counts": counts,
        }


_BACKENDS = {
    "stub": StubSegmenter,
    "bbox_fill": BboxFillSegmenter,
    "yolo_world": BboxFillSegmenter,
    "yolo_world_sam2": YoloWorldSam2Segmenter,
}

# Launch/eval still pass require_backend:=yolo_world. The working
# implementation is BboxFillSegmenter, which reports bbox_fill:<weights>.
_BACKEND_ALIASES = {
    "bbox_fill": "bbox_fill",
    "yolo_world": "bbox_fill",
}


def backend_kind(backend):
    """Prefix of stats['backend'], e.g. bbox_fill:weights -> bbox_fill."""
    return str(backend or "").split(":")[0].split("(")[0].strip()


def backend_matches_require(backend, require):
    """Whether a live backend string satisfies require_backend.

    Empty require disables the guard. Stub fallbacks never match. yolo_world
    and bbox_fill are the same YOLO-World box-fill backend; yolo_world_sam2
    is a different backend and does not satisfy yolo_world.
    """
    want = str(require or "").strip()
    if not want:
        return True
    kind = backend_kind(backend)
    if not kind or kind.startswith("stub"):
        return False
    return _BACKEND_ALIASES.get(kind, kind) == _BACKEND_ALIASES.get(want, want)


def build_segmenter(config):
    """Construct a segmenter from a config dict.

    Expected keys:
        backend: "stub" | "bbox_fill" | "yolo_world" | "yolo_world_sam2"
        prompts: list[str]
        class_mapping: dict[str, int]   (optional)
        confidence_threshold: float     (optional, default 0.2)
        robot_arm_confidence_threshold: float (optional, default 0.5)
        cargo_min_confidence: float     (optional, default 0.2)
        model_name: str                 (optional, YOLO-World checkpoint)
        device: str                     (optional, "cpu" | "cuda:0")
        sam2_checkpoint: str            (optional, SAM2 model checkpoint)
        sam2_model_type: str            (optional, SAM2 model config name)
        self_body_row_start_frac: float (optional, 0 disables)
        temporal_window_frames: int     (optional, 5; <=1 disables hold)
        temporal_min_positive_ratio: float (optional, 0.5)
        temporal_scene_change_mad: float   (optional, 10)
        temporal_bbox_iou_reset: float     (optional, 0.3)
    """
    backend = str(config.get("backend", "stub"))
    prompts = list(config.get("prompts", []))
    class_mapping = config.get("class_mapping")
    conf = float(config.get(
        "confidence_threshold", DEFAULT_YOLO_CONFIDENCE_THRESHOLD))
    model_name = str(config.get("model_name", "yolov8s-world.pt"))
    device = str(config.get("device", "cpu"))
    self_body_frac = float(config.get("self_body_row_start_frac", 0.0))

    def _finish(segmenter):
        segmenter.self_body_row_start_frac = self_body_frac
        segmenter.robot_arm_confidence_threshold = float(
            config.get("robot_arm_confidence_threshold",
                       DEFAULT_ROBOT_ARM_CONFIDENCE_THRESHOLD))
        segmenter.cargo_min_confidence = float(
            config.get("cargo_min_confidence",
                       DEFAULT_CARGO_MIN_CONFIDENCE))
        window = int(config.get("temporal_window_frames", 5) or 0)
        if window > 1:
            from luggage_perception.detection_temporal_gate import (
                DetectionTemporalGate)
            segmenter.temporal_gate = DetectionTemporalGate(
                window_size=window,
                min_positive_ratio=float(
                    config.get("temporal_min_positive_ratio", 0.5)),
                scene_change_mad=float(
                    config.get("temporal_scene_change_mad", 10.0)),
                bbox_iou_reset=float(
                    config.get("temporal_bbox_iou_reset", 0.3)),
            )
        return segmenter

    sam2_cfg = config.get("sam2", {}) if isinstance(config.get("sam2"), dict) else {}
    sam2_checkpoint = str(sam2_cfg.get("checkpoint", config.get(
        "sam2_checkpoint", "facebook/sam2-hiera-small")))
    sam2_model_type = str(sam2_cfg.get("model_type", config.get(
        "sam2_model_type", "sam2_hiera_s")))

    if backend == "stub":
        return _finish(StubSegmenter(prompts, class_mapping, conf))

    try:
        cls = _BACKENDS[backend]
    except KeyError:
        raise ValueError("unknown semantic backend: %r" % backend)

    try:
        kwargs = {"model_name": model_name, "device": device}
        if cls is YoloWorldSam2Segmenter:
            kwargs["sam2_checkpoint"] = sam2_checkpoint
            kwargs["sam2_model_type"] = sam2_model_type
        return _finish(cls(prompts, class_mapping, conf, **kwargs))
    except ImportError as exc:
        stub = StubSegmenter(prompts, class_mapping, conf)
        stub._last_stats["backend"] = "stub(fallback:%s:%s)" % (backend, exc.name)
        return _finish(stub)
    except Exception as exc:  # noqa: BLE001
        stub = StubSegmenter(prompts, class_mapping, conf)
        stub._last_stats["backend"] = "stub(fallback:%s:%s)" % (backend, type(exc).__name__)
        return _finish(stub)


def colorize_label_map(label_map):
    """Render a label map as an HxWx3 uint8 BGR array for RViz / debug."""
    h, w = label_map.shape[:2]
    out = np.zeros((h, w, 3), dtype=np.uint8)
    for label_id, color in LABEL_PALETTE_BGR.items():
        mask = label_map == label_id
        out[mask] = color
    return out


def draw_detections_overlay(rgb_image, detections, label_names=None):
    """Draw detection bboxes/masks/labels on an RGB image for RViz / debug.

    Returns an HxWx3 uint8 BGR image (suitable for ``cv2_to_imgmsg(bgr8)``).
    ``detections`` is the list of dicts produced by
    ``SemanticSegmenter.segment``: each has ``label``, ``prompt``,
    ``confidence``, ``bbox`` (xyxy pixel coords), and optionally ``mask``
    (HxW bool, from the SAM2 backend). cv2 is imported lazily so this module
    still imports cleanly when OpenCV is absent - the ROS node only calls this
    when ``publish_overlay`` is true, and cv_bridge already pulls in OpenCV.
    """
    import cv2  # noqa: WPS433  lazy: keep core import-clean for unit tests

    if label_names is None:
        label_names = DEFAULT_LABEL_NAMES
    bgr = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
    if not detections:
        return bgr

    for det in detections:
        label_id = int(det["label"])
        color = LABEL_PALETTE_BGR.get(label_id, (0, 255, 255))
        prompt = str(det.get("prompt", "?"))
        conf = float(det.get("confidence", 0.0))
        x1, y1, x2, y2 = (int(v) for v in det["bbox"])

        mask = det.get("mask")
        if mask is not None:
            m = np.asarray(mask).astype(bool)
            if m.any():
                # Semi-transparent fill: blend the colored mask region with the
                # underlying image. addWeighted over the full frame is cheap
                # enough for debug viz and leaves non-mask pixels unchanged.
                blend = bgr.copy()
                blend[m] = color
                bgr = cv2.addWeighted(blend, 0.35, bgr, 0.65, 0)
                contours, _ = cv2.findContours(
                    m.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(bgr, contours, -1, color, 2)

        cv2.rectangle(bgr, (x1, y1), (x2, y2), color, 2)

        label_name = label_names.get(label_id, str(label_id))
        text = "%s/%s %.2f" % (label_name, prompt, conf)
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        ty = max(y1 - 4, th + 2)
        cv2.rectangle(bgr, (x1, ty - th - 2), (x1 + tw + 4, ty + 2), color, -1)
        cv2.putText(bgr, text, (x1 + 2, ty),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    return bgr
