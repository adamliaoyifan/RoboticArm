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

    def copy(self):
        return SegmenterOutput(
            stamp=self.stamp,
            frame_id=self.frame_id,
            label_map=np.copy(self.label_map),
            detections=tuple(_copy_detection(d) for d in self.detections),
            instance_map=(None if self.instance_map is None
                          else np.copy(self.instance_map)),
            stats=dict(self.stats),
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
        out.append(item)
    return out


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

    def __init__(self, prompts, class_mapping=None, confidence_threshold=0.25):
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

    def update(self, rgb_uint8, stamp, frame_id):
        """Run ``segment()`` and store a deep-copied snapshot.

        The backends may reuse internal buffers between calls, so every array
        is copied here; a previously returned ``SegmenterOutput`` is never
        mutated by a later ``update``. No publishing, no I/O. ``stamp`` /
        ``frame_id`` must come from the input header, not from wall clock.
        """
        label_map, detections = self.segment(rgb_uint8)
        n_before = cargo_detection_count(detections)
        body = combined_self_body_mask(
            self.self_body_mask, self.self_body_row_start_frac,
            label_map.shape[:2])
        label_map, detections, instance_map, n_self = apply_self_body_mask(
            label_map, detections, body, instance_map=self._instance_map)
        n_after = cargo_detection_count(detections)
        stats = dict(self._last_stats)
        stats["n_yolo_cargo_before_self_body"] = int(n_before)
        stats["n_dropped_self_body"] = int(max(0, n_before - n_after))
        stats["raw_cargo"] = bool(n_after > 0)
        stats["held"] = False
        # Self-body first, then the vote. A panel flank scored as cargo on
        # every frame made the window think it always saw cargo, so a real
        # miss never reached the majority test.
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
        stats["detections"] = compact_detections(detections)
        stats["label_counts"] = {
            int(label): int((label_map == label).sum())
            for label in DEFAULT_LABEL_NAMES
        }
        stats["detection_count"] = len(detections)
        if n_self:
            stats["self_body_pixels"] = n_self
        self._last_stats = dict(stats)
        self._output = SegmenterOutput(
            stamp=float(stamp),
            frame_id=str(frame_id),
            label_map=np.copy(label_map),
            detections=tuple(_copy_detection(d) for d in detections),
            instance_map=(None if instance_map is None
                          else np.copy(instance_map)),
            stats=stats,
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

    def __init__(self, prompts, class_mapping=None, confidence_threshold=0.25):
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

    def __init__(self, prompts, class_mapping=None, confidence_threshold=0.25,
                 model_name="yolov8s-world.pt", device="cpu"):
        super().__init__(prompts, class_mapping, confidence_threshold)
        import time

        self._time = time
        self._device = str(device)
        self._model_name = str(model_name)
        clip_vendor = _setup_clip_vendor()
        from ultralytics import YOLOWorld  # noqa: WPS433

        self._model = YOLOWorld(self._model_name)
        self._model.set_classes(list(self.prompts))
        self._last_stats["backend"] = "bbox_fill:%s" % self._model_name
        if clip_vendor:
            self._last_stats["clip_vendor"] = clip_vendor

    def segment(self, rgb_image):
        import numpy as _np

        t0 = self._time.time()
        results = self._model.predict(
            rgb_image, conf=self.confidence_threshold, device=self._device,
            verbose=False,
        )
        inference_ms = (self._time.time() - t0) * 1000.0

        h, w = rgb_image.shape[:2]
        label_map = _np.zeros((h, w), dtype=_np.uint8)
        detections = []
        counts = {label: 0 for label in DEFAULT_LABEL_NAMES}

        if not results:
            self._last_stats = {
                "backend": self._last_stats["backend"],
                "inference_ms": inference_ms,
                "detection_count": 0,
                "label_counts": counts,
            }
            return label_map, detections

        result = results[0]
        boxes = getattr(result.boxes, "xyxy", None)
        classes = getattr(result.boxes, "cls", None)
        confs = getattr(result.boxes, "conf", None)
        if boxes is None or classes is None or len(boxes) == 0:
            self._last_stats = {
                "backend": self._last_stats["backend"],
                "inference_ms": inference_ms,
                "detection_count": 0,
                "label_counts": counts,
            }
            return label_map, detections

        boxes = boxes.cpu().numpy()
        classes = classes.cpu().numpy()
        confs = confs.cpu().numpy() if confs is not None else _np.zeros(len(boxes))

        for idx in range(len(boxes)):
            cls_idx = int(classes[idx])
            if cls_idx < 0 or cls_idx >= len(self.prompts):
                continue
            prompt = self.prompts[cls_idx]
            label_id = self.class_mapping.get(prompt, LABEL_UNKNOWN)
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
        self._last_stats = {
            "backend": self._last_stats["backend"],
            "inference_ms": inference_ms,
            "detection_count": len(detections),
            "label_counts": counts,
        }
        return label_map, detections


class YoloWorldSam2Segmenter(SemanticSegmenter):
    """YOLO-World detection + SAM2 mask refinement.

    Uses YOLO-World for open-vocabulary bbox detection, then refines each
    bbox into a pixel-accurate mask via SAM2's box-prompt interface. Produces
    both a class-level label_map (mono8, backward compatible) and a per-instance
    instance_map (uint16, 0 = background, 1..N per detection).
    """

    def __init__(self, prompts, class_mapping=None, confidence_threshold=0.25,
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


def build_segmenter(config):
    """Construct a segmenter from a config dict.

    Expected keys:
        backend: "stub" | "bbox_fill" | "yolo_world" | "yolo_world_sam2"
        prompts: list[str]
        class_mapping: dict[str, int]   (optional)
        confidence_threshold: float     (optional, default 0.25)
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
    conf = float(config.get("confidence_threshold", 0.25))
    model_name = str(config.get("model_name", "yolov8s-world.pt"))
    device = str(config.get("device", "cpu"))
    self_body_frac = float(config.get("self_body_row_start_frac", 0.0))

    def _finish(segmenter):
        segmenter.self_body_row_start_frac = self_body_frac
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
