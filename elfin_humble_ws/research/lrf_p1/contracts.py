"""Inference contracts: hardware-observable inputs and fail-closed reasons."""

from __future__ import division

FORBIDDEN_INFERENCE_KEYS = frozenset((
    "gazebo", "gazebo_state", "spawned", "spawned_identity", "mesh_id",
    "visual_id", "stl_path", "stl_sha256", "hidden", "hidden_mesh",
    "hidden_surface", "future", "future_frames", "label", "labels",
    "gt", "ground_truth", "eval_gt", "visible_surface_ratio",
    "occlusion_bucket", "visible_ratio", "complete_mesh", "catalog_id",
    "size_tier", "instance_id",
))

REASON_OK = "ok"
REASON_EMPTY = "DETECT_NO_CLOUD"
REASON_NONFINITE = "DETECT_NONFINITE"
REASON_TOO_FEW = "DETECT_TOO_FEW_POINTS"
REASON_MALFORMED = "DETECT_MALFORMED"
REASON_OOD = "DETECT_OOD"
REASON_BASELINE_FAILED = "DETECT_BASELINE_FAILED"
REASON_UNAVAILABLE = "MODEL_UNAVAILABLE"

PRIVILEGED_SUBSTRINGS = (
    "gazebo", "spawn", "hidden", "future", "label", "ground_truth",
    "visible_surface", "occlusion_bucket", "complete_mesh", "stl_sha",
)


def walk_forbidden(obj, path=""):
    """Yield dotted paths of forbidden keys. Used by leakage tests."""
    if isinstance(obj, dict):
        for key, value in obj.items():
            name = str(key)
            lower = name.lower()
            here = "%s.%s" % (path, name) if path else name
            if name in FORBIDDEN_INFERENCE_KEYS or any(
                    token in lower for token in PRIVILEGED_SUBSTRINGS):
                yield here
            for item in walk_forbidden(value, here):
                yield item
    elif isinstance(obj, (list, tuple)):
        for idx, value in enumerate(obj):
            here = "%s[%d]" % (path, idx)
            for item in walk_forbidden(value, here):
                yield item


def assert_inference_clean(observation):
    hits = list(walk_forbidden(observation))
    if hits:
        raise ValueError("privileged inference fields: %s" % ", ".join(hits))
