"""Trainable residual box completer with an ensemble uncertainty wrapper.

Selected method for the runnable spike: ridge residual ensemble on
observation-only features. See METHOD.md for the paper audit and why
pretrained PoinTr/AdaPoinTr weights are not the executed candidate.
"""

from __future__ import division

import math
import time

import numpy as np
from sklearn.linear_model import Ridge

from research.lrf_p1.baseline_adapter import estimate_from_observation
from research.lrf_p1.contracts import (
    REASON_EMPTY,
    REASON_MALFORMED,
    REASON_NONFINITE,
    REASON_OK,
    REASON_TOO_FEW,
    REASON_UNAVAILABLE,
    assert_inference_clean,
)
from research.lrf_p1.features import cloud_features

TARGETS = ("width", "depth", "height", "cx", "cy", "cz", "yaw")
K_EXPAND = 1.64485  # nominal 90% normal interval


class ResidualEnsemble(object):
    def __init__(self, n_models=5, alpha=1.0, seeds=(1, 2, 3, 4, 5)):
        self.n_models = int(n_models)
        self.alpha = float(alpha)
        self.seeds = tuple(int(s) for s in seeds[: self.n_models])
        self.models = None
        self.available = False

    def fit(self, features, targets):
        x = np.asarray(features, dtype=np.float64)
        y = np.asarray(targets, dtype=np.float64)
        if len(x) < 8 or x.shape != (len(x),) + (x.shape[1],) or y.ndim != 2:
            self.models = None
            self.available = False
            return
        models = []
        n = len(x)
        for seed in self.seeds:
            rng = np.random.RandomState(seed)
            idx = rng.randint(0, n, size=n)
            est = Ridge(alpha=self.alpha)
            est.fit(x[idx], y[idx])
            models.append(est)
        self.models = models
        self.available = True

    def predict_mean_std(self, feature):
        if not self.available or not self.models:
            return None, None
        feat = np.asarray(feature, dtype=np.float64).reshape(1, -1)
        preds = np.stack([m.predict(feat)[0] for m in self.models], axis=0)
        return preds.mean(axis=0), preds.std(axis=0)


def _box_vector(box):
    c = box["center_xyz"]
    return np.array([
        box["width"], box["depth"], box["height"],
        c[0], c[1], c[2], box["yaw"],
    ], dtype=np.float64)


def wrap_yaw(value):
    return math.atan2(math.sin(value), math.cos(value))


def yaw_delta(pred, gt):
    """Smallest signed yaw error, folding the 180-deg rectangle ambiguity."""
    delta = math.atan2(math.sin(gt - pred), math.cos(gt - pred))
    if abs(delta) > 0.5 * math.pi:
        delta -= math.copysign(math.pi, delta)
    return float(delta)


def apply_residual(base_vec, mean, lock_pose=False):
    """Add residual to a baseline box vector. lock_pose keeps XY/yaw; height grows up."""
    mean = np.asarray(mean, dtype=np.float64).copy()
    base_vec = np.asarray(base_vec, dtype=np.float64)
    if lock_pose:
        dh = float(mean[2])
        mean[3] = 0.0
        mean[4] = 0.0
        mean[5] = 0.5 * dh
        mean[6] = 0.0
    vec = base_vec + mean
    vec[6] = wrap_yaw(float(vec[6]))
    vec[0] = max(0.05, float(vec[0]))
    vec[1] = max(0.05, float(vec[1]))
    vec[2] = max(0.05, float(vec[2]))
    return vec


# Max residual applied on top of a successful baseline (metres / radians).
RESIDUAL_CLIP = np.array([0.06, 0.06, 0.04, 0.04, 0.04, 0.03, 0.20])


def infer(observation, ensemble, min_points=40, residual_clip=None, lock_pose=False):
    """Learned residual completion. Fail closed when the model is missing."""
    t0 = time.perf_counter()
    assert_inference_clean(observation)
    if (not lock_pose) and (ensemble is None or not getattr(ensemble, "available", False)):
        return {
            "ok": False,
            "reason": REASON_UNAVAILABLE,
            "observed": None,
            "inferred": None,
            "unknown": True,
            "box": None,
            "std": None,
            "latency_ms": (time.perf_counter() - t0) * 1000.0,
        }
    base = estimate_from_observation(observation, min_points=min_points)
    if lock_pose and base.get("ok") and base.get("box") is not None:
        box = base["box"]
        return {
            "ok": True,
            "reason": REASON_OK,
            "observed": box,
            "inferred": box,
            "unknown": [],
            "box": box,
            "std": [0.0] * len(TARGETS),
            "latency_ms": (time.perf_counter() - t0) * 1000.0,
            "source": "baseline_locked",
        }
    if not base["ok"] and base["reason"] in (
            REASON_EMPTY, REASON_MALFORMED, REASON_NONFINITE, REASON_TOO_FEW):
        return {
            "ok": False,
            "reason": base["reason"],
            "observed": None,
            "inferred": None,
            "unknown": True,
            "box": None,
            "std": None,
            "latency_ms": (time.perf_counter() - t0) * 1000.0,
        }
    feat = cloud_features(observation, base.get("box"))
    mean, std = ensemble.predict_mean_std(feat)
    latency = (time.perf_counter() - t0) * 1000.0
    if mean is None:
        return {
            "ok": False,
            "reason": REASON_UNAVAILABLE,
            "observed": base.get("box"),
            "inferred": None,
            "unknown": True,
            "box": None,
            "std": None,
            "latency_ms": latency,
        }
    if not np.isfinite(mean).all() or not np.isfinite(std).all():
        return {
            "ok": False,
            "reason": REASON_NONFINITE,
            "observed": base.get("box"),
            "inferred": None,
            "unknown": True,
            "box": None,
            "std": None,
            "latency_ms": latency,
        }
    if residual_clip is not None:
        clip = np.asarray(residual_clip, dtype=np.float64)
        if clip.shape == mean.shape:
            mean = np.clip(mean, -clip, clip)
    if base["ok"]:
        vec = apply_residual(_box_vector(base["box"]), mean, lock_pose=False)
    else:
        roi = observation.get("roi_center_xy") or (0.0, 0.0)
        vec = mean.copy()
        vec[3] += float(roi[0])
        vec[4] += float(roi[1])
        vec[0] = max(0.05, float(vec[0]))
        vec[1] = max(0.05, float(vec[1]))
        vec[2] = max(0.05, float(vec[2]))
        vec[6] = wrap_yaw(float(vec[6]))
    inferred = {
        "width": float(vec[0]),
        "depth": float(vec[1]),
        "height": float(vec[2]),
        "center_xyz": [float(vec[3]), float(vec[4]), float(vec[5])],
        "yaw": float(vec[6]),
    }
    unknown_dims = [name for name, s in zip(TARGETS, std) if float(s) > 0.08]
    return {
        "ok": True,
        "reason": REASON_OK if base["ok"] else base["reason"],
        "observed": base.get("box"),
        "inferred": inferred,
        "unknown": unknown_dims,
        "box": inferred,
        "std": [float(v) for v in std],
        "latency_ms": latency,
        "source": "inferred",
    }


def expanded_box(box, std, k=K_EXPAND):
    c = list(box["center_xyz"])
    return {
        "width": box["width"] + 2.0 * k * float(std[0]),
        "depth": box["depth"] + 2.0 * k * float(std[1]),
        "height": box["height"] + 2.0 * k * float(std[2]),
        "center_xyz": [
            c[0],
            c[1],
            c[2],
        ],
        "yaw": box["yaw"],
        "std_cx": k * float(std[3]),
        "std_cy": k * float(std[4]),
        "std_cz": k * float(std[5]),
    }


def contains_gt(expanded, gt):
    dx = abs(expanded["center_xyz"][0] - gt["center_xyz"][0])
    dy = abs(expanded["center_xyz"][1] - gt["center_xyz"][1])
    dz = abs(expanded["center_xyz"][2] - gt["center_xyz"][2])
    return (
        expanded["width"] + 1e-9 >= gt["width"]
        and expanded["depth"] + 1e-9 >= gt["depth"]
        and expanded["height"] + 1e-9 >= gt["height"]
        and dx <= 0.5 * (expanded["width"] - gt["width"]) + expanded.get("std_cx", 0.0)
        and dy <= 0.5 * (expanded["depth"] - gt["depth"]) + expanded.get("std_cy", 0.0)
        and dz <= 0.5 * (expanded["height"] - gt["height"]) + expanded.get("std_cz", 0.0)
    )


# Re-export reason codes used by OOD tests.
REASON_EMPTY = REASON_EMPTY
REASON_MALFORMED = REASON_MALFORMED
REASON_NONFINITE = REASON_NONFINITE
REASON_TOO_FEW = REASON_TOO_FEW
