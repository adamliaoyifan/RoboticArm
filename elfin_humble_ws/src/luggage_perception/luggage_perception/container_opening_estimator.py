#!/usr/bin/env python3
"""Pure Python hybrid fiducial + RGB-D container opening estimator.

The module deliberately has no ROS or OpenCV imports.  Inputs and outputs are
small dataclasses containing NumPy arrays, which keeps the geometry testable and
lets ROS (or another middleware) provide thin message adapters.
"""

from __future__ import division

from dataclasses import dataclass, field
import math

import numpy as np


SOURCE_TAG_DEPTH = "tag_depth"
SOURCE_TAG_ONLY = "tag_only"
SOURCE_DEPTH_ONLY = "depth_only"
SOURCE_PRIOR = "prior"


def _vec3(value, name):
    value = np.asarray(value, dtype=float).reshape(-1)
    if value.size != 3 or not np.all(np.isfinite(value)):
        raise ValueError("%s must contain three finite values" % name)
    return value


def _unit(value, name):
    value = _vec3(value, name)
    norm = float(np.linalg.norm(value))
    if norm < 1e-9:
        raise ValueError("%s must be non-zero" % name)
    return value / norm


def _orthogonal_axes(normal, width_axis=None):
    normal = _unit(normal, "normal")
    if width_axis is not None:
        width = _vec3(width_axis, "width_axis")
        width = width - normal * float(np.dot(width, normal))
        if np.linalg.norm(width) > 1e-8:
            width /= np.linalg.norm(width)
            return width, np.cross(normal, width)
    seed = np.array([1.0, 0.0, 0.0])
    if abs(float(np.dot(seed, normal))) > 0.85:
        seed = np.array([0.0, 1.0, 0.0])
    width = seed - normal * float(np.dot(seed, normal))
    width /= np.linalg.norm(width)
    return width, np.cross(normal, width)


@dataclass
class OpeningPrior:
    """Coarse opening pose and rectangular aperture.

    ``source`` is ``"tag"`` for a live fiducial observation and ``"prior"``
    for a configured/static fallback.  ``width_axis`` lies in the opening
    plane; the height axis is derived as ``normal x width_axis``.
    """

    center: object
    normal: object
    width_axis: object
    width: float
    height: float
    stamp: float = 0.0
    confidence: float = 0.65
    source: str = "tag"

    def __post_init__(self):
        self.center = _vec3(self.center, "center")
        self.normal = _unit(self.normal, "normal")
        self.width_axis, _ = _orthogonal_axes(self.normal, self.width_axis)
        self.width = float(self.width)
        self.height = float(self.height)
        self.stamp = float(self.stamp)
        self.confidence = float(np.clip(self.confidence, 0.0, 1.0))
        if self.width <= 0.0 or self.height <= 0.0:
            raise ValueError("prior aperture dimensions must be positive")
        if self.source not in ("tag", "prior"):
            raise ValueError("prior source must be 'tag' or 'prior'")


@dataclass
class EstimatorConfig:
    plane_distance_threshold: float = 0.012
    prior_plane_window: float = 0.12
    prior_aperture_margin: float = 0.20
    ransac_iterations: int = 160
    min_depth_points: int = 35
    max_depth_points: int = 6000
    aperture_quantile: float = 0.03
    min_aperture_extent: float = 0.05
    max_normal_deviation_deg: float = 35.0
    max_age_sec: float = 0.75
    min_confidence: float = 0.45
    covariance_floor_position: float = 0.002
    covariance_floor_angle_deg: float = 0.5
    random_seed: int = 17

    @classmethod
    def from_dict(cls, values):
        values = values or {}
        known = {item.name for item in cls.__dataclass_fields__.values()}
        return cls(**{key: values[key] for key in values if key in known})


@dataclass
class OpeningEstimate:
    center: np.ndarray
    normal: np.ndarray
    width_axis: np.ndarray
    width: float
    height: float
    source_status: str
    confidence: float
    pose_covariance: np.ndarray
    aperture_covariance: np.ndarray
    stamp: float
    age_sec: float
    fresh: bool
    accepted: bool
    rejection_reason: str = ""
    diagnostics: dict = field(default_factory=dict)

    @property
    def height_axis(self):
        return np.cross(self.normal, self.width_axis)

    def as_dict(self):
        return {
            "center": self.center.tolist(),
            "normal": self.normal.tolist(),
            "width_axis": self.width_axis.tolist(),
            "height_axis": self.height_axis.tolist(),
            "width": float(self.width),
            "height": float(self.height),
            "source_status": self.source_status,
            "confidence": float(self.confidence),
            "pose_covariance": self.pose_covariance.reshape(-1).tolist(),
            "aperture_covariance": self.aperture_covariance.reshape(-1).tolist(),
            "stamp": float(self.stamp),
            "age_sec": float(self.age_sec),
            "fresh": bool(self.fresh),
            "accepted": bool(self.accepted),
            "rejection_reason": self.rejection_reason,
            "diagnostics": dict(self.diagnostics),
        }


class ContainerOpeningEstimator:
    """Robustly fit an opening plane and rectangular aperture."""

    def __init__(self, config=None):
        self.config = (
            config if isinstance(config, EstimatorConfig)
            else EstimatorConfig.from_dict(config)
        )

    def estimate(self, prior=None, depth_points=None, depth_stamp=None,
                 now=None, hardware_strict=False, sensor_origin=None):
        """Estimate the opening and apply freshness/safety policy.

        ``depth_points`` must be Nx3 in the same frame as ``prior``.
        ``sensor_origin`` is optional and only resolves the normal sign when
        operating without a tag.
        """
        now = float(now if now is not None else 0.0)
        points = self._clean_points(depth_points)
        if prior is not None and not isinstance(prior, OpeningPrior):
            prior = OpeningPrior(**prior)
        fit = self._fit_depth(points, prior, sensor_origin)

        if fit is not None:
            center, normal, width_axis, width, height, stats = fit
            status = SOURCE_TAG_DEPTH if (
                prior is not None and prior.source == "tag"
            ) else SOURCE_DEPTH_ONLY
            stamp = float(depth_stamp if depth_stamp is not None else now)
            if status == SOURCE_TAG_DEPTH and prior.stamp > 0.0:
                # Fusion is only as fresh as its oldest required observation.
                stamp = min(stamp, prior.stamp)
            confidence = self._depth_confidence(stats, prior)
            pose_cov, aperture_cov = self._covariance(
                stats, width, height, normal, width_axis
            )
        elif prior is not None:
            center = prior.center.copy()
            normal = prior.normal.copy()
            width_axis = prior.width_axis.copy()
            width, height = prior.width, prior.height
            status = SOURCE_TAG_ONLY if prior.source == "tag" else SOURCE_PRIOR
            stamp = prior.stamp if prior.stamp > 0.0 else now
            confidence = prior.confidence * (
                0.72 if status == SOURCE_TAG_ONLY else 0.40
            )
            pose_cov, aperture_cov = self._prior_covariance(prior, status)
            stats = {
                "input_points": int(points.shape[0]),
                "inlier_points": 0,
                "inlier_ratio": 0.0,
                "plane_rmse": None,
            }
        else:
            return None

        age = max(0.0, now - stamp) if now > 0.0 and stamp > 0.0 else 0.0
        fresh = age <= self.config.max_age_sec
        confidence *= math.exp(-age / max(self.config.max_age_sec, 1e-6))
        rejection = ""
        if hardware_strict and status == SOURCE_PRIOR:
            rejection = "prior_only_disallowed"
        elif hardware_strict and not fresh:
            rejection = "stale_estimate"
        elif hardware_strict and confidence < self.config.min_confidence:
            rejection = "low_confidence"

        diagnostics = {
            key: value for key, value in stats.items()
            if key != "aperture_samples"
        }
        diagnostics.update({
            "hardware_strict": bool(hardware_strict),
            "max_age_sec": float(self.config.max_age_sec),
            "min_confidence": float(self.config.min_confidence),
        })
        return OpeningEstimate(
            center=np.asarray(center), normal=np.asarray(normal),
            width_axis=np.asarray(width_axis), width=float(width),
            height=float(height), source_status=status,
            confidence=float(np.clip(confidence, 0.0, 1.0)),
            pose_covariance=pose_cov, aperture_covariance=aperture_cov,
            stamp=float(stamp), age_sec=float(age), fresh=bool(fresh),
            accepted=not bool(rejection), rejection_reason=rejection,
            diagnostics=diagnostics,
        )

    def _clean_points(self, points):
        if points is None:
            return np.empty((0, 3), dtype=float)
        points = np.asarray(points, dtype=float)
        if points.size == 0:
            return np.empty((0, 3), dtype=float)
        points = points.reshape((-1, 3))
        points = points[np.all(np.isfinite(points), axis=1)]
        if points.shape[0] > self.config.max_depth_points:
            indices = np.linspace(
                0, points.shape[0] - 1, self.config.max_depth_points
            ).astype(int)
            points = points[indices]
        return points

    def _fit_depth(self, points, prior, sensor_origin):
        cfg = self.config
        input_count = points.shape[0]
        if prior is not None and input_count:
            delta = points - prior.center
            height_axis = np.cross(prior.normal, prior.width_axis)
            mask = (
                (np.abs(delta.dot(prior.normal)) <= cfg.prior_plane_window)
                & (np.abs(delta.dot(prior.width_axis))
                   <= prior.width * (0.5 + cfg.prior_aperture_margin))
                & (np.abs(delta.dot(height_axis))
                   <= prior.height * (0.5 + cfg.prior_aperture_margin))
            )
            points = points[mask]
        if points.shape[0] < cfg.min_depth_points:
            return None

        rng = np.random.RandomState(cfg.random_seed)
        best = None
        cos_gate = math.cos(math.radians(cfg.max_normal_deviation_deg))
        for _ in range(max(1, int(cfg.ransac_iterations))):
            sample = points[rng.choice(points.shape[0], 3, replace=False)]
            normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
            norm = np.linalg.norm(normal)
            if norm < 1e-9:
                continue
            normal /= norm
            if prior is not None and abs(float(np.dot(normal, prior.normal))) < cos_gate:
                continue
            distances = np.abs((points - sample[0]).dot(normal))
            inliers = distances <= cfg.plane_distance_threshold
            count = int(np.count_nonzero(inliers))
            if count < cfg.min_depth_points:
                continue
            residual = float(np.median(distances[inliers]))
            score = (count, -residual)
            if best is None or score > best[0]:
                best = (score, inliers)
        if best is None:
            return None

        inlier_points = points[best[1]]
        center = np.median(inlier_points, axis=0)
        centered = inlier_points - center
        _u, _s, vh = np.linalg.svd(centered, full_matrices=False)
        normal = vh[-1]
        if prior is not None:
            if np.dot(normal, prior.normal) < 0.0:
                normal = -normal
        elif sensor_origin is not None:
            toward_sensor = _vec3(sensor_origin, "sensor_origin") - center
            if np.dot(normal, toward_sensor) < 0.0:
                normal = -normal

        distances = np.abs(centered.dot(normal))
        median = float(np.median(distances))
        mad = float(np.median(np.abs(distances - median)))
        robust_limit = max(cfg.plane_distance_threshold, median + 3.5 * 1.4826 * mad)
        inlier_points = inlier_points[distances <= robust_limit]
        if inlier_points.shape[0] < cfg.min_depth_points:
            return None
        center = np.mean(inlier_points, axis=0)

        if prior is not None:
            width_axis, height_axis = _orthogonal_axes(normal, prior.width_axis)
        else:
            projected = inlier_points - center
            covariance = projected.T.dot(projected) / max(1, projected.shape[0] - 1)
            values, vectors = np.linalg.eigh(covariance)
            width_axis = vectors[:, int(np.argmax(values))]
            width_axis -= normal * np.dot(width_axis, normal)
            width_axis /= np.linalg.norm(width_axis)
            height_axis = np.cross(normal, width_axis)

        uv = np.column_stack((
            (inlier_points - center).dot(width_axis),
            (inlier_points - center).dot(height_axis),
        ))
        q = float(np.clip(cfg.aperture_quantile, 0.0, 0.25))
        lower = np.quantile(uv, q, axis=0)
        upper = np.quantile(uv, 1.0 - q, axis=0)
        # Quantiles reject isolated edge outliers. For approximately uniform
        # aperture samples, compensate their predictable (1 - 2q) shrinkage.
        extents = (upper - lower) / max(1.0 - 2.0 * q, 0.5)
        if np.any(extents < cfg.min_aperture_extent):
            return None
        center = center + width_axis * (lower[0] + upper[0]) * 0.5
        center = center + height_axis * (lower[1] + upper[1]) * 0.5

        residuals = np.abs((inlier_points - center).dot(normal))
        stats = {
            "input_points": int(input_count),
            "candidate_points": int(points.shape[0]),
            "inlier_points": int(inlier_points.shape[0]),
            "inlier_ratio": float(inlier_points.shape[0]) / float(points.shape[0]),
            "plane_rmse": float(np.sqrt(np.mean(residuals ** 2))),
            "aperture_samples": uv,
        }
        return (
            center, normal, width_axis, float(extents[0]), float(extents[1]), stats
        )

    def _depth_confidence(self, stats, prior):
        cfg = self.config
        count_score = min(1.0, stats["inlier_points"] / float(
            max(cfg.min_depth_points * 4, 1)
        ))
        ratio_score = np.clip((stats["inlier_ratio"] - 0.20) / 0.65, 0.0, 1.0)
        rmse_score = math.exp(
            -stats["plane_rmse"] / max(cfg.plane_distance_threshold, 1e-6)
        )
        data_score = 0.30 * count_score + 0.35 * ratio_score + 0.35 * rmse_score
        if prior is None or prior.source != "tag":
            return 0.85 * data_score
        return 0.80 * data_score + 0.20 * prior.confidence

    def _covariance(self, stats, width, height, normal, width_axis):
        cfg = self.config
        count = max(stats["inlier_points"], 1)
        sigma_plane = max(
            cfg.covariance_floor_position,
            stats["plane_rmse"] / math.sqrt(count),
        )
        sigma_lateral = max(
            cfg.covariance_floor_position,
            max(width, height) / math.sqrt(12.0 * count),
        )
        sigma_angle = max(
            math.radians(cfg.covariance_floor_angle_deg),
            stats["plane_rmse"] / max(min(width, height), 1e-6),
        )
        height_axis = np.cross(normal, width_axis)
        basis = np.column_stack((width_axis, height_axis, normal))
        position_covariance = basis.dot(np.diag([
            sigma_lateral ** 2, sigma_lateral ** 2, sigma_plane ** 2
        ])).dot(basis.T)
        pose = np.zeros((6, 6), dtype=float)
        pose[:3, :3] = position_covariance
        pose[3:, 3:] = np.eye(3) * sigma_angle ** 2
        uv = stats["aperture_samples"]
        spread = np.var(uv, axis=0) / count
        aperture = np.diag([
            max(cfg.covariance_floor_position ** 2, spread[0]),
            max(cfg.covariance_floor_position ** 2, spread[1]),
        ])
        return pose, aperture

    def _prior_covariance(self, prior, status):
        scale = 1.0 if status == SOURCE_TAG_ONLY else 4.0
        position_sigma = max(0.01, 0.04 * max(prior.width, prior.height)) * scale
        angle_sigma = math.radians(3.0) * scale
        pose = np.diag(
            [position_sigma ** 2] * 3 + [angle_sigma ** 2] * 3
        )
        aperture = np.diag([
            (0.05 * prior.width * scale) ** 2,
            (0.05 * prior.height * scale) ** 2,
        ])
        return pose, aperture
