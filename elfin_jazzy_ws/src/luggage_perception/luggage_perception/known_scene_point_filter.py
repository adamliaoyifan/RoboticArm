#!/usr/bin/env python3
"""Filter points belonging to configured, already-modelled static geometry."""

from __future__ import division

import math


def _rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _transpose(matrix):
    return tuple(tuple(matrix[j][i] for j in range(3)) for i in range(3))


def _mat_vec(matrix, vector):
    return tuple(
        sum(matrix[i][j] * vector[j] for j in range(3))
        for i in range(3)
    )


class OrientedBox:
    def __init__(self, name, center, size, rpy=(0.0, 0.0, 0.0)):
        self.name = str(name)
        self.center = tuple(float(v) for v in center)
        self.half = tuple(float(v) * 0.5 for v in size)
        self._world_to_local = _transpose(_rpy_matrix(*rpy))

    def contains(self, point, padding=0.0):
        delta = tuple(float(point[i]) - self.center[i] for i in range(3))
        local = _mat_vec(self._world_to_local, delta)
        return all(
            abs(local[i]) <= self.half[i] + float(padding)
            for i in range(3)
        )


class KnownScenePointFilter:
    """Analytic masks for container shell, pedestal, platform, and ground."""

    def __init__(
        self,
        container_outer=None,
        container_inner=None,
        solid_boxes=None,
        base_in_world=None,
        padding=0.03,
        enabled=True,
        filter_ground=True,
    ):
        self.container_outer = container_outer
        self.container_inner = container_inner
        self.solid_boxes = list(solid_boxes or [])
        self.base_in_world = base_in_world
        self.padding = max(0.0, float(padding))
        self.enabled = bool(enabled)
        self.filter_ground = bool(filter_ground)
        self.last_stats = {}

    @classmethod
    def from_scene_config(cls, config, padding=0.03, enabled=True, filter_ground=True):
        from luggage_description.scene_tf_config_utils import (
            _compose,
            _invert_transform,
            base_in_world,
            container_in_base_link,
            container_outer_dimensions,
            container_usable_center_in_base_link,
            container_usable_dimensions,
            pedestal_config,
            pedestal_enabled,
            pickup_platform_config,
            pickup_platform_enabled,
            pickup_platform_in_world,
        )

        container_xyz, container_rpy = container_in_base_link(config)
        outer_l, outer_w, outer_h = container_outer_dimensions(config)
        inner_l, inner_w, inner_h = container_usable_dimensions(config)
        outer_center_offset, _ = _compose(
            [0.0, 0.0, 0.0], container_rpy,
            [0.0, 0.0, outer_h * 0.5], [0.0, 0.0, 0.0],
        )
        inner_center = container_usable_center_in_base_link(config)
        outer = OrientedBox(
            "container_outer",
            [container_xyz[i] + outer_center_offset[i] for i in range(3)],
            [outer_l, outer_w, outer_h],
            container_rpy,
        )
        inner = OrientedBox(
            "container_inner",
            inner_center,
            [inner_l, inner_w, inner_h],
            container_rpy,
        )

        solids = []
        if pedestal_enabled(config):
            ped = pedestal_config(config)
            # In base coordinates the robot base is at the pedestal top.
            solids.append(OrientedBox(
                "pedestal",
                [0.0, 0.0, -ped["size"][2] * 0.5],
                ped["size"],
                [0.0, 0.0, 0.0],
            ))

        base_xyz, base_rpy = base_in_world(config)
        world_to_base_xyz, world_to_base_rpy = _invert_transform(base_xyz, base_rpy)
        if pickup_platform_enabled(config):
            platform = pickup_platform_config(config)
            platform_xyz, platform_rpy = pickup_platform_in_world(config)
            center_world, center_rpy = _compose(
                platform_xyz, platform_rpy,
                [0.0, 0.0, platform["size"][2] * 0.5],
                [0.0, 0.0, 0.0],
            )
            center_base, rpy_base = _compose(
                world_to_base_xyz, world_to_base_rpy,
                center_world, center_rpy,
            )
            solids.append(OrientedBox(
                "pickup_platform", center_base, platform["size"], rpy_base
            ))

        return cls(
            container_outer=outer,
            container_inner=inner,
            solid_boxes=solids,
            base_in_world=(base_xyz, base_rpy),
            padding=padding,
            enabled=enabled,
            filter_ground=filter_ground,
        )

    def _on_ground(self, point):
        if not self.filter_ground or self.base_in_world is None:
            return False
        base_xyz, base_rpy = self.base_in_world
        rotated = _mat_vec(_rpy_matrix(*base_rpy), point)
        world_z = base_xyz[2] + rotated[2]
        return abs(world_z) <= self.padding

    def classify(self, point):
        if not self.enabled:
            return None
        if (
            self.container_outer is not None
            and self.container_outer.contains(point, self.padding)
            and (
                self.container_inner is None
                or not self.container_inner.contains(point, -self.padding)
            )
        ):
            return "container_shell"
        for box in self.solid_boxes:
            if box.contains(point, self.padding):
                return box.name
        if self._on_ground(point):
            return "ground"
        return None

    def filter_points(self, points):
        kept = []
        kept_indices = []
        dropped_by_class = {}
        for index, point in enumerate(points):
            reason = self.classify(point)
            if reason is None:
                kept.append(point)
                kept_indices.append(index)
            else:
                dropped_by_class[reason] = dropped_by_class.get(reason, 0) + 1
        self.last_stats = {
            "enabled": self.enabled,
            "raw_count": len(points),
            "filtered_count": len(kept),
            "dropped_static": len(points) - len(kept),
            "dropped_by_class": dropped_by_class,
        }
        return kept, kept_indices
