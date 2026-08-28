#!/usr/bin/env python3
"""Resolve scene mesh assets shared by Gazebo and MoveIt."""

from __future__ import division

import os

from luggage_description._share import gazebo_share
from luggage_description.scene_tf_config_utils import gazebo_container_model


def container_model_name(scene_config):
    """Return the configured Gazebo container model name."""
    return gazebo_container_model(scene_config)


def _container_model_dir(scene_config):
    return os.path.join(
        gazebo_share(),
        "models",
        container_model_name(scene_config),
    )


def container_collision_mesh_path(scene_config):
    """Return the installed collision STL path for the configured container."""
    return os.path.join(
        _container_model_dir(scene_config),
        "meshes",
        "container_collision.stl",
    )


def container_visual_mesh_path(scene_config):
    """Return the installed visual STL path for the configured container."""
    return os.path.join(
        _container_model_dir(scene_config),
        "meshes",
        "container_visual.stl",
    )


def require_existing_mesh(path):
    """Validate a mesh path and return it, raising a useful error if missing."""
    if not os.path.exists(path):
        raise FileNotFoundError("Container mesh not found: %s" % path)
    return path
