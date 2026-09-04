#!/usr/bin/env python3
"""Load active-loading box catalog configs."""

from __future__ import division

import math
import os

import yaml

from luggage_description._share import description_config_path


def default_box_catalog_path():
    return description_config_path("box_catalog.yaml.example")


def box_catalog_path_from_scene(scene_config):
    catalog_name = scene_config.get("container", {}).get("box_catalog_config")
    if not catalog_name:
        return default_box_catalog_path()
    return description_config_path(str(catalog_name))


def load_box_catalog(path=None, scene_config=None):
    path = path or (box_catalog_path_from_scene(scene_config) if scene_config else None)
    path = path or default_box_catalog_path()
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def box_catalog_yaw_defaults(catalog_config):
    """Catalog-level yaw randomization defaults (mode, [min, max])."""
    mode = str(catalog_config.get("yaw_mode", "discrete")).strip().lower()
    yaw_range = catalog_config.get("yaw_range", [-math.pi, math.pi])
    yaw_range = [float(yaw_range[0]), float(yaw_range[1])]
    return mode, yaw_range


def box_size_range(catalog_config):
    """Per-axis (min, max) sampling envelope as ((w0,w1),(d0,d1),(h0,h1)).

    Falls back to the bounding envelope of the catalog entries so a config
    without an explicit ``size_range`` still supports continuous sampling.
    """
    section = catalog_config.get("size_range") or {}
    axes = ("width", "depth", "height")
    if all(axis in section for axis in axes):
        return tuple(
            (float(section[axis][0]), float(section[axis][1])) for axis in axes)
    sizes = [entry["size"] for entry in box_catalog_entries(catalog_config)]
    if not sizes:
        raise ValueError("box catalog has neither size_range nor entries")
    return tuple(
        (min(s[i] for s in sizes), max(s[i] for s in sizes)) for i in range(3))


def box_mass_model(catalog_config):
    """Density-from-volume mass model parameters.

    Returns a dict with ``density_base_kg_m3``, ``density_slope_kg_m6``,
    ``density_jitter``, ``min_kg`` and ``max_kg``.
    """
    section = catalog_config.get("mass_model") or {}
    return {
        "density_base_kg_m3": float(
            section.get("density_base_kg_m3", 119.7)),
        "density_slope_kg_m6": float(
            section.get("density_slope_kg_m6", 468.5)),
        "density_jitter": float(section.get("density_jitter", 0.0)),
        "min_kg": float(section.get("min_kg", 0.5)),
        "max_kg": float(section.get("max_kg", 23.0)),
    }


def nominal_box_mass(size, mass_model):
    """Un-jittered mass for ``size`` under ``mass_model`` (kg), clamped."""
    volume = float(size[0]) * float(size[1]) * float(size[2])
    density = (
        mass_model["density_base_kg_m3"]
        + mass_model["density_slope_kg_m6"] * volume)
    return _clamp_mass(density * volume, mass_model)


def _clamp_mass(mass_kg, mass_model):
    return max(
        mass_model["min_kg"], min(mass_model["max_kg"], float(mass_kg)))


def sample_box_mass(size, mass_model, rng):
    """Mass for ``size`` with the configured density jitter applied."""
    jitter = mass_model["density_jitter"]
    factor = 1.0 + rng.uniform(-jitter, jitter) if jitter > 0.0 else 1.0
    return _clamp_mass(nominal_box_mass(size, mass_model) * factor, mass_model)


def sample_box_size(size_range, rng, decimals=3):
    """Uniform per-axis sample inside ``size_range``.

    Rounded so the spawned SDF, the manifest and the logs all agree on the
    exact value; an unrounded float would differ by float-formatting alone.
    """
    return [
        round(rng.uniform(float(low), float(high)), decimals)
        for low, high in size_range
    ]


def box_catalog_entries(catalog_config):
    default_mode, default_range = box_catalog_yaw_defaults(catalog_config)
    entries = []
    for item in catalog_config.get("box_catalog", []):
        size = [float(v) for v in item.get("size", [0.70, 0.45, 0.28])]
        mode = str(item.get("yaw_mode", default_mode)).strip().lower()
        yaw_range = item.get("yaw_range", default_range)
        yaw_range = [float(yaw_range[0]), float(yaw_range[1])]
        entries.append(
            {
                "id": item.get("id", item.get("model", "standard")),
                "model": item.get("model", "suitcase_standard"),
                "visual": str(item.get("visual") or "").strip(),
                "size": size,
                "weight": float(item.get("weight", 1.0)),
                "mass_kg": float(item.get("mass_kg", 0.0)),
                "allowed_yaws": [float(v) for v in item.get("allowed_yaws", [0.0])],
                "yaw_mode": mode,
                "yaw_range": yaw_range,
            }
        )
    return entries


def choose_catalog_entry(entries, rng):
    """Weighted random catalog row (small / medium / large)."""
    if not entries:
        raise ValueError("box catalog is empty")
    total = sum(max(0.0, entry["weight"]) for entry in entries)
    if total <= 0.0:
        return rng.choice(entries)
    pick = rng.uniform(0.0, total)
    running = 0.0
    for entry in entries:
        running += max(0.0, entry["weight"])
        if pick <= running:
            return entry
    return entries[-1]


def sample_pickup_box(entries, rng, size_mode="catalog",
                      size_range=None, mass_model=None):
    """Return (entry, size, mass_kg, generated).

    ``catalog`` (default spawn): the entry's fixed small/medium/large size.
    ``continuous``: sample inside *size_range* (opt-in; mesh spawn rejects it).
    """
    entry = choose_catalog_entry(entries, rng)
    mode = str(size_mode or "catalog").strip().lower()
    if mode == "continuous":
        if size_range is None or mass_model is None:
            raise ValueError("continuous size_mode needs size_range and mass_model")
        size = sample_box_size(size_range, rng)
        mass = sample_box_mass(size, mass_model, rng)
        return entry, size, mass, True
    size = list(entry["size"])
    mass = float(entry.get("mass_kg", 0.0))
    return entry, size, mass, False
