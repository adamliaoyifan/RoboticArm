#!/usr/bin/env python3
"""Fixed box sequences for offline packing replay (P0-a).

Replicates ``pickup_box_spawner_node._choose_entry()`` weighted sampling with
a fixed seed, so offline replay sequences are deterministic and drawn from the
same distribution as Gazebo smoke runs (design §9.1). Pure Python, no ROS.

Caveat: the spawner also draws yaw/jitter from the same RNG between box
selections, so the *exact* interleaving may differ from a live Gazebo run.
For the offline baseline this is acceptable -- the requirement is
reproducibility (same seed -> same sequence) and the correct size distribution,
not byte-identical Gazebo replay.
"""

from __future__ import division

import os
import random

import yaml

_DESC_CONFIG = os.path.normpath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..",
                 "luggage_description", "config", "box_catalog.yaml.example"))


def load_catalog(path=None):
    path = path or _DESC_CONFIG
    with open(path, "r") as handle:
        return yaml.safe_load(handle)


def catalog_entries(catalog_config):
    entries = []
    for item in catalog_config.get("box_catalog", []):
        entries.append({
            "id": item.get("id", "standard"),
            "size": [float(v) for v in item.get("size", [0.70, 0.45, 0.28])],
            "weight": float(item.get("weight", 1.0)),
        })
    return entries


def choose_entry(entries, rng):
    """Replicates pickup_box_spawner_node._choose_entry weighted sampling."""
    total = sum(max(0.0, e["weight"]) for e in entries)
    if total <= 0.0:
        return rng.choice(entries)
    pick = rng.uniform(0.0, total)
    running = 0.0
    for e in entries:
        running += max(0.0, e["weight"])
        if pick <= running:
            return e
    return entries[-1]


def size_range(catalog_config):
    """Per-axis (min, max) envelope, mirroring box_catalog_utils.box_size_range."""
    section = catalog_config.get("size_range") or {}
    axes = ("width", "depth", "height")
    if all(axis in section for axis in axes):
        return tuple(
            (float(section[axis][0]), float(section[axis][1])) for axis in axes)
    sizes = [entry["size"] for entry in catalog_entries(catalog_config)]
    return tuple(
        (min(s[i] for s in sizes), max(s[i] for s in sizes)) for i in range(3))


def sample_size(ranges, rng, decimals=3):
    return [
        round(rng.uniform(float(low), float(high)), decimals)
        for low, high in ranges
    ]


def generate_sequence(seed, length, entries=None, catalog_path=None,
                      size_mode="continuous", ranges=None):
    """Return a deterministic list of box_size [l,w,h] for the given seed.

    ``continuous`` matches what the spawner now produces; the offline gate has
    to be drawn from the same distribution as the runtime, or the two diverge
    silently. ``catalog`` keeps the historical three-size sequences for
    comparison against earlier replay numbers.
    """
    rng = random.Random(seed)
    if size_mode == "continuous":
        if ranges is None:
            ranges = size_range(load_catalog(catalog_path))
        return [sample_size(ranges, rng) for _ in range(length)]
    if entries is None:
        entries = catalog_entries(load_catalog(catalog_path))
    return [list(choose_entry(entries, rng)["size"]) for _ in range(length)]


def generate_sequences(seeds, length, catalog_path=None,
                       size_mode="continuous"):
    """Return {seed: [box_size,...]} for each seed (shared catalog)."""
    catalog = load_catalog(catalog_path)
    entries = catalog_entries(catalog)
    ranges = size_range(catalog)
    return {
        seed: generate_sequence(
            seed, length, entries, size_mode=size_mode, ranges=ranges)
        for seed in seeds
    }
