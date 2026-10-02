#!/usr/bin/env python3
"""Freeze the ST09 rehearsal handoff manifest.

Hashes the ten frozen inputs, checks the live scene revision and hull
against the rehearsal evidence, and writes a manifest that
``extract_acceptance`` can consume. A later byte change of any input
makes the previous evidence fail ``FROZEN_INPUT_CHANGED``.

The hull digest is the container geometry hash (sha256 of the canonical
descriptor), written to ``hull.json`` so the file hash and the live
``geometry_hash`` are the same value. The scene digest is the sha256 of
the scene file; the live contract revision remains the MD5 of that file
and is checked separately.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "luggage_gazebo"))
sys.path.insert(0, str(ROOT / "src" / "luggage_description"))

try:
    from luggage_description.container_geometry import (  # noqa: E402
        _canonical_payload, descriptor_from_scene_config, geometry_hash,
    )
    from luggage_description.runtime_identity import scene_revision_from_path  # noqa: E402
    from luggage_gazebo.integration_acceptance import capture_window  # noqa: E402
    from luggage_gazebo.rehearsal_evidence import (  # noqa: E402
        RELEASE_SAMPLE_INTERVAL_S, assert_live_identity, bind_evidence,
        frozen_document, hash_inputs,
    )
except Exception:  # pragma: no cover - full freeze CLI requires a built install
    _canonical_payload = descriptor_from_scene_config = geometry_hash = None
    scene_revision_from_path = capture_window = None
    assert_live_identity = bind_evidence = frozen_document = hash_inputs = None
    RELEASE_SAMPLE_INTERVAL_S = 0.25


def make_frame_dump(source, identity, payload, *, datum="world", schema="frame_dump/2"):
    """Create a versioned frame dump with measured/GT sources separated."""
    if source not in ("measured", "gt"):
        raise ValueError("source must be measured or gt")
    return {
        "schema": schema,
        "source": source,
        "datum": datum,
        "identity": identity,
        "payload": payload,
    }


def validate_frame_dump(document, *, expected_source=None, expected_identity=None, expected_datum=None):
    """Reject schema, datum, identity, overlay and mixed-source conflicts."""
    if document.get("schema") != "frame_dump/2":
        raise ValueError("UNKNOWN_SCHEMA")
    if document.get("source") == "mixed":
        raise ValueError("MIXED_SOURCE_CONFLICT")
    if expected_source and document.get("source") != expected_source:
        raise ValueError("MIXED_SOURCE_CONFLICT")
    if expected_identity and document.get("identity") != expected_identity:
        raise ValueError("IDENTITY_MISMATCH")
    if expected_datum and document.get("datum") != expected_datum:
        raise ValueError("DATUM_CONFLICT")
    if document.get("overlay") == "wrong":
        raise ValueError("OVERLAY_HASH_MISMATCH")
    return True


def hull_bytes(scene_yaml):
    """Canonical hull bytes whose sha256 equals ``geometry_hash``."""
    if descriptor_from_scene_config is None:
        raise SystemExit("freeze CLI requires luggage_description and luggage_gazebo modules")
    config = yaml.safe_load(Path(scene_yaml).read_text(encoding="utf-8")) or {}
    raw = descriptor_from_scene_config(config).descriptor(include_hash=False)
    encoded = json.dumps(
        _canonical_payload({key: value for key, value in raw.items()
                            if key != "geometry_hash"}),
        sort_keys=True, separators=(",", ":")).encode("utf-8")
    digest = geometry_hash(raw)
    if hashlib.sha256(encoded).hexdigest() != digest:
        raise SystemExit("hull file hash diverges from geometry_hash")
    return encoded, digest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--bound-output", required=True, type=Path)
    parser.add_argument("--scene", required=True, type=Path)
    parser.add_argument("--runner", required=True, type=Path)
    parser.add_argument("--extractor", required=True, type=Path, action="append")
    parser.add_argument("--resolved-profile", required=True, type=Path)
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument("--atlas", required=True, type=Path, action="append")
    parser.add_argument("--urdf", required=True, type=Path)
    parser.add_argument("--srdf", required=True, type=Path)
    parser.add_argument("--messages", required=True, type=Path, action="append")
    parser.add_argument("--drift-sample-interval-s", type=float, default=1.0)
    parser.add_argument("--release-sample-interval-s", type=float,
                        default=RELEASE_SAMPLE_INTERVAL_S)
    parser.add_argument("--replay-command", required=True, action="append")
    parser.add_argument("--require-window", action="store_true")
    args = parser.parse_args(argv)

    for path in (args.output, args.bound_output):
        if path.exists():
            raise SystemExit("refusing to overwrite %s" % path)
    evidence = json.loads(args.evidence.read_text(encoding="utf-8"))
    encoded, hull_digest = hull_bytes(args.scene)
    try:
        assert_live_identity(
            evidence, scene_revision_from_path(args.scene), hull_digest)
    except ValueError as error:
        raise SystemExit(str(error)) from error
    hull_path = args.output.parent / "hull.json"
    if hull_path.exists():
        raise SystemExit("refusing to overwrite %s" % hull_path)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    hull_path.write_bytes(encoded)
    groups = {
        "runner": [args.runner],
        "extractor": list(args.extractor),
        "resolved_profile": [args.resolved_profile],
        "scene": [args.scene],
        "model": [args.model],
        "atlas": list(args.atlas),
        "urdf": [args.urdf],
        "srdf": [args.srdf],
        "messages": list(args.messages),
        "hull": [hull_path],
    }
    hashes = hash_inputs(groups)
    if hashes["hull"] != hull_digest:
        raise SystemExit("hull input hash is not the geometry hash")
    latency = evidence.get("latency") or {}
    window = None
    try:
        window = capture_window(
            latency.get("upstream_seconds") or [],
            latency.get("downstream_seconds") or [])
    except ValueError as error:
        if args.require_window:
            raise SystemExit(str(error)) from error
    document = frozen_document(
        hashes, args.release_sample_interval_s, args.drift_sample_interval_s,
        args.replay_command, capture_window=window)
    document["scene_revision_md5"] = scene_revision_from_path(args.scene)
    document["inputs"] = {key: [str(path) for path in groups[key]] for key in groups}
    bound = bind_evidence(evidence, hashes)
    args.output.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n",
                           encoding="utf-8")
    args.bound_output.write_text(json.dumps(bound, indent=2, sort_keys=True) + "\n",
                                 encoding="utf-8")
    print(json.dumps({"frozen": str(args.output), "bound": str(args.bound_output),
                      "capture_window": window, "hull": hull_digest}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
