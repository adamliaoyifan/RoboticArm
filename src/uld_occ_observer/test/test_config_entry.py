#!/usr/bin/env python3
"""Prove discretize true/false reaches CLI, profile CLI, and eval_native."""
import hashlib
import json
import subprocess
import sys
import tempfile
from pathlib import Path

import yaml


def geometry_text():
    canonical = (
        '{"ceiling_z":1.2,"floor_z":0.0,"frame_id":"container_link",'
        '"geometry_hash":"fixture_geometry","length":0.8,"schema_version":1,"width":0.8}'
    )
    digest = hashlib.sha256(canonical.encode()).hexdigest()
    return (
        "schema: uld_geometry_export_v1\n"
        f"descriptor_sha256: {digest}\n"
        f"descriptor_json: '{canonical}'\n"
    )


def config_text(discretize):
    text = "resolution_m: 0.05\ndepth_z_min_m: 0.52\ndepth_z_max_m: 6.0\nclock_domain: test_clock\n"
    if discretize is not None:
        text += f"discretize_endpoints: {'true' if discretize else 'false'}\n"
    return text


def run(cmd):
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise SystemExit(result.stderr or result.stdout or f"failed: {cmd}")
    return result


def main():
    cli, profile, eval_bin, default_yaml = sys.argv[1:5]
    default_bytes = Path(default_yaml).read_bytes()
    with tempfile.TemporaryDirectory(prefix="occ-entry-") as raw:
        root = Path(raw)
        (root / "geometry.yaml").write_text(geometry_text())
        reports = {}
        for name, flag in (("off", False), ("on", True)):
            cfg = root / f"{name}.yaml"
            cfg.write_text(config_text(flag))
            out = root / f"{name}.json"
            run([eval_bin, "--geometry", str(root / "geometry.yaml"), "--config", str(cfg),
                 "--gate", "config-wire", "--out", str(out)])
            reports[name] = json.loads(out.read_text())
        if reports["off"]["discretize"] != "false" or reports["on"]["discretize"] != "true":
            raise SystemExit("eval_native did not report the loaded flag")
        if reports["off"]["free_keys"] == reports["on"]["free_keys"]:
            raise SystemExit("eval_native store ignored discretize")
        points = [[float(reports["on"][k]) for k in ("ax", "ay", "az")],
                  [float(reports["on"][k]) for k in ("bx", "by", "bz")]]

        def integrate(flag):
            cfg = root / ("cli-on.yaml" if flag else "cli-off.yaml")
            cfg.write_text(config_text(flag))
            command = {
                "op": "integrate",
                "sensor_id": "test",
                "observation_id": "1",
                "optical_frame": "optical",
                "clock_domain": "test_clock",
                "geometry_hash": "fixture_geometry",
                "epoch": 1,
                "stamp_ns": 1000,
                "points_m": points,
                "transform": {"stamp_ns": 1000, "translation": [0, 0, 0],
                              "rotation": [1, 0, 0, 0, 1, 0, 0, 0, 1]},
            }
            commands = root / ("cmd-on.yaml" if flag else "cmd-off.yaml")
            out = root / ("cli-on-out.yaml" if flag else "cli-off-out.yaml")
            commands.write_text(yaml.safe_dump([command]))
            run([cli, "--geometry", str(root / "geometry.yaml"), "--config", str(cfg),
                 "--commands", str(commands), "--out", str(out)])
            return yaml.safe_load(out.read_text())[0]["free_keys"]

        if integrate(False) == integrate(True):
            raise SystemExit("observer_cli ignored discretize")

        def profile_free(flag):
            cfg = root / ("profile-on.yaml" if flag else "profile-off.yaml")
            cfg.write_text(config_text(flag))
            frame = {
                "sensor_id": "test", "observation_id": "1", "optical_frame": "optical",
                "clock_domain": "test_clock", "geometry_hash": "fixture_geometry",
                "epoch": 1, "stamp_ns": 1000, "points_m": points,
                "transform": {"stamp_ns": 1000, "translation": [0, 0, 0],
                              "rotation": [1, 0, 0, 0, 1, 0, 0, 0, 1]},
            }
            frame_path = root / ("frame-on.yaml" if flag else "frame-off.yaml")
            frame_path.write_text(yaml.safe_dump(frame))
            manifest = {
                "case_id": "entry", "mode": "off", "surface": False,
                "frames": [{"path": frame_path.name,
                            "sha256": hashlib.sha256(frame_path.read_bytes()).hexdigest(),
                            "warmup": False, "expect_status": "INTEGRATED"}],
            }
            man = root / ("man-on.yaml" if flag else "man-off.yaml")
            man.write_text(yaml.safe_dump(manifest))
            out = root / ("profile-on-out.yaml" if flag else "profile-off-out.yaml")
            run([profile, "--geometry", str(root / "geometry.yaml"), "--config", str(cfg),
                 "--manifest", str(man), "--out", str(out)])
            row = next(yaml.safe_load_all(out.read_text()))
            return row["receipt"]["free_keys"]

        if profile_free(False) == profile_free(True):
            raise SystemExit("profile_cli ignored discretize")
        if Path(default_yaml).read_bytes() != default_bytes:
            raise SystemExit("default yaml bytes changed")
    print("config_entry=ok")


if __name__ == "__main__":
    main()
