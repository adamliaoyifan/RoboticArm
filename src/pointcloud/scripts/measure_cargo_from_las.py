#!/usr/bin/env python3
"""Parametric cargo pipeline: preprocess -> fit -> CAD mesh -> Gazebo SDF."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from cargo_cad_model import build_cad_model  # noqa: E402
from cargo_io import CloudCompareBinError, load_point_cloud  # noqa: E402
from cargo_parametric_fit import fit_parametric_model, write_model_outputs  # noqa: E402
from cargo_preprocess import preprocess  # noqa: E402
from generate_gazebo_model import generate_model, log_stl_aabb  # noqa: E402


def load_config(path: Path) -> dict:
    with path.open(encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _robotarm_root() -> Path:
    return SCRIPT_DIR.parent.parent.parent.parent.parent


def _input_path(cfg: dict) -> Path:
    raw = cfg.get("input_cloud") or cfg.get("input_las")
    if not raw:
        raise KeyError("Config must set input_las or input_cloud")
    return Path(raw).expanduser()


def run(cfg: dict, only: str | None, install_gazebo: bool) -> int:
    output_dir = Path(cfg.get("output_dir", "./output"))
    output_dir.mkdir(parents=True, exist_ok=True)

    prep = None
    fit = None

    if only in (None, "preprocess", "fit", "cad", "gazebo"):
        input_path = _input_path(cfg)
        if not input_path.exists():
            print(f"ERROR: input point cloud not found: {input_path}", file=sys.stderr)
            return 1
        try:
            xyz, rgb, _ = load_point_cloud(input_path)
        except CloudCompareBinError as exc:
            print(f"ERROR: {exc}", file=sys.stderr)
            return 1
        prep = preprocess(xyz, rgb, cfg["preprocess"], output_dir=output_dir)
        print("Preprocess stats:", prep.stats)
        if only == "preprocess":
            print("Inspect body_points.ply / leg_points.ply in CloudCompare.")
            return 0

    if only in (None, "fit", "cad", "gazebo"):
        if prep is None:
            print("ERROR: run preprocess first", file=sys.stderr)
            return 1
        fit = fit_parametric_model(
            prep.body_xyz,
            prep.legs_xyz,
            prep.transform,
            prep.stats,
            cfg.get("fit", {}),
        )
        write_model_outputs(fit, output_dir)
        print("\n".join(fit.report_lines))
        if only == "fit":
            return 0

    if only in (None, "cad", "gazebo"):
        model_path = output_dir / "parametric_model.json"
        if fit is None and model_path.exists():
            model = json.loads(model_path.read_text(encoding="utf-8"))
        elif fit is not None:
            model = fit.model
        else:
            print("ERROR: run fit first", file=sys.stderr)
            return 1

        cad = build_cad_model(model, output_dir)
        print(f"Visual mesh: {cad.visual_path} ({cad.visual_path.stat().st_size} bytes)")
        print(f"Collision boxes: {len(cad.collision_boxes)}")
        print(f"Debug: {cad.debug_path}")
        if only == "cad":
            return 0

    if only is None or install_gazebo or only == "gazebo":
        gazebo_cfg = cfg.get("gazebo", {})
        model_name = gazebo_cfg.get("model_name", "airport_container_measured")
        install_dir = None
        if install_gazebo:
            install_dir = (
                _robotarm_root()
                / "elfin_noetic_ws"
                / "src"
                / "luggage_gazebo"
                / "models"
                / model_name
            )
        vis = output_dir / "meshes" / "container_visual.stl"
        if not vis.exists():
            print(f"ERROR: missing {vis}", file=sys.stderr)
            return 1
        log_stl_aabb(vis)
        path = generate_model(output_dir, model_name, install_dir)
        print(f"Gazebo model: {path}")

    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-c",
        "--config",
        type=Path,
        default=SCRIPT_DIR.parent / "config" / "measure_cargo.yaml",
    )
    parser.add_argument(
        "--only",
        choices=["preprocess", "fit", "cad", "gazebo"],
        default=None,
        help="preprocess | fit | cad | gazebo (default: full pipeline)",
    )
    parser.add_argument("--install-gazebo", action="store_true")
    args = parser.parse_args()

    if not args.config.exists():
        example = args.config.parent / "measure_cargo.yaml.example"
        print(f"Config missing: {args.config}\nCopy: cp {example} {args.config}", file=sys.stderr)
        return 1

    only = args.only
    if args.install_gazebo and only is None:
        only = None
    return run(load_config(args.config), only, args.install_gazebo)


if __name__ == "__main__":
    raise SystemExit(main())
