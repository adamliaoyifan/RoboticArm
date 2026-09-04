#!/usr/bin/env python3
"""Generate Gazebo model.sdf with visual mesh and box collision primitives."""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path
from typing import Any


def _pose_str(center: list[float]) -> str:
    return f"{center[0]} {center[1]} {center[2]} 0 0 0"


def _size_str(size: list[float]) -> str:
    return f"{size[0]} {size[1]} {size[2]}"


def build_sdf(model_name: str, collision_boxes: list[dict[str, Any]], link_pose: str) -> str:
    collisions = []
    for i, box in enumerate(collision_boxes):
        c, s = box["center"], box["size"]
        collisions.append(
            f"""      <collision name="collision_{i}_{box.get('name', 'box')}">
        <pose>{_pose_str(c)}</pose>
        <geometry><box><size>{_size_str(s)}</size></box></geometry>
      </collision>"""
        )
    col_xml = "\n".join(collisions)
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>true</static>
    <link name="container_link">
      <pose>{link_pose}</pose>
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://{model_name}/meshes/container_visual.stl</uri>
            <scale>1 1 1</scale>
          </mesh>
        </geometry>
        <material>
          <ambient>0.25 0.35 0.45 1</ambient>
          <diffuse>0.35 0.45 0.55 1</diffuse>
        </material>
      </visual>
{col_xml}
    </link>
  </model>
</sdf>
"""


CONFIG_TEMPLATE = """<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>2.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>parametric cargo pipeline</name>
  </author>
  <description>Parametric hexagonal container with legs and opening</description>
</model>
"""


def log_stl_aabb(path: Path) -> None:
    import numpy as np
    import open3d as o3d

    mesh = o3d.io.read_triangle_mesh(str(path))
    verts = np.asarray(mesh.vertices)
    if verts.size == 0:
        print(f"WARN empty STL: {path}")
        return
    mn, mx = verts.min(0), verts.max(0)
    print(f"STL AABB {path.name}: min={mn.round(3).tolist()} max={mx.round(3).tolist()}")


def generate_model(
    output_dir: Path,
    model_name: str,
    install_dir: Path | None = None,
) -> Path:
    mesh_src = output_dir / "meshes"
    for name in ("container_visual.stl", "container_collision.stl"):
        if not (mesh_src / name).exists():
            raise FileNotFoundError(f"Missing mesh: {mesh_src / name}")
        log_stl_aabb(mesh_src / name)

    boxes_path = output_dir / "collision_boxes.json"
    if boxes_path.exists():
        collision_boxes = json.loads(boxes_path.read_text(encoding="utf-8"))
    else:
        collision_boxes = [{"name": "fallback", "center": [0, 0, 1], "size": [2, 2, 2]}]

    link_pose = "0 0 0 0 0 0"
    gazebo_out = output_dir / "gazebo" / model_name
    gazebo_out.mkdir(parents=True, exist_ok=True)
    mesh_dst = gazebo_out / "meshes"
    mesh_dst.mkdir(parents=True, exist_ok=True)
    for name in ("container_visual.stl", "container_collision.stl"):
        shutil.copy2(mesh_src / name, mesh_dst / name)

    (gazebo_out / "model.sdf").write_text(
        build_sdf(model_name, collision_boxes, link_pose), encoding="utf-8"
    )
    (gazebo_out / "model.config").write_text(
        CONFIG_TEMPLATE.format(model_name=model_name), encoding="utf-8"
    )

    if install_dir is not None:
        install_dir = install_dir.resolve()
        if install_dir.exists():
            shutil.rmtree(install_dir)
        shutil.copytree(gazebo_out, install_dir)
        return install_dir
    return gazebo_out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "output",
    )
    parser.add_argument("--model-name", default="airport_container_measured")
    parser.add_argument("--install", action="store_true")
    parser.add_argument("--model-dir", type=Path, default=None)
    args = parser.parse_args()

    install_dir = None
    if args.install:
        if args.model_dir:
            install_dir = args.model_dir
        else:
            pointcloud_dir = Path(__file__).resolve().parents[1]
            robotarm = pointcloud_dir.parent.parent.parent.parent
            install_dir = (
                robotarm
                / "elfin_noetic_ws"
                / "src"
                / "luggage_gazebo"
                / "models"
                / args.model_name
            )

    out = generate_model(args.output_dir, args.model_name, install_dir)
    print(f"Gazebo model at: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
