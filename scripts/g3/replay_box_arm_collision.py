#!/usr/bin/env python3
"""Compare captured nominal/physical joints against exact box/link2 geometry.

Uses the physical spawn SDF and constant tool-to-box transform established by
the follow trace. This diagnoses physical geometry; it does not assume that
the certifier used the same payload representation.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import trimesh

REPO = Path(__file__).resolve().parents[2]
for relative in ("scripts", "src/joint_command_capture/python"):
    sys.path.insert(0, str(REPO / relative))
from replay_collision_attribution import Urdf, _origin_of, _resolve_package_path
from joint_command_capture.records_schema import load_run


def pose(xyz, quaternion):
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    transform[:3, 3] = xyz
    return transform


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--probe", default="/tmp/phase2_mesh_box_collision_probe")
    args = parser.parse_args()
    urdf = Urdf(args.urdf.read_text())
    element = urdf.link_els["elfin_link2"].find("collision")
    xyz, rotation = _origin_of(element)
    collision_origin = np.eye(4)
    collision_origin[:3, :3] = rotation
    collision_origin[:3, 3] = xyz
    mesh_element = element.find("geometry/mesh")
    mesh = trimesh.load(_resolve_package_path(mesh_element.get("filename")),
                        force="mesh", process=False)
    mesh.vertices *= np.array([float(v) for v in mesh_element.get("scale", "1 1 1").split()])
    sdf = ET.parse(args.run / "spawn.sdf").getroot()
    shapes = []
    for collision in sdf.findall("model/link/collision"):
        size = [float(v) for v in collision.findtext("geometry/box/size").split()]
        values = [float(v) for v in collision.findtext("pose", "0 0 0 0 0 0").split()]
        transform = pose(values[:3], Rotation.from_euler("xyz", values[3:]).as_quat())
        shapes.append((collision.get("name"), size, transform))
    follow = [json.loads(line) for line in (args.run/"joint_capture/follow_trace.jsonl").open()]
    offsets = [np.linalg.inv(pose(row["panel_xyz"], row["panel_quat"])) @
               pose(row["box_xyz"], row["box_quat"]) for row in follow[::100]]
    tool_from_box = offsets[0]
    dump = next((args.run/"driver_out/dumps").glob("box_*"))
    trial = json.loads((dump/"trial.json").read_text())
    leg = trial["extras"]["submitted_legs"][0]
    if leg["trajectory"]["truncated"]:
        raise ValueError("nominal trajectory is incomplete")
    datasets = [("nominal", index, point["q"], point["t"])
                for index, point in enumerate(leg["trajectory"]["points"])]
    actual = load_run(args.run/"joint_capture")["plugin_read"].records
    actual = actual[actual["sim_time_ns"] >= int(leg["submit_sim_stamp"]*1e9)]
    for index in range(0, len(actual), 50):
        row = actual[index]
        datasets.append(("actual", index, row["position"][:6].tolist(),
                         float(row["sim_time_ns"]*1e-9)))
    cases, metadata = [], {}
    for kind, index, joints, stamp in datasets:
        transforms = urdf.link_transforms(joints)
        mesh_tf = transforms["elfin_link2"] @ collision_origin
        box_tf = transforms["suction_contact_frame"] @ tool_from_box
        for name, size, origin in shapes:
            label = "%s_%d_%s" % (kind, index, name)
            cases.append(" ".join(map(str, [label, *mesh_tf.flat, *size, *(box_tf @ origin).flat])))
            metadata[label] = dict(kind=kind, index=index, time=stamp, collision=name,
                                   joints=joints)
    lines = ["%d %d" % (len(mesh.vertices), len(mesh.faces))]
    lines.extend(" ".join(map(str, v)) for v in mesh.vertices)
    lines.extend(" ".join(map(str, face)) for face in mesh.faces)
    lines.append(str(len(cases)))
    lines.extend(cases)
    output = subprocess.run([args.probe], input="\n".join(lines)+"\n",
                            text=True, capture_output=True, check=True)
    hits, minimum = [], {}
    for line in output.stdout.splitlines():
        label, collision, distance = line.split()
        item = metadata[label]
        minimum[item["kind"]] = min(minimum.get(item["kind"], float("inf")), float(distance))
        if collision == "1":
            hits.append(dict(item, distance=float(distance)))
    result = dict(
        geometry="URDF elfin_link2 mesh vs all five physical SDF boxes",
        tool_from_box=tool_from_box.tolist(),
        max_follow_offset_matrix_delta=float(max(np.max(abs(offset-tool_from_box)) for offset in offsets)),
        cases=len(cases), dataset_counts=dict(Counter(row[0] for row in datasets)),
        hit_counts=dict(Counter(row["kind"] for row in hits)), minimum_distance=minimum,
        first_hits={kind: next((row for row in hits if row["kind"] == kind), None)
                    for kind in ("nominal", "actual")}, hits=hits)
    (args.run/"offline_box_arm_collision.json").write_text(json.dumps(result, indent=2)+"\n")
    result.pop("hits")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
