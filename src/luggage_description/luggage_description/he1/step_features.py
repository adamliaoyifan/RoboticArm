"""Parse official D555 STEP cylinders and seating planes (millimetres)."""

from __future__ import division

import re
from collections import defaultdict

import numpy as np

from luggage_description.he1.geometry import as_unit, fit_plane


_ENTITY = re.compile(r"^#(\d+)\s*=\s*([A-Z0-9_]+)\s*\((.*)\)\s*;\s*$")
_FLOAT = re.compile(r"[-+]?\d+\.\d+(?:[Ee][-+]?\d+)?|[-+]?\d+[Ee][-+]?\d+")
_REF = re.compile(r"#(\d+)")


def parse_step_entities(text):
    entities = {}
    for line in text.splitlines():
        match = _ENTITY.match(line.strip())
        if match:
            entities[int(match.group(1))] = (match.group(2), match.group(3))
    return entities


def _floats(body):
    return [float(token) for token in _FLOAT.findall(body)]


def _refs(body):
    return [int(token) for token in _REF.findall(body)]


def _cartesian(entities, ident):
    kind, body = entities[ident]
    if kind != "CARTESIAN_POINT":
        raise KeyError(ident)
    return np.array(_floats(body), dtype=np.float64)


def _direction(entities, ident):
    kind, body = entities[ident]
    if kind != "DIRECTION":
        raise KeyError(ident)
    return as_unit(_floats(body))


def _placement(entities, ident):
    kind, body = entities[ident]
    if kind != "AXIS2_PLACEMENT_3D":
        raise KeyError(ident)
    refs = _refs(body)
    origin = _cartesian(entities, refs[0])
    axis = _direction(entities, refs[1])
    return origin, axis


def iter_cylinders(entities):
    for ident, (kind, body) in entities.items():
        if kind != "CYLINDRICAL_SURFACE":
            continue
        refs = _refs(body)
        origin, axis = _placement(entities, refs[0])
        radius = _floats(body)[-1]
        yield ident, origin, axis, float(radius)


def cluster_cylinders(cylinders, radius_min, radius_max, lateral_tol=0.4, axis_dot=0.995):
    selected = [
        item for item in cylinders
        if radius_min - 1e-9 <= item[3] <= radius_max + 1e-9
    ]
    used = [False] * len(selected)
    holes = []
    for i, (_ident, origin, axis, radius) in enumerate(selected):
        if used[i]:
            continue
        axis = as_unit(axis)
        if axis[int(np.argmax(np.abs(axis)))] < 0.0:
            axis = -axis
        group = [(origin, axis, radius)]
        used[i] = True
        for j in range(i + 1, len(selected)):
            if used[j]:
                continue
            _id2, origin2, axis2, radius2 = selected[j]
            axis2 = as_unit(axis2)
            if axis2[int(np.argmax(np.abs(axis2)))] < 0.0:
                axis2 = -axis2
            if abs(radius2 - radius) > 0.15:
                continue
            if abs(float(axis.dot(axis2))) < axis_dot:
                continue
            delta = origin2 - origin
            lateral = np.linalg.norm(delta - delta.dot(axis) * axis)
            if lateral < lateral_tol:
                group.append((origin2, axis2, radius2))
                used[j] = True
        centres = np.mean([item[0] for item in group], axis=0)
        axes = as_unit(np.mean([item[1] for item in group], axis=0))
        radii = float(np.mean([item[2] for item in group]))
        holes.append({
            "centre": centres,
            "axis": axes,
            "radius": radii,
            "support_surfaces": len(group),
            "rms": 0.0,
        })
    return holes


def extract_d555_features(step_text):
    entities = parse_step_entities(step_text)
    cylinders = list(iter_cylinders(entities))
    units = "millimetres"
    if "SI_UNIT ( .MILLI., .METRE. )" not in step_text and "SI_UNIT(.MILLI.,.METRE.)" not in step_text.replace(" ", ""):
        if "SI_UNIT ( $, .METRE. )" in step_text:
            units = "metres"

    solids = [
        body.strip(" '")
        for ident, (kind, body) in entities.items()
        if kind == "MANIFOLD_SOLID_BREP"
    ]

    m4 = cluster_cylinders(cylinders, 2.05, 2.15)
    # Official end-mount pair: r=2.1 mm, 125.40 mm apart, rear face z~-48.
    pair = None
    best = 1e9
    for i in range(len(m4)):
        for j in range(i + 1, len(m4)):
            delta = np.linalg.norm(m4[i]["centre"] - m4[j]["centre"])
            err = abs(delta - 125.40)
            same_z = abs(m4[i]["centre"][2] - m4[j]["centre"][2]) < 1.0
            same_y = abs(m4[i]["centre"][1] - m4[j]["centre"][1]) < 1.0
            if same_z and same_y and err < best:
                best = err
                pair = (i, j, float(delta))
    if pair is None:
        raise RuntimeError("no 125.40 mm M4 pair in D555 STEP")

    left_i, right_i, spacing = pair
    if m4[left_i]["centre"][0] > m4[right_i]["centre"][0]:
        left_i, right_i = right_i, left_i
    hole_l = dict(m4[left_i])
    hole_r = dict(m4[right_i])
    hole_l["id"] = "CAD_M4_NEG"
    hole_r["id"] = "CAD_M4_POS"
    hole_l["source"] = "STEP CYLINDRICAL_SURFACE cluster"
    hole_r["source"] = "STEP CYLINDRICAL_SURFACE cluster"

    plane_points = np.vstack((hole_l["centre"], hole_r["centre"]))
    # Seating = rear face through the two M4 centres; +Z toward the optical
    # windows (CAD z increases from -48 rear to 0 front).
    centroid = plane_points.mean(axis=0)
    normal = np.array([0.0, 0.0, 1.0])
    if centroid[2] > -10.0:
        # Front-face mount would already be near z=0; keep +Z anyway.
        pass
    seat = {
        "id": "CAD_SEAT_REAR",
        "centroid": centroid,
        "normal": normal,
        "rms": 0.0,
        "support": 2,
        "source": "plane through official 2x M4, +Z toward front windows",
    }

    envelope = {
        "datasheet_mm": [167.0, 42.0, 48.0],
        "cad_aabb_mm": None,
    }
    return {
        "units": units,
        "solids": solids,
        "entity_count": len(entities),
        "cylinder_count": len(cylinders),
        "holes": [hole_l, hole_r],
        "spacing_mm": spacing,
        "spacing_datasheet_mm": 125.40,
        "spacing_datasheet_tol_mm": 0.20,
        "seating": seat,
        "envelope": envelope,
        "m4_cluster_count": len(m4),
    }


def load_step_features(path):
    with open(path, "r", encoding="utf-8", errors="replace") as handle:
        text = handle.read()
    header = "\n".join(text.splitlines()[:16])
    features = extract_d555_features(text)
    features["header"] = header
    return features
