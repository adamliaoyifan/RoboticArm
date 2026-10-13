#!/usr/bin/env python3
"""The single home of the pre-ST05 volume-centered candidate encoding.

ST05 moved placement candidates to container box-center semantics
(``center_container`` with z = floor + support + h/2 and an explicit
box-axis ``box_yaw``). Everything that still needs the historical v1
fields — volume-relative ``center_local``, container-rotated
``center_base``, the ambiguous ``yaw = map_yaw + box_yaw`` label — gets
them from here. No other module may reintroduce the half-height
arithmetic.
"""

import math


def v1_candidate_view(candidate, surface_view):
    """Augment one ST05 candidate with the historical v1 fields.

    ``surface_view`` is the solver's normalized surface view carrying
    ``center_base`` (volume center in container axes), ``yaw`` (map yaw)
    and ``inner_size``. Returns ``candidate`` with ``center_local``,
    ``center_base`` (per-candidate container/rotated box center) and
    ``yaw`` added — exactly the pre-ST05 encoding, derived from the
    container-center truth instead of being computed inline.
    """
    center = candidate["center_container"]
    box_yaw = float(candidate["box_yaw"])
    map_center = surface_view["center_base"]
    map_yaw = float(surface_view.get("yaw", 0.0))
    lz = float(center[2]) - float(map_center[2])
    bx = map_center[0] + math.cos(map_yaw) * center[0] - math.sin(map_yaw) * center[1]
    by = map_center[1] + math.sin(map_yaw) * center[0] + math.cos(map_yaw) * center[1]
    candidate["center_local"] = [float(center[0]), float(center[1]), lz]
    candidate["center_base"] = [bx, by, map_center[2] + lz]
    candidate["yaw"] = map_yaw + box_yaw
    return candidate
