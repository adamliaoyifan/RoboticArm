"""Synthetic top-center grasp for the placement scan.

``T_tcp_box`` is Rx(pi) and the box center sits ``height/2`` along the
tool +Z. The contact face is then on the TCP and the payload hangs on
the tool-down side. Translation comes from this box's height. A recorded
bias from one historical suitcase is not copied onto other sizes.
``register_attach_from_request`` inverts the returned grasp back to that
attachment.
"""

from __future__ import division

import math


def synthetic_top_grasp(size, offset=None):
    """Return ``(position, quaternion_xyzw)`` of the TCP in the box frame."""
    from luggage_planning.planning_contract.attachment import compose_transform, invert_transform

    half_height = 0.5 * float(size[2])
    grasp_t, grasp_q = invert_transform(
        (0.0, 0.0, half_height), (1.0, 0.0, 0.0, 0.0))
    if offset:
        yaw = float(offset["yaw_rad"])
        half = 0.5 * yaw
        grasp_t, grasp_q = compose_transform(
            (float(offset["xy_m"][0]), float(offset["xy_m"][1]),
             float(offset["z_m"])),
            (0.0, 0.0, math.sin(half), math.cos(half)),
            grasp_t, grasp_q)
    return [float(v) for v in grasp_t], [float(v) for v in grasp_q]
