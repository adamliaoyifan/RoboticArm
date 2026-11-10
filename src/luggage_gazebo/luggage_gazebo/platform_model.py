"""Bind the pickup platform's rendered and physical box to scene dimensions."""

import math
import xml.etree.ElementTree as ET


def platform_sdf(template_xml, size):
    """Return a private SDF with bottom-centred visual/collision geometry.

    The scene config also drives platform-top TF and box spawning. Loading the
    stock model dimensions independently can leave static pickup boxes floating.
    This function changes simulation geometry only, never perception inputs.
    """
    dimensions = tuple(float(value) for value in size)
    if len(dimensions) != 3 or not all(
            math.isfinite(value) and value > 0 for value in dimensions):
        raise ValueError("platform size must contain three positive finite values")
    root = ET.fromstring(template_xml)
    link = root.find("model/link[@name='platform_link']")
    if link is None:
        raise ValueError("pickup platform template has no platform_link")
    for kind in ("visual", "collision"):
        parts = link.findall(kind)
        if len(parts) != 1:
            raise ValueError("pickup platform requires exactly one " + kind)
        part = parts[0]
        box_size = part.find("geometry/box/size")
        pose = part.find("pose")
        if box_size is None or pose is None:
            raise ValueError("pickup platform requires a posed box " + kind)
        box_size.text = " ".join(format(value, '.17g') for value in dimensions)
        pose.text = "0 0 " + format(dimensions[2] / 2, '.17g') + " 0 0 0"
    return ET.tostring(root, encoding="unicode")
