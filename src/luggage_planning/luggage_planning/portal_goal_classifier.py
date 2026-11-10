"""Classify why a PORTAL_TRANSIT goal was refused (MoveIt -27).

Input is one record written by ``MoveItEdgeAdapter._keep_portal_failure``
plus the URDF joint limits. The verdict is ordered; the first matching
category wins and every check that ran is listed in ``checks``.

Categories
----------
GOAL_CONSTRAINT_DEFECT      constraints do not describe goal_q (name, value,
                            or non-positive tolerance)
GOAL_OUTSIDE_URDF_LIMITS    a goal joint lies outside the URDF limit
GOAL_IN_COLLISION           the certifier validity query on the pre-call
                            scene reports COLLISION (contacts listed)
CERTIFIER_VALID_MOVEIT_INVALID  goal passes limits and certifier validity yet
                            MoveIt refused it: ACM or scene disagreement,
                            or a MoveIt-side limit not in the URDF
NOT_EVALUATED               validity query unavailable; nothing concluded

URDF limits are not proof of MoveIt's effective limits (joint_limits.yaml
may narrow them); that case lands in CERTIFIER_VALID_MOVEIT_INVALID.
"""

import re

JOINT_NAMES = tuple("elfin_joint%d" % index for index in range(1, 7))


def urdf_joint_limits(urdf_text, names=JOINT_NAMES):
    """Return ``{joint: (lower, upper)}`` parsed as XML.

    A joint with no ``<limit>``, a non-numeric bound or lower > upper is left
    out; the classifier then reports the limit check as not evaluated rather
    than as an out-of-limit goal. Unparseable XML returns ``{}``.
    """
    import xml.etree.ElementTree as ET

    try:
        try:
            root = ET.fromstring(urdf_text)
        except ET.ParseError:
            # A captured fragment without a <robot> root is still XML.
            root = ET.fromstring("<robot>%s</robot>" % urdf_text)
    except (ET.ParseError, TypeError, ValueError):
        return {}
    limits = {}
    for joint in root.iter("joint"):
        name = joint.get("name")
        if name not in names:
            continue
        limit = joint.find("limit")
        if limit is None:
            continue
        try:
            lower = float(limit.get("lower"))
            upper = float(limit.get("upper"))
        except (TypeError, ValueError):
            continue
        if lower == lower and upper == upper and lower <= upper:
            limits[name] = (lower, upper)
    return limits


def classify_portal_record(record, limits, names=JOINT_NAMES):
    goal = [float(v) for v in record["goal_q"]]
    checks = []

    defects = []
    constraints = {item["joint_name"]: item
                   for item in record.get("goal_constraints", [])}
    for name, value in zip(names, goal):
        item = constraints.get(name)
        if item is None:
            defects.append("%s: no constraint" % name)
            continue
        if abs(float(item["position"]) - value) > 1e-9:
            defects.append("%s: constraint %.9f != goal %.9f" % (
                name, float(item["position"]), value))
        if float(item["tolerance_above"]) <= 0 or \
                float(item["tolerance_below"]) <= 0:
            defects.append("%s: non-positive tolerance" % name)
    checks.append({"check": "goal_constraints", "ok": not defects,
                   "detail": defects})

    outside = []
    unknown = []
    for name, value in zip(names, goal):
        if name not in limits:
            unknown.append("%s: no URDF limit found" % name)
            continue
        lower, upper = limits[name]
        if value < lower or value > upper:
            outside.append("%s=%.6f outside [%.4f, %.4f]" % (
                name, value, lower, upper))
    checks.append({"check": "urdf_limits", "ok": not outside,
                   "evaluated": not unknown, "detail": outside,
                   "not_evaluated": unknown})

    validity = record.get("validity_on_submission_scene") or {}
    status = validity.get("status")
    reason = str(validity.get("reason", ""))
    checks.append({"check": "certifier_validity", "status": status,
                   "valid": validity.get("valid"), "reason": reason,
                   "contacts": validity.get("contacts", [])})

    if defects:
        category = "GOAL_CONSTRAINT_DEFECT"
    elif outside:
        category = "GOAL_OUTSIDE_URDF_LIMITS"
    elif unknown or status != "queried":
        # A joint whose limit is unknown cannot be called inside or outside.
        category = "NOT_EVALUATED"
    elif not validity.get("valid") and reason == "COLLISION":
        category = "GOAL_IN_COLLISION"
    elif validity.get("valid"):
        category = "CERTIFIER_VALID_MOVEIT_INVALID"
    else:
        category = "NOT_EVALUATED"
    return {"category": category, "checks": checks,
            "exact_sha256": record.get("exact_sha256"),
            "error_code": record.get("error_code")}
