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
    """Return ``{joint: (lower, upper)}`` from URDF text."""
    limits = {}
    for name in names:
        block = re.search(
            r'<joint\s+name="%s"[^>]*>.*?</joint>' % re.escape(name),
            urdf_text, re.S)
        limit = re.search(r"<limit[^>]*>", block.group(0)) if block else None
        if not limit:
            continue
        lower = re.search(r'lower="([^"]+)"', limit.group(0))
        upper = re.search(r'upper="([^"]+)"', limit.group(0))
        if lower and upper:
            limits[name] = (float(lower.group(1)), float(upper.group(1)))
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
    for name, value in zip(names, goal):
        if name not in limits:
            outside.append("%s: no URDF limit found" % name)
            continue
        lower, upper = limits[name]
        if value < lower or value > upper:
            outside.append("%s=%.6f outside [%.4f, %.4f]" % (
                name, value, lower, upper))
    checks.append({"check": "urdf_limits", "ok": not outside,
                   "detail": outside})

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
    elif status != "queried":
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
