#!/usr/bin/env python3
"""Parsers and the pre-motion gate for the static-box contact diagnostic.

The gz Contact system publishes only when a sensor collision has contacts
(ign-gazebo6 Contact.cc). An empty contact file is therefore not, by itself,
evidence that nothing touched. ``capture_ready`` is what the driver checks
before the first certified leg. ``absence_is_evidence`` is what the offline
analysis is allowed to conclude afterwards.
"""
import json
import os
import time
import xml.etree.ElementTree as ET

FOLLOW_FRESH_SEC = 1.5
RECORDER_FRESH_SEC = 3.0


def brace_blocks(text, name):
    """Bodies of ``name { ... }`` blocks, brace-matched."""
    out = []
    i = 0
    token = name
    while True:
        j = text.find(token, i)
        if j < 0:
            return out
        if j > 0 and (text[j - 1].isalnum() or text[j - 1] == "_"):
            i = j + 1
            continue
        k = j + len(token)
        while k < len(text) and text[k] in " \t":
            k += 1
        if k >= len(text) or text[k] != "{":
            i = j + 1
            continue
        depth = 0
        for p in range(k, len(text)):
            if text[p] == "{":
                depth += 1
            elif text[p] == "}":
                depth -= 1
                if depth == 0:
                    out.append(text[k + 1:p])
                    i = p + 1
                    break
        else:
            return out


def _num(body, key):
    import re
    # ``nsec`` contains the letters ``sec``; the key must not match inside it.
    # A missing field is the protobuf default 0 (gz omits sec when it is 0).
    match = re.search(
        r"(?<![A-Za-z_])%s:\s*([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)" % key,
        body)
    return float(match.group(1)) if match else 0.0


def _header_stamp(text):
    headers = brace_blocks(text, "header")
    if not headers:
        return None
    stamps = brace_blocks(headers[0], "stamp")
    if not stamps:
        return None
    return _num(stamps[0], "sec") + _num(stamps[0], "nsec") * 1e-9


def parse_contacts(text):
    """One gz Contacts text message -> list of pair records."""
    stamp = _header_stamp(text)
    rows = []
    for body in brace_blocks(text, "contact"):
        entities = brace_blocks(body, "collision1") + brace_blocks(body, "collision2")
        # collision1/collision2 are singular; brace_blocks finds those names.
        names = []
        for ent in entities:
            import re
            found = re.findall(r'name:\s*"([^"]*)"', ent)
            names.append(found[0] if found else "")
        if len(names) < 2:
            import re
            names = re.findall(r'name:\s*"([^"]*)"', body)[:2]
        depths = []
        import re
        for raw in re.findall(
                r"depth:\s*([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)", body):
            depths.append(float(raw))
        force = 0.0
        for wrench in brace_blocks(body, "force"):
            fx, fy, fz = _num(wrench, "x"), _num(wrench, "y"), _num(wrench, "z")
            force = max(force, (fx * fx + fy * fy + fz * fz) ** 0.5)
        rows.append({
            "stamp": stamp,
            "a": names[0] if names else "",
            "b": names[1] if len(names) > 1 else "",
            "depth_m": max(depths) if depths else 0.0,
            "force_n": force,
        })
    return rows


def parse_pose_v(text):
    """One gz Pose_V text message -> (stamp, [(name, xyz, quat), ...])."""
    stamp = _header_stamp(text)
    rows = []
    for body in brace_blocks(text, "pose"):
        import re
        named = re.search(r'name:\s*"([^"]*)"', body)
        if not named:
            continue
        pos = brace_blocks(body, "position")
        ori = brace_blocks(body, "orientation")
        if not pos or not ori:
            continue
        rows.append((
            named.group(1),
            (_num(pos[0], "x"), _num(pos[0], "y"), _num(pos[0], "z")),
            (_num(ori[0], "x"), _num(ori[0], "y"), _num(ori[0], "z"),
             _num(ori[0], "w")),
        ))
    return stamp, rows


def topic_has_publisher(topic_info):
    text = topic_info or ""
    lowered = text.lower()
    if "no publisher" in lowered or "publishers: none" in lowered:
        return False
    return "publisher" in lowered


def _fresh(path, now, limit):
    try:
        return now - os.path.getmtime(path) <= limit and os.path.getsize(path) > 0
    except OSError:
        return False


def capture_ready(follow_trace, topic_info, box_sdf_path, robot_urdf_path,
                  recorder_status_path, now=None):
    """Each check is content. All must pass before motion."""
    now = time.time() if now is None else float(now)
    checks = []

    def add(name, ok, detail):
        checks.append({"check": name, "ok": bool(ok), "detail": detail})

    follow_ok = False
    detail = "missing"
    if follow_trace and _fresh(follow_trace, now, FOLLOW_FRESH_SEC):
        try:
            lines = open(follow_trace, encoding="utf-8").read().strip().splitlines()[-20:]
            oks = [json.loads(line) for line in lines if line.strip()]
            follow_ok = any(
                row.get("ok") and len(row.get("box_xyz") or []) == 3 for row in oks)
            detail = "%d/%d recent ticks ok, age %.2fs" % (
                sum(bool(r.get("ok")) for r in oks), len(oks),
                now - os.path.getmtime(follow_trace))
        except (OSError, json.JSONDecodeError, IndexError) as exc:
            detail = str(exc)
    add("follow_trace_live", follow_ok, detail)

    add("contact_topic_publisher", topic_has_publisher(topic_info),
        (topic_info or "")[:180])

    sdf_ok = False
    sdf_detail = "missing"
    if box_sdf_path and os.path.isfile(box_sdf_path):
        text = open(box_sdf_path, encoding="utf-8").read()
        sdf_ok = ("<static>true</static>" in text
                  and 'name="body_collision"' in text
                  and 'type="contact"' in text
                  and "<collision>body_collision</collision>" in text)
        sdf_detail = "static+sensor" if sdf_ok else "sdf is not the static sensed carry model"
    add("box_sdf_static_with_sensor", sdf_ok, sdf_detail)

    urdf_ok = False
    urdf_detail = "missing"
    if robot_urdf_path and os.path.isfile(robot_urdf_path):
        try:
            root = ET.parse(robot_urdf_path).getroot()
            counts = {}
            for link in root.findall("link"):
                n = len(link.findall("collision"))
                if n:
                    counts[link.get("name")] = n
            urdf_ok = counts.get("suction_panel", 0) > 0 and counts.get("elfin_link2", 0) > 0
            urdf_detail = "collision links: %s" % ",".join(sorted(counts))
        except ET.ParseError as exc:
            urdf_detail = str(exc)
    add("robot_urdf_collisions", urdf_ok, urdf_detail)

    rec_ok = False
    rec_detail = "missing"
    if recorder_status_path and _fresh(recorder_status_path, now, RECORDER_FRESH_SEC):
        try:
            status = json.loads(open(recorder_status_path, encoding="utf-8").read())
            rec_ok = bool(status.get("alive")) and int(status.get("pose_messages", 0)) > 0
            rec_detail = "pose_messages=%s contact_messages=%s" % (
                status.get("pose_messages"), status.get("contact_messages"))
        except (OSError, json.JSONDecodeError, TypeError, ValueError) as exc:
            rec_detail = str(exc)
    add("pose_recorder_live", rec_ok, rec_detail)

    return all(item["ok"] for item in checks), {"checks": checks}


def classify_pair(name_a, name_b, box_token="pickup_box"):
    """Which body the box is touching, or ``not_box``."""
    a = name_a or ""
    b = name_b or ""
    if box_token in a and box_token in b:
        return "box_box"
    if box_token in a:
        other = b
    elif box_token in b:
        other = a
    else:
        return "not_box"
    if "suction_panel" in other or "suction_contact" in other:
        return "suction_interface"
    robot = ("elfin_link", "elfin_base", "camera", "mid360", "livox", "eef_")
    if any(tok in other for tok in robot):
        return "robot_body"
    return "other"


def absence_is_evidence(saw_contact, publisher_ok, recorder_covered,
                        panel_inside_box):
    """Whether "no contact of a class" may be stated as a finding.

    A contact seen anywhere in the run shows the sensor path works, so a
    later gap is evidence. With no contact at all, a gap is evidence only
    when the recorder and the publisher were alive AND the suction frame
    was outside the box bounds (the bodies were not in a position where
    contact was expected). A frame inside the box with zero messages is
    inconclusive: the plugin does not publish empty contact lists.
    """
    if saw_contact and recorder_covered:
        return True
    if publisher_ok and recorder_covered and not panel_inside_box and not saw_contact:
        return True
    return False
