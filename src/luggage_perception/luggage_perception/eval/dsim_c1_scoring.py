"""DSI-C1 RGB-D join-gate scoring. Livox is reported separately, never here."""
from __future__ import division

import re


C1_LIMITS = {
    "d3_emission_over_rgb": 0.80,
    "d4_paired_depth_over_emitted": 0.95,
    "semantic_exact_join_over_depth": 0.95,
    "detector_support_depth_hit_over_joined": 0.95,
    "stale_drop_ratio_max": 0.05,
    "rate_hz_min": 13.5,
    "rate_hz_max": 16.5,
    "four_product_identity": 0.95,
}

FORBIDDEN_CAMERA_CLOUD_RES = (
    re.compile(r"^/camera/.*/points$"),
    re.compile(r"^/d435/points$"),
    re.compile(r"^/d555/.*/points$"),
    re.compile(r"^/luggage/preprocessed/camera/.*/points$"),
)

ALLOWED_POINTCLOUD_TOPICS = (
    "/livox/lidar",
    "/luggage/semantic/cargo_points",
    "/luggage/semantic/obstacle_points",
)


def camera_native_clouds(pc2_topics):
    found = []
    for line in pc2_topics or []:
        topic = str(line).split()[0]
        for pat in FORBIDDEN_CAMERA_CLOUD_RES:
            if pat.search(topic):
                found.append(topic)
                break
    return found


def parse_publisher_count(topic_info_text):
    for line in (topic_info_text or "").splitlines():
        if "Publisher count" in line:
            try:
                return int(line.split(":")[-1].strip())
            except ValueError:
                return None
    return None


def detector_hit_ratio(stream_stats):
    """same-stamp support-depth hits / joined cargo in the scored window."""
    n = 0
    hits = 0
    for rec in stream_stats or []:
        n += 1
        lookup = rec.get("raw_lookup") or {}
        if str(lookup.get("raw_lookup_status") or "") == "hit":
            hits += 1
        elif rec.get("raw_lookup_status") == "hit":
            hits += 1
    if n <= 0:
        return {"joined_cargo": 0, "hits": 0, "ratio": None}
    return {"joined_cargo": n, "hits": hits, "ratio": float(hits) / float(n)}


def score_c1(payload):
    """Return a decision-complete C1 verdict. Does not inspect Livox joins."""
    lim = C1_LIMITS
    failures = []
    rates = payload.get("rates") or {}
    identity = payload.get("identity") or {}
    d34 = payload.get("d34") or {}
    detector = detector_hit_ratio(payload.get("detector_stream_stats") or [])
    pc2 = payload.get("pointcloud2_topics") or []
    clouds = camera_native_clouds(pc2)
    clock_n = parse_publisher_count(payload.get("clock_info") or "")

    def _rate(name):
        rec = rates.get(name) or {}
        return rec.get("hz")

    color_hz = _rate("color")
    depth_hz = _rate("depth_mm")
    for label, hz in (("color", color_hz), ("adapted_depth", depth_hz)):
        if hz is None:
            failures.append("%s rate missing" % label)
        elif hz < lim["rate_hz_min"] or hz > lim["rate_hz_max"]:
            failures.append("%s rate %.3f Hz outside %.1f-%.1f" % (
                label, hz, lim["rate_hz_min"], lim["rate_hz_max"]))

    ident_ratio = identity.get("four_product_exact_ratio")
    if ident_ratio is None:
        failures.append("four-product identity missing")
    elif ident_ratio < lim["four_product_identity"]:
        failures.append("four-product identity %.3f < %.2f" % (
            ident_ratio, lim["four_product_identity"]))

    d3 = d34.get("d3_emission_over_rgb")
    if d3 is None:
        failures.append("D3 emission/colour missing")
    elif d3 < lim["d3_emission_over_rgb"]:
        failures.append("D3 emission/colour %.3f < %.2f" % (
            d3, lim["d3_emission_over_rgb"]))

    d4 = d34.get("d4_paired_depth_over_emitted")
    if d4 is None:
        failures.append("D4 paired depth/emitted missing")
    elif d4 < lim["d4_paired_depth_over_emitted"]:
        failures.append("D4 paired depth/emitted %.3f < %.2f" % (
            d4, lim["d4_paired_depth_over_emitted"]))

    filt = d34.get("d4_filter") or {}
    join_ratio = filt.get("exact_join_joined_over_depth")
    if join_ratio is None:
        failures.append("semantic exact-join ratio missing")
    elif join_ratio < lim["semantic_exact_join_over_depth"]:
        failures.append("semantic exact-join %.3f < %.2f" % (
            join_ratio, lim["semantic_exact_join_over_depth"]))

    stale = filt.get("stale_drop_ratio")
    if stale is None:
        failures.append("stale drop ratio missing")
    elif stale >= lim["stale_drop_ratio_max"]:
        failures.append("stale drop ratio %.3f >= %.2f" % (
            stale, lim["stale_drop_ratio_max"]))

    if detector["ratio"] is None:
        failures.append("detector joined cargo is zero")
    elif detector["ratio"] < lim["detector_support_depth_hit_over_joined"]:
        failures.append("detector support-depth hits %.3f < %.2f" % (
            detector["ratio"], lim["detector_support_depth_hit_over_joined"]))

    if clouds:
        failures.append("camera-native PointCloud2 present: %s" % (
            ", ".join(clouds)))

    if clock_n is None:
        failures.append("/clock publisher count missing")
    elif clock_n != 1:
        failures.append("/clock publisher count %s != 1" % clock_n)

    unexpected = []
    for line in pc2:
        topic = str(line).split()[0]
        if topic in ALLOWED_POINTCLOUD_TOPICS:
            continue
        if topic in clouds:
            continue
        unexpected.append(topic)

    return {
        "pass": not failures,
        "failures": failures,
        "limits": dict(lim),
        "color_hz": color_hz,
        "depth_mm_hz": depth_hz,
        "identity_ratio": ident_ratio,
        "d3_emission_over_rgb": d3,
        "d4_paired_depth_over_emitted": d4,
        "semantic_exact_join_over_depth": join_ratio,
        "stale_drop_ratio": stale,
        "detector": detector,
        "camera_native_clouds": clouds,
        "unexpected_pointclouds": unexpected,
        "clock_publishers": clock_n,
    }
