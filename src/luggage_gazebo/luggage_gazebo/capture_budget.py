"""Continuous-bag budget for one ST09 live run.

The 2026-10-05 single-box bag stored 51.176 s of full-rate color, depth
and instance mask and already held 1.13 GiB. A 1200 s session at that rate
cannot stay under the frozen 1 GiB cap. Fat sensor topics are therefore
republished at ``THROTTLE_PERIOD_S`` and only those copies are recorded.
The original full-rate bag is left in place; this module does not delete it.
"""
from __future__ import annotations

# Publisher verified live 2026-10-08 (handoff_verify01): this topic has one
# publisher; the controller-manager-namespaced /S20/... name has none.
CONTROLLER_STATE_TOPIC = "/elfin_arm_controller/controller_state"

RUN_BUDGET_BYTES = 1024 ** 3
SESSION_WORST_S = 1200.0
THROTTLE_PERIOD_S = 3.0

# Payload bytes and message counts from
# docs/status/evidence/frame_migration/ST09-parallel-I-s6-readiness/7ec5c40/single_box/bag
SAMPLE_DURATION_S = 51175718448 / 1e9
SAMPLE_PAYLOAD = {
    "/clock": (572832, 47736),
    "/joint_states": (1507952, 4772),
    # 50 Hz measured on the live topic. Size is a serialised six-joint
    # message (744 B) plus header, rounded up to 900 B; the bag has not yet
    # recorded this topic, so replace with the measured size afterwards.
    CONTROLLER_STATE_TOPIC: (2303100, 2559),
    "/luggage/cargo_map/surface_2d": (63560, 1),
    "/luggage/cargo_map/surface_v2": (49492, 1),
    "/luggage/current_box": (424, 4),
    "/luggage/perception/size_eval/spawned": (548, 1),
    "/luggage/preprocessed/camera/color/camera_info": (277632, 723),
    "/luggage/preprocessed/camera/color/image": (497024568, 719),
    "/luggage/preprocessed/camera/depth/image": (333210456, 723),
    "/luggage/semantic/cargo_points": (25071330, 708),
    "/luggage/semantic/cargo_points_untracked": (12688812, 705),
    "/luggage/semantic/instance_mask": (330906096, 718),
    "/luggage_detector/stream_stats_json": (4221444, 703),
    "/tf": (560952, 954),
    "/vacuum/events_json": (120, 1),
    "/vacuum/state": (63052, 1433),
}

# input topic, throttled topic recorded in the bag, ROS type
FAT_TOPICS = (
    ("/luggage/preprocessed/camera/color/image",
     "/luggage/capture/color/image", "sensor_msgs/msg/Image"),
    ("/luggage/preprocessed/camera/depth/image",
     "/luggage/capture/depth/image", "sensor_msgs/msg/Image"),
    ("/luggage/semantic/instance_mask",
     "/luggage/capture/instance_mask", "sensor_msgs/msg/Image"),
    ("/luggage/semantic/cargo_points",
     "/luggage/capture/cargo_points", "sensor_msgs/msg/PointCloud2"),
    ("/luggage/semantic/cargo_points_untracked",
     "/luggage/capture/cargo_points_untracked", "sensor_msgs/msg/PointCloud2"),
)

LIGHT_TOPICS = (
    "/clock",
    "/tf",
    "/joint_states",
    # Handoff evidence: controller reference vs measured, across the window
    # between one leg's success result and the next leg's submission.
    CONTROLLER_STATE_TOPIC,
    "/vacuum/state",
    "/vacuum/events_json",
    "/luggage/current_box",
    "/luggage/finalized_models",
    "/luggage/perception/size_eval/spawned",
    "/luggage/cargo_map/surface_2d",
    "/luggage/cargo_map/surface_v2",
    "/luggage/preprocessed/camera/color/camera_info",
    "/luggage_detector/stream_stats_json",
    "/motion_planner/last_boundary",
)


def bag_topics():
    """Topics the continuous recorder may subscribe to."""
    return list(LIGHT_TOPICS) + [item[1] for item in FAT_TOPICS]


def worst_case_bytes(session_s=SESSION_WORST_S, period_s=THROTTLE_PERIOD_S):
    """Upper bound using the measured frame sizes and the light-topic rate."""
    fat_names = {item[0] for item in FAT_TOPICS}
    light = 0
    for name, (payload, _count) in SAMPLE_PAYLOAD.items():
        if name not in fat_names:
            light += payload
    light_bytes = light / SAMPLE_DURATION_S * float(session_s)
    samples = float(session_s) / float(period_s)
    fat_bytes = 0.0
    for name, _out, _type in FAT_TOPICS:
        payload, count = SAMPLE_PAYLOAD[name]
        fat_bytes += (payload / count) * samples
    return int(light_bytes + fat_bytes)


def within_budget(session_s=SESSION_WORST_S, period_s=THROTTLE_PERIOD_S):
    return worst_case_bytes(session_s, period_s) < RUN_BUDGET_BYTES
