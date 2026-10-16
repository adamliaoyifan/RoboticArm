"""Prefer fresh measured work over same-object continuity, never authorize it."""
from dataclasses import dataclass


@dataclass(frozen=True)
class EstimateEvidence:
    stamp_ns: int
    generation: int
    instance_id: str
    source: str
    cargo_count: int


def retain_measured_pending(pending, incoming, *, now_ns, max_age_ns):
    """Keep one pending measurement only across an unambiguous held update.

    Missing identity, a different object, a new measurement, rejected/unknown
    source, multiple cargo detections, or expiry restores latest-only scheduling.
    The ordinary geometry, acquisition-time and authorization gates still run.
    """
    if pending is None or incoming is None:
        return False
    if (pending.source != 'measure' or incoming.source != 'hold_track'
            or pending.cargo_count != 1 or incoming.cargo_count > 1
            or pending.generation <= 0 or not pending.instance_id
            or (pending.generation, pending.instance_id) !=
               (incoming.generation, incoming.instance_id)):
        return False
    return (max_age_ns > 0 and pending.stamp_ns > 0
            and pending.stamp_ns <= incoming.stamp_ns <= now_ns
            and 0 <= now_ns - pending.stamp_ns <= max_age_ns)
