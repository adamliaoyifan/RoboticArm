#!/usr/bin/env python3
"""Bounded cargo-join and epoch records.

These records name stamps, generations, counts, and source transitions.
They do not change association, support fitting, or pick authorization.
"""

from __future__ import division


def epochs_match(acquire_generation, acquire_instance_id,
                live_generation, live_instance_id):
    """True when a join may commit into the tracker.

    Identity is frozen when the join is taken. A later current-box change
    makes this false, and the caller must drop the join without touching
    the track. No epoch on either side (generation 0, empty id) matches,
    so a host that never publishes ``current_box`` is unchanged.
    """
    return (
        int(acquire_generation or 0), str(acquire_instance_id or "")
    ) == (
        int(live_generation or 0), str(live_instance_id or "")
    )


def stamp_sec(stamp):
    """Seconds from a float, a ROS stamp, or None."""
    if stamp is None:
        return None
    if isinstance(stamp, (int, float)):
        return float(stamp)
    sec = getattr(stamp, "sec", None)
    if sec is None:
        return None
    nsec = getattr(stamp, "nanosec", 0)
    return float(sec) + 1e-9 * float(nsec or 0)


class JoinBoundaryTrace(object):
    """Ring of epoch and join events for one filter process."""

    def __init__(self, maxlen=128):
        self.maxlen = max(1, int(maxlen))
        self._events = []
        self.epoch_stamp = None
        self.epoch_generation = 0
        self.epoch_instance_id = ""

    def note_epoch(self, generation, instance_id, ros_stamp, depth_stamp=None):
        """Record the moment a current-box epoch is accepted."""
        self.epoch_generation = int(generation or 0)
        self.epoch_instance_id = str(instance_id or "")
        self.epoch_stamp = stamp_sec(ros_stamp)
        event = {
            "kind": "epoch",
            "generation": int(self.epoch_generation),
            "instance_id": str(self.epoch_instance_id),
            "epoch_stamp": self.epoch_stamp,
            "depth_stamp_at_epoch": stamp_sec(depth_stamp),
        }
        self._append(event)
        return event

    def note_join(self, depth_stamp, mask_stamp=None, instance_stamp=None,
                  mask_nonzero=None, instance_nonzero=None,
                  cargo_instance_id=0, untracked_points=0,
                  measured_points=0, tracked_points_before=0,
                  tracked_points=0, pre_source="", post_source="",
                  acquire_generation=0, acquire_instance_id="",
                  commit_generation=0, commit_instance_id=""):
        """Record one processed depth/mask join and the tracker transition.

        ``pre_epoch_join`` is true when this join is committed under the
        accepted epoch but its depth stamp is strictly older than that
        epoch, or when the epoch changed while the join was in flight
        (acquired under one generation, committed under another).
        """
        acq = stamp_sec(depth_stamp)
        acquire_generation = int(acquire_generation or 0)
        commit_generation = int(commit_generation or 0)
        crossed = acquire_generation != commit_generation
        older = False
        if (self.epoch_stamp is not None and acq is not None
                and acq + 1e-9 < float(self.epoch_stamp)
                and commit_generation == int(self.epoch_generation)
                and int(self.epoch_generation) > 0):
            older = True
        event = {
            "kind": "join",
            "depth_stamp": acq,
            "mask_stamp": stamp_sec(mask_stamp),
            "instance_stamp": stamp_sec(instance_stamp),
            "join_stamp": acq,
            "mask_nonzero": (
                None if mask_nonzero is None else int(mask_nonzero)),
            "instance_nonzero": (
                None if instance_nonzero is None else int(instance_nonzero)),
            "cargo_instance_id": int(cargo_instance_id or 0),
            "untracked_points": int(untracked_points or 0),
            "measured_points": int(measured_points or 0),
            "tracked_points_before": int(tracked_points_before or 0),
            "tracked_points": int(tracked_points or 0),
            "pre_source": str(pre_source or ""),
            "post_source": str(post_source or ""),
            "acquire_generation": acquire_generation,
            "acquire_instance_id": str(acquire_instance_id or ""),
            "commit_generation": commit_generation,
            "commit_instance_id": str(commit_instance_id or ""),
            "epoch_stamp": self.epoch_stamp,
            "epoch_generation": int(self.epoch_generation),
            "pre_epoch_join": bool(older or crossed),
            "epoch_crossed_during_join": bool(crossed),
        }
        self._append(event)
        return event

    def snapshot(self):
        return [dict(row) for row in self._events]

    def _append(self, event):
        self._events.append(event)
        extra = len(self._events) - self.maxlen
        if extra > 0:
            del self._events[:extra]


def _post_epoch_joins(events):
    epochs = [row for row in events if row.get("kind") == "epoch"]
    joins = [row for row in events if row.get("kind") == "join"]
    if not epochs:
        return None, joins
    epoch = epochs[-1]
    generation = int(epoch.get("generation") or 0)
    post = [
        row for row in joins
        if int(row.get("commit_generation") or 0) == generation
    ]
    return epoch, post


def _selection_note(auth_reads):
    """How authorization's selected frame relates to the window it read."""
    reads = list(auth_reads or [])
    if not reads:
        return {"outcome": "not_evaluated", "reason": "no_auth_reads"}
    selected_sources = []
    hidden = None
    for read in reads:
        window = list(read.get("window") or [])
        chosen = [row for row in window if row.get("selected")]
        source = ""
        if chosen:
            source = str(chosen[-1].get("pca_source") or "")
        selected_sources.append(source)
        if source == "hold_track" and hidden is None:
            for row in window:
                if row.get("selected"):
                    continue
                if str(row.get("pca_source") or "") != "measure":
                    continue
                if not row.get("support_valid"):
                    continue
                hidden = {
                    "selected_stamp": chosen[-1].get("stamp") if chosen else None,
                    "measure_stamp": row.get("stamp"),
                    "measure_generation": row.get("generation"),
                }
                break
    if hidden is not None and all(
            source == "hold_track" for source in selected_sources):
        hidden["outcome"] = "fresh_measure_hidden_by_later_hold"
        hidden["n_reads"] = len(reads)
        return hidden
    if selected_sources and all(
            source == "hold_track" for source in selected_sources):
        return {
            "outcome": "selected_hold_only",
            "n_reads": len(reads),
        }
    if any(source == "measure" for source in selected_sources):
        return {"outcome": "selected_measure", "n_reads": len(reads)}
    return {
        "outcome": "not_evaluated",
        "reason": "no_selected_source",
        "n_reads": len(reads),
    }


def classify_hold_boundary(join_trace, auth_reads=None, mask_trace=None):
    """Name the earliest observed divergence. Empty input is not a cause.

    Order: in-flight or pre-epoch admission, then the first post-epoch
    join that is not a fresh measure, then a measure that authorization
    left behind a newer hold. Mask emptiness is reported only when no
    post-epoch join exists to explain the tracker.
    """
    events = list(join_trace or [])
    epoch, post = _post_epoch_joins(events)
    selection = _selection_note(auth_reads)
    if epoch is None and not post and selection["outcome"] == "not_evaluated":
        return {
            "outcome": "not_evaluated",
            "reason": "no_join_or_auth_trace",
            "selection": selection,
        }
    pre = [row for row in post if row.get("pre_epoch_join")]
    if pre:
        first = pre[0]
        return {
            "outcome": "pre_epoch_join_admitted",
            "depth_stamp": first.get("depth_stamp"),
            "epoch_stamp": first.get("epoch_stamp"),
            "acquire_generation": first.get("acquire_generation"),
            "commit_generation": first.get("commit_generation"),
            "epoch_crossed_during_join": bool(
                first.get("epoch_crossed_during_join")),
            "post_source": first.get("post_source"),
            "measured_points": first.get("measured_points"),
            "tracked_points": first.get("tracked_points"),
            "selection": selection,
        }
    seen_measure = 0
    for row in post:
        if (str(row.get("post_source") or "") == "measure"
                and int(row.get("measured_points") or 0) > 0):
            seen_measure += 1
            continue
        return {
            "outcome": "post_epoch_not_measure",
            "post_source": str(row.get("post_source") or ""),
            "pre_source": str(row.get("pre_source") or ""),
            "depth_stamp": row.get("depth_stamp"),
            "measured_points": int(row.get("measured_points") or 0),
            "tracked_points_before": int(row.get("tracked_points_before") or 0),
            "tracked_points": int(row.get("tracked_points") or 0),
            "mask_nonzero": row.get("mask_nonzero"),
            "prior_post_epoch_measures": int(seen_measure),
            "selection": selection,
        }
    if post:
        return {
            "outcome": "post_epoch_measure",
            "depth_stamp": post[-1].get("depth_stamp"),
            "measured_points": int(post[-1].get("measured_points") or 0),
            "selection": selection,
        }
    masks = []
    if epoch is not None and epoch.get("epoch_stamp") is not None:
        boundary = float(epoch["epoch_stamp"])
        masks = [
            row for row in (mask_trace or [])
            if row.get("stamp") is not None
            and float(row["stamp"]) + 1e-9 >= boundary
        ]
    if masks:
        nonzero = [
            row for row in masks
            if int(row.get("mask_nonzero") or 0) > 0
        ]
        if not nonzero:
            return {
                "outcome": "post_epoch_mask_empty",
                "n_masks": len(masks),
                "selection": selection,
            }
        return {
            "outcome": "mask_without_join",
            "n_masks": len(masks),
            "n_nonzero_masks": len(nonzero),
            "selection": selection,
        }
    if selection["outcome"] == "fresh_measure_hidden_by_later_hold":
        result = dict(selection)
        result["selection"] = selection
        return result
    return {
        "outcome": "not_evaluated",
        "reason": "no_post_epoch_join",
        "selection": selection,
    }
