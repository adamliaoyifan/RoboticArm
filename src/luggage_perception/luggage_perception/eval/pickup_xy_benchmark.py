"""Offline pickup XY benchmark against manual RGB labels.

This module is deliberately ROS-free. Bag replay/extraction produces a JSON
candidate file; manual labeling produces a JSON labels file. The scorer only
decides whether a strategy's world-XY contact point is better than the current
PCA center under the acceptance rule documented for real deployment.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from statistics import median
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple


BASELINE_STRATEGY = "pca_center"
DEFAULT_STRATEGIES = (
    BASELINE_STRATEGY,
    "yolo_bbox_center_top_plane",
    "lid_inlier_center",
    "robust_blended_center",
)


@dataclass(frozen=True)
class Candidate:
    frame_id: str
    bag_path: str
    stamp: float
    strategy: str
    xy: Tuple[float, float]


@dataclass(frozen=True)
class Label:
    frame_id: str
    bag_path: str
    stamp: float
    xy: Tuple[float, float]
    pixel: Optional[Tuple[float, float]] = None


def _as_xy(value: Sequence[float], field: str) -> Tuple[float, float]:
    if len(value) < 2:
        raise ValueError("%s must contain at least x,y" % field)
    x = float(value[0])
    y = float(value[1])
    if not (math.isfinite(x) and math.isfinite(y)):
        raise ValueError("%s must be finite" % field)
    return x, y


def _label_key(record: Mapping[str, object]) -> str:
    if record.get("frame_id"):
        return str(record["frame_id"])
    bag_path = str(record.get("bag_path") or "")
    stamp = float(record.get("stamp") or 0.0)
    return "%s@%.9f" % (bag_path, stamp)


def load_labels(path: Path) -> Dict[str, Label]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    records = payload["labels"] if isinstance(payload, dict) else payload
    labels = {}
    for record in records:
        frame_id = _label_key(record)
        if "suction_safe_lid_center_world_xy" in record:
            xy = _as_xy(record["suction_safe_lid_center_world_xy"], "label xy")
        else:
            xy = _as_xy(record["world_xy"], "label world_xy")
        pixel = None
        if record.get("suction_safe_lid_center_pixel") is not None:
            pixel = _as_xy(
                record["suction_safe_lid_center_pixel"], "label pixel")
        labels[frame_id] = Label(
            frame_id=frame_id,
            bag_path=str(record.get("bag_path") or ""),
            stamp=float(record.get("stamp") or 0.0),
            xy=xy,
            pixel=pixel,
        )
    return labels


def load_candidates(path: Path) -> List[Candidate]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    records = payload["frames"] if isinstance(payload, dict) else payload
    candidates = []
    for record in records:
        frame_id = _label_key(record)
        strategies = record.get("strategies") or {}
        for strategy, data in strategies.items():
            if data is None:
                continue
            if isinstance(data, Mapping):
                xy_value = data.get("world_xy") or data.get("xy")
            else:
                xy_value = data
            candidates.append(Candidate(
                frame_id=frame_id,
                bag_path=str(record.get("bag_path") or ""),
                stamp=float(record.get("stamp") or 0.0),
                strategy=str(strategy),
                xy=_as_xy(xy_value, "candidate %s xy" % strategy),
            ))
    return candidates


def _percentile(values: Sequence[float], pct: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(float(v) for v in values)
    rank = (len(ordered) - 1) * float(pct) / 100.0
    lo = int(math.floor(rank))
    hi = int(math.ceil(rank))
    if lo == hi:
        return ordered[lo]
    weight = rank - lo
    return ordered[lo] * (1.0 - weight) + ordered[hi] * weight


def score_candidates(
    labels: Mapping[str, Label],
    candidates: Iterable[Candidate],
    baseline_strategy: str = BASELINE_STRATEGY,
) -> Mapping[str, object]:
    by_frame_strategy: Dict[Tuple[str, str], Candidate] = {}
    for candidate in candidates:
        by_frame_strategy[(candidate.frame_id, candidate.strategy)] = candidate

    baseline_errors = {}
    for frame_id, label in labels.items():
        baseline = by_frame_strategy.get((frame_id, baseline_strategy))
        if baseline is not None:
            baseline_errors[frame_id] = _distance_m(baseline.xy, label.xy)

    strategies = sorted({strategy for _, strategy in by_frame_strategy})
    results = {}
    for strategy in strategies:
        rows = []
        improved = 0
        comparable = 0
        for frame_id, label in labels.items():
            candidate = by_frame_strategy.get((frame_id, strategy))
            if candidate is None:
                continue
            error = _distance_m(candidate.xy, label.xy)
            baseline_error = baseline_errors.get(frame_id)
            if baseline_error is not None and strategy != baseline_strategy:
                comparable += 1
                if error < baseline_error:
                    improved += 1
            rows.append({
                "frame_id": frame_id,
                "bag_path": label.bag_path or candidate.bag_path,
                "stamp": label.stamp or candidate.stamp,
                "error_m": error,
                "baseline_error_m": baseline_error,
            })
        errors = [row["error_m"] for row in rows]
        results[strategy] = {
            "sample_count": len(errors),
            "median_error_m": median(errors) if errors else None,
            "p95_error_m": _percentile(errors, 95.0),
            "improved_fraction_vs_pca": (
                float(improved) / float(comparable) if comparable else None
            ),
            "outliers": [
                row for row in sorted(
                    rows, key=lambda item: item["error_m"], reverse=True)
                if row["error_m"] > 0.060
            ],
        }
    return {
        "baseline_strategy": baseline_strategy,
        "label_count": len(labels),
        "results": results,
    }


def acceptance(summary: Mapping[str, object], strategy: str) -> Mapping[str, object]:
    result = summary["results"].get(strategy)
    if not result:
        return {"strategy": strategy, "outcome": "not_evaluated"}
    sample_count = int(result["sample_count"])
    median_error = result["median_error_m"]
    p95_error = result["p95_error_m"]
    improved = result["improved_fraction_vs_pca"]
    measurable = (
        sample_count > 0
        and median_error is not None
        and p95_error is not None
        and improved is not None
    )
    if not measurable:
        return {"strategy": strategy, "outcome": "not_evaluated"}
    passed = (
        float(median_error) <= 0.030
        and float(p95_error) <= 0.060
        and float(improved) >= 0.80
    )
    return {
        "strategy": strategy,
        "outcome": "pass" if passed else "fail",
        "sample_count": sample_count,
        "median_error_m": median_error,
        "p95_error_m": p95_error,
        "improved_fraction_vs_pca": improved,
    }


def _distance_m(left: Sequence[float], right: Sequence[float]) -> float:
    return math.hypot(float(left[0]) - float(right[0]),
                      float(left[1]) - float(right[1]))


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--labels", required=True, type=Path)
    parser.add_argument("--candidates", required=True, type=Path)
    parser.add_argument(
        "--strategy", default="robust_blended_center",
        help="Strategy to evaluate against the acceptance gate.")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)

    summary = dict(score_candidates(
        load_labels(args.labels), load_candidates(args.candidates)))
    summary["acceptance"] = acceptance(summary, args.strategy)
    text = json.dumps(summary, indent=2, sort_keys=True)
    if args.output:
        args.output.write_text(text + "\n", encoding="utf-8")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
