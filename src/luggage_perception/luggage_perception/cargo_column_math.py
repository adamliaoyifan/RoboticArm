"""Column arithmetic shared by the full-grid and dirty-column projections.

Inputs are arrays with Z on axis 0. This module does not import either
projection module and does not accept a mapper.
"""

import numpy as np


def column_summary(grid, active, source, labels, resolution, inner_h):
    """Return height, clearance, state, known_ratio, confidence, semantic_label.

    ``grid`` / ``source`` / ``labels`` keep Z on axis 0. ``active`` is
    broadcastable to ``grid``. The remaining axes are the column index.
    """
    occupied = (grid == 2) & active
    observed = (grid != 0) & active
    unknown = (grid == 0) & active
    has_top = occupied.any(axis=0)
    top = grid.shape[0] - 1 - occupied[::-1].argmax(axis=0)
    height = np.where(has_top, np.minimum((top + 1) * resolution, inner_h), 0.)
    state = np.where(has_top, "occupied", np.where(unknown.any(axis=0), "unknown", "free"))
    top_source = np.take_along_axis(source, top[None, ...], axis=0)[0]
    confidence = np.where(
        has_top, np.where(top_source == 2, "geometry", "sensor"),
        np.where(state == "free", "sensor", "none"))
    count = np.asarray(active).sum(axis=0)
    ratio = np.divide(observed.sum(axis=0), count, out=np.zeros(count.shape), where=count != 0)
    valid = occupied & (labels > 0)
    semantic = np.zeros(occupied.shape[1:], dtype=np.int64)
    flat_valid = valid.reshape(valid.shape[0], -1)
    flat_labels = labels.reshape(labels.shape[0], -1)
    flat_semantic = semantic.reshape(-1)
    for index in np.flatnonzero(flat_valid.any(axis=0)):
        values = flat_labels[:, index][flat_valid[:, index]]
        unique, first, frequencies = np.unique(values, return_index=True, return_counts=True)
        winners = np.flatnonzero(frequencies == frequencies.max())
        flat_semantic[index] = unique[winners[np.argmin(first[winners])]]
    return dict(
        height=height, clearance=inner_h - height, state=state,
        known_ratio=ratio, confidence=confidence, semantic_label=semantic)
