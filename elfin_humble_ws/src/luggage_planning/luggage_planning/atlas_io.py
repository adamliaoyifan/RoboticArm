#!/usr/bin/env python3
"""Deterministic file helpers for reachability atlas artifacts."""

from __future__ import annotations

import io
import os
import zipfile

import numpy as np


_ZIP_TIMESTAMP = (1980, 1, 1, 0, 0, 0)


def save_npz_deterministic(path: str, arrays: dict[str, object]) -> None:
    """Write an NPZ whose bytes depend only on sorted array names and values."""
    directory = os.path.dirname(path) or "."
    os.makedirs(directory, exist_ok=True)
    temporary = path + ".tmp"
    with zipfile.ZipFile(
            temporary, "w", compression=zipfile.ZIP_DEFLATED,
            compresslevel=9) as archive:
        for name in sorted(arrays):
            buffer = io.BytesIO()
            np.lib.format.write_array(
                buffer, np.asanyarray(arrays[name]), allow_pickle=False)
            info = zipfile.ZipInfo("%s.npy" % name, _ZIP_TIMESTAMP)
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = 0o600 << 16
            archive.writestr(info, buffer.getvalue(), compress_type=zipfile.ZIP_DEFLATED,
                             compresslevel=9)
    os.replace(temporary, path)


__all__ = ["save_npz_deterministic"]
