"""Shared pytest fixtures for the ST09-B route test battery.

Keeps the unified manifest entry ``python3 -m pytest tests/st09_parallel/B``
working with or without an install overlay: the repo root (for
``st09_contracts``) and the src tree (for ``luggage_planning``) are both
put on ``sys.path`` idempotently. ROS-dependent tests skip loudly instead
of failing collection.
"""

from __future__ import division

import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
for _path in (str(_REPO_ROOT), str(_REPO_ROOT / "src" / "luggage_planning")):
    if _path not in sys.path:
        sys.path.insert(0, _path)
