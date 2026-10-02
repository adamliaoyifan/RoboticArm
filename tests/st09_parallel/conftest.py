"""Shared pytest path setup for the ST09 parallel task directories.

Adds the workspace root (repo root) to sys.path so the ROS-free
``st09_contracts`` package resolves regardless of the install state.
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
