"""Locate the frozen planning-contract manifest.

Installed copies live in the luggage_planning share tree. A source checkout
falls back to ``config/planning_contract`` beside this package.
"""

from pathlib import Path

_FILENAME = "resolved_planning_contract.json"


def resolved_planning_contract_path():
    try:
        from ament_index_python.packages import get_package_share_directory
        installed = (Path(get_package_share_directory("luggage_planning"))
                     / "config" / "planning_contract" / _FILENAME)
        if installed.is_file():
            return installed
    except Exception:
        pass
    return (Path(__file__).resolve().parents[2] / "config" / "planning_contract"
            / _FILENAME)
