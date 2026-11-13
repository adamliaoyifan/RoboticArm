"""Run source tests on top of the pinned ROS/generated-interface installation."""
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
os.environ['PYTEST_DISABLE_PLUGIN_AUTOLOAD'] = '1'
# Prepend application sources without replacing the ROS Python search path.
# Production commands never call this test-only source override.
for package in ('luggage_description', 'luggage_mapping', 'luggage_perception',
                'luggage_packing', 'luggage_planning', 'luggage_gazebo'):
    sys.path.insert(0, str(ROOT/'src'/package))
    # Focused cross-package fixtures are imported by several integration
    # tests. Keep them available without changing installed ROS interfaces.
    test_directory = ROOT/'src'/package/'test'
    if test_directory.is_dir():
        sys.path.insert(0, str(test_directory))

import pytest

if __name__ == '__main__':
    raise SystemExit(pytest.main(sys.argv[1:]))
