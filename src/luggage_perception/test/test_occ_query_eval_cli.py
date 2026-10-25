"""Invalid workload requests are rejected before creating an evidence run."""
from pathlib import Path
import subprocess
import sys

import pytest


SCRIPT = Path(__file__).resolve().parents[3]/'scripts/g3/occ_query_eval.py'


@pytest.mark.parametrize('arguments', [
    ['--seeds', '0'], ['--seed-start', '-1'], ['--transactions', '0'],
    ['--distance-points', '-1'], ['--resolutions', '.03'],
    ['--resolutions', '.01,.01'], ['--boxes', '2'], ['--shapes', 'unknown'],
])
def test_invalid_matrix_creates_no_artifacts(tmp_path, arguments):
    out = tmp_path/'must_not_exist'
    result = subprocess.run([sys.executable, str(SCRIPT), 'oracle', '--out', str(out),
                             *arguments], capture_output=True, text=True)
    assert result.returncode == 2
    assert not out.exists()


@pytest.mark.parametrize('command', ['oracle', 'replay'])
def test_help_needs_no_ros_graph_or_product_import(command):
    result = subprocess.run([sys.executable, str(SCRIPT), command, '--help'],
                            capture_output=True, text=True)
    assert result.returncode == 0
