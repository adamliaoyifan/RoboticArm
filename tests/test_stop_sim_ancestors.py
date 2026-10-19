"""Stopper fallback must not terminate its rehearsal supervisor.

Mock discovery so this test can never target a real ROS/simulation process.
The discovered PID is a disposable parent subprocess owned by this test.
"""
import os
from pathlib import Path
import subprocess
import sys


def test_resource_path_match_does_not_kill_stopper_parent(tmp_path):
    discovery = tmp_path/'pgrep'
    discovery.write_text('''#!/bin/bash
case "$*" in
  *"/install"*) printf '%s /tmp/install/luggage_perception/lib/resource-reader\n' "$STOPPER_TEST_PID" ;;
  *) exit 1 ;;
esac
''')
    discovery.chmod(0o755)
    parent = tmp_path/'parent.py'
    parent.write_text('''import os, subprocess, sys
env = dict(os.environ, STOPPER_TEST_PID=str(os.getpid()))
result = subprocess.run([sys.argv[1], '--pidfile', sys.argv[2]], env=env,
                        capture_output=True, text=True)
print(result.stdout, end='')
print('PARENT_SURVIVED', flush=True)
raise SystemExit(result.returncode)
''')
    stopper = Path(__file__).resolve().parents[1]/'scripts/stop_sim.sh'
    result = subprocess.run([sys.executable,str(parent),str(stopper),str(tmp_path/'absent.pid')],
        env=dict(os.environ,PATH=str(tmp_path)+os.pathsep+os.environ['PATH']),
        capture_output=True,text=True,timeout=15)
    assert result.returncode == 0, result.stderr
    assert 'PARENT_SURVIVED' in result.stdout
    assert 'no leftover stack' in result.stdout
