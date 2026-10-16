"""A completed rehearsal must never stop a replacement simulation session."""
import os
from pathlib import Path
import signal
import subprocess
import sys


def test_only_owned_group_stopped_even_with_foreign_pidfile(tmp_path):
    repo = Path(__file__).resolve().parents[3]
    own = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'],
                           start_new_session=True)
    foreign = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'],
                               start_new_session=True)
    pidfile = tmp_path / 'sim.pid'
    pidfile.write_text(str(foreign.pid))
    env = dict(os.environ, ELFIN_SIM_PIDFILE=str(pidfile), WAIT_INT='1', WAIT_TERM='1')
    try:
        command = ['bash', str(repo / 'scripts/stop_sim.sh'), '--owned-pgid', str(own.pid)]
        subprocess.run(command, env=env, check=True, capture_output=True, timeout=10)
        own.wait(timeout=2)
        assert foreign.poll() is None
        assert pidfile.read_text() == str(foreign.pid)
        # The old leader is gone. Repeating cleanup must still not fall back.
        subprocess.run(command, env=env, check=True, capture_output=True, timeout=10)
        assert foreign.poll() is None
    finally:
        for process in (own, foreign):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=2)
