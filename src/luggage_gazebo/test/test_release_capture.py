import json
import sys
import time
import os
import subprocess

from luggage_gazebo.release_capture import record


def command(source):
    return [sys.executable, '-u', '-c', source]


def test_preserves_exact_bytes_and_join_offsets(tmp_path):
    manifest = record(tmp_path, {'poses': command("print('header { stamp { sec: 2 } }')")}, duration=2)
    raw = (tmp_path/'poses.raw').read_bytes()
    assert raw == b'header { stamp { sec: 2 } }\n'
    chunks = [json.loads(line) for line in (tmp_path/'chunks.jsonl').read_text().splitlines()]
    assert sum(row['length'] for row in chunks) == len(raw)
    assert chunks[0]['offset'] == 0
    assert chunks[0]['monotonic'] >= manifest['start_monotonic']
    assert manifest['streams']['poses']['eof']
    assert manifest['streams']['poses']['returncode'] == 0


def test_byte_limit_marks_incomplete_and_stops_producer(tmp_path):
    manifest = record(tmp_path, {'contacts': command("import os; os.write(1,b'x'*10000); import time; time.sleep(10)")},
                      duration=2, byte_limit=100)
    assert (tmp_path/'contacts.raw').stat().st_size == 100
    assert manifest['streams']['contacts']['limited']
    assert manifest['streams']['contacts']['returncode'] is not None
    assert not manifest['contact_absence_proven']


def test_silent_topic_has_deadline_and_no_absence_claim(tmp_path):
    start = time.monotonic()
    manifest = record(tmp_path, {'contacts': command('import time; time.sleep(10)')}, duration=.2)
    assert time.monotonic()-start < 3
    assert manifest['end_reason'] == 'duration'
    assert manifest['streams']['contacts']['bytes'] == 0
    assert manifest['streams']['contacts']['returncode'] is not None
    assert not manifest['contact_absence_proven']


def test_dead_reader_is_visible(tmp_path):
    manifest = record(tmp_path, {'contacts': command('import sys; sys.exit(4)')}, duration=2)
    assert manifest['streams']['contacts']['returncode'] == 4
    assert manifest['streams']['contacts']['eof']


def test_termination_flushes_manifest_and_reaps_child(tmp_path):
    child_pid = tmp_path/'child.pid'
    source = ("import os,time; from pathlib import Path; "
              f"Path({str(child_pid)!r}).write_text(str(os.getpid())); time.sleep(30)")
    parent = subprocess.Popen(command(
        'from luggage_gazebo.release_capture import record; '
        f'record({str(tmp_path)!r}, {{"contacts": {command(source)!r}}})'))
    try:
        deadline = time.monotonic()+3
        while not child_pid.exists() and time.monotonic() < deadline:
            time.sleep(.01)
        assert child_pid.exists()
        parent.terminate()
        assert parent.wait(timeout=5) == 0
        manifest = json.loads((tmp_path/'manifest.json').read_text())
        assert manifest['end_reason'] == 'signal'
        try:
            os.kill(int(child_pid.read_text()), 0)
        except ProcessLookupError:
            pass
        else:
            raise AssertionError('capture child leaked')
    finally:
        if parent.poll() is None:
            parent.kill()
            parent.wait()
