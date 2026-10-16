"""Bounded, observation-only Gazebo release capture; never a planning input.

Raw topic bytes retain entity IDs, simulation stamps and contact details.
Chunk offsets carry host timestamps for joining driver boundary events. Empty
contact output is not evidence of no contact. Duration/byte limits are explicit
in the manifest; consumers must check whether their event interval is covered.
"""
import argparse
import json
import os
from pathlib import Path
import selectors
import signal
import subprocess
import time


def record(output, commands, duration=90., byte_limit=8*1024*1024):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    selector = selectors.DefaultSelector()
    children = []
    files = []
    stopped = False
    start = time.monotonic()
    manifest = {'start_wall': time.time(), 'start_monotonic': start,
                'duration_limit_s': duration, 'per_stream_byte_limit': byte_limit,
                'streams': {}, 'contact_absence_proven': False}

    def stop(_signum, _frame):
        nonlocal stopped
        stopped = True

    previous = {sig: signal.signal(sig, stop) for sig in (signal.SIGTERM, signal.SIGINT)}
    try:
        with (output/'chunks.jsonl').open('w') as journal:
            for name, command in commands.items():
                raw = (output/(name+'.raw')).open('wb')
                error = (output/(name+'.stderr')).open('wb')
                files.extend([raw, error])
                child = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=error)
                children.append((name, child))
                selector.register(child.stdout, selectors.EVENT_READ, (name, raw))
                manifest['streams'][name] = {'bytes': 0, 'limited': False, 'eof': False}
            (output/'ready.json').write_text(json.dumps({
                'wall': time.time(), 'meaning': 'readers started; publisher readiness unverified'})+'\n')
            while not stopped and time.monotonic()-start < duration and selector.get_map():
                for key, _ in selector.select(timeout=.1):
                    name, raw = key.data
                    chunk = os.read(key.fd, 65536)
                    state = manifest['streams'][name]
                    if not chunk:
                        state['eof'] = True
                        selector.unregister(key.fileobj)
                        continue
                    remaining = byte_limit-state['bytes']
                    kept = chunk[:remaining]
                    raw.write(kept)
                    raw.flush()
                    journal.write(json.dumps({'stream': name, 'offset': state['bytes'],
                        'length': len(kept), 'wall': time.time(),
                        'monotonic': time.monotonic()})+'\n')
                    journal.flush()
                    state['bytes'] += len(kept)
                    if len(chunk) >= remaining:
                        state['limited'] = True
                        selector.unregister(key.fileobj)
                        next(child for n, child in children if n == name).terminate()
            manifest['end_reason'] = ('signal' if stopped else
                'duration' if time.monotonic()-start >= duration else 'streams_ended')
    finally:
        for name, child in children:
            if child.poll() is None:
                child.terminate()
            try:
                child.wait(timeout=2.)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=2.)
            manifest['streams'][name]['returncode'] = child.returncode
            child.stdout.close()
        for handle in files:
            handle.close()
        selector.close()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        manifest.update(end_wall=time.time(), end_monotonic=time.monotonic())
        (output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--contact-topic', required=True)
    args = parser.parse_args()
    record(args.output, {
        'poses': ['ign', 'topic', '-e', '-t', '/world/airport_loading/pose/info'],
        'contacts': ['ign', 'topic', '-e', '-t', args.contact_topic]})


if __name__ == '__main__':
    main()
