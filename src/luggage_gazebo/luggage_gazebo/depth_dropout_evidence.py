"""Bounded diagnostic input ownership for the opt-in simulated sensor fixture."""
from collections import OrderedDict
import hashlib
import io
import json
from pathlib import Path

import numpy as np


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


class DepthDropoutEvidence:
    def __init__(self, root, selection_file, *, ring_frames=128, ring_bytes=128*1024*1024,
                 trace_bytes=31*1024*1024, selected_frames=16, case_bytes=2*1024*1024):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=False)
        self.selection_file = Path(selection_file)
        self.ring_frames, self.ring_bytes = ring_frames, ring_bytes
        self.trace_bytes, self.max_saved_frames, self.case_bytes = trace_bytes, selected_frames, case_bytes
        # The legacy selected_frames argument now caps all saved frame roles.
        self._ring = OrderedDict()
        self._bytes = self._trace_size = self._event_offset = self._events = 0
        self._selected = set()
        self._payloads = {}
        self.diagnostics = []
        self.rows, self.missing, self.counts = [], [], {}
        self.trace_truncated = False

    def record(self, stamp_ns, frame, before, after, effect):
        row = dict(stamp_ns=int(stamp_ns), frame=frame, shape=list(before.shape),
                   input_sha256=digest(before.tobytes()), output_sha256=digest(after.tobytes()), **effect)
        reason = row['reason']
        self.counts[reason] = self.counts.get(reason, 0)+1
        line = (json.dumps(row)+'\n').encode()
        if self._trace_size+len(line) <= self.trace_bytes:
            with (self.root/'trace.jsonl').open('ab') as stream:
                stream.write(line)
            self._trace_size += len(line)
        else:
            self.trace_truncated = True
        key = (frame, int(stamp_ns))
        if key in self._ring:
            self._bytes -= self._ring.pop(key)[0].nbytes*2
        self._ring[key] = (before.copy(), after.copy(), row)
        self._bytes += before.nbytes+after.nbytes
        while len(self._ring) > self.ring_frames or self._bytes > self.ring_bytes:
            _, (a, b, _) = self._ring.popitem(last=False)
            self._bytes -= a.nbytes+b.nbytes
        self.poll_selections()

    def poll_selections(self):
        if not self.selection_file.exists():
            return
        # The producer owns at most64 lines. Retry partial final writes on next tick.
        with self.selection_file.open('rb') as stream:
            stream.seek(self._event_offset)
            while self._events < 64:
                line = stream.readline(64*1024+1)
                if not line:
                    break
                if len(line) > 64*1024:
                    self.missing.append('SELECTION_LINE_TOO_LARGE')
                    self._events = 64
                    break
                if not line.endswith(b'\n'):
                    break
                self._event_offset += len(line)
                self._events += 1
                try:
                    event = json.loads(line)
                    if event['status'] == 'saved':
                        self._pin(event)
                    elif event['status'] != 'already_saved':
                        self.missing.append('SELECTION_'+event['status'].upper())
                except (ValueError, KeyError, TypeError) as error:
                    self.missing.append('INVALID_SELECTION:'+str(error))

    def _pin(self, event):
        identity = tuple(event['identity'])
        key = identity+(int(event['stamp_ns']),)
        if key in self._selected:
            return
        self._selected.add(key)
        payload = self._persist_frame(key[-1])
        if payload is not None:
            self.rows.append(dict(identity=identity, suction_case=event['case'], **payload))

    def _persist_frame(self, stamp_ns):
        if stamp_ns in self._payloads:
            return self._payloads[stamp_ns]
        if len(self._payloads) >= self.max_saved_frames:
            self.missing.append('FRAME_PAYLOAD_LIMIT')
            return None
        matches = [value for (_, stamp), value in self._ring.items() if stamp == stamp_ns]
        if len(matches) != 1:
            self.missing.append('EXACT_FRAME_MISSING:'+str(stamp_ns))
            return None
        before, after, trace = matches[0]
        if trace.get('published') is False:
            self.missing.append('REQUESTED_FRAME_NOT_PUBLISHED:'+str(stamp_ns))
            return None
        buffer = io.BytesIO()
        np.savez_compressed(buffer, before_mm=before, after_mm=after)
        raw = buffer.getvalue()
        if len(raw) > self.case_bytes:
            self.missing.append('FRAME_BYTE_LIMIT:'+str(stamp_ns))
            return None
        name = 'frame-%03d.npz' % len(self._payloads)
        with (self.root/name).open('xb') as stream:
            stream.write(raw)
        payload = dict(**trace, file=name, file_sha256=digest(raw))
        self._payloads[stamp_ns] = payload
        return payload

    def pin_diagnostic(self, event):
        """A detector anomaly references a frame; it does not select a grasp."""
        payload = self._persist_frame(event['stamp_ns'])
        if payload is not None:
            self.diagnostics.append(dict(event, **payload))

    def close(self):
        self.poll_selections()
        if self.selection_file.exists() and self.selection_file.stat().st_size != self._event_offset:
            self.missing.append('SELECTION_TRUNCATED_OR_OVER_BUDGET')
        # Keep a small actual tail even if observation never admitted a pickup.
        # Its role explicitly makes no exact-consumption or selection claim.
        tails = 0
        for (_, stamp), (_, _, row) in reversed(self._ring.items()):
            if tails >= 4 or len(self._payloads) >= self.max_saved_frames:
                break
            if stamp in self._payloads or row.get('published') is False:
                continue
            payload = self._persist_frame(stamp)
            if payload is not None:
                self.diagnostics.append(dict(kind='unselected_tail', **payload))
                tails += 1
        summary = dict(schema='simulation_depth_dropout_evidence_v1', counts=self.counts,
                       selected=self.rows, diagnostics=self.diagnostics, missing=self.missing,
                       trace_truncated=self.trace_truncated,
                       complete=not self.missing and not self.trace_truncated)
        (self.root/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
        return summary
