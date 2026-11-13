"""Read existing bounded detector diagnostic receipts, without sensor authority."""
from itertools import islice
import json
from pathlib import Path

from luggage_gazebo.depth_dropout_evidence import digest


class DetectorAnomalyEvents:
    def __init__(self, root, evidence):
        self.root = Path(root)
        self.evidence = evidence
        self.seen, self.pending = set(), set()
        self.epoch_limit_reported = False

    def poll(self):
        if not self.root.exists():
            return
        epochs = list(islice((p for p in self.root.iterdir() if p.is_dir()), 5))
        if len(epochs) > 4 and not self.epoch_limit_reported:
            self.evidence.missing.append('DETECTOR_CAPTURE_EPOCH_LIMIT')
            self.epoch_limit_reported = True
        for epoch in epochs[:4]:
            # FrameCapture owns one pinned anomaly per epoch, not a frame stream.
            path = next(epoch.glob('anomaly_*/frame.json'), None)
            if path is None or path in self.seen:
                continue
            with path.open('rb') as stream:
                raw = stream.read(64*1024+1)
            if len(raw) > 64*1024:
                self.evidence.missing.append('DETECTOR_CAPTURE_MANIFEST_BYTE_LIMIT')
                self.seen.add(path)
                continue
            try:
                record = json.loads(raw)
            except json.JSONDecodeError:
                self.pending.add(path)  # Writer may still be completing the JSON.
                continue
            self.pending.discard(path)
            self.seen.add(path)
            if (record.get('role') != 'anomaly' or type(record.get('stamp_ns')) is not int
                    or record['stamp_ns'] <= 0 or not isinstance(record.get('epoch'), dict)):
                self.evidence.missing.append('DETECTOR_CAPTURE_MANIFEST_INVALID')
                continue
            self.evidence.pin_diagnostic(dict(kind='detector_anomaly',stamp_ns=record['stamp_ns'],
                detector_case=str(path.relative_to(self.root)),
                detector_record_sha256=digest(raw), epoch=record['epoch']))

    def close(self):
        self.poll()
        if self.pending:
            self.evidence.missing.append('DETECTOR_CAPTURE_MANIFEST_TRUNCATED')
