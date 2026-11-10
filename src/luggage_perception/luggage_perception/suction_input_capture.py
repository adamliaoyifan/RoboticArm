"""Optional bounded exact suction-evaluator inputs, without geometry authority."""
from dataclasses import asdict
from collections import OrderedDict
import json
from pathlib import Path
from threading import RLock

import numpy as np


class SuctionInputCapture:
    def __init__(self, root, *, max_identities=4, per_identity=4, max_case_bytes=8*1024*1024,
                 buffer_frames=8):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.max_identities = max_identities
        self.per_identity = per_identity
        self.max_case_bytes = max_case_bytes
        self._stamps = {}
        self._pending = OrderedDict()
        self._buffer_frames = buffer_frames
        self._lock = RLock()
        self._events = 0

    def capture(self, depth, intrinsics, optical_to_world, top, region, model, evaluation, *, stamp_ns):
        self.stage(depth, intrinsics, optical_to_world, top, region, model, evaluation,
                   stamp_ns=stamp_ns)
        return self.pin(evaluation.instance_id, evaluation.generation, stamp_ns)['status'] == 'saved'

    def stage(self, depth, intrinsics, optical_to_world, top, region, model, evaluation, *, stamp_ns):
        """Own copies of the latest bounded inputs; do not choose a capture frame."""
        identity = (evaluation.instance_id, evaluation.generation)
        stamp = int(stamp_ns)
        top_fields = asdict(top)
        labels = top_fields.pop('plane_labels')
        arrays = dict(depth_mm=np.array(depth, copy=True), optical_to_world=np.array(optical_to_world, copy=True),
                      plane_labels=np.asarray(labels))
        if region is not None:
            arrays['instance_region'] = np.array(region, copy=True)
        def encode(value):
            if isinstance(value, np.ndarray): return value.tolist()
            if isinstance(value, np.generic): return value.item()
            raise TypeError(type(value).__name__)
        record = dict(schema='suction-input-capture/1',
            scope='diagnostic measured inputs; not a newly authorized grasp',
            identity=identity, stamp_ns=stamp, stamp=float(evaluation.stamp), frame=evaluation.frame,
            intrinsics={key:float(getattr(intrinsics,key)) for key in ('fx','fy','cx','cy')},
            top=top_fields, contact_model=model.as_dict(), evaluation=asdict(evaluation),
            instance_region_present=region is not None)
        encoded = json.dumps(record, default=encode, indent=2).encode()
        if len(encoded) + sum(a.nbytes for a in arrays.values()) > self.max_case_bytes:
            raise ValueError('suction input capture byte budget exceeded')
        with self._lock:
            key = identity + (stamp,)
            self._pending[key] = (arrays, encoded)
            self._pending.move_to_end(key)
            while len(self._pending) > self._buffer_frames:
                self._pending.popitem(last=False)

    def pin(self, instance_id, generation, stamp_ns):
        """Persist exactly the selected identity/stamp, never a nearest frame."""
        identity = (str(instance_id), int(generation))
        stamp = int(stamp_ns)
        with self._lock:
            stamps = self._stamps.get(identity, set())
            key = identity + (stamp,)
            if stamp in stamps:
                result = dict(status='already_saved')
            elif (len(stamps) >= self.per_identity or
                  (identity not in self._stamps and len(self._stamps) >= self.max_identities)):
                result = dict(status='capture_limit')
            elif key not in self._pending:
                result = dict(status='exact_input_missing')
            else:
                arrays, encoded = self._pending[key]
                case = self.root / ('case-%03d' % sum(len(v) for v in self._stamps.values()))
                case.mkdir(exist_ok=False)
                np.savez_compressed(case/'inputs.npz', **arrays)
                (case/'record.json').write_bytes(encoded+b'\n')
                self._stamps.setdefault(identity,set()).add(stamp)
                result = dict(status='saved', case=case.name)
            result.update(identity=identity, stamp_ns=stamp)
            if self._events < 64:
                with (self.root/'selection_events.jsonl').open('a') as stream:
                    stream.write(json.dumps(result)+'\n')
                self._events += 1
            return result
