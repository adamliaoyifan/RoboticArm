"""Optional bounded exact suction-evaluator inputs, without geometry authority."""
from dataclasses import asdict
import json
from pathlib import Path

import numpy as np


class SuctionInputCapture:
    def __init__(self, root, *, max_identities=4, per_identity=4, max_case_bytes=8*1024*1024):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.max_identities = max_identities
        self.per_identity = per_identity
        self.max_case_bytes = max_case_bytes
        self._stamps = {}

    def capture(self, depth, intrinsics, optical_to_world, top, region, model, evaluation, *, stamp_ns):
        identity = (evaluation.instance_id, evaluation.generation)
        stamps = self._stamps.get(identity, set())
        stamp = int(stamp_ns)
        if stamp in stamps or len(stamps) >= self.per_identity:
            return False
        if identity not in self._stamps and len(self._stamps) >= self.max_identities:
            return False
        top_fields = asdict(top)
        labels = top_fields.pop('plane_labels')
        arrays = dict(depth_mm=np.asarray(depth), optical_to_world=np.asarray(optical_to_world),
                      plane_labels=np.asarray(labels))
        if region is not None:
            arrays['instance_region'] = np.asarray(region)
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
        case = self.root / ('case-%03d' % sum(len(v) for v in self._stamps.values()))
        case.mkdir(exist_ok=False)
        np.savez_compressed(case/'inputs.npz', **arrays)
        (case/'record.json').write_bytes(encoded+b'\n')
        self._stamps.setdefault(identity,set()).add(stamp)
        return True
