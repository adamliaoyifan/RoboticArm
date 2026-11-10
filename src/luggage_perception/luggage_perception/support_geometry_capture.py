"""Opt-in bounded support-fit replay inputs; no online geometry authority."""
from dataclasses import asdict
import json
from pathlib import Path
import numpy as np


class SupportGeometryCapture:
    def __init__(self, root, max_cases=4, max_case_bytes=64*1024*1024):
        self.root=Path(root)
        self.root.mkdir(parents=True,exist_ok=True)
        self.max_cases=max_cases
        self.max_case_bytes=max_case_bytes
        self.seen=set()

    def capture(self, identity, stamp_ns, cargo, raw, config, result, history, *, allow_invalid=False):
        if identity in self.seen or len(self.seen)>=self.max_cases or (not result.height_valid and not allow_invalid):
            return False
        if raw is None:
            raise ValueError('valid measured height without raw support input')
        cargo=np.asarray(cargo);raw=np.asarray(raw)
        if cargo.nbytes+raw.nbytes>self.max_case_bytes:
            raise ValueError('support capture byte budget exceeded')
        case=self.root/('case-%02d'%len(self.seen))
        case.mkdir(exist_ok=False)
        def encode(value):
            if isinstance(value,np.ndarray):return value.tolist()
            if isinstance(value,np.generic):return value.item()
            raise TypeError(type(value).__name__)
        record=dict(identity=identity,stamp_ns=int(stamp_ns),frame_id='world',
                    config=asdict(config),top=asdict(result.box.top),
                    support=asdict(result.support) if result.support is not None else None,
                    height_valid=bool(result.height_valid),box=asdict(result.box),
                    support_history=list(history),scope='diagnostic inputs only')
        np.savez_compressed(case/'inputs.npz',cargo=cargo,raw=raw)
        (case/'record.json').write_text(json.dumps(record,default=encode,indent=2)+'\n')
        self.seen.add(identity)
        return True
