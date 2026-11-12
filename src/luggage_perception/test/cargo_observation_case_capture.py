"""Small exact-frame captures, including expected-refusal cases."""
import json
import os
from pathlib import Path
import re

import numpy as np


class CaseCapture:
    def __init__(self, node_id, temporary):
        root = Path(os.environ.get('CARGO_OBSERVATION_EVIDENCE_DIR',str(temporary)))
        self.path = root/re.sub('[^a-zA-Z0-9_.-]','_',node_id)
        self.path.mkdir(parents=True,exist_ok=True)
        self.sequence = 0
        self.value('case',dict(node_id=node_id))

    def _next(self, name, suffix):
        self.sequence += 1
        return self.path/('%03d_%s.%s'%(self.sequence,name,suffix))

    def value(self, name, value):
        self._next(name,'json').write_text(json.dumps(value,sort_keys=True,indent=2))

    def arrays(self, **values):
        np.savez_compressed(self._next('inputs','npz'),**values)

    def message(self, name, message):
        from rclpy.serialization import serialize_message
        self._next(name,'cdr').write_bytes(serialize_message(message))
