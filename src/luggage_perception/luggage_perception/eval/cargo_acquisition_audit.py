"""Offline acquisition evidence, without ROS, geometry tolerances or identity claims."""
from collections import Counter, defaultdict
import struct

import numpy as np


def cdr_header(data):
    """Read a leading ROS2 Header, rejecting unsupported/truncated CDR layouts."""
    if len(data) < 17 or bytes(data[:2]) not in (b'\x00\x00', b'\x00\x01'):
        raise ValueError('unsupported or truncated CDR Header')
    endian = '<' if data[1] == 1 else '>'
    sec, nsec, length = struct.unpack_from(endian + 'iII', data, 4)
    if nsec >= 1_000_000_000 or length < 1 or 16 + length > len(data):
        raise ValueError('malformed Header stamp/frame')
    frame = bytes(data[16:16 + length])
    if frame[-1] != 0:
        raise ValueError('frame string is not terminated')
    return sec * 1_000_000_000 + nsec, frame[:-1].decode('utf8')


def distribution(values):
    values = np.asarray(values, dtype=np.float64)
    if not values.size:
        return {'count': 0}
    return dict(count=int(values.size), minimum=float(values.min()),
                median=float(np.median(values)), p95=float(np.percentile(values, 95)),
                maximum=float(values.max()))


class AcquisitionIndex:
    """Full-source hash conflicts remain visible; exact joins never use nearest time."""

    def __init__(self):
        self.records = defaultdict(list)
        self.hashes = defaultdict(lambda: defaultdict(set))

    def add(self, topic, stamp, frame, log_time, payload_hash):
        self.records[topic].append((stamp, frame, log_time))
        self.hashes[topic][(stamp, frame)].add(payload_hash)

    def exact_keys(self, topics):
        if not topics:
            return []
        keys = set(self.hashes[topics[0]])
        for topic in topics[1:]:
            keys &= set(self.hashes[topic])
        return sorted(key for key in keys if key[0] > 0 and key[1] and
                      all(len(self.hashes[t][key]) == 1 for t in topics))

    def summary(self):
        result = {}
        for topic, records in self.records.items():
            stamps = np.asarray([x[0] for x in records], dtype=np.int64)
            keys = [(x[0], x[1]) for x in records]
            result[topic] = {
                'message_count': len(records), 'unique_keys': len(set(keys)),
                'duplicate_keys': sum(n - 1 for n in Counter(keys).values()),
                'conflicting_keys': sum(len(v) > 1 for v in self.hashes[topic].values()),
                'nonpositive_stamp_count': int((stamps <= 0).sum()),
                'frames': dict(Counter(x[1] for x in records)),
                'consecutive_header_delta_ms': distribution(np.diff(stamps) / 1e6),
                'log_minus_header_ms': distribution([(x[2] - x[0]) / 1e6 for x in records]),
                'first_header_ns': int(stamps.min()), 'last_header_ns': int(stamps.max()),
            }
        return result
