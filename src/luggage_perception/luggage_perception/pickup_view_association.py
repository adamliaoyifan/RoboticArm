"""Owned measured image/candidate inputs; no cross-view identity decision.

Complete means all candidates in one received detector message. It does not
assert that detection found every physical box, or that a bbox is a support face.
"""
from dataclasses import dataclass
import math

import numpy as np


MAX_ASSOCIATION_BYTES = 8 * 1024 * 1024  # Transport resources, not accuracy.
MAX_REPORTED_CANDIDATES = 256


@dataclass(frozen=True)
class BufferedPickupRgb:
    stamp_ns: int
    optical_frame_id: str
    shape: tuple
    rgb8: bytes

    def __post_init__(self):
        shape = tuple(self.shape)
        pixels = bytes(self.rgb8)
        if (type(self.stamp_ns) is not int or self.stamp_ns < 0
                or not isinstance(self.optical_frame_id, str) or not self.optical_frame_id.strip()
                or len(shape) != 3 or shape[2] != 3
                or any(type(v) is not int or v <= 0 for v in shape)
                or len(pixels) != math.prod(shape) or len(pixels) > MAX_ASSOCIATION_BYTES):
            raise ValueError('PICKUP_ASSOCIATION_RGB_LAYOUT_OR_SOURCE_INVALID')
        object.__setattr__(self, 'shape', shape)
        object.__setattr__(self, 'rgb8', pixels)

    @classmethod
    def capture(cls, image, *, stamp_ns, optical_frame_id, encoding):
        image = np.asarray(image)
        if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
            raise ValueError('PICKUP_ASSOCIATION_RGB8_REQUIRED')
        if encoding not in ('rgb8', 'bgr8'):
            raise ValueError('PICKUP_ASSOCIATION_RGB_ENCODING_UNSUPPORTED')
        if image.nbytes > MAX_ASSOCIATION_BYTES:
            raise ValueError('PICKUP_ASSOCIATION_RGB_OVER_BUDGET')
        normalized = image[:, :, ::-1] if encoding == 'bgr8' else image
        return cls(stamp_ns, optical_frame_id, tuple(image.shape), normalized.tobytes())

    def array(self):
        return np.frombuffer(self.rgb8, dtype=np.uint8).reshape(self.shape)


@dataclass(frozen=True)
class ReportedPickupCandidate:
    label: int
    prompt: str
    confidence: float
    bbox: tuple
    held: bool

    def __post_init__(self):
        bbox = tuple(self.bbox)
        if (type(self.label) is not int or not -(2**31) <= self.label < 2**31
                or not isinstance(self.prompt, str) or len(self.prompt.encode()) > 4096
                or not math.isfinite(self.confidence) or not 0 <= self.confidence <= 1
                or len(bbox) != 4 or any(type(v) is not int or not -(2**31) <= v < 2**31 for v in bbox)
                or bbox[2] <= bbox[0] or bbox[3] <= bbox[1] or type(self.held) is not bool):
            raise ValueError('PICKUP_ASSOCIATION_CANDIDATE_INVALID')
        object.__setattr__(self, 'bbox', bbox)
        # The source wire is float32: preserve that representation in the digest.
        object.__setattr__(self, 'confidence', float(np.float32(self.confidence)))

    def metadata(self):
        return dict(label=self.label, prompt=self.prompt, confidence=self.confidence,
                    bbox=self.bbox, held=self.held)


def selected_candidate_index(candidates):
    """Existing confidence/area selection, preserving the first exact tie."""
    if not candidates:
        raise ValueError('PICKUP_ASSOCIATION_CANDIDATES_EMPTY')
    return max(range(len(candidates)), key=lambda i: (
        candidates[i].confidence,
        (candidates[i].bbox[2]-candidates[i].bbox[0]) *
        (candidates[i].bbox[3]-candidates[i].bbox[1])))


@dataclass(frozen=True)
class PickupAssociationInputs:
    rgb: BufferedPickupRgb
    candidates: tuple
    selected_index: int
    frame_seq: int
    generation: int
    instance_id: str
    backend: str

    def __post_init__(self):
        candidates = tuple(self.candidates)
        if (type(self.rgb) is not BufferedPickupRgb or not candidates
                or len(candidates) > MAX_REPORTED_CANDIDATES
                or any(type(v) is not ReportedPickupCandidate for v in candidates)
                or type(self.selected_index) is not int or not 0 <= self.selected_index < len(candidates)
                or type(self.frame_seq) is not int or not 0 <= self.frame_seq < 2**32
                or type(self.generation) is not int or not 0 <= self.generation < 2**32
                or not isinstance(self.instance_id, str) or not self.instance_id.strip()
                or not isinstance(self.backend, str)):
            raise ValueError('PICKUP_ASSOCIATION_POOL_INVALID_OR_OVER_BUDGET')
        size = len(self.rgb.rgb8) + sum(len(c.prompt.encode())+128 for c in candidates)
        size += len(self.instance_id.encode()) + len(self.backend.encode())
        if size > MAX_ASSOCIATION_BYTES:
            raise ValueError('PICKUP_ASSOCIATION_INPUTS_OVER_BUDGET')
        object.__setattr__(self, 'candidates', candidates)

    @property
    def size_bytes(self):
        return (len(self.rgb.rgb8)+sum(len(c.prompt.encode())+128 for c in self.candidates)
                +len(self.instance_id.encode())+len(self.backend.encode())
                +len(self.rgb.optical_frame_id.encode())+1024)

    def metadata(self):
        return dict(rgb_stamp_ns=self.rgb.stamp_ns, rgb_frame_id=self.rgb.optical_frame_id,
                    rgb_shape=self.rgb.shape, candidates=[c.metadata() for c in self.candidates],
                    selected_index=self.selected_index, frame_seq=self.frame_seq,
                    generation=self.generation, instance_id=self.instance_id, backend=self.backend,
                    candidate_pool_complete=True)
