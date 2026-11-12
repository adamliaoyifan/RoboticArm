"""Read-only join of OCC's typed grid and acquisition/confirmation publication.

The existing v2 grid does not carry occupancy_confirmed or map_source. Reading
it alone cannot prove that the preceding placement was observed. Join matching
revisions and verify grid contents, and retain only measured, confirmed maps.
No callbacks or operations here mutate the map owner.
"""
from dataclasses import dataclass
import hashlib
import json
from threading import Lock


def freeze(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


@dataclass(frozen=True)
class ConfirmedSurface:
    record_json: bytes
    surface_cdr: bytes
    confirmation_json: bytes

    @property
    def identity(self):
        return hashlib.sha256(self.record_json + self.confirmation_json).hexdigest()

    def record(self):
        return json.loads(self.record_json)


class ConfirmedSurfaceStore:
    """Latest typed and confirmation messages only; asynchronous order is OK."""
    def __init__(self, *, confirmation_required=False):
        # Node resolves the map owner's require_confirmed_map parameter by a
        # read-only parameter query. A default "confirmed" flag with that gate
        # disabled cannot authorize committed geometry after placement.
        self._confirmation_required = bool(confirmation_required)
        self._lock = Lock()
        self._typed = None
        self._confirmation = None
        self._typed_version = None
        self._confirmation_version = None

    def set_confirmation_required(self, required):
        with self._lock:
            self._confirmation_required = bool(required)

    def invalidate(self):
        with self._lock:
            self._typed = None
            self._confirmation = None
            # Retain version watermarks so delayed old messages cannot revive it.

    def update_typed(self, admitted_record, cdr):
        # Caller must first pass surface_from_message / validate_surface.
        encoded = freeze(admitted_record)
        if not cdr:
            raise ValueError('typed surface CDR required')
        with self._lock:
            version = (admitted_record['scene_revision'], admitted_record['geometry_hash'],
                       int(admitted_record['map_revision']))
            if (self._typed_version is not None and version[:2] == self._typed_version[:2]
                    and version[2] < self._typed_version[2]):
                return
            self._typed_version = version
            self._typed = (encoded, bytes(cdr))

    def update_confirmation(self, legacy_surface):
        encoded = freeze(legacy_surface)
        with self._lock:
            version = (legacy_surface['geometry_hash'], int(legacy_surface['map_revision']),
                       int(legacy_surface.get('change_token', 0)))
            if (self._confirmation_version is not None and version[0] == self._confirmation_version[0]
                    and version[1:] < self._confirmation_version[1:]):
                return
            self._confirmation_version = version
            self._confirmation = encoded

    def current(self):
        with self._lock:
            if self._typed is None or self._confirmation is None:
                return None
            encoded, cdr = self._typed
            confirmation = self._confirmation
        surface, legacy = json.loads(encoded), json.loads(confirmation)
        # release_integrated alone can be a planned-box commit with no live
        # observation. It is not enough, even if confirmation is disabled.
        if int(legacy.get('placed_count', 0)) > 0 and not self._confirmation_required:
            return None
        if (legacy.get('map_source') != 'measured' or
                legacy.get('occupancy_confirmed') is not True or
                type(legacy.get('acquisition_stamp_ns')) is not int or
                legacy['acquisition_stamp_ns'] <= 0 or
                legacy.get('pending_changes') != []):
            return None
        if (surface.get('map_revision') != legacy.get('map_revision') or
                surface.get('geometry_hash') != legacy.get('geometry_hash') or
                surface.get('frame_id') != 'container_link' or
                legacy.get('frame') != 'container_local'):
            return None
        # Do not join same-labelled but different cell data. v1 "height" is
        # above floor; v2 surface_z is relative to container origin.
        if surface.get('state') != legacy.get('state'):
            return None
        heights = legacy.get('height')
        tops = surface.get('surface_z_container')
        if not isinstance(heights, list) or not isinstance(tops, list) or len(heights) != len(tops):
            return None
        floor = surface.get('floor_z_container')
        if not isinstance(floor, (int, float)):
            return None
        for r, (height_row, top_row) in enumerate(zip(heights, tops)):
            if len(height_row) != len(top_row):
                return None
            for c, (height, top) in enumerate(zip(height_row, top_row)):
                if surface['state'][r][c] == 'unknown':
                    if top is not None:
                        return None
                elif (not isinstance(height, (int, float)) or
                      not isinstance(top, (int, float)) or abs(top-floor-height) > 1e-9):
                    return None
        return ConfirmedSurface(encoded, cdr, confirmation)
