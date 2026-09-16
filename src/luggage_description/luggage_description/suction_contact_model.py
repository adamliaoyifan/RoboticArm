#!/usr/bin/env python3
"""Versioned suction contact model: strict loader + plain data object.

DYNAMIC-SUCTION plan section B (ST-2 gate B0). The hardware launch must
require an explicit readable config and log its absolute path plus
SHA-256; missing, malformed, non-positive or oversized geometry fails
startup. This module is the single loader for that contract. It is
ROS-free (config I/O is the established ``luggage_description`` pattern,
see ``scene_tf_config_utils``); the evaluator consumes only the plain
:class:`ContactModel` object and never reads files.

Every rejection carries a machine-readable reason code so B0 can assert
schema behaviour (a text assertion would only check spelling).
"""

from __future__ import division

import hashlib
import math

import yaml

CONTACT_MODEL_MISSING_KEY = "CONTACT_MODEL_MISSING_KEY"
CONTACT_MODEL_DUPLICATE_KEY = "CONTACT_MODEL_DUPLICATE_KEY"
CONTACT_MODEL_NAN = "CONTACT_MODEL_NAN"
CONTACT_MODEL_NON_POSITIVE = "CONTACT_MODEL_NON_POSITIVE"
CONTACT_MODEL_OVERSIZED = "CONTACT_MODEL_OVERSIZED"
CONTACT_MODEL_BAD_TYPE = "CONTACT_MODEL_BAD_TYPE"
CONTACT_MODEL_UNKNOWN_FOOTPRINT = "CONTACT_MODEL_UNKNOWN_FOOTPRINT"

#: largest permitted footprint dimension (m); plan section B.
MAX_FOOTPRINT_DIMENSION_M = 0.30

#: schema keys -> (type, minimum-exclusive-where-numeric) expected values.
POSITIVE_FLOAT_KEYS = (
    "boundary_margin_m",
    "cell_size_m",
    "candidate_grid_m",
    "max_rms_residual_m",
    "max_p95_residual_m",
    "max_peak_to_valley_m",
    "max_adjacent_step_m",
    "adjacent_step_distance_m",
    "bimodal_min_separation_m",
    "min_candidate_separation_m",
)
UNIT_FRACTION_KEYS = (
    "min_valid_cell_fraction",
    "min_mask_coverage",
    "min_connected_plane_fraction",
    "bimodal_min_fraction",
    "max_candidate_iou",
)
POSITIVE_INT_KEYS = ("max_candidates", "max_rejected_diagnostics")
POSITIVE_DEG_KEYS = ("max_normal_deviation_p95_deg", "max_normal_tilt_deg")
REQUIRED_KEYS = tuple(
    ["model_version", "contact_frame", "footprint_type",
     "footprint_size_xy_m"]
    + list(POSITIVE_FLOAT_KEYS) + list(UNIT_FRACTION_KEYS)
    + list(POSITIVE_INT_KEYS) + list(POSITIVE_DEG_KEYS))


class SuctionContactModelError(ValueError):
    """Schema violation; ``reason`` is a stable machine code."""

    def __init__(self, reason, detail):
        super(SuctionContactModelError, self).__init__(
            "%s: %s" % (reason, detail))
        self.reason = str(reason)
        self.detail = str(detail)


class _StrictLoader(yaml.SafeLoader):
    """SafeLoader that rejects duplicate mapping keys."""


def _no_duplicate_keys(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        try:
            duplicate = key in mapping
        except TypeError:
            raise yaml.constructor.ConstructorError(
                None, None, "unhashable key %r" % (key,), key_node.start_mark)
        if duplicate:
            raise yaml.constructor.ConstructorError(
                None, None, "duplicate key %r" % (key,), key_node.start_mark)
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_StrictLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _no_duplicate_keys)


def _finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


class ContactModel(object):
    """Plain, immutable-ish contact geometry + thresholds (no I/O).

    ``identity_hash`` is the SHA-256 of the exact file bytes the model was
    loaded from; candidate records must carry the same model version and
    hash (B0).
    """

    __slots__ = (
        "model_version", "contact_frame", "footprint_type",
        "footprint_size_xy_m", "boundary_margin_m",
        "cell_size_m", "candidate_grid_m",
        "min_valid_cell_fraction", "min_mask_coverage",
        "min_connected_plane_fraction", "max_rms_residual_m",
        "max_p95_residual_m", "max_peak_to_valley_m",
        "max_normal_deviation_p95_deg", "max_adjacent_step_m",
        "adjacent_step_distance_m", "bimodal_min_separation_m",
        "bimodal_min_fraction", "max_normal_tilt_deg",
        "max_candidates", "min_candidate_separation_m",
        "max_candidate_iou", "max_rejected_diagnostics",
        "identity_hash",
    )

    def __init__(self, **fields):
        for key in self.__slots__:
            setattr(self, key, fields[key])

    # Convenience halves (footprint is axis-aligned in the top-plane
    # frame; 0.18 x 0.18 is square so in-plane rotation is a no-op).
    @property
    def footprint_half_xy_m(self):
        return (self.footprint_size_xy_m[0] / 2.0,
                self.footprint_size_xy_m[1] / 2.0)

    def as_dict(self):
        return {key: getattr(self, key) for key in self.__slots__}


def validate_contact_model_fields(data, source="config"):
    """Validate a parsed mapping -> plain dict of typed fields.

    Shared by the file loader and any in-memory construction (synthetic
    tests inject smaller footprints through here). Raises
    :class:`SuctionContactModelError` with a stable reason code.
    """
    if not isinstance(data, dict):
        raise SuctionContactModelError(
            CONTACT_MODEL_BAD_TYPE, "%s is not a mapping" % source)
    missing = [k for k in REQUIRED_KEYS if k not in data]
    if missing:
        raise SuctionContactModelError(
            CONTACT_MODEL_MISSING_KEY, "missing %s" % ",".join(missing))

    version = data["model_version"]
    if isinstance(version, bool) or not isinstance(version, int) \
            or version < 1:
        raise SuctionContactModelError(
            CONTACT_MODEL_BAD_TYPE, "model_version must be a positive int")
    frame = data["contact_frame"]
    if not isinstance(frame, str) or not frame.strip():
        raise SuctionContactModelError(
            CONTACT_MODEL_BAD_TYPE, "contact_frame must be a non-empty str")
    if data["footprint_type"] != "rectangle":
        raise SuctionContactModelError(
            CONTACT_MODEL_UNKNOWN_FOOTPRINT,
            "footprint_type %r not supported" % (data["footprint_type"],))

    size = data["footprint_size_xy_m"]
    if not isinstance(size, (list, tuple)) or len(size) != 2:
        raise SuctionContactModelError(
            CONTACT_MODEL_BAD_TYPE, "footprint_size_xy_m must be [x, y]")
    for value in size:
        if not _finite(value):
            raise SuctionContactModelError(CONTACT_MODEL_NAN,
                                           "footprint %r" % (value,))
        if float(value) <= 0.0:
            raise SuctionContactModelError(
                CONTACT_MODEL_NON_POSITIVE, "footprint %r" % (value,))
        if float(value) > MAX_FOOTPRINT_DIMENSION_M:
            raise SuctionContactModelError(
                CONTACT_MODEL_OVERSIZED,
                "footprint %.3f > %.3f" % (float(value),
                                           MAX_FOOTPRINT_DIMENSION_M))
    size = (float(size[0]), float(size[1]))

    fields = {
        "model_version": int(version),
        "contact_frame": str(frame),
        "footprint_type": str(data["footprint_type"]),
        "footprint_size_xy_m": size,
    }
    for key in POSITIVE_FLOAT_KEYS:
        value = data[key]
        if not _finite(value):
            raise SuctionContactModelError(CONTACT_MODEL_NAN, key)
        if float(value) <= 0.0:
            raise SuctionContactModelError(CONTACT_MODEL_NON_POSITIVE, key)
        fields[key] = float(value)
    for key in UNIT_FRACTION_KEYS:
        value = data[key]
        if not _finite(value):
            raise SuctionContactModelError(CONTACT_MODEL_NAN, key)
        if not 0.0 < float(value) <= 1.0:
            raise SuctionContactModelError(
                CONTACT_MODEL_BAD_TYPE,
                "%s must be in (0, 1]" % key)
        fields[key] = float(value)
    for key in POSITIVE_INT_KEYS:
        value = data[key]
        if isinstance(value, bool) or not isinstance(value, int) \
                or value < 1:
            raise SuctionContactModelError(
                CONTACT_MODEL_BAD_TYPE, "%s must be a positive int" % key)
        fields[key] = int(value)
    for key in POSITIVE_DEG_KEYS:
        value = data[key]
        if not _finite(value):
            raise SuctionContactModelError(CONTACT_MODEL_NAN, key)
        if not 0.0 < float(value) < 90.0:
            raise SuctionContactModelError(
                CONTACT_MODEL_BAD_TYPE, "%s must be in (0, 90)" % key)
        fields[key] = float(value)
    fields["identity_hash"] = ""
    return fields


def load_suction_contact_model(path):
    """Read + validate a contact model file -> :class:`ContactModel`.

    Duplicate YAML keys are rejected by the strict loader. The returned
    object carries ``identity_hash`` = SHA-256 of the file bytes (B0: the
    hardware launch logs path + this hash; candidate records repeat it).
    """
    with open(path, "rb") as handle:
        raw = handle.read()
    try:
        data = yaml.load(raw.decode("utf-8"), Loader=_StrictLoader)
    except yaml.constructor.ConstructorError as exc:
        raise SuctionContactModelError(
            CONTACT_MODEL_DUPLICATE_KEY, str(exc.problem))
    except yaml.YAMLError as exc:
        raise SuctionContactModelError(
            CONTACT_MODEL_BAD_TYPE, "unparseable YAML: %s" % exc)
    fields = validate_contact_model_fields(data, source=str(path))
    fields["identity_hash"] = hashlib.sha256(raw).hexdigest()
    return ContactModel(**fields)
