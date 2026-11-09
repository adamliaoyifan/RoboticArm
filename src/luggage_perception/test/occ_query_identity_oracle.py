"""Full primary-fact hash oracle for the fixed-load pressure fixture.

Implements the documented schema recipes without importing query indexes,
canonical storage, delta capture, or product hash helpers. Pressure only
integrates views after setup, so every named setup box remains in the ledger.
"""

import hashlib
import json

import numpy as np


def primary_voxel_digest(mapper, store, geometry, schema):
    assert schema in (1, 2), "UNSUPPORTED_INDEX_SCHEMA"
    nx, ny, nz = mapper.nx, mapper.ny, mapper.nz
    shape = (nx, ny, nz)
    origin = (-mapper.inner_l / 2, -mapper.inner_w / 2, mapper._hull.floor_z)
    compact_json = lambda value, **kwargs: json.dumps(
        value, separators=(",", ":"), **kwargs
    ).encode()
    root = hashlib.sha256(compact_json((shape, origin, mapper.resolution, mapper.geometry_hash)))
    dtype = np.dtype(
        [
            ("state", "u1"),
            ("source", "u1"),
            ("label", "<i8"),
            ("instance", "<i8"),
            ("logodds", "<f8"),
            ("locked", "?"),
            ("seen", "?"),
            ("shadow", "<f8"),
            ("shadow_label", "<i8"),
            ("shadow_instance", "<i8"),
        ]
    )
    columns = {
        "state": np.frombuffer(mapper._grid, "u1"),
        "source": np.frombuffer(mapper._source, "u1"),
        "label": np.frombuffer(mapper._labels, "<i8"),
        "instance": np.frombuffer(mapper._instance_ids, "<i8"),
        "logodds": np.asarray(mapper._occupancy.values, dtype="<f8"),
        "locked": np.asarray(mapper._occupancy.geometry_locked, dtype="?"),
        "seen": np.frombuffer(mapper._sensor_seen, "u1"),
        "shadow": np.frombuffer(mapper._sensor_values, "<f8"),
        "shadow_label": np.frombuffer(mapper._sensor_labels, "<i8"),
        "shadow_instance": np.frombuffer(mapper._sensor_instance_ids, "<i8"),
    }
    active = np.frombuffer(mapper._active, "u1")
    blocks_x, blocks_y = (nx + 15) // 16, (ny + 15) // 16
    geometry_blocks = {}
    for identifier, members in geometry.items():
        indices = np.asarray(sorted(members), dtype=np.int64)
        ids = indices % nx // 16 + blocks_x * (
            indices // nx % ny // 16 + blocks_y * (indices // (nx * ny) // 16)
        )
        for block in np.unique(ids):
            geometry_blocks.setdefault(int(block), {})[identifier] = indices[ids == block]
    block = 0
    for z in range(0, nz, 16):
        for y in range(0, ny, 16):
            for x in range(0, nx, 16):
                keys = (
                    np.arange(x, min(nx, x + 16))[None, None, :]
                    + nx
                    * (
                        np.arange(y, min(ny, y + 16))[None, :, None]
                        + ny * np.arange(z, min(nz, z + 16))[:, None, None]
                    )
                ).ravel()
                records = np.empty(len(keys), dtype=dtype)
                for field, values in columns.items():
                    records[field] = values[keys]
                facts = records.tobytes()
                mask = active[keys].tobytes()
                if schema == 1:
                    digest = hashlib.sha256(facts + mask)
                else:
                    leaf_bytes = 64 * dtype.itemsize
                    leaves = b"".join(
                        hashlib.sha256(facts[i : i + leaf_bytes]).digest()
                        for i in range(0, len(facts), leaf_bytes)
                    )
                    digest = hashlib.sha256(
                        b"cargo-map-block-v2\0" + len(keys).to_bytes(8, "little") + leaves + mask
                    )
                for identifier, members in sorted(geometry_blocks.get(block, {}).items()):
                    name = identifier.encode()
                    offsets = np.searchsorted(keys, members).astype("<u2").tobytes()
                    digest.update(
                        len(name).to_bytes(4, "little")
                        + name
                        + len(offsets).to_bytes(4, "little")
                        + offsets
                    )
                root.update(digest.digest())
                block += 1
    assert len(geometry) == len(mapper._placed_boxes), "UNSUPPORTED_PRESSURE_LEDGER"
    bindings = sorted(
        ((tuple(record["center"]), tuple(record["size"]), float(record["yaw"])), identifier)
        for identifier, record in zip(geometry, mapper._placed_boxes)
    )
    root.update(compact_json(bindings))
    root.update(compact_json(sorted(geometry)))
    snapshot = store.snapshot()
    root.update(
        compact_json(
            (
                snapshot.acquisition_stamp_ns,
                snapshot.map_source,
                snapshot.cargo_observed,
                snapshot.change_token,
                snapshot.pending_details,
                snapshot.last_settle,
                snapshot.ledger_details,
            ),
            sort_keys=True,
        )
    )
    return root.hexdigest()
