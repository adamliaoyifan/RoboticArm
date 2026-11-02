# OCC store state ownership

`OccupancyMapStore.apply_update` holds `_txn_lock` for the whole call:
idempotency, premise and evidence checks, voxel mutation, confirmation
transitions, revision changes, snapshot build, and query-index publish.

| State | Owner | After a rejected update | After query publish returns false |
|---|---|---|---|
| voxel grid, log-odds, locks, sensor shadow | `CargoVolumeMapper`, mutated only inside the store lock | unchanged when the mapper raises before writing | already committed for that transaction |
| epoch, processed ledger, pending confirmation | `OccupancyMapStore` | unchanged on rejection before mutation | unchanged by the publish failure itself |
| `MapSnapshot` and status bundle | store, replaced only at the end of an accepted transaction | previous snapshot remains current | `publish_status` installs the new snapshot, then indexes are cleared |
| query generations and session tokens | `QuerySnapshotRegistry` | untouched | old sessions stay; new `open` raises `RESOURCE_LIMIT` until reset |
| v1/v2/committed dicts | `occupancy_map_projection` return values | not published | rebuilt from the snapshot argument only |

`occupancy_map_projection` takes a snapshot plus explicit geometry, request,
or frame arguments. It does not read store private fields. Returned lists
are new objects.

Confirmation remains inside the store. It does not own a separate lock.
