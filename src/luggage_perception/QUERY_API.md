# Cargo map query API, schema 1

`OccupancyMapStore.apply_update` owns facts and derived index publication.
Consumers acquire immutable `MapQuerySnapshot` handles; no query can write
the mapper, change fusion, renew a runtime lease or create observed facts.
The legacy content digest retains its meaning. The query identity adds a
canonical voxel digest and index schema version to the legacy three legs.

Python reads include summary, cell/ROI/frontier/instance pages, exact
point-to-voxel-AABB distances, full surface and bounded surface changes.
Coordinates are `container_link`. Cell ROI intersection has positive volume;
surface ROI selects whole columns by positive XY intersection. Surface
projection includes inactive-column behavior from the legacy full oracle.
Pages use stable x-fastest flat-index order; legacy frontier enumeration
keeps its old order. Limits default to 1024 and cannot exceed 4096.

There are four typed ROS services:

| Service | Read operation |
|---|---|
| `/cargo_map/get_summary_v2` | Frozen counts/provenance/confirmation |
| `/cargo_map/query_cells_v2` | Cells, frontiers, instances, frontier count, one cell |
| `/cargo_map/query_distance_v2` | Occupied and optionally separate unknown distance |
| `/cargo_map/get_surface_patch_v2` | Full surface arrays or bounded column patch |

Every request supplies a nonempty `request_id` (at most 128 characters),
`frame_id=container_link`, and either `use_current=true` with an empty
expected identity/token, or a full five-leg expected identity. World-frame
queries are rejected; the adapter does not fetch latest TF. Response metadata
echoes the request and frozen identity/grid, completion, cursor, work and time.
Error responses have a named status/reason and `complete=false`.

The first request may return a `snapshot_token`. Continue pages with that
token, its full response identity, and `use_current=false`; the token can
resolve the preceding retained generation even after CURRENT advances.
There are at most two retained generations, 128 sessions and a five-second
monotonic lifetime. Resuming never extends it. Set `release_after_query=true`
to close explicitly, including after partial reads; a returned cursor then
does not retain a session. Invalid queries do not close an existing valid
token unless release was explicitly requested. Expired data never silently
switches to CURRENT. Resource limits refuse new sessions/publications by name.

Cell states and sources and instance kinds are typed enums. Sensor instances
require an explicit `legacy:<epoch>` namespace until a producer binding exists.
Committed-box instances require an actual existing box identity; ledger
positions are not identities. Missing/ambiguous bindings are named failures.

Distance policy `SEPARATE_UNKNOWN` returns independent occupied and unknown
results. `OCCUPIED_ONLY` leaves unknown unqueried. MATCH is exact to voxel
volume with smallest-flat-index tie breaking. NO_MATCH proves no target cells;
RADIUS_CLEAR has no exact distance and carries only its proved lower bound.
Outside/budget errors never certify safety. Defaults: up to 256 points,
radius .30m (maximum map diagonal), two million examined cells, 50,000 blocks
and 200ms. A Python `DistanceBudget` can only reduce work/time limits. This
point query does not replace payload or hull collision certification.

Full surface arrays have local index `x + shape_x*y`, with explicit validity
for nullable v2 heights; invalid wire heights are canonical zero and cannot be
treated as support. Patch records have global XY indices and a retained,
same-epoch `since_identity`. Both nominal legacy fields and v2 datum fields
are read from the same frozen publication.
