# Shared read-only map boundary

`luggage_mapping` depends on map/geometry/message contracts, not packing or
planning. `surface` and `source` hold immutable snapshots/provenance, `admission`
owns common validity/epoch gates, and `observer` reads one exact O lease for both
surface and volume. Packing and complete-operation preparation use that reader.
Old planning import paths are compatibility exports of the same class objects.

`configuration.owner_configuration` resolves one deployment grid and writes
content-addressed immutable O geometry/config files. The integrated hardware
placement launch selects `occupancy_backend=observer` (default) or `grid_g`,
starting exactly the selected producer. Source files are not modified.

O's default service-input mode awaits an actual integration client; it does not
assert a free/ready ULD on launch. Subscription mode requires an explicit raw
optical point-cloud topic/frame. Segmented cargo points cannot substitute for
sensor rays. Sensor ingestion and the complete default operation client are
separate convergence units. G uses its native single-snapshot query path with
volume evidence enabled; G full transport and O regional access still differ.

`ObserverSnapshotPort.current` retains strict source-version semantics.
`planning_current` obtains fresh complete evidence and compares exact native
classification, active coverage and measured support heights, while requiring
the same epoch/configuration/validity generation and nondecreasing acquisition
versions. O log-odds-only revisions can therefore preserve a planning verdict;
known/unknown/occupied or support changes cannot. It never refreshes the retained
observation timestamp. Complete-operation prepare and execution share this
content comparison; changed content still requires full native recertification.
