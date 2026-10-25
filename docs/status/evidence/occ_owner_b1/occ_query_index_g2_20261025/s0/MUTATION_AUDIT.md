# Frozen baseline write-path audit

Base 352191b3, plan b9f4b2dc. Primary buffers remain bytearray, array(q/d), LogOddsGrid Python lists. No storage migration.

| Boundary | Fields potentially changed | Delta capture |
|---|---|---|
| mark_occupied_voxel | state/source/label/instance/logodds/lock/shadow | before live mutation |
| mark_free_world | state/source/label/instance/logodds/shadow | before live mutation |
| integrate_points | hits and misses, same fields | before batch mutation; no reordered fusion |
| integrate_depth_view (existing parked path) | same evidence; depth phase unchanged | before batch mutation; never enable channel |
| _rasterize_placed_box | state/source/logodds/lock | before rasterization |
| _retract_box_voxels | live/source/identity/logodds/lock restored from shadow | before retraction |
| reset/_clear_grid including preserve_placed | all live/shadow | explicit rebuild |
| fill_unoccupied_as_free | state only, legacy evidence unchanged | before assignment |
| note_external_change/pending/confirmation | identity only | empty delta, same publication |

Store apply_update remains sole external writer. Replays/rejections never publish a new bundle. Legacy frontier iteration is ix,iy,iz, unlike flat-index order required by the new API; retain legacy order separately. Label/source changes without state changes must remain visible. No occupancy, sampling, acceptance or planning change is authorized.
