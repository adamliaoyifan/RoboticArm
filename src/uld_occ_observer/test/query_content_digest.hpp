#pragma once
#include "uld_occ_observer/snapshot.hpp"
#include <string>

namespace uld_occ_observer {
// Hash of query results the caller already holds. This does not read the map.
std::string query_content_sha256(const CellPage& roi,
                                 const std::vector<SurfaceColumn>* surface_first,
                                 const std::vector<SurfaceColumn>* surface_second);
}  // namespace uld_occ_observer
