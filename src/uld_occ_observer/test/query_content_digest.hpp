#pragma once
#include "uld_occ_observer/snapshot.hpp"
#include <string>

namespace uld_occ_observer {
// Validates every node and parent before lossless pruning of a deserialized copy.
// Throws std::runtime_error for invalid trees; never repairs occupancy.
std::string canonical_probability_digest(const std::string& tree_bytes);
// Throws on a failed digest invariant; called by FusionProfileChecks.
void check_probability_digest();

// Hash of query results the caller already holds. This does not read the map.
std::string query_content_sha256(const CellPage& roi,
                                 const std::vector<SurfaceColumn>* surface_first,
                                 const std::vector<SurfaceColumn>* surface_second);
}  // namespace uld_occ_observer
