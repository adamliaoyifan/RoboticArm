#pragma once
#include "uld_occ_observer/observer.hpp"
namespace uld_occ_observer {
// One checksummed YAML bundle atomically renamed; no partially paired sidecar/tree.
void write_checkpoint(const Checkpoint& checkpoint, const std::string& path);
Checkpoint read_checkpoint(const std::string& path);
}  // namespace uld_occ_observer
