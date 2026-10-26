#pragma once
#include "uld_occ_observer/snapshot_repository.hpp"
#include <yaml-cpp/yaml.h>
namespace uld_occ_observer {
OpticalObservation observation_from_yaml(const YAML::Node& input);
YAML::Node identity_to_yaml(const SnapshotIdentity& identity);
YAML::Node receipt_to_yaml(const UpdateReceipt& receipt);
YAML::Node cells_to_yaml(const CellPage& page);
YAML::Node surface_to_yaml(const std::vector<SurfaceColumn>& surface);
}  // namespace uld_occ_observer
