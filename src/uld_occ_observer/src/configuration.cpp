#include "uld_occ_observer/configuration.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <sstream>
#include <set>

namespace uld_occ_observer {
namespace {
std::string read_file(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw Error("CONFIG_FILE_UNAVAILABLE");
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}
}
ObserverConfiguration load_configuration(const std::string& geometry_file,
                                         const std::string& config_file) {
  try {
    const auto geometry_bytes = read_file(geometry_file);
    const auto config_bytes = read_file(config_file);
    const auto envelope = YAML::Load(geometry_bytes);
    if (envelope["schema"].as<std::string>() != "uld_geometry_export_v1")
      throw Error("GEOMETRY_SCHEMA_MISMATCH");
    const auto canonical = envelope["descriptor_json"].as<std::string>();
    const auto digest = envelope["descriptor_sha256"].as<std::string>();
    if (sha256(canonical) != digest) throw Error("GEOMETRY_ARTIFACT_CORRUPT");
    const auto descriptor = YAML::Load(canonical);
    if (descriptor["frame_id"].as<std::string>() != "container_link" ||
        descriptor["schema_version"].as<int>() != 1) throw Error("GEOMETRY_SCHEMA_MISMATCH");
    ObserverConfiguration result;
    auto& geometry = result.geometry;
    geometry.geometry_hash = descriptor["geometry_hash"].as<std::string>();
    geometry.artifact_digest = digest;
    geometry.length = descriptor["length"].as<double>();
    geometry.width = descriptor["width"].as<double>();
    geometry.floor_z = descriptor["floor_z"].as<double>();
    geometry.ceiling_z = descriptor["ceiling_z"].as<double>();
    if (descriptor["chamfer"]) {
      const auto c = descriptor["chamfer"];
      if (c["side"].as<std::string>() != "positive_y") throw Error("UNSUPPORTED_CHAMFER");
      geometry.chamfer = Chamfer{c["floor_y"].as<double>(), c["wall_y"].as<double>(), c["wall_z"].as<double>()};
    }
    const auto config = YAML::Load(config_bytes);
    const std::set<std::string> allowed{"resolution_m", "depth_z_min_m", "depth_z_max_m", "clock_domain"};
    for (const auto& item : config)
      if (!allowed.count(item.first.as<std::string>())) throw Error("UNKNOWN_CONFIG_FIELD");
    result.resolution = config["resolution_m"].as<double>();
    result.depth = {config["depth_z_min_m"].as<double>(), config["depth_z_max_m"].as<double>()};
    result.depth.validate();
    result.clock_domain = config["clock_domain"].as<std::string>();
    if (result.clock_domain.empty()) throw Error("INVALID_CLOCK_DOMAIN");
    result.digest = sha256(geometry_bytes + "\n" + config_bytes + "\nprobability-profile-v1");
    UldDomain validate(geometry, result.resolution);
    return result;
  } catch (const YAML::Exception&) { throw Error("INVALID_CONFIGURATION_FORMAT"); }
}
}  // namespace uld_occ_observer
