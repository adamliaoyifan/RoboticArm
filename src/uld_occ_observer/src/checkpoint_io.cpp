#include "uld_occ_observer/checkpoint_io.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <filesystem>
#include <unistd.h>

namespace uld_occ_observer {
void write_checkpoint(const Checkpoint& c, const std::string& path) {
  YAML::Node payload;
  payload["schema"] = "uld_observer_checkpoint_v1";
  const auto& id = c.identity;
  payload["epoch"] = id.epoch;
  payload["map_revision"] = id.map_revision;
  payload["observed_revision"] = id.observed_revision;
  payload["stamp_ns"] = id.last_integrated_stamp_ns;
  payload["geometry_hash"] = id.geometry_hash;
  payload["config_digest"] = id.config_digest;
  payload["clock_domain"] = id.clock_domain;
  payload["tree"] = YAML::Binary(reinterpret_cast<const unsigned char*>(c.tree_bytes.data()), c.tree_bytes.size());
  payload["sensors"] = YAML::Node(YAML::NodeType::Sequence);
  for (const auto& entry : c.sensor_stamps) {
    YAML::Node sensor; sensor["id"] = entry.first; sensor["stamp_ns"] = entry.second;
    payload["sensors"].push_back(sensor);
  }
  payload["dedup"] = YAML::Node(YAML::NodeType::Sequence);
  for (const auto& key : c.fingerprint_order) {
    YAML::Node item; item["sensor"] = key.first; item["id"] = key.second;
    item["digest"] = c.fingerprints.at(key); payload["dedup"].push_back(item);
  }
  const auto bytes = YAML::Dump(payload);
  YAML::Node envelope;
  envelope["payload"] = bytes; envelope["sha256"] = sha256(bytes);
  const auto temporary = path + ".tmp." + std::to_string(getpid());
  if (std::filesystem::exists(path)) throw Error("CHECKPOINT_ALREADY_EXISTS");
  { std::ofstream output(temporary, std::ios::binary);
    output << YAML::Dump(envelope); output.flush();
    if (!output) throw Error("CHECKPOINT_WRITE_FAILED"); }
  std::filesystem::rename(temporary, path);
}
Checkpoint read_checkpoint(const std::string& path) {
  try {
    const auto envelope = YAML::LoadFile(path);
    const auto payload_bytes = envelope["payload"].as<std::string>();
    if (sha256(payload_bytes) != envelope["sha256"].as<std::string>())
      throw Error("CHECKPOINT_CHECKSUM_MISMATCH");
    const auto payload = YAML::Load(payload_bytes);
    if (payload["schema"].as<std::string>() != "uld_observer_checkpoint_v1")
      throw Error("CHECKPOINT_SCHEMA_MISMATCH");
    Checkpoint c;
    c.identity = {payload["epoch"].as<std::uint64_t>(), payload["map_revision"].as<std::uint64_t>(),
        payload["observed_revision"].as<std::uint64_t>(), payload["stamp_ns"].as<std::int64_t>(),
        payload["geometry_hash"].as<std::string>(), payload["config_digest"].as<std::string>(),
        payload["clock_domain"].as<std::string>()};
    const auto tree = payload["tree"].as<YAML::Binary>();
    c.tree_bytes.assign(reinterpret_cast<const char*>(tree.data()), tree.size());
    if (!payload["sensors"].IsSequence() || !payload["dedup"].IsSequence())
      throw Error("CHECKPOINT_SIDECAR_MISSING");
    for (const auto& item : payload["sensors"]) {
      if (!c.sensor_stamps.emplace(item["id"].as<std::string>(), item["stamp_ns"].as<std::int64_t>()).second)
        throw Error("INVALID_CHECKPOINT_STAMP");
    }
    for (const auto& item : payload["dedup"]) {
      const auto key = std::make_pair(item["sensor"].as<std::string>(), item["id"].as<std::string>());
      if (!c.fingerprints.emplace(key, item["digest"].as<std::string>()).second)
        throw Error("INVALID_CHECKPOINT_DEDUP");
      c.fingerprint_order.push_back(key);
    }
    return c;
  } catch (const YAML::Exception&) { throw Error("INVALID_CHECKPOINT_FORMAT"); }
}
}  // namespace uld_occ_observer
