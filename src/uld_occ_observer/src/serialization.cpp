#include "uld_occ_observer/serialization.hpp"
namespace uld_occ_observer {
OpticalObservation observation_from_yaml(const YAML::Node& input) {
  OpticalObservation result;
  auto& id = result.identity;
  id.sensor_id = input["sensor_id"].as<std::string>();
  id.observation_id = input["observation_id"].as<std::string>();
  id.optical_frame = input["optical_frame"].as<std::string>();
  id.clock_domain = input["clock_domain"].as<std::string>();
  id.geometry_hash = input["geometry_hash"].as<std::string>();
  id.epoch = input["epoch"].as<std::uint64_t>();
  id.stamp_ns = input["stamp_ns"].as<std::int64_t>();
  const auto tf = input["transform"];
  result.uld_from_optical.stamp_ns = tf["stamp_ns"].as<std::int64_t>();
  const auto rotation = tf["rotation"].as<std::vector<double>>();
  const auto translation = tf["translation"].as<std::vector<double>>();
  if (rotation.size() != 9 || translation.size() != 3) throw Error("INVALID_TRANSFORM_FORMAT");
  std::copy(rotation.begin(), rotation.end(), result.uld_from_optical.rotation.begin());
  result.uld_from_optical.translation = {translation[0],translation[1],translation[2]};
  if (!input["points_m"].IsSequence()) throw Error("INVALID_POINT_FORMAT");
  for (const auto& item : input["points_m"]) {
    const auto point = item.as<std::vector<double>>();
    if (point.size() != 3) throw Error("INVALID_POINT_FORMAT");
    result.points_m.push_back({point[0],point[1],point[2]});
  }
  return result;
}
YAML::Node identity_to_yaml(const SnapshotIdentity& id) {
  YAML::Node out;
  out["epoch"] = id.epoch; out["map_revision"] = id.map_revision;
  out["observed_revision"] = id.observed_revision;
  out["last_integrated_stamp_ns"] = id.last_integrated_stamp_ns;
  out["geometry_hash"] = id.geometry_hash; out["config_digest"] = id.config_digest;
  out["clock_domain"] = id.clock_domain; return out;
}
YAML::Node receipt_to_yaml(const UpdateReceipt& r) {
  auto out = identity_to_yaml(r.identity);
  out["sensor_id"] = r.sensor_id; out["observation_id"] = r.observation_id; out["input_stamp_ns"] = r.input_stamp_ns;
  out["status"] = r.status; out["raw_points"] = r.raw_points;
  out["accepted_points"] = r.accepted_points; out["free_keys"] = r.free_keys;
  out["hit_keys"] = r.hit_keys; out["changed_keys"] = r.changed_keys;
  out["occupancy_changed_keys"] = r.occupancy_changed_keys;
  out["rebuilt_blocks"] = r.rebuilt_blocks;
  out["preprocessing_ms"] = r.preprocessing_ms; out["preparation_ms"] = r.preparation_ms;
  out["fusion_ms"] = r.fusion_ms; out["total_ms"] = r.total_ms; return out;
}
YAML::Node cells_to_yaml(const CellPage& page) {
  YAML::Node out;
  out["next_cursor"] = page.next_cursor; out["done"] = page.done;
  out["cells"] = YAML::Node(YAML::NodeType::Sequence);
  for (const auto& cell : page.cells) {
    YAML::Node row;
    row["index"] = std::vector<int>{cell.index.x,cell.index.y,cell.index.z};
    row["state"] = !cell.active ? "out_of_domain" : !cell.known ? "unknown" :
                    cell.value.occupied ? "occupied" : "free";
    if (cell.known) row["log_odds"] = cell.value.log_odds;
    out["cells"].push_back(row);
  }
  return out;
}
YAML::Node surface_to_yaml(const std::vector<SurfaceColumn>& surface) {
  YAML::Node out(YAML::NodeType::Sequence);
  for (const auto& column : surface) {
    YAML::Node row;
    row["x"] = column.x; row["y"] = column.y;
    row["active_count"] = column.active_count; row["known_count"] = column.known_count;
    row["unknown_present"] = column.known_count < column.active_count;
    row["has_occupied"] = column.has_occupied;
    if (column.has_occupied) row["top_z"] = column.top_z;
    out.push_back(row);
  }
  return out;
}
}  // namespace uld_occ_observer
