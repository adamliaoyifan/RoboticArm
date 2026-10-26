#include "uld_occ_observer/configuration.hpp"
#include "uld_occ_observer/checkpoint_io.hpp"
#include "uld_occ_observer/serialization.hpp"
#include <fstream>
#include <iostream>

using namespace uld_occ_observer;
namespace {
CellIndex index(const YAML::Node& value) {
  const auto values = value.as<std::vector<int>>();
  if (values.size() != 3) throw Error("INVALID_INDEX");
  return {values[0],values[1],values[2]};
}
}
int main(int argc, char** argv) {
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "observer_cli --geometry EXPORT.yaml --config CONFIG.yaml --commands REQUESTS.yaml --out RESPONSES.yaml\n"
              << "Requests: integrate, acquire, cells, surface, release, reset, checkpoint, restore, diagnostics.\n"
              << "Geometry export and optical meters/stamps/TF are mandatory; no Gazebo or ROS required.\n";
    return 0;
  }
  try {
    std::map<std::string,std::string> options;
    if (argc != 9) throw Error("INVALID_ARGUMENTS_USE_HELP");
    for (int i = 1; i < argc; i += 2) options.emplace(argv[i],argv[i+1]);
    const auto config = load_configuration(options.at("--geometry"), options.at("--config"));
    ObservationStore store(config.geometry,config.resolution,config.depth,config.clock_domain,config.digest);
    SnapshotRepository repository(store);
    const auto commands = YAML::LoadFile(options.at("--commands"));
    if (!commands.IsSequence()) throw Error("INVALID_COMMAND_SEQUENCE");
    YAML::Node output(YAML::NodeType::Sequence);
    std::string last_handle;
    bool failed = false;
    for (const auto& command : commands) {
      YAML::Node response;
      try {
        const auto operation = command["op"].as<std::string>();
        response["op"] = operation;
        response["status"] = "OK";
        if (operation == "integrate") {
          const auto receipt = store.integrate(observation_from_yaml(command));
          response = receipt_to_yaml(receipt);
          response["op"] = operation;
        } else if (operation == "acquire") {
          const auto lease = repository.acquire(command["now_ns"].as<std::int64_t>(),
              command["max_age_ns"].as<std::int64_t>(1000000000),
              command["allow_stale"].as<bool>(false), command["expected_epoch"].as<std::uint64_t>(0),
              command["expected_revision"] ? std::optional<std::uint64_t>(command["expected_revision"].as<std::uint64_t>()) : std::nullopt);
          last_handle = lease.handle; response["handle"] = lease.handle;
          response["identity"] = identity_to_yaml(lease.identity);
          response["age_ns"] = lease.age_ns; response["stale"] = lease.stale;
        } else if (operation == "cells" || operation == "surface") {
          const auto snapshot = repository.get(command["handle"].as<std::string>(last_handle));
          response["identity"] = identity_to_yaml(snapshot->identity());
          if (operation == "cells") response["data"] = cells_to_yaml(snapshot->read_cells(
              index(command["begin"]),index(command["end"]),command["cursor"].as<std::size_t>(0),command["limit"].as<std::size_t>(4096)));
          else response["data"] = surface_to_yaml(*snapshot->surface());
        } else if (operation == "release") repository.release(command["handle"].as<std::string>(last_handle));
        else if (operation == "reset") response["epoch"] = store.reset(command["expected_epoch"].as<std::uint64_t>());
        else if (operation == "checkpoint") write_checkpoint(store.checkpoint(),command["path"].as<std::string>());
        else if (operation == "restore") response["epoch"] = store.restore(read_checkpoint(command["path"].as<std::string>()),command["expected_epoch"].as<std::uint64_t>());
        else if (operation == "diagnostics") {
          const auto d = store.diagnostics(); response["identity"] = identity_to_yaml(d.identity);
          response["tree_nodes"] = d.tree_nodes; response["known_cells"] = d.known_cells;
          response["blocks"] = d.blocks; response["estimated_tree_bytes"] = d.estimated_tree_bytes;
          response["received"] = d.received; response["integrated"] = d.integrated;
          response["rejected"] = d.rejected; response["duplicates"] = d.duplicates;
          response["full_serializations"] = d.full_serializations;
          response["octomap_version"] = "1.9.8";
        } else throw Error("UNSUPPORTED_OPERATION");
      } catch (const std::exception& error) { response["status"] = error.what(); }
      const auto status = response["status"].as<std::string>();
      const auto expected = command["expect_status"].as<std::string>(command["op"].as<std::string>() == "integrate" ? "INTEGRATED" : "OK");
      response["expected_status"] = expected;
      if (status != expected) failed = true;
      output.push_back(response);
      // Preserve each completed response even when a later command fails.
      std::ofstream file(options.at("--out")); file << YAML::Dump(output);
      if (!file) throw Error("OUTPUT_WRITE_FAILED");
    }
    return failed ? 3 : 0;
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 3; }
}
