#include "uld_occ_observer/configuration.hpp"
#include "uld_occ_observer/checkpoint_io.hpp"
#include "uld_occ_observer/serialization.hpp"
#include "uld_occ_observer/freespace_query.hpp"
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
    ObservationStore store(config.geometry,config.resolution,config.depth,config.clock_domain,config.digest,
                           config.discretize_endpoints);
    SnapshotRepository repository(store);
    const auto commands = YAML::LoadFile(options.at("--commands"));
    if (!commands.IsSequence()) throw Error("INVALID_COMMAND_SEQUENCE");
    YAML::Node output(YAML::NodeType::Sequence);
    std::string last_handle;
    bool failed = false;
    // The artifact is written at the end of the run (and from this guard on
    // any early exit), not after every command: thousands-of-commands F
    // manifests made the per-command full rewrite quadratic in file IO. A
    // hard crash mid-run loses the tail; a handled failure keeps everything.
    struct WriteAtExit {
      YAML::Node* output;
      std::ofstream* file;
      ~WriteAtExit() { if (file && file->is_open()) *file << YAML::Dump(*output); }
    } writer{&output, nullptr};
    std::ofstream file(options.at("--out"));
    if (!file) throw Error("OUTPUT_WRITE_FAILED");
    writer.file = &file;
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
        } else if (operation == "freespace") {
          // Full-volume known-free candidate query through the same core the
          // ROS service uses. Pages follow the deterministic enumeration, so
          // a manifest can walk the whole feasible list with cursor/limit.
          const auto snapshot = repository.get(command["handle"].as<std::string>(last_handle));
          FreespaceRequest request;
          const auto box = command["box"].as<std::vector<double>>();
          if (box.size() != 3) throw Error("INVALID_BOX");
          request.box_length_m = box[0];
          request.box_width_m = box[1];
          request.box_height_m = box[2];
          request.yaw_mode = command["yaw_mode"].as<int>(2);
          request.cursor = command["cursor"].as<std::size_t>(0);
          request.limit = command["limit"].as<std::size_t>(4096);
          const auto page = query_freespace(*snapshot, request);
          response["identity"] = identity_to_yaml(snapshot->identity());
          response["next_cursor"] = page.next_cursor;
          response["done"] = page.done;
          response["grid_total"] = page.grid_total;
          response["feasible_total"] = page.feasible_total;
          response["rejected_out_of_domain"] = page.rejected_out_of_domain;
          response["rejected_collision"] = page.rejected_collision;
          response["rejected_clearance"] = page.rejected_clearance;
          response["rejected_unknown"] = page.rejected_unknown;
          response["rejected_support"] = page.rejected_support;
          YAML::Node list(YAML::NodeType::Sequence);
          for (const auto& candidate : page.candidates) {
            YAML::Node entry;
            entry["cell"].push_back(candidate.cell_x);
            entry["cell"].push_back(candidate.cell_y);
            entry["cell"].push_back(candidate.cell_z);
            entry["origin"].push_back(candidate.origin_x);
            entry["origin"].push_back(candidate.origin_y);
            entry["origin"].push_back(candidate.origin_z);
            entry["yaw_deg"] = candidate.yaw_deg;
            entry["support_ratio"] = candidate.support_ratio;
            entry["support_kind"] = candidate.support_kind;
            entry["verify_begin"].push_back(candidate.verify_begin[0]);
            entry["verify_begin"].push_back(candidate.verify_begin[1]);
            entry["verify_begin"].push_back(candidate.verify_begin[2]);
            entry["verify_end"].push_back(candidate.verify_end[0]);
            entry["verify_end"].push_back(candidate.verify_end[1]);
            entry["verify_end"].push_back(candidate.verify_end[2]);
            list.push_back(entry);
          }
          response["candidates"] = list;
          const auto& domain = *snapshot->domain();
          response["policy"] = std::map<std::string, double>{
              {"support_tol_m", PlacementPolicy{}.support_tol_m},
              {"min_support_ratio", PlacementPolicy{}.min_support_ratio},
              {"clearance_margin_m", PlacementPolicy{}.clearance_margin_m},
              {"same_layer_clearance_m", PlacementPolicy{}.same_layer_clearance_m},
              {"resolution", domain.resolution()}};
          // Same honesty rule as the service: NO_DATA / NO_KNOWN_FREE are
          // semantic answers, not failures -- the CLI's expect_status pins
          // whichever the manifest means to observe.
          const auto semantic = snapshot->known_entry_count() == 0 ? "NO_DATA"
              : page.feasible_total == 0 ? "NO_KNOWN_FREE" : "OK";
          response["semantic_status"] = semantic;
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
    }
    file << YAML::Dump(output);
    if (!file) throw Error("OUTPUT_WRITE_FAILED");
    return failed ? 3 : 0;
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 3; }
}
