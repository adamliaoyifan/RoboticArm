#include "uld_occ_observer/configuration.hpp"
#include "uld_occ_observer/interface_trace.hpp"
#include "uld_occ_observer/checkpoint_io.hpp"
#include "uld_occ_observer/serialization.hpp"
#include "query_content_digest.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sys/resource.h>
#include <ctime>

using namespace uld_occ_observer;
namespace {
using Clock = std::chrono::steady_clock;
struct BoundRequest {
  explicit BoundRequest(const std::string& id) { interface_trace_bind_request(id.c_str()); }
  ~BoundRequest() { interface_trace_bind_request(""); }
};
double ms(Clock::time_point a, Clock::time_point b) {
  return std::chrono::duration<double,std::milli>(b-a).count();
}
std::string read_bytes(const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw Error("INPUT_UNAVAILABLE:"+path.string());
  return {std::istreambuf_iterator<char>(input),std::istreambuf_iterator<char>()};
}
}
int main(int argc, char** argv) {
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "observer_profile_cli --geometry FILE --config FILE --manifest FILE --out FILE\n"
      << "Manifest: mode(off/stages/detailed), case_id, frames[{path,sha256,warmup}],\n"
      << "optional surface(true), state_digest(false). state_digest also hashes the\n"
      << "already queried ROI and surface outside integrate and query clocks.\n"
      << "Outputs append-flushed YAML documents.\n"
      << "Input frames use observer_cli integrate schema; no implicit IDs or TF.\n"
      << "Exit0=recording completed,3=error/status mismatch; performance is not scored here.\n";
    return 0;
  }
  try {
    if (argc != 9) throw Error("INVALID_ARGUMENTS");
    std::map<std::string,std::string> opt;
    for (int i=1;i<argc;i+=2) if (!opt.emplace(argv[i],argv[i+1]).second) throw Error("DUPLICATE_OPTION");
    for (const auto& entry : opt)
      if (entry.first!="--geometry" && entry.first!="--config" && entry.first!="--manifest" && entry.first!="--out")
        throw Error("UNKNOWN_OPTION");
    const std::filesystem::path manifest_path = opt.at("--manifest");
    const auto manifest_bytes = read_bytes(manifest_path);
    const auto manifest = YAML::Load(manifest_bytes);
    const auto mode_name = manifest["mode"].as<std::string>();
    const auto mode = mode_name=="off" ? ProfileMode::Off : mode_name=="stages" ? ProfileMode::Stages :
      mode_name=="detailed" ? ProfileMode::Detailed : throw Error("INVALID_PROFILE_MODE");
    if (!manifest["frames"].IsSequence() || !manifest["frames"].size()) throw Error("NO_FRAMES");
    const auto cfg = load_configuration(opt.at("--geometry"),opt.at("--config"));
    interface_trace_configure_from_env();
    ObservationStore store(cfg.geometry,cfg.resolution,cfg.depth,cfg.clock_domain,cfg.digest,
                           cfg.discretize_endpoints);
    SnapshotRepository repo(store);
    if (std::filesystem::exists(opt.at("--out"))) throw Error("OUTPUT_ALREADY_EXISTS");
    std::ofstream output(opt.at("--out"));
    if (!output) throw Error("OUTPUT_UNAVAILABLE");
    bool failed = false; std::size_t index=0;
    const auto run_started = Clock::now();
    for (const auto& entry : manifest["frames"]) {
      const auto sample_start = Clock::now();
      YAML::Node row;
      row["schema"] = "observer_profile_v1";
      row["case_id"] = manifest["case_id"].as<std::string>();
      row["manifest_sha256"] = sha256(manifest_bytes);
      row["index"] = index++;
      row["warmup"] = entry["warmup"].as<bool>(false);
      row["mode"] = mode_name;
      row["monotonic_start_ns"] = std::chrono::duration_cast<std::chrono::nanoseconds>(sample_start.time_since_epoch()).count();
      row["status"] = "OK";
      try {
        const auto path = manifest_path.parent_path()/entry["path"].as<std::string>();
        const auto bytes = read_bytes(path);
        row["input_path"] = path.string(); row["input_sha256"] = sha256(bytes);
        if (row["input_sha256"].as<std::string>() != entry["sha256"].as<std::string>()) throw Error("INPUT_HASH_MISMATCH");
        const auto request = YAML::Load(bytes);
        const auto observation = observation_from_yaml(request);
        BoundRequest bound_request(observation.identity.observation_id);
        const auto before = Clock::now();
        row["input_load_parse_ms"] = ms(sample_start,before);
        const auto cpu_before = std::clock();
        const auto receipt = store.integrate(observation,mode);
        const auto after = Clock::now();
        row["integrate_external_ms"] = ms(before,after);
        row["integrate_process_cpu_ms"] = 1000.0*(std::clock()-cpu_before)/CLOCKS_PER_SEC;
        row["receipt"] = receipt_to_yaml(receipt);
        if (receipt.status != entry["expect_status"].as<std::string>("INTEGRATED")) {
          row["status"] = "UNEXPECTED_RECEIPT_STATUS"; failed = true;
        }
        const auto diagnostics = store.diagnostics();
        row["tree_nodes"] = diagnostics.tree_nodes;
        row["known_cells"] = diagnostics.known_cells;
        row["block_count"] = diagnostics.blocks;
        row["cell_block_bytes"] = sizeof(CellBlock);
        row["tree_memory_estimate_bytes"] = diagnostics.estimated_tree_bytes;
        CellPage roi_page;
        std::shared_ptr<const std::vector<SurfaceColumn>> surface_first;
        std::shared_ptr<const std::vector<SurfaceColumn>> surface_second;
        bool queried = false;
        if (diagnostics.identity.last_integrated_stamp_ns) {
          const auto acquire_start = Clock::now();
          const auto lease = repo.acquire(diagnostics.identity.last_integrated_stamp_ns,0);
          const auto acquired = Clock::now();
          const auto snap = repo.get(lease.handle);
          const auto got = Clock::now();
          const auto shape = store.domain()->shape();
          roi_page = snap->read_cells({0,0,0},{std::min(16,shape.x),std::min(16,shape.y),std::min(16,shape.z)},0,4096);
          const auto read = Clock::now();
          row["acquire_ms"] = ms(acquire_start,acquired); row["get_handle_ms"] = ms(acquired,got);
          row["roi_ms"] = ms(got,read); row["roi_cells"] = roi_page.cells.size();
          row["snapshot_identity"] = identity_to_yaml(snap->identity());
          if (manifest["surface"].as<bool>(true)) {
            for (int call=0;call<2;++call) {
              SurfaceProfile profile;
              const auto begin = Clock::now();
              const auto surface = snap->surface(&profile);
              const auto end = Clock::now();
              if (call == 0) surface_first = surface;
              else surface_second = surface;
              YAML::Node sample;
              sample["external_ms"] = ms(begin,end);
              sample["cache_hit"] = profile.cache_hit;
              sample["profile_enabled"] = mode!=ProfileMode::Off;
              if (mode!=ProfileMode::Off) sample["profile"] = surface_profile_to_yaml(profile);
              sample["columns"] = surface->size();
              row[call==0 ? "surface_first_call" : "surface_second_call"] = sample;
            }
          }
          repo.release(lease.handle);
          queried = true;
        }
        if (manifest["state_digest"].as<bool>(false)) {
          const auto begin = Clock::now();
          const auto tree_bytes = store.checkpoint().tree_bytes;
          row["probability_tree_sha256"] = sha256(tree_bytes);
          row["checkpoint_digest_ms"] = ms(begin,Clock::now());
          const auto canonical_begin = Clock::now();
          row["canonical_probability_sha256"] = canonical_probability_digest(tree_bytes);
          row["canonical_probability_digest_ms"] = ms(canonical_begin, Clock::now());
          if (queried) {
            const auto content_begin = Clock::now();
            row["roi_cell_count"] = roi_page.cells.size();
            row["surface_column_count"] = surface_first ? surface_first->size() : 0;
            row["surface_second_column_count"] = surface_second ? surface_second->size() : 0;
            row["roi_surface_content_sha256"] = query_content_sha256(
                roi_page, surface_first.get(), surface_second.get());
            row["content_digest_ms"] = ms(content_begin, Clock::now());
          }
        }
      } catch (const std::exception& error) { row["status"] = error.what(); failed = true; }
      struct rusage usage{}; getrusage(RUSAGE_SELF,&usage);
      row["process_peak_rss_kib"] = usage.ru_maxrss;  // peak, explicitly not current RSS
      row["sample_compute_ms"] = ms(sample_start,Clock::now());
      output << "---\n" << YAML::Dump(row) << '\n'; output.flush();
      if (!output) throw Error("OUTPUT_WRITE_FAILED");
      if (failed) break;  // preserve first failure; do not continue from an ambiguous state
    }
    std::cout << "recorded=" << index << " run_wall_ms=" << ms(run_started,Clock::now())
              << " performance_verdict=not_evaluated\n";
    interface_trace_flush();
    return failed ? 3 : 0;
  } catch (const std::exception& error) {
    interface_trace_flush();
    std::cerr << error.what() << '\n';
    return 3;
  }
}
