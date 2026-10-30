#include "uld_occ_observer/configuration.hpp"
#include "uld_occ_observer/interface_trace.hpp"
#include "query_content_digest.hpp"

#include "uld_occ_observer/observer.hpp"
#include "uld_occ_observer/snapshot_repository.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;

double milliseconds(Clock::time_point start, Clock::time_point end) {
  return std::chrono::duration<double, std::milli>(end - start).count();
}

std::vector<uld_occ_observer::Vec3> read_points(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw uld_occ_observer::Error("POINTS_UNAVAILABLE");
  std::vector<uld_occ_observer::Vec3> points;
  double x = 0, y = 0, z = 0;
  while (input >> x >> y >> z) points.push_back({x, y, z});
  if (points.size() != 30000) throw uld_occ_observer::Error("POINTS_NOT_30000");
  return points;
}

uld_occ_observer::OpticalObservation make_observation(const uld_occ_observer::ObserverConfiguration& config,
                                                     const std::vector<uld_occ_observer::Vec3>& points,
                                                     std::uint64_t epoch) {
  uld_occ_observer::OpticalObservation observation;
  observation.identity.sensor_id = "cam0";
  observation.identity.observation_id = "probe-1";
  observation.identity.optical_frame = "d555_color_optical_frame";
  observation.identity.clock_domain = config.clock_domain;
  observation.identity.geometry_hash = config.geometry.geometry_hash;
  observation.identity.epoch = epoch;
  observation.identity.stamp_ns = 1000000000;
  observation.uld_from_optical.stamp_ns = observation.identity.stamp_ns;
  const double rotation[9] = {0, 0, 1, -1, 0, 0, 0, -1, 0};
  for (int index = 0; index < 9; ++index) observation.uld_from_optical.rotation[index] = rotation[index];
  observation.uld_from_optical.translation = {-1.15, 0.0, 1.20};
  observation.points_m = points;
  return observation;
}

std::string content_hash(uld_occ_observer::ObservationStore& store,
                         uld_occ_observer::SnapshotRepository& repository) {
  const auto diagnostics = store.diagnostics();
  const auto lease = repository.acquire(diagnostics.identity.last_integrated_stamp_ns, 0);
  const auto snapshot = repository.get(lease.handle);
  const auto shape = store.domain()->shape();
  const auto page = snapshot->read_cells(
      {0, 0, 0}, {std::min(16, shape.x), std::min(16, shape.y), std::min(16, shape.z)}, 0, 4096);
  const auto first = snapshot->surface(nullptr);
  const auto second = snapshot->surface(nullptr);
  const auto hash = uld_occ_observer::query_content_sha256(page, first.get(), second.get());
  repository.release(lease.handle);
  return hash;
}
}  // namespace

int main(int argc, char** argv) {
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "interface_probe_cli --op integrate|acquire|roi|surface "
                 "--geometry FILE --config FILE --points FILE --out FILE\n"
              << "Times one fresh-process operation. Trace follows ULD_OCC_INTERFACE_TRACE.\n"
              << "Content hash is computed outside the timed call. Exit 0 records, 3 fails.\n";
    return 0;
  }
  try {
    if (argc != 11) throw uld_occ_observer::Error("INVALID_ARGUMENTS");
    std::map<std::string, std::string> options;
    for (int index = 1; index < argc; index += 2) {
      if (!options.emplace(argv[index], argv[index + 1]).second)
        throw uld_occ_observer::Error("DUPLICATE_OPTION");
    }
    const auto op = options.at("--op");
    if (op != "integrate" && op != "acquire" && op != "roi" && op != "surface")
      throw uld_occ_observer::Error("INVALID_OP");
    uld_occ_observer::interface_trace_configure_from_env();
    const auto config = uld_occ_observer::load_configuration(options.at("--geometry"), options.at("--config"));
    const auto points = read_points(options.at("--points"));
    uld_occ_observer::ObservationStore store(config.geometry, config.resolution, config.depth,
                                            config.clock_domain, config.digest, config.discretize_endpoints);
    uld_occ_observer::SnapshotRepository repository(store);
    const auto observation = make_observation(config, points, store.diagnostics().identity.epoch);
    double sample_ms = 0;
    double surface_first_ms = -1;
    double surface_second_ms = -1;
    std::string status = "OK";
    if (op == "integrate") {
      const auto start = Clock::now();
      const auto receipt = store.integrate(observation);
      sample_ms = milliseconds(start, Clock::now());
      status = receipt.status;
    } else {
      const auto receipt = store.integrate(observation);
      status = receipt.status;
      if (status != "INTEGRATED") throw uld_occ_observer::Error(status);
      if (op == "acquire") {
        const auto diagnostics = store.diagnostics();
        const auto start = Clock::now();
        const auto lease = repository.acquire(diagnostics.identity.last_integrated_stamp_ns, 0);
        sample_ms = milliseconds(start, Clock::now());
        repository.release(lease.handle);
      } else if (op == "roi") {
        const auto diagnostics = store.diagnostics();
        const auto lease = repository.acquire(diagnostics.identity.last_integrated_stamp_ns, 0);
        const auto snapshot = repository.get(lease.handle);
        const auto shape = store.domain()->shape();
        const auto start = Clock::now();
        const auto page = snapshot->read_cells(
            {0, 0, 0}, {std::min(16, shape.x), std::min(16, shape.y), std::min(16, shape.z)}, 0, 4096);
        sample_ms = milliseconds(start, Clock::now());
        if (page.cells.size() != 4096) throw uld_occ_observer::Error("ROI_LENGTH");
        repository.release(lease.handle);
      } else {
        const auto diagnostics = store.diagnostics();
        const auto lease = repository.acquire(diagnostics.identity.last_integrated_stamp_ns, 0);
        const auto snapshot = repository.get(lease.handle);
        const auto first_start = Clock::now();
        const auto first = snapshot->surface(nullptr);
        surface_first_ms = milliseconds(first_start, Clock::now());
        const auto second_start = Clock::now();
        const auto second = snapshot->surface(nullptr);
        surface_second_ms = milliseconds(second_start, Clock::now());
        sample_ms = surface_first_ms;
        if (first->size() != second->size() || first->empty()) throw uld_occ_observer::Error("SURFACE_LENGTH");
        repository.release(lease.handle);
      }
    }
    if (status != "INTEGRATED" && op == "integrate") throw uld_occ_observer::Error(status);
    const auto hash = content_hash(store, repository);
    const char* trace = std::getenv("ULD_OCC_INTERFACE_TRACE");
    std::ofstream output(options.at("--out"));
    if (!output) throw uld_occ_observer::Error("OUTPUT_UNAVAILABLE");
    output << "{\n  \"op\": \"" << op << "\",\n  \"trace\": \""
           << (trace && std::string(trace) == "1" ? "on" : "off")
           << "\",\n  \"sample_ms\": " << sample_ms
           << ",\n  \"surface_first_ms\": " << surface_first_ms
           << ",\n  \"surface_second_ms\": " << surface_second_ms
           << ",\n  \"content_sha256\": \"" << hash << "\",\n  \"status\": \"" << status
           << "\",\n  \"not_scored\": true\n}\n";
    uld_occ_observer::interface_trace_flush();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    uld_occ_observer::interface_trace_flush();
    return 3;
  }
}
