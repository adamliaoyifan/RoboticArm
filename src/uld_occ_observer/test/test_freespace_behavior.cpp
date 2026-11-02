// Free-space query core behavior tests (unified delivery contract §2/§3).
//
// Scenes are built with tightly aimed synthetic rays so every scenario knows
// exactly which cells are observed free, observed occupied, or unknown:
//  - downward boresight  R=[1,0,0; 0,-1,0; 0,0,-1]: optical z = O.z - T.z,
//    so every endpoint at least 0.52 m below the sensor passes the depth
//    filter; the ray sweeps its own column above the hit.
//  - +x boresight        R=[0,-1,0; 0,0,-1; 1,0,0]: optical z = T.x - O.x,
//    so a ray at fixed (y, z) from x=-1.15 to the far wall sweeps exactly
//    one row of one z layer and leaves its endpoint occupied AT THE WALL,
//    never inside the patch under test.
// A brute-force per-candidate re-check (cell reads, no prefix sums)
// verifies every returned candidate; the Python oracle in test/benchmark
// does the full-set equality for the 360-case F matrix.
#include "uld_occ_observer/configuration.hpp"
#include "uld_occ_observer/freespace_query.hpp"
#include "uld_occ_observer/snapshot_repository.hpp"
#include "uld_occ_observer/resource_usage.hpp"
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

using namespace uld_occ_observer;

namespace {
int failures = 0;
void expect(bool value, const char* message) {
  if (value) return;
  std::cerr << "FAIL " << message << '\n';
  ++failures;
}
ObserverConfiguration load() {
  const char* geometry = std::getenv("OCC_FS_GEOMETRY");
  const char* config = std::getenv("OCC_FS_CONFIG");
  if (!geometry) geometry = "/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run01/fixtures/geometry_chamfer.yaml";
  if (!config) config = "/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run01/fixtures/config_chamfer_r0p02.yaml";
  return load_configuration(geometry, config);
}
const std::array<double, 9> kDown = {1, 0, 0, 0, -1, 0, 0, 0, -1};
// Rows {0,0,1; 0,-1,0; 1,0,0}: the optical z column is ULD +x, so optical.z
// equals the +x travel of a ray and every endpoint on the far side passes
// the depth filter.
const std::array<double, 9> kPlusX = {0, 0, 1, 0, -1, 0, 1, 0, 0};
OpticalObservation fan(const ObserverConfiguration& cfg, const std::string& id,
                       std::int64_t stamp, const Vec3& origin,
                       const std::array<double, 9>& rotation,
                       const std::vector<Vec3>& targets,
                       std::uint64_t epoch = 1) {
  OpticalObservation item;
  item.identity.sensor_id = "cam0";
  item.identity.observation_id = id;
  item.identity.optical_frame = "synthetic";
  item.identity.clock_domain = cfg.clock_domain;
  item.identity.geometry_hash = cfg.geometry.geometry_hash;
  item.identity.epoch = epoch;
  item.identity.stamp_ns = stamp;
  item.uld_from_optical.rotation = rotation;
  item.uld_from_optical.translation = origin;
  item.uld_from_optical.stamp_ns = stamp;
  item.points_m.reserve(targets.size());
  for (const auto& target : targets) {
    const Vec3 d{target.x - origin.x, target.y - origin.y, target.z - origin.z};
    item.points_m.push_back({rotation[0] * d.x + rotation[3] * d.y + rotation[6] * d.z,
                             rotation[1] * d.x + rotation[4] * d.y + rotation[7] * d.z,
                             rotation[2] * d.x + rotation[5] * d.y + rotation[8] * d.z});
  }
  return item;
}
Vec3 cell_center(const UldDomain& domain, int x, int y, int z) {
  return domain.center({x, y, z});
}
// Brute-force re-check of one returned candidate straight from cell reads.
std::string brute_problem(const Snapshot& snapshot, const PlacementCandidate& c,
                          const PlacementPolicy& policy) {
  const auto domain = snapshot.domain();
  const double res = domain->resolution();
  const int fx = c.verify_end[0] - c.verify_begin[0];
  const int fy = c.verify_end[1] - c.verify_begin[1];
  const int fz = c.verify_end[2] - c.verify_begin[2];
  if (fx <= 0 || fy <= 0 || fz <= 0) return "bad_verify_range";
  for (int x = c.verify_begin[0]; x < c.verify_end[0]; ++x)
    for (int y = c.verify_begin[1]; y < c.verify_end[1]; ++y)
      for (int z = c.verify_begin[2]; z < c.verify_end[2]; ++z) {
        const auto sample = snapshot.cell({x, y, z});
        if (!sample.active) return "inactive_in_volume";
        if (!sample.known) return "unknown_in_volume";
        if (sample.value.occupied) return "occupied_in_volume";
      }
  const int slab = static_cast<int>(std::ceil(policy.support_tol_m / res - 1e-9));
  const int side = static_cast<int>(std::ceil(policy.same_layer_clearance_m / res - 1e-9));
  std::size_t supported = 0;
  for (int x = c.verify_begin[0]; x < c.verify_begin[0] + fx; ++x)
    for (int y = c.verify_begin[1]; y < c.verify_begin[1] + fy; ++y) {
      bool column = false;
      for (int z = c.cell_z - slab; z < c.cell_z; ++z) {
        if (z < 0) continue;
        const auto sample = snapshot.cell({x, y, z});
        if (sample.known && sample.value.occupied) { column = true; break; }
      }
      if (column) ++supported;
    }
  const auto columns = static_cast<double>(fx) * fy;
  const bool floor_supported = c.cell_z * res <= policy.support_tol_m + 1e-9;
  const double ratio = floor_supported ? 1.0 : supported / columns;
  if (ratio < policy.min_support_ratio - 1e-9) return "support_ratio_below_min";
  if (std::abs(ratio - c.support_ratio) > 1e-6) return "support_ratio_mismatch";
  for (int x = c.verify_begin[0] - side; x < c.verify_begin[0] + fx + side; ++x)
    for (int y = c.verify_begin[1] - side; y < c.verify_begin[1] + fy + side; ++y)
      for (int z = c.verify_begin[2]; z < c.verify_begin[2] + fz; ++z) {
        if (x < 0 || y < 0 || z < 0) continue;
        const auto sample = snapshot.cell({x, y, z});
        if (sample.known && sample.value.occupied) return "occupied_within_lateral_clearance";
      }
  const double ceiling = domain->descriptor().ceiling_z;
  const double world_top = domain->origin().z + static_cast<double>(c.verify_end[2]) * res;
  if (ceiling - world_top < policy.clearance_margin_m - 1e-9) return "ceiling_clearance";
  const double ox = domain->origin().x + static_cast<double>(c.cell_x) * res;
  const double oy = domain->origin().y + static_cast<double>(c.cell_y) * res;
  const double oz = domain->origin().z + static_cast<double>(c.cell_z) * res;
  if (std::abs(ox - c.origin_x) > 1e-9 || std::abs(oy - c.origin_y) > 1e-9
      || std::abs(oz - c.origin_z) > 1e-9) return "origin_mismatch";
  return "";
}
struct Scene {
  ObservationStore store;
  std::int64_t stamp = 1000000000;
  int observations = 0;
  explicit Scene(const ObserverConfiguration& cfg)
      : store(cfg.geometry, cfg.resolution, cfg.depth, cfg.clock_domain, cfg.digest, false) {}
  // One ray per (row, layer): from x=-1.15 straight to the far wall at the
  // exact (y, z) of the target cell. Only that row's layer is swept; the hit
  // lands at the wall, outside any interior patch.
  void sweep_box(const ObserverConfiguration& cfg, int x0, int y0, int x1, int y1,
                 int z_from, int z_to) {
    const auto domain = store.domain();
    for (int z = z_from; z <= z_to; ++z)
      for (int y = y0; y < y1; ++y) {
        const auto at = cell_center(*domain, 0, y, z);
        const auto wall = cell_center(*domain, domain->shape().x - 1, y, z);
        store.integrate(fan(cfg, "sweep_" + std::to_string(observations++),
                            stamp += 10000000, {-1.15, at.y, at.z}, kPlusX, {wall}));
      }
  }
  void drop_hits(const ObserverConfiguration& cfg, int z_cell,
                 const std::vector<std::pair<int, int>>& columns) {
    const auto domain = store.domain();
    std::vector<Vec3> targets;
    double cx = 0, cy = 0;
    for (const auto& [x, y] : columns) {
      const auto center = cell_center(*domain, x, y, z_cell);
      targets.push_back(center);
      cx += center.x; cy += center.y;
    }
    cx /= targets.size(); cy /= targets.size();
    store.integrate(fan(cfg, "hit_" + std::to_string(observations++), stamp += 10000000,
                        {cx, cy, domain->descriptor().ceiling_z - 0.05}, kDown, targets));
  }
};
std::size_t collect(const Snapshot& snapshot, const FreespaceRequest& request,
                    std::vector<PlacementCandidate>* out = nullptr) {
  std::size_t cursor = 0, total = 0;
  bool done = false;
  while (!done) {
    FreespaceRequest page_request = request;
    page_request.cursor = cursor;
    const auto page = query_freespace(snapshot, page_request);
    if (out) out->insert(out->end(), page.candidates.begin(), page.candidates.end());
    total += page.candidates.size();
    cursor = page.next_cursor;
    done = page.done;
  }
  return total;
}
void test_empty_tree(const ObserverConfiguration& cfg) {
  Scene scene(cfg);
  // A store that never integrated has no published snapshot; the query then
  // never runs at all (the chain answers NO_DATA before any enumeration).
  bool no_data = false;
  try {
    scene.store.snapshot();
  } catch (const Error& error) {
    no_data = std::string(error.what()) == "NO_DATA";
  }
  expect(no_data, "empty store has no snapshot to query");
  // Integrate one observation outside the domain grid? Not possible: use a
  // single ray that only marks the far wall, leaving the interior unknown.
  const auto domain = scene.store.domain();
  const auto wall = cell_center(*domain, domain->shape().x - 1, domain->shape().y / 2, 5);
  scene.store.integrate(fan(cfg, "wall_touch", scene.stamp += 10000000,
                            {-1.15, wall.y, wall.z}, kPlusX, {wall}));
  const auto snapshot = scene.store.snapshot();
  FreespaceRequest request{0.30, 0.20, 0.20, 2, 0, 4096};
  const auto page = query_freespace(*snapshot, request);
  expect(page.feasible_total == 0, "empty tree has no candidates");
  expect(page.grid_total > 0, "empty tree still enumerates the grid");
  expect(page.rejected_unknown > 0, "empty tree rejects as unknown");
  expect(page.grid_total == page.feasible_total + page.rejected_out_of_domain
             + page.rejected_collision + page.rejected_clearance + page.rejected_unknown
             + page.rejected_support, "counter identity on the empty tree");
}
void test_floor_prior_and_observed_support(const ObserverConfiguration& cfg) {
  Scene scene(cfg);
  const auto domain = scene.store.domain();
  const int sx = domain->shape().x, sy = domain->shape().y;
  const int x0 = sx / 4, y0 = sy / 4, x1 = x0 + 24, y1 = y0 + 24;
  scene.sweep_box(cfg, x0, y0, x1, y1, 0, 9);
  const auto snapshot = scene.store.snapshot();
  expect(snapshot->known_entry_count() > 0, "sweep observed cells");
  FreespaceRequest request{0.30, 0.20, 0.20, 0, 0, 65536};
  std::vector<PlacementCandidate> candidates;
  collect(*snapshot, request, &candidates);
  expect(!candidates.empty(), "floor-prior candidates exist");
  bool saw_floor_prior = false;
  for (const auto& c : candidates) {
    const auto problem = brute_problem(*snapshot, c, PlacementPolicy{});
    expect(problem.empty(), ("brute check: " + problem).c_str());
    if (c.support_kind == 2 && c.cell_z == 0) saw_floor_prior = true;
    if (c.support_kind == 2)
      expect(c.cell_z * domain->resolution() <= 0.05 + 1e-9, "prior rests within tol of floor");
  }
  expect(saw_floor_prior, "floor-prior kind present with swept volume");
  // A support plane at z=10 over a 15x10 sub-patch: hits from above.
  std::vector<std::pair<int, int>> supported;
  for (int x = x0; x < x0 + 15; ++x)
    for (int y = y0; y < y0 + 10; ++y) supported.emplace_back(x, y);
  scene.drop_hits(cfg, 10, supported);
  // Volume above the plane must also be observed free over the FULL footprint
  // (horizontal sweeps leave no interior occupied).
  scene.sweep_box(cfg, x0, y0, x0 + 15, y0 + 10, 11, 30);
  const auto after = scene.store.snapshot();
  std::vector<PlacementCandidate> stacked;
  collect(*after, request, &stacked);
  expect(!stacked.empty(), "candidates exist after the support plane");
  bool saw_observed = false;
  for (const auto& c : stacked) {
    const auto problem = brute_problem(*after, c, PlacementPolicy{});
    expect(problem.empty(), ("brute check after plane: " + problem).c_str());
    if (c.support_kind == 1 && c.cell_z == 11) saw_observed = true;
  }
  expect(saw_observed, "observed-support kind rests on the plane");
}
void test_support_ratio_boundary(const ObserverConfiguration& cfg) {
  const auto domain_base = Scene(cfg).store.domain();
  const int sx = domain_base->shape().x, sy = domain_base->shape().y;
  const int x0 = sx / 4, y0 = sy / 4;
  const auto center_of = [&](int x, int y, int z) { return cell_center(*domain_base, x, y, z); };
  // Exactly 60% of a 15x10 footprint supported -> accepted.
  {
    Scene scene(cfg);
    scene.sweep_box(cfg, x0, y0, x0 + 15, y0 + 10, 0, 9);
    std::vector<std::pair<int, int>> supported;
    for (int x = x0; x < x0 + 9; ++x)
      for (int y = y0; y < y0 + 10; ++y) supported.emplace_back(x, y);
    scene.drop_hits(cfg, 10, supported);
    scene.sweep_box(cfg, x0, y0, x0 + 15, y0 + 10, 11, 30);
    const auto snapshot = scene.store.snapshot();
    FreespaceRequest request{0.30, 0.20, 0.20, 0, 0, 65536};
    std::vector<PlacementCandidate> candidates;
    collect(*snapshot, request, &candidates);
    const auto at_patch = std::count_if(candidates.begin(), candidates.end(),
        [&](const PlacementCandidate& c) {
          return c.cell_x == x0 && c.cell_y == y0 && c.cell_z == 11;
        });
    expect(at_patch == 1, "exact 0.6 ratio accepted");
    (void)center_of;
  }
  // 89 of 150 columns: below the frozen 0.6 minimum.
  {
    Scene scene(cfg);
    scene.sweep_box(cfg, x0, y0, x0 + 15, y0 + 10, 0, 9);
    std::vector<std::pair<int, int>> supported;
    for (int x = x0; x < x0 + 9; ++x)
      for (int y = y0; y < y0 + 9; ++y) supported.emplace_back(x, y);   // 81
    for (int y = y0; y < y0 + 8; ++y) supported.emplace_back(x0 + 8, y);  // +8 = 89
    scene.drop_hits(cfg, 10, supported);
    scene.sweep_box(cfg, x0, y0, x0 + 15, y0 + 10, 11, 30);
    const auto snapshot = scene.store.snapshot();
    FreespaceRequest request{0.30, 0.20, 0.20, 0, 0, 65536};
    std::vector<PlacementCandidate> candidates;
    collect(*snapshot, request, &candidates);
    const auto at_patch = std::count_if(candidates.begin(), candidates.end(),
        [&](const PlacementCandidate& c) {
          return c.cell_x == x0 && c.cell_y == y0 && c.cell_z == 11;
        });
    expect(at_patch == 0, "ratio below 0.6 rejected");
  }
}
void test_paging_determinism_and_counters(const ObserverConfiguration& cfg) {
  Scene scene(cfg);
  const auto domain = scene.store.domain();
  const int sx = domain->shape().x, sy = domain->shape().y;
  scene.sweep_box(cfg, sx / 4, sy / 4, sx / 4 + 24, sy / 4 + 24, 0, 9);
  const auto snapshot = scene.store.snapshot();
  FreespaceRequest request{0.30, 0.20, 0.20, 2, 0, 65536};
  std::vector<PlacementCandidate> whole;
  collect(*snapshot, request, &whole);
  for (int rep = 0; rep < 30; ++rep) {
    std::vector<PlacementCandidate> paged;
    FreespaceRequest small = request;
    small.limit = 7;
    collect(*snapshot, small, &paged);
    expect(paged == whole, "paged enumeration equals the single page");
    const auto page = query_freespace(*snapshot, request);
    expect(page.grid_total == page.feasible_total + page.rejected_out_of_domain
               + page.rejected_collision + page.rejected_clearance + page.rejected_unknown
               + page.rejected_support, "counter identity every call");
  }
  for (std::size_t i = 1; i < whole.size(); ++i) {
    const auto& a = whole[i - 1];
    const auto& b = whole[i];
    const bool ordered = a.yaw_deg < b.yaw_deg
        || (a.yaw_deg == b.yaw_deg && (std::array<int,3>{a.cell_x, a.cell_y, a.cell_z}
              < std::array<int,3>{b.cell_x, b.cell_y, b.cell_z}));
    expect(ordered, "candidate order is deterministic");
  }
}
void test_yaw_footprints(const ObserverConfiguration& cfg) {
  Scene scene(cfg);
  const auto domain = scene.store.domain();
  const int sx = domain->shape().x, sy = domain->shape().y;
  scene.sweep_box(cfg, sx / 4, sy / 4, sx / 4 + 24, sy / 4 + 24, 0, 9);
  const auto snapshot = scene.store.snapshot();
  FreespaceRequest yaw0{0.30, 0.20, 0.20, 0, 0, 65536};
  FreespaceRequest yaw90{0.30, 0.20, 0.20, 1, 0, 65536};
  FreespaceRequest both{0.30, 0.20, 0.20, 2, 0, 65536};
  std::vector<PlacementCandidate> a, b, ab;
  collect(*snapshot, yaw0, &a);
  collect(*snapshot, yaw90, &b);
  collect(*snapshot, both, &ab);
  expect(a.size() + b.size() == ab.size(), "both-yaw mode is the union");
  for (const auto& c : a) expect(c.yaw_deg == 0, "yaw 0 mode labels");
  for (const auto& c : b) expect(c.yaw_deg == 90, "yaw 90 mode labels");
  expect(!ab.empty() && ab.front().yaw_deg == 0 && ab.back().yaw_deg == 90,
         "yaw 0 candidates precede yaw 90");
}
void test_unknown_volume_never_free(const ObserverConfiguration& cfg) {
  Scene scene(cfg);
  const auto domain = scene.store.domain();
  const int sy = domain->shape().y;
  const int y0 = sy / 3, y1 = y0 + 20;
  // Horizontal rays sweep whole rows; the observed set is exactly those rows
  // at layers 0..9. Candidates may sit anywhere along them but nowhere else.
  scene.sweep_box(cfg, 0, y0, 1, y1, 0, 9);
  const auto snapshot = scene.store.snapshot();
  FreespaceRequest request{0.30, 0.20, 0.20, 2, 0, 65536};
  std::vector<PlacementCandidate> candidates;
  collect(*snapshot, request, &candidates);
  for (const auto& c : candidates) {
    const bool rows = c.verify_end[1] > y0 && c.verify_begin[1] < y1;
    const bool layers = c.verify_end[2] <= 10;
    expect(rows && layers, "candidate volume stays inside the observed rows");
    expect(brute_problem(*snapshot, c, PlacementPolicy{}).empty(), "brute on partial map");
  }
  expect(!candidates.empty(), "partial map still answers inside the rows");
}
void barrier_once(const ObserverConfiguration& cfg, Scene& scene,
                  const std::shared_ptr<const Snapshot>& snapshot, int rep) {
  const auto domain = scene.store.domain();
  std::atomic<bool> holding{false}, release{false};
  scene.store.set_lock_probe_for_test([&] {
    holding.store(true);
    while (!release.load()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
  });
  const auto target = cell_center(*domain, domain->shape().x / 2, domain->shape().y / 2, 3);
  std::thread writer([&] {
    scene.store.integrate(fan(cfg, "barrier" + std::to_string(rep),
                              scene.stamp + rep * 10000000 + 5000000,
                              {-1.15, 0, 1.2}, kDown, {target}));
  });
  while (!holding.load()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
  const auto started = std::chrono::steady_clock::now();
  FreespaceRequest request{0.30, 0.20, 0.20, 2, 0, 65536};
  const auto page = query_freespace(*snapshot, request);
  SurfaceProfile profile;
  snapshot->surface(&profile);
  const auto roi = snapshot->read_cells({domain->shape().x / 4, domain->shape().y / 4, 0},
      {domain->shape().x / 4 + 8, domain->shape().y / 4 + 8, 8}, 0, 4096);
  (void)roi;
  const double elapsed = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
  expect(page.feasible_total > 0, "query succeeds under the write barrier");
  expect(elapsed < 200.0, "barrier read within 200 ms");
  expect(holding.load(), "write lock still held when the read finished");
  release.store(true);
  writer.join();
  scene.store.set_lock_probe_for_test(nullptr);
}
void test_write_lock_barrier(const ObserverConfiguration& cfg) {
  // Contract §3: an existing lease finishes ROI + surface + freespace within
  // 200 ms while the store write lock is held by a test barrier.
  Scene scene(cfg);
  const auto domain = scene.store.domain();
  const int sx = domain->shape().x, sy = domain->shape().y;
  scene.sweep_box(cfg, sx / 4, sy / 4, sx / 4 + 24, sy / 4 + 24, 0, 9);
  for (int rep = 0; rep < 30; ++rep) {
    SnapshotRepository repository(scene.store);
    const auto lease = repository.acquire(scene.stamp + 1000000000, 3600000000000LL, true);
    const auto snapshot = repository.get(lease.handle);
    barrier_once(cfg, scene, snapshot, rep);
    repository.release(lease.handle);
  }
}
void test_scratch_accounting(const ObserverConfiguration& cfg) {
  Scene scene(cfg);
  const auto domain = scene.store.domain();
  const int sx = domain->shape().x, sy = domain->shape().y;
  scene.sweep_box(cfg, sx / 4, sy / 4, sx / 4 + 24, sy / 4 + 24, 0, 9);
  const auto snapshot = scene.store.snapshot();
  auto& totals = AllocationTotals::instance();
  totals.reset_for_test();
  FreespaceRequest request{0.30, 0.20, 0.20, 2, 0, 65536};
  for (int rep = 0; rep < 30; ++rep) {
    query_freespace(*snapshot, request);
    expect(totals.live("query_scratch") == 0, "query scratch returns to zero");
  }
  expect(totals.peak("query_scratch") > 0, "query scratch peak recorded");
  expect(totals.read_view_peak() == 0 && totals.combined_peak() == 0,
         "scratch stays outside the read-view/cache scope");
}
void test_revision_and_epoch_identity(const ObserverConfiguration& cfg) {
  Scene scene(cfg);
  const auto domain = scene.store.domain();
  const int sx = domain->shape().x, sy = domain->shape().y;
  const int x0 = sx / 4, y0 = sy / 4;
  scene.sweep_box(cfg, x0, y0, x0 + 24, y0 + 24, 0, 9);
  SnapshotRepository repository(scene.store);
  const auto lease_old = repository.acquire(scene.stamp + 1000000000, 3600000000000LL, true);
  const auto old = repository.get(lease_old.handle);
  std::vector<std::pair<int, int>> plane;
  for (int x = x0; x < x0 + 15; ++x)
    for (int y = y0; y < y0 + 10; ++y) plane.emplace_back(x, y);
  scene.drop_hits(cfg, 10, plane);
  scene.sweep_box(cfg, x0, y0, x0 + 15, y0 + 10, 11, 30);
  const auto latest = scene.store.snapshot();
  expect(latest->identity().map_revision > old->identity().map_revision, "revision advanced");
  FreespaceRequest request{0.30, 0.20, 0.20, 0, 0, 65536};
  std::vector<PlacementCandidate> before, after;
  collect(*old, request, &before);
  collect(*latest, request, &after);
  expect(after.size() > before.size(), "new support plane adds candidates");
  bool saw_on_plane = false;
  for (const auto& c : after)
    if (c.cell_z == 11 && c.cell_x == x0 && c.cell_y == y0) saw_on_plane = true;
  expect(saw_on_plane, "resting on the new plane visible only in the new view");
  for (const auto& c : before)
    expect(c.cell_z != 11 || c.cell_x != x0 || c.cell_y != y0,
           "old view never leaks the new plane");
  const auto new_epoch = scene.store.reset(scene.store.published_epoch());
  expect(new_epoch == old->identity().epoch + 1, "epoch advanced by reset");
  // The reset store publishes nothing until a fresh observation integrates;
  // after one wall touch the new epoch answers and the OLD epoch is refused.
  const auto domain2 = scene.store.domain();
  const auto wall2 = cell_center(*domain2, domain2->shape().x - 1,
                                 domain2->shape().y / 2, 5);
  const auto receipt = scene.store.integrate(fan(cfg, "after_reset",
      scene.stamp += 10000000, {-1.15, wall2.y, wall2.z}, kPlusX, {wall2},
      scene.store.published_epoch()));
  expect(receipt.status == "INTEGRATED" && receipt.identity.epoch == new_epoch,
         "fresh observation integrates on the new epoch");
  bool epoch_mismatch = false;
  try {
    repository.acquire(scene.stamp + 2000000000, 3600000000000LL, true, old->identity().epoch);
  } catch (const Error& error) {
    epoch_mismatch = std::string(error.what()) == "EPOCH_MISMATCH";
  }
  expect(epoch_mismatch, "old epoch rejected after reset");
  const auto fresh = scene.store.snapshot();
  expect(fresh->identity().epoch == new_epoch, "fresh snapshot carries the new epoch");
  std::vector<PlacementCandidate> reset_view;
  collect(*fresh, request, &reset_view);
  for (const auto& c : reset_view)
    expect(c.cell_z != 11 || c.cell_x != x0 || c.cell_y != y0,
           "reset view never leaks the pre-reset plane");
}
void test_pressure_paths(const ObserverConfiguration& cfg) {
  Scene scene(cfg);
  const auto domain = scene.store.domain();
  const int sx = domain->shape().x, sy = domain->shape().y;
  scene.sweep_box(cfg, sx / 4, sy / 4, sx / 4 + 24, sy / 4 + 24, 0, 9);
  for (int rep = 0; rep < 30; ++rep) {
    SnapshotRepository repository(scene.store);
    std::vector<std::string> handles;
    bool pressure_at_nine = false;
    int rejected_at = -1;
    for (int i = 0; i < 9; ++i) {
      try {
        handles.push_back(repository.acquire(scene.stamp + 1000000000, 3600000000000LL, true).handle);
      } catch (const Error& error) {
        pressure_at_nine = std::string(error.what()) == "SNAPSHOT_PRESSURE";
        rejected_at = i;
      }
    }
    expect(pressure_at_nine && rejected_at == 8, "pressure exactly at the 9th handle");
    expect(handles.size() == 8, "eight handles stay open");
    FreespaceRequest request{0.30, 0.20, 0.20, 2, 0, 65536};
    std::vector<PlacementCandidate> reference;
    collect(*repository.get(handles[0]), request, &reference);
    for (std::size_t i = 1; i < handles.size(); ++i) {
      std::vector<PlacementCandidate> same;
      collect(*repository.get(handles[i]), request, &same);
      expect(same == reference, "shared-revision handles answer identically");
    }
    for (const auto& handle : handles) repository.release(handle);
    bool reopened = false;
    try {
      repository.acquire(scene.stamp + 1000000000, 3600000000000LL, true);
      reopened = true;
    } catch (const Error&) {
    }
    expect(reopened, "handle capacity recovers after release");
  }
}

void test_ttl_exception_and_concurrency(const ObserverConfiguration& cfg) {
  // C-gate classes: TTL expiry, error-path handle validity, and concurrent
  // cache builds -- each repeated; queries never take the store lock.
  Scene scene(cfg);
  const auto domain = scene.store.domain();
  const int sx = domain->shape().x, sy = domain->shape().y;
  scene.sweep_box(cfg, sx / 4, sy / 4, sx / 4 + 24, sy / 4 + 24, 0, 9);
  for (int rep = 0; rep < 30; ++rep) {
    SnapshotRepository repository(scene.store);
    repository.set_ttl_for_test(std::chrono::milliseconds(30));
    const auto lease = repository.acquire(scene.stamp + 1000000000,
                                          3600000000000LL, true);
    bool expired = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    try {
      repository.get(lease.handle);
    } catch (const Error& error) {
      expired = std::string(error.what()) == "INVALID_OR_EXPIRED_HANDLE";
    }
    expect(expired, "TTL expires the handle");
    bool invalid = false;
    try {
      repository.get("bogus-handle");
    } catch (const Error& error) {
      invalid = std::string(error.what()) == "INVALID_OR_EXPIRED_HANDLE";
    }
    expect(invalid, "unknown handle rejected");
    FreespaceRequest request{0.30, 0.20, 0.20, 2, 0, 4096};
    bool bad_page = false;
    try {
      query_freespace(*scene.store.snapshot(), request);
      request.limit = 0;
      query_freespace(*scene.store.snapshot(), request);
    } catch (const Error& error) {
      bad_page = std::string(error.what()) == "INVALID_PAGE_LIMIT";
    }
    expect(bad_page, "zero page limit rejected");
  }
  // Concurrent queries + surface first-builds answer identically and raise
  // no failure; the SurfaceCache mutex builds the column buffer once.
  const auto snapshot = scene.store.snapshot();
  for (int rep = 0; rep < 6; ++rep) {
    std::vector<std::vector<PlacementCandidate>> results(4);
    std::vector<std::thread> workers;
    std::atomic<int> failures_here{0};
    for (int worker = 0; worker < 4; ++worker) {
      workers.emplace_back([&, worker] {
        try {
          FreespaceRequest request{0.30, 0.20, 0.20, 2, 0, 65536};
          std::size_t cursor = 0;
          bool done = false;
          while (!done) {
            request.cursor = cursor;
            const auto page = query_freespace(*snapshot, request);
            results[worker].insert(results[worker].end(),
                                    page.candidates.begin(), page.candidates.end());
            cursor = page.next_cursor;
            done = page.done;
          }
          SurfaceProfile profile;
          snapshot->surface(&profile);
        } catch (const std::exception&) {
          ++failures_here;
        }
      });
    }
    for (auto& worker : workers) worker.join();
    expect(failures_here.load() == 0, "concurrent query/surface without failure");
    for (int worker = 1; worker < 4; ++worker)
      expect(results[worker] == results[0], "concurrent queries agree");
  }
}
void test_owner_class_split(const ObserverConfiguration& cfg) {
  // gen4 per-owner attribution: the same bytes move lease -> client-held ->
  // released as the last holder lets go, the split always sums to the merged
  // fields, and nothing accumulates across 30 repetitions.
  Scene scene(cfg);
  const auto domain = scene.store.domain();
  const int sx = domain->shape().x, sy = domain->shape().y;
  const int x0 = sx / 4, y0 = sy / 4;
  scene.sweep_box(cfg, x0, y0, x0 + 24, y0 + 24, 0, 9);
  for (int rep = 0; rep < 30; ++rep) {
    SnapshotRepository repository(scene.store);
    const auto before = repository.usage();
    expect(before.lease_view_block_bytes == 0 && before.client_held_view_block_bytes == 0
               && before.old_view_block_bytes == 0,
           "clean baseline has no view-exclusive blocks");
    // Lease the current view, then flip FRESH free cells to occupied so the
    // store deterministically rewrites those blocks and drops the leased
    // copies from its current set (a converged re-sweep would change nothing:
    // old-view bytes appear exactly on real divergence). The shared_ptr
    // lives in this inner scope so its destructor is the LAST holder.
    const int tile = rep % 16;
    const int hit_z = rep < 16 ? 5 : 8;
    std::vector<std::pair<int, int>> plane;
    for (int x = x0 + (tile % 4) * 5; x < x0 + (tile % 4) * 5 + 5; ++x)
      for (int y = y0 + (tile / 4) * 5; y < y0 + (tile / 4) * 5 + 5; ++y)
        plane.emplace_back(x, y);
    std::uint64_t leased_bytes = 0;
    {
      const auto lease = repository.acquire(scene.stamp + 1000000000,
                                            3600000000000LL, true);
      const auto held = repository.get(lease.handle);
      scene.drop_hits(cfg, hit_z, plane);
      const auto leased = repository.usage();
      leased_bytes = leased.lease_view_block_bytes;
      expect(leased.lease_view_block_bytes > 0, "leased old view carries block bytes");
      expect(leased.lease_view_block_bytes + leased.retained_view_block_bytes
                 + leased.client_held_view_block_bytes == leased.old_view_block_bytes,
             "block split sums to old_view_block_bytes (lease held)");
      expect(leased.lease_view_directory_entries + leased.retained_view_directory_entries
                 + leased.client_held_view_directory_entries + scene.store.directory_entries()
                 == leased.view_directory_entries,
             "directory split sums to view_directory_entries (lease held)");
      // Last holder: release the handle but keep the shared_ptr -- the bytes
      // must move to the client-held class, still counted, never dropped.
      repository.release(lease.handle);
      const auto released = repository.usage();
      expect(released.lease_view_block_bytes == 0, "no lease class after release");
      expect(released.client_held_view_block_bytes == leased_bytes,
             "bytes move to client-held while the last holder lives");
      expect(released.lease_view_block_bytes + released.retained_view_block_bytes
                 + released.client_held_view_block_bytes == released.old_view_block_bytes,
             "block split sums to old_view_block_bytes (client held)");
    }
    // The final reference dropped with the scope: the class empties and the
    // merged old-view bytes return to the baseline -- nothing lingers.
    const auto baseline_old = before.old_view_block_bytes;
    const auto settled = repository.usage();
    expect(settled.client_held_view_block_bytes == 0 && settled.lease_view_block_bytes == 0,
           "owner classes empty after the last holder drops");
    expect(settled.old_view_block_bytes == baseline_old,
           "old-view bytes return to the baseline after the last release");
    expect(settled.external_surface_buffer_bytes <= settled.surface_cache_bytes,
           "external surface share stays inside surface_cache_bytes");
  }
}
}  // namespace

int main() {
  const auto cfg = load();
  test_empty_tree(cfg);
  test_floor_prior_and_observed_support(cfg);
  test_support_ratio_boundary(cfg);
  test_paging_determinism_and_counters(cfg);
  test_yaw_footprints(cfg);
  test_unknown_volume_never_free(cfg);
  test_write_lock_barrier(cfg);
  test_scratch_accounting(cfg);
  test_revision_and_epoch_identity(cfg);
  test_pressure_paths(cfg);
  test_ttl_exception_and_concurrency(cfg);
  test_owner_class_split(cfg);
  if (failures) {
    std::cerr << failures << " freespace behavior failures\n";
    return 1;
  }
  std::cout << "freespace behavior OK\n";
  return 0;
}
