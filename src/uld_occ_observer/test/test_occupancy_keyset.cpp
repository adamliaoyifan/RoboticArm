#include "uld_occ_observer/configuration.hpp"
#include "uld_occ_observer/occupancy_grid.hpp"
#include "uld_occ_observer/observer.hpp"
#include <octomap/OcTree.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <cstdlib>
#include <unistd.h>

using namespace uld_occ_observer;

void require(bool value, const char* message) {
  if (!value) throw Error(message);
}

void configure(octomap::OcTree& tree) {
  tree.setProbHit(.7);
  tree.setProbMiss(.4);
  tree.setOccupancyThres(.5);
  tree.setClampingThresMin(.1192);
  tree.setClampingThresMax(.971);
}

struct ReferenceSets {
  std::set<CellIndex> frees;
  std::set<CellIndex> hits;
};

ReferenceSets reference_sets(const UldDomain& domain, const Vec3& origin,
                             const std::vector<Vec3>& points, double resolution) {
  ReferenceSets out;
  octomap::OcTree probe(resolution);
  const auto origin_om = domain.octomap_coordinates(origin);
  octomap::KeyRay ray;
  for (const auto& endpoint : points) {
    const auto point = domain.octomap_coordinates(endpoint);
    octomap::OcTreeKey endpoint_key;
    require(probe.coordToKeyChecked(point.x, point.y, point.z, endpoint_key), "reference endpoint key");
    require(probe.computeRayKeys(octomap::point3d(origin_om.x, origin_om.y, origin_om.z),
                                 octomap::point3d(point.x, point.y, point.z), ray),
            "reference ray");
    for (const auto& key : ray) {
      const auto index = cell_index_from_key(key);
      if (domain.active(index)) out.frees.insert(index);
    }
    if (domain.contains(endpoint)) {
      const auto index = cell_index_from_key(endpoint_key);
      if (domain.active(index)) out.hits.insert(index);
    }
  }
  for (const auto& hit : out.hits) out.frees.erase(hit);
  return out;
}

std::set<CellIndex> cells_of(const octomap::KeySet& keys) {
  std::set<CellIndex> out;
  for (const auto& key : keys) out.insert(cell_index_from_key(key));
  return out;
}

OpticalObservation observation(const std::vector<Vec3>& points, std::int64_t stamp) {
  OpticalObservation out;
  out.identity = {"test", "keyset", "optical", "test_clock", "fixture_geometry", 1, stamp};
  out.uld_from_optical.stamp_ns = stamp;
  out.points_m = points;
  return out;
}

octomap::OcTreeKey endpoint_key(const UldDomain& domain, double resolution, const Vec3& point) {
  octomap::OcTree probe(resolution);
  const auto om = domain.octomap_coordinates(point);
  octomap::OcTreeKey key;
  require(probe.coordToKeyChecked(om.x, om.y, om.z, key), "point left the key range");
  return key;
}

OccupancyScan collect_of(OccupancyGrid& grid, const Vec3& origin, const std::vector<Vec3>& points,
                         const UldDomain& domain, OccupancyScanOptions options,
                         OccupancyDiagnostic* diagnostic) {
  OccupancyDiagnostic off(ProfileMode::Off);
  OccupancyDiagnostic& session = diagnostic ? *diagnostic : off;
  OccupancyScan scan;
  grid.collect(scan, origin, points, domain, options, session);
  return scan;
}

OccupancyScan cast_one(const UldDomain& domain, double resolution, const Vec3& origin, const Vec3& point,
                       bool discretize) {
  OccupancyGrid grid(resolution);
  return collect_of(grid, origin, {point}, domain, OccupancyScanOptions{discretize}, nullptr);
}

void check_discretize(const UldDomain& domain, double resolution) {
  const Vec3 origin{-0.3, -0.3, 0.6};
  bool found = false;
  Vec3 first, second;
  const auto shape = domain.shape();
  for (int x = 0; x < shape.x && !found; ++x) {
    for (int y = 0; y < shape.y && !found; ++y) {
      for (int z = 0; z < shape.z && !found; ++z) {
        const CellIndex index{x, y, z};
        if (!domain.active(index)) continue;
        const Vec3 center = domain.center(index);
        const Vec3 a{center.x - 0.4 * resolution, center.y - 0.4 * resolution, center.z - 0.4 * resolution};
        const Vec3 b{center.x + 0.4 * resolution, center.y + 0.4 * resolution, center.z + 0.4 * resolution};
        if (!domain.contains(a) || !domain.contains(b)) continue;
        if (!(endpoint_key(domain, resolution, a) == endpoint_key(domain, resolution, b))) continue;
        if (cells_of(cast_one(domain, resolution, origin, a, false).free_cells) ==
            cells_of(cast_one(domain, resolution, origin, b, false).free_cells))
          continue;
        first = a;
        second = b;
        found = true;
      }
    }
  }
  require(found, "no same-cell pair with different free rays");

  std::vector<Vec3> repeated(30, first);
  OccupancyGrid grid(resolution);
  const auto once = cast_one(domain, resolution, origin, first, true);
  const auto many = collect_of(grid, origin, repeated, domain, OccupancyScanOptions{true}, nullptr);
  require(many.endpoints_used == 1 && many.rays_attempted == 1, "repeated point still cast extra rays");
  require(cells_of(many.free_cells) == cells_of(once.free_cells), "repeated point changed free evidence");
  require(cells_of(many.occupied_cells) == cells_of(once.occupied_cells), "repeated point changed hits");

  const auto only_first = cast_one(domain, resolution, origin, first, false);
  const auto only_second = cast_one(domain, resolution, origin, second, false);
  OccupancyGrid both_grid(resolution);
  const auto both = collect_of(both_grid, origin, {first, second}, domain, {}, nullptr);
  const auto forward = collect_of(both_grid, origin, {first, second}, domain, OccupancyScanOptions{true}, nullptr);
  const auto reverse = collect_of(both_grid, origin, {second, first}, domain, OccupancyScanOptions{true}, nullptr);
  require(both.endpoints_used == 2 && both.rays_attempted == 2, "discretize leaked into the default path");
  require(forward.endpoints_used == 1 && reverse.endpoints_used == 1, "same-cell pair kept both rays");
  require(cells_of(forward.occupied_cells) == cells_of(reverse.occupied_cells), "order changed occupied cells");
  require(cells_of(forward.free_cells) == cells_of(only_first.free_cells), "forward ray is not the first point");
  require(cells_of(reverse.free_cells) == cells_of(only_second.free_cells), "reverse ray is not the first point");
  require(cells_of(forward.free_cells) != cells_of(reverse.free_cells), "order reversal did not change the free ray");
  require(cells_of(both.free_cells) != cells_of(forward.free_cells), "skipping the second ray left the free set unchanged");
  const Vec3 other{0.2, -0.2, 0.75};
  require(!(endpoint_key(domain, resolution, first) == endpoint_key(domain, resolution, other)),
          "unique-endpoint fixture collided");
  OccupancyGrid unique_grid(resolution);
  const std::vector<Vec3> unique_points{first, other};
  const auto plain = collect_of(unique_grid, origin, unique_points, domain, {}, nullptr);
  const auto discrete = collect_of(unique_grid, origin, unique_points, domain, OccupancyScanOptions{true}, nullptr);
  require(discrete.endpoints_used == unique_points.size(), "already-unique endpoints were dropped");
  require(cells_of(plain.free_cells) == cells_of(discrete.free_cells) &&
              cells_of(plain.occupied_cells) == cells_of(discrete.occupied_cells),
          "discretize changed already-unique evidence");
}

void check_chamfer_hit() {
  const double resolution = 0.05;
  GeometryDescriptor geometry{"chamfer", "digest", 1.49, 1.97, 0.53, 2.01, Chamfer{0.55, 0.985, 0.9}};
  UldDomain domain(geometry, resolution);
  const auto shape = domain.shape();
  bool found = false;
  Vec3 inside, outside;
  octomap::OcTreeKey shared;
  for (int x = 0; x < shape.x && !found; ++x) {
    for (int y = 0; y < shape.y && !found; ++y) {
      for (int z = 0; z < shape.z && !found; ++z) {
        const CellIndex index{x, y, z};
        if (!domain.active(index)) continue;
        const Vec3 center = domain.center(index);
        Vec3 in_point, out_point;
        bool got_in = false, got_out = false;
        for (int sx = -1; sx <= 1 && !(got_in && got_out); sx += 2) {
          for (int sy = -1; sy <= 1 && !(got_in && got_out); sy += 2) {
            for (int sz = -1; sz <= 1 && !(got_in && got_out); sz += 2) {
              const Vec3 sample{center.x + sx * 0.4 * resolution, center.y + sy * 0.4 * resolution,
                                center.z + sz * 0.4 * resolution};
              if (!(endpoint_key(domain, resolution, sample) == endpoint_key(domain, resolution, center)))
                continue;
              if (domain.contains(sample)) { in_point = sample; got_in = true; }
              else { out_point = sample; got_out = true; }
            }
          }
        }
        if (!got_in || !got_out) continue;
        inside = in_point;
        outside = out_point;
        shared = endpoint_key(domain, resolution, center);
        found = true;
      }
    }
  }
  require(found, "no chamfer cell contains both an inside and outside point");
  const Vec3 origin{-0.2, 0.0, 1.2};
  OccupancyGrid grid(resolution);
  const std::vector<std::vector<Vec3>> orders{{inside, outside}, {outside, inside}};
  for (const auto& order : orders) {
    const auto scan = collect_of(grid, origin, order, domain, OccupancyScanOptions{true}, nullptr);
    require(scan.endpoints_used == 1, "chamfer pair cast two free rays");
    require(cells_of(scan.occupied_cells).count(cell_index_from_key(shared)) == 1,
            "inside chamfer point lost its hit");
  }
}

std::filesystem::path make_temp_dir() {
  char pattern[] = "/tmp/occ-keyset-XXXXXX";
  if (!mkdtemp(pattern)) throw Error("TEMP_DIR_FAILED");
  return pattern;
}

void write_geometry_export(const std::filesystem::path& path, const std::string& hash) {
  const std::string canonical =
      std::string("{\"ceiling_z\":1.2,\"floor_z\":0.0,\"frame_id\":\"container_link\",") +
      "\"geometry_hash\":\"" + hash + "\",\"length\":0.8,\"schema_version\":1,\"width\":0.8}";
  std::ofstream out(path);
  out << "schema: uld_geometry_export_v1\n"
      << "descriptor_sha256: " << sha256(canonical) << "\n"
      << "descriptor_json: '" << canonical << "'\n";
}

void check_config_identity() {
  const auto scratch = make_temp_dir();
  const auto geometry = scratch / "geometry.yaml";
  write_geometry_export(geometry, "fixture_geometry");
  const std::filesystem::path original =
      std::filesystem::path(OBSERVER_SOURCE_DIR) / "config" / "observer.yaml";
  std::ifstream input(original);
  std::stringstream buffer;
  buffer << input.rdbuf();
  const std::string bytes = buffer.str();
  const auto loaded = load_configuration(geometry, original);
  require(!loaded.discretize_endpoints, "default config enabled discretize");
  const auto again = load_configuration(geometry, original);
  require(loaded.digest == again.digest, "default digest is not stable");
  const auto enabled_path = scratch / "enabled.yaml";
  {
    std::ofstream out(enabled_path);
    out << bytes << "discretize_endpoints: true\n";
  }
  const auto enabled = load_configuration(geometry, enabled_path);
  require(enabled.discretize_endpoints, "enabled config did not reach the loader");
  require(enabled.digest != loaded.digest, "enabled config kept the default digest");
  ObservationStore from_file(enabled.geometry, enabled.resolution, enabled.depth, enabled.clock_domain,
                             enabled.digest, enabled.discretize_endpoints);
  const Vec3 repeated{0.1, 0.0, 0.8};
  auto obs = observation(std::vector<Vec3>(30, repeated), 1000);
  obs.identity.geometry_hash = enabled.geometry.geometry_hash;
  obs.identity.clock_domain = enabled.clock_domain;
  const auto receipt = from_file.integrate(obs);
  require(receipt.status == "INTEGRATED", "configured store rejected the cloud");
  OccupancyGrid probe(enabled.resolution);
  UldDomain domain(enabled.geometry, enabled.resolution);
  const auto scan = collect_of(probe, {0, 0, 0}, std::vector<Vec3>(30, repeated), domain,
                                  OccupancyScanOptions{true}, nullptr);
  require(receipt.free_keys == scan.free_cells.size() && receipt.hit_keys == scan.occupied_cells.size(),
          "store flag did not match discretize=true");
  const auto unknown_path = scratch / "unknown.yaml";
  {
    std::ofstream out(unknown_path);
    out << bytes << "not_a_field: 1\n";
  }
  bool rejected = false;
  try { load_configuration(geometry, unknown_path); }
  catch (const Error& error) { rejected = std::string(error.what()) == "UNKNOWN_CONFIG_FIELD"; }
  require(rejected, "unknown config field was accepted");
  std::ifstream after(original);
  std::stringstream after_buffer;
  after_buffer << after.rdbuf();
  require(after_buffer.str() == bytes, "default observer.yaml bytes changed");
  std::filesystem::remove_all(scratch);
}

void check_profile_diagnostics(const UldDomain& domain, double resolution, const Vec3& origin,
                               const std::vector<Vec3>& points) {
  OccupancyGrid off_grid(resolution);
  OccupancyDiagnostic off(ProfileMode::Off);
  const auto off_scan = collect_of(off_grid, origin, points, domain, {}, &off);
  require(off.report().rays_attempted == 0 && off.report().ray_keys_generated == 0 &&
              off.report().active_ray_key_visits == 0 && off.report().domain_check_ms == 0 &&
              off.report().free_insert_ms == 0 && off.report().release_free_ms == 0,
          "Off profile collected diagnostics");
  OccupancyGrid stages_grid(resolution);
  OccupancyDiagnostic stages(ProfileMode::Stages);
  const auto stages_scan = collect_of(stages_grid, origin, points, domain, {}, &stages);
  OccupancyGrid detailed_grid(resolution);
  OccupancyDiagnostic detailed(ProfileMode::Detailed);
  const auto detailed_scan = collect_of(detailed_grid, origin, points, domain, {}, &detailed);
  require(cells_of(off_scan.free_cells) == cells_of(stages_scan.free_cells) &&
              cells_of(stages_scan.free_cells) == cells_of(detailed_scan.free_cells),
          "profile mode changed the free set");
  require(cells_of(off_scan.occupied_cells) == cells_of(detailed_scan.occupied_cells),
          "profile mode changed the hit set");
  require(stages.report().rays_attempted == points.size() && stages.report().ray_keys_generated > 0,
          "Stages did not collect diagnostics");
  require(stages.report().domain_check_ms == 0 && stages.report().free_insert_ms == 0 &&
              stages.report().hit_erase_ms == 0,
          "Stages read split clocks");
  require(detailed.report().domain_check_ms > 0 && detailed.report().free_insert_ms > 0,
          "Detailed did not time domain checks and inserts");
  GeometryDescriptor geometry{"fixture_geometry", "fixture_digest", 0.8, 0.8, 0, 1.2, {}};
  ObservationStore a(geometry, resolution, {}, "test_clock", "config");
  ObservationStore b(geometry, resolution, {}, "test_clock", "config");
  ObservationStore c(geometry, resolution, {}, "test_clock", "config");
  const auto oa = a.integrate(observation(points, 1000), ProfileMode::Off);
  const auto ob = b.integrate(observation(points, 1000), ProfileMode::Stages);
  const auto oc = c.integrate(observation(points, 1000), ProfileMode::Detailed);
  require(oa.status == "INTEGRATED" && a.checkpoint().tree_bytes == b.checkpoint().tree_bytes &&
              b.checkpoint().tree_bytes == c.checkpoint().tree_bytes,
          "profile mode changed the probability tree");
  require(oa.profile.rays_attempted == 0 && oa.profile.ray_keys_generated == 0, "Off receipt collected diagnostics");
  require(ob.free_keys == oc.free_keys && ob.hit_keys == oc.hit_keys && ob.changed_keys == oc.changed_keys,
          "profile mode changed published counts");
}

void check_scan_reuse(const UldDomain& domain, double resolution) {
  const Vec3 origin{0, 0, 0};
  const std::vector<Vec3> first{{0.1, 0, 0.8}, {-0.2, 0.1, 0.9}};
  const std::vector<Vec3> second{{0.2, -0.1, 0.85}, {0.0, 0.1, 0.75}, {0.15, 0, 1.0}};
  OccupancyGrid grid(resolution);
  OccupancyScan scan;
  OccupancyDiagnostic detailed(ProfileMode::Detailed);
  grid.collect(scan, origin, first, domain, {}, detailed);
  require(!scan.free_cells.empty() && detailed.report().active_ray_key_visits > 0 &&
              detailed.report().ray_keys_generated > 0,
          "enabled collect left no diagnostics");
  const auto enabled_visits = detailed.report().active_ray_key_visits;
  const auto enabled_duplicates = detailed.report().free_insert_duplicates;
  require(enabled_visits > 0, "enabled profile counted no visits");

  OccupancyDiagnostic off(ProfileMode::Off);
  grid.collect(scan, origin, second, domain, {}, off);
  OccupancyGrid fresh(resolution);
  OccupancyScan expected;
  fresh.collect(expected, origin, second, domain, {}, off);
  OccupancyScan first_only;
  fresh.collect(first_only, origin, first, domain, {}, off);
  require(cells_of(first_only.free_cells) != cells_of(expected.free_cells),
          "reuse inputs did not differ");
  require(cells_of(scan.free_cells) == cells_of(expected.free_cells),
          "reused scan kept the previous free set");
  require(cells_of(scan.occupied_cells) == cells_of(expected.occupied_cells),
          "reused scan kept the previous hit set");
  require(scan.rays_attempted == second.size() && scan.endpoints_used == second.size(),
          "reused scan kept the previous ray count");
  require(off.report().ray_keys_generated == 0 && off.report().active_ray_key_visits == 0 &&
              off.report().inactive_ray_key_visits == 0 && off.report().free_insert_duplicates == 0 &&
              off.report().domain_check_ms == 0 && off.report().free_insert_ms == 0,
          "Off reuse collected diagnostics");
  require(detailed.report().active_ray_key_visits == enabled_visits &&
              detailed.report().free_insert_duplicates == enabled_duplicates &&
              detailed.report().ray_keys_generated > 0,
          "Off collect changed counters left by the enabled pass");
}

void check_failed_endpoint_skips_apply(const UldDomain& domain, double resolution) {
  GeometryDescriptor geometry{"fixture_geometry", "fixture_digest", 0.8, 0.8, 0, 1.2, {}};
  const Vec3 origin{0, 0, 0};
  const std::vector<Vec3> valid{{0.1, 0, 0.8}, {0.2, 0.0, 0.9}};
  std::vector<Vec3> mixed = valid;
  mixed.push_back({100000.0, 0.0, 1.0});
  ObservationStore store(geometry, resolution, {}, "test_clock", "config");
  require(store.integrate(observation(valid, 1000)).status == "INTEGRATED", "seed frame rejected");
  const auto before = store.checkpoint();
  const auto known = store.diagnostics().known_cells;
  const auto identity = store.diagnostics().identity;
  auto bad = observation(mixed, 2000);
  bad.identity.observation_id = "bad-endpoint";
  const auto receipt = store.integrate(bad);
  require(receipt.status == "ENDPOINT_OUT_OF_KEY_RANGE", "illegal endpoint was integrated");
  require(receipt.free_keys == 0 && receipt.hit_keys == 0 && receipt.changed_keys == 0,
          "failed integrate published a partial scan");
  require(store.checkpoint().tree_bytes == before.tree_bytes, "failed integrate changed the tree");
  require(store.diagnostics().known_cells == known, "failed integrate changed known cells");
  const auto after = store.diagnostics().identity;
  require(after.epoch == identity.epoch && after.map_revision == identity.map_revision &&
              after.observed_revision == identity.observed_revision &&
              after.last_integrated_stamp_ns == identity.last_integrated_stamp_ns,
          "failed integrate advanced store identity");

  OccupancyGrid grid(resolution);
  OccupancyScan scan;
  OccupancyDiagnostic off(ProfileMode::Off);
  bool rejected = false;
  try { grid.collect(scan, origin, mixed, domain, {}, off); }
  catch (const Error& error) { rejected = std::string(error.what()) == "ENDPOINT_OUT_OF_KEY_RANGE"; }
  require(rejected, "collect accepted an endpoint outside the key range");
  require(scan.rays_attempted == valid.size() && !scan.free_cells.empty(),
          "failure did not keep the valid prefix");
  const auto nodes_before = grid.nodeCount();
  grid.collect(scan, origin, valid, domain, {}, off);
  OccupancyScan fresh;
  OccupancyGrid other(resolution);
  other.collect(fresh, origin, valid, domain, {}, off);
  require(cells_of(scan.free_cells) == cells_of(fresh.free_cells) &&
              cells_of(scan.occupied_cells) == cells_of(fresh.occupied_cells),
          "reuse after failure kept a partial result");
  require(grid.nodeCount() == nodes_before, "collect wrote the tree");
}

void apply_reference(octomap::OcTree& tree, const ReferenceSets& keys) {
  for (const auto& index : keys.frees) tree.updateNode(key_from_cell_index(index), false, false);
  for (const auto& index : keys.hits) tree.updateNode(key_from_cell_index(index), true, false);
}

void check_sequence_and_restore(const UldDomain& domain, double resolution) {
  GeometryDescriptor geometry{"fixture_geometry", "fixture_digest", 0.8, 0.8, 0, 1.2, {}};
  const Vec3 origin{0, 0, 0};
  const std::vector<Vec3> script{{0.1, 0, 0.8},   {0.12, 0.02, 0.9}, {-0.1, 0.05, 0.7},
                                 {0.2, -0.1, 0.85}, {0.0, 0.1, 0.75}, {0.15, 0, 1.0},
                                 {-0.2, -0.05, 0.8}, {0.05, 0.05, 0.65}, {0.1, 0, 0.8},
                                 {0.22, 0.0, 1.05}};
  octomap::OcTree reference(resolution);
  configure(reference);
  ObservationStore store(geometry, resolution, {}, "test_clock", "config");
  std::set<CellIndex> ever;
  for (std::size_t i = 0; i < script.size(); ++i) {
    const std::vector<Vec3> frame{script[i]};
    const auto expected = reference_sets(domain, origin, frame, resolution);
    float before_hit = 0;
    bool had_hit = false;
    if (!expected.hits.empty()) {
      const auto* node = reference.search(key_from_cell_index(*expected.hits.begin()));
      had_hit = node != nullptr;
      if (node) before_hit = node->getLogOdds();
    }
    apply_reference(reference, expected);
    auto obs = observation(frame, static_cast<std::int64_t>(1000 * (i + 1)));
    obs.identity.observation_id = "seq-" + std::to_string(i);
    const auto receipt = store.integrate(obs);
    require(receipt.status == "INTEGRATED", "sequence frame rejected");
    OccupancyGrid check(resolution);
    const auto scan = collect_of(check, origin, frame, domain, {}, nullptr);
    require(cells_of(scan.free_cells) == expected.frees && cells_of(scan.occupied_cells) == expected.hits,
            "sequence free/hit set mismatch");
    const auto snapshot = store.snapshot();
    auto compare = [&](const std::set<CellIndex>& cells) {
      for (const auto& index : cells) {
        ever.insert(index);
        const auto* node = reference.search(key_from_cell_index(index));
        const auto sample = snapshot->cell(index);
        require(node && sample.known && sample.value.log_odds == node->getLogOdds(), "sequence log odds mismatch");
        require(sample.value.occupied == reference.isNodeOccupied(node), "sequence occupancy mismatch");
      }
    };
    compare(expected.frees);
    compare(expected.hits);
    require(store.diagnostics().known_cells == ever.size(), "known count diverged");
    if (i == 8) require(had_hit && before_hit != 0, "repeat frame did not revisit a cell");
  }
  auto saturated = observation({script.back()}, 20000);
  saturated.identity.observation_id = "sat-0";
  for (int i = 0; i < 8 && store.integrate(saturated).changed_keys != 0; ++i) {
    saturated.identity.observation_id = "sat-" + std::to_string(i + 1);
    saturated.identity.stamp_ns = 21000 + i;
  }
  saturated.identity.observation_id = "sat-final";
  saturated.identity.stamp_ns = 30000;
  require(store.integrate(saturated).changed_keys == 0, "repeated frame did not saturate");

  ObservationStore continued(geometry, resolution, {}, "test_clock", "config");
  for (int i = 0; i < 4; ++i) {
    auto obs = observation({script[static_cast<std::size_t>(i)]}, 1000 * (i + 1));
    obs.identity.observation_id = "pre-" + std::to_string(i);
    require(continued.integrate(obs).status == "INTEGRATED", "prefix rejected");
  }
  const auto saved = continued.checkpoint();
  const auto saved_epoch = continued.diagnostics().identity.epoch;
  ObservationStore restored(geometry, resolution, {}, "test_clock", "config");
  const auto new_epoch = restored.restore(saved, restored.diagnostics().identity.epoch);
  require(new_epoch != saved_epoch, "restore left the epoch unchanged");
  require(restored.checkpoint().tree_bytes == saved.tree_bytes, "restored tree bytes differ");
  for (int i = 4; i < 8; ++i) {
    auto left = observation({script[static_cast<std::size_t>(i)]}, 1000 * (i + 1));
    left.identity.observation_id = "post-" + std::to_string(i);
    auto right = left;
    right.identity.epoch = new_epoch;
    require(continued.integrate(left).status == "INTEGRATED", "uninterrupted suffix rejected");
    require(restored.integrate(right).status == "INTEGRATED", "restored suffix rejected");
  }
  require(continued.checkpoint().tree_bytes == restored.checkpoint().tree_bytes,
          "continued updates diverged after restore");

  const auto intact = continued.checkpoint();
  const auto intact_epoch = continued.diagnostics().identity.epoch;
  const auto intact_known = continued.diagnostics().known_cells;
  const auto intact_cell = continued.snapshot()->cell({0, 0, 0});
  Checkpoint bad = intact;
  bad.tree_bytes = "not-a-tree";
  bool rejected = false;
  try { continued.restore(bad, intact_epoch); }
  catch (const Error&) { rejected = true; }
  require(rejected, "bad file was restored");
  require(continued.diagnostics().identity.epoch == intact_epoch, "bad restore changed epoch");
  require(continued.diagnostics().known_cells == intact_known, "bad restore changed known");
  require(continued.checkpoint().tree_bytes == intact.tree_bytes, "bad restore changed the tree");
  require(continued.snapshot()->cell({0, 0, 0}).value.log_odds == intact_cell.value.log_odds &&
              continued.snapshot()->cell({0, 0, 0}).known == intact_cell.known,
          "bad restore changed the snapshot");
}

int main() {
  try {
    const double resolution = 0.05;
    GeometryDescriptor geometry{"fixture_geometry", "fixture_digest", 0.8, 0.8, 0, 1.2, {}};
    UldDomain domain(geometry, resolution);
    const Vec3 origin{0, 0, 0};
    const std::vector<Vec3> points{{0.1, 0, 0.8}, {0.1, 0, 0.8}, {0.1, 0, 0.8},
                                   {-0.2, 0.1, 0.9}, {0.7, 0, 1.0}};
    const auto expected = reference_sets(domain, origin, points, resolution);
    OccupancyGrid grid(resolution);
    const auto scan = collect_of(grid, origin, points, domain, {}, nullptr);
    require(cells_of(scan.free_cells) == expected.frees, "free set mismatch");
    require(cells_of(scan.occupied_cells) == expected.hits, "hit set mismatch");
    require(scan.rays_attempted == points.size(), "ray count changed");
    require(scan.endpoints_used == points.size(), "discretize changed the ray set");

    octomap::OcTree reference(resolution);
    configure(reference);
    for (const auto& index : expected.frees)
      reference.updateNode(key_from_cell_index(index), false, false);
    for (const auto& index : expected.hits)
      reference.updateNode(key_from_cell_index(index), true, false);

    ObservationStore store(geometry, resolution, {}, "test_clock", "config");
    const auto receipt = store.integrate(observation(points, 1000));
    require(receipt.status == "INTEGRATED", "integrate failed");
    require(receipt.free_keys == expected.frees.size(), "receipt free count");
    require(receipt.hit_keys == expected.hits.size(), "receipt hit count");
    require(receipt.changed_keys == expected.frees.size() + expected.hits.size(),
            "mutation count differs from touched cells");
    const auto snapshot = store.snapshot();
    auto check = [&](const std::set<CellIndex>& cells) {
      for (const auto& index : cells) {
        const auto sample = snapshot->cell(index);
        const auto* node = reference.search(key_from_cell_index(index));
        require(node != nullptr, "missing reference node");
        require(sample.known, "snapshot lost a touched cell");
        require(sample.value.log_odds == node->getLogOdds(), "log odds mismatch");
        require(sample.value.occupied == reference.isNodeOccupied(node), "occupancy mismatch");
      }
    };
    check(expected.frees);
    check(expected.hits);
    std::cout << "mutation_bytes=" << receipt.changed_keys * sizeof(OccupancyMutation)
              << " mutations=" << receipt.changed_keys << "\n";

    const auto before = store.checkpoint();
    const auto epoch = store.diagnostics().identity.epoch;
    const auto known = store.diagnostics().known_cells;
    Checkpoint bad = before;
    bad.tree_bytes = "not-a-tree";
    bool rejected = false;
    try { store.restore(bad, epoch); }
    catch (const Error& error) { rejected = std::string(error.what()) == "INVALID_CHECKPOINT_TREE"; }
    require(rejected, "corrupt tree was accepted");
    require(store.diagnostics().identity.epoch == epoch, "failed restore changed epoch");
    require(store.diagnostics().known_cells == known, "failed restore changed known cells");
    require(store.checkpoint().tree_bytes == before.tree_bytes, "failed restore replaced the tree");

    octomap::OcTree outside(resolution);
    configure(outside);
    outside.updateNode(octomap::point3d(10, 10, 10), true, false);
    std::ostringstream bytes(std::ios::binary);
    require(outside.write(bytes), "outside tree write");
    Checkpoint foreign = before;
    foreign.tree_bytes = bytes.str();
    rejected = false;
    try { store.restore(foreign, epoch); }
    catch (const Error& error) { rejected = std::string(error.what()) == "INVALID_CHECKPOINT_CELL"; }
    require(rejected, "out-of-domain leaf was accepted");
    require(store.checkpoint().tree_bytes == before.tree_bytes, "rejected leaf replaced the tree");

    std::vector<Vec3> cloud;
    cloud.reserve(4000);
    for (int i = 0; i < 4000; ++i) {
      const double u = static_cast<double>(i) / 4000.0;
      cloud.push_back({-0.3 + 0.5 * u, 0.05 * ((i % 7) - 3), 0.55 + 0.4 * u});
    }
    const auto t0 = std::chrono::steady_clock::now();
    const auto ref_cloud = reference_sets(domain, origin, cloud, resolution);
    const auto t1 = std::chrono::steady_clock::now();
    OccupancyGrid timed(resolution);
    const auto key_cloud = collect_of(timed, origin, cloud, domain, {}, nullptr);
    const auto t2 = std::chrono::steady_clock::now();
    require(cells_of(key_cloud.free_cells) == ref_cloud.frees, "cloud free set mismatch");
    require(cells_of(key_cloud.occupied_cells) == ref_cloud.hits, "cloud hit set mismatch");
    const auto set_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    const auto key_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
    std::cout << "dedup_compare points=" << cloud.size()
              << " free=" << ref_cloud.frees.size() << " hit=" << ref_cloud.hits.size()
              << " set_ms=" << set_ms << " keyset_ms=" << key_ms << "\n";
    check_discretize(domain, resolution);
    check_chamfer_hit();
    check_profile_diagnostics(domain, resolution, origin, points);
    check_scan_reuse(domain, resolution);
    check_failed_endpoint_skips_apply(domain, resolution);
    check_sequence_and_restore(domain, resolution);
    check_config_identity();
    std::cout << "discretize_checks=ok\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
