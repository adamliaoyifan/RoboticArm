#include "independent_oracle.hpp"
#include "uld_occ_observer/configuration.hpp"
#include "uld_occ_observer/snapshot_repository.hpp"
#include "uld_occ_observer/checkpoint_io.hpp"
#include <octomap/OcTree.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <numeric>
#include <random>
#include <set>
#include <sstream>
#include <thread>
#include <unistd.h>

using namespace uld_occ_observer;
using observer_oracle::Cell;
using observer_oracle::OccupancyOracle;

namespace {
using Clock = std::chrono::steady_clock;

observer_oracle::Geometry oracle_geometry(const ObserverConfiguration& cfg) {
  observer_oracle::Geometry g;
  g.geometry_hash = cfg.geometry.geometry_hash;
  g.length = cfg.geometry.length;
  g.width = cfg.geometry.width;
  g.floor_z = cfg.geometry.floor_z;
  g.ceiling_z = cfg.geometry.ceiling_z;
  g.resolution = cfg.resolution;
  if (cfg.geometry.chamfer) {
    g.has_chamfer = true;
    g.chamfer = {cfg.geometry.chamfer->floor_y, cfg.geometry.chamfer->wall_y,
                 cfg.geometry.chamfer->wall_z};
  }
  return g;
}

observer_oracle::DepthZ oracle_depth(const DepthZConfig& d) { return {d.min_m, d.max_m}; }

RigidTransform look_into_container(double x, double y, double z, std::int64_t stamp) {
  RigidTransform t;
  // optical X -> ULD -Y, optical Y -> ULD -Z, optical Z -> ULD +X
  t.rotation = {0, 0, 1, -1, 0, 0, 0, -1, 0};
  t.translation = {x, y, z};
  t.stamp_ns = stamp;
  return t;
}

OpticalObservation base_observation(const ObserverConfiguration& cfg, const std::string& id,
                                    std::int64_t stamp, const RigidTransform& tf) {
  OpticalObservation o;
  o.identity.sensor_id = "cam0";
  o.identity.observation_id = id;
  o.identity.optical_frame = "d555_color_optical_frame";
  o.identity.clock_domain = cfg.clock_domain;
  o.identity.geometry_hash = cfg.geometry.geometry_hash;
  o.identity.epoch = 1;
  o.identity.stamp_ns = stamp;
  o.uld_from_optical = tf;
  return o;
}

struct Box {
  observer_oracle::Vec3 center;
  observer_oracle::Vec3 half;
};

std::vector<Box> make_boxes(int count, const observer_oracle::Geometry& g) {
  std::vector<Box> boxes;
  const observer_oracle::Vec3 half{0.10, 0.10, 0.10};
  int n = 0;
  for (int layer = 0; layer < 2 && n < count; ++layer) {
    const int rows = (layer == 0) ? 3 : 1;
    const int cols = 5;
    for (int r = 0; r < rows && n < count; ++r) {
      for (int c = 0; c < cols && n < count; ++c) {
        const double x = -0.45 + c * 0.22;
        const double y = -0.55 + r * 0.28;
        const double z = g.floor_z + half.z + layer * (2 * half.z + 0.01);
        boxes.push_back({{x, y, z}, half});
        ++n;
      }
    }
  }
  return boxes;
}

bool inside_box(const observer_oracle::Vec3& p, const Box& b) {
  return std::fabs(p.x - b.center.x) <= b.half.x && std::fabs(p.y - b.center.y) <= b.half.y &&
         std::fabs(p.z - b.center.z) <= b.half.z;
}

observer_oracle::Vec3 sample_surface(const Box& b, std::mt19937_64& rng) {
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  std::uniform_int_distribution<int> face(0, 5);
  observer_oracle::Vec3 p = b.center;
  switch (face(rng)) {
    case 0: p.x -= b.half.x; p.y += b.half.y * u(rng); p.z += b.half.z * u(rng); break;
    case 1: p.x += b.half.x; p.y += b.half.y * u(rng); p.z += b.half.z * u(rng); break;
    case 2: p.y -= b.half.y; p.x += b.half.x * u(rng); p.z += b.half.z * u(rng); break;
    case 3: p.y += b.half.y; p.x += b.half.x * u(rng); p.z += b.half.z * u(rng); break;
    case 4: p.z -= b.half.z; p.x += b.half.x * u(rng); p.y += b.half.y * u(rng); break;
    default: p.z += b.half.z; p.x += b.half.x * u(rng); p.y += b.half.y * u(rng); break;
  }
  return p;
}

std::vector<Vec3> to_sut(const std::vector<observer_oracle::Vec3>& pts) {
  std::vector<Vec3> out;
  out.reserve(pts.size());
  for (const auto& p : pts) out.push_back({p.x, p.y, p.z});
  return out;
}

std::vector<observer_oracle::Vec3> generate_optical_cloud(const observer_oracle::Geometry& g,
                                                         const std::vector<Box>& boxes,
                                                         const RigidTransform& tf,
                                                         std::size_t count, double outside_frac,
                                                         double cross_frac, std::uint32_t seed,
                                                         bool include_invalid) {
  std::mt19937_64 rng(seed);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  std::uniform_real_distribution<double> z_valid(0.55, 2.4);
  std::vector<observer_oracle::Vec3> optical;
  optical.reserve(count);
  observer_oracle::Rigid rigid;
  rigid.rotation = tf.rotation;
  rigid.translation = {tf.translation.x, tf.translation.y, tf.translation.z};
  // Inverse: optical = R^T (uld - t)
  auto to_optical = [&](observer_oracle::Vec3 uld) {
    const double dx = uld.x - rigid.translation.x;
    const double dy = uld.y - rigid.translation.y;
    const double dz = uld.z - rigid.translation.z;
    const auto& r = rigid.rotation;
    return observer_oracle::Vec3{r[0] * dx + r[3] * dy + r[6] * dz,
                                 r[1] * dx + r[4] * dy + r[7] * dz,
                                 r[2] * dx + r[5] * dy + r[8] * dz};
  };
  const std::size_t n_out = static_cast<std::size_t>(count * outside_frac);
  const std::size_t n_cross = static_cast<std::size_t>(count * cross_frac);
  for (std::size_t i = 0; i < count; ++i) {
    observer_oracle::Vec3 uld;
    if (include_invalid && i == 0) {
      optical.push_back({0, 0, std::numeric_limits<double>::quiet_NaN()});
      continue;
    }
    if (i < n_out) {
      uld = {g.length, g.width * (unit(rng) - 0.5), g.floor_z + 0.4 + unit(rng)};
    } else if (i < n_out + n_cross) {
      uld = {g.length / 2 + 0.4, 0.0, g.floor_z + 0.6};
    } else if (!boxes.empty() && unit(rng) < 0.6) {
      uld = sample_surface(boxes[i % boxes.size()], rng);
    } else {
      uld = {(unit(rng) - 0.5) * g.length * 0.8, (unit(rng) - 0.5) * g.width * 0.6,
             g.floor_z + 0.05 + unit(rng) * 0.9};
    }
    auto p = to_optical(uld);
    if (p.z < 0.05) p.z = 0.55 + 0.2 * unit(rng);
    optical.push_back(p);
  }
  return optical;
}

struct CompareResult {
  bool pass{true};
  std::string reason;
  std::size_t frames{};
  std::size_t mismatches{};
};

CompareResult compare_touched(const OccupancyOracle& oracle, const Snapshot& snap,
                              const Diagnostics& diag, const observer_oracle::FrameKeys& keys) {
  CompareResult r;
  if (diag.known_cells != oracle.known_count()) {
    r.pass = false;
    r.reason = "known_count " + std::to_string(diag.known_cells) + "!=" +
               std::to_string(oracle.known_count());
    ++r.mismatches;
    return r;
  }
  auto check = [&](const observer_oracle::Cell& cell) {
    const auto it = oracle.known().find(cell);
    if (it == oracle.known().end()) return;
    const CellIndex idx{cell.x, cell.y, cell.z};
    const auto sample = snap.cell(idx);
    if (!sample.known || !sample.active) {
      r.pass = false;
      r.reason = "missing oracle cell";
      ++r.mismatches;
      return;
    }
    if (std::fabs(sample.value.log_odds - it->second.log_odds) > 1e-6f) {
      r.pass = false;
      r.reason = "logodds";
      ++r.mismatches;
      return;
    }
    if (sample.value.occupied != it->second.occupied) {
      r.pass = false;
      r.reason = "occupancy_class";
      ++r.mismatches;
    }
  };
  for (const auto& cell : keys.hits) {
    check(cell);
    if (!r.pass) return r;
  }
  for (const auto& cell : keys.frees) {
    check(cell);
    if (!r.pass) return r;
  }
  return r;
}

std::string json_escape(const std::string& s) {
  std::string o;
  o.reserve(s.size());
  for (char c : s) {
    if (c == '"' || c == '\\') o.push_back('\\');
    o.push_back(c);
  }
  return o;
}

int write_report(const std::string& path, bool pass, const std::string& gate,
                 const std::map<std::string, std::string>& fields) {
  std::ofstream out(path);
  out << "{\"gate\":\"" << json_escape(gate) << "\",\"pass\":" << (pass ? "true" : "false");
  for (const auto& kv : fields)
    out << ",\"" << json_escape(kv.first) << "\":\"" << json_escape(kv.second) << "\"";
  out << "}\n";
  return pass ? 0 : 3;
}

int run_o1(const ObserverConfiguration& cfg, int seeds, int frames, int boxes, std::size_t cloud,
           double outside, double cross, const std::string& out) {
  const auto geom = oracle_geometry(cfg);
  const auto depth = oracle_depth(cfg.depth);
  const auto box_list = make_boxes(boxes, geom);
  OccupancyOracle oracle(geom);
  ObservationStore store(cfg.geometry, cfg.resolution, cfg.depth, cfg.clock_domain, cfg.digest, cfg.discretize_endpoints);
  std::size_t mismatches = 0;
  std::string reason = "ok";
  std::size_t done = 0;
  for (int seed = 0; seed < seeds; ++seed) {
    for (int f = 0; f < frames; ++f) {
      const std::int64_t stamp = 1'000'000'000LL * (seed * frames + f + 1);
      auto tf = look_into_container(-1.15, 0.0, 1.20, stamp);
      auto obs = base_observation(cfg, "o1-" + std::to_string(seed) + "-" + std::to_string(f), stamp, tf);
      auto cloud_pts = generate_optical_cloud(geom, box_list, tf, cloud, outside, cross,
                                              static_cast<uint32_t>(seed * 10007 + f), f == 0);
      obs.points_m = to_sut(cloud_pts);
      observer_oracle::Rigid rigid{tf.rotation, {tf.translation.x, tf.translation.y, tf.translation.z}};
      const auto prepared = observer_oracle::prepare(cloud_pts, rigid, depth);
      const auto keys = observer_oracle::evidence_keys(geom, prepared.origin, prepared.uld_points);
      oracle.apply_frame(keys);
      const auto receipt = store.integrate(obs);
      ++done;
      if (receipt.status != "INTEGRATED" && receipt.status != "NO_VALID_POINTS") {
        mismatches++;
        reason = receipt.status;
        break;
      }
      if (receipt.status == "NO_VALID_POINTS") {
        if (!prepared.filter.kept.empty()) {
          mismatches++;
          reason = "false_empty";
          break;
        }
        continue;
      }
      if (receipt.hit_keys != keys.hits.size() || receipt.free_keys != keys.frees.size()) {
        mismatches++;
        reason = "key_counts";
        octomap::OcTree probe(geom.resolution);
        std::size_t native_free = 0, indep_free = keys.frees.size(), ray_disagree = 0;
        const auto origin = observer_oracle::to_octomap(geom, prepared.origin);
        std::set<observer_oracle::Cell> native_frees, native_hits;
        for (const auto& end_uld : prepared.uld_points) {
          const auto end = observer_oracle::to_octomap(geom, end_uld);
          octomap::KeyRay ray;
          probe.computeRayKeys(octomap::point3d(origin.x, origin.y, origin.z),
                               octomap::point3d(end.x, end.y, end.z), ray);
          std::vector<observer_oracle::Cell> nkeys;
          for (const auto& key : ray) {
            observer_oracle::Cell c{static_cast<int>(key[0]) - 32768, static_cast<int>(key[1]) - 32768,
                                    static_cast<int>(key[2]) - 32768};
            nkeys.push_back(c);
            if (observer_oracle::active(geom, c)) native_frees.insert(c);
          }
          const auto ikeys = observer_oracle::traverse_ray(origin, end, geom.resolution);
          if (nkeys != ikeys) ++ray_disagree;
          if (observer_oracle::contains(geom, end_uld)) {
            observer_oracle::Cell idx{observer_oracle::coord_to_index(end.x, geom.resolution),
                                      observer_oracle::coord_to_index(end.y, geom.resolution),
                                      observer_oracle::coord_to_index(end.z, geom.resolution)};
            if (observer_oracle::active(geom, idx)) native_hits.insert(idx);
          }
        }
        for (const auto& hit : native_hits) native_frees.erase(hit);
        native_free = native_frees.size();
        std::ofstream dump(out + ".mismatch.txt");
        dump << "seed=" << seed << " frame=" << f << " status=" << receipt.status
             << " sut_hit=" << receipt.hit_keys << " sut_free=" << receipt.free_keys
             << " indep_hit=" << keys.hits.size() << " indep_free=" << indep_free
             << " native_hit=" << native_hits.size() << " native_free=" << native_free
             << " ray_disagree=" << ray_disagree << " accepted=" << prepared.filter.kept.size()
             << "\n";
        break;
      }
      try {
        const auto snap = *store.snapshot();
        auto cmp = compare_touched(oracle, snap, store.diagnostics(), keys);
        if (!cmp.pass) {
          mismatches++;
          reason = cmp.reason;
          std::ofstream dump(out + ".mismatch.txt");
          dump << "seed=" << seed << " frame=" << f << " " << cmp.reason
               << " sut_known=" << store.diagnostics().known_cells
               << " oracle_known=" << oracle.known_count()
               << " sut_hit=" << receipt.hit_keys << " sut_free=" << receipt.free_keys
               << " indep_hit=" << keys.hits.size() << " indep_free=" << keys.frees.size()
               << "\n";
          break;
        }
      } catch (const Error& e) {
        mismatches++;
        reason = e.what();
        break;
      }
    }
    if (mismatches) break;
  }
  return write_report(out, mismatches == 0, "O1",
                      {{"frames", std::to_string(done)},
                       {"mismatches", std::to_string(mismatches)},
                       {"reason", reason},
                       {"known", std::to_string(oracle.known_count())}});
}

int run_filter_cases(const ObserverConfiguration& cfg, const std::string& out) {
  OpticalDepthPreprocessor pre(cfg.depth);
  int fail = 0;
  std::string reason = "ok";
  auto check = [&](Vec3 p, bool keep, const char* name) {
    const auto r = pre.prepare({p});
    const bool kept = r.optical_points.size() == 1;
    if (kept != keep) {
      ++fail;
      reason = name;
    }
  };
  check({0, 0, 0.52}, true, "z_min_inclusive");
  check({0, 0, std::nextafter(0.52, 0.0)}, false, "z_min_nextafter_below");
  check({0, 0, 0.519}, false, "z_min_minus_1mm");
  check({0, 0, 0.521}, true, "z_min_plus_1mm");
  check({0, 0, 6.0}, true, "z_max_inclusive");
  check({0, 0, std::nextafter(6.0, 7.0)}, false, "z_max_nextafter_above");
  check({0, 0, 6.001}, false, "z_max_plus_1mm");
  check({0, 0, 5.999}, true, "z_max_minus_1mm");
  // Off-axis Euclidean > 6 with Z <= 6 must keep.
  check({5.0, 0, 5.5}, true, "offaxis_z_le_6");
  const double r = std::sqrt(0.4 * 0.4 + 0.4 * 0.4 + 0.4 * 0.4);
  if (r < 0.52) {
    ++fail;
    reason = "near_euclid_setup";
  }
  check({0.4, 0.0, 0.40}, false, "z_below_even_if_euclid");
  ObservationStore store(cfg.geometry, cfg.resolution, cfg.depth, cfg.clock_domain, cfg.digest, cfg.discretize_endpoints);
  const auto before = store.diagnostics();
  auto tf = look_into_container(-1.15, 0, 1.2, 1000);
  auto obs = base_observation(cfg, "empty", 1000, tf);
  obs.points_m = {{0, 0, 0.1}, {0, 0, 7.0}};
  const auto receipt = store.integrate(obs);
  if (receipt.status != "NO_VALID_POINTS") {
    ++fail;
    reason = "empty_status";
  }
  const auto after = store.diagnostics();
  if (after.identity.observed_revision != before.identity.observed_revision ||
      after.identity.map_revision != before.identity.map_revision ||
      after.integrated != before.integrated) {
    ++fail;
    reason = "empty_refreshed";
  }
  return write_report(out, fail == 0, "O1-filter",
                      {{"failures", std::to_string(fail)}, {"reason", reason}});
}

std::size_t rss_kb() {
  std::ifstream status("/proc/self/status");
  std::string line;
  while (std::getline(status, line)) {
    if (line.rfind("VmRSS:", 0) == 0) return std::stoul(line.substr(6));
  }
  return 0;
}

bool query_cells(const OccupancyOracle& oracle, const Snapshot& snap,
                 const observer_oracle::Geometry& geom, std::size_t budget, std::string* reason) {
  std::size_t queried = 0;
  const std::size_t known_budget = budget / 2;
  const std::size_t stride =
      std::max<std::size_t>(1, oracle.known_count() / std::max<std::size_t>(1, known_budget));
  std::size_t i = 0;
  for (const auto& kv : oracle.known()) {
    if ((i++ % stride) != 0) continue;
    const auto sample = snap.cell(CellIndex{kv.first.x, kv.first.y, kv.first.z});
    if (!sample.known || !sample.active) {
      *reason = "query_missing_known";
      return false;
    }
    if (std::fabs(sample.value.log_odds - kv.second.log_odds) > 1e-6f) {
      *reason = "query_logodds";
      return false;
    }
    if (sample.value.occupied != kv.second.occupied) {
      *reason = "query_class";
      return false;
    }
    if (++queried >= known_budget) break;
  }
  const auto sh = observer_oracle::shape(geom);
  std::size_t unknown = 0;
  for (int z = 0; z < sh.z && queried + unknown < budget; z += std::max(1, sh.z / 40)) {
    for (int y = 0; y < sh.y && queried + unknown < budget; y += std::max(1, sh.y / 40)) {
      for (int x = 0; x < sh.x && queried + unknown < budget; x += std::max(1, sh.x / 40)) {
        observer_oracle::Cell cell{x, y, z};
        if (!observer_oracle::active(geom, cell)) continue;
        const auto sample = snap.cell(CellIndex{x, y, z});
        const auto it = oracle.known().find(cell);
        if (it == oracle.known().end()) {
          if (sample.known) {
            *reason = "query_false_known";
            return false;
          }
          ++unknown;
        } else {
          if (!sample.known || std::fabs(sample.value.log_odds - it->second.log_odds) > 1e-6f) {
            *reason = "query_grid_mismatch";
            return false;
          }
        }
      }
    }
  }
  if (queried + unknown < 1) {
    *reason = "query_none";
    return false;
  }
  return true;
}

bool mutation_cases(ObservationStore& store, const OpticalObservation& last, std::uint64_t epoch,
                    std::string* reason) {
  const auto before = store.diagnostics();
  auto dup = last;
  const auto dup_r = store.integrate(dup);
  if (dup_r.status != "ALREADY_PROCESSED") {
    *reason = "duplicate_status:" + dup_r.status;
    return false;
  }
  const auto after_dup = store.diagnostics();
  if (after_dup.known_cells != before.known_cells ||
      after_dup.identity.map_revision != before.identity.map_revision ||
      after_dup.identity.observed_revision != before.identity.observed_revision) {
    *reason = "duplicate_mutated";
    return false;
  }
  auto conflict = last;
  if (!conflict.points_m.empty()) conflict.points_m[0].z += 0.03;
  else conflict.points_m.push_back({0, 0, 1.3});
  const auto conf_r = store.integrate(conflict);
  if (conf_r.status != "CONFLICT") {
    *reason = "conflict_status:" + conf_r.status;
    return false;
  }
  auto late = last;
  late.identity.observation_id = last.identity.observation_id + "-order";
  late.identity.stamp_ns = last.identity.stamp_ns - 1;
  late.uld_from_optical.stamp_ns = late.identity.stamp_ns;
  const auto order_r = store.integrate(late);
  if (order_r.status != "OUT_OF_ORDER") {
    *reason = "order_status:" + order_r.status;
    return false;
  }
  auto epoch_obs = last;
  epoch_obs.identity.observation_id = last.identity.observation_id + "-epoch";
  epoch_obs.identity.epoch = epoch + 9;
  epoch_obs.identity.stamp_ns = last.identity.stamp_ns + 1'000'000'000LL;
  epoch_obs.uld_from_optical.stamp_ns = epoch_obs.identity.stamp_ns;
  const auto ep_r = store.integrate(epoch_obs);
  if (ep_r.status != "EPOCH_MISMATCH") {
    *reason = "epoch_status:" + ep_r.status;
    return false;
  }
  const auto after = store.diagnostics();
  if (after.known_cells != before.known_cells ||
      after.identity.map_revision != before.identity.map_revision) {
    *reason = "named_failure_mutated";
    return false;
  }
  return true;
}

int run_o2(const ObserverConfiguration& cfg, int seeds, int frames, int boxes, const std::string& out) {
  const auto geom = oracle_geometry(cfg);
  const auto depth = oracle_depth(cfg.depth);
  const auto box_list = make_boxes(boxes, geom);
  OccupancyOracle oracle(geom);
  ObservationStore store(cfg.geometry, cfg.resolution, cfg.depth, cfg.clock_domain, cfg.digest, cfg.discretize_endpoints);
  std::size_t mismatches = 0;
  std::string reason = "ok";
  std::size_t done = 0;
  std::size_t queries = 0;
  OpticalObservation last;
  for (int seed = 0; seed < seeds; ++seed) {
    for (int f = 0; f < frames; ++f) {
      const std::int64_t stamp = 1'000'000'000LL * (seed * frames + f + 1);
      auto tf = look_into_container(-1.15, 0.0, 1.20, stamp);
      auto obs = base_observation(cfg, "o2-" + std::to_string(seed) + "-" + std::to_string(f), stamp, tf);
      auto cloud_pts = generate_optical_cloud(geom, box_list, tf, 48, 0.2, 0.2,
                                              static_cast<uint32_t>(seed * 10007 + f), f == 0);
      obs.points_m = to_sut(cloud_pts);
      observer_oracle::Rigid rigid{tf.rotation, {tf.translation.x, tf.translation.y, tf.translation.z}};
      const auto prepared = observer_oracle::prepare(cloud_pts, rigid, depth);
      const auto keys = observer_oracle::evidence_keys(geom, prepared.origin, prepared.uld_points);
      oracle.apply_frame(keys);
      const auto receipt = store.integrate(obs);
      ++done;
      if (receipt.status != "INTEGRATED" && receipt.status != "NO_VALID_POINTS") {
        mismatches++;
        reason = receipt.status;
        break;
      }
      if (receipt.status == "NO_VALID_POINTS") continue;
      last = obs;
      if (receipt.hit_keys != keys.hits.size() || receipt.free_keys != keys.frees.size()) {
        mismatches++;
        reason = "key_counts";
        break;
      }
      try {
        const auto snap = *store.snapshot();
        auto cmp = compare_touched(oracle, snap, store.diagnostics(), keys);
        if (!cmp.pass) {
          mismatches++;
          reason = cmp.reason;
          break;
        }
      } catch (const Error& e) {
        mismatches++;
        reason = e.what();
        break;
      }
    }
    if (mismatches) break;
    try {
      const auto snap = *store.snapshot();
      if (!query_cells(oracle, snap, geom, 10000, &reason)) {
        mismatches++;
        break;
      }
      queries += 10000;
      if (!mutation_cases(store, last, store.diagnostics().identity.epoch, &reason)) {
        mismatches++;
        break;
      }
    } catch (const Error& e) {
      mismatches++;
      reason = e.what();
      break;
    }
  }
  return write_report(out, mismatches == 0, "O2",
                      {{"frames", std::to_string(done)},
                       {"queries", std::to_string(queries)},
                       {"mismatches", std::to_string(mismatches)},
                       {"reason", reason},
                       {"known", std::to_string(oracle.known_count())}});
}

int run_o3(const ObserverConfiguration& cfg, const std::string& out) {
  int repeats_ok = 0;
  std::string reason = "ok";
  std::uint64_t serializations = 0;
  int reader_fail_total = 0;
  int pressure_ok = 0;
  for (int rep = 0; rep < 3; ++rep) {
    ObservationStore store(cfg.geometry, cfg.resolution, cfg.depth, cfg.clock_domain, cfg.digest, cfg.discretize_endpoints);
    SnapshotRepository repo(store);
    auto tf0 = look_into_container(-1.15, 0, 1.2, 1'000'000'000);
    auto first = base_observation(cfg, "s0", 1'000'000'000, tf0);
    first.points_m = {{0, 0, 1.2}};
    if (store.integrate(first).status != "INTEGRATED") {
      reason = "seed_integrate";
      break;
    }
    std::atomic<int> fail{0};
    std::vector<std::thread> readers;
    for (int i = 0; i < 3; ++i) {
      readers.emplace_back([&] {
        for (int n = 0; n < 1000; ++n) {
          try {
            auto lease = repo.acquire(1'000'000'000'000LL, 1'000'000'000'000LL, true);
            auto snap = repo.get(lease.handle);
            const auto id1 = snap->identity();
            const auto page = snap->read_cells({0, 0, 0}, {8, 8, 8}, 0, 512);
            (void)page.done;
            if (snap->identity().epoch != id1.epoch) ++fail;
            repo.release(lease.handle);
          } catch (...) {
            ++fail;
          }
        }
      });
    }
    for (int f = 1; f < 1000; ++f) {
      auto tf = look_into_container(-1.15, 0, 1.2, 1'000'000'000LL * (f + 1));
      auto obs = base_observation(cfg, "s" + std::to_string(f), 1'000'000'000LL * (f + 1), tf);
      obs.points_m = {{0.01 * (f % 5), 0.0, 1.2}};
      store.integrate(obs);
    }
    for (auto& t : readers) t.join();
    reader_fail_total += fail.load();
    try {
      std::vector<std::string> handles;
      for (int i = 0; i < 8; ++i)
        handles.push_back(repo.acquire(1'000'000'000'000LL, 1'000'000'000'000LL, true).handle);
      bool pressure = false;
      try {
        repo.acquire(1'000'000'000'000LL, 1'000'000'000'000LL, true);
      } catch (const Error& e) {
        pressure = std::string(e.what()).find("PRESSURE") != std::string::npos;
      }
      for (const auto& h : handles) repo.release(h);
      const auto d = store.diagnostics();
      serializations += d.full_serializations;
      if (fail.load() == 0 && pressure && d.full_serializations == 0) {
        ++repeats_ok;
        ++pressure_ok;
      } else {
        reason = fail.load() ? "reader_fail" : (pressure ? "serialized" : "no_pressure");
      }
    } catch (const Error& e) {
      reason = e.what();
    }
  }
  return write_report(out, repeats_ok == 3, "O3",
                      {{"repeats_ok", std::to_string(repeats_ok)},
                       {"reader_fail", std::to_string(reader_fail_total)},
                       {"pressure_ok", std::to_string(pressure_ok)},
                       {"full_serializations", std::to_string(serializations)},
                       {"reason", reason}});
}

int run_o4(const ObserverConfiguration& cfg, const std::string& dir) {
  int overwrite_ok = 0, restore_ok = 0, corrupt_ok = 0, missing_ok = 0;
  std::string reason = "ok";
  std::uint64_t last_epoch = 0;
  for (int rep = 0; rep < 3; ++rep) {
    ObservationStore store(cfg.geometry, cfg.resolution, cfg.depth, cfg.clock_domain, cfg.digest, cfg.discretize_endpoints);
    auto tf = look_into_container(-1.15, 0, 1.2, 1'000'000'000);
    auto obs = base_observation(cfg, "c0", 1'000'000'000, tf);
    obs.points_m = {{0, 0, 1.1}};
    if (store.integrate(obs).status != "INTEGRATED") {
      reason = "seed";
      break;
    }
    const auto ckpt_path = dir + ".ckpt." + std::to_string(rep);
    std::error_code ec;
    std::filesystem::remove(ckpt_path, ec);
    write_checkpoint(store.checkpoint(), ckpt_path);
    try {
      write_checkpoint(store.checkpoint(), ckpt_path);
    } catch (const Error& e) {
      if (std::string(e.what()) == "CHECKPOINT_ALREADY_EXISTS") ++overwrite_ok;
    }
    const auto epoch_before = store.diagnostics().identity.epoch;
    store.reset(epoch_before);
    last_epoch = store.restore(read_checkpoint(ckpt_path), store.diagnostics().identity.epoch);
    if (last_epoch != epoch_before) ++restore_ok;
    (void)store.snapshot()->cell(CellIndex{0, 0, 0});
  }
  {
    const auto missing = dir + ".ckpt.missing";
    std::error_code ec;
    std::filesystem::remove(missing, ec);
    for (int rep = 0; rep < 3; ++rep) {
      try {
        read_checkpoint(missing);
      } catch (const Error&) {
        ++missing_ok;
      }
    }
  }
  for (int rep = 0; rep < 3; ++rep) {
    const auto ckpt_path = dir + ".ckpt.corrupt." + std::to_string(rep);
    std::error_code ec;
    std::filesystem::remove(ckpt_path, ec);
    ObservationStore store(cfg.geometry, cfg.resolution, cfg.depth, cfg.clock_domain, cfg.digest, cfg.discretize_endpoints);
    auto tf = look_into_container(-1.15, 0, 1.2, 1'000'000'000);
    auto obs = base_observation(cfg, "c0", 1'000'000'000, tf);
    obs.points_m = {{0, 0, 1.1}};
    store.integrate(obs);
    write_checkpoint(store.checkpoint(), ckpt_path);
    {
      std::ofstream f(ckpt_path, std::ios::app);
      f << "corrupt";
    }
    const auto before = store.diagnostics();
    try {
      read_checkpoint(ckpt_path);
    } catch (const Error&) {
      ++corrupt_ok;
    }
    const auto after = store.diagnostics();
    if (after.known_cells != before.known_cells || after.identity.epoch != before.identity.epoch)
      reason = "corrupt_mutated_state";
  }
  const bool pass = overwrite_ok == 3 && restore_ok == 3 && corrupt_ok == 3 && missing_ok == 3 &&
                    reason == "ok";
  return write_report(dir, pass, "O4",
                      {{"overwrite_ok", std::to_string(overwrite_ok)},
                       {"restore_ok", std::to_string(restore_ok)},
                       {"corrupt_ok", std::to_string(corrupt_ok)},
                       {"missing_ok", std::to_string(missing_ok)},
                       {"new_epoch", std::to_string(last_epoch)},
                       {"reason", reason}});
}

int run_o8(const ObserverConfiguration& cfg, int boxes, const std::string& out) {
  const auto geom = oracle_geometry(cfg);
  const auto box_list = make_boxes(boxes, geom);
  ObservationStore store(cfg.geometry, cfg.resolution, cfg.depth, cfg.clock_domain, cfg.digest, cfg.discretize_endpoints);
  auto tf = look_into_container(-1.15, 0, 1.2, 1'000'000'000);
  auto cloud = to_sut(generate_optical_cloud(geom, box_list, tf, 128, 0.0, 0.0, 7, false));
  std::vector<std::size_t> rss_warm, rss_a, rss_b;
  std::size_t changed_after = 0;
  std::size_t occupancy_after = 0;
  std::size_t known_sat = 0;
  int f = 0;
  auto step = [&](bool score_window, std::vector<std::size_t>* rss) {
    ++f;
    const std::int64_t stamp = 1'000'000'000LL * f;
    auto obs = base_observation(cfg, "o8-" + std::to_string(f), stamp, look_into_container(-1.15, 0, 1.2, stamp));
    obs.points_m = cloud;
    const auto receipt = store.integrate(obs);
    if (receipt.status != "INTEGRATED" && receipt.status != "ALREADY_PROCESSED")
      throw Error(receipt.status);
    if (score_window) {
      changed_after += receipt.changed_keys;
      occupancy_after += receipt.occupancy_changed_keys;
      rss->push_back(rss_kb());
    }
  };
  try {
    for (int i = 0; i < 120; ++i) step(false, &rss_warm);
    known_sat = store.diagnostics().known_cells;
    for (int i = 0; i < 120; ++i) step(true, &rss_a);
    for (int i = 0; i < 120; ++i) step(true, &rss_b);
    for (int i = 0; i < 30; ++i) step(true, &rss_b);
    const auto known_end = store.diagnostics().known_cells;
    auto median = [](std::vector<std::size_t> v) {
      if (v.empty()) return std::size_t{0};
      std::sort(v.begin(), v.end());
      return v[v.size() / 2];
    };
    const auto ma = median(rss_a);
    const auto mb = median(rss_b);
    const bool rss_ok = ma == 0 || mb <= ma + static_cast<std::size_t>(0.05 * static_cast<double>(ma));
    const bool pass = changed_after == 0 && occupancy_after == 0 && known_end == known_sat && rss_ok;
    return write_report(out, pass, "O8",
                        {{"changed_after_sat", std::to_string(changed_after)},
                         {"known_sat", std::to_string(known_sat)},
                         {"known_end", std::to_string(known_end)},
                         {"rss_median_a_kb", std::to_string(ma)},
                         {"rss_median_b_kb", std::to_string(mb)},
                         {"frames", std::to_string(f)}});
  } catch (const Error& e) {
    return write_report(out, false, "O8", {{"reason", e.what()}});
  }
}

double percentile(std::vector<double> values, double q) {
  if (values.empty()) return 0;
  std::sort(values.begin(), values.end());
  const std::size_t idx = static_cast<std::size_t>(std::ceil(q * values.size())) - 1;
  return values[std::min(idx, values.size() - 1)];
}

int run_bench(const ObserverConfiguration& cfg, int boxes, std::size_t cloud, int warmup, int repeat,
              const std::string& out, bool capture) {
  const auto geom = oracle_geometry(cfg);
  const auto box_list = make_boxes(boxes, geom);
  std::vector<double> first_surface_ms, first_integrate_ms;
  for (int i = 0; i < repeat; ++i) {
    ObservationStore cold(cfg.geometry, cfg.resolution, cfg.depth, cfg.clock_domain, cfg.digest, cfg.discretize_endpoints);
    SnapshotRepository cold_repo(cold);
    const std::int64_t stamp = 1'000'000'000LL * (i + 1);
    auto tf = look_into_container(-1.15, 0, 1.2, stamp);
    auto obs = base_observation(cfg, "first" + std::to_string(i), stamp, tf);
    obs.points_m = to_sut(generate_optical_cloud(geom, box_list, tf, cloud, 0.5, 0.0,
                                                static_cast<uint32_t>(i + 9000), false));
    const auto t0 = Clock::now();
    const auto receipt = cold.integrate(obs);
    const auto t1 = Clock::now();
    if (receipt.status != "INTEGRATED" && receipt.status != "NO_VALID_POINTS")
      throw Error(receipt.status);
    auto lease = cold_repo.acquire(stamp + 10, 2'000'000'000, true);
    auto snap = cold_repo.get(lease.handle);
    const auto t2 = Clock::now();
    (void)snap->surface();
    const auto t3 = Clock::now();
    cold_repo.release(lease.handle);
    auto ms = [](Clock::time_point a, Clock::time_point b) {
      return std::chrono::duration<double, std::milli>(b - a).count();
    };
    first_integrate_ms.push_back(ms(t0, t1));
    first_surface_ms.push_back(ms(t2, t3));
    if (capture) (void)cold.diagnostics();
  }
  ObservationStore store(cfg.geometry, cfg.resolution, cfg.depth, cfg.clock_domain, cfg.digest, cfg.discretize_endpoints);
  std::vector<double> integrate_ms, acquire_ms, roi_ms, surface_ms;
  SnapshotRepository repo(store);
  auto once = [&](int i, bool score) {
    const std::int64_t stamp = 1'000'000'000LL * (i + 1);
    auto tf = look_into_container(-1.15, 0, 1.2, stamp);
    auto obs = base_observation(cfg, "b" + std::to_string(i), stamp, tf);
    obs.points_m = to_sut(generate_optical_cloud(geom, box_list, tf, cloud, 0.5, 0.0,
                                                static_cast<uint32_t>(i + 1), false));
    const auto t0 = Clock::now();
    const auto receipt = store.integrate(obs);
    const auto t1 = Clock::now();
    if (receipt.status != "INTEGRATED" && receipt.status != "NO_VALID_POINTS")
      throw Error(receipt.status);
    auto lease = repo.acquire(stamp + 10, 2'000'000'000, true);
    const auto t2 = Clock::now();
    auto snap = repo.get(lease.handle);
    const auto t3 = Clock::now();
    (void)snap->read_cells({0, 0, 0}, {16, 16, 16}, 0, 4096);
    const auto t4 = Clock::now();
    (void)snap->surface();
    const auto t5 = Clock::now();
    repo.release(lease.handle);
    if (!score) return;
    auto ms = [](Clock::time_point a, Clock::time_point b) {
      return std::chrono::duration<double, std::milli>(b - a).count();
    };
    integrate_ms.push_back(ms(t0, t1));
    acquire_ms.push_back(ms(t1, t2));
    roi_ms.push_back(ms(t3, t4));
    surface_ms.push_back(ms(t4, t5));
    if (capture) (void)store.diagnostics();
  };
  for (int i = 0; i < warmup; ++i) once(i, false);
  for (int i = 0; i < repeat; ++i) once(warmup + i, true);
  const bool pass = percentile(integrate_ms, 0.95) <= (cloud >= 100000 ? 2000.0 : 500.0) &&
                    *std::max_element(integrate_ms.begin(), integrate_ms.end()) <=
                        (cloud >= 100000 ? 5000.0 : 1000.0) &&
                    percentile(acquire_ms, 0.95) <= 100.0 &&
                    percentile(roi_ms, 0.95) <= 100.0 && percentile(surface_ms, 0.95) <= 100.0 &&
                    percentile(first_surface_ms, 0.95) <= 100.0 &&
                    *std::max_element(first_surface_ms.begin(), first_surface_ms.end()) <= 250.0;
  return write_report(out, pass, "bench",
                      {{"p95_integrate_ms", std::to_string(percentile(integrate_ms, 0.95))},
                       {"max_integrate_ms", std::to_string(*std::max_element(integrate_ms.begin(), integrate_ms.end()))},
                       {"p95_acquire_ms", std::to_string(percentile(acquire_ms, 0.95))},
                       {"p95_roi_ms", std::to_string(percentile(roi_ms, 0.95))},
                       {"p95_surface_ms", std::to_string(percentile(surface_ms, 0.95))},
                       {"p95_first_surface_ms", std::to_string(percentile(first_surface_ms, 0.95))},
                       {"max_first_surface_ms", std::to_string(*std::max_element(first_surface_ms.begin(), first_surface_ms.end()))},
                       {"p95_first_integrate_ms", std::to_string(percentile(first_integrate_ms, 0.95))},
                       {"n", std::to_string(repeat)}});
}

}  // namespace

int run_config_wire(const ObserverConfiguration& cfg, const std::string& out) {
  UldDomain domain(cfg.geometry, cfg.resolution);
  ObservationStore store(cfg.geometry, cfg.resolution, cfg.depth, cfg.clock_domain, cfg.digest,
                         cfg.discretize_endpoints);
  RigidTransform tf;
  tf.stamp_ns = 1000;
  const Vec3 origin{0, 0, 0};
  octomap::OcTree probe(cfg.resolution);
  bool found = false;
  Vec3 first, second;
  const auto shape = domain.shape();
  for (int x = 0; x < shape.x && !found; ++x) {
    for (int y = 0; y < shape.y && !found; ++y) {
      for (int z = 0; z < shape.z && !found; ++z) {
        if (!domain.active({x, y, z})) continue;
        const auto center = domain.center({x, y, z});
        const Vec3 a{center.x - 0.4 * cfg.resolution, center.y - 0.4 * cfg.resolution,
                     center.z - 0.4 * cfg.resolution};
        const Vec3 b{center.x + 0.4 * cfg.resolution, center.y + 0.4 * cfg.resolution,
                     center.z + 0.4 * cfg.resolution};
        if (a.z < cfg.depth.min_m || b.z < cfg.depth.min_m || a.z > cfg.depth.max_m || b.z > cfg.depth.max_m)
          continue;
        if (!domain.contains(a) || !domain.contains(b)) continue;
        octomap::OcTreeKey ka, kb;
        const auto oa = domain.octomap_coordinates(a);
        const auto ob = domain.octomap_coordinates(b);
        if (!probe.coordToKeyChecked(oa.x, oa.y, oa.z, ka) || !probe.coordToKeyChecked(ob.x, ob.y, ob.z, kb) ||
            !(ka == kb))
          continue;
        auto obs_a = base_observation(cfg, "a", 1000, tf);
        obs_a.points_m = {a};
        auto obs_b = base_observation(cfg, "b", 2000, tf);
        obs_b.points_m = {b};
        ObservationStore sa(cfg.geometry, cfg.resolution, cfg.depth, cfg.clock_domain, cfg.digest, false);
        ObservationStore sb(cfg.geometry, cfg.resolution, cfg.depth, cfg.clock_domain, cfg.digest, false);
        const auto ra = sa.integrate(obs_a);
        const auto rb = sb.integrate(obs_b);
        if (ra.free_keys != rb.free_keys) {
          first = a;
          second = b;
          found = true;
        }
      }
    }
  }
  if (!found) return write_report(out, false, "config-wire", {{"reason", "no_pair"}});
  auto paired = base_observation(cfg, "pair", 1000, tf);
  paired.points_m = {first, second};
  const auto receipt = store.integrate(paired);
  std::ostringstream ax, ay, az, bx, by, bz;
  ax << first.x; ay << first.y; az << first.z;
  bx << second.x; by << second.y; bz << second.z;
  return write_report(out, receipt.status == "INTEGRATED", "config-wire",
                      {{"discretize", cfg.discretize_endpoints ? "true" : "false"},
                       {"free_keys", std::to_string(receipt.free_keys)},
                       {"hit_keys", std::to_string(receipt.hit_keys)},
                       {"ax", ax.str()}, {"ay", ay.str()}, {"az", az.str()},
                       {"bx", bx.str()}, {"by", by.str()}, {"bz", bz.str()}});
}

int main(int argc, char** argv) {
  try {
    std::map<std::string, std::string> opt;
    for (int i = 1; i + 1 < argc; i += 2) opt[argv[i]] = argv[i + 1];
    if (opt.count("--help") || argc == 1) {
      std::cout << "observer_eval_native --geometry G --config C --gate GATE --out FILE "
                   "[--seeds N --frames N --boxes N --cloud N --outside F --cross F "
                   "--warmup N --repeat N]\n";
      return 0;
    }
    const auto cfg = load_configuration(opt.at("--geometry"), opt.at("--config"));
    const auto gate = opt.at("--gate");
    const auto out = opt.at("--out");
    const int seeds = std::stoi(opt.count("--seeds") ? opt["--seeds"] : "10");
    const int frames = std::stoi(opt.count("--frames") ? opt["--frames"] : "100");
    const int boxes = std::stoi(opt.count("--boxes") ? opt["--boxes"] : "0");
    const std::size_t cloud = std::stoul(opt.count("--cloud") ? opt["--cloud"] : "128");
    const double outside = std::stod(opt.count("--outside") ? opt["--outside"] : "0.5");
    const double cross = std::stod(opt.count("--cross") ? opt["--cross"] : "0.0");
    const int warmup = std::stoi(opt.count("--warmup") ? opt["--warmup"] : "3");
    const int repeat = std::stoi(opt.count("--repeat") ? opt["--repeat"] : "30");
    if (gate == "config-wire") return run_config_wire(cfg, out);
    if (gate == "o1-filter") return run_filter_cases(cfg, out);
    if (gate == "o1") return run_o1(cfg, seeds, frames, boxes, cloud, outside, cross, out);
    if (gate == "o2") return run_o2(cfg, seeds, frames, boxes, out);
    if (gate == "o3") return run_o3(cfg, out);
    if (gate == "o4") return run_o4(cfg, out);
    if (gate == "o8") return run_o8(cfg, boxes, out);
    if (gate == "bench")
      return run_bench(cfg, boxes, cloud, warmup, repeat, out, false);
    if (gate == "bench-capture")
      return run_bench(cfg, boxes, cloud, warmup, repeat, out, true);
    throw Error("UNKNOWN_GATE");
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 3;
  }
}
