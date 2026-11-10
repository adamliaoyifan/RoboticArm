#include "load_model.hpp"
#include <openssl/evp.h>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <random>
#include <sstream>
#include <stdexcept>

namespace occ_load {
namespace {

using observer_oracle::Rigid;
using observer_oracle::Vec3;
using observer_oracle::Geometry;

constexpr double kEps = 1e-9;

std::string sha256_bytes(const std::string& bytes) {
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned int size = 0;
  if (EVP_Digest(bytes.data(), bytes.size(), digest, &size, EVP_sha256(), nullptr) != 1)
    throw std::runtime_error("HASH_FAILURE");
  std::ostringstream out;
  for (unsigned int i = 0; i < size; ++i)
    out << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(digest[i]);
  return out.str();
}

std::size_t floor_product(long double value) {
  if (value < 0) throw std::runtime_error("NEGATIVE_QUOTA");
  return static_cast<std::size_t>(std::floor(value + 1e-9L));
}

Vec3 to_optical(const Rigid& pose, const Vec3& uld) {
  const double dx = uld.x - pose.translation.x;
  const double dy = uld.y - pose.translation.y;
  const double dz = uld.z - pose.translation.z;
  const auto& r = pose.rotation;
  return {r[0] * dx + r[3] * dy + r[6] * dz,
          r[1] * dx + r[4] * dy + r[7] * dz,
          r[2] * dx + r[5] * dy + r[8] * dz};
}

struct Interval {
  double lo{0};
  double hi{1};
  bool ok{true};
};

Interval clip_halfspace(Interval in, const Vec3& origin, const Vec3& direction,
                        double nx, double ny, double nz, double limit) {
  if (!in.ok) return in;
  const double along = nx * direction.x + ny * direction.y + nz * direction.z;
  const double at = nx * origin.x + ny * origin.y + nz * origin.z;
  if (std::fabs(along) <= 1e-15) {
    if (at > limit) in.ok = false;
    return in;
  }
  const double hit = (limit - at) / along;
  if (along > 0) in.hi = std::min(in.hi, hit);
  else in.lo = std::max(in.lo, hit);
  if (!(in.lo < in.hi)) in.ok = false;
  return in;
}

Interval clip_aabb(Interval in, const Vec3& origin, const Vec3& direction,
                   double x_min, double x_max, double y_min, double y_max,
                   double z_min, double z_max) {
  in = clip_halfspace(in, origin, direction, 1, 0, 0, x_max);
  in = clip_halfspace(in, origin, direction, -1, 0, 0, -x_min);
  in = clip_halfspace(in, origin, direction, 0, 1, 0, y_max);
  in = clip_halfspace(in, origin, direction, 0, -1, 0, -y_min);
  in = clip_halfspace(in, origin, direction, 0, 0, 1, z_max);
  in = clip_halfspace(in, origin, direction, 0, 0, -1, -z_min);
  return in;
}

bool open_overlap(const Interval& interval) {
  if (!interval.ok) return false;
  const double lo = std::max(interval.lo, 1e-8);
  const double hi = std::min(interval.hi, 1.0 - 1e-8);
  return hi - lo > 1e-8;
}

bool segment_hits_domain(const Geometry& geometry, const Vec3& origin, const Vec3& end) {
  const Vec3 direction{end.x - origin.x, end.y - origin.y, end.z - origin.z};
  const double hx = geometry.length / 2 + kEps;
  const double hy = geometry.width / 2 + kEps;
  const double x_min = -geometry.length / 2 - kEps;
  const double y_min = -geometry.width / 2 - kEps;
  const double z_min = geometry.floor_z - kEps;
  const double z_max = geometry.ceiling_z + kEps;
  if (!geometry.has_chamfer) {
    return open_overlap(clip_aabb({}, origin, direction, x_min, hx, y_min, hy, z_min, z_max));
  }
  const double wall = geometry.chamfer.wall_z - kEps;
  const double slope = (geometry.chamfer.wall_y - geometry.chamfer.floor_y) /
                       (geometry.chamfer.wall_z - geometry.floor_z);
  Interval upper = clip_aabb({}, origin, direction, x_min, hx, y_min, hy, wall, z_max);
  Interval lower = clip_aabb({}, origin, direction, x_min, hx, y_min, hy, z_min, wall);
  const double limit = geometry.chamfer.floor_y - slope * geometry.floor_z + kEps;
  lower = clip_halfspace(lower, origin, direction, 0, 1, -slope, limit);
  return open_overlap(upper) || open_overlap(lower);
}

Vec3 sample_surface(const Box& box, std::mt19937_64& rng) {
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  std::uniform_int_distribution<int> face(0, 5);
  Vec3 point = box.center;
  switch (face(rng)) {
    case 0: point.x -= box.half.x; point.y += box.half.y * unit(rng); point.z += box.half.z * unit(rng); break;
    case 1: point.x += box.half.x; point.y += box.half.y * unit(rng); point.z += box.half.z * unit(rng); break;
    case 2: point.y -= box.half.y; point.x += box.half.x * unit(rng); point.z += box.half.z * unit(rng); break;
    case 3: point.y += box.half.y; point.x += box.half.x * unit(rng); point.z += box.half.z * unit(rng); break;
    case 4: point.z -= box.half.z; point.x += box.half.x * unit(rng); point.y += box.half.y * unit(rng); break;
    default: point.z += box.half.z; point.x += box.half.x * unit(rng); point.y += box.half.y * unit(rng); break;
  }
  return point;
}

struct Sampler {
  double ix0{}, ix1{}, iy0{}, iy1{}, iz0{}, iz1{};
  double mx0{}, mx1{}, my0{}, my1{}, mz0{}, mz1{};
  double cx0{}, cx1{}, cy0{}, cy1{}, cz0{}, cz1{};
};

Sampler make_sampler(const Geometry& geometry, const Rigid& pose, double z_min, double z_max) {
  const Vec3 origin = observer_oracle::apply(pose, {0, 0, 0});
  const double hx = geometry.length / 2;
  const double hy = geometry.width / 2;
  const double valid_x0 = pose.translation.x + z_min;
  const double valid_x1 = pose.translation.x + z_max;
  Sampler out;
  out.ix0 = std::max(-hx + 0.05, valid_x0 + 0.05);
  out.ix1 = std::min(hx - 0.05, valid_x1 - 0.05);
  out.iy0 = -hy + 0.05;
  out.iy1 = geometry.has_chamfer ? std::min(geometry.chamfer.floor_y - 0.05, hy - 0.05) : hy - 0.05;
  out.iz0 = geometry.floor_z + 0.08;
  out.iz1 = geometry.ceiling_z - 0.08;
  if (!(out.ix0 < out.ix1 && out.iy0 < out.iy1 && out.iz0 < out.iz1))
    throw std::runtime_error("INTERIOR_REGION_EMPTY");
  out.mx0 = out.ix0;
  out.mx1 = std::min(out.ix0 + 0.10, out.ix1);
  const double dx = out.mx1 - origin.x;
  const double t_enter = (-hx - 0.02 - origin.x) / dx;
  if (!(dx > 0 && t_enter > 0.05)) throw std::runtime_error("MISS_REGION_EMPTY");
  const double ey = origin.y + (hy - origin.y) / t_enter + 0.05;
  out.my0 = ey;
  out.my1 = ey + 0.35;
  out.mz0 = std::max(geometry.floor_z + 0.15, 0.8);
  out.mz1 = std::min(geometry.ceiling_z - 0.15, 1.7);
  if (!(out.mx0 < out.mx1 && out.mz0 < out.mz1)) throw std::runtime_error("MISS_REGION_EMPTY");
  out.cx0 = hx + 0.15;
  out.cx1 = hx + 0.45;
  out.cy0 = -0.15;
  out.cy1 = 0.15;
  out.cz0 = geometry.floor_z + 0.25;
  out.cz1 = std::min(geometry.ceiling_z - 0.25, geometry.floor_z + 1.2);
  if (!(out.cx0 < out.cx1 && out.cz0 < out.cz1)) throw std::runtime_error("CROSS_REGION_EMPTY");
  return out;
}

Vec3 uniform_box(std::mt19937_64& rng, double x0, double x1, double y0, double y1, double z0, double z1) {
  std::uniform_real_distribution<double> ux(x0, x1);
  std::uniform_real_distribution<double> uy(y0, y1);
  std::uniform_real_distribution<double> uz(z0, z1);
  return {ux(rng), uy(rng), uz(rng)};
}

void accept(std::vector<Vec3>& sink, const Geometry& geometry, const Rigid& pose,
            const Vec3& uld, EndpointClass expected, double z_min, double z_max,
            std::size_t& attempts) {
  ++attempts;
  const Vec3 optical = to_optical(pose, uld);
  if (classify_endpoint(geometry, pose, optical, z_min, z_max) != expected)
    throw std::runtime_error("SAMPLER_CLASS_REJECTED");
  sink.push_back(optical);
}

std::string counts_json(const ClassCounts& counts) {
  std::ostringstream out;
  out << "{\"interior\":" << counts.interior << ",\"cross\":" << counts.cross
      << ",\"miss\":" << counts.miss << ",\"invalid\":" << counts.invalid << "}";
  return out.str();
}

void require(bool condition, const char* message, std::string& errors) {
  if (!condition) {
    if (!errors.empty()) errors += ";";
    errors += message;
  }
}

}  // namespace

Quotas quotas_for(std::size_t valid_count, double outside_fraction, double cross_fraction) {
  if (!(outside_fraction >= 0 && outside_fraction <= 1 && cross_fraction >= 0 && cross_fraction <= 1))
    throw std::runtime_error("LOAD_FRACTION_OUT_OF_RANGE");
  const long double n = static_cast<long double>(valid_count);
  Quotas out;
  out.cross = floor_product(n * static_cast<long double>(outside_fraction) *
                            static_cast<long double>(cross_fraction));
  out.miss = floor_product(n * static_cast<long double>(outside_fraction) *
                           (1.0L - static_cast<long double>(cross_fraction)));
  if (out.cross + out.miss > valid_count) throw std::runtime_error("QUOTA_OVERFLOW");
  out.interior = valid_count - out.cross - out.miss;
  return out;
}

std::uint32_t frame_seed(const std::string& group_id, int base_seed, int round_index) {
  const std::string text = std::string(kGeneratorVersion) + "|" + group_id + "|" +
                           std::to_string(base_seed) + "|" + std::to_string(round_index);
  const std::string digest = sha256_bytes(text);
  std::uint32_t seed = 0;
  for (int i = 0; i < 4; ++i) {
    const int hi = std::isdigit(digest[i * 2]) ? digest[i * 2] - '0' : digest[i * 2] - 'a' + 10;
    const int lo = std::isdigit(digest[i * 2 + 1]) ? digest[i * 2 + 1] - '0' : digest[i * 2 + 1] - 'a' + 10;
    seed |= static_cast<std::uint32_t>((hi << 4) | lo) << (8 * i);
  }
  return seed;
}

observer_oracle::Rigid sensor_pose() {
  observer_oracle::Rigid pose;
  pose.rotation = {0, 0, 1, -1, 0, 0, 0, -1, 0};
  pose.translation = {-1.15, 0.0, 1.20};
  return pose;
}

std::vector<Box> make_boxes(int count, const Geometry& geometry) {
  std::vector<Box> boxes;
  const Vec3 half{0.10, 0.10, 0.10};
  int placed = 0;
  for (int layer = 0; layer < 2 && placed < count; ++layer) {
    const int rows = layer == 0 ? 3 : 1;
    for (int row = 0; row < rows && placed < count; ++row) {
      for (int col = 0; col < 5 && placed < count; ++col) {
        boxes.push_back({{ -0.45 + col * 0.22, -0.55 + row * 0.28,
                           geometry.floor_z + half.z + layer * (2 * half.z + 0.01)},
                         half});
        ++placed;
      }
    }
  }
  return boxes;
}

EndpointClass classify_endpoint(const Geometry& geometry, const Rigid& pose,
                                const Vec3& optical, double z_min, double z_max) {
  if (!observer_oracle::finite(optical) || optical.z < z_min || optical.z > z_max)
    return EndpointClass::Invalid;
  const Vec3 end = observer_oracle::apply(pose, optical);
  if (observer_oracle::contains(geometry, end)) return EndpointClass::Interior;
  const Vec3 origin = observer_oracle::apply(pose, {0, 0, 0});
  if (segment_hits_domain(geometry, origin, end)) return EndpointClass::ExteriorCross;
  return EndpointClass::ExteriorMiss;
}

ClassCounts classify_cloud(const Geometry& geometry, const Rigid& pose,
                           const std::vector<Vec3>& optical, double z_min, double z_max) {
  ClassCounts counts;
  for (const auto& point : optical) {
    switch (classify_endpoint(geometry, pose, point, z_min, z_max)) {
      case EndpointClass::Interior: ++counts.interior; break;
      case EndpointClass::ExteriorCross: ++counts.cross; break;
      case EndpointClass::ExteriorMiss: ++counts.miss; break;
      case EndpointClass::Invalid: ++counts.invalid; break;
    }
  }
  return counts;
}

Cloud generate_cloud(const Geometry& geometry, const std::vector<Box>& boxes,
                     std::size_t valid_count, double outside_fraction, double cross_fraction,
                     std::uint32_t seed, double z_min, double z_max) {
  Cloud cloud;
  cloud.quotas = quotas_for(valid_count, outside_fraction, cross_fraction);
  const Rigid pose = sensor_pose();
  const Sampler region = make_sampler(geometry, pose, z_min, z_max);
  std::mt19937_64 rng(seed);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  std::uniform_int_distribution<int> side(0, 1);
  std::vector<Vec3> interior, cross, miss;
  interior.reserve(cloud.quotas.interior);
  cross.reserve(cloud.quotas.cross);
  miss.reserve(cloud.quotas.miss);
  auto propose = [&](EndpointClass expected, std::size_t needed, std::vector<Vec3>& sink, auto draw) {
    std::size_t guard = 0;
    while (sink.size() < needed) {
      if (++guard > needed * 40 + 100) throw std::runtime_error("LOAD_SAMPLE_EXHAUSTED");
      try {
        accept(sink, geometry, pose, draw(), expected, z_min, z_max, cloud.attempts);
      } catch (const std::runtime_error& error) {
        if (std::string(error.what()) != "SAMPLER_CLASS_REJECTED") throw;
      }
    }
  };
  propose(EndpointClass::Interior, cloud.quotas.interior, interior, [&] {
    if (!boxes.empty() && unit(rng) < kInteriorBoxAttempt)
      return sample_surface(boxes[static_cast<std::size_t>(unit(rng) * boxes.size()) % boxes.size()], rng);
    return uniform_box(rng, region.ix0, region.ix1, region.iy0, region.iy1, region.iz0, region.iz1);
  });
  propose(EndpointClass::ExteriorMiss, cloud.quotas.miss, miss, [&] {
    Vec3 point = uniform_box(rng, region.mx0, region.mx1, region.my0, region.my1, region.mz0, region.mz1);
    if (side(rng)) point.y = -point.y;
    return point;
  });
  propose(EndpointClass::ExteriorCross, cloud.quotas.cross, cross, [&] {
    return uniform_box(rng, region.cx0, region.cx1, region.cy0, region.cy1, region.cz0, region.cz1);
  });
  cloud.optical.reserve(valid_count);
  cloud.optical.insert(cloud.optical.end(), interior.begin(), interior.end());
  cloud.optical.insert(cloud.optical.end(), miss.begin(), miss.end());
  cloud.optical.insert(cloud.optical.end(), cross.begin(), cross.end());
  for (std::size_t i = cloud.optical.size(); i > 1; --i) {
    std::uniform_int_distribution<std::size_t> pick(0, i - 1);
    std::swap(cloud.optical[i - 1], cloud.optical[pick(rng)]);
  }
  cloud.counts = classify_cloud(geometry, pose, cloud.optical, z_min, z_max);
  if (cloud.counts.interior != cloud.quotas.interior || cloud.counts.cross != cloud.quotas.cross ||
      cloud.counts.miss != cloud.quotas.miss || cloud.counts.invalid != 0)
    throw std::runtime_error("CLASSIFIER_QUOTA_MISMATCH");
  std::string bytes;
  bytes.resize(cloud.optical.size() * 3 * sizeof(double));
  for (std::size_t i = 0; i < cloud.optical.size(); ++i) {
    const double xyz[3] = {cloud.optical[i].x, cloud.optical[i].y, cloud.optical[i].z};
    std::memcpy(bytes.data() + i * sizeof(xyz), xyz, sizeof(xyz));
  }
  cloud.sha256 = sha256_bytes(bytes);
  return cloud;
}

CheckResult self_check(const Geometry& loaded, double z_min, double z_max) {
  std::string errors;
  const auto pose = sensor_pose();
  Geometry chamfer = loaded;
  require(std::fabs(chamfer.length - 1.49) < 1e-9, "length", errors);
  require(std::fabs(chamfer.width - 1.97) < 1e-9, "width", errors);
  require(chamfer.has_chamfer, "chamfer", errors);
  const Quotas half = quotas_for(30000, 0.5, 0.5);
  require(half.cross == 7500 && half.miss == 7500 && half.interior == 15000, "quota_half", errors);
  const Quotas high = quotas_for(100000, 0.9, 0.0);
  require(high.cross == 0 && high.miss == 90000 && high.interior == 10000, "quota_high", errors);
  const Quotas odd = quotas_for(10, 0.5, 0.5);
  require(odd.cross == 2 && odd.miss == 2 && odd.interior == 6, "quota_remainder_interior", errors);
  const Vec3 inside = to_optical(pose, {0, 0, 1.2});
  require(classify_endpoint(chamfer, pose, inside, z_min, z_max) == EndpointClass::Interior,
          "inside", errors);
  const Vec3 crossing = to_optical(pose, {1.2, 0, 1.2});
  require(classify_endpoint(chamfer, pose, crossing, z_min, z_max) == EndpointClass::ExteriorCross,
          "cross", errors);
  const Vec3 missing = to_optical(pose, {-0.55, 1.9, 1.2});
  require(classify_endpoint(chamfer, pose, missing, z_min, z_max) == EndpointClass::ExteriorMiss,
          "miss", errors);
  Vec3 low = inside;
  low.z = 0.1;
  require(classify_endpoint(chamfer, pose, low, z_min, z_max) == EndpointClass::Invalid, "z_filter", errors);
  Geometry cuboid = chamfer;
  cuboid.has_chamfer = false;
  const Vec3 cut = to_optical(pose, {0.0, 0.80, 0.60});
  require(classify_endpoint(chamfer, pose, cut, z_min, z_max) != EndpointClass::Interior, "chamfer_cut", errors);
  require(classify_endpoint(cuboid, pose, cut, z_min, z_max) == EndpointClass::Interior, "cuboid_keeps_cut", errors);
  const auto boxes = make_boxes(5, chamfer);
  const auto first = generate_cloud(chamfer, boxes, 300, 0.9, 0.5, 11, z_min, z_max);
  const auto second = generate_cloud(chamfer, boxes, 300, 0.9, 0.5, 11, z_min, z_max);
  require(first.sha256 == second.sha256, "hash_stable", errors);
  require(first.counts.invalid == 0 && first.optical.size() == 300, "valid_count", errors);
  const auto other = generate_cloud(chamfer, boxes, 300, 0.5, 0.0, 11, z_min, z_max);
  require(other.sha256 != first.sha256 && other.counts.miss != first.counts.miss, "fraction_changes_cloud", errors);
  const auto cube = generate_cloud(cuboid, make_boxes(0, cuboid), 200, 0.5, 0.5, 3, z_min, z_max);
  require(cube.counts.cross == 50 && cube.counts.miss == 50 && cube.counts.interior == 100, "cuboid_quota", errors);
  const auto seed_a = frame_seed("r01_b00_chamfer_c30000_o50_x00", 0, 0);
  const auto seed_b = frame_seed("r01_b00_chamfer_c30000_o50_x00", 0, 1);
  require(seed_a != seed_b, "seed_round", errors);
  CheckResult result;
  result.pass = errors.empty();
  std::ostringstream json;
  json << "{\"pass\":" << (result.pass ? "true" : "false")
       << ",\"generator\":\"" << kGeneratorVersion << "\",\"errors\":\"" << errors
       << "\",\"sample\":" << counts_json(first.counts) << "}\n";
  result.json = json.str();
  return result;
}

}  // namespace occ_load
