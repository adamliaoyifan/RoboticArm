#include "uld_occ_observer/geometry.hpp"
#include <chrono>
#include <cmath>
#include <mutex>

namespace uld_occ_observer {
UldDomain::UldDomain(GeometryDescriptor descriptor, double resolution)
    : descriptor_(std::move(descriptor)), resolution_(resolution) {
  const auto& d = descriptor_;
  if (d.geometry_hash.empty() || d.artifact_digest.empty() ||
      !std::isfinite(d.length) || !std::isfinite(d.width) ||
      !std::isfinite(d.floor_z) || !std::isfinite(d.ceiling_z) ||
      !std::isfinite(resolution_) || resolution_ <= 0 ||
      d.length <= 0 || d.width <= 0 || d.ceiling_z <= d.floor_z)
    throw Error("INVALID_GEOMETRY");
  origin_ = {-d.length/2, -d.width/2, d.floor_z};
  // OctoMap has a finite 16-bit key space per axis; never overflow the grid DTO.
  const Vec3 extent{d.length/resolution_, d.width/resolution_,
                    (d.ceiling_z-d.floor_z)/resolution_};
  if (!finite(extent) || extent.x >= 32768 || extent.y >= 32768 || extent.z >= 32768)
    throw Error("GRID_OUT_OF_KEY_RANGE");
  shape_ = {static_cast<int>(std::ceil(extent.x)), static_cast<int>(std::ceil(extent.y)),
            static_cast<int>(std::ceil(extent.z))};
  if (d.chamfer) {
    const auto& c = *d.chamfer;
    if (!finite({c.floor_y, c.wall_y, c.wall_z}) || c.wall_z <= d.floor_z ||
        c.wall_z >= d.ceiling_z || c.floor_y < -d.width/2 ||
        c.floor_y > c.wall_y || c.wall_y > d.width/2)
      throw Error("INVALID_CHAMFER");
  }
}

bool UldDomain::contains(const Vec3& p) const {
  constexpr double eps = 1e-9;  // geometry kernel EPS, including its wall-z branch
  const auto& d = descriptor_;
  if (!finite(p) || p.x < -d.length/2-eps || p.x > d.length/2+eps ||
      p.y < -d.width/2-eps || p.y > d.width/2+eps ||
      p.z < d.floor_z-eps || p.z > d.ceiling_z+eps) return false;
  if (d.chamfer && p.z < d.chamfer->wall_z-eps) {
    const auto& c = *d.chamfer;
    const double slope = (c.wall_y-c.floor_y)/(c.wall_z-d.floor_z);
    if (p.y > c.floor_y + slope*(p.z-d.floor_z) + eps) return false;
  }
  return true;
}

const std::vector<std::uint32_t>& UldDomain::column_active_counts(
    DomainInitObservation* observation, bool measure_time) const {
  // call_once runs this body on one caller. A throw leaves the flag unset, so a later call retries.
  // The cache vector is published only after the full geometry walk.
  std::call_once(active_once_, [&] {
    using Clock = std::chrono::steady_clock;
    const auto started = measure_time ? Clock::now() : Clock::time_point{};
    const auto columns = static_cast<std::size_t>(shape_.x) * static_cast<std::size_t>(shape_.y);
    const auto scanned = static_cast<std::uint64_t>(shape_.x) * static_cast<std::uint64_t>(shape_.y) *
                         static_cast<std::uint64_t>(shape_.z);
    std::vector<std::uint32_t> counts(columns, 0);
    for (int y = 0; y < shape_.y; ++y) {
      for (int x = 0; x < shape_.x; ++x) {
        std::uint32_t count = 0;
        for (int z = 0; z < shape_.z; ++z) count += active({x, y, z});
        counts[static_cast<std::size_t>(y) * shape_.x + x] = count;
      }
    }
    active_counts_ = std::move(counts);
    if (observation != nullptr) {
      observation->performed = true;
      observation->scanned_cells = scanned;
      if (measure_time) {
        observation->milliseconds =
            std::chrono::duration<double, std::milli>(Clock::now() - started).count();
      }
    }
  });
  return active_counts_;
}

bool UldDomain::active(const CellIndex& i) const {
  return i.x >= 0 && i.y >= 0 && i.z >= 0 && i.x < shape_.x &&
         i.y < shape_.y && i.z < shape_.z && contains(center(i));
}

std::optional<CellIndex> UldDomain::index_of(const Vec3& point) const {
  if (!contains(point)) return std::nullopt;
  const auto p = octomap_coordinates(point);
  CellIndex index{static_cast<int>(std::floor(p.x/resolution_)),
                  static_cast<int>(std::floor(p.y/resolution_)),
                  static_cast<int>(std::floor(p.z/resolution_))};
  return active(index) ? std::optional<CellIndex>(index) : std::nullopt;
}

Vec3 UldDomain::center(const CellIndex& i) const {
  return {origin_.x + (i.x+.5)*resolution_, origin_.y + (i.y+.5)*resolution_,
          origin_.z + (i.z+.5)*resolution_};
}
Vec3 UldDomain::octomap_coordinates(const Vec3& p) const {
  return {p.x-origin_.x, p.y-origin_.y, p.z-origin_.z};
}
}  // namespace uld_occ_observer
