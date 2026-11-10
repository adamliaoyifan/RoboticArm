#include "uld_occ_observer/freespace_query.hpp"
#include "uld_occ_observer/block_storage.hpp"
#include "uld_occ_observer/resource_usage.hpp"
#include <algorithm>
#include <cmath>

namespace uld_occ_observer {
namespace {
int cells_for(double meters, double resolution) {
  if (!(meters > 0.0) || !std::isfinite(meters)) throw Error("INVALID_BOX_DIMENSION");
  const double cells = std::ceil(meters / resolution - 1e-9);
  if (!(cells >= 1.0) || cells > 1e9) throw Error("INVALID_BOX_DIMENSION");
  return static_cast<int>(cells);
}
int ceil_cells(double value) { return static_cast<int>(std::ceil(value - 1e-9)); }
int floor_cells(double value) { return static_cast<int>(std::floor(value + 1e-9)); }

// Dimensional prefix sums over the full domain (free / occupied / active) plus
// the per-column highest occupied cell. One-cell margins on every axis make
// every range sum a closed-form difference with no low-end clamping. The
// buffers are RAII-tracked under the "query_scratch" class: transient, owned
// by this call, outside the read-view/cache gate scope, visible in
// class_peaks so the resource table still lists who allocates and releases.
class DomainPrefix {
 public:
  DomainPrefix(const UldDomain& domain, const Snapshot& snapshot) {
    const auto shape = domain.shape();
    sx_ = shape.x; sy_ = shape.y; sz_ = shape.z;
    ystride_ = static_cast<std::size_t>(sz_ + 1);
    xstride_ = static_cast<std::size_t>(sy_ + 1) * ystride_;
    const std::size_t total = (static_cast<std::size_t>(sx_) + 1)
        * (static_cast<std::size_t>(sy_) + 1) * (static_cast<std::size_t>(sz_) + 1);
    occupied_.assign(total, 0);
    free_.assign(total, 0);
    active_.assign(total, 0);
    bytes_ = total * 3 * sizeof(std::uint32_t) + static_cast<std::size_t>(sx_) * sy_ * sizeof(int);
    AllocationTotals::instance().add("query_scratch", bytes_);
    for (int x = 0; x < sx_; ++x)
      for (int y = 0; y < sy_; ++y)
        for (int z = 0; z < sz_; ++z)
          if (domain.active({x, y, z}))
            active_[flat(x + 1, y + 1, z + 1)] = 1;
    top_.assign(static_cast<std::size_t>(sx_) * sy_, -1);
    snapshot.for_each_known_cell([&](const CellIndex& index, const CellValue& value) {
      if (!domain.active(index)) return;
      const auto flat_cell = [this](int x, int y, int z) {
        return static_cast<std::size_t>(x) * xstride_
            + static_cast<std::size_t>(y) * ystride_ + static_cast<std::size_t>(z);
      };
      if (value.occupied) {
        occupied_[flat_cell(index.x + 1, index.y + 1, index.z + 1)] = 1;
        const auto column = static_cast<std::size_t>(index.y) * sx_ + index.x;
        if (index.z > top_[column]) top_[column] = index.z;
      } else {
        free_[flat_cell(index.x + 1, index.y + 1, index.z + 1)] = 1;
      }
    });
    scan();
  }
  ~DomainPrefix() { AllocationTotals::instance().subtract("query_scratch", bytes_); }
  DomainPrefix(const DomainPrefix&) = delete;
  DomainPrefix& operator=(const DomainPrefix&) = delete;

  std::uint64_t occupied(int x0, int y0, int z0, int x1, int y1, int z1) const {
    return range(occupied_, x0, y0, z0, x1, y1, z1);
  }
  std::uint64_t free_cells(int x0, int y0, int z0, int x1, int y1, int z1) const {
    return range(free_, x0, y0, z0, x1, y1, z1);
  }
  std::uint64_t active(int x0, int y0, int z0, int x1, int y1, int z1) const {
    return range(active_, x0, y0, z0, x1, y1, z1);
  }
  int top(int x, int y) const { return top_[static_cast<std::size_t>(y) * sx_ + x]; }
  int sx() const { return sx_; }
  int sy() const { return sy_; }
  int sz() const { return sz_; }

 private:
  std::size_t flat(int x, int y, int z) const {
    return static_cast<std::size_t>(x) * xstride_
        + static_cast<std::size_t>(y) * ystride_ + static_cast<std::size_t>(z);
  }
  // Inclusive scans along z, then y, then x turn the indicator grids into
  // dimensional prefix sums in place.
  void scan() {
    for (auto* grid : {&occupied_, &free_, &active_}) {
      auto& g = *grid;
      for (int x = 0; x <= sx_; ++x)
        for (int y = 0; y <= sy_; ++y) {
          std::uint32_t running = 0;
          for (int z = 0; z <= sz_; ++z) {
            const auto at = flat(x, y, z);
            running += g[at];
            g[at] = running;
          }
        }
      for (int x = 0; x <= sx_; ++x)
        for (int z = 0; z <= sz_; ++z) {
          std::uint32_t running = 0;
          for (int y = 0; y <= sy_; ++y) {
            const auto at = flat(x, y, z);
            running += g[at];
            g[at] = running;
          }
        }
      for (int y = 0; y <= sy_; ++y)
        for (int z = 0; z <= sz_; ++z) {
          std::uint32_t running = 0;
          for (int x = 0; x <= sx_; ++x) {
            const auto at = flat(x, y, z);
            running += g[at];
            g[at] = running;
          }
        }
    }
  }
  std::uint64_t range(const std::vector<std::uint32_t>& g,
                      int x0, int y0, int z0, int x1, int y1, int z1) const {
    x0 = std::max(0, std::min(x0, sx_));
    y0 = std::max(0, std::min(y0, sy_));
    z0 = std::max(0, std::min(z0, sz_));
    x1 = std::max(x0, std::min(x1, sx_));
    y1 = std::max(y0, std::min(y1, sy_));
    z1 = std::max(z0, std::min(z1, sz_));
    const auto at = [&](int x, int y, int z) -> std::uint64_t {
      return g[flat(x, y, z)];
    };
    return at(x1, y1, z1) - at(x0, y1, z1) - at(x1, y0, z1) - at(x1, y1, z0)
        + at(x0, y0, z1) + at(x0, y1, z0) + at(x1, y0, z0) - at(x0, y0, z0);
  }
  int sx_{}, sy_{}, sz_{};
  std::size_t xstride_{}, ystride_{}, bytes_{};
  std::vector<std::uint32_t> occupied_, free_, active_;
  std::vector<int> top_;
};
}  // namespace

FreespacePage query_freespace(const Snapshot& snapshot, const FreespaceRequest& request,
                              const PlacementPolicy& policy) {
  const int yaw_modes[3][2] = {{0, -1}, {90, -1}, {0, 90}};
  const int yaw_count[3] = {1, 1, 2};
  if (request.yaw_mode < 0 || request.yaw_mode > 2) throw Error("INVALID_YAW_MODE");
  if (request.limit == 0 || request.limit > 65536) throw Error("INVALID_PAGE_LIMIT");
  const auto& domain = *snapshot.domain();
  const double resolution = domain.resolution();
  const auto shape = domain.shape();
  const auto origin = domain.origin();
  const double ceiling = domain.descriptor().ceiling_z;
  const int fz = cells_for(request.box_height_m, resolution);
  // Support slab: occupied tops within support_tol below the box bottom.
  const int slab = ceil_cells(policy.support_tol_m / resolution);
  // Floor prior: the bottom is within support_tol above the floor plane.
  const int floor_cells_below = floor_cells(policy.support_tol_m / resolution);
  // Lateral clearance: occupied columns whose world gap to the footprint is
  // below same_layer_clearance reject the candidate (z overlapping the box).
  const int side = ceil_cells(policy.same_layer_clearance_m / resolution);
  DomainPrefix prefix(domain, snapshot);
  FreespacePage page;
  page.region_begin[0] = page.region_begin[1] = page.region_begin[2] = 0;
  page.region_end[0] = shape.x;
  page.region_end[1] = shape.y;
  page.region_end[2] = shape.z;
  std::vector<int> tops;
  std::size_t feasible_index = 0, emitted = 0;
  for (int mode = 0; mode < yaw_count[request.yaw_mode]; ++mode) {
    const int yaw = yaw_modes[request.yaw_mode][mode];
    const int fx = cells_for(yaw == 0 ? request.box_length_m : request.box_width_m, resolution);
    const int fy = cells_for(yaw == 0 ? request.box_width_m : request.box_length_m, resolution);
    for (int cx = 0; cx + fx <= prefix.sx(); ++cx) {
      for (int cy = 0; cy + fy <= prefix.sy(); ++cy) {
        tops.clear();
        for (int x = cx; x < cx + fx; ++x)
          for (int y = cy; y < cy + fy; ++y) tops.push_back(prefix.top(x, y));
        std::sort(tops.begin(), tops.end());
        const auto columns = static_cast<std::size_t>(fx) * fy;
        const auto volume = columns * static_cast<std::size_t>(fz);
        for (int cz = 0; cz + fz <= prefix.sz(); ++cz) {
          ++page.grid_total;
          if (prefix.active(cx, cy, cz, cx + fx, cy + fy, cz + fz) != volume) {
            ++page.rejected_out_of_domain;
            continue;
          }
          if (prefix.occupied(cx - side, cy - side, cz, cx + fx + side, cy + fy + side, cz + fz)) {
            ++page.rejected_collision;
            continue;
          }
          const double world_top = origin.z + static_cast<double>(cz + fz) * resolution;
          if (ceiling - world_top < policy.clearance_margin_m - 1e-9) {
            ++page.rejected_clearance;
            continue;
          }
          if (prefix.free_cells(cx, cy, cz, cx + fx, cy + fy, cz + fz) != volume) {
            ++page.rejected_unknown;
            continue;
          }
          // Observed support: occupied tops in [max(0, cz - slab), cz - 1].
          // The no-occupancy sentinel is -1, so the window never opens below
          // 0: at cz=0 an empty column must not count as observed support.
          std::size_t observed = 0;
          if (cz >= 1) {
            const auto low = std::lower_bound(tops.begin(), tops.end(),
                                              std::max(0, cz - slab));
            const auto high = std::upper_bound(tops.begin(), tops.end(), cz - 1);
            if (high > low) observed = static_cast<std::size_t>(high - low);
          }
          const bool floor_supported = cz <= floor_cells_below;
          const double ratio = floor_supported
              ? 1.0 : static_cast<double>(observed) / static_cast<double>(columns);
          if (ratio < policy.min_support_ratio - 1e-9) {
            ++page.rejected_support;
            continue;
          }
          if (feasible_index >= request.cursor && emitted < request.limit) {
            PlacementCandidate candidate;
            candidate.cell_x = cx;
            candidate.cell_y = cy;
            candidate.cell_z = cz;
            candidate.origin_x = origin.x + static_cast<double>(cx) * resolution;
            candidate.origin_y = origin.y + static_cast<double>(cy) * resolution;
            candidate.origin_z = origin.z + static_cast<double>(cz) * resolution;
            candidate.yaw_deg = yaw;
            candidate.support_ratio = ratio;
            candidate.support_kind = observed > 0 ? 1 : 2;
            candidate.verify_begin[0] = cx;
            candidate.verify_begin[1] = cy;
            candidate.verify_begin[2] = cz;
            candidate.verify_end[0] = cx + fx;
            candidate.verify_end[1] = cy + fy;
            candidate.verify_end[2] = cz + fz;
            page.candidates.push_back(candidate);
            ++emitted;
          }
          ++feasible_index;
        }
      }
    }
  }
  page.feasible_total = feasible_index;
  page.next_cursor = request.cursor + emitted;
  page.done = page.next_cursor >= feasible_index;
  return page;
}
}  // namespace uld_occ_observer
