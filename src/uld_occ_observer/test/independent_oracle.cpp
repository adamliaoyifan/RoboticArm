#include "independent_oracle.hpp"
#include <algorithm>
#include <limits>
#include <set>

namespace observer_oracle {
namespace {
constexpr double kEps = 1e-9;
}

FilterStats filter_optical_z(const std::vector<Vec3>& points, const DepthZ& cfg) {
  FilterStats out;
  out.raw = points.size();
  out.mask.assign(points.size(), 0);
  out.kept.reserve(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto& p = points[i];
    if (!finite(p)) {
      ++out.nonfinite;
      continue;
    }
    if (p.z < cfg.min_m) {
      ++out.below;
      continue;
    }
    if (p.z > cfg.max_m) {
      ++out.above;
      continue;
    }
    out.mask[i] = 1;
    out.kept.push_back(p);
  }
  return out;
}

bool contains(const Geometry& g, const Vec3& p) {
  if (!finite(p) || p.x < -g.length / 2 - kEps || p.x > g.length / 2 + kEps ||
      p.y < -g.width / 2 - kEps || p.y > g.width / 2 + kEps ||
      p.z < g.floor_z - kEps || p.z > g.ceiling_z + kEps)
    return false;
  if (g.has_chamfer && p.z < g.chamfer.wall_z - kEps) {
    const double slope =
        (g.chamfer.wall_y - g.chamfer.floor_y) / (g.chamfer.wall_z - g.floor_z);
    if (p.y > g.chamfer.floor_y + slope * (p.z - g.floor_z) + kEps) return false;
  }
  return true;
}

Cell shape(const Geometry& g) {
  return {static_cast<int>(std::ceil(g.length / g.resolution)),
          static_cast<int>(std::ceil(g.width / g.resolution)),
          static_cast<int>(std::ceil((g.ceiling_z - g.floor_z) / g.resolution))};
}

Vec3 center(const Geometry& g, const Cell& i) {
  const auto o = g.origin();
  return {o.x + (i.x + 0.5) * g.resolution, o.y + (i.y + 0.5) * g.resolution,
          o.z + (i.z + 0.5) * g.resolution};
}

Vec3 to_octomap(const Geometry& g, const Vec3& p) {
  const auto o = g.origin();
  return {p.x - o.x, p.y - o.y, p.z - o.z};
}

bool active(const Geometry& g, const Cell& i) {
  const auto s = shape(g);
  return i.x >= 0 && i.y >= 0 && i.z >= 0 && i.x < s.x && i.y < s.y && i.z < s.z &&
         contains(g, center(g, i));
}

int coord_to_index(double coord, double resolution) {
  return static_cast<int>(std::floor((1.0 / resolution) * coord));
}

std::optional<Cell> index_of(const Geometry& g, const Vec3& p) {
  if (!contains(g, p)) return std::nullopt;
  const auto o = to_octomap(g, p);
  Cell i{coord_to_index(o.x, g.resolution), coord_to_index(o.y, g.resolution),
         coord_to_index(o.z, g.resolution)};
  return active(g, i) ? std::optional<Cell>(i) : std::nullopt;
}

std::vector<Cell> traverse_ray(const Vec3& origin_in, const Vec3& end_in, double resolution) {
  // Line-by-line independent transcription of OcTreeBaseImpl::computeRayKeys
  // numeric steps (float point3d, float length, float border offset). Does not
  // call computeRayKeys.
  const double resolution_factor = 1.0 / resolution;
  const float origin[3] = {static_cast<float>(origin_in.x), static_cast<float>(origin_in.y),
                           static_cast<float>(origin_in.z)};
  const float endp[3] = {static_cast<float>(end_in.x), static_cast<float>(end_in.y),
                         static_cast<float>(end_in.z)};
  int start[3], goal[3];
  for (int i = 0; i < 3; ++i) {
    start[i] = static_cast<int>(std::floor(resolution_factor * origin[i]));
    goal[i] = static_cast<int>(std::floor(resolution_factor * endp[i]));
  }
  std::vector<Cell> keys;
  if (start[0] == goal[0] && start[1] == goal[1] && start[2] == goal[2]) return keys;
  keys.push_back({start[0], start[1], start[2]});
  float direction[3] = {endp[0] - origin[0], endp[1] - origin[1], endp[2] - origin[2]};
  const float length = std::sqrt(direction[0] * direction[0] + direction[1] * direction[1] +
                                 direction[2] * direction[2]);
  if (!(length > 0.f)) return keys;
  direction[0] /= length;
  direction[1] /= length;
  direction[2] /= length;
  int step[3]{};
  double t_max[3]{}, t_delta[3]{};
  int cur[3] = {start[0], start[1], start[2]};
  for (int i = 0; i < 3; ++i) {
    if (direction[i] > 0.f)
      step[i] = 1;
    else if (direction[i] < 0.f)
      step[i] = -1;
    else
      step[i] = 0;
    if (step[i] != 0) {
      double voxel_border = (static_cast<double>(cur[i]) + 0.5) * resolution;
      voxel_border += static_cast<float>(step[i] * resolution * 0.5);
      t_max[i] = (voxel_border - origin[i]) / direction[i];
      t_delta[i] = resolution / std::fabs(static_cast<double>(direction[i]));
    } else {
      t_max[i] = std::numeric_limits<double>::max();
      t_delta[i] = std::numeric_limits<double>::max();
    }
  }
  while (true) {
    unsigned dim;
    if (t_max[0] < t_max[1])
      dim = (t_max[0] < t_max[2]) ? 0 : 2;
    else
      dim = (t_max[1] < t_max[2]) ? 1 : 2;
    cur[dim] += step[dim];
    t_max[dim] += t_delta[dim];
    if (cur[0] == goal[0] && cur[1] == goal[1] && cur[2] == goal[2]) break;
    const double dist = std::min(t_max[0], std::min(t_max[1], t_max[2]));
    if (dist > static_cast<double>(length)) break;
    keys.push_back({cur[0], cur[1], cur[2]});
  }
  return keys;
}

Prepared prepare(const std::vector<Vec3>& optical, const Rigid& tf, const DepthZ& z) {
  Prepared out;
  out.filter = filter_optical_z(optical, z);
  out.origin = apply(tf, {0, 0, 0});
  out.uld_points.reserve(out.filter.kept.size());
  for (const auto& p : out.filter.kept) out.uld_points.push_back(apply(tf, p));
  return out;
}

FrameKeys evidence_keys(const Geometry& g, const Vec3& origin_uld, const std::vector<Vec3>& ends_uld) {
  std::set<Cell> hits, frees;
  const auto origin = to_octomap(g, origin_uld);
  for (const auto& end_uld : ends_uld) {
    const auto end = to_octomap(g, end_uld);
    for (const auto& key : traverse_ray(origin, end, g.resolution)) {
      if (active(g, key)) frees.insert(key);
    }
    if (contains(g, end_uld)) {
      Cell idx{coord_to_index(end.x, g.resolution), coord_to_index(end.y, g.resolution),
               coord_to_index(end.z, g.resolution)};
      if (active(g, idx)) hits.insert(idx);
    }
  }
  for (const auto& hit : hits) frees.erase(hit);
  return {std::vector<Cell>(hits.begin(), hits.end()),
          std::vector<Cell>(frees.begin(), frees.end())};
}

OccupancyOracle::OccupancyOracle(Geometry geometry) : geometry_(std::move(geometry)) {
  hit_ = logodds(0.7);
  miss_ = logodds(0.4);
  min_ = logodds(0.1192);
  max_ = logodds(0.971);
}

void OccupancyOracle::update(const Cell& cell, bool hit) {
  const float delta = hit ? hit_ : miss_;
  auto it = known_.find(cell);
  if (it != known_.end()) {
    if ((delta >= 0.f && it->second.log_odds >= max_) ||
        (delta <= 0.f && it->second.log_odds <= min_))
      return;
    float next = it->second.log_odds + delta;
    if (next < min_) next = min_;
    if (next > max_) next = max_;
    it->second.log_odds = next;
    it->second.occupied = next >= 0.f;
    return;
  }
  float next = delta;
  if (next < min_) next = min_;
  if (next > max_) next = max_;
  known_[cell] = {next, next >= 0.f};
}

void OccupancyOracle::apply_frame(const FrameKeys& keys) {
  for (const auto& cell : keys.frees) update(cell, false);
  for (const auto& cell : keys.hits) update(cell, true);
}

}  // namespace observer_oracle
