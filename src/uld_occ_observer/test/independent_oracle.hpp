#pragma once
// Independent occupancy oracle. This translation unit must not include
// uld_occ_observer headers or call computeRayKeys for expected keys.
#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace observer_oracle {

struct Vec3 {
  double x{}, y{}, z{};
};

struct Cell {
  int x{}, y{}, z{};
  bool operator<(const Cell& o) const {
    return std::array<int, 3>{x, y, z} < std::array<int, 3>{o.x, o.y, o.z};
  }
  bool operator==(const Cell& o) const { return x == o.x && y == o.y && z == o.z; }
};

struct Chamfer {
  double floor_y{}, wall_y{}, wall_z{};
};

struct Geometry {
  std::string geometry_hash;
  double length{}, width{}, floor_z{}, ceiling_z{}, resolution{0.01};
  bool has_chamfer{};
  Chamfer chamfer{};
  Vec3 origin() const { return {-length / 2, -width / 2, floor_z}; }
};

struct DepthZ {
  double min_m{0.52};
  double max_m{6.0};
};

struct FilterStats {
  std::vector<Vec3> kept;
  std::vector<uint8_t> mask;
  std::size_t raw{}, nonfinite{}, below{}, above{};
};

struct Rigid {
  std::array<double, 9> rotation{1, 0, 0, 0, 1, 0, 0, 0, 1};
  Vec3 translation{};
};

struct Prepared {
  std::vector<Vec3> uld_points;
  Vec3 origin{};
  FilterStats filter;
};

struct FrameKeys {
  std::vector<Cell> hits;
  std::vector<Cell> frees;
};

struct CellState {
  float log_odds{};
  bool occupied{};
};

inline bool finite(const Vec3& p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

inline float logodds(double p) {
  return static_cast<float>(std::log(p / (1.0 - p)));
}

inline Vec3 apply(const Rigid& t, const Vec3& p) {
  return {t.rotation[0] * p.x + t.rotation[1] * p.y + t.rotation[2] * p.z + t.translation.x,
          t.rotation[3] * p.x + t.rotation[4] * p.y + t.rotation[5] * p.z + t.translation.y,
          t.rotation[6] * p.x + t.rotation[7] * p.y + t.rotation[8] * p.z + t.translation.z};
}

FilterStats filter_optical_z(const std::vector<Vec3>& points, const DepthZ& cfg);
bool contains(const Geometry& g, const Vec3& p);
Cell shape(const Geometry& g);
Vec3 center(const Geometry& g, const Cell& i);
Vec3 to_octomap(const Geometry& g, const Vec3& p);
bool active(const Geometry& g, const Cell& i);
std::optional<Cell> index_of(const Geometry& g, const Vec3& p);
int coord_to_index(double coord, double resolution);
std::vector<Cell> traverse_ray(const Vec3& origin_octo, const Vec3& end_octo, double resolution);
Prepared prepare(const std::vector<Vec3>& optical, const Rigid& tf, const DepthZ& z);
FrameKeys evidence_keys(const Geometry& g, const Vec3& origin_uld, const std::vector<Vec3>& ends_uld);

class OccupancyOracle {
 public:
  static constexpr float kHit = 0.84729786f;   // logodds(0.7) as float
  static constexpr float kMiss = -0.4054651f;  // logodds(0.4)
  static constexpr float kMin = -2.0f;         // filled in ctor from 0.1192
  static constexpr float kMax = 3.5f;
  explicit OccupancyOracle(Geometry geometry);
  void apply_frame(const FrameKeys& keys);
  const std::map<Cell, CellState>& known() const { return known_; }
  std::size_t known_count() const { return known_.size(); }
  float hit_log() const { return hit_; }
  float miss_log() const { return miss_; }
  float clamp_min() const { return min_; }
  float clamp_max() const { return max_; }

 private:
  void update(const Cell& cell, bool hit);
  Geometry geometry_;
  std::map<Cell, CellState> known_;
  float hit_{}, miss_{}, min_{}, max_{};
};

}  // namespace observer_oracle
